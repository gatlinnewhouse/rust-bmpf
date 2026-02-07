use crate::{
    consts::{BOX_DIM, DEFAULT_GPS_VAR, IMU_A_VAR, IMU_R_VAR, MAX_SPEED, NEG_BOX_DIM, TWO_PI},
    resample::{Resample, Resampler},
    types::{ACoord, CCoord, Particles},
    utils::normalize_angle,
};
use gpoint::GPoint;
use multiversion::multiversion;
use std::{fmt::Write, fs::OpenOptions, io::BufWriter};
use ziggurat_rs::Ziggurat;

pub struct BpfState {
    pub(crate) pstates: [Particles; 2],
    pub(crate) which_particle: bool,
    pub(crate) resampler: Resampler,
    pub(crate) sort: bool,
    pub(crate) nparticles: usize,
    pub report_particles: i32,
    pub(crate) best_particle: bool,
    pub(crate) resample_interval: usize,
    pub(crate) resample_count: usize,
    pub vehicle: CCoord,
    pub(crate) gps: CCoord,
    pub(crate) imu: ACoord,
    pub(crate) filename_buf: String,
    pub(crate) rng: Ziggurat,
    pub(crate) inv_nparticles: f64,
    pub(crate) gps_var: f64,
    pub(crate) inv_gps_var: f64,
}

impl Default for BpfState {
    fn default() -> Self {
        Self {
            pstates: [Particles::default(), Particles::default()],
            which_particle: false,
            resampler: Resampler::new("naive", 100),
            sort: false,
            nparticles: 100,
            report_particles: 1000,
            best_particle: false,
            resample_interval: 1,
            resample_count: 0,
            vehicle: CCoord::default(),
            gps: CCoord::default(),
            imu: ACoord::default(),
            filename_buf: String::with_capacity(64),
            rng: Ziggurat::default(),
            inv_nparticles: 1.0 / 100.0,
            gps_var: DEFAULT_GPS_VAR,
            inv_gps_var: 1.0 / DEFAULT_GPS_VAR,
        }
    }
}

impl BpfState {
    pub fn new(
        resampler: &str,
        sort: bool,
        nparticles: usize,
        report_particles: i32,
        best_particle: bool,
        resample_interval: usize,
        gps_var: f64,
    ) -> Self {
        Self {
            pstates: [Particles::new(nparticles), Particles::new(nparticles)],
            which_particle: false,
            resampler: Resampler::new(resampler, nparticles),
            sort,
            nparticles,
            report_particles,
            best_particle,
            resample_interval,
            resample_count: 0,
            vehicle: CCoord::default(),
            gps: CCoord::default(),
            imu: ACoord::default(),
            filename_buf: String::with_capacity(64),
            rng: Ziggurat::default(),
            inv_nparticles: 1.0 / nparticles as f64,
            gps_var,
            inv_gps_var: 1.0 / gps_var,
        }
    }

    pub fn with_seed(mut self, seed: u32) -> Self {
        self.rng = Ziggurat::new(seed);
        self
    }

    pub fn init_particles(&mut self) {
        self.which_particle = false;
        for i in 0..self.nparticles {
            self.pstates[0].init_particle_state(i, &mut self.rng);
            self.pstates[0].weight[i] = self.inv_nparticles;
        }
    }

    pub fn parse_line(&mut self, line: String) -> i32 {
        let mut parts = line.split(' ');
        let t_ms: i32 = parts
            .next()
            .expect("Failed to get next split in parse_line")
            .parse()
            .expect("Failed to parse t_ms");
        self.vehicle.x = parts
            .next()
            .unwrap()
            .parse()
            .expect("Failed to parse vehicle x");
        self.vehicle.y = parts
            .next()
            .unwrap()
            .parse()
            .expect("Failed to parse vehicle y");
        self.gps.x = parts
            .next()
            .unwrap()
            .parse()
            .expect("Failed to parse gps x");
        self.gps.y = parts
            .next()
            .unwrap()
            .parse()
            .expect("Failed to parse gps y");
        self.imu.r = parts
            .next()
            .unwrap()
            .parse()
            .expect("Failed to parse imu r");
        self.imu.t = parts
            .next()
            .unwrap()
            .parse()
            .expect("Failed to parse imu t");
        t_ms
    }

    pub fn bpf_step(&mut self, t: f64, dt: f64, report: bool) {
        let which = self.which_particle as usize;
        let inv_dt = 1.0 / dt;

        // Update states (RNG-dependent, cannot vectorize)
        for i in 0..self.nparticles {
            self.pstates[which].update_particle_state(i, dt, 1, &mut self.rng);
        }

        // Fused probability computation and weight update
        let tweight = compute_weights_fused(
            self.pstates[which].posn_x.as_slice(),
            self.pstates[which].posn_y.as_slice(),
            self.pstates[which].vel_r.as_slice(),
            self.pstates[which].vel_t.as_slice(),
            self.pstates[which].weight.as_mut_slice(),
            self.gps.x,
            self.gps.y,
            self.inv_gps_var,
            self.imu.r,
            self.imu.t,
            inv_dt,
        );

        #[cfg(feature = "debug")]
        assert!(tweight > 0.00001, "{} < 0.00001", GPoint(tweight));

        // Normalize weights
        let invtweight = 1.0 / tweight;
        normalize_weights(self.pstates[which].weight.as_mut_slice(), invtweight);

        // Compute weighted estimates
        let mut est_posn_x = 0.0;
        let mut est_posn_y = 0.0;
        let mut est_vel_t = 0.0;

        if !self.best_particle {
            let (ex, ey) = weighted_position_sum(
                self.pstates[which].weight.as_slice(),
                self.pstates[which].posn_x.as_slice(),
                self.pstates[which].posn_y.as_slice(),
            );
            est_posn_x = ex;
            est_posn_y = ey;

            // Angle averaging (scalar - circular quantity)
            for i in 0..self.nparticles {
                let w = self.pstates[which].weight[i];
                est_vel_t = normalize_angle(est_vel_t + w * self.pstates[which].vel_t[i]);
            }
        }

        // Report particles if needed
        if report {
            self.filename_buf.clear();
            let _ = write!(&mut self.filename_buf, "benchtmp/particles-{}.dat", t);
            let file = OpenOptions::new()
                .append(true)
                .create(true)
                .open(&self.filename_buf)
                .unwrap_or_else(|_| panic!("Could not open file"));
            let mut writer = BufWriter::new(file);
            for i in 0..self.nparticles {
                use std::io::Write;
                let _ = writeln!(
                    writer,
                    "{} {} {}",
                    GPoint(self.pstates[which].posn_x[i]),
                    GPoint(self.pstates[which].posn_y[i]),
                    GPoint(self.pstates[which].weight[i])
                );
            }
        }

        // Resample if needed
        self.resample_count = (self.resample_count + 1) % self.resample_interval;
        if self.resample_count == 0 {
            let [ref mut p0, ref mut p1] = self.pstates;
            let (current, new) = if self.which_particle {
                (p1, p0)
            } else {
                (p0, p1)
            };
            let _best = self.resampler.resample(
                tweight,
                self.nparticles,
                current,
                self.nparticles,
                new,
                self.sort,
                &mut self.rng,
            );
            self.which_particle = !self.which_particle;
            let which = self.which_particle as usize;
            for i in 0..self.nparticles {
                self.pstates[which].weight[i] = self.inv_nparticles;
            }
        }

        // Find best particle
        let which = self.which_particle as usize;
        let (best, _best_weight) = find_best(self.pstates[which].weight.as_slice());

        #[cfg(feature = "diagnostic-print")]
        {
            let (_, worst, _, worst_weight) =
                find_best_worst(self.pstates[which].weight.as_slice());
            print!(
                "  {} {} {}",
                GPoint(_best_weight),
                GPoint(self.pstates[which].posn_x[best]),
                GPoint(self.pstates[which].posn_y[best]),
            );
            print!(
                "  {} {} {}",
                GPoint(worst_weight),
                GPoint(self.pstates[which].posn_x[worst]),
                GPoint(self.pstates[which].posn_y[worst]),
            );
        }

        #[cfg(not(feature = "diagnostic-print"))]
        {
            print!(
                "  {} {}",
                GPoint(self.pstates[which].posn_x[best]),
                GPoint(self.pstates[which].posn_y[best])
            );
        }

        if !self.best_particle {
            print!("  {} {}", GPoint(est_posn_x), GPoint(est_posn_y));
        }
    }
}

// =============================================================================
// SIMD-enabled helper functions (inlined into this module for cache locality)
// =============================================================================

/// Fused weight computation: gps_prob * imu_prob * weight, returns total
#[multiversion(targets = "simd")]
fn compute_weights_fused(
    posn_x: &[f64],
    posn_y: &[f64],
    vel_r: &[f64],
    vel_t: &[f64],
    weights: &mut [f64],
    gps_x: f64,
    gps_y: f64,
    inv_gps_var: f64,
    imu_r: f64,
    imu_t: f64,
    inv_dt: f64,
) -> f64 {
    let n = weights.len();
    debug_assert_eq!(posn_x.len(), n);
    debug_assert_eq!(posn_y.len(), n);
    debug_assert_eq!(vel_r.len(), n);
    debug_assert_eq!(vel_t.len(), n);

    let neg_half_gps = -0.5 * inv_gps_var * inv_gps_var;
    let imu_r_var_scaled = IMU_R_VAR * inv_dt;
    let imu_a_var_scaled = IMU_A_VAR * inv_dt;
    let inv_r_var = 1.0 / imu_r_var_scaled;
    let inv_a_var = 1.0 / imu_a_var_scaled;
    let neg_half_r = -0.5 * inv_r_var * inv_r_var;
    let neg_half_a = -0.5 * inv_a_var * inv_a_var;

    let mut total = 0.0;

    for i in 0..n {
        let px = posn_x[i];
        let py = posn_y[i];
        let vr = vel_r[i];
        let vt = vel_t[i];

        // GPS probability
        let gps_prob = if px < NEG_BOX_DIM || px > BOX_DIM || py < NEG_BOX_DIM || py > BOX_DIM {
            0.0
        } else {
            let dx = px - gps_x;
            let dy = py - gps_y;
            libm::exp(neg_half_gps * (dx * dx + dy * dy))
        };

        // IMU probability
        let imu_prob = if vr < 0.0 || vr > MAX_SPEED {
            0.0
        } else {
            let dr = vr - imu_r;
            let dth_raw = (vt - imu_t).abs();
            let dth = dth_raw.min((dth_raw - TWO_PI).abs());
            libm::exp(neg_half_r * dr * dr) * libm::exp(neg_half_a * dth * dth)
        };

        // Update weight
        let w = gps_prob * imu_prob * weights[i];
        weights[i] = w;
        total += w;
    }

    total
}

/// Normalize weights in place
#[multiversion(targets = "simd")]
fn normalize_weights(weights: &mut [f64], scale: f64) {
    for w in weights.iter_mut() {
        *w *= scale;
    }
}

/// Weighted sum of positions
#[multiversion(targets = "simd")]
fn weighted_position_sum(weights: &[f64], posn_x: &[f64], posn_y: &[f64]) -> (f64, f64) {
    let mut sum_x = 0.0;
    let mut sum_y = 0.0;
    for ((&w, &x), &y) in weights.iter().zip(posn_x.iter()).zip(posn_y.iter()) {
        sum_x += w * x;
        sum_y += w * y;
    }
    (sum_x, sum_y)
}

/// Find best (max weight) particle
#[multiversion(targets = "simd")]
fn find_best(weights: &[f64]) -> (usize, f64) {
    let mut best_idx = 0;
    let mut best_w = weights[0];
    for (i, &w) in weights.iter().enumerate().skip(1) {
        if w > best_w {
            best_w = w;
            best_idx = i;
        }
    }
    (best_idx, best_w)
}

#[cfg(feature = "diagnostic-print")]
fn find_best_worst(weights: &[f64]) -> (usize, usize, f64, f64) {
    let mut best_idx = 0;
    let mut worst_idx = 0;
    let mut best_w = weights[0];
    let mut worst_w = weights[0];
    for (i, &w) in weights.iter().enumerate().skip(1) {
        if w > best_w {
            best_w = w;
            best_idx = i;
        } else if w < worst_w {
            worst_w = w;
            worst_idx = i;
        }
    }
    (best_idx, worst_idx, best_w, worst_w)
}
