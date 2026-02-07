use crate::{
    consts::DEFAULT_GPS_VAR,
    resample::{Resample, Resampler},
    simd::{
        fill_slice, find_best_worst, gps_prob_batch, imu_prob_batch, scale_slice, update_weights,
        weighted_circular_mean, weighted_sum,
    },
    types::{ACoord, CCoord, Particles},
    utils::normalize_angle,
};
use gpoint::GPoint;
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
    // Scratch buffers for SIMD operations (avoid repeated allocation)
    gps_probs: Vec<f64>,
    imu_probs: Vec<f64>,
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
            gps_probs: vec![0.0; 100],
            imu_probs: vec![0.0; 100],
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
            gps_probs: vec![0.0; nparticles],
            imu_probs: vec![0.0; nparticles],
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
        }
        // Use SIMD fill for weights
        fill_slice(self.pstates[0].weight.as_mut_slice(), self.inv_nparticles);
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

        // =========================================
        // 1. Update particle states (still sequential due to RNG dependency)
        // =========================================
        for i in 0..self.nparticles {
            self.pstates[which].update_particle_state(i, dt, 1, &mut self.rng);
        }

        // =========================================
        // 2. Batch compute GPS probabilities (SIMD)
        // =========================================
        gps_prob_batch(
            self.pstates[which].posn_x.as_slice(),
            self.pstates[which].posn_y.as_slice(),
            self.gps.x,
            self.gps.y,
            self.inv_gps_var,
            &mut self.gps_probs,
        );

        // =========================================
        // 3. Batch compute IMU probabilities (SIMD)
        // =========================================
        imu_prob_batch(
            self.pstates[which].vel_r.as_slice(),
            self.pstates[which].vel_t.as_slice(),
            self.imu.r,
            self.imu.t,
            inv_dt,
            &mut self.imu_probs,
        );

        // =========================================
        // 4. Update weights and compute total (SIMD)
        // =========================================
        let tweight = update_weights(
            &self.gps_probs,
            &self.imu_probs,
            self.pstates[which].weight.as_mut_slice(),
        );

        #[cfg(feature = "debug")]
        assert!(tweight > 0.00001, "{} < 0.00001", GPoint(tweight));

        // =========================================
        // 5. Normalize weights (SIMD)
        // =========================================
        let invtweight = 1.0 / tweight;
        scale_slice(self.pstates[which].weight.as_mut_slice(), invtweight);

        // =========================================
        // 6. Compute weighted position estimate (SIMD)
        // =========================================
        let (est_posn_x, est_posn_y, est_vel_r, est_vel_t) = if !self.best_particle {
            let weights = self.pstates[which].weight.as_slice();
            let ex = weighted_sum(weights, self.pstates[which].posn_x.as_slice());
            let ey = weighted_sum(weights, self.pstates[which].posn_y.as_slice());
            let er = weighted_sum(weights, self.pstates[which].vel_r.as_slice());
            let et = weighted_circular_mean(weights, self.pstates[which].vel_t.as_slice());
            // Note: Angle averaging is complex, keep scalar for correctness
            // (circular mean would be more accurate but more complex)
            (ex, ey, er, et)
        } else {
            (0.0, 0.0, 0.0, 0.0)
        };

        // =========================================
        // 7. Report particles if needed
        // =========================================
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

        // =========================================
        // 8. Resample if needed
        // =========================================
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
            // Reset weights using SIMD
            fill_slice(
                self.pstates[which].weight.as_mut_slice(),
                self.inv_nparticles,
            );
        }

        // =========================================
        // 9. Find best/worst particle (SIMD)
        // =========================================
        let which = self.which_particle as usize;
        let (best, _worst, _best_weight, _worst_weight) =
            find_best_worst(self.pstates[which].weight.as_slice());

        #[cfg(feature = "diagnostic-print")]
        {
            print!(
                "  {} {} {}",
                GPoint(_best_weight),
                GPoint(self.pstates[which].posn_x[best]),
                GPoint(self.pstates[which].posn_y[best]),
            );
            print!(
                "  {} {} {}",
                GPoint(_worst_weight),
                GPoint(self.pstates[which].posn_x[_worst]),
                GPoint(self.pstates[which].posn_y[_worst]),
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
