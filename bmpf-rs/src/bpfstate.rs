use crate::{
    consts::DEFAULT_GPS_VAR,
    resample::{Resample, Resampler},
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
        let mut tweight;
        let mut best;
        #[cfg(feature = "diagnostic-print")]
        let mut worst = 0usize;
        let mut best_weight;
        let mut worst_weight;

        let mut est_posn_x = 0.0;
        let mut est_posn_y = 0.0;
        let mut est_vel_r = 0.0;
        let mut est_vel_t = 0.0;

        let which = self.which_particle as usize;

        #[cfg(feature = "debug")]
        {
            tweight = 0.0;
            for i in 0..self.nparticles {
                tweight += self.pstates[which].weight[i];
            }
            assert!(tweight > 0.00001, "{} < 0.00001", GPoint(tweight));
        }

        tweight = 0.0;
        let inv_dt = 1.0 / dt;

        for i in 0..self.nparticles {
            self.pstates[which].update_particle_state(i, dt, 1, &mut self.rng);
            let gp = self.gps.gps_prob(&self.pstates[which], i, self.gps_var);
            let ip = self.imu.imu_prob(&self.pstates[which], i, inv_dt);
            let w = gp * ip * self.pstates[which].weight[i];

            #[cfg(feature = "debug")]
            {
                if i == 0 {
                    eprintln!("gp={} ip={} w={}", GPoint(gp), GPoint(ip), GPoint(w));
                }
            }

            self.pstates[which].weight[i] = w;
            tweight += w;
        }

        #[cfg(feature = "debug")]
        assert!(tweight > 0.00001, "{} < 0.00001", GPoint(tweight));

        let invtweight = 1.0 / tweight;
        for i in 0..self.nparticles {
            self.pstates[which].weight[i] *= invtweight;
        }

        if !self.best_particle {
            for i in 0..self.nparticles {
                let w = self.pstates[which].weight[i];
                est_posn_x += w * self.pstates[which].posn_x[i];
                est_posn_y += w * self.pstates[which].posn_y[i];
                est_vel_r += w * self.pstates[which].vel_r[i];
                est_vel_t = normalize_angle(est_vel_t + w * self.pstates[which].vel_t[i]);
            }
        }

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

        self.resample_count = (self.resample_count + 1) % self.resample_interval;
        if self.resample_count == 0 {
            let [ref mut p0, ref mut p1] = self.pstates;
            let (current, new) = if self.which_particle {
                (p1, p0)
            } else {
                (p0, p1)
            };
            best = self.resampler.resample(
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

        let which = self.which_particle as usize;
        best_weight = self.pstates[which].weight[0];
        worst_weight = self.pstates[which].weight[0];
        best = 0;
        #[cfg(feature = "diagnostic-print")]
        {
            worst = 0;
        }

        for i in 1..self.nparticles {
            if self.pstates[which].weight[i] > best_weight {
                best = i;
                best_weight = self.pstates[which].weight[i];
            } else if self.pstates[which].weight[i] < worst_weight {
                #[cfg(feature = "diagnostic-print")]
                {
                    worst = i;
                }
                worst_weight = self.pstates[which].weight[i];
            }
        }

        #[cfg(feature = "diagnostic-print")]
        {
            print!(
                "  {} {} {}",
                GPoint(best_weight),
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
