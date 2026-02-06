use crate::{
    gaussian,
    resample::{Resample, Resampler},
    sim::{
        AVAR, BOX_DIM, CosDirn, FAST_DIRECTION, GPS_VAR, IMU_A_VAR, IMU_R_VAR, MAX_SPEED, NDIRNS,
        RVAR, angle_dirn, clip_box, clip_speed, normalize_angle, normalize_dirn,
    },
    uniform,
};
use gpoint::GPoint;
use std::{cmp::Ordering, f64::consts::PI, fs::OpenOptions, io::Write};

#[derive(Default, Clone, Copy)]
pub struct CCoord {
    pub x: f64,
    pub y: f64,
}

fn gprob(delta: f64, sd: f64) -> f64 {
    (-0.5 * delta * delta / (sd * sd)).exp()
}

impl CCoord {
    fn gps_measure(&self) -> CCoord {
        let mut result = *self;
        result.x += gaussian(unsafe { GPS_VAR });
        result.y += gaussian(unsafe { GPS_VAR });
        result
    }

    fn gps_prob(&self, state: &VehicleState) -> f64 {
        if state.posn.x < -BOX_DIM
            || state.posn.x > BOX_DIM
            || state.posn.y < -BOX_DIM
            || state.posn.y > BOX_DIM
        {
            return 0.0;
        }
        let px = gprob(state.posn.x - self.x, unsafe { GPS_VAR });
        let py = gprob(state.posn.y - self.y, unsafe { GPS_VAR });
        px * py
    }
}

#[derive(Default, Clone, Copy)]
pub struct ACoord {
    pub r: f64,
    pub t: f64,
}

impl ACoord {
    fn measure(&self, dt: f64) -> ACoord {
        let mut result = *self;
        result.r += gaussian(IMU_R_VAR * dt);
        result.t = normalize_angle(result.t + gaussian(IMU_A_VAR * dt));
        if result.r < 0.0 {
            result.r = -result.r;
            result.t = normalize_angle(result.t + PI);
        }
        result
    }

    fn imu_prob(&self, state: &VehicleState, dt: f64) -> f64 {
        if state.vel.r < 0.0 || state.vel.r > MAX_SPEED {
            return 0.0;
        }
        let pr = gprob(state.vel.r - self.r, IMU_R_VAR / dt);
        let dth = (state.vel.t - self.t)
            .abs()
            .min(((state.vel.t - self.t).abs() - 2.0 * PI).abs());
        let pt = gprob(dth, IMU_A_VAR / dt);
        pr * pt
    }
}

#[derive(PartialEq, Debug)]
enum BounceProblem {
    BounceOk,
    BounceX,
    BounceY,
    BounceXY,
}

#[derive(Clone, Default, Copy)]
pub struct VehicleState {
    pub posn: CCoord,
    vel: ACoord,
    cos_dirn: CosDirn,
}

impl VehicleState {
    #[inline]
    pub fn gps_measure(&self) -> CCoord {
        self.posn.gps_measure()
    }

    #[inline]
    pub fn imu_measure(&self, dt: f64) -> ACoord {
        self.vel.measure(dt)
    }

    fn bounce(&mut self, r: f64, t: f64, dt: f64, _noise: i32) -> BounceProblem {
        let dc0;
        let dms0;
        let mut x0;
        let mut y0;
        let mut x1;
        let mut y1;
        if FAST_DIRECTION == 1 {
            dc0 = angle_dirn(t);
            dms0 = normalize_dirn(dc0 + NDIRNS / 4);
            x0 = self.posn.x + r * self.cos_dirn.data[dc0 as usize] * dt;
            y0 = self.posn.y + r * self.cos_dirn.data[dms0 as usize] * dt;
        } else {
            x0 = self.posn.x + r * t.cos() * dt;
            y0 = self.posn.y - r * t.sin() * dt;
        }
        x1 = clip_box(x0);
        y1 = clip_box(y0);
        if x0 == x1 && y0 == y1 {
            self.posn.x = x1;
            self.posn.y = y1;
            self.vel.t = t;
            self.vel.r = r;
            return BounceProblem::BounceOk;
        }
        if FAST_DIRECTION == 1 {
            x0 = self.posn.x + r * t.cos() * dt;
            y0 = self.posn.y - r * t.sin() * dt;
            x1 = clip_box(x0);
            y1 = clip_box(y0);
            if x0 == x1 && y0 == y1 {
                self.posn.x = x1;
                self.posn.y = y1;
                self.vel.t = t;
                self.vel.r = r;
                return BounceProblem::BounceOk;
            }
        }
        if y0 == y1 {
            return BounceProblem::BounceX;
        } else if x0 == x1 {
            return BounceProblem::BounceY;
        }
        BounceProblem::BounceXY
    }

    pub fn init_state(&mut self) {
        self.posn.x = (uniform() * 2.0 - 1.0) * BOX_DIM;
        self.posn.y = (uniform() * 2.0 - 1.0) * BOX_DIM;
        self.vel.r = uniform();
        self.vel.t = normalize_angle(uniform() * (PI / 2.0f64));
        self.cos_dirn.init_dirn();
    }

    pub fn update_state(&mut self, dt: f64, noise: i32) {
        let mut r0 = clip_speed(self.vel.r + gaussian(RVAR) * ((1 + 8 * noise) as f64));
        let mut t0 = normalize_angle(self.vel.t + gaussian(AVAR) * ((1 + 8 * noise) as f64));
        let mut b = self.bounce(r0, t0, dt, noise);
        if b != BounceProblem::BounceOk {
            r0 = self.vel.r;
            t0 = self.vel.t;
            b = self.bounce(r0, t0, dt, 0);
            match b {
                BounceProblem::BounceOk => (),
                BounceProblem::BounceX => {
                    t0 = normalize_angle(PI - t0);
                    b = self.bounce(r0, t0, dt, 0);
                }
                BounceProblem::BounceY => {
                    t0 = normalize_angle(2.0 * PI - t0);
                    b = self.bounce(r0, t0, dt, 0);
                }
                BounceProblem::BounceXY => {
                    t0 = normalize_angle(PI + t0);
                    b = self.bounce(r0, t0, dt, 0)
                }
            }
        }
        assert!(b == BounceProblem::BounceOk, "{:?} != BounceOk", b)
    }
}

#[derive(Default, Clone, Copy)]
pub struct ParticleInfo {
    pub state: VehicleState,
    pub weight: f64,
}

#[inline]
fn sgn(x: f64) -> Ordering {
    if x < 0.0 {
        return Ordering::Less;
    } else if x > 0.0 {
        return Ordering::Greater;
    }
    Ordering::Equal
}

impl ParticleInfo {
    pub fn cmp_weight(&self, other: &Self) -> std::cmp::Ordering {
        sgn(self.weight - other.weight).reverse()
    }
}

#[derive(Clone)]
pub struct Particles {
    pub data: Vec<ParticleInfo>,
}

impl Default for Particles {
    fn default() -> Self {
        Self {
            data: vec![ParticleInfo::default(); 100],
        }
    }
}

impl Particles {
    fn new(nparticles: usize) -> Self {
        Self {
            data: vec![ParticleInfo::default(); nparticles],
        }
    }
}

pub struct BpfState {
    pstates: Vec<Particles>,
    which_particle: bool,
    resampler: Resampler,
    sort: bool,
    nparticles: usize,
    pub report_particles: i32,
    best_particle: bool,
    resample_interval: usize,
    resample_count: usize,
    pub vehicle: CCoord,
    gps: CCoord,
    imu: ACoord,
}

impl Default for BpfState {
    fn default() -> Self {
        Self {
            pstates: vec![Particles::default(); 2],
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
    ) -> Self {
        Self {
            pstates: vec![Particles::new(nparticles); 2],
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
        }
    }

    pub fn init_particles(&mut self) {
        let invscale = 1.0 / self.nparticles as f64;
        self.which_particle = false;
        for particle in &mut self.pstates[0].data {
            particle.state.init_state();
            particle.weight = invscale;
        }
    }

    pub fn parse_line(&mut self, line: String) -> i32 {
        let measures = line.split(" ").collect::<Vec<&str>>();
        self.vehicle.x = measures[1]
            .parse::<f64>()
            .expect("Failed to parse vehicle x to f64");
        self.vehicle.y = measures[2]
            .parse::<f64>()
            .expect("Failed to parse vehicle y to f64");
        self.gps.x = measures[3]
            .parse::<f64>()
            .expect("Failed to parse gps x to f64");
        self.gps.y = measures[4]
            .parse::<f64>()
            .expect("Failed to parse gps y to f64");
        self.imu.r = measures[5]
            .parse::<f64>()
            .expect("Failed to parse imu r to f64");
        self.imu.t = measures[6]
            .parse::<f64>()
            .expect("Failed to parse imu t to f64");

        measures[0]
            .parse::<i32>()
            .expect("Failed to parse t_ms return value to i32")
    }

    pub fn bpf_step(&mut self, t: f64, dt: f64, report: bool) {
        let mut tweight;
        let mut best;
        #[cfg(feature = "diagnostic-print")]
        let mut worst = 0usize;
        let mut best_weight;
        let mut worst_weight;
        let mut est_state = VehicleState::default();
        // est_state.init_state();
        #[cfg(feature = "debug")]
        {
            tweight = 0.0;
            for p in self.pstates[self.which_particle as usize].data {
                tweight += p.weight;
            }
            assert!(tweight > 0.00001, "{} < 0.00001", GPoint(tweight));
        }
        tweight = 0.0;
        for (i, p) in self.pstates[self.which_particle as usize]
            .data
            .iter_mut()
            .enumerate()
        {
            p.state.update_state(dt, 1);
            let gp = self.gps.gps_prob(&p.state);
            let ip = self.imu.imu_prob(&p.state, dt);
            let w = gp * ip * p.weight;
            #[cfg(feature = "debug")]
            {
                if i == 0 {
                    eprintln!("gp={} ip={} w={}", GPoint(gp), GPoint(ip), GPoint(w));
                    eprintln!(
                        "gps=({} {}), imu=(r={}, t={})",
                        GPoint(self.gps.x),
                        GPoint(self.gps.y),
                        GPoint(self.imu.r),
                        GPoint(self.imu.t)
                    );
                }
            }
            p.weight = w;
            tweight += w;
        }
        #[cfg(feature = "debug")]
        assert!(tweight > 0.00001, "{} < 0.00001", GPoint(tweight));
        let invtweight = 1.0 / tweight;
        for p in self.pstates[self.which_particle as usize].data.iter_mut() {
            p.weight *= invtweight;
        }
        est_state.posn.x = 0.0;
        est_state.posn.y = 0.0;
        est_state.vel.r = 0.0;
        est_state.vel.t = 0.0;
        if !self.best_particle {
            for p in self.pstates[self.which_particle as usize].data.iter() {
                let s = &p.state;
                let w = p.weight;
                est_state.posn.x += w * s.posn.x;
                est_state.posn.y += w * s.posn.y;
                est_state.vel.r += w * s.vel.r;
                est_state.vel.t = normalize_angle(est_state.vel.t + w * s.vel.t);
            }
        }
        if report {
            let filename = format!("benchtmp/particles-{}.dat", t);
            let mut file = OpenOptions::new()
                .append(true)
                .create(true)
                .open(filename)
                .unwrap_or_else(|_| panic!("Could not open file at benchtmp/particles-{}.dat", t));
            for p in self.pstates[self.which_particle as usize].data.iter() {
                let px = p.state.posn.x;
                let py = p.state.posn.y;
                let w = p.weight;
                if let Err(e) = writeln!(file, "{} {} {}", GPoint(px), GPoint(py), GPoint(w)) {
                    eprintln!("Could not write to benchtmp/particles-{}.dat: {}", t, e)
                }
            }
        }
        self.resample_count = (self.resample_count + 1) % self.resample_interval;
        if self.resample_count == 0 {
            let mut new_particle = self.pstates[!self.which_particle as usize].clone();
            best = self.resampler.resample(
                tweight,
                self.nparticles,
                &mut self.pstates[self.which_particle as usize],
                self.nparticles,
                &mut new_particle,
                self.sort,
            );
            self.pstates[!self.which_particle as usize] = new_particle.clone();
            self.which_particle = !self.which_particle;
            for p in self.pstates[self.which_particle as usize].data.iter_mut() {
                p.weight = 1.0 / self.nparticles as f64;
            }
        }
        {
            best_weight = self.pstates[self.which_particle as usize].data[0].weight;
            worst_weight = self.pstates[self.which_particle as usize].data[0].weight;
            best = 0;
            #[cfg(feature = "diagnostic-print")]
            {
                worst = 0;
            }
            for (i, p) in self.pstates[self.which_particle as usize]
                .data
                .iter()
                .enumerate()
                .skip(1)
            {
                if p.weight > best_weight {
                    best = i;
                    best_weight = p.weight;
                } else if p.weight < worst_weight {
                    #[cfg(feature = "diagnostic-print")]
                    {
                        worst = i;
                    }
                    worst_weight = p.weight;
                }
            }
        }
        #[cfg(feature = "diagnostic-print")]
        {
            print!(
                "  {} {} {}",
                GPoint(best_weight),
                GPoint(
                    self.pstates[self.which_particle as usize].data[best]
                        .state
                        .posn
                        .x
                ),
                GPoint(
                    self.pstates[self.which_particle as usize].data[best]
                        .state
                        .posn
                        .y
                ),
            );
            print!(
                "  {} {} {}",
                GPoint(worst_weight),
                GPoint(
                    self.pstates[self.which_particle as usize].data[worst]
                        .state
                        .posn
                        .x
                ),
                GPoint(
                    self.pstates[self.which_particle as usize].data[worst]
                        .state
                        .posn
                        .y
                ),
            );
        }
        #[cfg(not(feature = "diagnostic-print"))]
        {
            print!(
                "  {} {}",
                GPoint(
                    self.pstates[self.which_particle as usize].data[best]
                        .state
                        .posn
                        .x
                ),
                GPoint(
                    self.pstates[self.which_particle as usize].data[best]
                        .state
                        .posn
                        .y
                )
            );
        }
        if !self.best_particle {
            print!(
                "  {} {}",
                GPoint(est_state.posn.x),
                GPoint(est_state.posn.y)
            );
        }
    }
}
