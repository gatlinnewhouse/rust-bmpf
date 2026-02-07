use crate::{
    aligned_vec::AVec,
    resample::{Resample, Resampler},
    sim::{
        AVAR, BOX_DIM, COS_DIRN, DEFAULT_GPS_VAR, FAST_DIRECTION, IMU_A_VAR, IMU_R_VAR, MAX_SPEED,
        NDIRNS, NEG_BOX_DIM, PI_OVER_TWO, RVAR, TWO_PI, angle_dirn, clip_box, clip_speed,
        normalize_angle, normalize_dirn,
    },
};
use gpoint::GPoint;
use std::{cmp::Ordering, f64::consts::PI, fmt::Write, fs::OpenOptions, io::BufWriter};
use ziggurat_rs::Ziggurat;

#[derive(Default, Clone, Copy)]
#[repr(C)]
pub struct CCoord {
    pub x: f64,
    pub y: f64,
}

#[derive(Default, Clone, Copy)]
#[repr(C)]
pub struct ACoord {
    pub r: f64,
    pub t: f64,
}

#[derive(Clone, Copy)]
pub struct PosnRef<'a> {
    pub x: &'a f64,
    pub y: &'a f64,
}

#[derive(Clone, Copy)]
pub struct VelRef<'a> {
    pub r: &'a f64,
    pub t: &'a f64,
}

#[derive(Clone, Copy)]
pub struct ParticleRef<'a> {
    pub posn_x: &'a f64,
    pub posn_y: &'a f64,
    pub vel_r: &'a f64,
    pub vel_t: &'a f64,
    pub weight: &'a f64,
}

#[inline]
fn gprob(delta: f64, sd: f64) -> f64 {
    let inv_sd = 1.0 / sd;
    let scaled = delta * inv_sd;
    (-0.5 * scaled * scaled).exp()
}

impl CCoord {
    pub fn gps_measure(&self, rng: &mut Ziggurat, gps_var: f64) -> CCoord {
        CCoord {
            x: self.x + rng.gaussian(gps_var),
            y: self.y + rng.gaussian(gps_var),
        }
    }

    #[inline]
    pub fn gps_prob(&self, particles: &Particles, i: usize, gps_var: f64) -> f64 {
        let px = particles.posn_x[i];
        let py = particles.posn_y[i];

        if px < NEG_BOX_DIM || px > BOX_DIM || py < NEG_BOX_DIM || py > BOX_DIM {
            return 0.0;
        }
        gprob(px - self.x, gps_var) * gprob(py - self.y, gps_var)
    }
}

impl ACoord {
    pub fn measure(&self, dt: f64, rng: &mut Ziggurat) -> ACoord {
        let mut r = self.r + rng.gaussian(IMU_R_VAR * dt);
        let mut t = normalize_angle(self.t + rng.gaussian(IMU_A_VAR * dt));
        if r < 0.0 {
            r = -r;
            t = normalize_angle(t + PI);
        }
        ACoord { r, t }
    }

    #[inline]
    pub fn imu_prob(&self, particles: &Particles, i: usize, inv_dt: f64) -> f64 {
        let vr = particles.vel_r[i];
        let vt = particles.vel_t[i];

        if vr < 0.0 || vr > MAX_SPEED {
            return 0.0;
        }
        let pr = gprob(vr - self.r, IMU_R_VAR * inv_dt);
        let dth = (vt - self.t)
            .abs()
            .min(((vt - self.t).abs() - TWO_PI).abs());
        let pt = gprob(dth, IMU_A_VAR * inv_dt);
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

#[derive(Clone)]
pub struct Particles {
    // Position
    pub posn_x: AVec<f64>,
    pub posn_y: AVec<f64>,
    // Velocity
    pub vel_r: AVec<f64>,
    pub vel_t: AVec<f64>,
    // Weight
    pub weight: AVec<f64>,
}

impl Default for Particles {
    fn default() -> Self {
        Self::new(100)
    }
}

impl Particles {
    pub fn new(nparticles: usize) -> Self {
        Self {
            posn_x: AVec::new(nparticles),
            posn_y: AVec::new(nparticles),
            vel_r: AVec::new(nparticles),
            vel_t: AVec::new(nparticles),
            weight: AVec::new(nparticles),
        }
    }

    #[inline]
    pub fn len(&self) -> usize {
        self.weight.len()
    }

    #[inline]
    pub fn get(&self, i: usize) -> ParticleRef<'_> {
        ParticleRef {
            posn_x: &self.posn_x[i],
            posn_y: &self.posn_y[i],
            vel_r: &self.vel_r[i],
            vel_t: &self.vel_t[i],
            weight: &self.weight[i],
        }
    }

    #[inline]
    pub fn posn(&self, i: usize) -> PosnRef<'_> {
        PosnRef {
            x: &self.posn_x[i],
            y: &self.posn_y[i],
        }
    }

    #[inline]
    pub fn vel(&self, i: usize) -> VelRef<'_> {
        VelRef {
            r: &self.vel_r[i],
            t: &self.vel_t[i],
        }
    }

    #[inline]
    pub fn copy_within(&mut self, dst: usize, src: usize) {
        self.posn_x[dst] = self.posn_x[src];
        self.posn_y[dst] = self.posn_y[src];
        self.vel_r[dst] = self.vel_r[src];
        self.vel_t[dst] = self.vel_t[src];
        self.weight[dst] = self.weight[src];
    }

    #[inline]
    pub fn copy_from(&mut self, dst: usize, other: &Particles, src: usize) {
        self.posn_x[dst] = other.posn_x[src];
        self.posn_y[dst] = other.posn_y[src];
        self.vel_r[dst] = other.vel_r[src];
        self.vel_t[dst] = other.vel_t[src];
        self.weight[dst] = other.weight[src];
    }

    #[inline]
    pub fn swap(&mut self, i: usize, j: usize) {
        self.posn_x.swap(i, j);
        self.posn_y.swap(i, j);
        self.vel_r.swap(i, j);
        self.vel_t.swap(i, j);
        self.weight.swap(i, j);
    }

    #[inline]
    pub fn cmp_weight(&self, a: usize, b: usize) -> Ordering {
        self.weight[b]
            .partial_cmp(&self.weight[a])
            .unwrap_or(Ordering::Equal)
    }

    pub fn sort_by_weight(&mut self) {
        let n = self.len();
        if n <= 1 {
            return;
        }

        let mut indices: Vec<usize> = (0..n).collect();
        indices.sort_by(|&a, &b| self.cmp_weight(a, b));

        let mut visited = vec![false; n];

        for start in 0..n {
            if visited[start] || indices[start] == start {
                visited[start] = true;
                continue;
            }

            let mut current = start;

            let tmp_px = self.posn_x[start];
            let tmp_py = self.posn_y[start];
            let tmp_vr = self.vel_r[start];
            let tmp_vt = self.vel_t[start];
            let tmp_w = self.weight[start];

            loop {
                let next = indices[current];
                visited[current] = true;

                if next == start {
                    self.posn_x[current] = tmp_px;
                    self.posn_y[current] = tmp_py;
                    self.vel_r[current] = tmp_vr;
                    self.vel_t[current] = tmp_vt;
                    self.weight[current] = tmp_w;
                    break;
                }

                self.posn_x[current] = self.posn_x[next];
                self.posn_y[current] = self.posn_y[next];
                self.vel_r[current] = self.vel_r[next];
                self.vel_t[current] = self.vel_t[next];
                self.weight[current] = self.weight[next];

                current = next;
            }
        }
    }

    pub fn init_particle_state(&mut self, i: usize, rng: &mut Ziggurat) {
        self.posn_x[i] = (rng.uniform() * 2.0 - 1.0) * BOX_DIM;
        self.posn_y[i] = (rng.uniform() * 2.0 - 1.0) * BOX_DIM;
        self.vel_r[i] = rng.uniform();
        self.vel_t[i] = normalize_angle(rng.uniform() * PI_OVER_TWO);
    }

    pub fn update_particle_state(&mut self, i: usize, dt: f64, noise: i32, rng: &mut Ziggurat) {
        let mut r0 = clip_speed(self.vel_r[i] + rng.gaussian(RVAR) * ((1 + 8 * noise) as f64));
        let mut t0 = normalize_angle(self.vel_t[i] + rng.gaussian(AVAR) * ((1 + 8 * noise) as f64));
        let mut b = self.bounce(i, r0, t0, dt, noise);

        if b != BounceProblem::BounceOk {
            r0 = self.vel_r[i];
            t0 = self.vel_t[i];
            b = self.bounce(i, r0, t0, dt, 0);
            match b {
                BounceProblem::BounceOk => (),
                BounceProblem::BounceX => {
                    t0 = normalize_angle(PI - t0);
                    b = self.bounce(i, r0, t0, dt, 0);
                }
                BounceProblem::BounceY => {
                    t0 = normalize_angle(TWO_PI - t0);
                    b = self.bounce(i, r0, t0, dt, 0);
                }
                BounceProblem::BounceXY => {
                    t0 = normalize_angle(PI + t0);
                    b = self.bounce(i, r0, t0, dt, 0);
                }
            }
        }
        assert!(b == BounceProblem::BounceOk, "{:?} != BounceOk", b);
    }

    fn bounce(&mut self, i: usize, r: f64, t: f64, dt: f64, _noise: i32) -> BounceProblem {
        let mut x0;
        let mut y0;

        if FAST_DIRECTION == 1 {
            let dc0 = angle_dirn(t);
            let dms0 = normalize_dirn(dc0 + NDIRNS / 4);
            x0 = self.posn_x[i] + r * COS_DIRN.data[dc0 as usize] * dt;
            y0 = self.posn_y[i] + r * COS_DIRN.data[dms0 as usize] * dt;
        } else {
            x0 = self.posn_x[i] + r * t.cos() * dt;
            y0 = self.posn_y[i] - r * t.sin() * dt;
        }

        let mut x1 = clip_box(x0);
        let mut y1 = clip_box(y0);

        if x0 == x1 && y0 == y1 {
            self.posn_x[i] = x1;
            self.posn_y[i] = y1;
            self.vel_t[i] = t;
            self.vel_r[i] = r;
            return BounceProblem::BounceOk;
        }

        if FAST_DIRECTION == 1 {
            x0 = self.posn_x[i] + r * t.cos() * dt;
            y0 = self.posn_y[i] - r * t.sin() * dt;
            x1 = clip_box(x0);
            y1 = clip_box(y0);
            if x0 == x1 && y0 == y1 {
                self.posn_x[i] = x1;
                self.posn_y[i] = y1;
                self.vel_t[i] = t;
                self.vel_r[i] = r;
                return BounceProblem::BounceOk;
            }
        }

        if y0 == y1 {
            BounceProblem::BounceX
        } else if x0 == x1 {
            BounceProblem::BounceY
        } else {
            BounceProblem::BounceXY
        }
    }
}

// ============================================================================
// Single vehicle state (for ground truth simulation in examples)
// ============================================================================

/// Single vehicle state for ground truth simulation
/// This is separate from the SoA Particles structure which is for the particle filter
#[derive(Clone, Default, Copy)]
#[repr(C)]
pub struct VehicleState {
    pub posn: CCoord,
    pub vel: ACoord,
}

impl VehicleState {
    #[inline]
    pub fn gps_measure(&self, rng: &mut Ziggurat, gps_var: f64) -> CCoord {
        self.posn.gps_measure(rng, gps_var)
    }

    #[inline]
    pub fn imu_measure(&self, dt: f64, rng: &mut Ziggurat) -> ACoord {
        self.vel.measure(dt, rng)
    }

    pub fn init_state(&mut self, rng: &mut Ziggurat) {
        self.posn.x = (rng.uniform() * 2.0 - 1.0) * BOX_DIM;
        self.posn.y = (rng.uniform() * 2.0 - 1.0) * BOX_DIM;
        self.vel.r = rng.uniform();
        self.vel.t = normalize_angle(rng.uniform() * PI_OVER_TWO);
    }

    pub fn update_state(&mut self, dt: f64, noise: i32, rng: &mut Ziggurat) {
        let mut r0 = clip_speed(self.vel.r + rng.gaussian(RVAR) * ((1 + 8 * noise) as f64));
        let mut t0 = normalize_angle(self.vel.t + rng.gaussian(AVAR) * ((1 + 8 * noise) as f64));
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
                    t0 = normalize_angle(TWO_PI - t0);
                    b = self.bounce(r0, t0, dt, 0);
                }
                BounceProblem::BounceXY => {
                    t0 = normalize_angle(PI + t0);
                    b = self.bounce(r0, t0, dt, 0);
                }
            }
        }
        assert!(b == BounceProblem::BounceOk, "{:?} != BounceOk", b);
    }

    fn bounce(&mut self, r: f64, t: f64, dt: f64, _noise: i32) -> BounceProblem {
        let mut x0;
        let mut y0;

        if FAST_DIRECTION == 1 {
            let dc0 = angle_dirn(t);
            let dms0 = normalize_dirn(dc0 + NDIRNS / 4);
            x0 = self.posn.x + r * COS_DIRN.data[dc0 as usize] * dt;
            y0 = self.posn.y + r * COS_DIRN.data[dms0 as usize] * dt;
        } else {
            x0 = self.posn.x + r * t.cos() * dt;
            y0 = self.posn.y - r * t.sin() * dt;
        }

        let mut x1 = clip_box(x0);
        let mut y1 = clip_box(y0);

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
            BounceProblem::BounceX
        } else if x0 == x1 {
            BounceProblem::BounceY
        } else {
            BounceProblem::BounceXY
        }
    }
}

pub struct BpfState {
    pstates: [Particles; 2],
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
    filename_buf: String,
    rng: Ziggurat,
    inv_nparticles: f64,
    gps_var: f64,
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
