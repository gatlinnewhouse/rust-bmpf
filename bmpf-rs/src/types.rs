use crate::{
    aligned_vec::AVec,
    consts::{
        AVAR, BOX_DIM, COS_DIRN, FAST_DIRECTION, IMU_A_VAR, IMU_R_VAR, MAX_SPEED, NDIRNS,
        NEG_BOX_DIM, PI_OVER_TWO, RVAR, TWO_PI,
    },
    utils::{angle_dirn, clip_box, clip_speed, gprob, normalize_angle, normalize_dirn},
};
use std::{cmp::Ordering, f64::consts::PI};
use ziggurat_rs::Ziggurat;

#[derive(Clone, Copy)]
pub struct CosDirn {
    pub data: [f64; NDIRNS as usize],
}

impl CosDirn {
    pub fn init_dirn(&mut self) {
        for i in 0..NDIRNS {
            let t = i as f64 * TWO_PI / NDIRNS as f64;
            self.data[i as usize] = t.cos();
        }
    }
}

impl Default for CosDirn {
    fn default() -> Self {
        Self {
            data: [0.0f64; NDIRNS as usize],
        }
    }
}

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

        if !(NEG_BOX_DIM..=BOX_DIM).contains(&px) || !(NEG_BOX_DIM..=BOX_DIM).contains(&py) {
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

        if !(0.0..=MAX_SPEED).contains(&vr) {
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
pub enum BounceProblem {
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

    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.len() == 0
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
