use crate::{
    consts::{AVAR, BOX_DIM, COS_DIRN, FAST_DIRECTION, NDIRNS, PI_OVER_TWO, RVAR, TWO_PI},
    types::{ACoord, BounceProblem, CCoord},
    utils::{angle_dirn, clip_box, clip_speed, normalize_angle, normalize_dirn},
};
use std::f64::consts::PI;
use ziggurat_rs::Ziggurat;

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
