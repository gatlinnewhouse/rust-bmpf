#[cfg(feature = "boxmuller")]
use crate::boxmuller;
#[cfg(feature = "erfinv")]
use crate::erfinv;
use crate::{
    boxmuller::gaussian,
    sim::{
        AVAR, BOX_DIM, CosDirn, FAST_DIRECTION, GPS_VAR, NDIRNS, RVAR, angle_dirn, clip_box,
        clip_speed, normalize_angle, normalize_dirn,
    },
    uniform,
};
use std::f64::consts::PI;

#[derive(Default, Clone)]
pub struct CCoord {
    pub x: f64,
    pub y: f64,
}

impl CCoord {
    fn gps_measure(&self) -> CCoord {
        let mut result = self.clone();
        #[cfg(feature = "boxmuller")]
        {
            result.x += unsafe { gaussian(GPS_VAR) };
            result.y += unsafe { gaussian(GPS_VAR) };
        }
        result
    }
}

#[derive(Default, Clone)]
pub struct ACoord {
    pub r: f64,
    pub t: f64,
}

impl ACoord {
    fn measure(&self, dt: f64) -> ACoord {
        let mut result = self.clone();
        #[cfg(feature = "boxmuller")]
        {
            use crate::sim::IMU_A_VAR;
            use crate::sim::IMU_R_VAR;

            result.r += unsafe { gaussian(IMU_R_VAR * dt) };
            result.t += normalize_angle(result.t + unsafe { gaussian(IMU_A_VAR * dt) });
        }
        if result.r < 0.0 {
            result.r = -result.r;
            result.t = normalize_angle(result.t + PI);
        }
        result
    }
}

#[derive(PartialEq)]
enum BounceProblem {
    BounceOk,
    BounceX,
    BounceY,
    BounceXY,
}

#[derive(Default)]
pub struct State {
    pub posn: CCoord,
    vel: ACoord,
    cos_dirn: CosDirn,
}

impl State {
    #[inline]
    pub fn gps_measure(&self) -> CCoord {
        self.posn.gps_measure()
    }

    #[inline]
    pub fn imu_measure(&self, dt: f64) -> ACoord {
        self.vel.measure(dt)
    }

    fn bounce(&mut self, r: f64, t: f64, dt: f64, noise: i32) -> BounceProblem {
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
        #[cfg(feature = "erfinv")]
        let mut r0 = clip_speed(
            self.vel.r + erfinv::gaussian(RVAR as f32) as f64 * ((1 + 8 * noise) as f64),
        );
        #[cfg(feature = "erfinv")]
        let mut t0 = normalize_angle(
            self.vel.t + erfinv::gaussian(AVAR as f32) as f64 * ((1 + 8 * noise) as f64),
        );
        #[cfg(feature = "boxmuller")]
        let mut r0 = clip_speed(
            self.vel.r + unsafe { boxmuller::gaussian(RVAR) } * ((1 + 8 * noise) as f64),
        );
        #[cfg(feature = "boxmuller")]
        let mut t0 = normalize_angle(
            self.vel.t + unsafe { boxmuller::gaussian(AVAR) } * ((1 + 8 * noise) as f64),
        );
        let mut b = self.bounce(r0, t0, dt, 0);
        if b != BounceProblem::BounceOk {
            r0 = self.vel.r;
            t0 = self.vel.t;
            b = self.bounce(r0, t0, dt, noise);
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
                    b = self.bounce(r0, t0, dt, noise)
                }
            }
        }
        assert!(b == BounceProblem::BounceOk)
    }
}

pub struct ParticleInfo {
    state: State,
    weight: f64,
}
