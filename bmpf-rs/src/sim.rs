use std::{f64::consts::PI, sync::LazyLock};

pub const BOX_DIM: f64 = 20.0;
pub const MAX_SPEED: f64 = 2.0;

pub const AVAR: f64 = PI / 32.0;
pub const RVAR: f64 = 0.1;
pub const DEFAULT_GPS_VAR: f64 = 1.0;
pub const IMU_R_VAR: f64 = 0.5;
pub const IMU_A_VAR: f64 = PI / 8.0;
pub const NDIRNS: i32 = 1024;

pub const FAST_DIRECTION: i32 = 0;

pub const TWO_PI: f64 = 2.0 * PI;
pub const PI_OVER_TWO: f64 = PI / 2.0;
pub const NEG_BOX_DIM: f64 = -BOX_DIM;

pub static COS_DIRN: LazyLock<CosDirn> = LazyLock::new(|| {
    let mut cd = CosDirn::default();
    cd.init_dirn();
    cd
});

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

#[inline]
pub fn angle_dirn(t: f64) -> i32 {
    (t * NDIRNS as f64 / (TWO_PI)).floor() as i32 % NDIRNS
}

#[inline]
pub fn normalize_dirn(d: i32) -> i32 {
    d.rem_euclid(NDIRNS)
}

#[inline]
pub fn normalize_angle(t: f64) -> f64 {
    t.rem_euclid(TWO_PI)
}

#[inline]
pub fn clip(x: f64, low: f64, high: f64) -> f64 {
    x.clamp(low, high)
}

#[inline]
pub fn clip_box(x: f64) -> f64 {
    clip(x, NEG_BOX_DIM, BOX_DIM)
}

#[inline]
pub fn clip_speed(x: f64) -> f64 {
    clip(x, 0.0, MAX_SPEED)
}
