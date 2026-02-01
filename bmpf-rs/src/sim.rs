use std::f64::consts::PI;

pub static BOX_DIM: f64 = 20.0;
static MAX_SPEED: f64 = 2.0;

pub static AVAR: f64 = PI / 32f64;
pub static RVAR: f64 = 0.1f64;
pub static mut GPS_VAR: f64 = 1.0f64;
pub static IMU_R_VAR: f64 = 0.5f64;
pub static IMU_A_VAR: f64 = PI / 8.0f64;
pub static NDIRNS: i32 = 1024;

pub static FAST_DIRECTION: i32 = 1;
static COS_DIRN: [f32; NDIRNS as usize] = [0f32; NDIRNS as usize];

#[derive(Clone, Copy)]
pub struct CosDirn {
    pub data: [f64; NDIRNS as usize],
}

impl CosDirn {
    pub fn init_dirn(&mut self) {
        for i in 0..NDIRNS {
            let t = i as f64 * 2.0f64 * PI / NDIRNS as f64;
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
    (t * NDIRNS as f64 / (2.0f64 * PI)).floor() as i32 % NDIRNS
}

#[inline]
pub fn normalize_dirn(mut d: i32) -> i32 {
    while d < 0 {
        d += NDIRNS;
    }
    d % NDIRNS
}

#[inline]
pub fn normalize_angle(mut t: f64) -> f64 {
    while t >= 2.0f64 * PI {
        t -= 2.0f64 * PI;
    }
    while t < 0.0f64 {
        t += 2.0f64 * PI;
    }
    t
}

#[inline]
pub fn clip(x: f64, low: f64, high: f64) -> f64 {
    high.min(x.max(low))
}

#[inline]
pub fn clip_box(x: f64) -> f64 {
    clip(x, -BOX_DIM, BOX_DIM)
}

#[inline]
pub fn clip_speed(x: f64) -> f64 {
    clip(x, 0.0, MAX_SPEED)
}
