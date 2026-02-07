use crate::consts::{BOX_DIM, MAX_SPEED, NDIRNS, NEG_BOX_DIM, TWO_PI};

#[inline]
pub fn gprob(delta: f64, sd: f64) -> f64 {
    let inv_sd = 1.0 / sd;
    let scaled = delta * inv_sd;
    (-0.5 * scaled * scaled).exp()
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
