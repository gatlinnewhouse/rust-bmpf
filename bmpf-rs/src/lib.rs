use rand::Rng;

#[cfg(feature = "boxmuller")]
pub mod boxmuller;
#[cfg(feature = "erfinv")]
pub mod erfinv;
pub mod resample;
pub mod sim;
pub mod types;

pub fn uniform() -> f64 {
    rand::rng().random()
}
