use libc::RAND_MAX;

#[cfg(feature = "boxmuller")]
pub mod boxmuller;
#[cfg(feature = "erfinv")]
pub mod erfinv;
pub mod resample;
pub mod sim;
pub mod types;

pub fn uniform() -> f32 {
    unsafe { libc::rand() as f32 / RAND_MAX as f32 }
}
