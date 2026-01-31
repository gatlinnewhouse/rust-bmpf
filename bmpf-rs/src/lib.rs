use rand::distr::{Distribution, Uniform};

#[cfg(feature = "boxmuller")]
pub mod boxmuller;
#[cfg(feature = "erfinv")]
pub mod erfinv;
pub mod resample;
pub mod sim;
pub mod types;

pub fn uniform() -> f64 {
    let mut rng = rand::rng();
    let uniform =
        Uniform::new::<f64, f64>(0.0, 1.0).expect("Unable to create uniform distribution for f64");
    uniform.sample(&mut rng)
}
