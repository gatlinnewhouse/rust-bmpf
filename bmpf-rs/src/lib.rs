use rand::distr::{Distribution, Uniform};

#[cfg(feature = "boxmuller")]
mod boxmuller;
#[cfg(feature = "erfinv")]
mod erfinv;
pub mod sim;
pub mod types;

pub fn uniform() -> f64 {
    let mut rng = rand::rng();
    let uniform =
        Uniform::new::<f64, f64>(0.0, 1.0).expect("Unable to create uniform distribution for f64");
    uniform.sample(&mut rng)
}
