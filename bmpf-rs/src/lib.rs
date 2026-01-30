use rand::distr::{Distribution, Uniform};

pub fn uniform() -> f64 {
    let mut rng = rand::rng();
    let uniform = Uniform::new::<f64, f64>(f64::MIN, f64::MAX)
        .expect("Unable to create uniform distribution for f64");
    uniform.sample(&mut rng)
}
