use crate::{resample::Resample, types::Particles};
use ziggurat_rs::Ziggurat;

#[derive(Default)]
pub struct Optimal {}

#[inline]
fn nform(n: i32, sort: bool, rng: &mut Ziggurat) -> f64 {
    if sort {
        return rng.polynomial(n);
    }
    1.0f64 - rng.uniform().powf(1.0 / (n + 1) as f64)
}

impl Resample for Optimal {
    fn resample(
        &mut self,
        scale: f64,
        m: usize,
        particle: &mut Particles,
        n: usize,
        new_particle: &mut Particles,
        sort: bool,
        rng: &mut Ziggurat,
    ) -> usize {
        let invscale = 1.0 / scale;
        let mut u0 = nform((n - 1) as i32, sort, rng) * scale;
        let mut j = 0;
        let mut t = 0f64;
        let mut best_w = 0f64;
        let mut best_i = 0usize;

        for i in 0..n {
            while t + particle.weight[j] < u0 && j < m {
                t += particle.weight[j];
                j += 1;
            }

            #[cfg(feature = "debug-optimal")]
            if j >= m {
                use std::process::abort;
                println!("fell off end s={:.14} t={:.14} u={:.14}", scale, t, u0);
                abort();
            }

            new_particle.copy_from(i, particle, j);
            new_particle.weight[i] *= invscale;

            if new_particle.weight[i] > best_w {
                best_w = new_particle.weight[i];
                best_i = i;
            }

            u0 = u0 + (scale - u0) * nform((n - i - 1) as i32, sort, rng);
        }

        best_i
    }
}
