use crate::{resample::Resample, types::Particles};
use std::process::abort;
use ziggurat_rs::Ziggurat;

#[derive(Default)]
pub struct Naive {}

/// Returns the index of a weighted-sampled particle
fn weighted_sample_index(scale: f64, m: usize, particles: &Particles, rng: &mut Ziggurat) -> usize {
    let w = rng.uniform() * scale;
    let mut t = 0f64;
    for i in 0..m {
        t += particles.weight[i];
        if t >= w {
            return i;
        }
    }
    #[cfg(feature = "debug-naive")]
    println!("total {} < target {}", t, w);
    abort();
}

impl Resample for Naive {
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
        let mut best_w = 0f64;
        let mut best_i = 0usize;
        let invscale = 1.0 / scale;

        if sort {
            particle.sort_by_weight();
        }

        for i in 0..n {
            let src = weighted_sample_index(scale, m, particle, rng);
            new_particle.copy_from(i, particle, src);
            new_particle.weight[i] *= invscale;
            if new_particle.weight[i] > best_w {
                best_w = new_particle.weight[i];
                best_i = i;
            }
        }
        best_i
    }
}

