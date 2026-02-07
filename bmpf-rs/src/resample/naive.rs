use crate::{
    resample::Resample,
    types::{ParticleInfo, Particles},
};
use std::process::abort;
use ziggurat_rs::Ziggurat;

#[derive(Default)]
pub struct Naive {}

fn weighted_sample<'a>(
    scale: f64,
    m: usize,
    particles: &'a Particles,
    rng: &'a mut Ziggurat,
) -> &'a ParticleInfo {
    let w = rng.uniform() * scale;
    let mut t = 0f64;
    for i in 0..m {
        t += particles.data[i].weight;
        if t >= w {
            return &particles.data[i];
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
            particle.data.sort_by(|a, b| a.cmp_weight(b));
        }
        for i in 0..n {
            new_particle.data[i] = *weighted_sample(scale, m, particle, rng);
            new_particle.data[i].weight *= invscale;
            if new_particle.data[i].weight > best_w {
                best_w = new_particle.data[i].weight;
                best_i = i;
            }
        }
        best_i
    }
}
