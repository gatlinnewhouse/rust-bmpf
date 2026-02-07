use crate::{resample::Resample, types::Particles};
use multiversion::multiversion;
use std::process::abort;
use ziggurat_rs::Ziggurat;

pub struct Naive {
    cumsum: Vec<f64>,
}

impl Default for Naive {
    fn default() -> Self {
        Self { cumsum: Vec::new() }
    }
}

impl Naive {
    fn ensure_capacity(&mut self, n: usize) {
        if self.cumsum.len() < n {
            self.cumsum.resize(n, 0.0);
        }
    }
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
        self.ensure_capacity(m);

        if sort {
            particle.sort_by_weight();
        }

        // Build prefix sum once
        build_prefix_sum(particle.weight.as_slice(), &mut self.cumsum[..m]);

        let mut best_w = 0.0;
        let mut best_i = 0;
        let invscale = 1.0 / scale;

        for i in 0..n {
            let target = rng.uniform() * scale;
            let src = find_sample_index(&self.cumsum[..m], target);

            new_particle.copy_from(i, particle, src);
            let w = new_particle.weight[i] * invscale;
            new_particle.weight[i] = w;

            if w > best_w {
                best_w = w;
                best_i = i;
            }
        }

        best_i
    }
}

/// Build prefix sum
#[multiversion(targets = "simd")]
fn build_prefix_sum(weights: &[f64], cumsum: &mut [f64]) {
    let mut sum = 0.0;
    for (w, c) in weights.iter().zip(cumsum.iter_mut()) {
        sum += w;
        *c = sum;
    }
}

/// Binary search for sample index
#[inline]
fn find_sample_index(cumsum: &[f64], target: f64) -> usize {
    match cumsum.binary_search_by(|x| x.partial_cmp(&target).unwrap_or(std::cmp::Ordering::Less)) {
        Ok(i) => i,
        Err(i) => {
            if i >= cumsum.len() {
                #[cfg(feature = "debug-naive")]
                {
                    println!(
                        "total {} < target {}",
                        cumsum.last().unwrap_or(&0.0),
                        target
                    );
                    abort();
                }
                #[cfg(not(feature = "debug-naive"))]
                {
                    cumsum.len().saturating_sub(1)
                }
            } else {
                i
            }
        }
    }
}
