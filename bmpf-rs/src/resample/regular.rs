use crate::{resample::Resample, types::Particles};
use multiversion::multiversion;
use ziggurat_rs::Ziggurat;

pub struct Regular {
    cumsum: Vec<f64>,
    indices: Vec<usize>,
}

impl Default for Regular {
    fn default() -> Self {
        Self {
            cumsum: Vec::new(),
            indices: Vec::new(),
        }
    }
}

impl Regular {
    fn ensure_capacity(&mut self, n: usize) {
        if self.cumsum.len() < n {
            self.cumsum.resize(n, 0.0);
            self.indices.resize(n, 0);
        }
    }
}

impl Resample for Regular {
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
        self.ensure_capacity(m.max(n));

        // Shuffle if requested
        if sort {
            for i in 0..m.saturating_sub(1) {
                let j = rng.rand32() as usize % (m - i) + i;
                particle.swap(j, i);
            }
        }

        // Build prefix sum (cumulative weights)
        build_prefix_sum(particle.weight.as_slice(), &mut self.cumsum[..m]);

        // Generate all sample points
        let step = scale / (n + 1) as f64;
        generate_sample_indices(&self.cumsum[..m], step, n, &mut self.indices[..n]);

        // Copy particles based on indices and find best
        let invscale = 1.0 / scale;
        let best_i = copy_and_scale_particles(particle, new_particle, &self.indices[..n], invscale);

        best_i
    }
}

/// Build prefix sum (cumulative sum of weights)
#[multiversion(targets = "simd")]
fn build_prefix_sum(weights: &[f64], cumsum: &mut [f64]) {
    let mut sum = 0.0;
    for (w, c) in weights.iter().zip(cumsum.iter_mut()) {
        sum += w;
        *c = sum;
    }
}

/// Generate sample indices using systematic resampling
/// u[i] = step * (i + 1), find j where cumsum[j-1] < u[i] <= cumsum[j]
#[multiversion(targets = "simd")]
fn generate_sample_indices(cumsum: &[f64], step: f64, n: usize, indices: &mut [usize]) {
    let m = cumsum.len();
    let mut j = 0;
    let mut u = step;

    for i in 0..n {
        // Advance j until cumsum[j] >= u
        while j < m && cumsum[j] < u {
            j += 1;
        }
        // Clamp to valid range
        indices[i] = j.min(m.saturating_sub(1));
        u += step;
    }
}

/// Copy particles and scale weights, return best index
#[multiversion(targets = "simd")]
fn copy_and_scale_particles(
    src: &Particles,
    dst: &mut Particles,
    indices: &[usize],
    invscale: f64,
) -> usize {
    let mut best_w = 0.0;
    let mut best_i = 0;

    for (i, &src_idx) in indices.iter().enumerate() {
        dst.posn_x[i] = src.posn_x[src_idx];
        dst.posn_y[i] = src.posn_y[src_idx];
        dst.vel_r[i] = src.vel_r[src_idx];
        dst.vel_t[i] = src.vel_t[src_idx];
        let w = src.weight[src_idx] * invscale;
        dst.weight[i] = w;

        if w > best_w {
            best_w = w;
            best_i = i;
        }
    }

    best_i
}
