use crate::{resample::Resample, types::Particles};
use ziggurat_rs::Ziggurat;

#[derive(Default)]
pub struct Regular {}

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
        let invscale = 1.0 / scale;
        let mut best_w = 0f64;
        let mut best_i = 0usize;

        // Shuffle if requested
        if sort {
            for i in 0..m.saturating_sub(1) {
                let j = rng.rand32() as usize % (m - i) + i;
                particle.swap(j, i);
            }
        }

        // Systematic resampling
        let step = scale / (n + 1) as f64;
        let mut u0 = step;
        let mut j = 0;
        let mut t = 0f64;

        for i in 0..n {
            while t + particle.weight[j] < u0 && j < m {
                t += particle.weight[j];
                j += 1;
            }

            #[cfg(feature = "debug-regular")]
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

            u0 += step;
        }

        best_i
    }
}
