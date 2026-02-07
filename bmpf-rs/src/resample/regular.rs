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

        // Shuffle
        if sort {
            for i in 0..m - 1 {
                let j = rng.rand32() as usize % (m - i) + i;
                particle.data.swap(j, i);
            }
        }

        // Merge
        let mut u0 = scale / (n + 1) as f64;
        let mut j = 0;
        let mut t = 0f64;
        for i in 0..n {
            while t + particle.data[j].weight < u0 && j < m {
                t += particle.data[j].weight;
                j += 1;
            }
            #[cfg(feature = "debug-regular")]
            if j >= m {
                use std::process::abort;

                println!("fell of end s={:.14} t ={:.14} u={:.14}", scale, t, u0);
                abort();
            }

            new_particle.data[i] = particle.data[j];
            new_particle.data[i].weight *= invscale;
            if new_particle.data[i].weight > best_w {
                best_w = new_particle.data[i].weight;
                best_i = i;
            }
            u0 += scale / (n + 1) as f64;
        }
        best_i
    }
}
