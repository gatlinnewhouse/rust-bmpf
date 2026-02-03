use crate::{polynomial, resample::Resample, uniform};

#[derive(Default)]
pub struct Optimal {}

#[inline]
fn nform(n: i32, sort: bool) -> f64 {
    if sort {
        return polynomial(n);
    }
    1.0f64 - uniform().powf(1.0 / (n + 1) as f64)
}

impl Resample for Optimal {
    fn resample(
        &mut self,
        scale: f64,
        m: usize,
        particle: &mut crate::types::Particles,
        n: usize,
        new_particle: &mut crate::types::Particles,
        sort: bool,
    ) -> usize {
        let invscale = 1.0 / scale;
        let mut u0 = nform((n - 1) as i32, sort) * scale;
        let mut j = 0;
        let mut t = 0f64;
        let mut best_w = 0f64;
        let mut best_i = 0usize;
        for i in 0..n {
            while t + particle.data[j].weight < u0 && j < m {
                t += particle.data[j].weight;
                j += 1;
            }
            #[cfg(feature = "debug-optimal")]
            if j >= m {
                use std::process::abort;

                println!("fell of end s={:.14} t ={:.14} u={:.14}", scale, t, u0);
                abort();
            }

            new_particle.data[i] = particle.data[j].clone();
            new_particle.data[i].weight *= invscale;
            if new_particle.data[i].weight > best_w {
                best_w = new_particle.data[i].weight;
                best_i = i;
            }
            u0 = u0 + (scale - u0) * nform((n - i - 1) as i32, sort);
        }
        best_i
    }
}
