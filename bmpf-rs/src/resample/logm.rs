use crate::{
    resample::Resample,
    types::{ParticleInfo, Particles},
};
use std::process::abort;
use ziggurat_rs::Ziggurat;

#[cfg(feature = "debug-heapify")]
static DW: f64 = 1.0e9;

pub struct Logm {
    tweight: Vec<f64>,
    #[cfg(feature = "debug-logm")]
    total_depth: usize,
}

impl<'a> Logm {
    pub fn new(mmax: usize) -> Self {
        Self {
            tweight: vec![0f64; mmax],
            #[cfg(feature = "debug-logm")]
            total_depth: 0,
        }
    }

    fn weighted_sample(
        &'a mut self,
        scale: f64,
        m: usize,
        particles: &'a Particles,
        rng: &'a mut Ziggurat,
    ) -> &'a ParticleInfo {
        let mut w = rng.uniform() * scale;
        #[cfg(feature = "debug-logm")]
        let mut j = 0usize;
        for mut i in 0..m {
            let left = 2 * i + 1;
            let right = 2 * i + 2;
            let mut lweight = 0f64;
            if left < m {
                lweight = self.tweight[left];
            }
            #[cfg(feature = "debug-logm")]
            {
                self.total_depth += 1;
            }
            if w < lweight {
                i = left;
                #[cfg(feature = "debug-logm")]
                {
                    j = i;
                }
                continue;
            }
            if w <= lweight + particles.data[i].weight {
                return &particles.data[i];
            }
            w -= lweight + particles.data[i].weight;
            i = right;
            #[cfg(feature = "debug-logm")]
            {
                j = i;
            }
        }
        #[cfg(feature = "debug-logm")]
        {
            j = (j - 1) / 2;
            println!(
                "fell off tree on {} with i={}, w[i]={}",
                w, j, particles.data[j].weight
            );
        }
        abort();
    }

    fn heapify(&mut self, m: usize, particles: &'a mut Particles) {
        for i in (0..(m - 1)).rev() {
            let left = 2 * i + 1;
            let right = 2 * i + 2;
            self.tweight[i] = particles.data[i].weight;
            if i >= m / 2 {
                continue;
            }
            self.tweight[i] += self.tweight[left];
            if right < m {
                self.tweight[i] += self.tweight[right];
            }
            let mut j = i;
            while j < m / 2 {
                let left = 2 * j + 1;
                let right = 2 * j + 2;
                let wj = particles.data[j].weight;
                let wleft = particles.data[left].weight;
                let mut nextj = left;
                if right < m {
                    let wright = particles.data[right].weight;
                    if wj >= wleft && wj >= wright {
                        break;
                    }
                    if wj < wright && (wj >= wleft || wright > wleft) {
                        nextj = right;
                    }
                } else {
                    if wj >= wleft {
                        break;
                    }
                }
                particles.data.swap(j, nextj);
                let dw = particles.data[j].weight - particles.data[nextj].weight;
                #[cfg(feature = "debug-logm")]
                assert!(dw >= 0.0f64);
                self.tweight[nextj] -= dw;
                j = nextj;
            }
        }
    }

    pub fn init_tweights(&mut self, m: usize, particles: &'a Particles) {
        let mut j = 0usize;
        for i in ((m / 2)..(m - 1)).rev() {
            self.tweight[i] = particles.data[i].weight;
            j = i - 1; // --i in for loop happens at end
        }
        self.tweight[j] = particles.data[j].weight;
        let mut left = 2 * j + 1;
        let mut right = 2 * j + 2;
        if right >= m {
            self.tweight[j] = particles.data[j].weight + particles.data[left].weight;
            j -= 1;
        }
        while j > 0 {
            left = 2 * j + 1;
            right = 2 * j + 2;
            self.tweight[j] = particles.data[j].weight + self.tweight[left] + self.tweight[right];
            j -= 1;
        }
    }

    #[cfg(feature = "debug-heapify")]
    fn check_tweights(&self, m: usize, particles: &'a Particles) {
        for i in (0..(m - 1)).rev() {
            let left = 2 * i + 1;
            let right = 2 * i + 2;
            let mut w = particles.data[i].weight;
            if left < m {
                w += self.tweight[left];
            }
            if right < m {
                w += self.tweight[right];
            }
            assert!((w - self.tweight[i]).abs() <= DW);
        }
    }
}

impl Resample for Logm {
    fn resample(
        &mut self,
        _scale: f64,
        m: usize,
        particle: &mut Particles,
        n: usize,
        new_particle: &mut Particles,
        sort: bool,
        rng: &mut Ziggurat,
    ) -> usize {
        let mut best_w = 0f64;
        let mut best_i = 0usize;
        if sort {
            self.heapify(m, particle);
            #[cfg(feature = "debug-heapify")]
            {
                self.check_tweights(m, particle);
                for i in (0..(m - 1)).rev() {
                    assert!(particle.data[i].weight <= particle.data[(i - 1) / 2].weight);
                }
            }
        } else {
            self.init_tweights(m, particle);
        }
        #[cfg(feature = "debug-logm")]
        {
            assert!(self.tweight[0] * (1.0 - DW) <= scale && scale <= self.tweight[0] * (1.0 + DW));
            self.total_depth = 0;
        }
        let invscale = 1.0 / self.tweight[0];
        for i in 0..n {
            new_particle.data[i] = *self.weighted_sample(self.tweight[0], m, particle, rng);
            new_particle.data[i].weight *= invscale;
            if new_particle.data[i].weight > best_w {
                best_w = new_particle.data[i].weight;
                best_i = i;
            }
        }
        #[cfg(feature = "debug-logm-search")]
        println!("{}", self.total_depth / m);
        best_i
    }
}
