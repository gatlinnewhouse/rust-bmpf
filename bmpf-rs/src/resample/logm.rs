use crate::{aligned_vec::AVec, resample::Resample, types::Particles};
use std::process::abort;
use ziggurat_rs::Ziggurat;

#[cfg(feature = "debug-heapify")]
static DW: f64 = 1.0e-9;

pub struct Logm {
    tweight: AVec<f64>,
    #[cfg(feature = "debug-logm")]
    total_depth: usize,
}

impl Logm {
    pub fn new(mmax: usize) -> Self {
        Self {
            tweight: AVec::new(mmax),
            #[cfg(feature = "debug-logm")]
            total_depth: 0,
        }
    }

    /// Returns the index of a weighted-sampled particle using heap traversal
    fn weighted_sample_index(
        &mut self,
        scale: f64,
        m: usize,
        particles: &Particles,
        rng: &mut Ziggurat,
    ) -> usize {
        let mut w = rng.uniform() * scale;
        #[cfg(feature = "debug-logm")]
        let mut j = 0usize;

        let mut i = 0;
        while i < m {
            let left = 2 * i + 1;
            let right = 2 * i + 2;
            let lweight = if left < m { self.tweight[left] } else { 0f64 };

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

            if w <= lweight + particles.weight[i] {
                return i;
            }

            w -= lweight + particles.weight[i];
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
                w, j, particles.weight[j]
            );
        }
        abort();
    }

    fn heapify(&mut self, m: usize, particles: &mut Particles) {
        for i in (0..(m.saturating_sub(1))).rev() {
            let left = 2 * i + 1;
            let right = 2 * i + 2;

            self.tweight[i] = particles.weight[i];

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
                let wj = particles.weight[j];
                let wleft = particles.weight[left];
                let mut nextj = left;

                if right < m {
                    let wright = particles.weight[right];
                    if wj >= wleft && wj >= wright {
                        break;
                    }
                    if wj < wright && (wj >= wleft || wright > wleft) {
                        nextj = right;
                    }
                } else if wj >= wleft {
                    break;
                }

                particles.swap(j, nextj);
                let dw = particles.weight[j] - particles.weight[nextj];
                #[cfg(feature = "debug-logm")]
                assert!(dw >= 0.0f64);
                self.tweight[nextj] -= dw;
                j = nextj;
            }
        }
    }

    fn init_tweights(&mut self, m: usize, particles: &Particles) {
        if m <= 1 {
            if m == 1 {
                self.tweight[0] = particles.weight[0];
            }
            return;
        }

        let mut j = 0usize;
        for i in ((m / 2)..(m.saturating_sub(1))).rev() {
            self.tweight[i] = particles.weight[i];
            j = i.saturating_sub(1);
        }

        self.tweight[j] = particles.weight[j];
        let mut left = 2 * j + 1;
        let mut right = 2 * j + 2;

        if right >= m {
            self.tweight[j] = particles.weight[j] + particles.weight[left];
            j = j.saturating_sub(1);
        }

        while j > 0 {
            left = 2 * j + 1;
            right = 2 * j + 2;
            self.tweight[j] = particles.weight[j] + self.tweight[left] + self.tweight[right];
            j -= 1;
        }

        // Handle root
        if m > 0 {
            left = 1;
            right = 2;
            self.tweight[0] = particles.weight[0];
            if left < m {
                self.tweight[0] += self.tweight[left];
            }
            if right < m {
                self.tweight[0] += self.tweight[right];
            }
        }
    }

    #[cfg(feature = "debug-heapify")]
    fn check_tweights(&self, m: usize, particles: &Particles) {
        for i in (0..(m.saturating_sub(1))).rev() {
            let left = 2 * i + 1;
            let right = 2 * i + 2;
            let mut w = particles.weight[i];
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
                for i in (1..m).rev() {
                    assert!(particle.weight[i] <= particle.weight[(i - 1) / 2]);
                }
            }
        } else {
            self.init_tweights(m, particle);
        }

        #[cfg(feature = "debug-logm")]
        {
            assert!(
                self.tweight[0] * (1.0 - DW) <= _scale && _scale <= self.tweight[0] * (1.0 + DW)
            );
            self.total_depth = 0;
        }

        let invscale = 1.0 / self.tweight[0];

        for i in 0..n {
            let src = self.weighted_sample_index(self.tweight[0], m, particle, rng);
            new_particle.copy_from(i, particle, src);
            new_particle.weight[i] *= invscale;
            if new_particle.weight[i] > best_w {
                best_w = new_particle.weight[i];
                best_i = i;
            }
        }

        #[cfg(feature = "debug-logm-search")]
        println!("{}", self.total_depth / m);

        best_i
    }
}
