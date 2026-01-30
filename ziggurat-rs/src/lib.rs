//! # Ziggurat: Fast Normal, Exponential, and Polynomial Distributed PRNGs
//!
//! This is a Rust port of Bart Massey's Ziggurat implementation, which was derived
//! from David Bateman's BSD-licensed reimplementation of Marsaglia and Tsang's
//! Ziggurat Method for Gaussian and exponential pseudo-random number generation.
//!
//! ## Original Copyright Notice
//!
//! Copyright (c) 2004, David Bateman
//! Copyright (c) 2007, Bart Massey
//!
//! Licensed under the 3-clause BSD License.
//!
//! The ISAAC PRNG implementation was placed in the Public Domain by Bob Jenkins.
//!
//! ## Performance
//!
//! The Ziggurat method provides vast performance improvements over the Box-Müller
//! method for generating normal variates. The basic cost is essentially two table
//! lookups, a floating-point multiply, a floating-point compare, and some amortized
//! operations.

mod constants;
mod isaac;
mod tables;

use constants::*;
use isaac::IsaacRng;

use crate::tables::{
    exponential::{EXPONENTIAL_F, EXPONENTIAL_K, EXPONENTIAL_W},
    normal::{NORMAL_F, NORMAL_K, NORMAL_W},
};

/// Main Ziggurat random number generator
pub struct Ziggurat {
    rng: IsaacRng,
    last: u32,
}

impl Ziggurat {
    /// Create a new Ziggurat generator with the given seed
    pub fn new(seed: u32) -> Self {
        let mut rng = IsaacRng::new();
        rng.seed(seed);
        Self {
            rng,
            last: 0x63636363,
        }
    }

    /// Get a random 32-bit unsigned integer
    #[inline]
    pub fn rand32(&mut self) -> u32 {
        self.rng.next_u32()
    }

    /// Generate a uniform random number in [0, 1)
    #[inline]
    pub fn uniform(&mut self) -> f64 {
        const SCALE: f64 = 5.42101086242752e-20;
        (4294967296.0 * self.rand32() as f64 + self.rand32() as f64) * SCALE
    }

    /// Generate a standard normal (Gaussian) random variable (mean=0, stddev=1)
    #[inline]
    pub fn normal(&mut self) -> f64 {
        // 32-bit mantissa
        let r = self.rand32();
        let rabs = r & 0x7fffffff;
        let idx = ((r ^ self.last) & 0xFF) as usize;
        self.last = r;

        // 99.3% of the time we return here on first try
        if rabs < NORMAL_K[idx] {
            return (r as i32) as f64 * NORMAL_W[idx];
        }

        self.rand_normal(r, idx)
    }

    /// Generate a Gaussian random variable with given standard deviation
    #[inline]
    pub fn gaussian(&mut self, sigma: f64) -> f64 {
        self.normal() * sigma
    }

    /// Generate an exponential random variable
    #[inline]
    pub fn exponential(&mut self) -> f64 {
        let r = self.rand32();
        let idx = ((r ^ self.last) & 0xFF) as usize;
        self.last = r;

        // 98.9% of the time we return here on first try
        if r < EXPONENTIAL_K[idx] {
            return r as f64 * EXPONENTIAL_W[idx];
        }

        self.rand_exponential(r, idx)
    }

    /// Generate a variate with distribution (1 - x)^n
    #[inline]
    pub fn polynomial(&mut self, n: i32) -> f64 {
        1.0 - self.uniform().powf(1.0 / (n as f64 + 1.0))
    }

    /// Slow path for normal distribution (tail and rejection sampling)
    fn rand_normal(&mut self, mut r: u32, mut idx: usize) -> f64 {
        loop {
            let rabs = r & 0x7fffffff;
            let x = (r as i32) as f64 * NORMAL_W[idx];

            if rabs < NORMAL_K[idx] {
                return x;
            }

            if idx == 0 {
                // Handle the tail using Marsaglia's method
                // Generate x = -ln(U_1)/r, y = -ln(U_2), until y+y > x*x
                let mut xx: f64;
                let mut yy: f64;
                loop {
                    xx = -ZIGGURAT_NOR_INV_R * self.uniform().ln();
                    yy = -self.uniform().ln();
                    if yy + yy > xx * xx {
                        break;
                    }
                }
                return if rabs & 0x100 != 0 {
                    -ZIGGURAT_NOR_R - xx
                } else {
                    ZIGGURAT_NOR_R + xx
                };
            } else if (NORMAL_F[idx - 1] - NORMAL_F[idx]) * self.uniform() + NORMAL_F[idx]
                < (-0.5 * x * x).exp()
            {
                return x;
            }

            r = self.rand32();
            idx = ((r ^ self.last) & 0xFF) as usize;
            self.last = r;
        }
    }

    /// Slow path for exponential distribution (tail and rejection sampling)
    fn rand_exponential(&mut self, mut r: u32, mut idx: usize) -> f64 {
        loop {
            let x = r as f64 * EXPONENTIAL_W[idx];

            if r < EXPONENTIAL_K[idx] {
                return x;
            } else if idx == 0 {
                // Handle the tail
                return ZIGGURAT_EXP_R - self.uniform().ln();
            } else if (EXPONENTIAL_F[idx - 1] - EXPONENTIAL_F[idx]) * self.uniform()
                + EXPONENTIAL_F[idx]
                < (-x).exp()
            {
                return x;
            }

            r = self.rand32();
            idx = ((r ^ self.last) & 0xFF) as usize;
            self.last = r;
        }
    }
}

impl Default for Ziggurat {
    fn default() -> Self {
        Self::new(17)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_uniform() {
        let mut rng = Ziggurat::new(42);
        for _ in 0..1000 {
            let u = rng.uniform();
            assert!((0.0..1.0).contains(&u));
        }
    }

    #[test]
    fn test_normal() {
        let mut rng = Ziggurat::new(42);
        let mut sum = 0.0;
        let mut sum_sq = 0.0;
        let n = 10000;

        for _ in 0..n {
            let x = rng.normal();
            sum += x;
            sum_sq += x * x;
        }

        let mean = sum / n as f64;
        let variance = sum_sq / n as f64 - mean * mean;

        // Check that mean is close to 0 and variance is close to 1
        assert!(
            (mean.abs()) < 0.1,
            "Mean should be close to 0, got {}",
            mean
        );
        assert!(
            (variance - 1.0).abs() < 0.1,
            "Variance should be close to 1, got {}",
            variance
        );
    }

    #[test]
    fn test_exponential() {
        let mut rng = Ziggurat::new(42);
        let mut sum = 0.0;
        let n = 10000;

        for _ in 0..n {
            let x = rng.exponential();
            assert!(x >= 0.0);
            sum += x;
        }

        let mean = sum / n as f64;
        // Exponential distribution has mean = 1
        assert!(
            (mean - 1.0).abs() < 0.1,
            "Mean should be close to 1, got {}",
            mean
        );
    }

    #[test]
    fn test_gaussian() {
        let mut rng = Ziggurat::new(42);
        let sigma = 2.5;
        let mut sum = 0.0;
        let mut sum_sq = 0.0;
        let n = 10000;

        for _ in 0..n {
            let x = rng.gaussian(sigma);
            sum += x;
            sum_sq += x * x;
        }

        let mean = sum / n as f64;
        let variance = sum_sq / n as f64 - mean * mean;
        let stddev = variance.sqrt();

        assert!(
            (mean.abs()) < 0.2,
            "Mean should be close to 0, got {}",
            mean
        );
        assert!(
            (stddev - sigma).abs() < 0.2,
            "Stddev should be close to {}, got {}",
            sigma,
            stddev
        );
    }

    #[test]
    fn test_polynomial() {
        let mut rng = Ziggurat::new(42);
        for _ in 0..1000 {
            let x = rng.polynomial(5);
            assert!((0.0..=1.0).contains(&x));
        }
    }
}
