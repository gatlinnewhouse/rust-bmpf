use crate::types::Particles;
use ziggurat_rs::Ziggurat;

/// Naive resampler
mod logm;
/// Naive resampler
mod naive;
/// Optimal resampler
mod optimal;
/// Regular resampler
mod regular;

pub trait Resample {
    fn resample(
        &mut self,
        scale: f64,
        m: usize,
        particle: &mut Particles,
        n: usize,
        new_particle: &mut Particles,
        sort: bool,
        rng: &mut Ziggurat,
    ) -> usize;
}

pub enum Resampler {
    Logm(logm::Logm),
    Naive(naive::Naive),
    Optimal(optimal::Optimal),
    Regular(regular::Regular),
}

impl Resampler {
    pub fn new(name: &str, mmax: usize) -> Self {
        match name {
            "logm" => Self::Logm(logm::Logm::new(mmax)),
            "naive" => Self::Naive(naive::Naive::default()),
            "optimal" => Self::Optimal(optimal::Optimal::default()),
            "regular" => Self::Regular(regular::Regular::default()),
            _ => panic!("Unknown resampler: {}", name),
        }
    }
}

impl Resample for Resampler {
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
        match self {
            Resampler::Logm(r) => r.resample(scale, m, particle, n, new_particle, sort, rng),
            Resampler::Naive(r) => r.resample(scale, m, particle, n, new_particle, sort, rng),
            Resampler::Optimal(r) => r.resample(scale, m, particle, n, new_particle, sort, rng),
            Resampler::Regular(r) => r.resample(scale, m, particle, n, new_particle, sort, rng),
        }
    }
}
