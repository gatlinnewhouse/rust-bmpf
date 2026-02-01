use crate::types::Particles;

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
            _ => panic!("No resampler specified"),
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
    ) -> usize {
        match self {
            Resampler::Logm(logm) => logm.resample(scale, m, particle, n, new_particle, sort),
            Resampler::Naive(naive) => naive.resample(scale, m, particle, n, new_particle, sort),
            Resampler::Optimal(optimal) => {
                optimal.resample(scale, m, particle, n, new_particle, sort)
            }
            Resampler::Regular(regular) => {
                regular.resample(scale, m, particle, n, new_particle, sort)
            }
        }
    }
}
