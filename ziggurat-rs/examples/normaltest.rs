#![feature(float_erf)]
use gpoint::GPoint;
use std::f64::consts::FRAC_1_SQRT_2;
use ziggurat_rs::Ziggurat;

const NV: usize = 10000000;
const NB: usize = 100;
const VAR: f64 = 1.0;

fn main() {
    let mut rng = Ziggurat::default();

    // Heap-allocated buffers
    let mut variate = vec![0.0f64; NV];
    let mut bin = vec![0usize; NB];

    let mut maxv = 0.0f64;
    let mut minv = 0.0f64;

    // Generate samples
    for v in &mut variate {
        *v = rng.gaussian(VAR);
    }

    // Find min/max
    for &v in &variate {
        if v < minv {
            minv = v;
        }
        if v > maxv {
            maxv = v;
        }
    }

    let binwidth = (maxv - minv) / NB as f64;

    // Normalization constant
    let a = NV as f64 * (binwidth * 0.5 * FRAC_1_SQRT_2 / VAR).erf();

    let b = -0.5 / (VAR * VAR);

    // Histogram
    for &v in &variate {
        let mut j = ((v - minv) / binwidth).floor() as usize;
        if j >= NB {
            j = NB - 1; // guard against max edge
        }
        bin[j] += 1;
    }

    // Output
    (0..NB).for_each(|i| {
        let x = binwidth * (i as f64 + 0.5) + minv;
        let expected = a * (x * x * b).exp();
        println!("{} {} {}", GPoint(x), bin[i], GPoint(expected));
    });
}
