use core::f64::{self};
use gpoint::GPoint;
use ziggurat_rs::Ziggurat;

static NV: usize = 10000000;
static NB: usize = 200;
static PN: i32 = 51;

fn main() {
    let mut variate = vec![0.0f64; NV];
    let mut bin = vec![0usize; NB];

    let mut z = Ziggurat::default();

    // generate samples
    for v in &mut variate {
        *v = z.polynomial(PN);
    }

    // find max
    let maxv = variate.iter().cloned().fold(f64::NEG_INFINITY, f64::max);

    let binwidth = maxv / NB as f64;

    // Histogram
    for &v in &variate {
        let mut j = (v / binwidth).floor() as usize;
        if j >= NB {
            j = NB - 1;
        }
        bin[j] += 1;
    }

    // Output
    (0..NB).for_each(|i| {
        let x = binwidth * (i as f64 + 0.5);
        let x0 = binwidth * i as f64;
        let x1 = x0 + binwidth;
        let a = (1.0f64 - x0).powi(PN + 1) - (1.0f64 - x1).powi(PN + 1);
        println!("{} {} {}", GPoint(x), bin[i], GPoint(NV as f64 * a));
    });
}
