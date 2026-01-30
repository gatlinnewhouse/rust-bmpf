use bmpf_rs::uniform;
use clap::Parser;
use ziggurat_rs::Ziggurat;

static NBINS: usize = 500;
static N: i32 = 50;

#[derive(Parser, Debug)]
#[command(version, about, long_about = None)]
struct Args {
    /// Number of iterations to run
    #[arg(short, long)]
    iterations: usize,
}

fn minrand(n: usize) -> f64 {
    let mut x = uniform();
    for _i in 0..n {
        let y = uniform();
        if y < x {
            x = y;
        }
    }
    x
}

fn main() {
    let mut real = [0i32; NBINS];
    let mut sim = [0i32; NBINS];
    let mut simp = [0i32; NBINS];

    let mut z = Ziggurat::default();

    let args = Args::parse();
    let n = args.iterations;
    for _i in 0..n {
        let mut x0 = minrand(N as usize);
        real[(NBINS as f64 * x0).floor() as usize] += 1;
        x0 = 1.0 - (uniform().powf(1.0 / (N as f64 + 1.0f64)));
        sim[(NBINS as f64 * x0).floor() as usize] += 1;
        x0 = z.polynomial(N);
        simp[(NBINS as f64 * x0).floor() as usize] += 1;
    }
    // println!("{} {}", rand_calls, rand_steps);
    for i in 0..NBINS {
        println!("{} {} {} {}", i / NBINS, real[i], sim[i], simp[i]);
        if real[i] == 0 && sim[i] == 0 && simp[i] == 0 {
            break;
        }
    }
}
