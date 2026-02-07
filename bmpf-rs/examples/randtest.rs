use clap::Parser;
use ziggurat_rs::Ziggurat;

static NBINS: f64 = 500f64;
static N: i32 = 50;

#[derive(Parser, Debug)]
#[command(version, about, long_about = None)]
struct Args {
    /// Number of iterations to run
    #[arg(short, long)]
    iterations: usize,
}

fn minrand(n: usize, rng: &mut Ziggurat) -> f64 {
    let mut x = rng.uniform();
    for _i in 0..n {
        let y = rng.uniform();
        if y < x {
            x = y;
        }
    }
    x
}

fn main() {
    let mut real = [0i32; NBINS as usize];
    let mut sim = [0i32; NBINS as usize];
    let mut simp = [0i32; NBINS as usize];

    let mut rng = Ziggurat::default();

    let args = Args::parse();
    let n = args.iterations;
    for _i in 0..n {
        let mut x0 = minrand(N as usize, &mut rng);
        real[(NBINS * x0).floor() as usize] += 1;
        x0 = 1.0f64 - ((rng.uniform()).powf(1.0 / (N as f64 + 1.0f64)));
        sim[(NBINS * x0).floor() as usize] += 1;
        x0 = rng.polynomial(N);
        simp[(NBINS * x0).floor() as usize] += 1;
    }
    // println!("{} {}", rand_calls, rand_steps);
    for i in 0..(NBINS as usize) {
        println!("{} {} {} {}", (i as f64 / NBINS), real[i], sim[i], simp[i]);
        if real[i] == 0 && sim[i] == 0 && simp[i] == 0 {
            break;
        }
    }
}
