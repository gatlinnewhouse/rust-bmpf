use bmpf_rs::types::BpfState;
use clap::Parser;
use gpoint::GPoint;
use std::{
    f64::consts::PI,
    fs::File,
    io::{self, BufRead},
    path::Path,
};

#[derive(Parser, Debug)]
#[command(version, about, long_about = None)]
struct Args {
    /// Number of particles
    #[arg(long, default_value_t = 100)]
    nparticles: usize,

    /// Sampler name
    #[arg(long)]
    sampler: String,

    /// File path
    #[arg(long)]
    file: String,

    /// Sort?
    #[arg(long, default_value_t = false)]
    sort: bool,

    /// Report particles?
    #[arg(long, default_value_t = 1000)]
    report_particles: i32,

    /// Best particle recording?
    #[arg(long, default_value_t = false)]
    best_particle: bool,

    /// Resampling interval
    #[arg(long, default_value_t = 1)]
    resample_interval: usize,

    /// Fast direction
    #[arg(long, default_value_t = 0)]
    fast_direction: i32,

    #[arg(long, default_value_t = PI / 32f64)]
    avar: f64,

    #[arg(long, default_value_t = 0.1f64)]
    rvar: f64,

    #[arg(long, default_value_t = 1.0f64)]
    gps_var: f64,

    #[arg(long, default_value_t = 0.5f64)]
    imu_r_var: f64,

    #[arg(long, default_value_t = PI / 8.0f64)]
    imu_a_var: f64,
}

fn read_lines<P>(filename: P) -> io::Result<io::Lines<io::BufReader<File>>>
where
    P: AsRef<Path>,
{
    let file = File::open(filename)?;
    Ok(io::BufReader::new(file).lines())
}

fn main() {
    let args = Args::parse();

    let mut state = BpfState::new(
        &args.sampler,
        args.sort,
        args.nparticles,
        args.report_particles,
        args.best_particle,
        args.resample_interval,
    );

    state.init_particles();
    let mut t_ms;
    let mut t_last = 0;
    let mut t = 0.0;
    if let Ok(mut lines) = read_lines(args.file) {
        // Read first line to initialize
        if let Some(Ok(first_line)) = lines.next() {
            t_ms = state.parse_line(first_line);
            t = t_ms as f64 * (1.0 / 1000.0);
        }

        for line in lines.map_while(Result::ok) {
            t_ms = state.parse_line(line);
            let t0 = t_ms as f64 * (1.0 / 1000.0);
            let dt = t0 - t;
            let mut report = false;
            if state.report_particles > 0 {
                report = t_ms - t_last >= state.report_particles;
            }
            t = t0;
            print!("{} {}", GPoint(state.vehicle.x), GPoint(state.vehicle.y));
            state.bpf_step(t, dt, report);
            if report {
                t_last = t_ms;
            }
            println!();
        }
    }
}
