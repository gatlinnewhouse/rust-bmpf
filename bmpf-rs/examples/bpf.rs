use bmpf_rs::types::BpfState;
use clap::Parser;
use std::{
    fs::File,
    io::{self, BufRead},
    path::Path,
};

#[derive(Parser, Debug)]
struct Args {
    /// Number of particles
    #[arg(short, long, default_value_t = 100)]
    nparticles: usize,

    /// Sampler name
    #[arg(short, long)]
    sampler: String,

    /// Sort?
    #[arg(short, long, default_value_t = false)]
    sort: bool,

    /// Report particles?
    #[arg(short, long, default_value_t = false)]
    report_particles: bool,

    /// Best particle recording?
    #[arg(short, long, default_value_t = false)]
    best_particle: bool,

    /// Resampling interval
    #[arg(short, long, default_value_t = 1)]
    resample_interval: usize,

    /// Fast direction
    #[arg(short, long, default_value_t = 1)]
    fast_direction: i32,

    #[arg(short, long)]
    avar: f64,

    #[arg(short, long)]
    rvar: f64,

    #[arg(short, long)]
    gps_var: f64,

    #[arg(short, long)]
    imu_r_var: f64,

    #[arg(short, long)]
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
    let mut state = BpfState::default();
    state.init_particles();
    let mut t_ms = 0;
    let mut t_last = 0;
    let mut t = 0.0;
    if let Ok(mut lines) = read_lines("./vehicle.dat") {
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
            print!("{} {}", state.vehicle.x, state.vehicle.y);
            state.bpf_step(t, dt, report);
            if report {
                t_last = t_ms;
            }
            print!("\n");
        }
    }
}
