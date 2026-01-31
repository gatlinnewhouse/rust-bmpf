use std::{
    fs::File,
    io::{self, BufRead},
    path::Path,
};

use bmpf_rs::types::BpfState;

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
    let mut first_line = true;
    if let Ok(lines) = read_lines("./vehicle.dat") {
        for line in lines.map_while(Result::ok) {
            if first_line {
                t_ms = state.parse_line(line);
                t = t_ms as f64 * (1.0 / 1000.0);
                first_line = false;
                continue;
            }
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
