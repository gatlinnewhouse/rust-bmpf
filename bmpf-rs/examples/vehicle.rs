use bmpf_rs::{CCoord, State};

fn run() {
    let mut t = 0.0f64;
    let mut dt = 0.01f64;
    while t <= 10.0f64 {
        let mut msec = (t * 1000f64 + 0.5f64).floor();
        // update state
        // ccoord gps
        // acoord imu
        // println!(
        //     "{} {} {} {} {} {} {}",
        //     msecs, vehicle.posn.x, vehicle.posn.y, gps.x, gps.y, imu.r, imu.t
        // );
        t += dt;
    }
}

fn main() {}
