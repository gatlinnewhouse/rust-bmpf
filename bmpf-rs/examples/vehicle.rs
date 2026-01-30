#[cfg(feature = "boxmuller")]
use bmpf_rs::boxmuller;
use bmpf_rs::{
    boxmuller::gaussian,
    sim::GPS_VAR,
    types::{CCoord, State},
};

fn gps_measure(vehicle: &State) -> CCoord {
    let mut result = vehicle.posn.clone();
    #[cfg(feature = "boxmuller")]
    {
        result.x += unsafe { gaussian(GPS_VAR) };
        result.y += unsafe { gaussian(GPS_VAR) };
    }
    result
}

fn run() {
    let mut t = 0.0f64;
    let mut dt = 0.01f64;

    let mut vehicle = State::default();
    vehicle.init_state();

    while t <= 10.0f64 {
        let mut msec = (t * 1000f64 + 0.5f64).floor();
        vehicle.update_state(dt, 0);
        let gps = gps_measure(&vehicle);
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
