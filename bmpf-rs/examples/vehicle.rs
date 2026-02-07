use bmpf_rs::{sim::DEFAULT_GPS_VAR, types::VehicleState};
use gpoint::GPoint;
use ziggurat_rs::Ziggurat;

#[derive(Default)]
struct Simulation {
    vehicle: VehicleState,
    rng: Ziggurat,
}

fn run() {
    let mut t = 0.0f64;
    let dt = 0.01f64;

    let mut s = Simulation::default();
    s.vehicle.init_state(&mut s.rng);

    while t <= 10.0f64 {
        let msec = (t * 1000f64 + 0.5f64).floor();
        s.vehicle.update_state(dt, 0, &mut s.rng);
        let gps = s.vehicle.gps_measure(&mut s.rng, DEFAULT_GPS_VAR);
        let imu = s.vehicle.imu_measure(dt, &mut s.rng);
        println!(
            "{} {} {} {} {} {} {}",
            msec,
            GPoint(s.vehicle.posn.x),
            GPoint(s.vehicle.posn.y),
            GPoint(gps.x),
            GPoint(gps.y),
            GPoint(imu.r),
            GPoint(imu.t)
        );
        t += dt;
    }
}

fn main() {
    run();
}
