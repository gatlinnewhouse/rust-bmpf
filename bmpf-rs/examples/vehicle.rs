use bmpf_rs::types::VehicleState;
use gpoint::GPoint;

fn run() {
    let mut t = 0.0f64;
    let dt = 0.01f64;

    let mut vehicle = VehicleState::default();
    vehicle.init_state();

    while t <= 10.0f64 {
        let msec = (t * 1000f64 + 0.5f64).floor();
        vehicle.update_state(dt, 0);
        let gps = vehicle.gps_measure();
        let imu = vehicle.imu_measure(dt);
        println!(
            "{} {} {} {} {} {} {}",
            msec,
            GPoint(vehicle.posn.x),
            GPoint(vehicle.posn.y),
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
