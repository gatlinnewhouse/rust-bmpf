use bmpf_rs::types::State;

fn run() {
    let mut t = 0.0f64;
    let mut dt = 0.01f64;

    let mut vehicle = State::default();
    vehicle.init_state();

    while t <= 10.0f64 {
        let msec = (t * 1000f64 + 0.5f64).floor();
        vehicle.update_state(dt, 0);
        let gps = vehicle.gps_measure();
        let imu = vehicle.imu_measure(dt);
        println!(
            "{} {} {} {} {} {} {}",
            msec, vehicle.posn.x, vehicle.posn.y, gps.x, gps.y, imu.r, imu.t
        );
        t += dt;
    }
}

fn main() {
    run();
}
