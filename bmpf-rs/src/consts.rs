use crate::types::CosDirn;
use std::{f64::consts::PI, sync::LazyLock};

pub static COS_DIRN: LazyLock<CosDirn> = LazyLock::new(|| {
    let mut cd = CosDirn::default();
    cd.init_dirn();
    cd
});

pub const BOX_DIM: f64 = 20.0;
pub const MAX_SPEED: f64 = 2.0;

pub const AVAR: f64 = PI / 32.0;
pub const RVAR: f64 = 0.1;
pub const DEFAULT_GPS_VAR: f64 = 1.0;
pub const IMU_R_VAR: f64 = 0.5;
pub const IMU_A_VAR: f64 = PI / 8.0;
pub const NDIRNS: i32 = 1024;

pub const FAST_DIRECTION: i32 = 0;

pub const TWO_PI: f64 = 2.0 * PI;
pub const PI_OVER_TWO: f64 = PI / 2.0;
pub const NEG_BOX_DIM: f64 = -BOX_DIM;
