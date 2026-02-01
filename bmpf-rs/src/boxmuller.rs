use crate::uniform;

static mut X1: f64 = 0.0;
static mut X2: f64 = 0.0;
static mut W: f64 = 0.0;
static mut Y1: f64 = 0.0;
static mut Y2: f64 = 0.0f64;
static mut HAVE_Y2: i64 = 0;

pub unsafe fn gaussian(sd: f64) -> f64 {
    unsafe {
        if HAVE_Y2 == 1 {
            HAVE_Y2 = 0;
            return Y2 * sd;
        }
        loop {
            X1 = 2.0 * uniform() as f64 - 1.0;
            X2 = 2.0 * uniform() as f64 - 1.0;
            W = X1 * X1 + X2 * X2;
            if W < 1.0 {
                break;
            }
        }
        W = ((-2.0 * W.ln()) / W).sqrt();
        Y1 = X1 * W;
        Y2 = X2 * W;
        HAVE_Y2 = 1;
        Y1 * sd
    }
}

#[test]
fn test_gaussian_not_nan() {
    unsafe {
        for _ in 0..100 {
            let val = gaussian(1.0);
            assert!(!val.is_nan(), "Gaussian returned NaN!");
            assert!(val.is_finite(), "Gaussian returned Inf!");
        }
    }
}
