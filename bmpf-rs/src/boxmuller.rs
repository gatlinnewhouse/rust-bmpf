use crate::uniform;

static mut Y2: f64 = 0.0f64;
static mut HAVE_Y2: bool = false;

pub unsafe fn gaussian(sd: f64) -> f64 {
    unsafe {
        if HAVE_Y2 {
            HAVE_Y2 = false;
            return Y2 * sd;
        }
        let mut x1;
        let mut x2;
        let mut w;
        loop {
            x1 = 2.0 * uniform() - 1.0;
            x2 = 2.0 * uniform() - 1.0;
            w = x1 * x1 + x2 * x2;
            if w < 1.0 {
                break;
            }
        }
        w = ((-2.0 * w.ln()) / w).sqrt();
        let y1 = x1 * w;
        Y2 = x2 * w;
        HAVE_Y2 = true;
        y1 * sd
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
