use std::cell::RefCell;
use ziggurat_rs::Ziggurat;

pub mod resample;
pub mod sim;
pub mod types;

thread_local! {
    static ZIGGURAT: RefCell<Ziggurat> = RefCell::new(Ziggurat::default());
}

pub fn uniform() -> f64 {
    ZIGGURAT.with(|z| z.borrow_mut().uniform())
}

pub fn gaussian(sigma: f64) -> f64 {
    ZIGGURAT.with(|z| z.borrow_mut().gaussian(sigma))
}

pub fn polynomial(n: i32) -> f64 {
    ZIGGURAT.with(|z| z.borrow_mut().polynomial(n))
}

pub fn rand32() -> u32 {
    ZIGGURAT.with(|z| z.borrow_mut().rand32())
}
