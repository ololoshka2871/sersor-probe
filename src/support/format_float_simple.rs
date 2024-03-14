use core::{fmt::Write, str::FromStr};

use alloc::string::String;
use num::traits::float::FloatCore;

pub fn format_float_simple(mut v: f32, percision: u32) -> String {
    if v.is_nan() {
        String::from_str("NaN").unwrap()
    } else {
        let mut res = String::new();
        if v < 0.0 {
            v = -v;
            res.push('-');
        }
        
        let mut a = v.floor() as i32;
        let mut r = ((v - a as f32) * 10.0.powi(percision as i32)).round() as i32;
        if r == 10i32.pow(percision) {
            a += 1;
            r = 0;
        }

        write!(&mut res, "{}.{}", a, r).unwrap();
        res
    }
}
