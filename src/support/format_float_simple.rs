use core::{fmt::Write, str::FromStr};

use alloc::string::String;
use num::traits::float::FloatCore;

pub fn format_float_simple(mut v: f32, percision: i32) -> String {
    if v.is_nan() {
        String::from_str("NaN").unwrap()
    } else {
        let mut res = String::new();
        if v < 0.0 {
            v = -v;
            res.push('-');
        }
        let a = v.floor();
        write!(
            &mut res,
            "{}.{}",
            a as i32,
            ((v - a) * 10.0.powi(percision)).round() as i32
        )
        .unwrap();
        res
    }
}
