use core::fmt::Write;

use num::traits::float::FloatCore;

pub fn format_float_simple(v: f32, percision: i32) -> crate::config::HlString {
    let a = v.floor();
    let mut res = crate::config::HlString::new();
    write!(
        &mut res,
        "{}.{}",
        a as i32,
        ((v - a) * 10.0.powi(percision)).round() as i32
    )
    .unwrap();
    res
}
