/// Маппинг диопазонов
/// *-------х-------*
/// ^min    ^v      ^max
/// percent = (v - min) / (max - min)
///
/// *-------x-------*
/// ^left   ^res    ^right
/// res = left + (right - left) * percent
pub fn map(v: f32, min: f32, max: f32, left: u16, right: u16) -> u16 {
    let percent = (v - min) / (max - min);
    let _left = left as i32;
    let _right = right as i32;

    let mapped_len = ((_right - _left).abs() as f32 * percent) as u16;
    if _left < _right {
        left + mapped_len
    } else {
        right + mapped_len
    }
}
