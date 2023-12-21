pub trait Reconfigure {
    fn set_speed(&mut self, speed: u32) -> bool;
}

pub trait Reset {
    fn reset(&mut self);
}
