use systick_monotonic::fugit::Hertz;

pub trait Reconfigure {
    fn set_speed(&mut self, speed: Hertz<u32>) -> bool;
}

pub trait Reset {
    fn reset(&mut self);
}
