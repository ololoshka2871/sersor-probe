mod i2c_device;
use alloc::string::String;
pub use i2c_device::{I2CAddress, I2CDevice};

mod modbus_device;

pub trait ValuesStorage: Send {
    fn size(&self) -> usize;
    fn copy_from(&mut self, src: &[u8]);
    fn render(&self);
    fn print(&self) -> String;
}
