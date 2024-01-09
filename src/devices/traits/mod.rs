mod i2c_device;
use alloc::{string::String, vec::Vec};
pub use i2c_device::{I2CAddress, I2CDevice};

mod modbus_device;

pub trait ValuesStorage: Send {
    fn size(&self) -> usize;
    fn copy_from(&mut self, src: &[u8]);
    fn sender_id(&self) -> String;
    fn print(&self) -> String;
    fn render(&self, field_width: u32) -> Vec<String>;       
}
