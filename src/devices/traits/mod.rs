use alloc::{boxed::Box, string::String, vec::Vec};

mod i2c_device;
pub use i2c_device::{I2CAddress, I2CDevice};

mod modbus_device;
pub use modbus_device::{ModbusDevice, DecodeError};

pub trait ValuesStorage: Send {
    fn size(&self) -> usize;
    fn copy_from(&mut self, src: &[u8]);
    fn sender_id(&self) -> String;
    fn print(&self) -> String;
    fn render(&self, field_width: u32) -> Vec<String>;
    fn as_mut_slice(&mut self) -> &mut [u8];
    fn write_bytes(&mut self, buf: &[u8]);
    fn write_f32(&mut self, value: f32);
}

pub trait Device: Sync {
    /// Returns the name of the device.
    fn name(&self) -> &'static str;

    /// Returns the size of the data blob that can be read from the device.
    fn data_size(&self) -> usize;

    /// Build empty data storage
    fn make_storage(&self) -> Box<dyn super::ValuesStorage>;
}
