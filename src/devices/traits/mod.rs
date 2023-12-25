mod i2c_device;
pub use i2c_device::{I2CAddress, I2CDevice};

mod modbus_device;

pub trait ValuesStorage: core::fmt::Debug {
    fn render(&self);
}
