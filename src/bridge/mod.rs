use embedded_hal::blocking::i2c::{Read, Write};

mod i2c_error;
pub use i2c_error::{HwError, I2CBridgeError};

mod i2c_command;
pub use i2c_command::MyI2COperation;

pub trait Builder<'a>: Sized + Sync {
    /// Create a new scan operation
    fn new_scan_op(data_buff: &'a mut [u8; 4], scan_addr: u8) -> Self;
    /// Create a new read operation
    fn new_read_op(data_buff: &'a mut [u8], dev_addr: u8, size: u8) -> Self;
    /// Create a new write operation
    fn new_write_op(data_buff: &'a mut [u8], dev_addr: u8, to_write: &'a [u8]) -> Self;
}

pub trait Execute<'a> {
    fn execute<I2C>(self, i2c: &mut I2C) -> Result<&'a [u8], I2CBridgeError>
    where
        I2C: Read + Write + crate::hw::Reconfigure + crate::hw::Reset,
        I2CBridgeError: From<<I2C as embedded_hal::blocking::i2c::Read>::Error>
            + From<<I2C as embedded_hal::blocking::i2c::Write>::Error>;
}
