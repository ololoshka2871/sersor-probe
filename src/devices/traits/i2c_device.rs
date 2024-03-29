use embedded_hal::blocking::i2c::{Read, Write};

use super::ValuesStorage;

pub type I2CAddress = u8;

pub trait I2CDevice<I2C>: super::Device
where
    I2C: Read + Write + crate::hw::Reconfigure + crate::hw::Reset,
{
    type Error;

    /// Probe the device at the given address.
    fn probe_i2c(&self, addr: I2CAddress, i2c: &mut I2C) -> Result<(), Self::Error>;

    /// Read the device at the given address. The data is stored in the provided storage.
    fn read_i2c(&self, addr: I2CAddress, dest: &mut dyn ValuesStorage, i2c: &mut I2C) -> Result<(), Self::Error>;
}
