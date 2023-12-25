use embedded_hal::blocking::i2c::{Read, Write};

pub type I2CAddress = u8;

pub trait I2CDevice<I2C>: Sync
where
    I2C: Read + Write + crate::hw::Reconfigure + crate::hw::Reset,
{
    type Error;

    /// Returns the name of the device.
    fn name(&self) -> &'static str;

    /// Returns the size of the data blob that can be read from the device.
    fn data_size(&self) -> usize;

    /// Probe the device at the given address.
    fn probe(&self, addr: I2CAddress, i2c: &mut I2C) -> Result<(), Self::Error>;

    /// Read the device at the given address. The data is stored in the provided storage.
    fn read(
        &self,
        addr: I2CAddress,
        dest: &mut dyn super::ValuesStorage,
        i2c: &mut I2C,
    ) -> Result<(), Self::Error>;
}
