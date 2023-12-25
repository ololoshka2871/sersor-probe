use embedded_hal::blocking::i2c::{Read, Write};

use crate::bridge::I2CBridgeError;

use super::traits::{I2CAddress, I2CDevice};

pub struct DeviceDba0;

const DBA0_ID: u16 = 0xDBA0;

impl<I2C> I2CDevice<I2C> for DeviceDba0
where
    I2C: Read + Write + crate::hw::Reconfigure + crate::hw::Reset,
    I2CBridgeError: From<<I2C as embedded_hal::blocking::i2c::Read>::Error>
        + From<<I2C as embedded_hal::blocking::i2c::Write>::Error>,
{
    type Error = I2CBridgeError;

    fn name(&self) -> &'static str {
        "DBA0"
    }

    fn data_size(&self) -> usize {
        4 * core::mem::size_of::<f32>()
    }

    fn probe(&self, addr: I2CAddress, i2c: &mut I2C) -> Result<(), Self::Error> {
        let id = super::probe_common::probe_i2c_0x80::<I2C, Self::Error>(addr, i2c)?;
        if id == DBA0_ID {
            Ok(())
        } else {
            Err(I2CBridgeError::NotSupported)
        }
    }

    fn read(
        &self,
        addr: I2CAddress,
        dest: &mut dyn super::traits::ValuesStorage,
        i2c: &mut I2C,
    ) -> Result<(), Self::Error>
    where
        I2C: Read + Write + crate::hw::Reconfigure + crate::hw::Reset,
        I2CBridgeError: From<<I2C as embedded_hal::blocking::i2c::Read>::Error>
            + From<<I2C as embedded_hal::blocking::i2c::Write>::Error>,
    {
        super::read_comon::read_i2c_leacy_pic::<I2C, Self::Error>(addr, dest, i2c)
    }
}
