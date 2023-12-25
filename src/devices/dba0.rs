use embedded_hal::blocking::i2c::{Read, Write};

use crate::{bridge::I2CBridgeError, config::HlString};

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

    fn read(&self, addr: I2CAddress, dest: &mut [u8], i2c: &mut I2C) -> Result<(), Self::Error>
    where
        I2C: Read + Write + crate::hw::Reconfigure + crate::hw::Reset,
        I2CBridgeError: From<<I2C as embedded_hal::blocking::i2c::Read>::Error>
            + From<<I2C as embedded_hal::blocking::i2c::Write>::Error>,
    {
        assert!(dest.len() == <Self as I2CDevice<I2C>>::data_size(self));
        super::read_comon::read_i2c_leacy_pic::<I2C, Self::Error>(addr, dest, i2c)
    }

    fn render(&self, _data: &[u8]) {
        defmt::trace!("ValuesStorage::render()");
    }

    fn print(&self, data: &[u8]) -> HlString {
        use core::fmt::Write;
        use tbytes::TBytesReaderFor;

        const NAMES: [&str; 4] = ["P", "T", "FP", "FT"];

        let mut s = HlString::new();
        write!(s, "{}::print {{ ", <Self as I2CDevice<I2C>>::name(self)).ok();
        let data = tbytes::TBytesReader::from(data);
        for n in NAMES.iter() {
            write!(
                s,
                "{}={} ",
                n,
                crate::support::format_float_simple(data.read().unwrap(), 2)
            )
            .ok();
        }
        write!(s, "}}").ok();
        s
    }
}
