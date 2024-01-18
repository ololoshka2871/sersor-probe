use alloc::boxed::Box;

use embedded_hal::blocking::i2c::{Read, Write};
use modbus_core::Response;

use crate::bridge::I2CBridgeError;

use super::{
    storages::PTFpFtstorage,
    traits::{I2CAddress, I2CDevice},
    ModbusDevice, ValuesStorage,
};

pub struct DeviceDba0;

const DBA0_ID: u16 = 0xDBA0;

impl super::Device for DeviceDba0 {
    fn name(&self) -> &'static str {
        "DBA0"
    }

    fn data_size(&self) -> usize {
        4 * core::mem::size_of::<f32>()
    }

    fn make_storage(&self) -> Box<dyn super::ValuesStorage> {
        Box::new(PTFpFtstorage::new(self.name()))
    }
}

impl<I2C> I2CDevice<I2C> for DeviceDba0
where
    I2C: Read + Write + crate::hw::Reconfigure + crate::hw::Reset,
    I2CBridgeError: From<<I2C as embedded_hal::blocking::i2c::Read>::Error>
        + From<<I2C as embedded_hal::blocking::i2c::Write>::Error>,
{
    type Error = I2CBridgeError;

    fn probe_i2c(&self, addr: I2CAddress, i2c: &mut I2C) -> Result<(), Self::Error> {
        let id = super::probe_common::probe_i2c_0x80::<I2C, Self::Error>(addr, i2c)?;
        if id == DBA0_ID {
            Ok(())
        } else {
            Err(I2CBridgeError::NotSupported)
        }
    }

    fn read_i2c(
        &self,
        addr: I2CAddress,
        dest: &mut dyn ValuesStorage,
        i2c: &mut I2C,
    ) -> Result<(), Self::Error>
    where
        I2C: Read + Write + crate::hw::Reconfigure + crate::hw::Reset,
        I2CBridgeError: From<<I2C as embedded_hal::blocking::i2c::Read>::Error>
            + From<<I2C as embedded_hal::blocking::i2c::Write>::Error>,
    {
        assert!(dest.size() == <Self as super::Device>::data_size(self));
        super::read_comon::read_i2c_leacy_pic::<I2C, Self::Error>(addr, dest, i2c)
    }
}

impl ModbusDevice for DeviceDba0 {
    type Error = u32;

    fn probe_resp(&self, id_resp: modbus_core::rtu::ResponseAdu<'_>) -> Result<(), ()> {
        if let Ok(Response::ReadHoldingRegisters(data)) = id_resp.pdu.0 {
            if Some(DBA0_ID) == data.get(0) {
                return Ok(());
            }
        }
        Err(())
    }

    fn build_data_request(&self) {}

    fn decode_resp(&self) -> Result<(), Self::Error> {
        Err(0)
    }
}
