use alloc::boxed::Box;

use embedded_hal::blocking::i2c::{Read, Write};
use modbus_core::{rtu::ResponseAdu, RequestPdu, Response};

use crate::bridge::I2CBridgeError;

use super::{
    storages::TFtstorage,
    traits::{I2CAddress, I2CDevice},
    DecodeError, ModbusDevice, ValuesStorage,
};

pub struct DeviceDba2;

const DBA2_ID: u16 = 0xDBA2;
const DATA_SIZE: usize = 2 * core::mem::size_of::<f32>();

impl super::Device for DeviceDba2 {
    fn name(&self) -> &'static str {
        "DBA2"
    }

    fn data_size(&self) -> usize {
        DATA_SIZE
    }

    fn make_storage(&self) -> Box<dyn super::ValuesStorage> {
        Box::new(TFtstorage::new(self.name()))
    }
}

impl<I2C> I2CDevice<I2C> for DeviceDba2
where
    I2C: Read + Write + crate::hw::Reconfigure + crate::hw::Reset,
    I2CBridgeError: From<<I2C as embedded_hal::blocking::i2c::Read>::Error>
        + From<<I2C as embedded_hal::blocking::i2c::Write>::Error>,
{
    type Error = I2CBridgeError;

    fn probe_i2c(&self, addr: I2CAddress, i2c: &mut I2C) -> Result<(), Self::Error> {
        let id = super::probe_common::probe_i2c_0x80::<I2C, Self::Error>(addr, i2c)?;
        if id == DBA2_ID {
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
        super::read_comon::read_i2c_legacy_pic::<I2C, Self::Error>(addr, dest, i2c)
    }
}

impl ModbusDevice for DeviceDba2 {
    fn probe_resp(&self, id_resp: ResponseAdu<'_>) -> Result<(), DecodeError> {
        match id_resp.pdu.0 {
            Err(e) => Err(DecodeError::ResponseError(e)),
            Ok(Response::ReadHoldingRegisters(data)) => {
                if Some(DBA2_ID) == data.get(0) {
                    Ok(())
                } else {
                    Err(DecodeError::InvalidDeviceId)
                }
            }
            _ => Err(DecodeError::InvalidResponseType),
        }
    }

    fn build_data_request_iter(&self) -> Box<dyn Iterator<Item = RequestPdu<'static>>> {
        Box::new(
            [
                modbus_core::RequestPdu(modbus_core::Request::ReadInputRegisters(0x02, 2)), // T
                modbus_core::RequestPdu(modbus_core::Request::ReadInputRegisters(0x06, 2)), // Ft
            ]
            .into_iter().rev(),
        )
    }

    fn decode_resps(
        &self,
        dest: &mut dyn super::ValuesStorage,
        resps: &mut dyn Iterator<Item = &(alloc::vec::Vec<u8>, &str)>,
        bus_id: &'static str,
    ) -> Result<(), DecodeError> {
        super::read_comon::modbus_resp_to_storage_linear::<byteorder::LittleEndian>(bus_id, dest, resps)
    }
}
