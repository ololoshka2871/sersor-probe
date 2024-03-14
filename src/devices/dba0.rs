use alloc::boxed::Box;

use embedded_hal::blocking::i2c::{Read, Write};
use modbus_core::{rtu::ResponseAdu, RequestPdu, Response};

use crate::bridge::I2CBridgeError;

use super::{
    storages::PTFpFtstorage,
    traits::{I2CAddress, I2CDevice},
    DecodeError, ModbusDevice, ValuesStorage,
};

pub struct DeviceDba0;

const DBA0_ID: u16 = 0xDBA0;
const DATA_SIZE: usize = 4 * core::mem::size_of::<f32>();

impl super::Device for DeviceDba0 {
    fn name(&self) -> &'static str {
        "DBA0"
    }

    fn data_size(&self) -> usize {
        DATA_SIZE
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
        super::read_comon::read_i2c_legacy_pic::<I2C, Self::Error>(addr, dest, i2c)
    }
}

impl ModbusDevice for DeviceDba0 {
    fn probe_resp(&self, id_resp: ResponseAdu<'_>) -> Result<(), DecodeError> {
        match id_resp.pdu.0 {
            Err(e) => Err(DecodeError::ResponseError(e)),
            Ok(Response::ReadHoldingRegisters(data)) => {
                if Some(DBA0_ID) == data.get(0) {
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
                // Специально разбито на 3 запроса для тестирования
                modbus_core::RequestPdu(modbus_core::Request::ReadInputRegisters(0x00, 2)), // P
                modbus_core::RequestPdu(modbus_core::Request::ReadInputRegisters(0x02, 2)), // T
                modbus_core::RequestPdu(modbus_core::Request::ReadInputRegisters(0x04, 4)), // Fp + Ft
            ]
            .into_iter(),
        )
    }

    fn decode_resps(
        &self,
        dest: &mut dyn super::ValuesStorage,
        resps: &mut dyn Iterator<Item = &(alloc::vec::Vec<u8>, &str)>,
        bus_id: &'static str,
    ) -> Result<(), DecodeError> {
        super::read_comon::modbus_resp_to_storage_linear(bus_id, dest, resps)
    }
}
