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
        super::read_comon::read_i2c_leacy_pic::<I2C, Self::Error>(addr, dest, i2c)
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

    fn build_data_request(&self) -> RequestPdu<'static> {
        modbus_core::RequestPdu(modbus_core::Request::ReadInputRegisters(
            0x00,
            (DATA_SIZE / core::mem::size_of::<u16>()) as u16,
        ))
    }

    fn decode_resp(
        &self,
        dest: &mut dyn super::ValuesStorage,
        resp: ResponseAdu<'_>,
    ) -> Result<(), DecodeError> {
        match resp.pdu.0 {
            Err(e) => Err(DecodeError::ResponseError(e)),
            Ok(Response::ReadInputRegisters(data)) => {
                if data.len() == DATA_SIZE / core::mem::size_of::<u16>() {
                    let mut buf = [0u8; DATA_SIZE];
                    for (i, w) in data.into_iter().enumerate() {
                        buf[i * core::mem::size_of::<u16>()..(i + 1) * core::mem::size_of::<u16>()]
                            .copy_from_slice(&w.to_be_bytes());
                    }
                    dest.copy_from(&buf);
                    Ok(())
                } else {
                    Err(DecodeError::InsuficientData)
                }
            }
            _ => Err(DecodeError::InvalidResponseType),
        }
    }
}
