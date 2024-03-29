use alloc::boxed::Box;

use embedded_hal::blocking::i2c::{Read, Write};
use modbus_core::{rtu::ResponseAdu, RequestPdu, Response};

use crate::{bridge::I2CBridgeError, devices::read_comon::read_i2c_f32};

use super::{
    storages::PTFpFtTCpuVInStorage,
    traits::{I2CAddress, I2CDevice},
    DecodeError, ModbusDevice, ValuesStorage,
};

pub struct DeviceC002;

const C002_ID: u16 = 0xC002;
const DATA_SIZE: usize = 6 * core::mem::size_of::<f32>();

const T_CPU_REG_ADDR: u8 = 0x26;
const V_IN_REG_ADDR: u8 = 0x27;

impl super::Device for DeviceC002 {
    fn name(&self) -> &'static str {
        "C002"
    }

    fn data_size(&self) -> usize {
        DATA_SIZE
    }

    fn make_storage(&self) -> Box<dyn super::ValuesStorage> {
        Box::new(PTFpFtTCpuVInStorage::new(self.name()))
    }
}

impl<I2C> I2CDevice<I2C> for DeviceC002
where
    I2C: Read + Write + crate::hw::Reconfigure + crate::hw::Reset,
    I2CBridgeError: From<<I2C as embedded_hal::blocking::i2c::Read>::Error>
        + From<<I2C as embedded_hal::blocking::i2c::Write>::Error>,
{
    type Error = I2CBridgeError;

    fn probe_i2c(&self, addr: I2CAddress, i2c: &mut I2C) -> Result<(), Self::Error> {
        let id = super::probe_common::probe_i2c_0x80::<I2C, Self::Error>(addr, i2c)?;
        if id == C002_ID {
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

        // P + T + Fp + Ft
        super::read_comon::read_i2c_common::<I2C, Self::Error>(
            addr,
            dest,
            i2c,
            4 * core::mem::size_of::<f32>() as u8,
        )?;

        // Tcpu
        let t_cpu = read_i2c_f32::<I2C, Self::Error>(addr, i2c, T_CPU_REG_ADDR)?;
        dest.write_f32(t_cpu);

        // Vin
        let v_in = read_i2c_f32::<I2C, Self::Error>(addr, i2c, V_IN_REG_ADDR)?;
        dest.write_f32(v_in);

        Ok(())
    }
}

impl ModbusDevice for DeviceC002 {
    fn probe_resp(&self, id_resp: ResponseAdu<'_>) -> Result<(), DecodeError> {
        match id_resp.pdu.0 {
            Err(e) => Err(DecodeError::ResponseError(e)),
            Ok(Response::ReadHoldingRegisters(data)) => {
                if Some(C002_ID) == data.get(0) {
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
                modbus_core::RequestPdu(modbus_core::Request::ReadInputRegisters(0x00, 4)), // P + T  
                modbus_core::RequestPdu(modbus_core::Request::ReadInputRegisters(0x08, 4)), // Fp + Ft
                modbus_core::RequestPdu(modbus_core::Request::ReadInputRegisters(0x20, 4)), // Tcpu + Vin
            ]
            .into_iter().rev(), // reverse order
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
