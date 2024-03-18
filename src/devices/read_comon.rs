use core::any::TypeId;

use alloc::vec::Vec;
use byteorder::{BigEndian, LittleEndian};
use embedded_hal::blocking::i2c::{Read, Write};
use modbus_core::Response;

use crate::bridge::{Builder, Execute, I2CBridgeError, MyI2COperation, RxBuffer};

use super::{DecodeError, I2CAddress, ValuesStorage};

const DATA_REG_LEGACY_I2C: u8 = 0x00;

pub fn read_i2c_common<I2C, Error>(
    addr: I2CAddress,
    dest: &mut dyn ValuesStorage,
    i2c: &mut I2C,
    size: u8,
) -> Result<(), I2CBridgeError>
where
    I2C: Read + Write + crate::hw::Reconfigure + crate::hw::Reset,
    I2CBridgeError: From<<I2C as embedded_hal::blocking::i2c::Read>::Error>
        + From<<I2C as embedded_hal::blocking::i2c::Write>::Error>,
{
    let mut buff = [0u8; MyI2COperation::MAX_BUF_SIZE];

    let cmd = MyI2COperation::new_write_op(&mut buff, addr, &[DATA_REG_LEGACY_I2C]);
    cmd.execute(i2c)?;

    let cmd = MyI2COperation::new_read_op(&mut buff, addr, size);
    let resp = cmd.execute(i2c)?;

    dest.copy_from(&resp[2..(2 + size as usize)]);
    Ok(())
}

pub fn read_i2c_legacy_pic<I2C, Error>(
    addr: I2CAddress,
    dest: &mut dyn ValuesStorage,
    i2c: &mut I2C,
) -> Result<(), I2CBridgeError>
where
    I2C: Read + Write + crate::hw::Reconfigure + crate::hw::Reset,
    I2CBridgeError: From<<I2C as embedded_hal::blocking::i2c::Read>::Error>
        + From<<I2C as embedded_hal::blocking::i2c::Write>::Error>,
{
    let size = dest.size() as u8;
    read_i2c_common::<I2C, Error>(addr, dest, i2c, size)
}

pub fn read_i2c_f32<I2C, Error>(
    dev_addr: I2CAddress,
    i2c: &mut I2C,
    addr: u8,
) -> Result<f32, I2CBridgeError>
where
    I2C: Read + Write + crate::hw::Reconfigure + crate::hw::Reset,
    I2CBridgeError: From<<I2C as embedded_hal::blocking::i2c::Read>::Error>
        + From<<I2C as embedded_hal::blocking::i2c::Write>::Error>,
{
    let mut buff = [0u8; MyI2COperation::MAX_BUF_SIZE];
    let addr = [addr];

    let cmd = MyI2COperation::new_write_op(&mut buff, dev_addr, &addr);
    cmd.execute(i2c)?;

    let cmd = MyI2COperation::new_read_op(&mut buff, dev_addr, core::mem::size_of::<f32>() as u8);
    let resp = cmd.execute(i2c)?;

    unsafe {
        Ok(core::mem::transmute::<&[u8], &[f32]>(&resp[2..2 + core::mem::size_of::<f32>()])[0])
    }
}

pub fn modbus_resp_to_storage_linear<Endian: 'static>(
    bus_id: &'static str,
    dest: &mut dyn super::ValuesStorage,
    resps: &mut dyn Iterator<Item = &(Vec<u8>, &str)>,
) -> Result<(), DecodeError> {
    let buf = dest.as_mut_slice();
    let mut offset = 0usize;

    while let Some((resp, bus)) = resps.next() {
        if bus != &bus_id {
            return Err(DecodeError::InvalidDeviceId);
        }

        if let Some(resp) = resp.as_slice().try_decode_response() {
            match resp.pdu.0 {
                Err(e) => return Err(DecodeError::ResponseError(e)),
                Ok(Response::ReadInputRegisters(data)) => {
                    for word in data.into_iter() {
                        if offset + core::mem::size_of::<u16>() > buf.len() {
                            return Err(DecodeError::InsuficientData);
                        }

                        buf[offset..offset + core::mem::size_of::<u16>()].copy_from_slice(
                            &match TypeId::of::<Endian>() {
                                e if e == TypeId::of::<BigEndian>() => word.to_be_bytes(),
                                e if e == TypeId::of::<LittleEndian>() => word.to_le_bytes(),
                                _ => panic!("Unsupported endian type!"),
                            },
                        );

                        offset += core::mem::size_of::<u16>();
                    }
                }
                _ => return Err(DecodeError::InvalidResponseType),
            }
        } else {
            return Err(DecodeError::DecodeFailed);
        }
    }

    if offset != buf.len() {
        Err(DecodeError::InsuficientData)
    } else {
        Ok(())
    }
}
