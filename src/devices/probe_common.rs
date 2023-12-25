use embedded_hal::blocking::i2c::{Read, Write};

use crate::bridge::{Builder, Execute, I2CBridgeError, MyI2COperation};

use super::I2CAddress;

const ID_REG_I2C: u8 = 0x80;

pub fn probe_i2c_0x80<I2C, Error>(addr: I2CAddress, i2c: &mut I2C) -> Result<u16, I2CBridgeError>
where
    I2C: Read + Write + crate::hw::Reconfigure + crate::hw::Reset,
    I2CBridgeError: From<<I2C as embedded_hal::blocking::i2c::Read>::Error>
        + From<<I2C as embedded_hal::blocking::i2c::Write>::Error>,
{
    let mut buff = [0u8; MyI2COperation::MIN_BUF_SIZE];

    let cmd = MyI2COperation::new_write_op(&mut buff, addr, &[ID_REG_I2C]);
    cmd.execute(i2c)?;

    let cmd = MyI2COperation::new_read_op(&mut buff, addr, core::mem::size_of::<u16>() as u8);
    let resp = cmd.execute(i2c)?;
    Ok(u16::from_le_bytes([resp[2], resp[3]]))
}
