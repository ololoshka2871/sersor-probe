use embedded_hal::blocking::i2c::{Read, Write};

use crate::bridge::{Builder, Execute, I2CBridgeError, MyI2COperation};

use super::I2CAddress;

const DATA_REG_LEGACY_I2C: u8 = 0x00;

pub fn read_i2c_leacy_pic<I2C, Error>(
    addr: I2CAddress,
    dest: &mut [u8],
    i2c: &mut I2C,
) -> Result<(), I2CBridgeError>
where
    I2C: Read + Write + crate::hw::Reconfigure + crate::hw::Reset,
    I2CBridgeError: From<<I2C as embedded_hal::blocking::i2c::Read>::Error>
        + From<<I2C as embedded_hal::blocking::i2c::Write>::Error>,
{
    let mut buff = [0u8; MyI2COperation::MAX_BUF_SIZE];

    let cmd = MyI2COperation::new_write_op(&mut buff, addr, &[DATA_REG_LEGACY_I2C]);
    cmd.execute(i2c)?;

    let cmd = MyI2COperation::new_read_op(&mut buff, addr, dest.len() as u8);
    let resp = cmd.execute(i2c)?;

    dest.copy_from_slice(&resp[2..2 + dest.len()]);
    Ok(())
}
