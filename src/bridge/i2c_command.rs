use super::I2CBridgeError;
use embedded_hal::blocking::i2c::{Read, Write};

enum I2COperation {
    /// Read data into the provided buffer
    Read = 0x0B,
    /// Write data from the provided buffer
    Write = 0x0A,
    /// Speed control
    Speed = 0x10,
    /// Reset i2c bus
    Reset = 0x69,
}

impl TryFrom<u8> for I2COperation {
    type Error = I2CBridgeError;

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            0x0B => Ok(I2COperation::Read),
            0x0A => Ok(I2COperation::Write),
            0x10 => Ok(I2COperation::Speed),
            0x69 => Ok(I2COperation::Reset),
            _ => Err(I2CBridgeError::InvalidCommand(value)),
        }
    }
}

pub struct MyI2COperation<'a> {
    data_buff: &'a mut [u8],
}

impl<'a> MyI2COperation<'a> {
    pub const MIN_BUF_SIZE: usize = 4;

    pub fn on(data_buff: &'a mut [u8]) -> Self {
        Self { data_buff }
    }
}

impl<'a> super::Builder<'a> for MyI2COperation<'a> {
    fn new_scan_op(data_buff: &'a mut [u8; 4], scan_addr: u8) -> Self {
        data_buff[0] = I2COperation::Read as u8;
        data_buff[1] = 1; // len
        data_buff[2] = scan_addr; // addr
        data_buff[3] = 0; // data storage place
        Self { data_buff }
    }

    fn new_read_op(data_buff: &'a mut [u8], dev_addr: u8, size: u8) -> Self {
        data_buff[0] = I2COperation::Read as u8;
        data_buff[1] = size; // len
        data_buff[2] = dev_addr; // addr
        Self { data_buff }
    }

    fn new_write_op(data_buff: &'a mut [u8], dev_addr: u8, to_write: &'a [u8]) -> Self {
        data_buff[0] = I2COperation::Write as u8;
        data_buff[1] = to_write.len() as u8 + 1; // len + dev_addr
        data_buff[2] = dev_addr; // addr
        data_buff[3..3 + to_write.len()].copy_from_slice(to_write);
        Self { data_buff }
    }
}

impl<'a> super::Execute<'a> for MyI2COperation<'a> {
    fn execute<I2C>(self, i2c: &mut I2C) -> Result<&'a [u8], I2CBridgeError>
    where
        I2C: Read + Write + crate::hw::Reconfigure + crate::hw::Reset,
        I2CBridgeError: From<<I2C as embedded_hal::blocking::i2c::Read>::Error>
            + From<<I2C as embedded_hal::blocking::i2c::Write>::Error>,
    {
        let opcode = self.data_buff[0];
        match I2COperation::try_from(opcode) {
            Ok(I2COperation::Read) => {
                let len = self.data_buff[1] as usize;
                if len < 1 || len > 60 {
                    return Err(I2CBridgeError::LengthError);
                }
                let dev_addr = self.data_buff[2];
                let dest = &mut self.data_buff[2..2 + len];

                defmt::trace!("I2C Read {} bytes from 0x{:X} ", len, dev_addr);
                i2c.read(dev_addr, dest)
                    .map_err(|e| I2CBridgeError::from(e))?;

                self.data_buff[0] = I2CBridgeError::Ok.into();
                self.data_buff[1] = len as u8;
                Ok(self.data_buff)
            }
            Ok(I2COperation::Write) => {
                let len = self.data_buff[1] as usize;
                if len < 2 || len > 61 {
                    return Err(I2CBridgeError::LengthError);
                }
                let dev_addr = self.data_buff[2];
                let src = &self.data_buff[3..3 + len - 1];

                defmt::trace!("I2C Write {} to 0x{:X} ", src, dev_addr);
                i2c.write(dev_addr, src)
                    .map_err(|e| I2CBridgeError::from(e))?;

                self.data_buff[0] = I2CBridgeError::Ok.into();
                self.data_buff[1] = 0;
                Ok(self.data_buff)
            }
            Ok(I2COperation::Speed) => {
                let new_speed_khz =
                    systick_monotonic::fugit::Hertz::<u32>::kHz(u16::from_le_bytes([
                        self.data_buff[1],
                        self.data_buff[2],
                    ]) as u32);
                defmt::trace!("I2C Speed change to {} kHz", new_speed_khz.to_kHz());
                if i2c.set_speed(new_speed_khz) {
                    self.data_buff[0] = I2CBridgeError::Ok.into();
                    self.data_buff[1] = 0;
                    Ok(self.data_buff)
                } else {
                    Err(I2CBridgeError::NotSupported)
                }
            }
            Ok(I2COperation::Reset) => {
                defmt::trace!("I2C Reset");
                i2c.reset();

                self.data_buff[0] = I2CBridgeError::Ok.into();
                self.data_buff[1] = 0;
                Ok(self.data_buff)
            }
            Err(e) => Err(e),
        }
    }
}
