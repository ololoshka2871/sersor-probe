use super::I2CBridgeError;
use embedded_hal::blocking::i2c::{Read, Write};

enum I2COperation {
    /// Read data into the provided buffer
    Read,
    /// Write data from the provided buffer
    Write,
    /// Speed control
    Speed,
    /// Reset i2c bus
    Reset,
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
    pub fn on(data_buff: &'a mut [u8]) -> Self {
        Self { data_buff }
    }

    pub fn execute<I2C>(self, i2c: &mut I2C) -> Result<&'a [u8], I2CBridgeError>
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
                    return Err(I2CBridgeError::ProtocolError);
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
                if len < 2 || len > 60 {
                    return Err(I2CBridgeError::ProtocolError);
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
