use embedded_hal::blocking::i2c::{Read, Write};
use stm32f1xx_hal::i2c::{BlockingI2c, Error};

pub struct I2cWraper<I2C, PINS> {
    i2c: BlockingI2c<I2C, PINS>,
}

impl<I2C, PINS> I2cWraper<I2C, PINS> {
    pub fn new(i2c: BlockingI2c<I2C, PINS>) -> Self {
        Self { i2c }
    }
}

impl<I2C, PINS> Read for I2cWraper<I2C, PINS>
where
    BlockingI2c<I2C, PINS>: Read<Error = Error>,
    I2C: stm32f1xx_hal::i2c::Instance,
{
    type Error = Error;

    fn read(&mut self, addr: u8, buffer: &mut [u8]) -> Result<(), Self::Error> {
        self.i2c.read(addr, buffer)
    }
}

impl<I2C, PINS> Write for I2cWraper<I2C, PINS>
where
    BlockingI2c<I2C, PINS>: Write<Error = Error>,
    I2C: stm32f1xx_hal::i2c::Instance,
{
    type Error = Error;

    fn write(&mut self, addr: u8, bytes: &[u8]) -> Result<(), Self::Error> {
        self.i2c.write(addr, bytes)
    }
}

impl<I2C, PINS> super::super::Reconfigure for I2cWraper<I2C, PINS>
where
    I2C: stm32f1xx_hal::i2c::Instance,
{
    fn set_speed(&mut self, _speed: u32) -> bool {
        false
    }
}

impl<I2C, PINS> super::super::Reset for I2cWraper<I2C, PINS>
where
    I2C: stm32f1xx_hal::i2c::Instance,
{
    fn reset(&mut self) {
        //self.i2c.
    }
}
