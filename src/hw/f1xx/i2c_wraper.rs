use crate::hw::Reset;

use super::{
    i2c::{BlockingI2c, Error},
    Mode,
};
use embedded_hal::blocking::i2c::{Read, Write};
use stm32f1xx_hal::i2c::Pins;
use stm32f1xx_hal::pac::{I2C1, I2C2};
use stm32f1xx_hal::{afio::MAPR, rcc::Clocks};

pub struct I2cWraper<I2C, PINS> {
    i2c: Option<BlockingI2c<I2C, PINS>>,
    clocks: Clocks,
    mode: Mode,
}

pub const START_TIMEOUT_US: u32 = 1000;
pub const START_RETRIES: u8 = 2;
pub const ADDR_TIMEOUT_US: u32 = 1000;
pub const DATA_TIMEOUT_US: u32 = 10000; // должен успеть проходить SCL stretch

impl<PINS> I2cWraper<I2C1, PINS>
where
    PINS: Pins<I2C1>,
{
    pub fn i2c1(i2c1: I2C1, pins: PINS, mapr: &mut MAPR, clocks: Clocks, mode: Mode) -> Self {
        let i2c = crate::hw::BlockingI2c::i2c1(
            i2c1,
            pins,
            mapr,
            mode,
            clocks,
            START_TIMEOUT_US,
            START_RETRIES,
            ADDR_TIMEOUT_US,
            DATA_TIMEOUT_US,
        );

        Self {
            i2c: Some(i2c),
            clocks,
            mode,
        }
    }

    pub fn new_sim(clocks: Clocks, mode: Mode) -> Self {
        Self {
            i2c: None,
            clocks,
            mode,
        }
    }
}

impl<PINS> I2cWraper<I2C2, PINS>
where
    PINS: Pins<I2C2>,
{
    pub fn i2c2(i2c2: I2C2, pins: PINS, clocks: Clocks, mode: Mode) -> Self {
        let i2c = crate::hw::BlockingI2c::i2c2(
            i2c2,
            pins,
            mode,
            clocks,
            START_TIMEOUT_US,
            START_RETRIES,
            ADDR_TIMEOUT_US,
            DATA_TIMEOUT_US,
        );

        Self {
            i2c: Some(i2c),
            clocks,
            mode,
        }
    }
}

impl<I2C, PINS> Read for I2cWraper<I2C, PINS>
where
    BlockingI2c<I2C, PINS>: Read<Error = Error>,
    I2C: stm32f1xx_hal::i2c::Instance,
{
    type Error = Error;

    fn read(&mut self, addr: u8, buffer: &mut [u8]) -> Result<(), Self::Error> {
        if let Some(i2c) = &mut self.i2c {
            i2c.read(addr, buffer)
        } else {
            Ok(())
        }
    }
}

impl<I2C, PINS> Write for I2cWraper<I2C, PINS>
where
    BlockingI2c<I2C, PINS>: Write<Error = Error>,
    I2C: stm32f1xx_hal::i2c::Instance,
{
    type Error = Error;

    fn write(&mut self, addr: u8, bytes: &[u8]) -> Result<(), Self::Error> {
        if let Some(i2c) = &mut self.i2c {
            i2c.write(addr, bytes)
        } else {
            Ok(())
        }
    }
}

impl<I2C, PINS> super::super::Reconfigure for I2cWraper<I2C, PINS>
where
    I2C: stm32f1xx_hal::i2c::Instance,
{
    fn set_speed(&mut self, speed: systick_monotonic::fugit::Hertz<u32>) -> bool {
        use stm32f1xx_hal::time::Hertz;

        match speed.to_kHz() {
            0..=100 => {
                self.mode = Mode::Standard {
                    frequency: Hertz::kHz(100),
                }
            }
            101..=400 => {
                self.mode = Mode::Fast {
                    frequency: Hertz::kHz(400),
                    duty_cycle: super::i2c::DutyCycle::Ratio16to9,
                }
            }
            _ => return false,
        }

        self.reset();

        true
    }
}

impl<I2C, PINS> super::super::Reset for I2cWraper<I2C, PINS>
where
    I2C: stm32f1xx_hal::i2c::Instance,
{
    fn reset(&mut self) {
        if let Some(i2c) = self.i2c.take() {
            let (i2c, pins) = i2c.free();
            self.i2c = Some(BlockingI2c::<I2C, PINS>::configure(
                i2c,
                pins,
                self.mode,
                self.clocks,
                START_TIMEOUT_US,
                START_RETRIES,
                ADDR_TIMEOUT_US,
                DATA_TIMEOUT_US,
            ));
        }
    }
}
