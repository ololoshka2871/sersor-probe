//! Copy-pasted from [stm32f1xx_hal](https://github.com/stm32-rs/stm32f1xx-hal) and make some methods public

//! Inter-Integrated Circuit (I2C) bus

// This document describes a correct i2c implementation and is what
// parts of this code is based on
// https://www.st.com/content/ccc/resource/technical/document/application_note/5d/ae/a3/6f/08/69/4e/9b/CD00209826.pdf/files/CD00209826.pdf/jcr:content/translations/en.CD00209826.pdf

#![allow(dead_code, unused_imports)]

use embedded_hal::blocking::i2c::{Read, Write, WriteRead};
use stm32f1xx_hal::afio::MAPR;
use stm32f1xx_hal::gpio::{self, Alternate, OpenDrain};
use stm32f1xx_hal::pac::{DWT, I2C1, I2C2, RCC};
use stm32f1xx_hal::rcc::{BusClock, Clocks, Enable, Reset};
use stm32f1xx_hal::time::{kHz, Hertz};

pub use stm32f1xx_hal::i2c::{Instance, Pins}; 

pub mod blocking;
pub use blocking::BlockingI2c;

/// I2C error
#[derive(Debug, Eq, PartialEq)]
#[non_exhaustive]
pub enum Error {
    /// Bus error
    Bus,
    /// Arbitration loss
    Arbitration,
    /// No ack received
    Acknowledge,
    /// Overrun/underrun
    Overrun,
    // Timeout waiting SCL low
    Timeout,
}

#[derive(Debug, Eq, PartialEq, Clone, Copy)]
pub enum DutyCycle {
    Ratio2to1,
    Ratio16to9,
}

#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub enum Mode {
    Standard {
        frequency: Hertz,
    },
    Fast {
        frequency: Hertz,
        duty_cycle: DutyCycle,
    },
}

impl Mode {
    pub fn standard(frequency: Hertz) -> Self {
        Mode::Standard { frequency }
    }

    pub fn fast(frequency: Hertz, duty_cycle: DutyCycle) -> Self {
        Mode::Fast {
            frequency,
            duty_cycle,
        }
    }

    pub fn get_frequency(&self) -> Hertz {
        match *self {
            Mode::Standard { frequency } => frequency,
            Mode::Fast { frequency, .. } => frequency,
        }
    }
}

impl From<Hertz> for Mode {
    fn from(frequency: Hertz) -> Self {
        if frequency <= kHz(100) {
            Self::Standard { frequency }
        } else {
            Self::Fast {
                frequency,
                duty_cycle: DutyCycle::Ratio2to1,
            }
        }
    }
}

/// I2C peripheral operating in master mode
pub struct I2c<I2C, PINS> {
    i2c: I2C,
    pins: PINS,
    mode: Mode,
    pclk1: Hertz,
}

impl<PINS> I2c<I2C1, PINS> {
    /// Creates a generic I2C1 object on pins PB6 and PB7 or PB8 and PB9 (if remapped)
    pub fn i2c1<M: Into<Mode>>(
        i2c: I2C1,
        pins: PINS,
        mapr: &mut MAPR,
        mode: M,
        clocks: Clocks,
    ) -> Self
    where
        PINS: Pins<I2C1>,
    {
        mapr.modify_mapr(|_, w| w.i2c1_remap().bit(PINS::REMAP));
        I2c::<I2C1, _>::configure(i2c, pins, mode, clocks)
    }
}

impl<PINS> I2c<I2C2, PINS> {
    /// Creates a generic I2C2 object on pins PB10 and PB11 using the embedded-hal `BlockingI2c` trait.
    pub fn i2c2<M: Into<Mode>>(i2c: I2C2, pins: PINS, mode: M, clocks: Clocks) -> Self
    where
        PINS: Pins<I2C2>,
    {
        I2c::<I2C2, _>::configure(i2c, pins, mode, clocks)
    }
}

impl<I2C, PINS> I2c<I2C, PINS>
where
    I2C: Instance,
{
    /// Configures the I2C peripheral to work in master mode
    fn configure<M: Into<Mode>>(i2c: I2C, pins: PINS, mode: M, clocks: Clocks) -> Self {
        let mode = mode.into();
        let rcc = unsafe { &(*RCC::ptr()) };
        I2C::enable(rcc);
        I2C::reset(rcc);

        let pclk1 = I2C::clock(&clocks);

        assert!(mode.get_frequency() <= kHz(400));

        let mut i2c = I2c {
            i2c,
            pins,
            mode,
            pclk1,
        };
        i2c.init();
        i2c
    }
}

impl<I2C, PINS> I2c<I2C, PINS>
where
    I2C: Instance,
{
    /// Initializes I2C. Configures the `I2C_TRISE`, `I2C_CRX`, and `I2C_CCR` registers
    /// according to the system frequency and I2C mode.
    fn init(&mut self) {
        let freq = self.mode.get_frequency();
        let pclk1_mhz = self.pclk1.to_MHz() as u16;

        self.i2c
            .cr2
            .write(|w| unsafe { w.freq().bits(pclk1_mhz as u8) });
        self.i2c.cr1.write(|w| w.pe().clear_bit());

        match self.mode {
            Mode::Standard { .. } => {
                self.i2c
                    .trise
                    .write(|w| w.trise().bits((pclk1_mhz + 1) as u8));
                self.i2c
                    .ccr
                    .write(|w| unsafe { w.ccr().bits(((self.pclk1 / (freq * 2)) as u16).max(4)) });
            }
            Mode::Fast { ref duty_cycle, .. } => {
                self.i2c
                    .trise
                    .write(|w| w.trise().bits((pclk1_mhz * 300 / 1000 + 1) as u8));

                self.i2c.ccr.write(|w| {
                    let (freq, duty) = match duty_cycle {
                        DutyCycle::Ratio2to1 => (((self.pclk1 / (freq * 3)) as u16).max(1), false),
                        DutyCycle::Ratio16to9 => (((self.pclk1 / (freq * 25)) as u16).max(1), true),
                    };

                    unsafe { w.ccr().bits(freq).duty().bit(duty).f_s().set_bit() }
                });
            }
        };

        self.i2c.cr1.modify(|_, w| w.pe().set_bit());
    }

    /// Perform an I2C software reset
    pub fn reset(&mut self) {
        self.i2c.cr1.write(|w| w.pe().set_bit().swrst().set_bit());
        self.i2c.cr1.reset();
        self.init();
    }

    /// Generate START condition
    fn send_start(&mut self) {
        self.i2c.cr1.modify(|_, w| w.start().set_bit());
    }

    /// Sends the (7-Bit) address on the I2C bus. The 8th bit on the bus is set
    /// depending on wether it is a read or write transfer.
    fn send_addr(&self, addr: u8, read: bool) {
        self.i2c
            .dr
            .write(|w| w.dr().bits(addr << 1 | (u8::from(read))));
    }

    /// Generate STOP condition
    fn send_stop(&self) {
        self.i2c.cr1.modify(|_, w| w.stop().set_bit());
    }

    /// Releases the I2C peripheral and associated pins
    pub fn release(self) -> (I2C, PINS) {
        (self.i2c, self.pins)
    }

    #[deprecated(since = "0.7.1", note = "Please use release instead")]
    pub fn free(self) -> (I2C, PINS) {
        self.release()
    }
}
