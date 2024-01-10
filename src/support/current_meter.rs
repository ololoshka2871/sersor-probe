#![allow(dead_code)]

use embedded_hal::blocking::i2c;

use byteorder::{BigEndian, ByteOrder};

use crate::hw::I2cWraper;

// from crate inc219
enum Register {
    // Configuration = 0x00,
    ShuntVoltage = 0x01,
    BusVoltage = 0x02,
    Power = 0x03,
    Current = 0x04,
    Calibration = 0x05,
}

struct INA219<I2C> {
    i2c: I2C,
}

impl<I2C, E> INA219<I2C>
where
    I2C: i2c::Write<Error = E> + i2c::Read<Error = E>,
{
    pub fn new(i2c: I2C) -> INA219<I2C> {
        INA219 { i2c }
    }

    pub fn calibrate(&mut self, value: u16, address: u8) -> Result<(), E> {
        self.i2c.write(
            address,
            &[Register::Calibration as u8, (value >> 8) as u8, value as u8],
        )?;
        Ok(())
    }

    pub fn shunt_voltage(&mut self, address: u8) -> Result<i16, E> {
        let value = self.read(Register::ShuntVoltage, address)?;
        Ok(value as i16)
    }

    pub fn voltage(&mut self, address: u8) -> Result<u16, E> {
        let value = self.read(Register::BusVoltage, address)?;
        Ok((value >> 3) * 4)
    }

    pub fn power(&mut self, address: u8) -> Result<i16, E> {
        let value = self.read(Register::Power, address)?;
        Ok(value as i16)
    }

    pub fn current(&mut self, address: u8) -> Result<i16, E> {
        let value = self.read(Register::Current, address)?;
        Ok(value as i16)
    }

    fn read(&mut self, register: Register, address: u8) -> Result<u16, E> {
        let mut buf: [u8; 2] = [0x00; 2];
        self.i2c.write(address, &[register as u8])?;
        self.i2c.read(address, &mut buf)?;
        Ok(BigEndian::read_u16(&buf))
    }
}

pub struct CurrentMeter<I2C, PINS, const N: usize> {
    ina219: INA219<I2cWraper<I2C, PINS>>,
    address: [u8; N],
}

impl<I2C, PINS, const N: usize> CurrentMeter<I2C, PINS, N>
where
    I2C: stm32f1xx_hal::i2c::Instance,
{
    pub fn new(i2c: I2cWraper<I2C, PINS>, address: [u8; N]) -> Self {
        Self {
            ina219: INA219::new(i2c),
            address,
        }
    }

    pub fn calibrate(
        &mut self,
        value: u16,
    ) -> Result<(), <I2cWraper<I2C, PINS> as i2c::Write>::Error> {
        for address in self.address.iter() {
            self.ina219.calibrate(value, *address)?;
        }
        Ok(())
    }

    pub fn shunt_voltage(
        &mut self,
    ) -> Result<[i16; N], <I2cWraper<I2C, PINS> as i2c::Read>::Error> {
        let mut result = [0; N];
        for (i, address) in self.address.iter().enumerate() {
            result[i] = self.ina219.shunt_voltage(*address)?;
        }
        Ok(result)
    }

    pub fn voltage(&mut self) -> Result<[u16; N], <I2cWraper<I2C, PINS> as i2c::Read>::Error> {
        let mut result = [0; N];
        for (i, address) in self.address.iter().enumerate() {
            result[i] = self.ina219.voltage(*address)?;
        }
        Ok(result)
    }

    pub fn power(&mut self) -> Result<[i16; N], <I2cWraper<I2C, PINS> as i2c::Read>::Error> {
        let mut result = [0; N];
        for (i, address) in self.address.iter().enumerate() {
            result[i] = self.ina219.power(*address)?;
        }
        Ok(result)
    }

    pub fn current(&mut self, lsb: f32) -> Result<[f32; N], <I2cWraper<I2C, PINS> as i2c::Read>::Error> {
        let mut result = [0f32; N];
        for (i, address) in self.address.iter().enumerate() {
            result[i] = lsb * self.ina219.current(*address)? as f32;
        }
        Ok(result)
    }
}
