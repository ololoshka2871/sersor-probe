use super::i2c::Error;

pub use crate::bridge::{HwError, I2CBridgeError};

impl From<Error> for I2CBridgeError {
    fn from(e: Error) -> Self {
        let hw_err = match e {
            Error::Bus => HwError::Bus,
            Error::Arbitration => HwError::Arbitration,
            Error::Acknowledge => HwError::Acknowledge,
            Error::Overrun => HwError::Overrun,
            Error::Timeout => HwError::Timeout,
            _ => HwError::Unknown,
        };

        Self::HwError(hw_err)
    }
}
