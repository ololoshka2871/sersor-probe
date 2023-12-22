use defmt_macros::Format;

#[derive(Format)]
pub enum HwError {
    /// Bus error
    Bus = 1 << 0,
    /// Arbitration loss
    Arbitration = 1 << 1,
    /// No ack received
    Acknowledge = 1 << 2,
    /// Overrun/underrun
    Overrun = 1 << 3,
    /// Timeout
    Timeout = 1 << 4,
    /// unknown error
    Unknown = HwError::Bus as isize
        | HwError::Arbitration as isize
        | HwError::Acknowledge as isize
        | HwError::Overrun as isize
        | HwError::Timeout as isize,
}

#[derive(Format)]
pub enum I2CBridgeError {
    Ok,
    LengthError,
    InvalidCommand(u8),
    NotSupported,
    HwError(HwError),
}

impl Into<u8> for I2CBridgeError {
    fn into(self) -> u8 {
        match self {
            I2CBridgeError::Ok => 0x00,
            I2CBridgeError::LengthError => 0x81,
            I2CBridgeError::InvalidCommand(v) => v | 0x80,
            I2CBridgeError::NotSupported => 0x82,
            I2CBridgeError::HwError(v) => v as u8 | 0xC0,
        }
    }
}
