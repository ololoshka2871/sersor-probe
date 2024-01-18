use core::fmt;

/// modbus-core Error
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Error {
    /// Invalid coil value
    CoilValue(u16),
    /// Invalid buffer size
    BufferSize,
    /// Invalid function code
    FnCode(u8),
    /// Invalid exception code
    ExceptionCode(u8),
    /// Invalid exception function code
    ExceptionFnCode(u8),
    /// Invalid CRC
    Crc(u16, u16),
    /// Invalid byte count
    ByteCount(u8),
    /// Length Mismatch
    LengthMismatch(usize, usize),
    /// Protocol not Modbus
    ProtocolNotModbus(u16),
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        use Error::*;

        match self {
            CoilValue(v) => write!(f, "Invalid coil value: {}", v),
            BufferSize => write!(f, "Invalid buffer size"),
            FnCode(fn_code) => write!(f, "Invalid function code: 0x{:0>2X}", fn_code),
            ExceptionCode(code) => write!(f, "Invalid exception code:0x {:0>2X}", code),
            ExceptionFnCode(code) => write!(f, "Invalid exception function code:0x {:0>2X}", code),
            Crc(expected, actual) => write!(
                f,
                "Invalid CRC: expected = 0x{:0>4X}, actual = 0x{:0>4X}",
                expected, actual
            ),
            ByteCount(cnt) => write!(f, "Invalid byte count: {}", cnt),
            LengthMismatch(length_field, pdu_len) => write!(
                f,
                "Length Mismatch: Length Field: {}, PDU Len + 1: {}",
                length_field, pdu_len
            ),
            ProtocolNotModbus(protocol_id) => write!(
                f,
                "Protocol not Modbus(0), recieved {} instead",
                protocol_id
            ),
        }
    }
}
