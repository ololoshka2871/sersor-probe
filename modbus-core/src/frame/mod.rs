mod coils;
mod data;
pub(crate) mod rtu;
pub(crate) mod tcp;

pub use self::{coils::*, data::*};
use byteorder::{BigEndian, ByteOrder};
use core::fmt;

/// A Modbus function code.
///
/// It is represented by an unsigned 8 bit integer.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FnCode {
    ReadCoils,
    ReadDiscreteInputs,
    WriteSingleCoil,
    WriteMultipleCoils,
    ReadInputRegisters,
    ReadHoldingRegisters,
    WriteSingleRegister,
    WriteMultipleRegisters,
    ReadWriteMultipleRegisters,
    #[cfg(feature = "rtu")]
    ReadExceptionStatus,
    #[cfg(feature = "rtu")]
    Diagnostics,
    #[cfg(feature = "rtu")]
    GetCommEventCounter,
    #[cfg(feature = "rtu")]
    GetCommEventLog,
    #[cfg(feature = "rtu")]
    ReportServerId,
    //TODO:
    //- ReadFileRecord
    //- WriteFileRecord
    //- MaskWriteRegiger
    //TODO:
    //- Read FifoQueue
    //- EncapsulatedInterfaceTransport
    //- CanOpenGeneralReferenceRequestAndResponsePdu
    //- ReadDeviceIdentification
    Custom(u8),
}

impl From<u8> for FnCode {
    fn from(c: u8) -> Self {
        use FnCode::*;

        match c {
            0x01 => ReadCoils,
            0x02 => ReadDiscreteInputs,
            0x05 => WriteSingleCoil,
            0x0F => WriteMultipleCoils,
            0x04 => ReadInputRegisters,
            0x03 => ReadHoldingRegisters,
            0x06 => WriteSingleRegister,
            0x10 => WriteMultipleRegisters,
            0x17 => ReadWriteMultipleRegisters,
            #[cfg(feature = "rtu")]
            0x07 => ReadExceptionStatus,
            #[cfg(feature = "rtu")]
            0x08 => Diagnostics,
            #[cfg(feature = "rtu")]
            0x0B => GetCommEventCounter,
            #[cfg(feature = "rtu")]
            0x0C => GetCommEventLog,
            #[cfg(feature = "rtu")]
            0x11 => ReportServerId,
            _ => Custom(c),
        }
    }
}

impl From<FnCode> for u8 {
    fn from(code: FnCode) -> Self {
        use FnCode::*;

        match code {
            ReadCoils => 0x01,
            ReadDiscreteInputs => 0x02,
            WriteSingleCoil => 0x05,
            WriteMultipleCoils => 0x0F,
            ReadInputRegisters => 0x04,
            ReadHoldingRegisters => 0x03,
            WriteSingleRegister => 0x06,
            WriteMultipleRegisters => 0x10,
            ReadWriteMultipleRegisters => 0x17,
            #[cfg(feature = "rtu")]
            ReadExceptionStatus => 0x07,
            #[cfg(feature = "rtu")]
            Diagnostics => 0x08,
            #[cfg(feature = "rtu")]
            GetCommEventCounter => 0x0B,
            #[cfg(feature = "rtu")]
            GetCommEventLog => 0x0C,
            #[cfg(feature = "rtu")]
            ReportServerId => 0x11,
            Custom(c) => c,
        }
    }
}

/// A Modbus sub-function code is represented by an unsigned 16 bit integer.
#[cfg(feature = "rtu")]
pub(crate) type SubFnCode = u16;

/// A Modbus address is represented by 16 bit (from `0` to `65535`).
pub(crate) type Address = u16;

/// A Coil represents a single bit.
///
/// - `true` is equivalent to `ON`, `1` and `0xFF00`.
/// - `false` is equivalent to `OFF`, `0` and `0x0000`.
pub(crate) type Coil = bool;

/// Modbus uses 16 bit for its data items (big-endian representation).
pub(crate) type Word = u16;

/// Number of items to process (`0` - `65535`).
pub(crate) type Quantity = u16;

/// Raw PDU data
type RawData<'r> = &'r [u8];

/// A request represents a message from the client (master) to the server (slave).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Request<'r> {
    ReadCoils(Address, Quantity),
    ReadDiscreteInputs(Address, Quantity),
    WriteSingleCoil(Address, Coil),
    WriteMultipleCoils(Address, Coils<'r>),
    ReadInputRegisters(Address, Quantity),
    ReadHoldingRegisters(Address, Quantity),
    WriteSingleRegister(Address, Word),
    WriteMultipleRegisters(Address, Data<'r>),
    ReadWriteMultipleRegisters(Address, Quantity, Address, Data<'r>),
    #[cfg(feature = "rtu")]
    ReadExceptionStatus,
    #[cfg(feature = "rtu")]
    Diagnostics(SubFnCode, Data<'r>),
    #[cfg(feature = "rtu")]
    GetCommEventCounter,
    #[cfg(feature = "rtu")]
    GetCommEventLog,
    #[cfg(feature = "rtu")]
    ReportServerId,
    //TODO:
    //- ReadFileRecord
    //- WriteFileRecord
    //- MaskWriteRegiger
    //TODO:
    //- Read FifoQueue
    //- EncapsulatedInterfaceTransport
    //- CanOpenGeneralReferenceRequestAndResponsePdu
    //- ReadDeviceIdentification
    Custom(FnCode, &'r [u8]),
}

/// A server (slave) exception response.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ExceptionResponse {
    pub function: FnCode,
    pub exception: Exception,
}

/// Represents a message from the client (slave) to the server (master).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RequestPdu<'r>(pub Request<'r>);

/// Represents a message from the server (slave) to the client (master).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ResponsePdu<'r>(pub Result<Response<'r>, ExceptionResponse>);

#[cfg(feature = "rtu")]
type Status = u16;
#[cfg(feature = "rtu")]
type EventCount = u16;
#[cfg(feature = "rtu")]
type MessageCount = u16;

/// The response data of a successfull request.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Response<'r> {
    ReadCoils(Coils<'r>),
    ReadDiscreteInputs(Coils<'r>),
    WriteSingleCoil(Address),
    WriteMultipleCoils(Address, Quantity),
    ReadInputRegisters(Data<'r>),
    ReadHoldingRegisters(Data<'r>),
    WriteSingleRegister(Address, Word),
    WriteMultipleRegisters(Address, Quantity),
    ReadWriteMultipleRegisters(Data<'r>),
    #[cfg(feature = "rtu")]
    ReadExceptionStatus(u8),
    #[cfg(feature = "rtu")]
    Diagnostics(Data<'r>),
    #[cfg(feature = "rtu")]
    GetCommEventCounter(Status, EventCount),
    #[cfg(feature = "rtu")]
    GetCommEventLog(Status, EventCount, MessageCount, &'r [u8]),
    #[cfg(feature = "rtu")]
    ReportServerId(&'r [u8], bool),
    //TODO:
    //- ReadFileRecord
    //- WriteFileRecord
    //- MaskWriteRegiger
    //TODO:
    //- Read FifoQueue
    //- EncapsulatedInterfaceTransport
    //- CanOpenGeneralReferenceRequestAndResponsePdu
    //- ReadDeviceIdentification
    Custom(FnCode, &'r [u8]),
}

impl<'r> From<Request<'r>> for FnCode {
    fn from(r: Request<'r>) -> Self {
        use FnCode as c;
        use Request::*;

        match r {
            ReadCoils(_, _) => c::ReadCoils,
            ReadDiscreteInputs(_, _) => c::ReadDiscreteInputs,
            WriteSingleCoil(_, _) => c::WriteSingleCoil,
            WriteMultipleCoils(_, _) => c::WriteMultipleCoils,
            ReadInputRegisters(_, _) => c::ReadInputRegisters,
            ReadHoldingRegisters(_, _) => c::ReadHoldingRegisters,
            WriteSingleRegister(_, _) => c::WriteSingleRegister,
            WriteMultipleRegisters(_, _) => c::WriteMultipleRegisters,
            ReadWriteMultipleRegisters(_, _, _, _) => c::ReadWriteMultipleRegisters,
            #[cfg(feature = "rtu")]
            ReadExceptionStatus => c::ReadExceptionStatus,
            #[cfg(feature = "rtu")]
            Diagnostics(_, _) => c::Diagnostics,
            #[cfg(feature = "rtu")]
            GetCommEventCounter => c::GetCommEventCounter,
            #[cfg(feature = "rtu")]
            GetCommEventLog => c::GetCommEventLog,
            #[cfg(feature = "rtu")]
            ReportServerId => c::ReportServerId,
            Custom(code, _) => code,
        }
    }
}

impl<'r> From<Response<'r>> for FnCode {
    fn from(r: Response<'r>) -> Self {
        use FnCode as c;
        use Response::*;

        match r {
            ReadCoils(_) => c::ReadCoils,
            ReadDiscreteInputs(_) => c::ReadDiscreteInputs,
            WriteSingleCoil(_) => c::WriteSingleCoil,
            WriteMultipleCoils(_, _) => c::WriteMultipleCoils,
            ReadInputRegisters(_) => c::ReadInputRegisters,
            ReadHoldingRegisters(_) => c::ReadHoldingRegisters,
            WriteSingleRegister(_, _) => c::WriteSingleRegister,
            WriteMultipleRegisters(_, _) => c::WriteMultipleRegisters,
            ReadWriteMultipleRegisters(_) => c::ReadWriteMultipleRegisters,
            #[cfg(feature = "rtu")]
            ReadExceptionStatus(_) => c::ReadExceptionStatus,
            #[cfg(feature = "rtu")]
            Diagnostics(_) => c::Diagnostics,
            #[cfg(feature = "rtu")]
            GetCommEventCounter(_, _) => c::GetCommEventCounter,
            #[cfg(feature = "rtu")]
            GetCommEventLog(_, _, _, _) => c::GetCommEventLog,
            #[cfg(feature = "rtu")]
            ReportServerId(_, _) => c::ReportServerId,
            Custom(code, _) => code,
        }
    }
}

/// A server (slave) exception.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Exception {
    IllegalFunction = 0x01,
    IllegalDataAddress = 0x02,
    IllegalDataValue = 0x03,
    ServerDeviceFailure = 0x04,
    Acknowledge = 0x05,
    ServerDeviceBusy = 0x06,
    MemoryParityError = 0x08,
    GatewayPathUnavailable = 0x0A,
    GatewayTargetDevice = 0x0B,
}

impl fmt::Display for Exception {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        use self::Exception::*;

        let desc = match *self {
            IllegalFunction => "Illegal function",
            IllegalDataAddress => "Illegal data address",
            IllegalDataValue => "Illegal data value",
            ServerDeviceFailure => "Server device failure",
            Acknowledge => "Acknowledge",
            ServerDeviceBusy => "Server device busy",
            MemoryParityError => "Memory parity error",
            GatewayPathUnavailable => "Gateway path unavailable",
            GatewayTargetDevice => "Gateway target device failed to respond",
        };
        write!(f, "{}", desc)
    }
}

impl<'r> Request<'r> {
    /// Number of bytes required for a serialized PDU frame.
    pub fn pdu_len(&self) -> usize {
        use Request::*;
        match *self {
            ReadCoils(_, _)
            | ReadDiscreteInputs(_, _)
            | ReadInputRegisters(_, _)
            | ReadHoldingRegisters(_, _)
            | WriteSingleRegister(_, _)
            | WriteSingleCoil(_, _) => 5,
            WriteMultipleCoils(_, coils) => 6 + coils.packed_len(),
            WriteMultipleRegisters(_, words) => 6 + words.data.len(),
            ReadWriteMultipleRegisters(_, _, _, words) => 10 + words.data.len(),
            Custom(_, data) => 1 + data.len(),
            #[cfg(feature = "rtu")]
            _ => unimplemented!(), // TODO
        }
    }
}

impl<'r> Response<'r> {
    /// Number of bytes required for a serialized PDU frame.
    pub fn pdu_len(&self) -> usize {
        use Response::*;
        match *self {
            ReadCoils(coils) | ReadDiscreteInputs(coils) => 2 + coils.packed_len(),
            WriteSingleCoil(_) => 3,
            WriteMultipleCoils(_, _) | WriteMultipleRegisters(_, _) | WriteSingleRegister(_, _) => {
                5
            }
            ReadInputRegisters(words)
            | ReadHoldingRegisters(words)
            | ReadWriteMultipleRegisters(words) => 2 + words.len() * 2,
            Custom(_, data) => 1 + data.len(),
            #[cfg(feature = "rtu")]
            _ => unimplemented!(), // TODO
        }
    }
}

#[cfg(test)]
mod tests {

    use super::*;

    #[test]
    fn function_code_into_u8() {
        let x: u8 = FnCode::WriteMultipleCoils.into();
        assert_eq!(x, 15);
        let x: u8 = FnCode::Custom(0xBB).into();
        assert_eq!(x, 0xBB);
    }

    #[test]
    fn function_code_from_u8() {
        assert_eq!(FnCode::from(15), FnCode::WriteMultipleCoils);
        assert_eq!(FnCode::from(0xBB), FnCode::Custom(0xBB));
    }

    #[test]
    fn function_code_from_request() {
        use Request::*;
        let requests = &[
            (ReadCoils(0, 0), 1),
            (ReadDiscreteInputs(0, 0), 2),
            (WriteSingleCoil(0, true), 5),
            (
                WriteMultipleCoils(
                    0,
                    Coils {
                        quantity: 0,
                        data: &[],
                    },
                ),
                0x0F,
            ),
            (ReadInputRegisters(0, 0), 0x04),
            (ReadHoldingRegisters(0, 0), 0x03),
            (WriteSingleRegister(0, 0), 0x06),
            (
                WriteMultipleRegisters(
                    0,
                    Data {
                        quantity: 0,
                        data: &[],
                    },
                ),
                0x10,
            ),
            (
                ReadWriteMultipleRegisters(
                    0,
                    0,
                    0,
                    Data {
                        quantity: 0,
                        data: &[],
                    },
                ),
                0x17,
            ),
            (Custom(FnCode::Custom(88), &[]), 88),
        ];
        for (req, expected) in requests {
            let code: u8 = FnCode::from(*req).into();
            assert_eq!(*expected, code);
        }
    }

    #[test]
    fn function_code_from_response() {
        use Response::*;
        let responses = &[
            (
                ReadCoils(Coils {
                    quantity: 0,
                    data: &[],
                }),
                1,
            ),
            (
                ReadDiscreteInputs(Coils {
                    quantity: 0,
                    data: &[],
                }),
                2,
            ),
            (WriteSingleCoil(0x0), 5),
            (WriteMultipleCoils(0x0, 0x0), 0x0F),
            (
                ReadInputRegisters(Data {
                    quantity: 0,
                    data: &[],
                }),
                0x04,
            ),
            (
                ReadHoldingRegisters(Data {
                    quantity: 0,
                    data: &[],
                }),
                0x03,
            ),
            (WriteSingleRegister(0, 0), 0x06),
            (WriteMultipleRegisters(0, 0), 0x10),
            (
                ReadWriteMultipleRegisters(Data {
                    quantity: 0,
                    data: &[],
                }),
                0x17,
            ),
            (Custom(FnCode::Custom(99), &[]), 99),
        ];
        for (req, expected) in responses {
            let code: u8 = FnCode::from(*req).into();
            assert_eq!(*expected, code);
        }
    }

    #[test]
    fn test_request_pdu_len() {
        assert_eq!(Request::ReadCoils(0x12, 5).pdu_len(), 5);
        assert_eq!(Request::WriteSingleRegister(0x12, 0x33).pdu_len(), 5);
        let buf = &mut [0, 0];
        assert_eq!(
            Request::WriteMultipleCoils(0, Coils::from_bools(&[true, false], buf).unwrap())
                .pdu_len(),
            7
        );
        // TODO: extend test
    }

    #[test]
    fn test_response_pdu_len() {
        let buf = &mut [0, 0];
        assert_eq!(
            Response::ReadCoils(Coils::from_bools(&[true], buf).unwrap()).pdu_len(),
            3
        );
        // TODO: extend test
    }
}
