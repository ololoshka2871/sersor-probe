use alloc::boxed::Box;
use modbus_core::{rtu::ResponseAdu, RequestPdu};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DecodeError {
    DecodeFailed,
    InvalidDeviceId,
    InsuficientData,
    InvalidResponseType,
    ResponseError(modbus_core::ExceptionResponse),
}

pub trait ModbusDevice: super::Device {
    /// Probe the device at the given address.
    fn probe_resp(&self, id_resp: ResponseAdu<'_>) -> Result<(), DecodeError>;

    /// Build the requests to read the device data.
    fn build_data_request_iter(&self) -> Box<dyn Iterator<Item = RequestPdu<'static>>>;

    /// Read the device at the given address. The data is stored in the provided storage.
    fn decode_resps(
        &self,
        dest: &mut dyn super::ValuesStorage,
        resps: &mut dyn Iterator<Item = &(alloc::vec::Vec<u8>, &str)>,
        bis_id: &'static str,
    ) -> Result<(), DecodeError>;
}
