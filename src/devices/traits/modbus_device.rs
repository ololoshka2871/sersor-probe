use modbus_core::{rtu::ResponseAdu, RequestPdu};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DecodeError {
    InvalidDeviceId,
    InsuficientData,
    InvalidResponseType,
    ResponseError(modbus_core::ExceptionResponse),
}

pub trait ModbusDevice: super::Device {
    /// Probe the device at the given address.
    fn probe_resp(&self, id_resp: ResponseAdu<'_>) -> Result<(), DecodeError>;

    /// Build the request to read the device data.
    fn build_data_request(&self) -> RequestPdu<'static>;

    /// Read the device at the given address. The data is stored in the provided storage.
    fn decode_resp(
        &self,
        dest: &mut dyn super::ValuesStorage,
        resp: ResponseAdu<'_>,
    ) -> Result<(), DecodeError>;
}
