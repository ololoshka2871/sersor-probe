pub trait ModbusDevice: super::Device {
    type Error;

    /// Probe the device at the given address.
    fn probe_resp(&self, id_resp: modbus_core::rtu::ResponseAdu<'_>) -> Result<(), ()>;

    /// Build the request to read the device data.
    fn build_data_request(&self);

    /// Read the device at the given address. The data is stored in the provided storage.
    fn decode_resp(&self) -> Result<(), Self::Error>;
}
