use modbus_core::rtu::{client, server, RequestAdu, ResponseAdu};

use crate::bridge::{BufferTrait, RxBuffer};

use super::Buffer;

impl<const SIZE: usize> RxBuffer for Buffer<SIZE> {
    fn try_decode_request(&self) -> Option<RequestAdu> {
        server::decode_request(self.as_slice()).unwrap_or_default()
    }

    // from modbus-core/src/codec/tcp/server.rs: decode_response()
    fn try_decode_response(&self) -> Option<ResponseAdu> {
        client::decode_response(self.as_slice()).unwrap_or_default()
    }
}

impl RxBuffer for &[u8] {
    fn try_decode_request(&self) -> Option<RequestAdu> {
        server::decode_request(self).unwrap_or_default()
    }

    fn try_decode_response(&self) -> Option<ResponseAdu> {
        client::decode_response(self).unwrap_or_default()
    }
}
