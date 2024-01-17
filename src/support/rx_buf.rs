use modbus_core::{
    rtu::{decode, server, DecodedFrame, Header, RequestAdu, ResponseAdu},
    ExceptionResponse, Response, ResponsePdu,
};

use crate::bridge::{BufferTrait, RxBuffer};

use super::Buffer;

// !Грязный хак! Этот тип не сделан публичным в модуле modbus_core
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum DecoderType {
    #[allow(unused)]
    Request,
    Response,
}

fn decode_response(data: &[u8]) -> Option<ResponseAdu> {
    decode(unsafe { core::mem::transmute(DecoderType::Response) }, data)
        .and_then(|frame| {
            if let Some((DecodedFrame { slave, pdu }, _frame_pos)) = frame {
                let hdr = Header { slave };
                // Decoding of the PDU should are unlikely to fail due
                // to transmission errors, because the frame's bytes
                // have already been verified at the TCP level.

                Response::try_from(pdu)
                    .map(Ok)
                    .or_else(|_| ExceptionResponse::try_from(pdu).map(Err))
                    .map(ResponsePdu)
                    .map(|pdu| Some(ResponseAdu { hdr, pdu }))
                    .map_err(|err| {
                        // Unrecoverable error
                        defmt::panic!(
                            "Failed to decode response PDU: {}",
                            defmt::Debug2Format(&err)
                        );
                    })
            } else {
                Ok(None)
            }
        })
        .unwrap_or_default()
}

impl<const SIZE: usize> RxBuffer for Buffer<SIZE> {
    fn try_decode_request(&self) -> Option<RequestAdu> {
        server::decode_request(self.as_slice()).unwrap_or_default()
    }

    // from modbus-core/src/codec/tcp/server.rs: decode_response()
    fn try_decode_response(&self) -> Option<ResponseAdu> {
        decode_response(self.as_slice())
    }
}

impl RxBuffer for &[u8] {
    fn try_decode_request(&self) -> Option<RequestAdu> {
        server::decode_request(self).unwrap_or_default()
    }

    fn try_decode_response(&self) -> Option<ResponseAdu> {
        decode_response(self)
    }
}
