//! Modbus RTU server (slave) specific functions.
use super::*;

/// Decode an RTU response.
pub fn decode_response(buf: &[u8]) -> Result<Option<ResponseAdu>> {
    decode(DecoderType::Response, buf)
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
                        error!("Failed to decode response PDU: {}", err);
                        err
                    })
            } else {
                Ok(None)
            }
        })
        .map_err(|_| {
            // Decoding the transport frame is non-destructive and must
            // never fail!
            unreachable!();
        })
}

/// Encode an RTU request.
pub fn encode_request(adu: &RequestAdu, buf: &mut [u8]) -> Result<usize> {
    let RequestAdu { hdr, pdu } = adu;
    if buf.len() < 2 {
        return Err(Error::BufferSize);
    }
    let len = pdu.encode(&mut buf[1..])?;
    if buf.len() < len + 3 {
        return Err(Error::BufferSize);
    }
    buf[0] = hdr.slave;
    let crc = crc16(&buf[0..=len]);
    BigEndian::write_u16(&mut buf[len + 1..], crc);
    Ok(len + 3)
}
