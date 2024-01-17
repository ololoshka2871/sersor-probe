use modbus_core::rtu::{RequestAdu, ResponseAdu};

pub trait RxBuffer {
    fn try_decode_request(&self) -> Option<RequestAdu>;
    fn try_decode_response(&self) -> Option<ResponseAdu>;
}

pub trait BufferTrait {
    fn reset(&mut self);
    fn len(&self) -> usize;
    fn is_empty(&self) -> bool;
    fn is_full(&self) -> bool;
    fn as_slice(&self) -> &[u8];
    fn write_me(&mut self) -> &mut [u8];
    fn add_offset(&mut self, size: usize);
    fn feed_byte(&mut self, byte: u8) -> bool;
}
