use alloc::vec::Vec;

use crate::bridge::BufferTrait;

pub struct Buffer<const SIZE: usize> {
    buffer: [u8; SIZE],
    offset: usize,
}

impl<const SIZE: usize> Buffer<SIZE> {
    pub fn new() -> Self {
        Self {
            buffer: [0; SIZE],
            offset: 0,
        }
    }
}

impl<const SIZE: usize> BufferTrait for Buffer<SIZE> {
    fn reset(&mut self) {
        self.offset = 0;
    }

    fn len(&self) -> usize {
        self.offset
    }

    fn is_empty(&self) -> bool {
        self.offset == 0
    }

    fn is_full(&self) -> bool {
        self.offset == SIZE
    }

    fn as_slice(&self) -> &[u8] {
        &self.buffer[..self.offset]
    }

    fn write_me(&mut self) -> &mut [u8] {
        &mut self.buffer[self.offset..]
    }

    fn add_offset(&mut self, size: usize) {
        self.offset += size;
    }

    fn feed_byte(&mut self, byte: u8) -> bool {
        if self.offset < SIZE {
            self.buffer[self.offset] = byte;
            self.offset += 1;
            true
        } else {
            false
        }
    }
}

//----------------------------------------------------------------

pub struct VecBuffer {
    buffer: Vec<u8>,
    offset: usize,
}

impl VecBuffer {
    pub fn new(vec: Vec<u8>) -> Self {
        Self {
            buffer: vec,
            offset: 0,
        }
    }
}

impl From<Vec<u8>> for VecBuffer {
    fn from(vec: Vec<u8>) -> Self {
        Self::new(vec)
    }
}

impl BufferTrait for VecBuffer {
    fn reset(&mut self) {
        self.offset = 0;
    }

    fn len(&self) -> usize {
        self.offset
    }

    fn is_empty(&self) -> bool {
        self.offset == 0
    }

    fn is_full(&self) -> bool {
        self.offset == self.buffer.len()
    }

    fn as_slice(&self) -> &[u8] {
        &self.buffer[..self.offset]
    }

    fn write_me(&mut self) -> &mut [u8] {
        &mut self.buffer[self.offset..]
    }

    fn add_offset(&mut self, size: usize) {
        self.offset += size;
    }

    fn feed_byte(&mut self, byte: u8) -> bool {
        if self.offset < self.buffer.len() {
            self.buffer[self.offset] = byte;
            self.offset += 1;
            true
        } else {
            false
        }
    }
}
