use alloc::vec::Vec;
use alloc::{format, string::String};

use crate::devices::ValuesStorage;

#[derive(defmt_macros::Format, Copy, Clone)]
pub struct PTFpFtstorage {
    pub sender: &'static str,
    pub data: [f32; 4],
    pub offset: usize,
}

impl PTFpFtstorage {
    pub fn new(sender: &'static str) -> Self {
        Self {
            sender,
            data: [0.0; 4],
            offset: 0,
        }
    }
}

impl ValuesStorage for PTFpFtstorage {
    fn size(&self) -> usize {
        self.data.len() * core::mem::size_of::<f32>()
    }

    fn copy_from(&mut self, src: &[u8]) {
        super::copy_from_common(&mut self.data, src, &mut self.offset);
    }

    fn sender_id(&self) -> String {
        super::sender_id_common(self.sender)
    }

    fn print(&self) -> String {
        use core::fmt::Write;

        let mut s = String::new();
        write!(s, "{{ ").ok();
        write!(
            s,
            "P={:.2}, T={:.2}, Fp={:.2}, Ft={:.2}",
            self.data[0], self.data[1], self.data[2], self.data[3]
        )
        .ok();
        write!(s, " }}").ok();
        s
    }

    fn render(&self, field_width: u32) -> Vec<String> {
        alloc::vec![
            format!("P={:>w$.2}", self.data[0], w = field_width as usize - 2),
            format!("T={:>w$.2}", self.data[1], w = field_width as usize - 2),
            format!("Fp={:>w$.1}", self.data[2], w = field_width as usize - 3),
            format!("Ft={:>w$.1}", self.data[3], w = field_width as usize - 3)
        ]
    }

    fn as_mut_slice(&mut self) -> &mut [u8] {
        unsafe {
            core::slice::from_raw_parts_mut(&mut self.data as *mut f32 as *mut u8, self.size())
        }
    }

    fn write_bytes(&mut self, buf: &[u8]) {
        super::write_bytes_common(&mut self.data, &mut self.offset, buf);
    }

    fn write_f32(&mut self, value: f32) {
        super::write_f32_common(&mut self.data, &mut self.offset, value);
    }
}
