use alloc::vec::Vec;
use alloc::{format, string::String};

use crate::devices::ValuesStorage;

use crate::support::format_float_simple as f;

#[derive(defmt_macros::Format, Copy, Clone)]
pub struct TFtstorage {
    pub sender: &'static str,
    pub data: [f32; 2],
    pub offset: usize,
}

impl TFtstorage {
    pub fn new(sender: &'static str) -> Self {
        Self {
            sender,
            data: [0.0; 2],
            offset: 0,
        }
    }
}

impl ValuesStorage for TFtstorage {
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
            "T={}, Ft={}",
            f(self.data[0], 2),
            f(self.data[1], 2),
        )
        .ok();
        write!(s, " }}").ok();
        s
    }

    fn render(&self, field_width: u32) -> Vec<String> {
        alloc::vec![
            format!("T={:>w$}", f(self.data[0], 2), w = field_width as usize - 2),
            format!(
                "Ft={:>w$}",
                f(self.data[1], 1),
                w = field_width as usize - 3
            )
        ]
    }

    fn as_mut_slice(&mut self) -> &mut [u8] {
        unsafe {
            core::slice::from_raw_parts_mut(
                &mut self.data as *mut f32 as *mut u8,
                self.size(),
            )
        }
    }

    fn write_bytes(&mut self, buf: &[u8]) {
        super::write_bytes_common(&mut self.data, &mut self.offset, buf);
    }

    fn write_f32(&mut self, value: f32) {
        super::write_f32_common(&mut self.data, &mut self.offset, value);
    }
}
