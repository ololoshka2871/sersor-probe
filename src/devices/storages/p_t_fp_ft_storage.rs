use alloc::vec::Vec;
use alloc::{format, string::String};
use tbytes::TBytesReaderFor;

use crate::devices::ValuesStorage;

use crate::support::format_float_simple as f;

#[derive(defmt_macros::Format, Copy, Clone)]
pub struct PTFpFtstorage {
    pub sender: &'static str,
    pub p: f32,
    pub t: f32,
    pub fp: f32,
    pub ft: f32,
}

impl PTFpFtstorage {
    pub fn new(sender: &'static str) -> Self {
        Self {
            sender,
            p: 0.0,
            t: 0.0,
            fp: 0.0,
            ft: 0.0,
        }
    }
}

impl ValuesStorage for PTFpFtstorage {
    fn size(&self) -> usize {
        4 * core::mem::size_of::<f32>()
    }

    fn copy_from(&mut self, src: &[u8]) {
        assert!(src.len() >= self.size());
        let reader = tbytes::TBytesReader::from(src);
        unsafe {
            self.p = reader.read().unwrap_unchecked();
            self.t = reader.read().unwrap_unchecked();
            self.fp = reader.read().unwrap_unchecked();
            self.ft = reader.read().unwrap_unchecked();
        }
    }

    fn sender_id(&self) -> String {
        format!("0x{}", self.sender)
    }

    fn print(&self) -> String {
        use core::fmt::Write;

        let mut s = String::new();
        write!(s, "{{ ").ok();
        write!(
            s,
            "P={}, T={}, Fp={}, Ft={}",
            f(self.p, 2),
            f(self.t, 2),
            f(self.fp, 2),
            f(self.ft, 2)
        )
        .ok();
        write!(s, " }}").ok();
        s
    }

    fn render(&self, field_width: u32) -> Vec<String> {
        alloc::vec![
            format!("P={:>w$}", f(self.p, 2), w = field_width as usize - 2),
            format!("T={:>w$}", f(self.t, 2), w = field_width as usize - 2),
            format!("Fp={:>w$}", f(self.fp, 1), w = field_width as usize - 3),
            format!("Ft={:>w$}", f(self.ft, 1), w = field_width as usize - 3)
        ]
    }
}
