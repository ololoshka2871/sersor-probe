use alloc::vec::Vec;
use alloc::{format, string::String};
use tbytes::TBytesReaderFor;

use crate::devices::ValuesStorage;

use crate::support::format_float_simple as f;

#[derive(defmt_macros::Format, Copy, Clone)]
pub struct PTFpFtstorage {
    pub sender: &'static str,
    pub data: [f32; 4],
}

impl PTFpFtstorage {
    pub fn new(sender: &'static str) -> Self {
        Self {
            sender,
            data: [0.0; 4],
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
            for i in 0..4 {
                self.data[i] = reader.read().unwrap_unchecked();
            }
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
            f(self.data[0], 2),
            f(self.data[1], 2),
            f(self.data[2], 2),
            f(self.data[3], 2)
        )
        .ok();
        write!(s, " }}").ok();
        s
    }

    fn render(&self, field_width: u32) -> Vec<String> {
        alloc::vec![
            format!("P={:>w$}", f(self.data[0], 2), w = field_width as usize - 2),
            format!("T={:>w$}", f(self.data[1], 2), w = field_width as usize - 2),
            format!(
                "Fp={:>w$}",
                f(self.data[2], 1),
                w = field_width as usize - 3
            ),
            format!(
                "Ft={:>w$}",
                f(self.data[3], 1),
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
}
