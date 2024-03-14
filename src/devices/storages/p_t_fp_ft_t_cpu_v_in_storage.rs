use alloc::vec::Vec;
use alloc::{format, string::String};

use crate::devices::ValuesStorage;

use crate::support::format_float_simple as f;

#[derive(defmt_macros::Format, Copy, Clone)]
pub struct PTFpFtTCpuVInStorage {
    pub sender: &'static str,
    pub data: [f32; 6],
    pub offset: usize,
}

impl PTFpFtTCpuVInStorage {
    pub fn new(sender: &'static str) -> Self {
        Self {
            sender,
            data: [0.0; 6],
            offset: 0,
        }
    }
}

impl ValuesStorage for PTFpFtTCpuVInStorage {
    fn size(&self) -> usize {
        6 * core::mem::size_of::<f32>()
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
            "P={}, T={}, Fp={}, Ft={}, Tcpu={}, Vin={}",
            f(self.data[0], 2),
            f(self.data[1], 2),
            f(self.data[2], 2),
            f(self.data[3], 2),
            f(self.data[4], 2),
            f(self.data[5], 2)
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
            ),
            format!(
                "Tcpu={:>w$}",
                f(self.data[4], 1),
                w = field_width as usize - 5
            ),
            format!(
                "Vin={:>w$}",
                f(self.data[5], 1),
                w = field_width as usize - 4
            ),
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
