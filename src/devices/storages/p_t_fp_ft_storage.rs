use alloc::string::String;
use tbytes::TBytesReaderFor;

use crate::devices::ValuesStorage;

#[derive(defmt_macros::Format, Copy, Clone, Default)]
pub struct PTFpFtstorage {
    pub p: f32,
    pub t: f32,
    pub fp: f32,
    pub ft: f32,
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

    fn render(&self) {
        defmt::trace!("ValuesStorage::render()");
    }

    fn print(&self) -> String {
        use crate::support::format_float_simple as f;
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
        write!(s, "}}").ok();
        s
    }
}
