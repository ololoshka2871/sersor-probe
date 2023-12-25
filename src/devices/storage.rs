use core::fmt::Write;

use crate::config::HlString;

use super::ValuesStorage;

impl<const N: usize> ValuesStorage for heapless::Vec<u8, N> {
    fn size(&self) -> u8 {
        self.len() as u8
    }

    fn copy_from(&mut self, src: &[u8]) {
        self.copy_from_slice(src)
    }

    fn render(&self) {
        defmt::trace!("ValuesStorage::render()");
    }

    fn print(&self) -> HlString {
        let mut s = HlString::new();
        write!(s, "ValuesStorage {{ ").ok();
        for b in self.iter() {
           write!(s, "{:X} ", b).unwrap();
        }
        write!(s, "}}").ok();
        s
    }
}
