mod p_t_fp_ft_storage;
pub use p_t_fp_ft_storage::PTFpFtstorage;

mod p_t_fp_ft_t_cpu_v_in_storage;
pub use p_t_fp_ft_t_cpu_v_in_storage::PTFpFtTCpuVInStorage;

use alloc::{format, string::String};
use tbytes::TBytesReaderFor;

pub fn copy_from_common(dest: &mut [f32], src: &[u8], offset_for_mod: &mut usize) {
    assert!(dest.len() >= src.len() / core::mem::size_of::<f32>());
    let reader = tbytes::TBytesReader::from(src);
    unsafe {
        for i in 0..src.len() / core::mem::size_of::<f32>(){
            dest[i] = reader.read().unwrap_unchecked();
            *offset_for_mod += 1;
        }
    }
}

pub fn sender_id_common(sender: &str) -> String {
    format!("0x{}", sender)
}

pub fn write_bytes_common(buf: &mut [f32], offset: &mut usize, src: &[u8]) {
    assert!(buf.len() <= buf.len() + *offset);
    let reader = tbytes::TBytesReader::from(src);
    unsafe {
        for _ in 0..buf.len() / core::mem::size_of::<f32>() {
            buf[*offset] = reader.read().unwrap_unchecked();
            *offset += 1;
        }
    }
}

pub fn write_f32_common(buf: &mut [f32], offset: &mut usize, value: f32) {
    buf[*offset] = value;
    *offset += 1;
}