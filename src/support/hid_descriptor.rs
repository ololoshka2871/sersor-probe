pub use serde::ser::{Serialize, SerializeTuple, Serializer};
pub use usbd_hid::descriptor::{AsInputReport, SerializedDescriptor};

use usbd_hid_macros::gen_hid_descriptor;

pub const HID_REPORT_SIZE: usize = 64;

#[gen_hid_descriptor(
    (collection = LOGICAL, usage_page = VENDOR_DEFINED_START, usage = 0x01) = {
        (usage = 0x01, usage_min = 0, usage_max = 512) = { i2c_read=input; };
        (usage = 0x02, usage_min = 0, usage_max = 512) = { i2c_write=output; };
    }
)]
#[allow(dead_code)]
pub struct HidDescriptor {
    // тут нельзя использовать [u8; HID_REPORT_SIZE], макрос ломается
    pub i2c_write: [u8; 64],
    pub i2c_read: [u8; 64],
}
