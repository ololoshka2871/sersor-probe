pub use serde::ser::{Serialize, SerializeTuple, Serializer};
pub use usbd_hid::descriptor::{AsInputReport, SerializedDescriptor};

use usbd_hid_macros::gen_hid_descriptor;

// 0x06, 0x00, 0xff,              // Usage Page (Vendor Defined Page 1)  0
// 0x09, 0x01,                    // Usage (Vendor Usage 1)              3
// 0xa1, 0x02,                    // Collection (Logical)                5
// 0x09, 0x3a,                    //  Usage (Vendor Usage 0x3a)          7
// 0x85, 0xa1,                    //  Report ID (161)                    9
// 0x15, 0x00,                    //  Logical Minimum (0)                11
// 0x26, 0xff, 0x00,              //  Logical Maximum (255)              13
// 0x75, 0x08,                    //  Report Size (8)                    16
// 0x95, 0x3f,                    //  Report Count (63)                  18
// 0xb1, 0x02,                    //  Feature (Data,Var,Abs)             20
// 0x09, 0x3a,                    //  Usage (Vendor Usage 0x3a)          22
// 0x85, 0xc0,                    //  Report ID (192)                    24
// 0x95, 0x3c,                    //  Report Count (60)                  26
// 0xb1, 0x02,                    //  Feature (Data,Var,Abs)             28
// 0x09, 0x3a,                    //  Usage (Vendor Usage 0x3a)          30
// 0x85, 0xc2,                    //  Report ID (194)                    32
// 0x95, 0x04,                    //  Report Count (4)                   34
// 0xb1, 0x02,                    //  Feature (Data,Var,Abs)             36
// 0x09, 0x01,                    //  Usage (Vendor Usage 1)             38
// 0x85, 0xd0,                    //  Report ID (208)                    40
// 0x95, 0x3f,                    //  Report Count (63)                  42
// 0x91, 0x02,                    //  Output (Data,Var,Abs)              44
// 0x09, 0x01,                    //  Usage (Vendor Usage 1)             46
// 0x85, 0xd0,                    //  Report ID (208)                    48
// 0x81, 0x02,                    //  Input (Data,Var,Abs)               50
// 0xc0,                          // End Collection                      52
#[gen_hid_descriptor(
    (collection = LOGICAL, usage_page = VENDOR_DEFINED_START, usage = 0x01) = {
        // Feature 0xA1 - ControlReport 
        (usage = 0x3A, report_id = 0xA1) = {
            #[item_settings data,variable,absolute] control_report=feature;
        };
        // Feature 0xC0 - I2CStatusReport
        (usage = 0x3A, report_id = 0xC0) = {
            #[item_settings data,variable,absolute] i2c_status_report=feature;
        };
        // Data report 0xC2 - I2CReadRequest
        // Суть в том, что в дискрипторе указано что это Feature? а в документации
        // указано, что Output. Это запрос произвезти чтение, данные будут в i2c_read_data
        (usage = 0x3A, report_id = 0xC2) = {
            #[item_settings data,variable,absolute] i2c_read_request=feature;
        };
        // Data report 0xD0 - I2CWriteRequest
        // на самом деле с 0xD0 по 0xDF, но для упрощенной версии и так должно быть норм
        (usage = 0x01, report_id = 0xD0) = {
            #[item_settings data,variable,absolute] i2c_write_request=output;
        };
        (usage = 0x01, report_id = 0xD0) = {
            #[item_settings data,variable,absolute] i2c_read_data=input;
        };
    }
)]
// AN_394_User_Guide_for_FT260.pdf
#[allow(dead_code)]
pub struct FT260HidDescriptor {
    pub control_report: [u8; 0x3F], // 4.4.2+
    pub i2c_status_report: [u8; 0x3c], // 4.5.1
    pub i2c_read_request: [u8; 4], // 4.5.3

    pub i2c_write_request: [u8; 0x3f], // 4.5.2
    pub i2c_read_data: [u8; 0x3f],
}
