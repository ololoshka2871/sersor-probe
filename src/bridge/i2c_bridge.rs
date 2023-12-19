enum FeatureType {
    Control = 0xA1,
    ReadRequest = 0xC2,
}

enum ControlReportType {
    I2CReset = 0x20,
    SetI2CSpeed = 0x22,
}

enum DataReportType {
    Status = 0xC0,
    Data = 0xD0,
}

pub struct I2CBridge {}

impl I2CBridge {
    pub fn new() -> Self {
        Self {}
    }

    pub fn feature_request(&mut self, report_id: u8, buf: &[u8]) {
        match report_id {
            r if r == FeatureType::Control as u8 => {
                match buf[0] {
                    c if c == ControlReportType::I2CReset as u8 => {
                        defmt::debug!("I2C Reset");
                    }
                    c if c == ControlReportType::SetI2CSpeed as u8 => {
                        let speed_khz = u16::from_le_bytes([buf[2], buf[1]]);
                        defmt::debug!("I2C Set speed = {} kHz", speed_khz);
                    }
                    c => {
                        defmt::error!("Unknown HID Control report type 0x{:X}", c)
                    }
                }
            }
            r if r == FeatureType::ReadRequest as u8 => {
                defmt::debug!("HID Read request report {}", buf);
            }
            r => {
                defmt::error!("Unknown HID Feature report id={} buf={}", r, buf)
            }
        }
    }

    pub fn data_request(&mut self, buf: &[u8]) {
        defmt::debug!("HID output data: {}", buf);
    }
}
