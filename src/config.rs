//-----------------------------------------------------------------------------

pub const XTAL_FREQ: u32 = 16_000_000;

//-----------------------------------------------------------------------------

// usb pull up
pub const USB_PULLUP_ACTVE_LEVEL: bool = false;

//-----------------------------------------------------------------------------

pub const SYSTICK_RATE_HZ: u32 = 1_000;

//-----------------------------------------------------------------------------

pub const HID_I2C_POLL_INTERVAL_MS: u8 = 10;

//-----------------------------------------------------------------------------

pub const I2C_ADDR_MIN: u8 = 0x0;
pub const I2C_ADDR_MIN_MAX: u8 = 0x7F;

pub const I2C_ERROR_MAX_COUNT: u8 = 3;

pub const CDC_ACM_MAX_PACKET_SIZE: u16 = 64;

//-----------------------------------------------------------------------------

pub const CURRENT_READ_INTERVAL_MS: u64 = 500;
pub const SCREENSAVER_TIMEOUT_MS: u64 = 10_000;
pub const CURRENT_THRESHOLD_MA: f32 = 0.1;

pub const R_SUNT_OM: f32 = 0.1;

// [Calibration Example 1](https://cdn-shop.adafruit.com/datasheets/ina219.pdf?p=17)
// Espected current in mA
// CURRENT_EXPECTED_A = 0.1;
// Minimum_LSB = Max_Expected_I/32767 = 3.05-6
// Maximum_LSB = Max_Expected_I/4069 = 24.41e-6
// LSB -> 10e-6
pub const LSB: f32 = 10e-6;

//-----------------------------------------------------------------------------

pub const MODBUS_RESP_TIMEOUT_MS: u64 = 50;

//-----------------------------------------------------------------------------

pub const HEAP_SIZE: usize = 8192;
