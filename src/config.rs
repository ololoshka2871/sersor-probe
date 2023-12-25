//-----------------------------------------------------------------------------

pub const XTAL_FREQ: u32 = 16_000_000;

//-----------------------------------------------------------------------------

// usb pull up
pub const USB_PULLUP_ACTVE_LEVEL: bool = false;

//-----------------------------------------------------------------------------

pub const STR_MAX_LEN: usize = 64;

pub const MAX_I2C_SEQUENCE_LEN: usize = 8;
pub const MAX_I2C_RX_LEN: usize = 32;

//-----------------------------------------------------------------------------

pub const SYSTICK_RATE_HZ: u32 = 1_000;

//-----------------------------------------------------------------------------

pub const HID_I2C_POLL_INTERVAL_MS: u8 = 10;

//-----------------------------------------------------------------------------

pub const I2C_ADDR_MIN: u8 = 0x0;
pub const I2C_ADDR_MIN_MAX: u8 = 0x7F;

pub const CDC_ACM_MAX_PACKET_SIZE: u16 = 64;

//-----------------------------------------------------------------------------

pub type HlString = heapless::String<STR_MAX_LEN>;
