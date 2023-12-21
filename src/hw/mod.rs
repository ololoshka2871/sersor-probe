pub mod i2c_wraper;
pub use i2c_wraper::*;

#[cfg(feature = "stm32f1xx")]
pub mod f1xx;
#[cfg(feature = "stm32f1xx")]
pub use f1xx::*;