#[cfg(feature = "stm32f1xx")]
pub mod f1xx_i2c_error;

#[cfg(feature = "stm32f103")]
mod f103;
#[cfg(feature = "stm32f103")]
pub(crate) use f103::*;