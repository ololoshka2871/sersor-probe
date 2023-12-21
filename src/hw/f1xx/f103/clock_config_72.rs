#![allow(dead_code)]

use stm32f1xx_hal::{
    device::rcc::cfgr::PLLXTPRE_A,
    rcc::{AdcPre, HPre, PPre, UsbPre},
};

// /PD *M /AD
pub(crate) const PLL_P_DIV: PLLXTPRE_A = PLLXTPRE_A::Div2;
pub(crate) const PLL_MUL: u32 = 9;
pub(crate) const APB1_DEVIDER: PPre = PPre::Div2;
pub(crate) const APB2_DEVIDER: PPre = PPre::Div1;

pub(crate) const AHB_DEVIDER: HPre = HPre::Div1;
pub(crate) const USB_DEVIDER: UsbPre = UsbPre::Div15;
pub(crate) const ADC_DEVIDER: AdcPre = AdcPre::Div8;
