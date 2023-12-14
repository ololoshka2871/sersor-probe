use stm32f1xx_hal::{device::rcc::cfgr::PLLXTPRE_A, prelude::_fugit_RateExtU32, time::Hertz};

pub(crate) trait ClockConfigProvider {
    fn core_frequency() -> Hertz;
    fn apb1_frequency() -> Hertz;
    fn apb2_frequency() -> Hertz;
    fn master_counter_frequency() -> Hertz;
    fn xtal2master_freq_multiplier() -> f32;
    fn to_config() -> MyConfig;
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct MyConfig {
    pub hse_p_div: PLLXTPRE_A,
    pub hse: Option<u32>,
    pub pllmul: Option<u8>,
    pub hpre: stm32f1xx_hal::rcc::HPre,
    pub ppre1: stm32f1xx_hal::rcc::PPre,
    pub ppre2: stm32f1xx_hal::rcc::PPre,
    pub usbpre: stm32f1xx_hal::rcc::UsbPre,
    pub adcpre: stm32f1xx_hal::rcc::AdcPre,
}

// stm32f1xx_hal::rcc::Clocks
#[allow(dead_code)]
pub(crate) struct Clocks {
    hclk: Hertz,
    pclk1: Hertz,
    pclk2: Hertz,
    ppre1: u8,
    ppre2: u8,
    sysclk: Hertz,
    adcclk: Hertz,
    usbclk_valid: bool,
}

impl MyConfig {
    pub(crate) fn get_clocks(&self) -> stm32f1xx_hal::rcc::Clocks {
        //stm32f1xx_hal::rcc::HSI
        const HSI: u32 = 8_000_000; // Hz

        let sysclk = if let Some(pllmul_bits) = self.pllmul {
            let pllsrcclk = if let Some(hse) = self.hse {
                match self.hse_p_div {
                    PLLXTPRE_A::DIV1 => hse,
                    PLLXTPRE_A::DIV2 => hse / 2,
                }
            } else {
                HSI / 2
            };
            pllsrcclk * (pllmul_bits as u32 + 2)
        } else if let Some(hse) = self.hse {
            hse
        } else {
            HSI
        };

        let hclk = if self.hpre as u8 >= 0b1100 {
            sysclk / (1 << (self.hpre as u8 - 0b0110))
        } else {
            sysclk / (1 << (self.hpre as u8 - 0b0111))
        };

        let ppre1 = 1 << (self.ppre1 as u8 - 0b011);
        let pclk1 = hclk / (ppre1 as u32);

        let ppre2 = 1 << (self.ppre2 as u8 - 0b011);
        let pclk2 = hclk / (ppre2 as u32);

        let apre = (self.adcpre as u8 + 1) << 1;
        let adcclk = pclk2 / (apre as u32);

        // the USB clock is only valid if an external crystal is used, the PLL is enabled, and the
        // PLL output frequency is a supported one.
        #[cfg(any(feature = "stm32f103", feature = "connectivity"))]
        let usbclk_valid = matches!(
            (self.hse, self.pllmul, sysclk),
            (Some(_), Some(_), 72_000_000) | (Some(_), Some(_), 48_000_000)
        );

        assert!(
            sysclk <= 72_000_000
                && hclk <= 72_000_000
                && pclk1 <= 36_000_000
                && pclk2 <= 72_000_000
                && adcclk <= 14_000_000
        );

        unsafe {
            core::mem::transmute(Clocks {
                hclk: hclk.Hz(),
                pclk1: pclk1.Hz(),
                pclk2: pclk2.Hz(),
                ppre1,
                ppre2,
                sysclk: sysclk.Hz(),
                adcclk: adcclk.Hz(),
                usbclk_valid,
            })
        }
    }
}
