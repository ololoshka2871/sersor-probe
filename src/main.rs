#![no_main]
#![no_std]
#![feature(macro_metavar_expr)]

mod config;
mod hw;
mod support;

use defmt_rtt as _; // global logger
use panic_probe as _;

use rtic::app;

use stm32f1xx_hal::afio::AfioExt;
use stm32f1xx_hal::dma::DmaExt;
use stm32f1xx_hal::flash::FlashExt;
use stm32f1xx_hal::gpio::{
    Floating, GpioExt, Input, Output, PushPull, PA0, PA1, PA2, PA3, PA4, PA5, PA6, PA7, PA9, PB3,
    PB4, PB5, PB6, PC13, PC14, PC15,
};
use stm32f1xx_hal::prelude::*;
use stm32f1xx_hal::rcc::{HPre, PPre};
use stm32f1xx_hal::time::Hertz;
use stm32f1xx_hal::timer::{PwmChannel, Timer};
use stm32f1xx_hal::usb::{Peripheral, UsbBus, UsbBusType};

use stm32f1xx_hal::dma::dma1;
use stm32f1xx_hal::pac::{Interrupt, TIM1, TIM2, TIM4};

use usb_device::prelude::{UsbDevice, UsbDeviceBuilder};

use usbd_serial::SerialPort;

use systick_monotonic::Systick;

use support::clocking::{ClockConfigProvider, MyConfig};

use hw::{ADC_DEVIDER, AHB_DEVIDER, APB1_DEVIDER, APB2_DEVIDER, PLL_MUL, PLL_P_DIV, USB_DEVIDER};

//-----------------------------------------------------------------------------

struct HighPerformanceClockConfigProvider;

impl HighPerformanceClockConfigProvider {
    fn ahb_dev2val(ahb_dev: HPre) -> u32 {
        match ahb_dev {
            HPre::Div1 => 1,
            HPre::Div2 => 2,
            HPre::Div4 => 4,
            HPre::Div8 => 8,
            HPre::Div16 => 16,
            HPre::Div64 => 64,
            HPre::Div128 => 128,
            HPre::Div256 => 256,
            HPre::Div512 => 512,
        }
    }

    fn apb_dev2val(apb_dev: PPre) -> u32 {
        match apb_dev {
            PPre::Div1 => 1,
            PPre::Div2 => 2,
            PPre::Div4 => 4,
            PPre::Div8 => 8,
            PPre::Div16 => 16,
        }
    }

    fn pll_mul_bits(mul: u32) -> u8 {
        (mul - 2) as u8
    }

    fn ppl_div2val(div: stm32f1xx_hal::device::rcc::cfgr::PLLXTPRE_A) -> u32 {
        match div {
            stm32f1xx_hal::device::rcc::cfgr::PLLXTPRE_A::Div1 => 1,
            stm32f1xx_hal::device::rcc::cfgr::PLLXTPRE_A::Div2 => 2,
        }
    }

    fn freeze(_acr: &mut stm32f1xx_hal::flash::ACR) -> stm32f1xx_hal::rcc::Clocks {
        use stm32f1xx_hal::time::MHz;

        let cfg = Self::to_config();

        let clocks = cfg.get_clocks();
        // adjust flash wait states
        let acr = unsafe { &*stm32f1xx_hal::device::FLASH::ptr() };
        unsafe {
            acr.acr.write(|w| {
                w.latency().bits(if clocks.sysclk() <= MHz(24) {
                    0b000
                } else if clocks.sysclk() <= MHz(48) {
                    0b001
                } else {
                    0b010
                })
            })
        }

        let rcc = unsafe { &*stm32f1xx_hal::device::RCC::ptr() };

        if cfg.hse.is_some() {
            // enable HSE and wait for it to be ready

            rcc.cr.modify(|_, w| w.hseon().set_bit());

            while rcc.cr.read().hserdy().bit_is_clear() {}
        }

        if let Some(pllmul_bits) = cfg.pllmul {
            // enable PLL and wait for it to be ready

            #[allow(unused_unsafe)]
            rcc.cfgr
                .modify(|_, w| unsafe { w.pllxtpre().variant(PLL_P_DIV) });

            #[allow(unused_unsafe)]
            rcc.cfgr.modify(|_, w| unsafe {
                w.pllmul().bits(pllmul_bits).pllsrc().bit(cfg.hse.is_some())
            });

            rcc.cr.modify(|_, w| w.pllon().set_bit());

            while rcc.cr.read().pllrdy().bit_is_clear() {}
        }

        rcc.cfgr.modify(|_, w| unsafe {
            w.adcpre().variant(cfg.adcpre);
            w.ppre2()
                .bits(cfg.ppre2 as u8)
                .ppre1()
                .bits(cfg.ppre1 as u8)
                .hpre()
                .bits(cfg.hpre as u8)
                .usbpre()
                .variant(cfg.usbpre)
                .sw()
                .bits(if cfg.pllmul.is_some() {
                    // PLL
                    0b10
                } else if cfg.hse.is_some() {
                    // HSE
                    0b1
                } else {
                    // HSI
                    0b0
                })
        });

        clocks
    }
}

impl ClockConfigProvider for HighPerformanceClockConfigProvider {
    fn core_frequency() -> Hertz {
        let f = crate::config::XTAL_FREQ / Self::ppl_div2val(PLL_P_DIV) * PLL_MUL
            / Self::ahb_dev2val(AHB_DEVIDER);
        Hertz::Hz(f)
    }

    fn apb1_frequency() -> Hertz {
        Hertz::Hz(Self::core_frequency().to_Hz() / Self::apb_dev2val(APB1_DEVIDER))
    }

    fn apb2_frequency() -> Hertz {
        Hertz::Hz(Self::core_frequency().to_Hz() / Self::apb_dev2val(APB2_DEVIDER))
    }

    // stm32_cube: if APB devider > 1, timers freq APB*2
    fn master_counter_frequency() -> Hertz {
        // TIM3 - APB1
        if APB2_DEVIDER == PPre::Div1 {
            Self::core_frequency()
        } else {
            Self::core_frequency() * 2
        }
    }

    fn xtal2master_freq_multiplier() -> f32 {
        PLL_MUL as f32
            / (Self::ppl_div2val(PLL_P_DIV)
                * Self::ahb_dev2val(AHB_DEVIDER)
                * Self::apb_dev2val(APB2_DEVIDER)) as f32
    }

    fn to_config() -> MyConfig {
        MyConfig {
            hse_p_div: PLL_P_DIV,
            hse: Some(crate::config::XTAL_FREQ),
            pllmul: Some(Self::pll_mul_bits(PLL_MUL)),
            hpre: AHB_DEVIDER,
            ppre1: APB1_DEVIDER,
            ppre2: APB2_DEVIDER,
            usbpre: USB_DEVIDER,
            adcpre: ADC_DEVIDER,
        }
    }
}

//-----------------------------------------------------------------------------

#[app(device = stm32f1xx_hal::pac, peripherals = true, dispatchers = [RTCALARM])]
mod app {
    use embedded_hal::blocking::serial;

    use super::*;

    #[shared]
    struct Shared {
        usb_device: UsbDevice<'static, UsbBusType>,
        serial1: SerialPort<'static, UsbBus<Peripheral>>,
        serial2: SerialPort<'static, UsbBus<Peripheral>>,
        //gcode_queue: heapless::Deque<gcode::GCode, { config::GCODE_QUEUE_SIZE }>,
        //request_queue: heapless::Deque<gcode::Request, { config::GCODE_QUEUE_SIZE }>,
    }

    #[local]
    struct Local {}

    #[monotonic(binds = SysTick, default = true)]
    type MonoTimer = Systick<{ config::SYSTICK_RATE_HZ }>;

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local, init::Monotonics) {
        static mut USB_BUS: Option<usb_device::bus::UsbBusAllocator<UsbBusType>> = None;

        let mut flash = ctx.device.FLASH.constrain();

        let dma_channels = ctx.device.DMA1.split();

        let mut gpioa = ctx.device.GPIOA.split();
        let mut gpiob = ctx.device.GPIOB.split();
        let mut gpioc = ctx.device.GPIOC.split();

        let mut afio = ctx.device.AFIO.constrain();
        let (pa15, pb3, pb4) = afio.mapr.disable_jtag(gpioa.pa15, gpiob.pb3, gpiob.pb4);

        let mut usb_pull_up = pa15.into_push_pull_output_with_state(
            &mut gpioa.crh,
            if !config::USB_PULLUP_ACTVE_LEVEL {
                stm32f1xx_hal::gpio::PinState::High
            } else {
                stm32f1xx_hal::gpio::PinState::Low
            },
        );

        let clocks = HighPerformanceClockConfigProvider::freeze(&mut flash.acr);
        defmt::info!("Clocks: {}", defmt::Debug2Format(&clocks));

        let mono = Systick::new(ctx.core.SYST, clocks.sysclk().to_Hz());

        let usb = Peripheral {
            usb: ctx.device.USB,
            pin_dm: gpioa.pa11,
            pin_dp: gpioa.pa12,
        };

        unsafe {
            USB_BUS.replace(UsbBus::new(usb));
        }

        let serial1 = SerialPort::new(unsafe { USB_BUS.as_ref().unwrap_unchecked() });
        let serial2 = SerialPort::new(unsafe { USB_BUS.as_ref().unwrap_unchecked() });

        let usb_dev = UsbDeviceBuilder::new(
            unsafe { USB_BUS.as_ref().unwrap_unchecked() },
            usb_device::prelude::UsbVidPid(0x16c0, 0x394f),
        )
        .manufacturer("SCTBElpa")
        .product("SensorProbe")
        .serial_number(stm32_device_signature::device_id_hex())
        .composite_with_iads()
        .build();

        defmt::info!("USB device");

        //---------------------------------------------------------------------

        // UART1
        let _uart1 = stm32f1xx_hal::serial::Serial::new(
            ctx.device.USART1,
            (
                gpiob.pb6.into_alternate_push_pull(&mut gpiob.crl),
                gpiob.pb7,
            ),
            &mut afio.mapr,
            stm32f1xx_hal::serial::Config::default().baudrate(9_600.bps()),
            &clocks,
        );

        //_uart1.reconfigure(config, clocks)

        // UART2
        let _uart2 = stm32f1xx_hal::serial::Serial::new(
            ctx.device.USART2,
            (
                gpioa.pa2.into_alternate_push_pull(&mut gpioa.crl),
                gpioa.pa3,
            ),
            &mut afio.mapr,
            stm32f1xx_hal::serial::Config::default().baudrate(9_600.bps()),
            &clocks,
        );

        defmt::info!("Serial ports");

        //---------------------------------------------------------------------

        usb_pull_up.toggle(); // enable USB
        defmt::info!("USB enabled");

        //---------------------------------------------------------------------

        (
            Shared {
                usb_device: usb_dev,
                serial1,
                serial2,
                //gcode_queue: heapless::Deque::new(),
                //request_queue: heapless::Deque::new(),
            },
            Local {},
            init::Monotonics(mono),
        )
    }

    //-------------------------------------------------------------------------

    #[task(binds = USB_HP_CAN_TX, shared = [usb_device, serial1, serial2], priority = 1)]
    fn usb_tx(ctx: usb_tx::Context) {
        let mut usb_device = ctx.shared.usb_device;
        let mut serial1 = ctx.shared.serial1;
        let mut serial2 = ctx.shared.serial2;

        if !(&mut usb_device, &mut serial1, &mut serial2)
            .lock(move |usb_device, serial1, serial2| super::usb_poll(usb_device, serial1, serial2))
        {
            cortex_m::peripheral::NVIC::mask(Interrupt::USB_HP_CAN_TX);
        }
    }

    #[task(binds = USB_LP_CAN_RX0, shared = [usb_device, serial1, serial2], priority = 1)]
    fn usb_rx0(ctx: usb_rx0::Context) {
        let mut usb_device = ctx.shared.usb_device;
        let mut serial1 = ctx.shared.serial1;
        let mut serial2 = ctx.shared.serial2;

        if !(&mut usb_device, &mut serial1, &mut serial2)
            .lock(move |usb_device, serial1, serial2| super::usb_poll(usb_device, serial1, serial2))
        {
            cortex_m::peripheral::NVIC::mask(Interrupt::USB_LP_CAN_RX0);
        }
    }

    //-------------------------------------------------------------------------

    #[idle(shared=[], local = [])]
    fn idle(ctx: idle::Context) -> ! {
        //let mut serial = ctx.shared.serial;

        loop {
            cortex_m::asm::wfi();
            unsafe {
                cortex_m::peripheral::NVIC::unmask(Interrupt::USB_HP_CAN_TX);
                cortex_m::peripheral::NVIC::unmask(Interrupt::USB_LP_CAN_RX0);
            }
        }
    }
}

fn usb_poll<B: usb_device::bus::UsbBus>(
    usb_dev: &mut usb_device::prelude::UsbDevice<'static, B>,
    serial1: &mut usbd_serial::SerialPort<'static, B>,
    serial2: &mut usbd_serial::SerialPort<'static, B>,
) -> bool
where
    B: usb_device::bus::UsbBus,
{
    if !usb_dev.poll(&mut [serial1, serial2]) {
        return true;
    }

    let mut buf = [0u8; 64];

    //let ctrl = serial1.line_coding(); // port config
    // read from serial1
    match serial1.read(&mut buf) {
        Ok(count) if count > 0 => {
            if let Ok(result) = core::str::from_utf8(&buf[..count]) {
                defmt::debug!("serial1: {} ({} bytes)", result, count);
            } else {
                defmt::debug!("serial1: {:?} ({} bytes)", &buf[..count], count);
            }
        }
        _ => {}
    }

    // read from serial2
    match serial2.read(&mut buf) {
        Ok(count) if count > 0 => {
            if let Ok(result) = core::str::from_utf8(&buf[..count]) {
                defmt::debug!("serial2: {} ({} bytes)", result, count);
            } else {
                defmt::debug!("serial2: {:?} ({} bytes)", &buf[..count], count);
            }
        }
        _ => {}
    }

    false
}
