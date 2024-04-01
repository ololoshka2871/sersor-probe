#![no_main]
#![no_std]
#![feature(macro_metavar_expr)]

pub mod bridge;
mod config;
mod devices;
mod display_state;
mod high_performance_clock_config_provider;
mod hw;
mod support;
mod uart_macro;

use defmt_rtt as _; // global logger
use panic_probe as _;

use rtic::app;

use stm32f1xx_hal::afio::AfioExt;
use stm32f1xx_hal::dma::DmaExt;
use stm32f1xx_hal::flash::FlashExt;
use stm32f1xx_hal::gpio::{
    Alternate, GpioExt, Input, OpenDrain, Output, PinState, PullUp, PA2, PA3, PA4, PA5, PA7, PB1,
    PB10, PB11, PB6, PB7, PB8, PB9,
};
use stm32f1xx_hal::spi::{NoMiso, Spi, Spi1NoRemap};
use stm32f1xx_hal::time::{Hertz, U32Ext};
use stm32f1xx_hal::usb::{Peripheral, UsbBus, UsbBusType};

use stm32f1xx_hal::pac::{Interrupt, I2C1, I2C2, SPI1, USART1, USART2};

use usb_device::prelude::{UsbDevice, UsbDeviceBuilder};

use usbd_serial::CdcAcmClass;

use support::MyLineCoding;

use crate::bridge::{BufferTrait, Builder, Execute, RxBuffer};

//-----------------------------------------------------------------------------

type TsensorI2c = hw::I2cWraper<I2C1, (PB8<Alternate<OpenDrain>>, PB9<Alternate<OpenDrain>>)>;

// register I2C devices
pub static I2C_DEVICES: &[&dyn devices::I2CDevice<TsensorI2c, Error = bridge::I2CBridgeError>] = &[
    &devices::DeviceDba0,
    &devices::DeviceDba2,
    &devices::DeviceC002,
];

// register Modbus devices
pub static MODBUS_DEVICES: &[&dyn devices::ModbusDevice] = &[
    &devices::DeviceDba0,
    &devices::DeviceDba2,
    &devices::DeviceC002,
];

//-----------------------------------------------------------------------------

// Global allocator
extern crate alloc;
use embedded_alloc::Heap;

#[global_allocator]
static HEAP: Heap = Heap::empty();

//-----------------------------------------------------------------------------

const UART1_BUS_BIND: &str = "UART";
const UART2_BUS_BIND: &str = "RS485";

//-----------------------------------------------------------------------------

defmt::timestamp!(
    "[T{=u64}]",
    crate::app::monotonics::MonoTimer::now().ticks()
);

//-----------------------------------------------------------------------------

#[app(device = stm32f1xx_hal::pac, peripherals = true, dispatchers = [RTCALARM, FLASH, CAN_RX1, CAN_SCE])]
mod app {
    use systick_monotonic::*;

    use super::*;

    #[shared]
    struct Shared {
        usb_device: UsbDevice<'static, UsbBusType>,
        serial1: CdcAcmClass<'static, UsbBus<Peripheral>>,
        serial2: CdcAcmClass<'static, UsbBus<Peripheral>>,
        hid_i2c: usbd_hid::hid_class::HIDClass<'static, UsbBus<Peripheral>>,

        i2c: TsensorI2c,
        display_state: display_state::DisplayState<{ config::SYSTICK_RATE_HZ }>,

        uart1: support::UartHalfDuplex<
            'B',
            5,
            USART1,
            (PB6<Alternate>, PB7<Input<PullUp>>),
            { config::SYSTICK_RATE_HZ },
        >,
        uart2: support::UartHalfDuplex<
            'A',
            1,
            USART2,
            (PA2<Alternate>, PA3<Input<PullUp>>),
            { config::SYSTICK_RATE_HZ },
        >,

        modbus1_dispatcher: bridge::ModbusDispatcher<
            { config::SYSTICK_RATE_HZ },
            { config::MODBUS_DISPATCHER_QUEUE_SIZE },
        >,
        modbus2_dispatcher: bridge::ModbusDispatcher<
            { config::SYSTICK_RATE_HZ },
            { config::MODBUS_DISPATCHER_QUEUE_SIZE },
        >,

        device_modbus_request: heapless::Deque<
            (modbus_core::rtu::RequestAdu<'static>, &'static str),
            { config::MODBUS_SELF_INQUARY_QUEUE_SIZE },
        >,
        device_modbus_response: heapless::Deque<
            (alloc::vec::Vec<u8>, &'static str),
            { config::MODBUS_SELF_INQUARY_QUEUE_SIZE },
        >,
    }

    #[local]
    struct Local {
        i2c_device:
            Option<&'static dyn devices::I2CDevice<TsensorI2c, Error = bridge::I2CBridgeError>>,
        i2c_error_count: u8,

        modbus_device: Option<&'static dyn devices::ModbusDevice>,
        modbus_error_count: u8,

        display: ssd1309::mode::GraphicsMode<
            display_interface_spi::SPIInterface<
                Spi<SPI1, Spi1NoRemap, (PA5<Alternate>, NoMiso, PA7<Alternate>), u8>,
                PB1<Output>,
                PA4<Output>,
            >,
        >,

        current_meter: support::CurrentMeter<
            I2C2,
            (PB10<Alternate<OpenDrain>>, PB11<Alternate<OpenDrain>>),
            3,
        >,
        clocks: stm32f1xx_hal::rcc::Clocks,

        uart1_mb_assembly_buffer: support::Buffer<{ bridge::MODBUS_BUFFER_SIZE_MAX }>,
        uart2_mb_assembly_buffer: support::Buffer<{ bridge::MODBUS_BUFFER_SIZE_MAX }>,

        modbus_detect_retrys: u8,
    }

    #[monotonic(binds = SysTick, default = true)]
    type MonoTimer = Systick<{ config::SYSTICK_RATE_HZ }>;

    #[init]
    fn init(mut ctx: init::Context) -> (Shared, Local, init::Monotonics) {
        use usbd_hid::descriptor::SerializedDescriptor;

        static mut USB_BUS: Option<usb_device::bus::UsbBusAllocator<UsbBusType>> = None;
        static mut HEAP_MEM: [core::mem::MaybeUninit<u8>; config::HEAP_SIZE] =
            [core::mem::MaybeUninit::uninit(); config::HEAP_SIZE];

        defmt::info!("Init...");

        ctx.core.DCB.enable_trace();
        ctx.core.DWT.enable_cycle_counter();
        defmt::info!("DWT...");

        unsafe { HEAP.init(HEAP_MEM.as_ptr() as usize, config::HEAP_SIZE) }
        defmt::info!("Heap {} bytes...", config::HEAP_SIZE);

        let mut flash = ctx.device.FLASH.constrain();

        let _dma_channels = ctx.device.DMA1.split(); // for defmt

        let mut gpioa = ctx.device.GPIOA.split();
        let mut gpiob = ctx.device.GPIOB.split();
        let mut _gpioc = ctx.device.GPIOC.split();

        let mut afio = ctx.device.AFIO.constrain();
        let (_pa15, _pb3, _pb4) = afio.mapr.disable_jtag(gpioa.pa15, gpiob.pb3, gpiob.pb4);

        let mut usb_pull_up = gpioa.pa10.into_push_pull_output_with_state(
            &mut gpioa.crh,
            if !config::USB_PULLUP_ACTVE_LEVEL {
                stm32f1xx_hal::gpio::PinState::High
            } else {
                stm32f1xx_hal::gpio::PinState::Low
            },
        );

        let clocks =
            high_performance_clock_config_provider::HighPerformanceClockConfigProvider::freeze(
                &mut flash.acr,
            );
        defmt::info!("Clocks: {}", defmt::Debug2Format(&clocks));

        //---------------------------------------------------------------------

        let usb = Peripheral {
            usb: ctx.device.USB,
            pin_dm: gpioa.pa11,
            pin_dp: gpioa.pa12,
        };

        unsafe {
            USB_BUS.replace(UsbBus::new(usb));
        }

        let serial1 = CdcAcmClass::new(
            unsafe { USB_BUS.as_ref().unwrap_unchecked() },
            config::CDC_ACM_MAX_PACKET_SIZE,
        );
        let serial2 = CdcAcmClass::new(
            unsafe { USB_BUS.as_ref().unwrap_unchecked() },
            config::CDC_ACM_MAX_PACKET_SIZE,
        );
        let hid_i2c = usbd_hid::hid_class::HIDClass::new(
            unsafe { USB_BUS.as_ref().unwrap_unchecked() },
            support::HidDescriptor::desc(),
            config::HID_I2C_POLL_INTERVAL_MS,
        );

        let usb_dev = UsbDeviceBuilder::new(
            unsafe { USB_BUS.as_ref().unwrap_unchecked() },
            usb_device::prelude::UsbVidPid(0x16c0, 0x394f),
        )
        .manufacturer("SCTBElpa")
        .product("SensorProbe")
        .serial_number(stm32_device_signature::device_id_hex())
        .composite_with_iads()
        .build();

        defmt::info!("USB composite device");

        //---------------------------------------------------------------------

        // UART1
        let uart1 = stm32f1xx_hal::serial::Serial::new(
            ctx.device.USART1,
            (
                gpiob.pb6.into_alternate_push_pull(&mut gpiob.crl),
                gpiob.pb7.into_pull_up_input(&mut gpiob.crl),
            ),
            &mut afio.mapr,
            stm32f1xx_hal::serial::Config::default()
                .parity(config::DEFAULT_MODBUS_PARITY)
                .stopbits(config::DEFAULT_MODBUS_STOP_BITS)
                .baudrate(config::DEFAULT_MODBUS_BAUD_RATE.bps()),
            &clocks,
        );
        let re_de1 = gpiob
            .pb5
            .into_push_pull_output_with_state(&mut gpiob.crl, PinState::Low);

        // UART2
        let uart2 = stm32f1xx_hal::serial::Serial::new(
            ctx.device.USART2,
            (
                gpioa.pa2.into_alternate_push_pull(&mut gpioa.crl),
                gpioa.pa3.into_pull_up_input(&mut gpioa.crl),
            ),
            &mut afio.mapr,
            stm32f1xx_hal::serial::Config::default()
                .parity(config::DEFAULT_MODBUS_PARITY)
                .stopbits(config::DEFAULT_MODBUS_STOP_BITS)
                .baudrate(config::DEFAULT_MODBUS_BAUD_RATE.bps()),
            &clocks,
        );

        let re_de2 = gpioa
            .pa1
            .into_push_pull_output_with_state(&mut gpioa.crl, PinState::Low);

        defmt::info!("Serial ports");

        //---------------------------------------------------------------------

        let i2c_wraper = hw::I2cWraper::i2c1(
            ctx.device.I2C1,
            (
                gpiob.pb8.into_alternate_open_drain(&mut gpiob.crh),
                gpiob.pb9.into_alternate_open_drain(&mut gpiob.crh),
            ),
            &mut afio.mapr,
            clocks,
            hw::Mode::Standard {
                frequency: Hertz::kHz(100),
            },
        );

        defmt::info!("I2C sensor port");

        //---------------------------------------------------------------------

        let ina219_i2c = hw::I2cWraper::i2c2(
            ctx.device.I2C2,
            (
                gpiob.pb10.into_alternate_open_drain(&mut gpiob.crh),
                gpiob.pb11.into_alternate_open_drain(&mut gpiob.crh),
            ),
            clocks,
            hw::Mode::Standard {
                frequency: Hertz::kHz(100),
            },
        );

        let mut current_meter = support::CurrentMeter::new(ina219_i2c, [0x40, 0x42, 0x41]);

        const CAL_VAL: f32 = 0.04096 / (config::LSB * config::R_SUNT_OM);
        static_assertions::const_assert!(CAL_VAL < u16::MAX as f32);

        if let Err(e) = current_meter.calibrate(CAL_VAL as u16) {
            defmt::error!("INA219 calibrate error: {:?}", defmt::Debug2Format(&e));
        } else {
            defmt::info!("INA219 calibrated");
        }

        //---------------------------------------------------------------------

        let di = display_interface_spi::SPIInterface::new(
            stm32f1xx_hal::spi::Spi::spi1(
                ctx.device.SPI1,
                (
                    gpioa.pa5.into_alternate_push_pull(&mut gpioa.crl),
                    stm32f1xx_hal::spi::NoMiso,
                    gpioa.pa7.into_alternate_push_pull(&mut gpioa.crl),
                ),
                &mut afio.mapr,
                embedded_hal::spi::MODE_2, // !
                Hertz::MHz(8),             // работает на 5 МГц
                clocks,
            ),
            gpiob.pb1.into_push_pull_output(&mut gpiob.crl),
            gpioa.pa4.into_push_pull_output(&mut gpioa.crl),
        );

        let mut disp: ssd1309::prelude::GraphicsMode<_> = ssd1309::Builder::new()
            .with_rotation(ssd1309::displayrotation::DisplayRotation::Rotate270)
            .connect(di)
            .into();
        let syst = {
            let mut reset = gpiob.pb0.into_push_pull_output(&mut gpiob.crl);
            let mut delay_provider =
                cortex_m::delay::Delay::new(ctx.core.SYST, clocks.hclk().to_Hz());

            disp.reset(&mut reset, &mut delay_provider).ok();
            delay_provider.free()
        };
        disp.init().unwrap();
        disp.flush().unwrap();

        defmt::info!("Display init");

        //---------------------------------------------------------------------

        usb_pull_up.toggle(); // enable USB
        defmt::info!("USB enabled");

        //---------------------------------------------------------------------

        let mut mono = Systick::new(syst, clocks.sysclk().to_Hz());

        //---------------------------------------------------------------------

        read_current::spawn_after(config::CURRENT_READ_INTERVAL_MS.millis()).ok();
        defmt::info!("Spawn read current task");

        //---------------------------------------------------------------------

        (
            Shared {
                usb_device: usb_dev,
                serial1,
                serial2,
                hid_i2c,
                i2c: i2c_wraper,
                display_state: display_state::DisplayState::init(mono.now() + 3_500.millis()), // 3_500.millis()
                modbus1_dispatcher: bridge::ModbusDispatcher::new(
                    config::MODBUS_RESP_TIMEOUT_MS.millis(),
                ),
                modbus2_dispatcher: bridge::ModbusDispatcher::new(
                    config::MODBUS_RESP_TIMEOUT_MS.millis(),
                ),
                uart1: support::UartHalfDuplex::new(uart1, re_de1),
                uart2: support::UartHalfDuplex::new(uart2, re_de2),

                device_modbus_request: heapless::Deque::new(),
                device_modbus_response: heapless::Deque::new(),
            },
            Local {
                i2c_device: None,
                i2c_error_count: 0,

                modbus_device: None,
                modbus_error_count: 0,

                display: disp,
                current_meter,

                clocks,

                uart1_mb_assembly_buffer:
                    support::Buffer::<{ bridge::MODBUS_BUFFER_SIZE_MAX }>::new(),
                uart2_mb_assembly_buffer:
                    support::Buffer::<{ bridge::MODBUS_BUFFER_SIZE_MAX }>::new(),

                modbus_detect_retrys: 0,
            },
            init::Monotonics(mono),
        )
    }

    //-------------------------------------------------------------------------

    #[task(binds = USART1, shared = [uart1, modbus1_dispatcher], local = [uart1_mb_assembly_buffer], priority = 4)]
    fn uart1(ctx: uart1::Context) {
        let mut uart = ctx.shared.uart1;
        let mut dispatcher = ctx.shared.modbus1_dispatcher;
        let mb_assembly_buffer = ctx.local.uart1_mb_assembly_buffer;

        uart_macro::uart_interrupt!(
            busname = UART1_BUS_BIND,
            uart = uart,
            modbus_dispatcher = dispatcher,
            modbus_assembly_buffer = mb_assembly_buffer
        );
    }

    #[task(binds = USART2, shared = [uart2, modbus2_dispatcher], local = [uart2_mb_assembly_buffer], priority = 4)]
    fn uart2(ctx: uart2::Context) {
        let mut uart = ctx.shared.uart2;
        let mut dispatcher = ctx.shared.modbus2_dispatcher;
        let mb_assembly_buffer = ctx.local.uart2_mb_assembly_buffer;

        uart_macro::uart_interrupt!(
            busname = UART2_BUS_BIND,
            uart = uart,
            modbus_dispatcher = dispatcher,
            modbus_assembly_buffer = mb_assembly_buffer
        );
    }

    //-------------------------------------------------------------------------

    #[task(binds = USB_HP_CAN_TX, shared = [usb_device, serial1, serial2, hid_i2c], priority = 5)]
    fn usb_tx(ctx: usb_tx::Context) {
        let mut usb_device = ctx.shared.usb_device;
        let mut serial1 = ctx.shared.serial1;
        let mut serial2 = ctx.shared.serial2;
        let mut hid_i2c = ctx.shared.hid_i2c;

        if (&mut usb_device, &mut serial1, &mut serial2, &mut hid_i2c).lock(
            |usb_device, serial1, serial2, hid_i2c| {
                usb_device.poll(&mut [serial1, serial2, hid_i2c])
            },
        ) {
            cortex_m::peripheral::NVIC::mask(Interrupt::USB_HP_CAN_TX);
        }
    }

    #[task(binds = USB_LP_CAN_RX0, shared = [usb_device, serial1, serial2, hid_i2c], priority = 5)]
    fn usb_rx0(ctx: usb_rx0::Context) {
        let mut usb_device = ctx.shared.usb_device;
        let mut serial1 = ctx.shared.serial1;
        let mut serial2 = ctx.shared.serial2;
        let mut hid_i2c = ctx.shared.hid_i2c;

        if (&mut usb_device, &mut serial1, &mut serial2, &mut hid_i2c).lock(
            |usb_device, serial1, serial2, hid_i2c| {
                usb_device.poll(&mut [serial1, serial2, hid_i2c])
            },
        ) {
            cortex_m::peripheral::NVIC::mask(Interrupt::USB_LP_CAN_RX0);
        }
    }

    //-------------------------------------------------------------------------

    #[task(shared = [i2c, display_state], local=[i2c_device, i2c_error_count], priority = 1)]
    fn i2c_process(ctx: i2c_process::Context, mut scan_addr: u8) {
        let mut i2c = ctx.shared.i2c;
        let mut display_state = ctx.shared.display_state;

        let device = ctx.local.i2c_device;
        let i2c_error_count = ctx.local.i2c_error_count;

        // reset
        if scan_addr == 0 {
            *device = None;
        }

        let mut report_scan = |scan_addr| {
            display_state
                .lock(|display_state| display_state.scan(display_state::ScanState::I2C(scan_addr)));
        };

        if let Some(dev) = device {
            // Known device on known address
            if let Some(storage) = i2c.lock(|i2c| {
                let mut storage = dev.make_storage();
                if let Err(e) = dev.read_i2c(scan_addr, storage.as_mut(), i2c) {
                    defmt::error!("{} @ {} error: {}", dev.name(), scan_addr, e);
                    *i2c_error_count += 1;
                    if *i2c_error_count >= config::I2C_ERROR_MAX_COUNT {
                        defmt::error!("{} @ {} not responding", dev.name(), scan_addr);
                        report_scan(0); // reset
                    }
                    None
                } else {
                    *i2c_error_count = 0;
                    Some(storage)
                }
            }) {
                defmt::debug!("{}: {}", dev.name(), storage.print().as_str());
                display_state.lock(|state| state.display_output(storage));
            }
        } else {
            scan_addr += 1;
            if scan_addr > config::MAX_ADDR_TO_SCAN {
                scan_addr = config::I2C_ADDR_MIN;
            }

            report_scan(scan_addr);

            i2c.lock(|i2c| {
                let mut buf = [0u8; 4];

                match bridge::MyI2COperation::new_scan_op(&mut buf, scan_addr).execute(i2c) {
                    Ok(_) => {
                        defmt::info!("Scanning I2C addr 0x{:X}, something detected!", scan_addr);
                        for dev in I2C_DEVICES {
                            if let Err(e) = dev.probe_i2c(scan_addr, i2c) {
                                defmt::error!("{} error: {}", dev.name(), e);
                            } else {
                                defmt::info!("{} found!", dev.name());

                                device.replace(*dev);
                                *i2c_error_count = 0;
                            }
                        }

                        if device.is_none() {
                            defmt::error!("Unknown device at 0x{:X}, skip...", scan_addr)
                        }
                    }
                    Err(_) => defmt::trace!("Scanning I2C addr 0x{:X}, no ansver", scan_addr),
                }
            });
        }

        update_display::spawn().ok(); // update display
    }

    //-------------------------------------------------------------------------

    #[task(shared = [display_state, device_modbus_request, device_modbus_response], local=[modbus_device, modbus_error_count, modbus_detect_retrys], priority = 1)]
    fn modbus_inquary_process(
        ctx: modbus_inquary_process::Context,
        bus_id: &'static str,
        mut scan_addr: modbus_core::rtu::SlaveId,
    ) {
        let (mut tx, mut rx) = (
            ctx.shared.device_modbus_request,
            ctx.shared.device_modbus_response,
        );
        let mut display_state = ctx.shared.display_state;

        let device = ctx.local.modbus_device;
        let modbus_error_count = ctx.local.modbus_error_count;
        let detect_retrys = ctx.local.modbus_detect_retrys;

        // reset
        if scan_addr == 0 {
            *device = None;
            rx.lock(|rx| rx.clear());
            tx.lock(|tx| tx.clear());
        }

        let mut send_data_reqs =
            |device: &dyn devices::ModbusDevice, scan_addr: modbus_core::rtu::SlaveId| {
                tx.lock(|tx| {
                    device.build_data_request_iter().for_each(|pdu| {
                        let req = modbus_core::rtu::RequestAdu {
                            hdr: modbus_core::rtu::Header { slave: scan_addr },
                            pdu,
                        };
                        tx.push_back((req, bus_id)).unwrap();
                    })
                });
            };

        let mut report_scan = |scan_addr| {
            display_state.lock(|display_state| {
                display_state.scan(match bus_id {
                    UART1_BUS_BIND => display_state::ScanState::UART(scan_addr),
                    UART2_BUS_BIND => display_state::ScanState::RS485(scan_addr),
                    _ => panic!(),
                })
            });
        };

        if let Some(dev) = device {
            // Known device on known address
            let storage = rx.lock(|rx| {
                if rx.is_empty() {
                    None
                } else {
                    let mut storage = dev.make_storage();
                    let res = if let Err(e) =
                        dev.decode_resps(storage.as_mut(), &mut rx.iter(), bus_id)
                    {
                        defmt::error!("{} error: {}", dev.name(), defmt::Debug2Format(&e));
                        None
                    } else {
                        Some(storage)
                    };

                    rx.clear();

                    res
                }
            });

            if let Some(storage) = storage {
                defmt::debug!("{}: {}", dev.name(), storage.print().as_str());
                display_state.lock(|state| state.display_output(storage));
                *modbus_error_count = 0;
            } else {
                *modbus_error_count += 1;
                if *modbus_error_count >= config::MODBUS_ERROR_MAX_COUNT {
                    defmt::error!("{} @ {} not responding", dev.name(), scan_addr);
                    report_scan(0); // reset
                }
            }

            send_data_reqs(*dev, scan_addr);
        } else {
            // Scan for device
            let detected = loop {
                if let Some((data, rx_bus)) = rx.lock(|rx| rx.pop_front()) {
                    if rx_bus == bus_id {
                        if let Some(true) = data.as_slice().try_decode_response().map(|resp| {
                            defmt::info!(
                                "Scanning {} addr 0x{:X}, something detected!",
                                bus_id,
                                resp.hdr.slave
                            );

                            for dev in MODBUS_DEVICES {
                                if let Err(e) = dev.probe_resp(resp) {
                                    defmt::error!(
                                        "{} error: {}",
                                        dev.name(),
                                        defmt::Debug2Format(&e)
                                    );
                                } else {
                                    defmt::info!("{} found!", dev.name());

                                    device.replace(*dev);
                                    *modbus_error_count = 0;
                                    *detect_retrys = 0;

                                    send_data_reqs(*dev, scan_addr);

                                    return true;
                                }
                            }

                            *detect_retrys += 1;

                            if *detect_retrys < config::MODBUS_DETECT_RETRYS {
                                scan_addr -= 1;
                                defmt::error!("Retry ({}) detecing...", *detect_retrys);
                            } else {
                                *detect_retrys = 0;
                                defmt::error!("Unknown device at 0x{:X}, skip...", scan_addr);
                            }

                            false
                        }) {
                            *detect_retrys = 0;
                            break true;
                        }
                    }
                } else {
                    break false;
                }
            };

            if !detected {
                scan_addr += 1;
                if scan_addr > config::MAX_ADDR_TO_SCAN {
                    scan_addr = 0;
                }

                // send scan request read holding register 0
                let req = modbus_core::rtu::RequestAdu {
                    hdr: modbus_core::rtu::Header { slave: scan_addr },
                    pdu: modbus_core::RequestPdu(modbus_core::Request::ReadHoldingRegisters(
                        0x00, 0x01,
                    )),
                };

                tx.lock(|tx| {
                    if tx.push_back((req, bus_id)).is_err() && scan_addr > 1 {
                        scan_addr -= 1;
                    }
                });

                report_scan(scan_addr);
            }
        }

        update_display::spawn().ok(); // update display
    }

    //-------------------------------------------------------------------------

    #[task(shared = [display_state], local=[display], priority = 1)]
    fn update_display(mut ctx: update_display::Context) {
        let display = ctx.local.display;
        ctx.shared
            .display_state
            .lock(|state| {
                display.clear();
                state.render(display, monotonics::MonoTimer::now())?;
                display.flush()
            })
            .unwrap();
    }

    //-------------------------------------------------------------------------

    #[task(shared = [display_state], local = [current_meter], priority = 1)]
    fn read_current(ctx: read_current::Context) {
        let mut display_state = ctx.shared.display_state;
        let current_meter = ctx.local.current_meter;

        let current_values = if let Ok(mut current) = current_meter.current(config::LSB) {
            current.iter_mut().for_each(|v| *v *= 1000f32);

            defmt::debug!("Current: {} mA", current);

            display_state::CurrentValues::from(current)
        } else {
            use micromath::F32Ext;

            let fi = monotonics::MonoTimer::now().ticks() as f32;
            let values = [
                ((fi / 25670.3 + 1.1).sin() * 10f32).max(-0.1),
                ((fi / 35040.0 + 5.3).sin() * 2f32).max(-0.1),
                ((fi / 10010.1 + 2.1).sin() * 6f32).max(-0.1),
            ];

            defmt::error!("Current read error, simulate: {} mA", values);

            display_state::CurrentValues::from(values)
        };

        let need_bus_process = display_state.lock(move |ds| ds.update_current(current_values));

        let start_delay = config::START_DURATION_MS.millis();
        match need_bus_process {
            Some(display_state::ScanState::UART(a)) => {
                modbus_inquary_process::spawn_after(
                    if a == 0 { start_delay } else { 0.millis() },
                    UART1_BUS_BIND,
                    a,
                )
                .ok();
            }
            Some(display_state::ScanState::RS485(a)) => {
                modbus_inquary_process::spawn_after(
                    if a == 0 { start_delay } else { 0.millis() },
                    UART2_BUS_BIND,
                    a,
                )
                .ok();
            }
            Some(display_state::ScanState::I2C(a)) => {
                i2c_process::spawn_after(if a == 0 { start_delay } else { 0.millis() }, a).ok();
            }
            _ => {}
        }

        read_current::spawn_after(config::CURRENT_READ_INTERVAL_MS.millis()).ok();
    }

    //-------------------------------------------------------------------------

    #[idle(shared=[serial1, serial2, hid_i2c, i2c, display_state,
        modbus1_dispatcher, modbus2_dispatcher, uart1, uart2,
        device_modbus_request, device_modbus_response], local = [clocks])]
    fn idle(ctx: idle::Context) -> ! {
        use bridge::RxBuffer;
        use uart_macro::{
            process_modbus_dispatcher, serialprocess_line_coding, try_read_vcom, try_tx_to_vcom,
            update_line_coding_if_changed,
        };

        fn extract_line_codding<'a>(
            vcom: &mut CdcAcmClass<'a, UsbBus<Peripheral>>,
        ) -> MyLineCoding {
            unsafe { core::mem::transmute_copy::<_, MyLineCoding>(vcom.line_coding()) }
        }

        let mut v_com1 = ctx.shared.serial1;
        let mut v_com2 = ctx.shared.serial2;
        let mut hid_i2c = ctx.shared.hid_i2c;
        let mut display_state = ctx.shared.display_state;

        let mut i2c = ctx.shared.i2c;

        let mut uart1 = ctx.shared.uart1;
        let mut uart2 = ctx.shared.uart2;

        let mut modbus1_buffer_ready = false;
        let mut modbus1_assembly_buffer =
            support::Buffer::<{ bridge::MODBUS_BUFFER_SIZE_MAX }>::new();
        let mut modbus1_resp_buffer: Option<support::VecBuffer> = None;

        let mut modbus2_buffer_ready = false;
        let mut modbus2_assembly_buffer =
            support::Buffer::<{ bridge::MODBUS_BUFFER_SIZE_MAX }>::new();
        let mut modbus2_resp_buffer: Option<support::VecBuffer> = None;

        let mut modbus_dispatcher1 = ctx.shared.modbus1_dispatcher;
        let mut modbus_dispatcher2 = ctx.shared.modbus2_dispatcher;

        let mut device_modbus_request = ctx.shared.device_modbus_request;
        let mut device_modbus_response = ctx.shared.device_modbus_response;

        let clocks = ctx.local.clocks;

        let mut prev_line_codings = [
            v_com1.lock(extract_line_codding),
            v_com2.lock(extract_line_codding),
        ];

        loop {
            cortex_m::interrupt::free(|_| unsafe {
                cortex_m::peripheral::NVIC::unmask(Interrupt::USB_HP_CAN_TX);
                cortex_m::peripheral::NVIC::unmask(Interrupt::USB_LP_CAN_RX0);

                cortex_m::asm::wfi();
            });

            {
                // process V_COM1
                if !serialprocess_line_coding!(
                    name = UART1_BUS_BIND,
                    vcom = v_com1,
                    uart = uart1,
                    prev_line_coding = &mut prev_line_codings[0],
                    clocks = clocks
                ) && !modbus1_buffer_ready
                {
                    try_read_vcom!(
                        name = UART1_BUS_BIND,
                        vcom = v_com1,
                        modbus_assembly_buffer = modbus1_assembly_buffer,
                        modbus_buffer_ready = modbus1_buffer_ready
                    );
                }

                process_modbus_dispatcher!(
                    name = UART1_BUS_BIND,
                    modbus_dispatcher = modbus_dispatcher1,
                    uart = uart1,
                    modbus_assembly_buffer = modbus1_assembly_buffer,
                    modbus_buffer_ready = modbus1_buffer_ready,
                    modbus_resp_buffer = modbus1_resp_buffer,
                    device_request = device_modbus_request,
                    device_response = device_modbus_response
                );

                try_tx_to_vcom!(
                    name = UART1_BUS_BIND,
                    modbus_resp_buffer = modbus1_resp_buffer,
                    vcom = v_com1
                );
            }

            {
                // process V_COM2
                if !serialprocess_line_coding!(
                    name = UART2_BUS_BIND,
                    vcom = v_com2,
                    uart = uart2,
                    prev_line_coding = &mut prev_line_codings[1],
                    clocks = clocks
                ) && !modbus2_buffer_ready
                {
                    try_read_vcom!(
                        name = UART2_BUS_BIND,
                        vcom = v_com2,
                        modbus_assembly_buffer = modbus2_assembly_buffer,
                        modbus_buffer_ready = modbus2_buffer_ready
                    );
                }

                process_modbus_dispatcher!(
                    name = UART2_BUS_BIND,
                    modbus_dispatcher = modbus_dispatcher2,
                    uart = uart2,
                    modbus_assembly_buffer = modbus2_assembly_buffer,
                    modbus_buffer_ready = modbus2_buffer_ready,
                    modbus_resp_buffer = modbus2_resp_buffer,
                    device_request = device_modbus_request,
                    device_response = device_modbus_response
                );

                try_tx_to_vcom!(
                    name = UART2_BUS_BIND,
                    modbus_resp_buffer = modbus2_resp_buffer,
                    vcom = v_com2
                );
            }

            {
                // HID-I2C
                let (res, mut buf) = hid_i2c.lock(|hid_i2c| {
                    let mut buf = [0u8; support::HID_REPORT_SIZE];
                    let res = hid_i2c.pull_raw_output(&mut buf);
                    (res, buf)
                });

                if res.is_ok() {
                    match i2c.lock(|i2c| bridge::MyI2COperation::on(&mut buf).execute(i2c)) {
                        Ok(_result) => {
                            defmt::trace!("I2C operation success");
                        }
                        Err(e) => {
                            defmt::error!("I2C error: {}", e);
                            buf[0] = e.into();
                            buf[1] = 0;
                        }
                    }

                    if let Err(e) = hid_i2c.lock(move |hid_i2c| hid_i2c.push_raw_input(&buf)) {
                        defmt::error!("HID-I2C push error: {}", defmt::Debug2Format(&e));
                    }
                }
            }

            // display
            if display_state.lock(|state| state.animate(monotonics::MonoTimer::now())) {
                update_display::spawn().ok();
            }
        }
    }
}
