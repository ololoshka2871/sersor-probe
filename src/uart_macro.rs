use usbd_serial::LineCoding;

use crate::{bridge::ModbusBuffer, support::MyLineCoding};

pub fn update_line_coding_if_changed(lc: &mut MyLineCoding, new: &LineCoding, bus_name: &'static str) -> bool {
    if lc.data_rate != new.data_rate()
        || lc.parity_type != new.parity_type()
        || lc.stop_bits != new.stop_bits()
        || lc.data_bits != new.data_bits()
    {
        *lc = unsafe { core::mem::transmute_copy::<_, MyLineCoding>(new) };
        defmt::info!("{}: New {}", bus_name, lc);
        true
    } else {
        false
    }
}

pub fn try_commit_request(buf: &mut ModbusBuffer, bus_name: &'static str) -> bool {
    if let Ok((tx_body, slave)) = buf.try_commit_request() {
        defmt::debug!("Request {} -> 0x{:X}: {}", bus_name, slave, tx_body);
        true
    } else {
        false
    }
}

macro_rules! process_modbus {
    (name=$bus_name: expr, vcom=$v_serial: expr, uart=$uart: expr, re_de=$re_de: expr,
        buf=$modbus_buffer: expr, prev_line_coding=$prev_line_coding: expr, clocks=$clocks: expr) => {{
        // reconfigure uart if line coding changed
        if !$v_serial.lock(|serial| {
            let line_coding_changed =
                update_line_coding_if_changed($prev_line_coding, serial.line_coding(), $bus_name);
            if line_coding_changed {
                $uart.lock(|uart| {
                    if let Err(e) = uart.reconfigure(*$prev_line_coding, $clocks) {
                        defmt::error!(
                            "{} reconfigure error: {:?}",
                            $bus_name,
                            defmt::Debug2Format(&e)
                        );
                    }
                });
            }
            line_coding_changed
        }) {
            // only if line coding not changed
            $modbus_buffer.lock(|mb_buf| {
                let got_pocket = match mb_buf.can_feed() {
                    Ok(0) => {
                        mb_buf.reset();
                        false
                    }
                    Ok(_) => {
                        $v_serial.lock(|serial| match serial.read_packet(mb_buf.feed_me_to()) {
                            Ok(count) if count > 0 => {
                                mb_buf.feed(count);
                                true
                            }
                            _ => false,
                        })
                    }
                    Err(nb::Error::WouldBlock) => false,
                    _ => unreachable!(),
                };

                if got_pocket && try_commit_request(mb_buf, $bus_name) {
                    let b = mb_buf.start_tx();
                    (&mut $uart, &mut $re_de).lock(|uart, re_de| {
                        re_de.set_high();
                        uart.write(b).ok();
                        uart.listen(stm32f1xx_hal::serial::Event::Txe);
                    });
                }
            });
        }
    }};
}

macro_rules! uart_interrupt {
    (busname=$name: expr, uart=$uart:expr, buffer=$modbus_buffer: expr, re_de=$re_de: expr, tx2rx_timeout_task = $tx2rx_timeout_task: expr) => {
        (&mut $uart, &mut $modbus_buffer).lock(|uart, buf| {
            if uart.is_rx_not_empty() {
                let r: Result<u8, _> = uart.read();
                if let Ok(b) = r {
                    defmt::trace!("{}: 0x{:X}", $name, b);
                }
            } else if uart.is_tx_empty() {
                if let Some(b) = buf.next_tx() {
                    uart.write(b).ok();
                } else {
                    uart.unlisten(stm32f1xx_hal::serial::Event::Txe);

                    uart.tx.bflush().ok(); // flush tx
                    while let Ok(_) = uart.rx.read() {} // flush rx

                    $re_de.lock(stm32f1xx_hal::gpio::Pin::set_low);
                    uart.listen(stm32f1xx_hal::serial::Event::Rxne);

                    $tx2rx_timeout_task;

                    defmt::trace!("{}: Tx done", $name);
                }
            } else {
                defmt::panic!("{}: unknown interrupt!", $name);
            }
        });
    };
}

pub(crate) use process_modbus;
pub(crate) use uart_interrupt;
