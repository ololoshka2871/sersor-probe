use usbd_serial::LineCoding;

use crate::{bridge::ModbusBufferBufferLock, support::MyLineCoding};

pub fn update_line_coding_if_changed(
    lc: &mut MyLineCoding,
    new: &LineCoding,
    bus_name: &'static str,
) -> bool {
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

pub fn try_commit_request(mut buf: ModbusBufferBufferLock, bus_name: &'static str) -> bool {
    let owner = buf.owner();
    match buf.try_commit_request() {
        Ok((tx_body, slave)) => {
            defmt::debug!(
                "Request {} from {} -> 0x{:X}: {}",
                bus_name,
                owner,
                slave,
                tx_body
            );
            true
        }
        Err(e) => {
            defmt::error!("{}: Incorrect buffer state {}", bus_name, e);
            false
        }
    }
}

macro_rules! process_modbus {
    (name=$bus_name: expr, vcom=$v_serial: expr, uart=$uart: expr, re_de=$re_de: expr,
        buf=$modbus_buffer: expr, prev_line_coding=$prev_line_coding: expr, clocks=$clocks: expr) => {{
        if $v_serial.lock(|serial| {
            update_line_coding_if_changed($prev_line_coding, serial.line_coding(), $bus_name)
        }) {
            // reconfigure uart if line coding changed
            $uart.lock(|uart| {
                if let Err(e) = uart.reconfigure(*$prev_line_coding, $clocks) {
                    defmt::error!(
                        "{} reconfigure error: {:?}",
                        $bus_name,
                        defmt::Debug2Format(&e)
                    );
                }
            });
        } else {
            // only if line coding not changed
            $modbus_buffer.lock(|mb_buf| {
                let actualy_transmitted =
                    mb_buf.with(crate::bridge::Owner::USB, |mut b| -> Option<usize> {
                        if let Ok(d) = b.try_transmitt() {
                            // transmitting to USB
                            if d.len() > 0 {
                                let actualy_transmitted =
                                    $v_serial.lock(|serial| match serial.write_packet(d) {
                                        Ok(count) if count > 0 => {
                                            defmt::trace!("{} -> USB: {} bytes", $bus_name, count);
                                            count
                                        }
                                        _ => {
                                            defmt::trace!("{} -> USB: END", $bus_name);
                                            0
                                        }
                                    });

                                b.bytes_transmitted(actualy_transmitted);
                                Some(actualy_transmitted)
                            } else {
                                Some(0)
                            }
                        } else {
                            None
                        }
                    });

                let rx_possible = match actualy_transmitted {
                    Ok(Some(0)) => {
                        mb_buf.reset();
                        true
                    }
                    Ok(Some(_)) => false,
                    Ok(None) => true,
                    Err(_) => false,
                };

                let got_pocket: bool = if rx_possible {
                    mb_buf
                        .with(crate::bridge::Owner::USB, |mut b| -> Result<bool, ()> {
                            match b.can_feed() {
                                Ok(0) => Err(()),
                                Ok(_) => Ok($v_serial.lock(|serial| {
                                    match serial.read_packet(b.feed_me_to()) {
                                        Ok(count) if count > 0 => {
                                            b.feed(count);
                                            true
                                        }
                                        _ => false,
                                    }
                                })),
                                Err(nb::Error::WouldBlock) => Ok(false),
                                _ => unreachable!(),
                            }
                        })
                        .map(|r| -> bool {
                            match r {
                                Ok(r) => r,
                                Err(_) => {
                                    mb_buf.reset(); // can't lock mb_buf in previous closure
                                    false
                                }
                            }
                        })
                        .unwrap_or(false)
                } else {
                    false
                };

                if got_pocket
                    && mb_buf
                        .with(crate::bridge::Owner::USB, |b| {
                            try_commit_request(b, $bus_name)
                        })
                        .unwrap_or(false)
                {
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
                    match buf.feed_rx_byte(b) {
                        Ok(false) => {
                            defmt::warn!("{}: buffer overflow", $name);

                            uart.unlisten(stm32f1xx_hal::serial::Event::Rxne);

                            buf.reset();
                        }
                        Ok(true) => {
                            // try parse response
                            match buf.try_commit_response() {
                                Ok((tx_body, slave)) => {
                                    defmt::debug!(
                                        "Got correct response {} from 0x{:X}: {}",
                                        $name,
                                        slave,
                                        tx_body
                                    );
                                }
                                Err(nb::Error::WouldBlock) => { /* insufficient data, continue */ }
                                Err(nb::Error::Other(e)) => {
                                    defmt::warn!("{}: {}", $name, e);
                                    buf.reset();
                                }
                            }
                        }
                        Err(e) => {
                            defmt::warn!("{}: Rx data, but wrong buffer state {}", $name, e);

                            uart.unlisten(stm32f1xx_hal::serial::Event::Rxne);
                        }
                    }
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
                    buf.start_rx();

                    $tx2rx_timeout_task;

                    defmt::trace!("{} (from {}): Tx done", $name, buf.owner());
                }
            } else {
                defmt::panic!("{}: unknown interrupt!", $name);
            }
        });
    };
}

pub(crate) use process_modbus;
pub(crate) use uart_interrupt;
