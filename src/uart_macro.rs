use usbd_serial::LineCoding;

use crate::support::MyLineCoding;

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

macro_rules! serialprocess_line_coding {
    (name=$bus_name: expr, vcom=$v_serial: expr, uart=$uart: expr, prev_line_coding=$prev_line_coding: expr, clocks=$clocks: expr) => {{
        let lc_changed = $v_serial.lock(|serial| {
            update_line_coding_if_changed($prev_line_coding, serial.line_coding(), $bus_name)
        });
        if lc_changed {
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
        }
        lc_changed
    }};
}

macro_rules! try_read_vcom {
    (name=$bus_name: expr, vcom=$v_serial: expr, modbus_assembly_buffer=$modbus_assembly_buffer: expr, modbus_buffer_ready=$modbus_buffer_ready: expr) => {
        $v_serial.lock(
            |cdc_acm| match cdc_acm.read_packet($modbus_assembly_buffer.write_me()) {
                Ok(n) if n > 0 => {
                    $modbus_assembly_buffer.add_offset(n);
                    $modbus_buffer_ready = match $modbus_assembly_buffer.try_decode_request() {
                        Some(_r) => true,
                        None => {
                            if $modbus_assembly_buffer.is_full() {
                                defmt::error!("{} buffer full, drop..", $bus_name);
                                $modbus_assembly_buffer.reset();
                            }
                            false
                        }
                    }
                }
                _ => {}
            },
        )
    };
}

macro_rules! process_modbus_dispatcher {
    (name=$bus_name: expr, modbus_dispatcher=$modbus_dispatcher: expr, uart=$uart: expr, modbus_assembly_buffer=$modbus_assembly_buffer: expr,
        modbus_buffer_ready=$modbus_buffer_ready: expr, modbus_resp_buffer=$modbus_resp_buffer: expr,
        device_request=$device_request: expr, device_response=$device_response: expr) => {
        $modbus_dispatcher.lock(|modbus_dispatcher| {
            // try commit request
            let now = monotonics::MonoTimer::now();
            if $modbus_buffer_ready {
                modbus_dispatcher.push_raw_request(
                    $modbus_assembly_buffer.as_slice(),
                    bridge::Requester::USB,
                    now,
                );
                $modbus_assembly_buffer.reset();
                $modbus_buffer_ready = false;
            } else {
                $device_request.lock(|device_request| {
                    while let Some((dr, bus_name)) = device_request.front() {
                        if bus_name == &$bus_name {
                            if modbus_dispatcher.push_request(dr, bridge::Requester::Device, now) {
                                device_request.pop_front();
                            }
                        } else {
                            break;
                        }
                    }
                });
            }

            // try start transmit request
            if modbus_dispatcher.ready_tx() {
                $uart.lock(|uart| {
                    if let Ok(tx) = uart.switch_tx(now) {
                        tx.write(modbus_dispatcher.start_tx()).ok();
                    };
                })
            }

            // try take ready response
            if let (true, Some((resp, _ts))) = (
                $modbus_resp_buffer.is_none(),
                modbus_dispatcher.try_take_resp_by_source(bridge::Requester::USB),
            ) {
                $modbus_resp_buffer.replace(resp.into());
            }

            $device_response.lock(|device_response| {
                while let (false, Some((resp, _ts))) = (
                    device_response.is_full(),
                    modbus_dispatcher.try_take_resp_by_source(bridge::Requester::Device),
                ) {
                    device_response.push_back((resp, $bus_name)).ok();
                }
            });
        });
    };
}

macro_rules! try_tx_to_vcom {
    (name=$bus_name: expr, modbus_resp_buffer=$modbus_resp_buffer: expr, vcom=$v_serial: expr) => {
        if let Some(tx_buf) = &mut $modbus_resp_buffer {
            $v_serial.lock(|cdc_acm| {
                let buf = tx_buf.write_me();
                let len = buf.len().min(cdc_acm.max_packet_size() as usize);
                match cdc_acm.write_packet(&buf[..len]) {
                    Ok(writen) => {
                        defmt::trace!("Transmit chank to V_{}: {}", $bus_name, writen);
                        tx_buf.add_offset(writen);
                    }
                    Err(e) => {
                        defmt::error!("V_{}: {}", $bus_name, defmt::Debug2Format(&e));
                    }
                }
            });

            // end of transmit
            if tx_buf.is_full() {
                $modbus_resp_buffer = None;
            }
        }
    };
}

macro_rules! uart_interrupt {
    (busname=$name: expr, uart=$uart:expr, modbus_dispatcher=$dispatcher: expr, modbus_assembly_buffer=$modbus_assembly_buffer: expr) => {
        $uart.lock(|uart| {
            let now = monotonics::MonoTimer::now();
            if let Some(rx) = uart.rx_irq() {
                if let Ok(byte) = rx.read() {
                    uart.reset_tx_delay(now);
                    if $modbus_assembly_buffer.feed_byte(byte) {
                        if let Some(adu) = $modbus_assembly_buffer.try_decode_response() {
                            defmt::trace!("{}: Valid response!", $name);
                            $dispatcher.lock(|dispatcher| {
                                dispatcher.dispatch_response(
                                    $modbus_assembly_buffer.as_slice(),
                                    adu,
                                    now,
                                )
                            });
                            $modbus_assembly_buffer.reset();
                        }
                    } else {
                        defmt::error!("{} Rx buffer overflow", $name);
                        $modbus_assembly_buffer.reset();
                    }
                }
            } else {
                if let Some(tx) = uart.tx_irq() {
                    match $dispatcher.lock(bridge::ModbusDispatcher::next_tx) {
                        Ok(byte) => {
                            tx.write(byte).ok();
                        }
                        Err(Some(ts)) => {
                            uart.switch_rx(now);
                            defmt::trace!("{} Tx complete (T{})", $name, ts.ticks());
                        }
                        Err(None) => {
                            uart.switch_rx(now);
                        }
                    }
                }
            }
        });
    };
}

//------------------------------------------------------------------------------------------------

// export macros
pub(crate) use process_modbus_dispatcher;
pub(crate) use serialprocess_line_coding;
pub(crate) use try_read_vcom;
pub(crate) use try_tx_to_vcom;
pub(crate) use uart_interrupt;
