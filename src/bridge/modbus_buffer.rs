use modbus_core::rtu::{server, SlaveId};

pub const MODBUS_BUFFER_SIZE_MAX: usize = 256;

#[derive(Clone, PartialEq, Debug, defmt::Format)]
pub enum ModbusBufferState {
    Idle,
    FillingRequest {
        offset: usize,
    },
    ValidRequest {
        size: usize,
        slave: SlaveId,
        func_code: u8,
    },
    Tx {
        slave: SlaveId,
        slave_func_code: u8,
        offset: usize,
        size: usize,
    },
    Rx {
        offset: usize,
        want_slave: SlaveId,
        want_func_code: u8,
        timeout_counter: u8,
    },
    ValidResponse {
        size: usize,
        slave: SlaveId,
        func_code: u8,
    },
    Transmitting {
        offset: usize,
        size: usize,
    },
}

#[derive(Debug)]
pub enum RxError {
    NoOwner,
    IncorrectDeviceAddress {
        want_slave: SlaveId,
        got_slave: SlaveId,
    },
    IncorrectFunction {
        want_func_code: u8,
        got_func_code: u8,
    },
    IncorrectPacket(modbus_core::Error),
}

impl defmt::Format for RxError {
    fn format(&self, fmt: defmt::Formatter) {
        match self {
            RxError::NoOwner => defmt::write!(fmt, "RxError::NoOwner"),
            RxError::IncorrectDeviceAddress {
                want_slave,
                got_slave,
            } => defmt::write!(
                fmt,
                "RxError: incorrect device address 0x{:X}/0x{:X}",
                want_slave,
                got_slave
            ),
            RxError::IncorrectFunction {
                want_func_code,
                got_func_code,
            } => defmt::write!(
                fmt,
                "RxError: incorrect function 0x{:X}/0x{:X}",
                want_func_code,
                got_func_code
            ),
            RxError::IncorrectPacket(e) => {
                defmt::write!(fmt, "RxError::IncorrectPacket({})", defmt::Debug2Format(e))
            }
        }
    }
}

#[derive(Debug, defmt::Format, Clone, Copy, PartialEq)]
pub enum Owner {
    Device,
    USB,
}

pub struct ModbusBuffer {
    buffer: [u8; MODBUS_BUFFER_SIZE_MAX],
    state: ModbusBufferState,
    owner: Option<Owner>,
}

pub struct ModbusBufferBufferLock<'a>(pub &'a mut ModbusBuffer);

impl<'a> ModbusBufferBufferLock<'a> {
    pub fn owner(&self) -> Owner {
        self.0.owner.unwrap()
    }

    pub fn try_transmitt(&self) -> nb::Result<&[u8], ()> {
        match self.0.state {
            ModbusBufferState::ValidResponse { size, .. } => Ok(&self.0.buffer[..size]),
            ModbusBufferState::Transmitting { offset, size } => Ok(&self.0.buffer[offset..size]),
            _ => Err(nb::Error::WouldBlock),
        }
    }

    pub fn bytes_transmitted(&mut self, transmitted: usize) {
        match &mut self.0.state {
            ModbusBufferState::ValidResponse { size, .. } => {
                self.0.state = ModbusBufferState::Transmitting {
                    offset: transmitted,
                    size: *size,
                };
            }
            ModbusBufferState::Transmitting { offset, size } => {
                if *offset + transmitted > *size {
                    panic!();
                } else {
                    *offset += transmitted;
                }
            }
            _ => panic!(),
        }
    }

    pub fn can_feed(&self) -> nb::Result<usize, ()> {
        match self.0.state {
            ModbusBufferState::Idle => Ok(MODBUS_BUFFER_SIZE_MAX),
            ModbusBufferState::FillingRequest { offset } => Ok(MODBUS_BUFFER_SIZE_MAX - offset),
            ModbusBufferState::ValidRequest { .. }
            | ModbusBufferState::ValidResponse { .. }
            | ModbusBufferState::Transmitting { .. } => Err(nb::Error::WouldBlock),
            ModbusBufferState::Tx { .. } => Err(nb::Error::WouldBlock),
            ModbusBufferState::Rx {
                offset,
                want_slave,
                want_func_code,
                ..
            } => {
                if (offset < 2)
                    || (self.0.buffer[0] == want_slave && self.0.buffer[1] & 0x7f == want_func_code)
                {
                    Ok(MODBUS_BUFFER_SIZE_MAX - offset)
                } else {
                    Err(nb::Error::WouldBlock)
                }
            }
        }
    }

    pub fn feed_me_to(&mut self) -> &mut [u8] {
        if self.0.owner.is_none() {
            panic!();
        }
        match self.0.state {
            ModbusBufferState::Idle => {
                self.0.state = ModbusBufferState::FillingRequest { offset: 0 };
                &mut self.0.buffer[..]
            }
            ModbusBufferState::FillingRequest { offset } | ModbusBufferState::Rx { offset, .. } => {
                &mut self.0.buffer[offset..]
            }
            ModbusBufferState::ValidRequest { .. }
            | ModbusBufferState::Tx { .. }
            | ModbusBufferState::ValidResponse { .. }
            | ModbusBufferState::Transmitting { .. } => panic!(),
        }
    }

    pub fn feed(&mut self, n: usize) {
        if self.0.owner.is_none() {
            panic!();
        }
        match &mut self.0.state {
            ModbusBufferState::Idle => panic!(),
            ModbusBufferState::FillingRequest { offset } | ModbusBufferState::Rx { offset, .. } => {
                if n + *offset > MODBUS_BUFFER_SIZE_MAX {
                    panic!();
                }
                *offset += n;
            }
            ModbusBufferState::ValidRequest { .. }
            | ModbusBufferState::Tx { .. }
            | ModbusBufferState::ValidResponse { .. }
            | ModbusBufferState::Transmitting { .. } => panic!(),
        }
    }

    pub fn feed_byte(&mut self, data_byte: u8) -> bool {
        if self.0.owner.is_none() {
            panic!();
        }
        match &mut self.0.state {
            ModbusBufferState::FillingRequest { offset } => {
                if *offset >= MODBUS_BUFFER_SIZE_MAX {
                    false
                } else {
                    self.0.buffer[*offset] = data_byte;
                    *offset += 1;
                    true
                }
            }
            ModbusBufferState::Idle
            | ModbusBufferState::ValidRequest { .. }
            | ModbusBufferState::Rx { .. }
            | ModbusBufferState::Tx { .. }
            | ModbusBufferState::ValidResponse { .. }
            | ModbusBufferState::Transmitting { .. } => panic!(),
        }
    }

    pub fn try_commit_request(&mut self) -> Result<(&[u8], SlaveId), &ModbusBufferState> {
        if self.0.owner.is_none() {
            panic!();
        }
        match self.0.state {
            ModbusBufferState::FillingRequest { offset } => {
                if let Ok(Some(req)) = server::decode_request(&self.0.buffer) {
                    self.0.state = ModbusBufferState::ValidRequest {
                        size: offset,
                        slave: req.hdr.slave,
                        func_code: self.0.buffer[1],
                    };
                    Ok((&self.0.buffer[..offset], req.hdr.slave))
                } else {
                    Err(&self.0.state)
                }
            }
            ModbusBufferState::ValidRequest { size, slave, .. } => {
                Ok((&self.0.buffer[..size], slave))
            }
            ModbusBufferState::Idle
            | ModbusBufferState::Tx { .. }
            | ModbusBufferState::Rx { .. }
            | ModbusBufferState::ValidResponse { .. }
            | ModbusBufferState::Transmitting { .. } => Err(&self.0.state),
        }
    }
}

impl Default for ModbusBuffer {
    fn default() -> Self {
        Self {
            buffer: [0; MODBUS_BUFFER_SIZE_MAX],
            state: ModbusBufferState::Idle,
            owner: None,
        }
    }
}

impl ModbusBuffer {
    pub fn state(&self) -> &ModbusBufferState {
        &self.state
    }

    pub fn owner(&self) -> Option<Owner> {
        self.owner
    }

    pub fn reset(&mut self) {
        defmt::trace!("ModbusBuffer::<{}>::reset", self.owner);
        self.state = ModbusBufferState::Idle;
        self.owner = None;
    }

    pub fn with<U, F>(&mut self, owner: Owner, f: F) -> Result<U, Owner>
    where
        F: FnOnce(ModbusBufferBufferLock<'_>) -> U,
    {
        match self.owner {
            None => {
                self.owner = Some(owner);
                let res = f(ModbusBufferBufferLock(self));
                Ok(res)
            }
            Some(o) if o == owner => Ok(f(ModbusBufferBufferLock(self))),
            Some(o) => Err(o),
        }
    }

    pub fn start_tx(&mut self) -> u8 {
        if self.owner.is_none() {
            panic!();
        }
        match self.state {
            ModbusBufferState::Idle
            | ModbusBufferState::FillingRequest { .. }
            | ModbusBufferState::Rx { .. }
            | ModbusBufferState::ValidResponse { .. }
            | ModbusBufferState::Transmitting { .. } => panic!(),
            ModbusBufferState::ValidRequest {
                slave,
                size,
                func_code,
            }
            | ModbusBufferState::Tx {
                slave,
                size,
                slave_func_code: func_code,
                ..
            } => {
                self.state = ModbusBufferState::Tx {
                    slave,
                    offset: 1,
                    size,
                    slave_func_code: func_code,
                };
                self.buffer[0]
            }
        }
    }

    pub fn next_tx(&mut self) -> Option<u8> {
        if self.owner.is_none() {
            return None;
        }
        match &mut self.state {
            ModbusBufferState::Idle
            | ModbusBufferState::FillingRequest { .. }
            | ModbusBufferState::ValidRequest { .. }
            | ModbusBufferState::ValidResponse { .. }
            | ModbusBufferState::Transmitting { .. }
            | ModbusBufferState::Rx { .. } => None,
            ModbusBufferState::Tx { offset, size, .. } => {
                if offset < size {
                    let b = self.buffer[*offset];
                    *offset += 1;
                    Some(b)
                } else {
                    None
                }
            }
        }
    }

    pub fn tx_test_timeout(&mut self, bus_name: &'static str) -> bool {
        if self.owner.is_none() {
            panic!();
        }
        match &mut self.state {
            ModbusBufferState::Tx { .. } => {
                defmt::error!("{}: tx -> rx timeout", bus_name);
                self.reset();
                false
            }
            ModbusBufferState::Rx {
                timeout_counter, ..
            } => {
                if *timeout_counter == 0 {
                    defmt::error!("{}: tx -> rx timeout", bus_name);
                    self.reset();
                    false
                } else {
                    *timeout_counter -= 1;
                    true
                }
            }
            _ => false,
        }
    }

    pub fn start_rx(&mut self) {
        if self.owner.is_none() {
            panic!();
        }
        match self.state {
            ModbusBufferState::Idle
            | ModbusBufferState::FillingRequest { .. }
            | ModbusBufferState::ValidRequest { .. }
            | ModbusBufferState::ValidResponse { .. }
            | ModbusBufferState::Transmitting { .. }
            | ModbusBufferState::Rx { .. } => defmt::panic!("state: {}", self.state),
            ModbusBufferState::Tx {
                slave,
                slave_func_code,
                ..
            } => {
                self.state = ModbusBufferState::Rx {
                    offset: 0,
                    want_slave: slave,
                    want_func_code: slave_func_code,
                    timeout_counter: 0,
                };
            }
        }
    }

    pub fn feed_rx_byte(&mut self, data_byte: u8) -> Result<bool, &ModbusBufferState> {
        if self.owner.is_none() {
            panic!();
        }
        match &mut self.state {
            ModbusBufferState::Rx {
                offset,
                timeout_counter,
                ..
            } => {
                if *offset >= MODBUS_BUFFER_SIZE_MAX {
                    Ok(false)
                } else {
                    self.buffer[*offset] = data_byte;
                    *offset += 1;
                    *timeout_counter = 1;
                    Ok(true)
                }
            }
            ModbusBufferState::Idle
            | ModbusBufferState::FillingRequest { .. }
            | ModbusBufferState::ValidRequest { .. }
            | ModbusBufferState::Tx { .. }
            | ModbusBufferState::ValidResponse { .. }
            | ModbusBufferState::Transmitting { .. } => Err(&self.state),
        }
    }

    pub fn try_commit_response(&mut self) -> nb::Result<(&[u8], SlaveId), RxError> {
        if self.owner.is_none() {
            return Err(nb::Error::Other(RxError::NoOwner));
        }
        match self.state {
            ModbusBufferState::Idle => Err(nb::Error::Other(RxError::NoOwner)),
            ModbusBufferState::Rx {
                offset,
                want_slave,
                want_func_code,
                ..
            } => {
                if offset == 0 {
                    Err(nb::Error::WouldBlock)
                } else if offset == 1 {
                    if self.buffer[0] != want_slave {
                        Err(nb::Error::Other(RxError::IncorrectDeviceAddress {
                            want_slave,
                            got_slave: self.buffer[0],
                        }))
                    } else {
                        Err(nb::Error::WouldBlock)
                    }
                } else if offset == 2 {
                    if self.buffer[1] & 0x7f != want_func_code {
                        Err(nb::Error::Other(RxError::IncorrectFunction {
                            want_func_code,
                            got_func_code: self.buffer[1],
                        }))
                    } else {
                        Err(nb::Error::WouldBlock)
                    }
                } else {
                    match modbus_core::rtu::response_pdu_len(&self.buffer[..offset]) {
                        Ok(None) => Err(nb::Error::WouldBlock),
                        Ok(Some(pdu_len)) => {
                            if pdu_len
                                + core::mem::size_of::<SlaveId>()
                                + core::mem::size_of::<u16>()
                                <= offset
                            {
                                match modbus_core::rtu::extract_frame(
                                    &self.buffer[..offset],
                                    pdu_len,
                                ) {
                                    Ok(None) => Err(nb::Error::WouldBlock),
                                    Ok(Some(f)) => {
                                        self.state = ModbusBufferState::ValidResponse {
                                            size: offset,
                                            slave: f.slave,
                                            func_code: want_func_code,
                                        };
                                        Ok((&self.buffer[..offset], f.slave))
                                    }
                                    Err(e) => Err(nb::Error::Other(RxError::IncorrectPacket(e))),
                                }
                            } else {
                                Err(nb::Error::WouldBlock)
                            }
                        }
                        Err(e) => Err(nb::Error::Other(RxError::IncorrectPacket(e))),
                    }
                }
            }
            ModbusBufferState::ValidResponse { slave, size, .. } => {
                Ok((&self.buffer[..size], slave))
            }
            ModbusBufferState::FillingRequest { .. }
            | ModbusBufferState::ValidRequest { .. }
            | ModbusBufferState::Tx { .. }
            | ModbusBufferState::Transmitting { .. } => panic!(),
        }
    }
}

impl core::ops::Index<usize> for ModbusBuffer {
    type Output = u8;

    fn index(&self, index: usize) -> &Self::Output {
        &self.buffer[index]
    }
}
