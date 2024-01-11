use modbus_core::rtu::*;

pub const MODBUS_BUFFER_SIZE_MAX: usize = 256;

pub enum ModbusBufferState {
    Idle,
    FillingRequest {
        offset: usize,
    },
    ValidRequest {
        size: usize,
        slave: modbus_core::rtu::SlaveId,
    },
}

pub struct ModbusBuffer {
    buffer: [u8; MODBUS_BUFFER_SIZE_MAX],
    state: ModbusBufferState,
}

impl Default for ModbusBuffer {
    fn default() -> Self {
        Self {
            buffer: [0; MODBUS_BUFFER_SIZE_MAX],
            state: ModbusBufferState::Idle,
        }
    }
}

impl ModbusBuffer {
    pub fn reset(&mut self) {
        self.state = ModbusBufferState::Idle;
    }

    pub fn can_feed(&self) -> nb::Result<usize, ()> {
        match self.state {
            ModbusBufferState::Idle => Ok(MODBUS_BUFFER_SIZE_MAX),
            ModbusBufferState::FillingRequest { offset } => Ok(MODBUS_BUFFER_SIZE_MAX - offset),
            ModbusBufferState::ValidRequest { .. } => Err(nb::Error::WouldBlock),
        }
    }

    pub fn feed_me_to(&mut self) -> &mut [u8] {
        match self.state {
            ModbusBufferState::Idle => {
                self.state = ModbusBufferState::FillingRequest { offset: 0 };
                &mut self.buffer[..]
            }
            ModbusBufferState::FillingRequest { offset } => &mut self.buffer[offset..],
            ModbusBufferState::ValidRequest { .. } => panic!(),
        }
    }

    pub fn feed(&mut self, n: usize) {
        match &mut self.state {
            ModbusBufferState::Idle => panic!(),
            ModbusBufferState::FillingRequest { offset } => {
                if n + *offset > MODBUS_BUFFER_SIZE_MAX {
                    panic!();
                }
                *offset += n;
            }
            ModbusBufferState::ValidRequest { .. } => panic!(),
        }
    }

    pub fn try_commit_request(&mut self) -> Result<(&[u8], SlaveId), ()> {
        match self.state {
            ModbusBufferState::Idle => panic!(),
            ModbusBufferState::FillingRequest { offset } => {
                if let Ok(Some(req)) = server::decode_request(&self.buffer) {
                    self.state = ModbusBufferState::ValidRequest {
                        size: offset,
                        slave: req.hdr.slave,
                    };
                    Ok((&self.buffer[..offset], req.hdr.slave))
                } else {
                    Err(())
                }
            }
            ModbusBufferState::ValidRequest { size, slave } => Ok((&self.buffer[..size], slave)),
        }
    }
}
