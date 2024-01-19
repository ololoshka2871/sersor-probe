use core::ops::{BitAnd, BitOrAssign};

use alloc::{collections::LinkedList, vec::Vec};
use modbus_core::rtu::{ResponseAdu, SlaveId};
use systick_monotonic::fugit::{TimerDurationU64, TimerInstantU64};

#[derive(Debug, defmt::Format, Clone, Copy, PartialEq)]
pub enum Requester {
    Device,
    USB,
    Both,
}

#[derive(Clone, PartialEq)]
pub enum RequestState<const FREQ_HZ: u32> {
    Stored,
    Tx(usize),
    WaitingForResponse,
    Success {
        data: Vec<u8>,
        ts: TimerInstantU64<FREQ_HZ>,
    },
}

impl BitOrAssign for Requester {
    fn bitor_assign(&mut self, rhs: Self) {
        *self = match (*self, rhs) {
            (Requester::Device, Requester::Device) => Requester::Device,
            (Requester::USB, Requester::USB) => Requester::USB,
            _ => Requester::Both,
        }
    }
}

impl BitAnd for Requester {
    type Output = bool;

    fn bitand(self, rhs: Self) -> Self::Output {
        match (self, rhs) {
            (Requester::Device, Requester::Device) => true,
            (Requester::USB, Requester::USB) => true,
            (Requester::Both, _) => true,
            _ => false,
        }
    }
}

struct Request<const FREQ_HZ: u32> {
    req_data: Vec<u8>,
    requester: Requester,
    state: RequestState<FREQ_HZ>,
    start: TimerInstantU64<FREQ_HZ>,
}

pub struct ModbusDispatcher<const FREQ_HZ: u32> {
    pending_requests: LinkedList<Request<FREQ_HZ>>,
    max_size: usize,
    timeout: TimerDurationU64<FREQ_HZ>,
}

impl<const FREQ_HZ: u32> ModbusDispatcher<FREQ_HZ> {
    pub fn new(max_size: usize, timeout: TimerDurationU64<FREQ_HZ>) -> Self {
        Self {
            pending_requests: LinkedList::new(),
            max_size,
            timeout,
        }
    }

    fn find_request(
        &mut self,
        requester: Requester,
        device: SlaveId,
        function: u8,
    ) -> Option<&mut Request<FREQ_HZ>> {
        self.pending_requests.iter_mut().find(|req| {
            let req_hdr = req.req_data.as_slice();
            let req_device = req_hdr[0];
            let req_function = req_hdr[1];
            let req_requester = req.requester;

            req_device == device
                && req_function == function
                && (req_requester == requester || req_requester == Requester::Both)
        })
    }

    fn try_cleanup(&mut self, now: TimerInstantU64<FREQ_HZ>) -> bool {
        if let Some(idx) = self
            .pending_requests
            .iter()
            .position(|req| req.start + self.timeout < now)
        {
            self.pending_requests.remove(idx);
            true
        } else {
            false
        }
    }

    pub fn push_raw_request<'r>(
        &mut self,
        buf: &[u8],
        requester: Requester,
        now: TimerInstantU64<FREQ_HZ>,
    ) -> bool {
        let req_device = buf[0];
        let req_function = buf[1];

        if let Some(req) = self.find_request(requester, req_device, req_function) {
            if !(req.requester & requester) {
                req.start = now;
                req.requester |= requester;
            }
            defmt::trace!("Pushed already existing Modbus request");
            true
        } else {
            if self.pending_requests.len() == self.max_size && !self.try_cleanup(now) {
                return false;
            }

            self.pending_requests.push_back(Request {
                req_data: buf.to_vec(),
                requester,
                state: RequestState::Stored,
                start: now,
            });
            defmt::trace!("Pushed new Modbus request");

            true
        }
    }

    pub fn push_request<'r>(
        &mut self,
        req: &modbus_core::rtu::RequestAdu<'r>,
        requester: Requester,
        now: TimerInstantU64<FREQ_HZ>,
    ) -> bool {
        let len = req.pdu.0.pdu_len()
            + core::mem::size_of_val(&req.hdr.slave)
            + core::mem::size_of::<u16>();
        let mut buf = Vec::with_capacity(len);
        buf.resize(len, 0);

        modbus_core::rtu::client::encode_request(req, &mut buf).unwrap();

        self.push_raw_request(&buf, requester, now)
    }

    pub fn ready_tx(&self) -> bool {
        let (stored_count, tx_count, ready_count) =
            self.pending_requests
                .iter()
                .fold((0usize, 0usize, 0usize), |mut counters, req| {
                    match &req.state {
                        RequestState::Stored => {
                            counters.0 += 1;
                        }
                        RequestState::Tx(_) => {
                            counters.1 += 1;
                        }
                        RequestState::Success { .. } => {
                            counters.2 += 1;
                        }
                        _ => {}
                    }
                    counters
                });
        tx_count == 0 && ready_count == 0 && stored_count > 0
    }

    pub fn start_tx(&mut self) -> u8 {
        for r in self.pending_requests.iter_mut() {
            if RequestState::Stored == r.state {
                r.state = RequestState::Tx(1);
                return r.req_data[0];
            }
        }
        panic!()
    }

    pub fn next_tx(&mut self) -> Result<u8, Option<TimerInstantU64<FREQ_HZ>>> {
        for r in self.pending_requests.iter_mut() {
            if let RequestState::Tx(offset) = r.state {
                if offset < r.req_data.len() {
                    r.state = RequestState::Tx(offset + 1);
                    return Ok(r.req_data[offset]);
                } else {
                    r.state = RequestState::WaitingForResponse;
                    return Err(Some(r.start));
                }
            }
        }
        Err(None)
    }

    pub fn dispatch_response<'r>(
        &mut self,
        data: &[u8],
        adu: ResponseAdu<'r>,
        now: TimerInstantU64<FREQ_HZ>,
    ) {
        for r in self.pending_requests.iter_mut() {
            if (r.state == RequestState::WaitingForResponse)
                && (r.req_data[0] == adu.hdr.slave)
                && (r.req_data[1] == data[1] & 0x7F)
            {
                if r.start + self.timeout > now {
                    r.state = RequestState::Success {
                        data: data.to_vec(),
                        ts: now,
                    };
                } else {
                    defmt::error!("Request {} timeouted", r.start.ticks());
                }
                return;
            }
        }
    }

    pub fn try_take_resp(&mut self) -> Option<(Requester, Vec<u8>, TimerInstantU64<FREQ_HZ>)> {
        if let Some(idx) = self
            .pending_requests
            .iter()
            .position(|req| matches!(req.state, RequestState::Success { .. }))
        {
            let r = self.pending_requests.remove(idx);
            if let RequestState::Success { data, ts } = r.state {
                defmt::trace!("Took response for request T{}", r.start.ticks());
                Some((r.requester, data, ts))
            } else {
                None
            }
        } else {
            None
        }
    }
}
