use core::convert::Infallible;

use stm32f1xx_hal::{
    gpio::{Output, Pin, PushPull},
    rcc::Clocks,
    serial::{Config, Event, Rx, Tx},
};
use systick_monotonic::{fugit::TimerInstantU64, ExtU64};

const SWITCH_TX_DELAY_MS: u64 = 5;

#[derive(defmt::Format, Clone, Copy, PartialEq, Eq)]
pub enum UartHalfDuplexState {
    Rx,
    Tx,
}

pub struct UartHalfDuplex<const P: char, const N: u8, USART, PINS, const FREQ_HZ: u32> {
    last_tx_ts: TimerInstantU64<FREQ_HZ>,
    state: UartHalfDuplexState,
    re_de: Pin<P, N, Output<PushPull>>,
    uart: stm32f1xx_hal::serial::Serial<USART, PINS>,
}

impl<const P: char, const N: u8, USART, PINS, const FREQ_HZ: u32>
    UartHalfDuplex<P, N, USART, PINS, FREQ_HZ>
where
    USART: stm32f1xx_hal::serial::Instance,
{
    pub fn new(
        uart: stm32f1xx_hal::serial::Serial<USART, PINS>,
        mut re_de: Pin<P, N, Output<PushPull>>,
    ) -> Self {
        re_de.set_low();
        Self {
            last_tx_ts: TimerInstantU64::from_ticks(0),
            state: UartHalfDuplexState::Rx,
            re_de,
            uart,
        }
    }

    pub fn reconfigure(
        &mut self,
        config: impl Into<Config>,
        clocks: &Clocks,
    ) -> nb::Result<(), Infallible> {
        self.uart.unlisten(Event::Rxne);
        self.uart.unlisten(Event::Txe);

        let res = self.uart.reconfigure(config, clocks);

        self.switch_state(self.state, self.last_tx_ts + SWITCH_TX_DELAY_MS.millis())
            .ok();

        res
    }

    pub fn switch_state(
        &mut self,
        state: UartHalfDuplexState,
        now: TimerInstantU64<FREQ_HZ>,
    ) -> Result<(), ()> {
        match state {
            UartHalfDuplexState::Rx => {
                self.uart.unlisten(Event::Txe);
                self.uart.tx.bflush().ok(); // flush tx // TODO: wait for idle?
                while let Ok(_) = self.uart.rx.read() {} // flush rx

                self.re_de.set_low();
                self.uart.listen(Event::Rxne);

                self.last_tx_ts = now;
            }
            UartHalfDuplexState::Tx => {
                if self.last_tx_ts + SWITCH_TX_DELAY_MS.millis() > now {
                    return Err(());
                }

                self.uart.unlisten(Event::Rxne);
                self.re_de.set_high();
                self.uart.listen(Event::Txe);
            }
        }
        self.state = state;

        Ok(())
    }

    pub fn reset_tx_delay(&mut self, now: TimerInstantU64<FREQ_HZ>) {
        self.last_tx_ts = now;
    }

    pub fn rx(&mut self) -> Option<&mut Rx<USART>> {
        if self.state == UartHalfDuplexState::Rx {
            Some(&mut self.uart.rx)
        } else {
            None
        }
    }

    pub fn rx_irq(&mut self) -> Option<&mut Rx<USART>> {
        if self.state == UartHalfDuplexState::Rx && self.uart.is_rx_not_empty() {
            Some(&mut self.uart.rx)
        } else {
            None
        }
    }

    pub fn tx(&mut self) -> Option<&mut Tx<USART>> {
        if self.state == UartHalfDuplexState::Tx {
            Some(&mut self.uart.tx)
        } else {
            None
        }
    }

    pub fn tx_irq(&mut self) -> Option<&mut Tx<USART>> {
        if self.state == UartHalfDuplexState::Tx && self.uart.is_tx_empty() {
            Some(&mut self.uart.tx)
        } else {
            None
        }
    }

    pub fn switch_tx(&mut self, now: TimerInstantU64<FREQ_HZ>) -> Result<&mut Tx<USART>, ()> {
        self.switch_state(UartHalfDuplexState::Tx, now)
            .map(|_| &mut self.uart.tx)
    }

    pub fn switch_rx(&mut self, now: TimerInstantU64<FREQ_HZ>) -> &mut Rx<USART> {
        self.switch_state(UartHalfDuplexState::Rx, now).ok();
        &mut self.uart.rx
    }
}
