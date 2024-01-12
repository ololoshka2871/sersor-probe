
#[derive(Copy, Clone)]
pub struct MyLineCoding {
    pub stop_bits: usbd_serial::StopBits,
    pub data_bits: u8,
    pub parity_type: usbd_serial::ParityType,
    pub data_rate: u32,
}

impl defmt::Format for MyLineCoding {
    fn format(&self, fmt: defmt::Formatter) {
        use usbd_serial::{ParityType, StopBits};

        defmt::write!(
            fmt,
            "LineCoding {{ stop_bits: {}, data_bits: {}, parity_type: {}, data_rate: {} }}",
            match self.stop_bits {
                StopBits::One => "1",
                StopBits::OnePointFive => "1.5",
                StopBits::Two => "2",
            },
            self.data_bits,
            match self.parity_type {
                ParityType::None => "None",
                ParityType::Odd => "Odd",
                ParityType::Event => "Even",
                ParityType::Mark => "Mark",
                ParityType::Space => "Space",
            },
            self.data_rate
        );
    }
}

impl Into<stm32f1xx_hal::serial::Config> for MyLineCoding {
    fn into(self) -> stm32f1xx_hal::serial::Config {
        use stm32f1xx_hal::serial::{Parity, StopBits as SB};
        use usbd_serial::{ParityType, StopBits};

        stm32f1xx_hal::serial::Config::default()
            .baudrate(stm32f1xx_hal::time::Bps(self.data_rate))
            .parity(match self.parity_type {
                ParityType::Odd => Parity::ParityOdd,
                ParityType::Event => Parity::ParityEven,
                _ => Parity::ParityNone,
            })
            .stopbits(match self.stop_bits {
                StopBits::One => SB::STOP1,
                StopBits::OnePointFive => SB::STOP1P5,
                StopBits::Two => SB::STOP2,
            })
    }
}
