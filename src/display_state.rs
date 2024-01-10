use core::{fmt::Display, ops::Add};

use alloc::{boxed::Box, format, string::String};
use embedded_graphics::{
    geometry::{AnchorPoint, Dimensions, Point, Size},
    mono_font::{self, MonoTextStyle, MonoTextStyleBuilder},
    pixelcolor::BinaryColor,
    primitives::{Circle, Line, Primitive, PrimitiveStyle, Rectangle},
    text::{self, renderer::CharacterStyle, Alignment, Baseline, Text, TextStyleBuilder},
    Drawable,
};

use ssd1309::mode::GraphicsMode;
use systick_monotonic::fugit::TimerInstantU64;

use crate::devices::ValuesStorage;

use crate::support::format_float_simple;

#[derive(PartialEq, Eq, Clone, Copy)]
pub enum ScanState {
    UART(u8),
    RS485(u8),
    I2C(u8),
}

impl ScanState {
    pub fn bus_name(&self) -> &'static str {
        Self::bus_name_from_usize(self.to_selected_current())
    }

    pub fn bus_name_from_usize(bus: usize) -> &'static str {
        match bus {
            0 => "232",
            1 => "485",
            2 => "I2C",
            _ => panic!(),
        }
    }

    pub fn bus_address(&self) -> String {
        let addr = match self {
            ScanState::UART(addr) => *addr,
            ScanState::RS485(addr) => *addr,
            ScanState::I2C(addr) => *addr,
        };
        format!("0x{:02X}", addr)
    }

    pub fn to_selected_current(&self) -> usize {
        match self {
            ScanState::UART(_) => 0,
            ScanState::RS485(_) => 1,
            ScanState::I2C(_) => 2,
        }
    }
}

enum State<const FREQ_HZ: u32> {
    /// После включения устройства на 3 секунды
    WellcomeScreen {
        end_at: TimerInstantU64<FREQ_HZ>,
        current_animation_step: u32,
    },
    /// Ожидание подключения устройств к портам, мониторинг потребляемого тока
    CurrentMonitoringScreen,
    /// Обнаружено подключение, сканирование шины
    DetectingScreen(ScanState),
    /// Отображение показаний найденного устройства
    OutputDisplayScreen {
        value_storage: Box<dyn ValuesStorage>,
    },
    /// Устройство отключено, заглдушка на 2 секунды и переход к CurrentMonitoringScreen
    DisconnectScreen,
}

impl Display for ScanState {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            ScanState::UART(addr) => write!(f, "Сканирование\nШины RS232\n0x{:02X}", addr),
            ScanState::RS485(addr) => write!(f, "Сканирование\nШины RS485\n0x{:02X}", addr),
            ScanState::I2C(addr) => write!(f, "Сканирование\nШины I2C\n0x{:02X}", addr),
        }
    }
}

pub struct DisplayState<const FREQ_HZ: u32> {
    state: State<FREQ_HZ>,
    big_font: MonoTextStyle<'static, BinaryColor>,
    small_font: MonoTextStyle<'static, BinaryColor>,
    inv_small_font: MonoTextStyle<'static, BinaryColor>,
    small_font_italic: MonoTextStyle<'static, BinaryColor>,
    prev_scan_state: Option<ScanState>,
}

impl<const FREQ_HZ: u32> DisplayState<FREQ_HZ> {
    pub fn init(wellcome_before: TimerInstantU64<FREQ_HZ>) -> Self {
        let big_font = MonoTextStyleBuilder::new()
            .font(&mono_font::iso_8859_5::FONT_9X18_BOLD)
            .text_color(BinaryColor::On)
            .build();

        let small_font = MonoTextStyleBuilder::new()
            .font(&mono_font::iso_8859_5::FONT_6X13)
            .text_color(BinaryColor::On)
            .build();

        let mut inv_small_font = small_font.clone();
        inv_small_font.set_background_color(Some(BinaryColor::On));
        inv_small_font.set_text_color(Some(BinaryColor::Off));

        let small_font_italic = MonoTextStyleBuilder::new()
            .font(&mono_font::iso_8859_5::FONT_6X13_ITALIC)
            .text_color(BinaryColor::On)
            .build();

        Self {
            state: State::WellcomeScreen {
                end_at: wellcome_before,
                current_animation_step: 0,
            },
            big_font,
            small_font,
            inv_small_font,
            small_font_italic,
            prev_scan_state: None,
        }
    }

    pub fn render<DI>(
        &self,
        disp: &mut GraphicsMode<DI>,
        _time: TimerInstantU64<FREQ_HZ>,
    ) -> Result<(), display_interface::DisplayError>
    where
        DI: display_interface::WriteOnlyDataCommand,
    {
        match &self.state {
            State::WellcomeScreen {
                end_at: _,
                current_animation_step,
            } => self.render_wellcome_screen(disp, *current_animation_step),
            State::CurrentMonitoringScreen => self.render_current_monitoring_screen(disp),
            State::DetectingScreen(s) => self.render_detecting_screen(disp, s),
            State::OutputDisplayScreen { value_storage: vs } => {
                self.render_output_display_screen(disp, vs.as_ref())
            }
            State::DisconnectScreen => self.render_disconnect_screen(disp),
        }
    }

    pub fn display_output(&mut self, value_storage: Box<dyn ValuesStorage>) {
        if let State::WellcomeScreen {
            end_at: _,
            current_animation_step: _,
        } = self.state
        {
            /* Ignore */
        } else {
            self.state = State::OutputDisplayScreen { value_storage };
        }
    }

    pub fn scan(&mut self, state: ScanState) {
        match &self.state {
            State::CurrentMonitoringScreen
            | State::DetectingScreen(_)
            | State::DisconnectScreen => {
                self.state = State::DetectingScreen(state);
            }
            _ => { /* Ignore */ }
        }
    }

    fn render_wellcome_screen<DI>(
        &self,
        display: &mut GraphicsMode<DI>,
        animation_step: u32,
    ) -> Result<(), display_interface::DisplayError>
    where
        DI: display_interface::WriteOnlyDataCommand,
    {
        defmt::trace!("render_wellcome_screen(_, {})", animation_step);

        let (display_w, display_h) = {
            let d = display.get_dimensions();
            (d.0 as i32, d.1 as i32)
        };

        Text::with_text_style(
            "Пробник",
            Point::new(display_w / 2, -3),
            self.big_font,
            TextStyleBuilder::new()
                .alignment(Alignment::Center)
                .baseline(Baseline::Top)
                .build(),
        )
        .draw(display)?;

        let _y = self.big_font.font.character_size.height as i32 + 10;
        Text::with_alignment(
            "логический\nпереходник",
            Point::new(display_w / 2, _y),
            self.small_font,
            Alignment::Center,
        )
        .draw(display)?;
        Line::new(
            Point::new(display_w / 10, _y + 3),
            Point::new(display_w * 9 / 10, _y + 3),
        )
        .into_styled(PrimitiveStyle::with_stroke(BinaryColor::On, 1))
        .draw(display)?;
        Text::with_text_style(
            "СКТБ ЭлПА\n2024",
            Point::new(
                display_w / 2,
                display_h - self.small_font_italic.font.character_size.height as i32,
            ),
            self.small_font_italic,
            TextStyleBuilder::new()
                .alignment(Alignment::Center)
                .baseline(Baseline::Bottom)
                .build(),
        )
        .draw(display)?;

        const CIRCLES_COUNT: i32 = 4;
        let circles_y = display_h * 2 / 3;
        let circles_x_start = display_w / 4;
        let curcles_step = display_w / (2 * CIRCLES_COUNT);
        const DIAMETER_TABLE: [u8; 11] = [3, 4, 5, 7, 9, 10, 9, 8, 5, 4, 3];
        const ANIMATION_OFFSET: i32 = (DIAMETER_TABLE.len() - 3) as i32;

        for c in 0..CIRCLES_COUNT {
            let circle_x = circles_x_start + (CIRCLES_COUNT - 1 - c) * curcles_step;
            let animation_step = animation_step as i32 - ANIMATION_OFFSET * c;
            let diameter = if animation_step < 0 || animation_step >= DIAMETER_TABLE.len() as i32 {
                3
            } else {
                DIAMETER_TABLE[animation_step as usize]
            };
            let circle = Circle::with_center(Point::new(circle_x, circles_y), diameter as u32)
                .into_styled(PrimitiveStyle::with_fill(BinaryColor::On));
            circle.draw(display)?;
        }

        Ok(())
    }

    fn render_current_monitoring_screen<DI>(
        &self,
        _display: &mut GraphicsMode<DI>,
    ) -> Result<(), display_interface::DisplayError>
    where
        DI: display_interface::WriteOnlyDataCommand,
    {
        defmt::trace!("render_current_monitoring_screen");
        Ok(())
    }

    fn render_detecting_screen<DI>(
        &self,
        display: &mut GraphicsMode<DI>,
        scan_state: &ScanState,
    ) -> Result<(), display_interface::DisplayError>
    where
        DI: display_interface::WriteOnlyDataCommand,
    {
        defmt::trace!("render_detecting_screen");

        let (display_w, display_h) = {
            let d = display.get_dimensions();
            (d.0 as i32, d.1 as i32)
        };

        // draw frame
        let rect = Rectangle::with_center(
            Point::new(display_w / 2, display_h / 3),
            Size::new(display_w as u32 - 2, (display_h / 2) as u32),
        );
        rect.into_styled(PrimitiveStyle::with_stroke(BinaryColor::On, 1))
            .draw(display)
            .unwrap();

        let inner_frame = rect.resized(rect.size.add(Size::new(2, 2)), AnchorPoint::Center);
        inner_frame
            .into_styled(PrimitiveStyle::with_stroke(BinaryColor::On, 1))
            .draw(display)
            .unwrap();

        // draw bus name
        Text::with_text_style(
            format!(
                "Скан\n{}\n{}",
                scan_state.bus_name(),
                scan_state.bus_address()
            )
            .as_str(),
            Point::new(display_w / 2, inner_frame.top_left.y + 5),
            self.big_font,
            TextStyleBuilder::new()
                .alignment(Alignment::Center)
                .baseline(Baseline::Top)
                .build(),
        )
        .draw(display)?;

        self.render_current_info(
            display,
            [15.3, 10.8, 3.2],
            Some(scan_state.to_selected_current()),
            Point::new(
                0,
                display_h - self.small_font.font.character_size.height as i32 * 3,
            ),
        )
    }

    fn render_current_info<DI>(
        &self,
        display: &mut GraphicsMode<DI>,
        currents: [f32; 3],
        selected_current: Option<usize>,
        initial_pos: Point,
    ) -> Result<(), display_interface::DisplayError>
    where
        DI: display_interface::WriteOnlyDataCommand,
    {
        for c in 0..currents.len() {
            let string = format!(
                "{}: {}",
                ScanState::bus_name_from_usize(c),
                format_float_simple(currents[c], 1)
            );
            let text = Text::with_text_style(
                string.as_str(),
                initial_pos.add(Size::new(
                    0,
                    self.small_font.font.character_size.height * c as u32,
                )),
                if Some(c) == selected_current {
                    self.inv_small_font
                } else {
                    self.small_font
                },
                TextStyleBuilder::new()
                    .alignment(Alignment::Left)
                    .baseline(Baseline::Top)
                    .build(),
            );
            text.draw(display)?;
        }

        Ok(())
    }

    fn render_output_display_screen<DI>(
        &self,
        display: &mut GraphicsMode<DI>,
        value_storage: &dyn ValuesStorage,
    ) -> Result<(), display_interface::DisplayError>
    where
        DI: display_interface::WriteOnlyDataCommand,
    {
        defmt::trace!("render_output_display_screen");

        let (display_w, display_h) = {
            let d = display.get_dimensions();
            (d.0 as i32, d.1 as i32)
        };

        display
            .bounding_box()
            .into_styled(PrimitiveStyle::with_stroke(BinaryColor::On, 1))
            .draw(display)
            .unwrap();

        if let Some(ss) = &self.prev_scan_state {
            let ts_center = TextStyleBuilder::new()
                .alignment(Alignment::Center)
                .baseline(Baseline::Top)
                .build();
            let ts_left = TextStyleBuilder::new()
                .alignment(Alignment::Left)
                .baseline(Baseline::Top)
                .build();

            let mut pos = Text::with_text_style(
                value_storage.sender_id().as_str(),
                Point::new(display_w / 2, 1),
                self.big_font,
                ts_center,
            )
            .draw(display)?;

            {
                let txt = format!("{} {}", ss.bus_name(), ss.bus_address());
                let text = Text::with_text_style(
                    txt.as_str(),
                    Point::new(
                        display_w / 2,
                        pos.y + self.big_font.font.character_size.height as i32,
                    ),
                    self.small_font,
                    ts_center,
                );
                pos = text.draw(display)?;
                Line::new(
                    Point::new(0, pos.y - 1),
                    Point::new(display_w - 1, pos.y - 1),
                )
                .into_styled(PrimitiveStyle::with_stroke(BinaryColor::On, 1))
                .draw(display)?;

                let bot = text.bounding_box().bottom_right().unwrap_or_default();
                Line::new(Point::new(0, bot.y), Point::new(display_w - 1, bot.y))
                    .into_styled(PrimitiveStyle::with_stroke(BinaryColor::On, 1))
                    .draw(display)?;
            };

            let field_width =
                (display_w as u32 - 4) / self.small_font.font.character_size.width as u32;

            pos.y += 3;

            for v in value_storage.render(field_width) {
                pos = Text::with_text_style(
                    v.as_str(),
                    Point::new(
                        2,
                        pos.y + self.small_font.font.character_size.height as i32 - 2,
                    ),
                    self.small_font,
                    ts_left,
                )
                .draw(display)?;
            }

            // Ток потребления
            {
                let txt = format!(
                    "I={:>w$}mA",
                    format_float_simple(10.2, 1),
                    w = field_width as usize - 2
                );
                let text = Text::with_text_style(
                    txt.as_str(),
                    Point::new(2, display_h - 1),
                    self.small_font,
                    TextStyleBuilder::new()
                        .alignment(Alignment::Left)
                        .baseline(Baseline::Bottom)
                        .build(),
                );
                text.draw(display)?;

                let y = text.bounding_box().top_left.y - 1;
                Line::new(Point::new(0, y), Point::new(display_w - 1, y))
                    .into_styled(PrimitiveStyle::with_stroke(BinaryColor::On, 1))
                    .draw(display)?;
            }
        }

        Ok(())
    }

    fn render_disconnect_screen<DI>(
        &self,
        _display: &mut GraphicsMode<DI>,
    ) -> Result<(), display_interface::DisplayError>
    where
        DI: display_interface::WriteOnlyDataCommand,
    {
        defmt::trace!("render_disconnect_screen");
        Ok(())
    }

    pub fn animate(&mut self, time: TimerInstantU64<FREQ_HZ>) -> bool {
        match &mut self.state {
            State::WellcomeScreen {
                end_at,
                current_animation_step,
            } => {
                if time >= *end_at {
                    self.state = State::CurrentMonitoringScreen;
                    true
                } else {
                    let an_s = end_at
                        .checked_duration_since(time)
                        .map(|d| (d.to_millis() / 100) as u32)
                        .unwrap_or(0);
                    let new_an_step = an_s != *current_animation_step;
                    if new_an_step {
                        *current_animation_step = an_s;
                    }
                    new_an_step
                }
            }
            State::DetectingScreen(s) => {
                self.prev_scan_state = Some(*s);
                false
            }
            _ => false,
        }
    }
}
