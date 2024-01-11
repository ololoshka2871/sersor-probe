use core::{
    fmt::Display,
    ops::{Add, Index},
};

use alloc::{boxed::Box, format, string::String};
use embedded_graphics::{
    geometry::{Dimensions, Point, Size},
    mono_font::{self, MonoTextStyle, MonoTextStyleBuilder},
    pixelcolor::BinaryColor,
    primitives::{Circle, Line, Primitive, PrimitiveStyle, Rectangle},
    text::{renderer::CharacterStyle, Alignment, Baseline, Text, TextStyleBuilder},
    transform::Transform,
    Drawable,
};

use ssd1309::mode::GraphicsMode;
use systick_monotonic::{fugit::TimerInstantU64, ExtU64};

use crate::{devices::ValuesStorage, support::RectangleExt};

use crate::support::format_float_simple;

#[derive(Clone, Copy, Default)]
pub struct CurrentValues {
    pub uart: f32,
    pub rs485: f32,
    pub i2c: f32,
}

impl CurrentValues {
    pub fn update(&mut self, current_values: [f32; 3]) {
        self.uart = current_values[0];
        self.rs485 = current_values[1];
        self.i2c = current_values[2];
    }

    const fn len(&self) -> usize {
        3
    }
}

impl From<[f32; 3]> for CurrentValues {
    fn from(current_values: [f32; 3]) -> Self {
        Self {
            uart: current_values[0],
            rs485: current_values[1],
            i2c: current_values[2],
        }
    }
}

impl Index<usize> for CurrentValues {
    type Output = f32;

    fn index(&self, index: usize) -> &Self::Output {
        match index {
            0 => &self.uart,
            1 => &self.rs485,
            2 => &self.i2c,
            _ => panic!(),
        }
    }
}

impl Index<&'static str> for CurrentValues {
    type Output = f32;

    fn index(&self, index: &'static str) -> &Self::Output {
        match index {
            "232" => &self.uart,
            "485" => &self.rs485,
            "I2C" => &self.i2c,
            _ => panic!(),
        }
    }
}

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

    pub fn reset(&mut self) {
        match self {
            ScanState::UART(a) => *a = 0,
            ScanState::RS485(a) => *a = 0,
            ScanState::I2C(a) => *a = 0,
        }
    }

    pub fn is_reset(&self) -> bool {
        match self {
            ScanState::UART(a) | ScanState::RS485(a) | ScanState::I2C(a) => *a == 0,
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
    CurrentMonitoringScreen {
        start: TimerInstantU64<FREQ_HZ>,
        star_base_pos: Point,
        current_animation_step: u32,
    },
    /// Обнаружено подключение, сканирование шины
    DetectingScreen(ScanState),
    /// Отображение показаний найденного устройства
    OutputDisplayScreen {
        value_storage: Box<dyn ValuesStorage>,
    },
    /// Устройство отключено, заглдушка на 2 секунды и переход к CurrentMonitoringScreen
    DisconnectScreen {
        end_at: TimerInstantU64<FREQ_HZ>,
        current_animation_step: u32,
    },
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
    current_values: CurrentValues,
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
            current_values: CurrentValues::default(),
        }
    }

    pub fn render<DI>(
        &self,
        disp: &mut GraphicsMode<DI>,
        time: TimerInstantU64<FREQ_HZ>,
    ) -> Result<(), display_interface::DisplayError>
    where
        DI: display_interface::WriteOnlyDataCommand,
    {
        match &self.state {
            State::WellcomeScreen {
                current_animation_step,
                ..
            } => self.render_wellcome_screen(disp, *current_animation_step),
            State::CurrentMonitoringScreen {
                start,
                star_base_pos,
                current_animation_step,
            } => self.render_current_monitoring_screen(
                disp,
                *start,
                star_base_pos,
                time,
                *current_animation_step,
            ),
            State::DetectingScreen(s) => self.render_detecting_screen(disp, s),
            State::OutputDisplayScreen { value_storage: vs } => {
                self.render_output_display_screen(disp, vs.as_ref())
            }
            State::DisconnectScreen {
                current_animation_step,
                ..
            } => self.render_disconnect_screen(disp, *current_animation_step),
        }
    }

    pub fn display_output(&mut self, value_storage: Box<dyn ValuesStorage>) {
        match self.state {
            State::DetectingScreen(_) | State::OutputDisplayScreen { .. } => {
                self.state = State::OutputDisplayScreen { value_storage };
            }
            _ => { /* Ignore */ }
        }
    }

    pub fn scan(&mut self, state: ScanState) {
        match &self.state {
            State::CurrentMonitoringScreen { .. }
            | State::DetectingScreen(_)
            | State::DisconnectScreen { .. } => {
                self.state = State::DetectingScreen(state);
            }
            _ => { /* Ignore */ }
        }
    }

    pub fn update_current(&mut self, current_values: CurrentValues) -> Option<ScanState> {
        self.current_values = current_values;

        match &self.state {
            // идет поиск, пусть идет дальше
            State::DetectingScreen(ss) => Some(*ss),
            // одет съем показаний, если ток больше crate::config::CURRENT_THRESHOLD_MA, то путь идет дальше
            State::OutputDisplayScreen { .. } => {
                if let Some(ss) = &self.prev_scan_state {
                    if self.current_values[ss.bus_name()] < crate::config::CURRENT_THRESHOLD_MA {
                        None
                    } else {
                        Some(*ss)
                    }
                } else {
                    None
                }
            }
            // идет мониторинг, если ток больше crate::config::CURRENT_THRESHOLD_MA, то начинаем сканирование это шины
            State::CurrentMonitoringScreen { .. } => {
                if current_values[0] > crate::config::CURRENT_THRESHOLD_MA {
                    Some(ScanState::UART(0))
                } else if current_values[1] > crate::config::CURRENT_THRESHOLD_MA {
                    Some(ScanState::RS485(0))
                } else if current_values[2] > crate::config::CURRENT_THRESHOLD_MA {
                    Some(ScanState::I2C(0))
                } else {
                    None
                }
            }
            // остальные состояния не требуют обновления
            State::WellcomeScreen { .. } | State::DisconnectScreen { .. } => None,
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
        display: &mut GraphicsMode<DI>,
        start: TimerInstantU64<FREQ_HZ>,
        star_base_pos: &Point,
        time: TimerInstantU64<FREQ_HZ>,
        current_animation_step: u32,
    ) -> Result<(), display_interface::DisplayError>
    where
        DI: display_interface::WriteOnlyDataCommand,
    {
        let (display_w, display_h) = {
            let d = display.get_dimensions();
            (d.0 as i32, d.1 as i32)
        };

        if time > start + crate::config::SCREENSAVER_TIMEOUT_MS.millis() {
            defmt::trace!("render_screensaver");
            fn draw_cross<DI: display_interface::WriteOnlyDataCommand>(
                display: &mut GraphicsMode<DI>,
                mut center: Point,
                size: Size,
                fill_color: BinaryColor,
                dimensions: (i32, i32),
            ) -> Result<(), display_interface::DisplayError> {
                center.x %= dimensions.0;
                center.y %= dimensions.1;

                Line::new(
                    Point::new(center.x - size.width as i32 / 2, center.y),
                    Point::new(center.x + size.width as i32 / 2, center.y),
                )
                .into_styled(PrimitiveStyle::with_stroke(fill_color, 1))
                .draw(display)?;

                Line::new(
                    Point::new(center.x, center.y - size.height as i32 / 2),
                    Point::new(center.x, center.y + size.height as i32 / 2),
                )
                .into_styled(PrimitiveStyle::with_stroke(fill_color, 1))
                .draw(display)
            }

            let second_star_base = Point::new(star_base_pos.y, star_base_pos.x);
            match current_animation_step {
                0 => {
                    Rectangle::with_center(
                        Point::new(star_base_pos.x % display_w, star_base_pos.y % display_h),
                        Size::new(1, 1),
                    )
                    .into_styled(PrimitiveStyle::with_fill(BinaryColor::On))
                    .draw(display)?;

                    draw_cross(
                        display,
                        second_star_base,
                        Size::new(10, 10),
                        BinaryColor::On,
                        (display_w, display_h),
                    )?;
                    draw_cross(
                        display,
                        second_star_base,
                        Size::new(4, 4),
                        BinaryColor::Off,
                        (display_w, display_h),
                    )?;
                }
                1 => {
                    draw_cross(
                        display,
                        *star_base_pos,
                        Size::new(4, 4),
                        BinaryColor::On,
                        (display_w, display_h),
                    )?;

                    draw_cross(
                        display,
                        second_star_base,
                        Size::new(8, 8),
                        BinaryColor::On,
                        (display_w, display_h),
                    )?;
                }
                2 => {
                    draw_cross(
                        display,
                        *star_base_pos,
                        Size::new(8, 8),
                        BinaryColor::On,
                        (display_w, display_h),
                    )?;

                    draw_cross(
                        display,
                        second_star_base,
                        Size::new(4, 4),
                        BinaryColor::On,
                        (display_w, display_h),
                    )?;
                }
                3 => {
                    draw_cross(
                        display,
                        *star_base_pos,
                        Size::new(10, 10),
                        BinaryColor::On,
                        (display_w, display_h),
                    )?;
                    draw_cross(
                        display,
                        *star_base_pos,
                        Size::new(4, 4),
                        BinaryColor::Off,
                        (display_w, display_h),
                    )?;

                    Rectangle::with_center(
                        Point::new(
                            second_star_base.x % display_w,
                            second_star_base.y % display_h,
                        ),
                        Size::new(1, 1),
                    )
                    .into_styled(PrimitiveStyle::with_fill(BinaryColor::On))
                    .draw(display)?;
                }
                _ => {}
            }

            Ok(())
        } else {
            defmt::trace!("render_current_monitoring_screen");
            Text::with_text_style(
                "Жду\nдатчик",
                Point::new(display_w / 2, 5),
                self.big_font,
                TextStyleBuilder::new()
                    .alignment(Alignment::Center)
                    .baseline(Baseline::Top)
                    .build(),
            )
            .draw(display)?;

            self.render_current_info(
                display,
                None,
                Point::new(
                    0,
                    display_h - self.small_font.font.character_size.height as i32 * 3,
                ),
            )
        }
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

        let inner_frame = rect.offset(-2);
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
        selected_current: Option<usize>,
        initial_pos: Point,
    ) -> Result<(), display_interface::DisplayError>
    where
        DI: display_interface::WriteOnlyDataCommand,
    {
        for c in 0..self.current_values.len() {
            let string = format!(
                "{}: {}",
                ScanState::bus_name_from_usize(c),
                format_float_simple(self.current_values[c], 1)
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
                    format_float_simple(self.current_values[ss.bus_name()], 1),
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
        display: &mut GraphicsMode<DI>,
        animation_step: u32,
    ) -> Result<(), display_interface::DisplayError>
    where
        DI: display_interface::WriteOnlyDataCommand,
    {
        defmt::trace!("render_disconnect_screen: {}", animation_step);

        let (display_w, display_h) = {
            let d = display.get_dimensions();
            (d.0 as i32, d.1 as i32)
        };

        // draw frame
        let rect = Rectangle::with_center(
            Point::new(display_w / 2, display_h / 3),
            Size::new(display_w as u32 - 2, (display_h * 3 / 5) as u32),
        );
        rect.into_styled(PrimitiveStyle::with_stroke(BinaryColor::On, 1))
            .draw(display)
            .unwrap();

        let inner_frame = rect.offset(-2);
        inner_frame
            .into_styled(PrimitiveStyle::with_stroke(BinaryColor::On, 1))
            .draw(display)
            .unwrap();

        // render socket
        let size = Size::new(
            (rect.size.width / 3) as u32,
            (rect.size.width * 3 / 7) as u32,
        );
        let bounding_rect = Rectangle::with_center(
            Point::new(
                display_w / 2,
                rect.bottom_right().unwrap_or_default().y - (size.height as i32 / 2) - 5,
            ),
            size,
        );

        {
            let left_line = bounding_rect.left_line().translate(Point::new(-4, 0));
            left_line
                .into_styled(PrimitiveStyle::with_stroke(BinaryColor::On, 1))
                .draw(display)?;
            let socket_rect = Rectangle::new(
                left_line.start + Size::new(0, left_line.delta().y.abs() as u32 / 10),
                Size::new(7, left_line.delta().y.abs() as u32 * 8 / 10),
            );
            socket_rect
                .into_styled(PrimitiveStyle::with_stroke(BinaryColor::On, 1))
                .draw(display)?;

            let conn_rect = socket_rect.translate(Point::new(13, 0)).resized_height(
                left_line.delta().y.abs() as u32 * 6 / 10,
                embedded_graphics::geometry::AnchorY::Center,
            );
            conn_rect
                .into_styled(PrimitiveStyle::with_fill(BinaryColor::On))
                .draw(display)?;
            let center_y = conn_rect.center().y;
            let con_x = conn_rect.top_left.x;
            Line::new(
                Point::new(con_x, center_y + conn_rect.size.height as i32 / 4),
                Point::new(con_x - 5, center_y + conn_rect.size.height as i32 / 4),
            )
            .into_styled(PrimitiveStyle::with_stroke(BinaryColor::On, 2))
            .draw(display)?;
            Line::new(
                Point::new(con_x, center_y - conn_rect.size.height as i32 / 4),
                Point::new(con_x - 5, center_y - conn_rect.size.height as i32 / 4),
            )
            .into_styled(PrimitiveStyle::with_stroke(BinaryColor::On, 2))
            .draw(display)?;
            Line::new(
                Point::new(con_x + conn_rect.size.width as i32, center_y),
                Point::new(con_x + conn_rect.size.width as i32 + 7, center_y),
            )
            .into_styled(PrimitiveStyle::with_stroke(BinaryColor::On, 4))
            .draw(display)?;
        }

        // render animation
        if animation_step % 2 == 0 {
            // render cross

            bounding_rect
                .diaganal1()
                .into_styled(PrimitiveStyle::with_stroke(BinaryColor::On, 1))
                .draw(display)?;
            bounding_rect
                .diaganal2()
                .into_styled(PrimitiveStyle::with_stroke(BinaryColor::On, 1))
                .draw(display)?;
        }

        // current info
        self.render_current_info(
            display,
            None,
            Point::new(
                0,
                display_h - self.small_font.font.character_size.height as i32 * 3,
            ),
        )?;

        // draw bus name
        if let Some(scan_state) = self.prev_scan_state {
            Text::with_text_style(
                format!("Обрыв!\n{}", scan_state.bus_name(),).as_str(),
                Point::new(display_w / 2, inner_frame.top_left.y + 3),
                self.big_font,
                TextStyleBuilder::new()
                    .alignment(Alignment::Center)
                    .baseline(Baseline::Top)
                    .build(),
            )
            .draw(display)?;
        }
        Ok(())
    }

    pub fn animate(&mut self, time: TimerInstantU64<FREQ_HZ>) -> bool {
        let recal_animation = move |end_at: TimerInstantU64<FREQ_HZ>,
                                    current_animation_step: u32,
                                    an_step_duration_ms: u64|
              -> Option<u32> {
            let an_s = end_at
                .checked_duration_since(time)
                .map(|d| (d.to_millis() / an_step_duration_ms) as u32)
                .unwrap_or(0);
            if an_s != current_animation_step {
                Some(an_s)
            } else {
                None
            }
        };

        let recalc_i_mon_animation = move |start: TimerInstantU64<FREQ_HZ>,
                                           prev_animation_step: u32,
                                           prev_star_pos: Option<Point>|
              -> (State<FREQ_HZ>, bool) {
            let prev_star_pos = prev_star_pos.unwrap_or(Point::new(32, 32));
            let animation_step = (((time - start).to_millis() % 1_000) / 250) as u32;
            if prev_animation_step != animation_step {
                if animation_step == 0 {
                    let t = time.ticks();
                    let x = t as u32 / t.count_zeros();
                    let y = t as u32 / t.count_ones();
                    (
                        State::CurrentMonitoringScreen {
                            start,
                            star_base_pos: Point::new(x as i32, y as i32),
                            current_animation_step: animation_step,
                        },
                        true,
                    )
                } else {
                    (
                        State::CurrentMonitoringScreen {
                            start,
                            star_base_pos: prev_star_pos,
                            current_animation_step: animation_step,
                        },
                        true,
                    )
                }
            } else {
                (
                    State::CurrentMonitoringScreen {
                        start,
                        star_base_pos: prev_star_pos,
                        current_animation_step: prev_animation_step,
                    },
                    false,
                )
            }
        };

        match &mut self.state {
            State::WellcomeScreen {
                end_at,
                current_animation_step,
            } => {
                if time >= *end_at {
                    let (s, r) = recalc_i_mon_animation(time, 0, None);
                    self.state = s;
                    r
                } else {
                    if let Some(new_an_step) =
                        recal_animation(*end_at, *current_animation_step, 100)
                    {
                        *current_animation_step = new_an_step;
                        true
                    } else {
                        false
                    }
                }
            }
            State::DetectingScreen(s) => {
                self.prev_scan_state = Some(*s);
                false
            }
            State::OutputDisplayScreen { .. } => {
                if let Some(ss) = self.prev_scan_state {
                    if self.current_values[ss.bus_name()] < crate::config::CURRENT_THRESHOLD_MA {
                        let end_at = time + 2u64.secs();
                        self.state = State::DisconnectScreen {
                            end_at,
                            current_animation_step: recal_animation(end_at, 0, 100).unwrap_or(0),
                        };
                        true
                    } else {
                        false
                    }
                } else {
                    false
                }
            }
            State::DisconnectScreen {
                end_at,
                current_animation_step,
            } => {
                if time >= *end_at {
                    let (s, r) = recalc_i_mon_animation(time, 0, None);
                    self.state = s;
                    r
                } else {
                    if let Some(new_an_step) =
                        recal_animation(*end_at, *current_animation_step, 500)
                    {
                        *current_animation_step = new_an_step;
                        true
                    } else {
                        false
                    }
                }
            }
            State::CurrentMonitoringScreen {
                start,
                current_animation_step,
                star_base_pos,
            } => {
                let (s, r) =
                    recalc_i_mon_animation(*start, *current_animation_step, Some(*star_base_pos));

                if self.current_values["I2C"] > crate::config::CURRENT_THRESHOLD_MA {
                    self.state = State::DetectingScreen(ScanState::I2C(0));
                    true
                } else {
                    self.state = s;
                    r
                }
            }
        }
    }
}
