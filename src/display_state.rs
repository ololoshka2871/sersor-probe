use alloc::boxed::Box;
use embedded_graphics::{
    draw_target::DrawTarget,
    geometry::{Dimensions, Point, Size},
    mono_font::{self, iso_8859_2::FONT_5X8},
    mono_font::{MonoTextStyle, MonoTextStyleBuilder},
    pixelcolor::BinaryColor,
    primitives::{Circle, Primitive, PrimitiveStyle, Rectangle},
    text::{Alignment, Baseline, Text},
    Drawable,
};
use num::rational::Ratio;
use ssd1309::mode::GraphicsMode;
use systick_monotonic::fugit::{Duration, TimerInstantU64};

use crate::devices::ValuesStorage;

enum State<const FREQ_HZ: u32> {
    /// После включения устройства на 3 секунды
    WellcomeScreen {
        end_at: TimerInstantU64<FREQ_HZ>,
        current_animation_step: u32,
    },
    /// Ожидание подключения устройств к портам, мониторинг потребляемого тока
    CurrentMonitoringScreen,
    /// Обнаружено подключение, сканирование шины
    DetectingScreen,
    /// Отображение показаний найденного устройства
    OutputDisplayScreen {
        value_storage: Box<dyn ValuesStorage>,
    },
    /// Устройство отключено, заглдушка на 2 секунды и переход к CurrentMonitoringScreen
    DisconnectScreen,
}

pub struct DisplayState<const FREQ_HZ: u32> {
    state: State<FREQ_HZ>,
}

impl<const FREQ_HZ: u32> DisplayState<FREQ_HZ> {
    pub fn init(wellcome_before: TimerInstantU64<FREQ_HZ>) -> Self {
        Self {
            state: State::WellcomeScreen {
                end_at: wellcome_before,
                current_animation_step: 0,
            },
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
        /*
        let style = MonoTextStyle::new(&FONT_5X8, BinaryColor::On);

        Text::new("Hello world!", Point::new(0, 0), style).draw(disp)?;
        Text::new("Hello Rust!", Point::new(0, 16), style).draw(disp)?;
        */

        match &self.state {
            State::WellcomeScreen {
                end_at: _,
                current_animation_step,
            } => self.render_wellcome_screen(disp, *current_animation_step),
            State::CurrentMonitoringScreen => self.render_current_monitoring_screen(disp),
            State::DetectingScreen => self.render_detecting_screen(disp),
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

    fn render_wellcome_screen<DI>(
        &self,
        display: &mut GraphicsMode<DI>,
        animation_step: u32,
    ) -> Result<(), display_interface::DisplayError>
    where
        DI: display_interface::WriteOnlyDataCommand,
    {
        defmt::trace!("render_wellcome_screen(_, {})", animation_step);

        let big_font = MonoTextStyleBuilder::new()
            .font(&mono_font::iso_8859_5::FONT_10X20)
            .text_color(BinaryColor::On)
            .build();

        let small_font_italic = MonoTextStyleBuilder::new()
            .font(&mono_font::iso_8859_5::FONT_6X13_ITALIC)
            .text_color(BinaryColor::On)
            .build();

        let (display_w, display_h) = {
            let d = display.get_dimensions();
            (d.0 as i32, d.1 as i32)
        };

        Text::with_baseline("Пробник", Point::new(25, -3), big_font, Baseline::Top)
            .draw(display)?;
        Text::with_alignment(
            "универсальный",
            Point::new(
                (display_w / 2).into(),
                big_font.font.character_size.height as i32 + 6,
            ),
            big_font,
            Alignment::Center,
        )
        .draw(display)?;
        Text::with_baseline(
            "СКТБ ЭлПА 2023",
            Point::new(23, display_h),
            small_font_italic,
            Baseline::Bottom,
        )
        .draw(display)?;

        const CIRCLES_COUNT: i32 = 4;
        let circles_y = display_h * 2 / 3;
        let circles_x_start = display_w / 4;
        let curcles_step = display_w / (2 * CIRCLES_COUNT);
        const DIAMETER_TABLE: [u8; 11] = [3, 4, 5, 7, 9, 10, 9, 8, 5, 4, 3];
        const ANIMATION_OFFSET: i32 = (DIAMETER_TABLE.len() - 3) as i32;

        for c in 0..CIRCLES_COUNT {
            let circle_x = circles_x_start + c * curcles_step;
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
    ) -> Result<(), display_interface::DisplayError>
    where
        DI: display_interface::WriteOnlyDataCommand,
    {
        defmt::trace!("render_detecting_screen");
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
        Ok(())
    }

    fn render_disconnect_screen<DI>(
        &self,
        display: &mut GraphicsMode<DI>,
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
            _ => false,
        }
    }
}
