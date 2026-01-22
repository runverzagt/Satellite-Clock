#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]
#![deny(clippy::large_stack_frames)]

use defmt::{info};
use embassy_executor::Spawner;
use embassy_time::Timer;
use esp_hal::Blocking;
use esp_hal::clock::CpuClock;
use esp_hal::gpio::DriveMode;
use esp_hal::ledc::{
    LSGlobalClkSource, Ledc, LowSpeed,
    channel::{self, ChannelIFace},
    timer::{self, TimerIFace},
};
use esp_hal::i2c::master::{Config,I2c};
use esp_hal::rmt::{PulseCode, Rmt};
use esp_hal::time::Rate;
use esp_hal::timer::timg::TimerGroup;
use esp_hal_smartled::{SmartLedsAdapter, buffer_size, smart_led_buffer};
use esp_println as _;
use smart_leds::{
    RGB8, SmartLedsWrite, brightness, gamma,
    hsv::{Hsv, hsv2rgb},
};
use ssd1306::mode::DisplayConfig;
use ssd1306::prelude::DisplayRotation;
use ssd1306::size::DisplaySize128x32;
use ssd1306::{I2CDisplayInterface, Ssd1306};
use embedded_graphics::{
    mono_font::{ascii::FONT_6X10, MonoTextStyleBuilder},
    pixelcolor::BinaryColor,
    prelude::*,
    text::{Baseline, Text},
};
use static_cell::StaticCell;

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    loop {}
}

// Create static memory for the RMT Buffer
const NUM_LEDS: usize = 1;
type RmtBuffType = [PulseCode; buffer_size(NUM_LEDS)];
static RMT_BUFF: StaticCell<RmtBuffType> = StaticCell::new();
static LS_TIMER: StaticCell<timer::Timer<'static, LowSpeed>> = StaticCell::new();

// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();

#[allow(
    clippy::large_stack_frames,
    reason = "it's not unusual to allocate larger buffers etc. in main"
)]
#[esp_rtos::main]
async fn main(spawner: Spawner) -> ! {
    // generator version: 1.2.0

    // NOTE: MUST init esp_hal before doing anything else
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    let sw_interrupt =
        esp_hal::interrupt::software::SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    esp_rtos::start(timg0.timer0, sw_interrupt.software_interrupt0);

    info!("Embassy initialized!");

    // Initalize peripherals
    // Configure RGB LED
    let rmt = Rmt::new(peripherals.RMT, Rate::from_mhz(80)).expect("Failed to init RMT0");
    let rmt_buffer: &'static mut RmtBuffType = RMT_BUFF.init(smart_led_buffer!(NUM_LEDS));
    let rgb_led = SmartLedsAdapter::new(rmt.channel0, peripherals.GPIO8, rmt_buffer);

    // Configure PWM
    let mut ledc = Ledc::new(peripherals.LEDC);
    ledc.set_global_slow_clock(LSGlobalClkSource::APBClk);
    let lstimer0: &mut timer::Timer<'static, LowSpeed> =
        LS_TIMER.init(ledc.timer::<LowSpeed>(timer::Number::Timer0));
    lstimer0
        .configure(timer::config::Config {
            duty: timer::config::Duty::Duty5Bit,
            clock_source: timer::LSClockSource::APBClk,
            frequency: Rate::from_khz(20),
        })
        .expect("unable to configure lstimer0");
    let mut pwm_channel0 = ledc.channel(channel::Number::Channel0, peripherals.GPIO10);
    pwm_channel0
        .configure(channel::config::Config {
            timer: lstimer0,
            duty_pct: 10,
            drive_mode: DriveMode::PushPull,
        })
        .expect("unable to configure pwm_channel0");

    // Configure i2c
    let i2c = I2c::new(peripherals.I2C0, Config::default()).unwrap()
        .with_sda(peripherals.GPIO6)
        .with_scl(peripherals.GPIO7);

    // Spawn tasks
    spawner.spawn(rgb_task(rgb_led)).unwrap();
    spawner.spawn(blink_task(pwm_channel0)).unwrap();
    spawner.must_spawn(display_task(i2c));
    loop {
        // Main loop doesn't do anything, but must await to allow tasks to run
        Timer::after_millis(1000).await;
    }

    // for inspiration have a look at the examples at https://github.com/esp-rs/esp-hal/tree/esp-hal-v1.0.0/examples
}

#[embassy_executor::task]
async fn rgb_task(mut led: SmartLedsAdapter<'static, { buffer_size(NUM_LEDS) }>) {
    const LEVEL: u8 = 5;
    let mut color = Hsv {
        hue: 0,
        sat: 255,
        val: 255,
    };
    let mut data: RGB8;

    info!("rgb_task started");

    loop {
        // Rotate through the hues
        for hue in 0..=255 {
            color.hue = hue;
            // Use smart_leds::hsv to convert from a hue to an rgb value
            data = hsv2rgb(color);
            //info!("R:{}, G:{}, B:{}", data.r, data.g, data.b);
            led.write(brightness(gamma([data].into_iter()), LEVEL))
                .unwrap();

            Timer::after_millis(10).await;
        }
    }
}

#[embassy_executor::task]
async fn blink_task(led: esp_hal::ledc::channel::Channel<'static, LowSpeed>) {
    const MAX_BRIGHTNESS: u8 = 100;
    const FLASH_PERIOD_MS: u64 = 1500;
    const FLASH_STEP_LENGTH_MS: u64 = (FLASH_PERIOD_MS) / (2 * MAX_BRIGHTNESS as u64);
    const PAUSE_DURATION_MS: u64 = 600;

    info!("blink_task started");

    loop {
        for brightness in 0..=MAX_BRIGHTNESS {
            led.set_duty(brightness).unwrap();
            Timer::after_millis(FLASH_STEP_LENGTH_MS).await;
        }
        for brightness in (0..=MAX_BRIGHTNESS).rev() {
            led.set_duty(brightness).unwrap();
            Timer::after_millis(FLASH_STEP_LENGTH_MS).await;
        }
        Timer::after_millis(PAUSE_DURATION_MS).await;
    }
}

#[embassy_executor::task]
async fn display_task(i2c: I2c<'static, Blocking>) {
    let interface = I2CDisplayInterface::new(i2c);
    let mut display = Ssd1306::new(interface, DisplaySize128x32, DisplayRotation::Rotate180)
        .into_buffered_graphics_mode();
    display.init().unwrap();

    let text_style = MonoTextStyleBuilder::new()
        .font(&FONT_6X10)
        .text_color(BinaryColor::On)
        .build();

    Text::with_baseline("Hello World!", Point::zero(), text_style, Baseline::Top)
        .draw(&mut display)
        .unwrap();

    Text::with_baseline("Hello Rust!", Point::new(0, 16), text_style, Baseline::Top)
        .draw(&mut display)
        .unwrap();

    display.flush().unwrap();

    loop {
        Timer::after_millis(1000).await;
    }
}