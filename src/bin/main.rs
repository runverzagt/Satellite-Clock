#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]
#![deny(clippy::large_stack_frames)]

use defmt::info;
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp_hal::gpio::{Output, OutputConfig, Level};
use esp_hal::mcpwm::timer::PeriodUpdatingMethod;
use esp_hal::{clock::CpuClock};
use esp_hal::timer::timg::TimerGroup;
use esp_hal::time::Rate;
use esp_hal::rmt::{PulseCode, Rmt};
use esp_println as _;
use esp_hal_smartled::{SmartLedsAdapter, smart_led_buffer, buffer_size};
use smart_leds::{RGB8, SmartLedsWrite, brightness, gamma, hsv::{Hsv, hsv2rgb}};
use static_cell::StaticCell;

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    loop {}
}

// Create static memory for the RMT Buffer
const NUM_LEDS: usize = 1;
static RMT_BUFF: StaticCell<[PulseCode; buffer_size(NUM_LEDS)]> = StaticCell::new();

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

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    let sw_interrupt =
        esp_hal::interrupt::software::SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    esp_rtos::start(timg0.timer0, sw_interrupt.software_interrupt0);

    info!("Embassy initialized!");

    // Initalize peripherals
    let rmt = Rmt::new(peripherals.RMT, Rate::from_mhz(80)).expect("Failed to init RMT0");
    let rmt_buffer: &'static mut [PulseCode; buffer_size(1)] = RMT_BUFF.init(smart_led_buffer!(NUM_LEDS));
    let rgb_led = SmartLedsAdapter::new(rmt.channel0, peripherals.GPIO8, rmt_buffer);
    let discrete_led = Output::new(peripherals.GPIO10, Level::Low, OutputConfig::default());

    // Spawn some tasks
    spawner.spawn(rgb_task(rgb_led)).unwrap();
    spawner.spawn(blink_task(discrete_led)).unwrap();

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
            led.write(brightness(gamma([data].into_iter()), LEVEL)).unwrap();

            Timer::after_millis(20).await;
        }
    }
}

#[embassy_executor::task]
async fn blink_task(mut led: Output<'static>) {
    enum LedStates {
        ON1,
        OFF1,
        ON2,
        OFF2,
    }
    const BLINK_DURATION: u64 = 100;
    const BLINK_PAUSE: u64 = 1000 - (3 * BLINK_DURATION);

    let mut state: LedStates = LedStates::OFF2;
    led.set_low();

    
    info!("blink_task started");

    loop {
        match state {
            LedStates::ON1 => {
                led.set_high();
                state = LedStates::OFF1;
                Timer::after_millis(BLINK_DURATION).await;
            },
            LedStates::OFF1 => {
                led.set_low();
                state = LedStates::ON2;
                Timer::after_millis(BLINK_DURATION).await;
            },
            LedStates::ON2 => {
                led.set_high();
                state = LedStates::OFF2;
                Timer::after_millis(BLINK_DURATION).await;
            },
            LedStates::OFF2 => {
                led.toggle();
                state = LedStates::ON1;
                Timer::after_millis(BLINK_PAUSE).await;
            }

        }
    }
}