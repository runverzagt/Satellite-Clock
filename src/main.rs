#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]
#![deny(clippy::large_stack_frames)]

mod time;
use chrono::{ DateTime, Datelike, Timelike, Utc, Weekday};
use embedded_graphics::pixelcolor::Rgb565;
use esp_hal::gpio::{Output, OutputConfig};
use esp_hal::time::Rate;
use lcd_async::raw_framebuf::RawFrameBuf;
use libscd::measurement::Measurement;
use time::{Clock, ntp_worker};

use core::f32::consts::PI;

use defmt::{info, error};
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer, };
use embassy_sync::{ blocking_mutex::raw::{NoopRawMutex, CriticalSectionRawMutex}, mutex::Mutex, watch::{Watch, Receiver, Sender}};

use embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice;

use esp_alloc as _;
use esp_backtrace as _;

use esp_hal::{Async};
use esp_hal::clock::CpuClock;
use esp_hal::spi::{Mode, master::{Config, Spi}};
use esp_hal::timer::timg::TimerGroup;

use esp_hal::{i2c::master::I2c};
use esp_println as _;


use libscd::asynchronous::scd4x::Scd4x;

use lcd_async::{Builder, models::ST7735s};
use lcd_async::interface::SpiInterface;
use lcd_async::options::Orientation;

use libm::sincosf;
use embedded_graphics::primitives::{Circle, Line, PrimitiveStyle, Rectangle};
use embedded_graphics::{
    mono_font::{ascii::*, MonoTextStyleBuilder},
    prelude::*,
    text::{Baseline, Text},
};

use static_cell::StaticCell;
use heapless::String;
use core::fmt::Write;


// Define the timestamp for defmt
defmt::timestamp!("{:02}:{:02}:{:02}", {embassy_time::Instant::now().as_secs() / 3600}, {(embassy_time::Instant::now().as_secs() % 3600) / 60}, embassy_time::Instant::now().as_secs() % 60);

// WIFI info
const SSID: &str = env!("SSID");
const PASSWORD: &str = env!("PASSWORD");
const WIDTH: usize = 80;
const HEIGHT: usize = 160;
const PIXEL_SIZE: usize = 2; // RGB565 = 2 bytes per pixel
const FRAME_BUF_SIZE: usize = WIDTH * HEIGHT * PIXEL_SIZE;
static FRAME_BUFFER: StaticCell<[u8; FRAME_BUF_SIZE]> = StaticCell::new();

// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();

#[allow(
    clippy::large_stack_frames,
    reason = "it's not unusual to allocate larger buffers etc. in main"
)]
#[esp_rtos::main]
async fn main(spawner: Spawner) -> ! {
    // NOTE: MUST init esp_hal before doing anything else
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    let sw_interrupt =
        esp_hal::interrupt::software::SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    esp_rtos::start(timg0.timer0, sw_interrupt.software_interrupt0);

    info!("Embassy initialized!");

    // Configure spi
    let sclk = peripherals.GPIO8;
    let mosi = peripherals.GPIO9;
    let cs = Output::new(peripherals.GPIO21, esp_hal::gpio::Level::Low, OutputConfig::default());
    let rst = Output::new(peripherals.GPIO10, esp_hal::gpio::Level::Low, OutputConfig::default());
    let dc = Output::new(peripherals.GPIO20, esp_hal::gpio::Level::Low, OutputConfig::default());

    let spi_bus= Spi::new(
        peripherals.SPI2, Config::default()
                            .with_frequency(Rate::from_mhz(10))
                            .with_mode(Mode::_0))
        .unwrap()
        .with_sck(sclk)
        .with_mosi(mosi)
        .into_async();

    // Create shared SPI bus
    static SPI_BUS: StaticCell<Mutex<NoopRawMutex, Spi<'static, Async>>> = StaticCell::new();
    let spi_bus = Mutex::new(spi_bus);
    let spi_bus = SPI_BUS.init(spi_bus);

    let spi_disp = SpiDevice::new(spi_bus, cs);
    let di = SpiInterface::new(spi_disp, dc);
    let mut delay = embassy_time::Delay; 

    let display= Builder::new(ST7735s, di)
        .reset_pin(rst)
        .display_size(WIDTH as u16, HEIGHT as u16)
        .orientation(Orientation::new().rotate(lcd_async::options::Rotation::Deg90))
        .display_offset(28, 0)
        .init(&mut delay)
        .await
        .unwrap();

    // Configure i2c
    Timer::after(Duration::from_millis(1000)).await;    // Allow 1000 ms to power up
    let delay = embassy_time::Delay; 
    let i2c_config = esp_hal::i2c::master::Config::default().with_frequency(Rate::from_khz(10));
    let i2c = I2c::new(peripherals.I2C0, i2c_config).unwrap()
        .with_sda(peripherals.GPIO0)
        .with_scl(peripherals.GPIO1)
        .into_async();

    // Start sensor
    let mut scd = Scd4x::new(i2c, delay);
    // When re-programming, the controller will be restarted,
    // but not the sensor. We try to stop it in order to
    // prevent the rest of the commands failing.
    if let Err(e) = scd.stop_periodic_measurement().await {
        defmt::panic!("Failed to stop periodic measurement: {:?}", e);
    }

    // info!("Sensor serial number: {:?}", scd.serial_number().await);
    if let Err(e) = scd.start_low_power_periodic_measurement().await {
        defmt::panic!("Failed to start periodic measurement: {:?}", e);
    }
    // Create a signal to send the sensor's reading
    static SENSOR_VALUE: Watch<CriticalSectionRawMutex, Measurement, 1> = Watch::new();

    static CLOCK: StaticCell<Clock> = StaticCell::new();
    let clock = CLOCK.init(Clock::new());

    // Spawn tasks
    spawner.must_spawn(display_task(display, clock, SENSOR_VALUE.receiver().expect("Unable to create reiever")));
    spawner.must_spawn(sensor_read_task(scd, SENSOR_VALUE.sender()));

    loop {
        Timer::after_secs(300).await;
    }

    // for inspiration have a look at the examples at https://github.com/esp-rs/esp-hal/tree/esp-hal-v1.0.0/examples
}

#[embassy_executor::task]
async fn display_task(
    mut disp: lcd_async::Display<SpiInterface<SpiDevice<'static, NoopRawMutex, Spi<'static, Async>, Output<'static>>, Output<'static>>, ST7735s, Output<'static>>, 
    clock: &'static Clock,
    mut sensor_reading: Receiver<'static, CriticalSectionRawMutex, Measurement, 1>
) 
{
    let frame_buffer = FRAME_BUFFER.init([0; FRAME_BUF_SIZE]);
    let mut raw_fb = RawFrameBuf::<Rgb565,_>::new(frame_buffer.as_mut_slice(),HEIGHT, WIDTH );
    raw_fb.clear(Rgb565::BLACK).unwrap();

    let thin_stroke = PrimitiveStyle::with_stroke(Rgb565::YELLOW, 1);
    let thin_stroke_off = PrimitiveStyle::with_stroke(Rgb565::BLACK, 1);

    let text_style = MonoTextStyleBuilder::new()
        .font(&FONT_9X18)
        .text_color(Rgb565::CYAN)
        .build();

    let clearing_text_style = MonoTextStyleBuilder::new()
        .font(&FONT_9X18)
        .text_color(Rgb565::BLACK)
        .build();

    static CENTER: Point = Point::new(1, 5);
    static CLOCK_POS:Point = Point::new(25, 55);
    static C02_POS:Point = Point::new(40, 2);
    static TEMP_POS:Point = Point::new(50, 20);
    static HUM_POS:Point = Point::new(50, 38);
    const FRAME_PERIOD_MS: u64 = 30;
    const SCANNER_PERIOD_MS: u64 = 3000;
    let angle_update = ((2f32 * PI) / (SCANNER_PERIOD_MS as f32)) * FRAME_PERIOD_MS as f32;
    let mut angle: f32 = angle_update;
    let mut old_time = DateTime::UNIX_EPOCH;
    let mut time = heapless::String::<15>::new();
    // Measurement variables
    let mut measurement: Measurement;
    let mut co2_str= heapless::String::<13>::new();
    write!(time, "Awaiting Time").unwrap();
    write!(co2_str, "CO2 Starting").unwrap();
    Text::with_baseline(co2_str.as_str(), C02_POS, text_style, Baseline::Top)
            .draw(&mut raw_fb)
            .unwrap();
    info!("display_task started");

    loop {
        // prepare the timer
        let period_timer = Timer::after_millis(FRAME_PERIOD_MS);

        // Draw the circles
        Circle::new(CENTER - convert_tl_to_center(20), 20)
            .into_styled(thin_stroke)
            .draw(&mut raw_fb).expect("Unable to draw circle1");
        Circle::new(CENTER - convert_tl_to_center(32), 32)
            .into_styled(thin_stroke)
            .draw(&mut raw_fb).expect("Unable to draw circle2");
        Circle::new(CENTER - convert_tl_to_center(44), 44)
            .into_styled(thin_stroke)
            .draw(&mut raw_fb).expect("Unable to draw circle3");
        Circle::new(CENTER - convert_tl_to_center(56), 56)
            .into_styled(thin_stroke)
            .draw(&mut raw_fb).expect("Unable to draw circle4");
        // Draw the radio line
        let (end_x, end_y) = sincosf(angle);
        let line = Line::new(CENTER, CENTER+Point::new(((28 as f32) * end_x) as i32, ((28 as f32) * end_y) as i32));
        line.into_styled(thin_stroke)
            .draw(&mut raw_fb).expect("Unable to draw line");

        // Use new_time to see if the time has changed enough to redraw the time
        let new_time = clock.now().await;
        let time_redraw = (new_time.minute() != old_time.minute()) || (new_time.hour() != old_time.hour());
        if !clock.time_has_been_set().await {
            Text::with_baseline(time.as_str(), CLOCK_POS, text_style, Baseline::Top)
                .draw(&mut raw_fb)
                .unwrap();
        }else if time_redraw {
            Text::with_baseline(time.as_str(), CLOCK_POS, clearing_text_style, Baseline::Top)
                .draw(&mut raw_fb)
                .unwrap();

            if clock.time_has_been_set().await {
                time = clock.get_date_time_str().await;
            } else {
            }

            Text::with_baseline(time.as_str(), CLOCK_POS, text_style, Baseline::Top)
                .draw(&mut raw_fb)
                .unwrap();

            // Update
            old_time = new_time;
        }

        // Update the sensor reading values
        if sensor_reading.try_changed().is_some() 
        {
            Rectangle::new(C02_POS, Size { width: 120, height: 56 })
                .into_styled(PrimitiveStyle::with_fill(Rgb565::BLACK))
                .draw(&mut raw_fb).expect("Unable to draw rect");

            measurement = sensor_reading.get().await;

            let mut temp_string = String::<4>::new();
            write!(temp_string, "{:4}", measurement.co2).unwrap();
            co2_str = String::from("CO2: ");
            co2_str.push_str(&temp_string).unwrap();
            co2_str.push_str(" PPM").unwrap();
            Text::with_baseline(co2_str.as_str(), C02_POS, text_style, Baseline::Top)
                    .draw(&mut raw_fb)
                    .unwrap();

            let mut temp_string = String::<4>::new();
            write!(temp_string, "{:.*}", 1, measurement.temperature).unwrap();
            co2_str = String::from("T: ");
            co2_str.push_str(&temp_string).unwrap();
            co2_str.push_str("C").unwrap();
            Text::with_baseline(co2_str.as_str(), TEMP_POS, text_style, Baseline::Top)
                    .draw(&mut raw_fb)
                    .unwrap();

            let mut temp_string = String::<4>::new();
            write!(temp_string, "{:.*}", 1, measurement.humidity).unwrap();
            co2_str = String::from("HUM: ");
            co2_str.push_str(&temp_string).unwrap();
            co2_str.push_str("%").unwrap();
            Text::with_baseline(co2_str.as_str(), HUM_POS, text_style, Baseline::Top)
                    .draw(&mut raw_fb)
                    .unwrap();
        }


        // Update the angle for the next frame
        angle -= angle_update;
        if angle > (2f32 * PI) {
            angle = 0f32;
        }

        disp.show_raw_data(0 as u16, 0 as u16,raw_fb.width() as u16, raw_fb.height() as u16, raw_fb.as_bytes())
            .await
            .unwrap();

        // Pend until the next frame is ready to be drawn
        period_timer.await;

        // Clear the radio line in preperation of drawing the next frame
        line.into_styled(thin_stroke_off)
            .draw(&mut raw_fb).expect("Unable to UNdraw line");

    }
}

// Create a point representing an offset from the top-left of a circle to its center
fn convert_tl_to_center(d: i32) -> Point {
    Point::new(d/2, d/2)
}

// This task reads the C02 sensor and sends the value to the supplied watch::Sender
#[embassy_executor::task]
async fn sensor_read_task(mut sensor: Scd4x<I2c<'static, Async>, embassy_time::Delay>,
    sensor_measurement: Sender<'static, CriticalSectionRawMutex, Measurement, 1>
) {
    loop {
        if sensor.data_ready().await.unwrap() {
            let m: Measurement = match sensor.read_measurement().await {
                Ok(m) => {
                    m
                },
                Err(e) => panic!("Unable to read value: {:?}", e)
            };

            info!("CO2: {}\nHumidity: {}\nTemperature: {}", m.co2, m.humidity, m.temperature);
            
            sensor_measurement.send(m);

            Timer::after(Duration::from_secs(30)).await;
        }
    }
}