#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]
#![deny(clippy::large_stack_frames)]

mod time;
use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::pixelcolor::raw::BigEndian;
use esp_hal::gpio::{Output, OutputConfig};
use esp_hal::time::Rate;
use time::{Clock, ntp_worker};

use core::f32::consts::PI;

use defmt::{info, error};
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer, };
use embassy_net::{Runner, StackResources, };
use embassy_sync::blocking_mutex::{NoopMutex, raw::NoopRawMutex, Mutex};
use embedded_hal_bus::spi::ExclusiveDevice;

use esp_alloc as _;
use esp_backtrace as _;

use esp_hal::Blocking;
use esp_hal::clock::CpuClock;
use esp_hal::spi::{Mode, master::{Config, Spi}};
use esp_hal::timer::timg::TimerGroup;
use esp_hal::{
    rng::Rng,
    ram
};
use esp_println as _;
use esp_radio::wifi::{ClientConfig, PowerSaveMode, WifiStaState};
use esp_radio::{
    wifi::{
        ModeConfig,
        WifiController,
        WifiDevice,
        WifiEvent,
    },
};

use mipidsi::interface::SpiInterface;
use mipidsi::{Builder, models::ST7735s};
use mipidsi::options::Orientation;
use libm::sincosf;
use embedded_graphics::primitives::{Circle, Line, PrimitiveStyle};
use embedded_graphics::{
    mono_font::{ascii::*, MonoTextStyleBuilder},
    prelude::*,
    text::{Baseline, Text},
};
use static_cell::StaticCell;

// Define the timestamp for defmt
defmt::timestamp!("{:02}:{:02}:{:02}", {embassy_time::Instant::now().as_secs() / 3600}, {(embassy_time::Instant::now().as_secs() % 3600) / 60}, embassy_time::Instant::now().as_secs() % 60);

// WIFI info
const SSID: &str = env!("SSID");
const PASSWORD: &str = env!("PASSWORD");
const WIDTH: usize = 80;
const HEIGHT: usize = 160;
const FRAME_BUF_SIZE: usize = WIDTH * HEIGHT * 2;
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
    let sclk = peripherals.GPIO6;
    let mosi = peripherals.GPIO7;
    let cs = Output::new(peripherals.GPIO18, esp_hal::gpio::Level::Low, OutputConfig::default());
    let rst = Output::new(peripherals.GPIO20, esp_hal::gpio::Level::Low, OutputConfig::default());
    let dc = Output::new(peripherals.GPIO19, esp_hal::gpio::Level::Low, OutputConfig::default());

    let spi_bus = Spi::new(
        peripherals.SPI2, Config::default()
                            .with_frequency(Rate::from_mhz(5))
                            .with_mode(Mode::_0))
        .unwrap()
        .with_sck(sclk)
        .with_mosi(mosi);
    let spi_disp = ExclusiveDevice::new(spi_bus, cs, embassy_time::Delay).unwrap();
    let di = SpiInterface::new(spi_disp, dc, FRAME_BUFFER.init([0; FRAME_BUF_SIZE]));
    let mut delay = embassy_time::Delay; 

    let display= Builder::new(ST7735s, di)
        .reset_pin(rst)
        .display_size(WIDTH as u16, HEIGHT as u16)
        .display_offset(25, 0)
        .orientation(Orientation::new().rotate(mipidsi::options::Rotation::Deg90))
        .init(&mut delay)
        .unwrap();

    // Configure Wifi
    esp_alloc::heap_allocator!(#[ram(reclaimed)] size: 64 * 1024);
    esp_alloc::heap_allocator!(size: 36 * 1024);
    static RADIO_CTRL: StaticCell<esp_radio::Controller> = StaticCell::new();
    static NETWRK_STACK:StaticCell<StackResources<3>> = StaticCell::new();

    let radio_init = RADIO_CTRL.init(esp_radio::init().expect("Failed to init Wifi"));
    let (controller, interfaces) =
        esp_radio::wifi::new(radio_init, peripherals.WIFI, Default::default())
        .expect("Failed to create new wifi controller/interface");
    let wifi_iface = interfaces.sta;
    
    let dhcpv4_config = embassy_net::Config::dhcpv4(Default::default());

    let rng = Rng::new();
    let seed = (rng.random() as u64) << 32 | rng.random() as u64;

    // Init network stack
    let (stack, runner) = embassy_net::new(
        wifi_iface,
        dhcpv4_config,
        NETWRK_STACK.init(StackResources::<3>::new()),
        seed,
    );

    static CLOCK: StaticCell<Clock> = StaticCell::new();
    let clock = CLOCK.init(Clock::new());

    // Spawn tasks
    spawner.must_spawn(display_task(display, clock));
    spawner.must_spawn(connection(controller));
    spawner.must_spawn(net_task(runner));

    wait_for_connection(stack).await;

    spawner.must_spawn(ntp_worker(stack, clock));

    loop {
        Timer::after_secs(300).await;
    }

    // for inspiration have a look at the examples at https://github.com/esp-rs/esp-hal/tree/esp-hal-v1.0.0/examples
}

#[embassy_executor::task]
async fn display_task(
    mut disp: mipidsi::Display<SpiInterface<'static, ExclusiveDevice<Spi<'static, Blocking>, Output<'static>, embassy_time::Delay>, Output<'static>>, ST7735s, Output<'static>>, 
    clock: &'static Clock,
) 
{
    disp.clear(Rgb565::BLACK).unwrap();

    let thin_stroke = PrimitiveStyle::with_stroke(Rgb565::WHITE, 1);
    let thin_stroke_off = PrimitiveStyle::with_stroke(Rgb565::BLACK, 1);

    let text_style = MonoTextStyleBuilder::new()
        .font(&FONT_7X13)
        .text_color(Rgb565::WHITE)
        .build();

    let clearing_text_style = MonoTextStyleBuilder::new()
        .font(&FONT_7X13)
        .text_color(Rgb565::BLACK)
        .build();

    static CENTER: Point = Point::new(1, 5);
    static CLOCK_POS:Point = Point::new(30, 50);
    const FRAME_PERIOD_MS: u64 = 30;
    const SCANNER_PERIOD_MS: u64 = 3000;
    let angle_update = ((2f32 * PI) / (SCANNER_PERIOD_MS as f32)) * FRAME_PERIOD_MS as f32;
    let mut angle: f32 = angle_update;
    info!("display_task started");

    loop {
        // prepare the timer
        let period_timer = Timer::after_millis(FRAME_PERIOD_MS);

        // Draw the circles
        Circle::new(CENTER - convert_tl_to_center(20), 20)
            .into_styled(thin_stroke)
            .draw(&mut disp).expect("Unable to draw circle1");
        Circle::new(CENTER - convert_tl_to_center(32), 32)
            .into_styled(thin_stroke)
            .draw(&mut disp).expect("Unable to draw circle2");
        Circle::new(CENTER - convert_tl_to_center(44), 44)
            .into_styled(thin_stroke)
            .draw(&mut disp).expect("Unable to draw circle3");
        Circle::new(CENTER - convert_tl_to_center(56), 56)
            .into_styled(thin_stroke)
            .draw(&mut disp).expect("Unable to draw circle4");
        // Draw the radio line
        let (end_x, end_y) = sincosf(angle);
        let line = Line::new(CENTER, CENTER+Point::new(((28 as f32) * end_x) as i32, ((28 as f32) * end_y) as i32));
        line.into_styled(thin_stroke)
            .draw(&mut disp).expect("Unable to draw line");

        let time = clock.get_date_time_str().await;

        Text::with_baseline(time.as_str(), CLOCK_POS, text_style, Baseline::Top)
            .draw(&mut disp)
            .unwrap();

        // Update the angle for the next frame
        angle -= angle_update;
        if angle > (2f32 * PI) {
            angle = 0f32;
        }

        // Pend until the next frame is ready to be drawn
        period_timer.await;

        // Clear the radio line in preperation of drawing the next frame
        line.into_styled(thin_stroke_off)
            .draw(&mut disp).expect("Unable to UNdraw line");

        Text::with_baseline(time.as_str(), CLOCK_POS, clearing_text_style, Baseline::Top)
            .draw(&mut disp)
            .unwrap();
    }
}

// Create a point representing an offset from the top-left of a circle to its center
fn convert_tl_to_center(d: i32) -> Point {
    Point::new(d/2, d/2)
}

#[embassy_executor::task]
async fn connection(mut controller: WifiController<'static>) {
    info!("start connection task");
    controller.set_power_saving(PowerSaveMode::Maximum).expect("Unable to raise power saving mode");
    loop {
        match esp_radio::wifi::sta_state() {
            WifiStaState::Connected => {
                // wait until we're no longer connected
                controller.wait_for_event(WifiEvent::StaDisconnected).await;
                Timer::after(Duration::from_millis(5000)).await;
            }
            _ => {}
        }
        if !matches!(controller.is_started(), Ok(true)) {
            let client_config = ModeConfig::Client(
                ClientConfig::default()
                    .with_ssid(SSID.into())
                    .with_password(PASSWORD.into()),
            );
            let res = controller.set_config(&client_config);
            info!("Wifi config returned: {:?}", res);
            let res = controller.start_async().await;
            info!("Wifi startup returned: {:?}", res);
        }
        info!("About to connect....");

        match controller.connect_async().await {
            Ok(_) => info!("Wifi connected!"),
            Err(e) => {
                info!("Failed to connect to wifi: {:?}", e);
                Timer::after(Duration::from_millis(5000)).await
            }
        }
    }
}

#[embassy_executor::task]
async fn net_task(mut runner: Runner<'static, WifiDevice<'static>>) {
    runner.run().await
}

async fn wait_for_connection(stack: embassy_net::Stack<'_>) {
    info!("Waiting for link to be up");
    loop {
        if stack.is_link_up() {
            break;
        }
        Timer::after(Duration::from_millis(500)).await;
    }

    info!("Waiting to get IP Addr");
    loop {
        if let Some(config) = stack.config_v4() {
            info!("Got IP: {}", config.address);
            break;
        }
        Timer::after(Duration::from_millis(500)).await;
    }
}