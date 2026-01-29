#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]
#![deny(clippy::large_stack_frames)]

use core::f32::consts::PI;
use core::net::Ipv4Addr;

use defmt::{info, println, error};
use embassy_executor::Spawner;
use embassy_net::{IpEndpoint, dns};
use embassy_net::tcp::client::{TcpClient, TcpClientState};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, mutex::Mutex};
use embassy_time::{Duration, Timer, Instant};
use embassy_futures::join::join;
use embassy_net::{Runner, StackResources, tcp::TcpSocket, udp::{PacketMetadata, UdpSocket}};
use sntpc::{get_time, NtpContext, NtpTimestampGenerator};
use sntpc_net_embassy::UdpSocketWrapper;
use chrono::{Date, DateTime, Datelike, TimeZone, Timelike, Utc, Weekday};
use core::net::{IpAddr, SocketAddr};
use heapless::String;
use core::fmt::Write;

use esp_alloc as _;
use esp_backtrace as _;

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
use esp_hal::{
    rng::Rng,
    ram
};
use esp_hal_smartled::{SmartLedsAdapter, buffer_size, smart_led_buffer};
use esp_println as _;
use esp_radio::wifi::{ClientConfig, PowerSaveMode, WifiStaState};
use esp_radio::{
    wifi::{
        ModeConfig,
        WifiController,
        WifiDevice,
        WifiEvent,
        ScanConfig,
    },
};

use reqwless::request::RequestBuilder;
use smoltcp::socket::tcp;
use reqwless::client::{HttpClient, TlsConfig};
use trouble_host::prelude::*;
use libm::sincosf;
use numtoa::NumToA;
use smart_leds::{
    RGB8, SmartLedsWrite, brightness, gamma,
    hsv::{Hsv, hsv2rgb},
};
use embedded_graphics::primitives::{Circle, PrimitiveStyle, Line};
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

#[derive(Copy, Clone, Default)]
struct Timestamp{
    duration: Duration,
}

impl NtpTimestampGenerator for Timestamp {
    fn init(&mut self) {
        // self.duration = SystemTime::now().duration_since(SystemTime::UNIX_EPOCH).unwrap();
        self.duration = Duration::from_ticks(Instant::now().as_ticks());
    }

    fn timestamp_sec(&self) -> u64 {
        self.duration.as_secs()
    }

    fn timestamp_subsec_micros(&self) -> u32 {
        (self.duration.as_micros() - (self.duration.as_secs() * 1_000_000)) as u32
    }
}

// WIFI info
const SSID: &str = env!("SSID");
const PASSWORD: &str = env!("PASSWORD");
const CONNECTIONS_MAX: usize = 1;
const L2CAP_CHANNELS_MAX: usize = 2; // Signal + att

// Create static memory for the RMT Buffer
const NUM_LEDS: usize = 1;
type RmtBuffType = [PulseCode; buffer_size(NUM_LEDS)];

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
    static RMT_BUFF: StaticCell<RmtBuffType> = StaticCell::new();
    let rmt_buffer: &'static mut RmtBuffType = RMT_BUFF.init(smart_led_buffer!(NUM_LEDS));
    let rgb_led = SmartLedsAdapter::new(rmt.channel0, peripherals.GPIO8, rmt_buffer);

    // Configure PWM
    static LS_TIMER: StaticCell<timer::Timer<'static, LowSpeed>> = StaticCell::new();
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

    // Spawn tasks
    spawner.spawn(rgb_task(rgb_led)).unwrap();
    spawner.spawn(blink_task(pwm_channel0)).unwrap();
    spawner.must_spawn(display_task(i2c));
    spawner.must_spawn(connection(controller));
    spawner.must_spawn(net_task(runner));

    wait_for_connection(stack).await;

    static CLOCK: StaticCell<Clock> = StaticCell::new();
    let clock = CLOCK.init(Clock::new());

    loop {
        access_website(stack, seed, clock).await;
        Timer::after_millis(5000).await;
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
    display.init().expect("Error initializing display");

    let thin_stroke = PrimitiveStyle::with_stroke(BinaryColor::On, 1);
    let thin_stroke_off = PrimitiveStyle::with_stroke(BinaryColor::Off, 1);

    static CENTER: Point = Point::new(13, 10);
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
            .draw(&mut display).expect("Unable to draw circle1");
        Circle::new(CENTER - convert_tl_to_center(32), 32)
            .into_styled(thin_stroke)
            .draw(&mut display).expect("Unable to draw circle2");
        Circle::new(CENTER - convert_tl_to_center(44), 44)
            .into_styled(thin_stroke)
            .draw(&mut display).expect("Unable to draw circle3");
        Circle::new(CENTER - convert_tl_to_center(56), 56)
            .into_styled(thin_stroke)
            .draw(&mut display).expect("Unable to draw circle4");
        // Draw the radio line
        let (end_x, end_y) = sincosf(angle);
        let line = Line::new(CENTER, CENTER+Point::new(((28 as f32) * end_x) as i32, ((28 as f32) * end_y) as i32));
        line.into_styled(thin_stroke)
            .draw(&mut display).expect("Unable to draw line");

        display.flush().expect("Unable to flush display");

        // Clear the radio line in preperation of drawing the next frame
        line.into_styled(thin_stroke_off)
            .draw(&mut display).expect("Unable to UNdraw line");

        // Update the angle for the next frame
        angle -= angle_update;
        if angle > (2f32 * PI) {
            angle = 0f32;
        }

        // Pend until the next frame is ready to be drawn
        period_timer.await;
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
    //info!("Device capabilities: {:?}", controller.capabilities().unwrap());
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

            info!("Scan");
            let scan_config = ScanConfig::default().with_max(10);
            let result = controller
                .scan_with_config_async(scan_config)
                .await
                .expect("Unable to set controller to scan");
            for ap in result {
                info!("{:?}", ap);
            }
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

async fn access_website(
    stack: embassy_net::Stack<'_>,
    tls_seed: u64,
    clock: &'static Clock,
) {
    const BUFF_SZ: usize = 4096;
    const HOST: &str = "pool.ntp.org";
    let mut rx_meta = [PacketMetadata::EMPTY; 16];
    let mut rx_buffer = [0; BUFF_SZ];
    let mut tx_meta = [PacketMetadata::EMPTY; 16];
    let mut tx_buffer = [0; BUFF_SZ];

    let mut socket = UdpSocket::new(stack, &mut rx_meta, &mut rx_buffer, &mut tx_meta, &mut tx_buffer);
    socket.bind(123).unwrap();
    let socket = UdpSocketWrapper::new(socket);
    let context = NtpContext::new(Timestamp::default());

    let ip_addr = stack.dns_query(HOST, dns::DnsQueryType::A).await.unwrap();
    let addr: IpAddr = ip_addr[0].into();
    let result = get_time(SocketAddr::from((addr, 123)), &socket, context).await.unwrap();
    info!("NTP response seconds: {}", result.seconds);
    let now = DateTime::from_timestamp(result.seconds as i64, 0).unwrap();
    clock.set_time(now).await;

    info!("Current time: {}", clock.get_date_time_str().await.as_str());
}

struct Clock {
    sys_start: Mutex<CriticalSectionRawMutex, DateTime<Utc>>,
}

impl Clock {
    pub(crate) fn new() -> Self {
        Self {
            sys_start: Mutex::new(DateTime::UNIX_EPOCH),
        }
    }

    pub(crate) async fn set_time(&self, now: DateTime<Utc>) {
        let mut sys_start = self.sys_start.lock().await;
        let elapsed = Instant::now().as_millis();
        *sys_start = now
            .checked_sub_signed(chrono::Duration::milliseconds(elapsed as i64))
            .expect("sys_start greater than current_ts")
            .checked_sub_signed(chrono::Duration::hours(8))
            .expect("trouble setting timezone")
    }

    pub(crate) async fn now(&self) -> DateTime<Utc>
    {
        let sys_start = self.sys_start.lock().await;
        let elapsed = Instant::now().as_millis();
        *sys_start + chrono::Duration::milliseconds(elapsed as i64)
    }

    pub(crate) async fn get_date_time_str(&self) -> String<10> {
        let dt = self.now().await;
        let day_title = match dt.weekday() {
            Weekday::Mon => "Mon",
            Weekday::Tue => "Tue",
            Weekday::Wed => "Wed",
            Weekday::Thu => "Thu",
            Weekday::Fri => "Fri",
            Weekday::Sat => "Sat",
            Weekday::Sun => "Sun",
        };
        let hours = dt.hour();
        let minutes = dt.minute();
        let seconds = dt.second();

        let mut result = String::<10>::new();
        //let time_delimiter = if seconds % 2 == 0 { ":" } else { " " };
        let time_delimiter = ":";
        write!(result, "{day_title} {hours:02}{time_delimiter}{minutes:02}").unwrap();
        result
    }
}