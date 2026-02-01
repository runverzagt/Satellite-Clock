use defmt::{info, error};
use embassy_net::{dns};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, mutex::Mutex};
use embassy_time::{Duration, Timer, Instant, with_timeout, TimeoutError};
use embassy_net::{ udp::{PacketMetadata, UdpSocket}};
use esp_radio::wifi::{ClientConfig, PowerSaveMode, WifiStaState};
use esp_radio::wifi::WifiApState;
use sntpc::{get_time, NtpContext, NtpTimestampGenerator, sntp_send_request, sntp_process_response};
use sntpc_net_embassy::UdpSocketWrapper;
use chrono::{ DateTime, Datelike, Timelike, Utc, Weekday};
use core::net::{Ipv4Addr, SocketAddr};
use heapless::String;
use core::fmt::Write;
use thiserror_no_std::Error;

use esp_alloc as _;
use esp_backtrace as _;

use esp_println as _;


// NTP Client implementation heavily inspired by https://github.com/vpikulik/sntpc_embassy/tree/main
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

#[derive(Error, Debug)]
pub enum SntpcError {
    #[error("to_socket_addrs")]
    ToSocketAddrs,
    #[error("no addr")]
    NoAddr,
    #[error("udp send")]
    UdpSend,
    #[error("dns query error")]
    DnsQuery(#[from] embassy_net::dns::Error),
    #[error("dns query error")]
    DnsEmptyResponse,
    #[error("sntc")]
    Sntc(#[from] sntpc::Error),
    #[error("can not parse ntp response")]
    BadNtpResponse,
    #[error("Timeout waiting for response from NTP server")]
    NetworkTimeout,
}

impl From<SntpcError> for sntpc::Error {
    fn from(err: SntpcError) -> Self {
        match err {
            SntpcError::ToSocketAddrs => Self::AddressResolve,
            SntpcError::NoAddr => Self::AddressResolve,
            SntpcError::UdpSend => Self::Network,
            _ => todo!(),
        }
    }
}

pub(crate) struct Clock {
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
        let time_delimiter = if seconds % 2 == 0 { ":" } else { " " };
        write!(result, "{day_title} {hours:02}{time_delimiter}{minutes:02}").unwrap();
        result
    }
}

#[embassy_executor::task]
pub async fn ntp_worker(
    stack: embassy_net::Stack<'static>,
    clock: &'static Clock,
) {
    loop {
        let sleep_sec : u64;
        match esp_radio::wifi::sta_state() {
            WifiStaState::Connected => {
                info!("NTP Request");
                sleep_sec = match ntp_request(stack,  clock).await {
                    Err(_) => {
                        error!("NTP error Response");
                        10
                    }
                    Ok(_) => 3600,
                };
            }
            _ => {
                sleep_sec = 600;
            }
        }
        Timer::after(Duration::from_secs(sleep_sec)).await;
    }
}

async fn ntp_request  (
    stack: embassy_net::Stack<'_>,
    clock: &'static Clock,
) -> Result<(), SntpcError> {
    const BUFF_SZ: usize = 4096;
    const HOST: &str = "time.cloudflare.com";

    info!("Prepare NTP lookup");
    let mut ip_addr = stack.dns_query(HOST, dns::DnsQueryType::A).await?;
    let addr= ip_addr.pop().ok_or(SntpcError::DnsEmptyResponse)?;
    info!("NTP DNS: {:?}", addr);

    let s_addr = SocketAddr::from((addr, 123));

    let mut rx_meta = [PacketMetadata::EMPTY; 16];
    let mut rx_buffer = [0; BUFF_SZ];
    let mut tx_meta = [PacketMetadata::EMPTY; 16];
    let mut tx_buffer = [0; BUFF_SZ];

    let mut socket = UdpSocket::new(stack, &mut rx_meta, &mut rx_buffer, &mut tx_meta, &mut tx_buffer);
    socket.bind(1234).expect("Unable to bind to UDP socket");

    let socket_wrapper = UdpSocketWrapper::new(socket);
    let context = NtpContext::new(Timestamp::default());

    stack.wait_config_up().await;

    info!("Requesting time...");
    let result = match with_timeout(Duration::from_secs(60), get_time(s_addr, &socket_wrapper, context)).await {
        Ok(a) => a.unwrap(),
        Err(_) => return Err(SntpcError::NetworkTimeout)
    };
    info!("NTP response seconds: {}, Roundtrip: {}us", result.seconds, result.roundtrip());
    let now = DateTime::from_timestamp(result.seconds as i64, 0).ok_or(SntpcError::BadNtpResponse)?;
    clock.set_time(now).await;
    info!("Current time: {}", clock.get_date_time_str().await.as_str());

    Ok(())
}