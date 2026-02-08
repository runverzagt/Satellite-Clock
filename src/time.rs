use defmt::{info, error};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, mutex::Mutex};
use embassy_time::{Duration, Timer, Instant, with_timeout};
use embassy_net::{ 
    dns,
    udp::{PacketMetadata, UdpSocket}, 
    dns::DnsSocket,
    tcp::client::{TcpClient, TcpClientState},
};
use esp_radio::wifi::{WifiStaState};
use reqwless::client::{HttpClient, TlsConfig};
use sntpc::{get_time, NtpContext, NtpTimestampGenerator};
use sntpc_net_embassy::UdpSocketWrapper;
use chrono::{ DateTime, Datelike, Timelike, Utc, Weekday};
use chrono::{FixedOffset, TimeZone};
use core::{fmt::Debug, net::{SocketAddr}};
use heapless::String;
use core::fmt::Write;
use core::option::Option;
use thiserror_no_std::Error;
use serde_json::{Value};

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

#[derive(Error, Debug, defmt::Format)]
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
    is_set: bool,
}

impl Clock {
    pub(crate) fn new() -> Self {
        Self {
            sys_start: Mutex::new(DateTime::UNIX_EPOCH),
            is_set: false
        }
    }

    pub(crate) async fn set_time(&self, now: DateTime<Utc>) {
        let mut sys_start = self.sys_start.lock().await;
        let elapsed = Instant::now().as_millis();
        *sys_start = now
            .checked_sub_signed(chrono::Duration::milliseconds(elapsed as i64))
            .expect("sys_start greater than current_ts")
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

    pub(crate) async fn set_timezone(&self, offset: i32) {
        info!("Updating TZ");
        let mut sys_start = self.sys_start.lock().await;
        *sys_start = sys_start.checked_add_signed(chrono::Duration::hours(offset as i64))
            .expect("trouble setting timezone");
    }
}

#[embassy_executor::task]
pub async fn ntp_worker(
    stack: embassy_net::Stack<'static>,
    clock: &'static Clock,
    tls_seed: u64,
) {
    loop {
        let sleep_sec : u64 = match esp_radio::wifi::sta_state() {
            WifiStaState::Connected => {
                info!("NTP Request");
                let ss = match ntp_request(stack,  clock).await {
                    Err(e) => {
                        error!("NTP error Response: {:?}", e);
                        10
                    }
                    Ok(_) => 3600,
                };
                info!("Get Location");
                match get_timezone(stack, tls_seed).await {
                    Ok(offset) => {
                        match offset {
                            Some(o) => {
                                clock.set_timezone(o).await;
                                info!("New time: {}", clock.get_date_time_str().await.as_str());
                            }
                            None => {
                                error!("Unable to parse timezone")
                            }
                        }
                    }
                    Err(e) => {
                        error!("Timezone Error: {:?}", e);
                    }
                }
                ss
            }
            // If the wifi isn't connected, sleep for 1 minute to see if it comes up
            _ => 600,
        };
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
        Ok(a) => a?,
        Err(_) => return Err(SntpcError::NetworkTimeout)
    };
    info!("NTP response seconds: {}, Roundtrip: {}us", result.seconds, result.roundtrip());
    let now = DateTime::from_timestamp(result.seconds as i64, 0).ok_or(SntpcError::BadNtpResponse)?;
    clock.set_time(now).await;
    info!("Current time: {}", clock.get_date_time_str().await.as_str());

    Ok(())
}

#[derive(Error, Debug, defmt::Format)]
enum TimezoneError {
    NoInternet,
    CouldNotConnect,
    JsonParseError
}

async fn get_timezone(
    stack: embassy_net::Stack<'_>,
    tls_seed: u64,
) -> Result<Option<i32>, TimezoneError>  {
    let mut rx_buffer = [0; 16640];
    let mut tx_buffer = [0; 4096];
    const BUFF_SZ: usize = 4096;
    const URL: &str = "https://myip.foo/api";

    match esp_radio::wifi::sta_state() {
        WifiStaState::Connected => {}
        _ => {return Err(TimezoneError::NoInternet)}
    };

    let dns = DnsSocket::new(stack);
    let tcp_state = TcpClientState::<1, 4096, 4096>::new();
    let tcp = TcpClient::new(stack, &tcp_state);

    let tls = TlsConfig::new(
        tls_seed,
        &mut rx_buffer,
        &mut tx_buffer,
        reqwless::client::TlsVerify::None,
    );

    let mut client = HttpClient::new_with_tls(&tcp, &dns, tls);
    let mut buffer = [0u8; 4096];
    let mut http_req = client
        .request(
            reqwless::request::Method::GET,
            URL,
        )
        .await
        .unwrap();
    let response = http_req.send(&mut buffer).await.unwrap();

    info!("Got response");
    let res = response.body().read_to_end().await.unwrap();

    let content = core::str::from_utf8(res).unwrap();
    info!("{}", content);

    let tz_string: Value = match serde_json::from_str(content) {
        Ok(s) => {s}
        Err(e) => {return Err(TimezoneError::JsonParseError)}
    };
    info!("{:?}", tz_string["location"]["timezone"].as_str().unwrap());

    let offset = convert_tz_str_to_offset(tz_string["location"]["timezone"].as_str().unwrap());

    info!("offset: {}", offset);

    return Ok(offset)
}

fn convert_tz_str_to_offset(tz: &str) -> Option<i32> {
    // We have to use this weird syntax because match doesn't use PartialEq
    match tz {
        _ if tz == "America/Los_Angeles" => Some(-8),
        _ if tz == "America/Denver" => Some(-7),
        _ if tz == "America/Chicago" => Some(-6),
        _ if tz == "America/New_York" => Some(-4),
        _ => None,
    }
}