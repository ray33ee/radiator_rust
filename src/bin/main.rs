#![no_std]
#![no_main]
#![feature(asm_experimental_arch)]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]
#![warn(clippy::all)]
#![warn(clippy::pedantic)]
#![warn(clippy::nursery)]
#![warn(clippy::cargo)]
#![warn(unsafe_code)]
#![warn(unused_extern_crates)]
#![warn(rust_2018_idioms)]
#![warn(missing_debug_implementations)]
#![warn(missing_copy_implementations)]
#![warn(unreachable_pub)]
#![warn(unused)]
#![warn(dead_code)]
#![warn(nonstandard_style)]
#![warn(trivial_casts)]
#![warn(trivial_numeric_casts)]
#![warn(variant_size_differences)]
extern crate alloc;

mod storages;
mod commands;
mod motor;
mod thermo;
mod rtc;
mod rgb;
mod fsm;
mod mode_button;
mod backtrace;
mod flashstore;

use embassy_futures::select::Either::First;
use esp_println::{println, print};
use alloc::{format, vec};
use alloc::string::String;
use alloc::vec::Vec;
use core::net::{IpAddr, Ipv4Addr};
use static_cell::StaticCell;
use esp_wifi::wifi::WifiDevice;
use esp_hal::{
    clock::CpuClock,
    main,
    timer::{
        timg::TimerGroup,
        systimer::SystemTimer,
    },
    rng::Rng,
    time::Instant,
    ledc::{
        timer::TimerIFace,
        channel::{ChannelIFace, ChannelHW},
        LSGlobalClkSource,
        Ledc,
    },
    gpio::{
        InputConfig,
        OutputConfig,
        Pull,
        Level,
        Output,
    },
    uart::{Uart},
    otg_fs::{Usb, UsbBus},
};
use crate::{
    motor::Motor,
    fsm::{
        BrightnessEntry,
        ScheduleEntry,
        Variant,
        FSM,
    },
    rgb::{State as RGBState, RGBLED, Color},
    rtc::RTC,
    thermo::Thermometer,

};
use embassy_executor::Spawner;
use embassy_time::{Delay, Duration, Timer};
use embassy_sync::{
    channel::Channel,
    blocking_mutex::raw::NoopRawMutex,
    mutex::Mutex,
};
use embassy_net::{
    tcp::TcpSocket,
    Stack,
    StackResources
};
use embedded_io::Write;
use serde_json::Value;
use smoltcp::wire::IpEndpoint;
use time::OffsetDateTime;
use usb_device::bus::UsbBusAllocator;
use crate::fsm::{CalibrateType, State, WarningType};
use crate::motor::ABSOLUTE_MAX_POSITION;
use usb_device::prelude::{UsbDeviceBuilder, UsbVidPid};
use usbd_serial::{SerialPort, USB_CLASS_CDC};
use crate::storages::WIFI_ADDRESS;

// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();

static RED_CH: StaticCell<esp_hal::ledc::channel::Channel<'static, esp_hal::ledc::LowSpeed>> = StaticCell::new();
static mut RED_REF: Option<&'static mut esp_hal::ledc::channel::Channel<'static, esp_hal::ledc::LowSpeed>> = None;

static GREEN_CH: StaticCell<esp_hal::ledc::channel::Channel<'static, esp_hal::ledc::LowSpeed>> = StaticCell::new();
static mut GREEN_REF: Option<&'static mut esp_hal::ledc::channel::Channel<'static, esp_hal::ledc::LowSpeed>> = None;

static BLUE_CH: StaticCell<esp_hal::ledc::channel::Channel<'static, esp_hal::ledc::LowSpeed>> = StaticCell::new();
static mut BLUE_REF: Option<&'static mut esp_hal::ledc::channel::Channel<'static, esp_hal::ledc::LowSpeed>> = None;


static INDEX_HTML: &[u8] = core::include_bytes!("../../web/index.html");
static SETTINGS_HTML: &[u8] = core::include_bytes!("../../web/settings.html");
static CALIBRATE_HTML: &[u8] = core::include_bytes!("../../web/calibrate.html");
static THERMOSTAT_HTML: &[u8] = core::include_bytes!("../../web/thermostat.html");
static NETWORK_HTML: &[u8] = core::include_bytes!("../../web/network.html");
static SUMMER_HTML: &[u8] = core::include_bytes!("../../web/summer.html");
static WINTER_HTML: &[u8] = core::include_bytes!("../../web/winter.html");
static BRIGHTNESS_HTML: &[u8] = core::include_bytes!("../../web/brightness.html");
static STYLES_CSS: &[u8] = core::include_bytes!("../../web/styles.css");
static FAVICON: &[u8] = core::include_bytes!("../../web/favicon.png");

pub(crate) const DUTY_MAX: u32 = 1 << 12;


static mut EP_MEMORY: [u32; 1024] = [0; 1024];

#[embassy_executor::task]
async fn net_task(mut runner: embassy_net::Runner<'static, WifiDevice<'static>>) -> ! {
    runner.run().await;
}


#[embassy_executor::task]
async fn world_time_task(
    stack: Stack<'static>,
    time_mutex: & 'static Mutex<NoopRawMutex, RTC>,

) -> ! {

    let mut rx_buf = [0u8; 2024];
    let mut tx_buf = [0u8; 2024];

    loop {

        let mut socket = TcpSocket::new(stack, &mut rx_buf, &mut tx_buf);

        let world_time_api_dns = "worldtimeapi.org";
        let world_time_api_port = 80;

        if let Ok(time_api_addrs) = stack.dns_query(world_time_api_dns, smoltcp::wire::DnsQueryType::A).await {
            if time_api_addrs.len() > 0 {

                let time_api_addr = time_api_addrs[0];

                let time_endpoint = IpEndpoint::new(time_api_addr, world_time_api_port);

                if let Ok(_) = socket.connect(time_endpoint).await {

                    if let Ok(_) = socket.write(b"GET /api/ip HTTP/1.1\r\n\
Host: worldtimeapi.org\r\n\
User-Agent: esp32-rust\r\n\
Connection: close\r\n\
\r\n").await        {
                        let result_got_time = socket.read_with(|bytes| {

                            let mut headers = [httparse::EMPTY_HEADER; 40];

                            let mut parser = httparse::Response::new(& mut headers);

                            if let Ok(thing) = parser.parse(bytes) {
                                if let httparse::Status::Complete(start) = thing {
                                    let json_str = core::str::from_utf8(&bytes[start..]).unwrap();

                                    let parsed: serde_json::Value = serde_json::from_str(json_str).unwrap();

                                    let datetime_string = parsed.get("datetime").unwrap().as_str();

                                    return (bytes.len(), Some(RTC::epoch_from_iso(datetime_string.unwrap())));

                                }
                            }

                            (bytes.len(), None)

                        }).await;

                        if let Ok(got_time) = result_got_time {
                            if let Some(epoch) = got_time {
                                let mut guard = time_mutex.lock().await;

                                guard.update_epoch(epoch);


                                socket.close();

                                //Put this thread to sleep when the time is found
                                break;
                            }
                        }

                        println!("Time not found");

                        socket.close();
                    } else {
                        socket.close();
                    }


                }


            }
        }




        Timer::after(Duration::from_secs(3)).await;

    }

    //If the time has been acquired, put the thread to sleep
    loop {
        Timer::after(Duration::from_secs(1)).await;
    }
}

async fn send_css<'a>(socket: & mut TcpSocket<'a>) {
    use embedded_io_async::Write;

    socket.write_all(b"HTTP/1.1 ").await.unwrap();
    socket.write_all(b"202 OK").await.unwrap();
    socket.write_all(b"\r\nContent-Type: ").await.unwrap();
    socket.write_all(b"text/css").await.unwrap();
    socket.write_all(b"\r\nContent-Length: ").await.unwrap();
    socket.write_all(format!("{}", STYLES_CSS.len()).as_bytes()).await.unwrap();
    // cache CSS so reloads don’t flash
    let _ = socket.write_all(b"\r\nCache-Control: public, max-age=604800").await;
    socket.write_all(b"\r\nConnection: close\r\n\r\n").await.unwrap();
    socket.write_all(STYLES_CSS).await.unwrap();
}

async fn send_favicon<'a>(socket: & mut TcpSocket<'a>) {
    use embedded_io_async::Write;

    socket.write_all(b"HTTP/1.1 ").await.unwrap();
    socket.write_all(b"202 OK").await.unwrap();
    socket.write_all(b"\r\nContent-Type: ").await.unwrap();
    socket.write_all(b"text/css").await.unwrap();
    socket.write_all(b"\r\nContent-Length: ").await.unwrap();
    socket.write_all(format!("{}", STYLES_CSS.len()).as_bytes()).await.unwrap();
    // cache CSS so reloads don’t flash
    let _ = socket.write_all(b"\r\nCache-Control: public, max-age=604800").await;
    socket.write_all(b"\r\nConnection: close\r\n\r\n").await.unwrap();
    socket.write_all(FAVICON).await.unwrap();
}

//todo: fix the length parameter so it matches the length of a templated file
async fn send_template<'a, 'x>(mime: & str, template: & [u8], socket: & mut TcpSocket<'a>, replacer: & 'x alloc::vec::Vec<& 'x str>) -> Result<(), embassy_net::tcp::Error> {

    use embedded_io_async::Write;

    socket.write_all(b"HTTP/1.1 200 OK\r\n").await?;
    socket.write_all(b"Content-Type: ").await?;
    socket.write_all(mime.as_bytes()).await?;
    socket.write_all(b"; charset=utf-8\r\n").await?;

    // 2) Content length

    let length = template.len() - (replacer.len() % 10) * 5 - (replacer.len() / 10) * 6 + replacer.iter().map(|&x| x.len()).sum::<usize>();

    socket.write_all(b"Content-Length: ").await?;
    socket.write_all(format!("{}", length).as_bytes()).await?;
    socket.write_all(b"\r\n").await?;

    // 3) Connection header
    socket.write_all(b"Connection: close\r\n").await?;

    // 4) End of headers
    socket.write_all(b"\r\n").await?;

    // 5) Body

    let html_str = core::str::from_utf8(template).unwrap();

    // First split at every `{{`
    let mut parts = html_str.split("{{");

    // The first chunk is always literal HTML
    if let Some(first) = parts.next() {
        socket.write_all(first.as_bytes()).await?;
    }

    // Every other chunk starts with a token (maybe) followed by "}}"
    for part in parts {
        if let Some((token, rest)) = part.split_once("}}") {
            // (Don’t send the token itself for now)
            let index = token.trim().parse::<usize>().unwrap();
            socket.write_all(replacer[index].as_bytes()).await?;

            socket.write_all(rest.as_bytes()).await?;
        } else {
            // No closing braces — treat as literal
            socket.write_all(b"{{").await?;
            socket.write_all(part.as_bytes()).await?;
        }
    }

    Ok(())
    //socket.write_all(INDEX_HTML).await.unwrap();
}

async fn send_403<'a>(socket: &mut TcpSocket<'a>) {
    use embedded_io_async::Write;

    let _ = socket.write_all(b"HTTP/1.1 403 Forbidden\r\n").await;
    let _ = socket.write_all(b"Content-Length: 0\r\n").await;
    let _ = socket.write_all(b"Connection: close\r\n\r\n").await;
}

async fn send_400<'a>(socket: &mut TcpSocket<'a>) {
    use embedded_io_async::Write;

    let _ = socket.write_all(b"HTTP/1.1 400 Bad Request\r\n").await;
    let _ = socket.write_all(b"Content-Length: 0\r\n").await;
    let _ = socket.write_all(b"Connection: close\r\n\r\n").await;
}

async fn send_404<'a>(socket: & mut TcpSocket<'a>) {
    use embedded_io_async::Write;

    let body = b"<!doctype html>\
<html lang=\"en\">\
<head><meta charset=\"utf-8\"><title>404 Not Found</title></head>\
<body style=\"font-family:sans-serif; text-align:center; padding:2em;\">\
  <h1>404 Not Found</h1>\
  <p>The requested resource was not found on this server.</p>\
</body></html>";

    let _ = socket.write_all(b"HTTP/1.1 404 Not Found\r\n").await;
    let _ = socket.write_all(b"Content-Type: text/html; charset=utf-8\r\n").await;
    let _ = socket.write_all(b"Connection: close\r\n\r\n").await;
    let _ = socket.write_all(body).await;
}

async fn send_200<'a>(socket: &mut TcpSocket<'a>, body: Option<& str>) {
    use embedded_io_async::Write;

    let _ = socket.write_all(b"HTTP/1.1 200 OK\r\n").await;
    let _ = socket.write_all(b"Content-Length: ").await;
    let _ = socket.write_all(format!("{}", body.as_ref().map(|x| x.len()).unwrap_or_else(||0)).as_bytes()).await;
    let _ = socket.write_all(b"\r\n").await;
    let _ = socket.write_all(b"Connection: close\r\n\r\n").await;
    if let Some(body) = body {
        let _ = socket.write_all(body.as_bytes()).await;
    }
}

async fn send_schedule<'a>(socket: &mut TcpSocket<'a>, schedule: & Vec<ScheduleEntry>) {

    send_200(socket, Some(serde_json::to_string(schedule).unwrap().as_str())).await;

}

async fn send_brightness_schedule<'a>(socket: &mut TcpSocket<'a>, schedule: & Vec<BrightnessEntry>) {

    send_200(socket, Some(serde_json::to_string(schedule).unwrap().as_str())).await;

}

enum RequestMain {
    GetStatus,
    Reset,
    Panic,
    Exception,
    ShortBoost,
    LongBoost,
    Cancel,
    SetShortDuration(u32),
    SetLongDuration(u32),
    Rainbow,
    CalibrateInfo,
    ThermostatInfo,
    NetworkInfo,
    ModeCalibrate,
    ModeSafe,
    SyncTime(i64),
    GetState,
    Descale,

    SetMax(u32),
    Push(u32),
    Pull(u32),
    UnlockZero,
    Lock,
    SettingsPage,

    SetSummer,
    SetWinter,

    SetThermostat(f32),

    GetSummerSchedule,
    GetWinterSchedule,
    GetBrightnessSchedule,

    SetSummerSchedule(Vec<ScheduleEntry>),
    SetWinterSchedule(Vec<ScheduleEntry>),
    SetBrightnessSchedule(Vec<BrightnessEntry>),

    SetStaticIp(bool, embassy_net::Ipv4Address, embassy_net::Ipv4Address, embassy_net::Ipv4Address, embassy_net::Ipv4Address, embassy_net::Ipv4Address)

}

enum ResponseMain {
    Status{
        temperature: Option<f32>,
        thermostat: f32,
        variant: fsm::Variant,
        current_time: Option<OffsetDateTime>,
        up_time: esp_hal::time::Instant,
        current_state: fsm::State,
        position: u32,
        max_pos: u32,
    },
    Motor {
        max_position: u32,
        is_locked: bool,
        position: u32,
    },
    Settings {
        reset_reason: u32,
        //core_temperature: Option<f32>,
        current_time: Option<OffsetDateTime>,
        brightness: u8,
    },
    Thermostat {
        thermostat: f32,
        short_duration: u32,
        long_duration: u32,
    },
    Network {
        mac_addr: [u8; 6],
        is_static: bool,
        ip_addr: embassy_net::Ipv4Address,
        gateway: embassy_net::Ipv4Address,
        dns1: embassy_net::Ipv4Address,
        dns2: embassy_net::Ipv4Address,
        dns3: embassy_net::Ipv4Address,

    },
    State {
        state: fsm::State,
    },
    SummerSchedule {
        sched: Vec<ScheduleEntry>,
    },
    WinterSchedule {
        sched: Vec<ScheduleEntry>,
    },
    BrightnessSchedule {
        sched: Vec<BrightnessEntry>,
    }


}

#[embassy_executor::task]
async fn webserver_task(
    stack: Stack<'static>,
    request: &'static Channel<NoopRawMutex, RequestMain, 1>,
    response: &'static Channel<NoopRawMutex, ResponseMain, 1>,
) -> ! {
    use httparse::{Request, EMPTY_HEADER};

    let mut rx_buf = [0u8; 3024];
    let mut tx_buf = [0u8; 3024];

    loop {
        let mut socket = TcpSocket::new(stack, &mut rx_buf, &mut tx_buf);
        socket.set_timeout(Some(Duration::from_secs(10)));

        if let Err(e) = socket.accept(80).await {
            println!("Accept failed: {:?}", e);
            continue;
        }

        let mut buf = [0u8; 3024];

        if let Ok(n) = socket.read(&mut buf).await {
            if n != 0 {
                // Try to parse the buffer as an HTTP request
                let mut headers = [EMPTY_HEADER; 16];
                let mut req = Request::new(&mut headers);

                match req.parse(&buf[..n]) {
                    Ok(status) if status.is_complete() => {
                        let body = core::str::from_utf8(&buf[status.unwrap()..n]).unwrap();
                        println!("Body: {}", body);
                        let method = req.method.unwrap_or("<none>");
                        let path = req.path.unwrap_or("<none>");

                        let (path, key, value) = match path.split_once("?") {
                            Some((path, query)) => {
                                match query.split_once("=") {
                                    Some((key, value)) => (path, Some(key), Some(value)),
                                    None => (path, None, None),
                                }
                            }
                            None => (path, None, None)
                        };

                        println!("Request found: {} {} ({:?} = {:?})", method, path, key, value);

                        match (method, path) {
                            ("GET", "/styles.css") => {

                                send_css(& mut socket).await;

                            }
                            ("GET", "/favicon.png") => {

                                send_favicon(& mut socket).await;

                            }
                            ("GET", "/") => {

                                request.send(RequestMain::GetStatus).await;

                                if let ResponseMain::Status { temperature, thermostat, variant, current_time, up_time, current_state, position, max_pos } = response.receive().await {

                                    let temperature = temperature.map(|t| format!("{:.1}", t)).unwrap_or_else(|| format!("Error"));
                                    let thermostat = format!("{:?}", thermostat);
                                    let variant = format!("{:?}", variant);
                                    let current_time = current_time.map(|t| format!("{:.3} {:02}:{:02}", t.weekday(), t.hour(), t.minute())).unwrap_or_else(|| format!("Error"));

                                    let aux_info = match &current_state {
                                        fsm::State::Boost(instant, duration) => {
                                            let seconds_left = (*duration - instant.elapsed()).as_secs();

                                            let minutes = seconds_left / 60;
                                            let seconds = seconds_left % 60;

                                            if minutes == 0 {
                                                format!("{}s", seconds)
                                            } else {
                                                format!("{}m {}s", minutes, seconds)
                                            }
                                        }
                                        fsm::State::Warning(code) => { format!("{:?}", code) }
                                        fsm::State::Calibrate(t) => { format!("{:?}", t) }
                                        _ => { format!("") }
                                    };

                                    let current_state = format!("{}", current_state);
                                    let valve_state = if position == 0 {
                                        format!("Open")
                                    } else {
                                        if position == max_pos {
                                            format!("Closed")
                                        } else {
                                            format!("Undefined")
                                        }
                                    };

                                    //todo: show extra information with state (warning code, boost duration. etc. add new cards)

                                    //todo: automatically refresh on boost and cancel

                                    let up_time = {
                                        let tu = up_time.elapsed().as_secs();
                                        let dur = time::Duration::seconds(tu as i64);

                                        let days = dur.whole_seconds() / 86400;
                                        let hours = (dur.whole_seconds() % 86_400) / 3_600;
                                        let minutes = (dur.whole_seconds() % 3_600) / 60;
                                        let seconds = dur.whole_seconds() % 60;

                                        if days == 0 {
                                            if hours == 0 {
                                                if minutes == 0 {
                                                    format!("{}s", seconds)
                                                } else {
                                                    format!("{}m {}s", minutes, seconds)
                                                }
                                            } else {
                                                format!("{}h {}m {}s", hours, minutes, seconds)
                                            }
                                        } else {
                                            format!("{}d {}h {}m {}s", days, hours, minutes, seconds)
                                        }
                                    };

                                    let cards = vec![
                                        temperature.as_str(),
                                        thermostat.as_str(),
                                        variant.as_str(),
                                        current_time.as_str(),
                                        up_time.as_str(),
                                        current_state.as_str(),
                                        aux_info.as_str(),
                                        valve_state.as_str(),
                                    ];


                                    let _ = send_template("text/html", INDEX_HTML, & mut socket, &cards).await;
                                } else {
                                    send_404(& mut socket).await;
                                }

                            }
                            ("POST", "/") => {
                                match (key, value) {
                                    (Some("boost"), Some("short")) => {
                                        println!("Short boost");
                                        request.send(RequestMain::ShortBoost).await;
                                        send_200(& mut socket, None).await;
                                    }
                                    (Some("boost"), Some("long")) => {
                                        println!("Long boost");
                                        request.send(RequestMain::LongBoost).await;
                                        send_200(& mut socket, None).await;
                                    }
                                    (Some("action"), Some("cancel")) => {
                                        println!("Cancel!");
                                        request.send(RequestMain::Cancel).await;
                                        send_200(& mut socket, None).await;
                                    }
                                    (Some("schedule"), Some("summer")) => {
                                        request.send(RequestMain::SetSummer).await;
                                        send_200(& mut socket, None).await;
                                    }
                                    (Some("schedule"), Some("winter")) => {
                                        request.send(RequestMain::SetWinter).await;
                                        send_200(& mut socket, None).await;
                                    }
                                    (Some("action"), Some("descale")) => {
                                        println!("Descale!");
                                        request.send(RequestMain::Descale).await;
                                        send_200(& mut socket, None).await;
                                    }
                                    _ => send_404(&mut socket).await,
                                }
                            }
                            ("GET", "/settings") => {

                                request.send(RequestMain::SettingsPage).await;

                                if let ResponseMain::Settings { reset_reason, current_time, brightness } = response.receive().await {

                                    let reset_code = format!("{}", reset_reason);

                                    let reset_reason = match reset_reason {
                                        1 => "Reset due to power-on event",
                                        2 => "Reset by external pin (N/A) for ESP32",
                                        3 => "Software reset via esp_restart",
                                        4 => "Software reset due to panic/exception",
                                        5 => "Reset (hardware or software) due to interrupt watchdog",
                                        6 => "Reset due to task watchdog",
                                        7 => "Reset due to other watchdogs",
                                        8 => "Reset after exiting deep sleep mode",
                                        9 => "Brownout reset (software or hardware)",
                                        10 => "Reset over SDIO",
                                        11 => "Reset by USB peripheral",
                                        12 => "Reset by JTAG",
                                        13 => "Reset due to efuse error",
                                        14 => "Reset due to power glitch detected",
                                        15 => "Reset due to CPU lock up (double exception)",
                                        21 => "Reset due to flashing device over USB or something maybe?",
                                        _ => "Unknown reset reason",
                                    };

                                    let current_time = current_time.map(|t| format!("{:04}-{:02}-{:02} {:02}:{:02}:{:02}", t.year(), t.month() as u32, t.day(), t.hour(), t.minute(), t.second())).unwrap_or_else(|| format!("Error"));

                                    let brightness = format!("{}", brightness);

                                    let cards = vec![reset_code.as_str(), reset_reason, "NO TEMP", brightness.as_str(), current_time.as_str()];


                                    let _ = send_template("text/html", SETTINGS_HTML, & mut socket, &cards).await;
                                } else {
                                    send_404(&mut socket).await;
                                }

                            }
                            ("POST", "/settings") => {
                                match (key, value) {
                                    (Some("sync_time"), Some(value)) => {
                                        request.send(RequestMain::SyncTime(value.parse::<i64>().unwrap())).await;
                                        println!("Also sync time: {}", value);
                                        send_200(& mut socket, None).await;
                                    }
                                    (Some("reset"), Some("true")) => {
                                        request.send(RequestMain::Reset).await;
                                        println!("Soft reset!");
                                        send_200(& mut socket, None).await;
                                    }
                                    (Some("panic"), Some("true")) => {
                                        request.send(RequestMain::Panic).await;
                                        println!("panic!");
                                        send_200(& mut socket, None).await;
                                    }
                                    (Some("exception"), Some("true")) => {
                                        request.send(RequestMain::Exception).await;
                                        println!("exception!");
                                        send_200(& mut socket, None).await;
                                    }
                                    (Some("action"), Some("rainbow")) => {
                                        request.send(RequestMain::Rainbow).await;
                                        println!("rainbow   !");
                                        send_200(& mut socket, None).await;
                                    }
                                    (Some("action"), Some("safemode")) => {
                                        request.send(RequestMain::ModeSafe).await;
                                        println!("Safe mode mode");
                                        send_200(& mut socket, None).await;
                                    }
                                    (Some("factory"), Some("true")) => {
                                        storages::factory_reset();
                                        send_200(& mut socket, None).await;
                                    }

                                    _ => send_404(&mut socket).await,
                                }
                            }
                            ("GET", "/calibrate") => {
                                request.send(RequestMain::CalibrateInfo).await;

                                if let ResponseMain::Motor { max_position, is_locked, position } = response.receive().await {

                                    let max = format!("{}", max_position);
                                    let is_locked = format!("{}", if is_locked {"Locked"} else {"Unlocked"});
                                    let pos = if storages::is_locked() { format!("Undefined") } else { format!("{}", position) };

                                    let cards = vec![max.as_str(), is_locked.as_str(), pos.as_str(), max.as_str()];


                                    let _ = send_template("text/html", CALIBRATE_HTML, & mut socket, &cards).await;
                                } else {
                                    send_404(& mut socket).await;
                                }

                            }
                            ("POST", "/calibrate") => {

                                request.send(RequestMain::GetState).await;

                                if let ResponseMain::State{ state } = response.receive().await {

                                    match (key, value) {
                                        (Some("action"), Some("calibrate")) => {
                                            request.send(RequestMain::ModeCalibrate).await;
                                        }
                                        (Some("set_max"), Some(max)) => {
                                            if state.is_calibrate() {

                                                let max = max.parse::<u32>().unwrap();

                                                if max <= motor::ABSOLUTE_MAX_POSITION {
                                                    request.send(RequestMain::SetMax(max)).await;
                                                    send_200(& mut socket, None).await;
                                                } else {
                                                    send_400(& mut socket).await;
                                                }

                                            } else {
                                                send_403(& mut socket).await;
                                            }
                                        }
                                        (Some("push"), Some(push)) => {
                                            if state.is_calibrate() {
                                                request.send(RequestMain::Push(push.parse::<u32>().unwrap())).await;
                                                send_200(& mut socket, None).await;
                                            } else {
                                                send_403(& mut socket).await;
                                            }

                                        }
                                        (Some("pull"), Some(pull)) => {
                                            if state.is_calibrate() {
                                                request.send(RequestMain::Pull(pull.parse::<u32>().unwrap())).await;
                                                send_200(& mut socket, None).await;
                                            } else {
                                                send_403(& mut socket).await;
                                            }
                                        }
                                        (Some("action"), Some("lock")) => {
                                            if state.is_calibrate() {
                                                request.send(RequestMain::Lock).await;
                                                send_200(& mut socket, None).await;
                                            } else {
                                                send_403(& mut socket).await;
                                            }
                                        }
                                        (Some("action"), Some("unlock_zero")) => {
                                            if state.is_calibrate() {
                                                request.send(RequestMain::UnlockZero).await;
                                                send_200(& mut socket, None).await;
                                            } else {
                                                send_403(& mut socket).await;
                                            }
                                        }

                                        _ => send_404(&mut socket).await,
                                    }
                                }

                            }
                            ("GET", "/thermostat") => {
                                request.send(RequestMain::ThermostatInfo).await;

                                if let ResponseMain::Thermostat { thermostat, short_duration, long_duration } = response.receive().await {

                                    let thermostat = format!("{}", thermostat);

                                    let short = format!("{}", short_duration / 60u32);
                                    let long = format!("{}", long_duration / 60u32);

                                    let cards = vec![thermostat.as_str(), short.as_str(), long.as_str()];


                                    let _ = send_template("text/html", THERMOSTAT_HTML, & mut socket, &cards).await;
                                } else {
                                    send_404(& mut socket).await;
                                }

                            }
                            ("POST", "/thermostat") => {
                                match (key, value) {
                                    (Some("set_thermostat"), Some(temperature)) => {
                                        request.send(RequestMain::SetThermostat(temperature.parse::<f32>().unwrap())).await;
                                        send_200(& mut socket, None).await;
                                    }
                                    (Some("short_duration"), Some(duration)) => {
                                        request.send(RequestMain::SetShortDuration(duration.parse::<u32>().unwrap() * 60)).await;
                                        println!("Set short duration");
                                        send_200(& mut socket, None).await;
                                    }
                                    (Some("long_duration"), Some(duration)) => {
                                        request.send(RequestMain::SetLongDuration(duration.parse::<u32>().unwrap() * 60)).await;
                                        println!("Set long duration");
                                        send_200(& mut socket, None).await;
                                    }
                                    _ => send_404(&mut socket).await,
                                }
                            }
                            ("GET", "/summer") => {


                                match (key, value) {
                                    (None, None) => {
                                        let cards = vec![];

                                        let _ = send_template("text/html", SUMMER_HTML, & mut socket, &cards).await;
                                    }
                                    (Some("start"), Some("true")) => {

                                        request.send(RequestMain::GetSummerSchedule).await;

                                        if let ResponseMain::SummerSchedule { sched } = response.receive().await {

                                            send_schedule(& mut socket, &sched).await;

                                        } else {
                                            send_404(& mut socket).await;
                                        }

                                    }
                                    _ => {
                                        send_404(&mut socket).await;
                                    }
                                }


                            }
                            ("POST", "/summer") => {
                                println!("'{:?}'", body);

                                let v: Vec<ScheduleEntry> = serde_json::from_str(body).unwrap();

                                request.send(RequestMain::SetSummerSchedule(v)).await;

                                send_200(& mut socket, None).await;
                            }
                            ("GET", "/winter") => {


                                match (key, value) {
                                    (None, None) => {
                                        let cards = vec![];

                                        let _ = send_template("text/html", WINTER_HTML, & mut socket, &cards).await;
                                    }
                                    (Some("start"), Some("true")) => {

                                        request.send(RequestMain::GetWinterSchedule).await;

                                        if let ResponseMain::WinterSchedule { sched } = response.receive().await {

                                            send_schedule(& mut socket, &sched).await;

                                        } else {
                                            send_404(& mut socket).await;
                                        }

                                    }
                                    _ => {
                                        send_404(&mut socket).await;
                                    }
                                }


                            }
                            ("POST", "/winter") => {
                                println!("'{:?}'", body);

                                let v: Vec<ScheduleEntry> = serde_json::from_str(body).unwrap();

                                request.send(RequestMain::SetWinterSchedule(v)).await;

                                send_200(& mut socket, None).await;
                            }
                            ("GET", "/brightness") => {


                                match (key, value) {
                                    (None, None) => {
                                        let cards = vec![];

                                        let _ = send_template("text/html", BRIGHTNESS_HTML, & mut socket, &cards).await;
                                    }
                                    (Some("start"), Some("true")) => {

                                        request.send(RequestMain::GetBrightnessSchedule).await;

                                        if let ResponseMain::BrightnessSchedule { sched } = response.receive().await {

                                            send_brightness_schedule(& mut socket, &sched).await;

                                        } else {
                                            send_404(& mut socket).await;
                                        }

                                    }
                                    _ => {
                                        send_404(&mut socket).await;
                                    }
                                }


                            }
                            ("POST", "/brightness") => {
                                println!("'{:?}'", body);

                                let v: Vec<BrightnessEntry> = serde_json::from_str(body).unwrap();

                                request.send(RequestMain::SetBrightnessSchedule(v)).await;

                                send_200(& mut socket, None).await;
                            }
                            ("GET", "/network") => {
                                request.send(RequestMain::NetworkInfo).await;

                                if let ResponseMain::Network { mac_addr, is_static, ip_addr, gateway, dns1, dns2, dns3 } = response.receive().await {

                                    let mac = format!("{:02X}:{:02X}:{:02X}:{:02X}:{:02X}:{:02X}",
                                                      mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);

                                    let static_ip = if ip_addr.to_bits() != 0 { format!("{}", ip_addr) } else { format!("") };
                                    let gateway = if gateway.to_bits() != 0 { format!("{}", gateway) } else { format!("") };
                                    let dns1 = if dns1.to_bits() != 0 { format!("{}", dns1) } else { format!("") };
                                    let dns2 = if dns2.to_bits() != 0 { format!("{}", dns2) } else { format!("") };
                                    let dns3 = if dns3.to_bits() != 0 { format!("{}", dns3) } else { format!("") };

                                    let is_static = if is_static { "true" } else { "false" };

                                    let cards = vec![mac.as_str(), static_ip.as_str(), gateway.as_str(), dns1.as_str(), dns2.as_str(), dns3.as_str(), is_static];


                                    let _ = send_template("text/html", NETWORK_HTML, & mut socket, &cards).await;
                                } else {
                                    send_404(& mut socket).await;
                                }

                            }
                            ("POST", "/network") => {
                                println!("network '{:?}'", body);

                                let v: Value = serde_json::from_str(body).unwrap();

                                println!("is_static: {}", v["is_static"]);
                                println!("ip: {}", v["ip"]);
                                println!("gateway: {}", v["gateway"]);
                                println!("dns: {:?}", v["dns1"]);

                                use core::str::FromStr;

                                let is_static = v["is_static"].as_bool().unwrap();
                                let static_ip = if v["ip"].as_str().unwrap() != "" { embassy_net::Ipv4Address::from_str(v["ip"].as_str().unwrap()).unwrap() } else { embassy_net::Ipv4Address::from_bits(0) };
                                let gateway = if v["gateway"].as_str().unwrap() != "" { embassy_net::Ipv4Address::from_str(v["gateway"].as_str().unwrap()).unwrap() } else { embassy_net::Ipv4Address::from_bits(0) };
                                let dns1 = if v["dns1"].as_str().unwrap() != "" { embassy_net::Ipv4Address::from_str(v["dns1"].as_str().unwrap()).unwrap() } else { embassy_net::Ipv4Address::from_bits(0) };
                                let dns2 = if v["dns2"].as_str().unwrap() != "" { embassy_net::Ipv4Address::from_str(v["dns2"].as_str().unwrap()).unwrap() } else { embassy_net::Ipv4Address::from_bits(0) };
                                let dns3 = if v["dns3"].as_str().unwrap() != "" { embassy_net::Ipv4Address::from_str(v["dns3"].as_str().unwrap()).unwrap() } else { embassy_net::Ipv4Address::from_bits(0) };


                                println!("IP: {}", static_ip);

                                request.send(RequestMain::SetStaticIp (
                                    is_static,
                                    static_ip,
                                    gateway,
                                    dns1,
                                    dns2,
                                    dns3,
                                )).await;


                                send_200(& mut socket, None).await;
                            }
                            _ => {
                                println!("Unknown");
                                send_404(&mut socket).await;
                            }
                        }



                    }
                    Ok(_) => {
                        println!("Partial / incomplete HTTP request");
                    }
                    Err(e) => {
                        println!("Invalid HTTP request: {:?}", e);
                    }
                }
            }
        }

        // ... after sending headers + body:
        let _ = socket.flush().await;
        // give TCP time to drain multi-segment send
        embassy_time::Timer::after(embassy_time::Duration::from_millis(20)).await;
        let _ = socket.close();

    }
}

#[embassy_executor::task]
async fn led_task(
    led_state: & 'static Channel<NoopRawMutex, crate::rgb::State, 1>,
    led_bright: & 'static Channel<NoopRawMutex, u8, 1>,
    initial_state: RGBState,
) -> ! {

    let red_channel = unsafe { RED_REF.as_mut().unwrap() }; //Safe
    let green_channel = unsafe { GREEN_REF.as_mut().unwrap() }; //Safe
    let blue_channel = unsafe { BLUE_REF.as_mut().unwrap() }; //Safe

    let mut rgb_led = RGBLED::new();

    rgb_led.set_state(initial_state);

    loop {


        if let Ok(state) = led_state.try_receive() {
            rgb_led.set_state(state);
        }

        if let Ok(brightness) = led_bright.try_receive() {
            rgb_led.set_brightness(brightness);
        }

        rgb_led.update(|r, g, b| {

            red_channel.set_duty_hw(DUTY_MAX - r);
            green_channel.set_duty_hw(DUTY_MAX - g);
            blue_channel.set_duty_hw(DUTY_MAX - b);
        });

        Timer::after(Duration::from_millis(5)).await;

    }

}

#[embassy_executor::task]
async fn usb_cdc(usb_bus: & 'static UsbBusAllocator<UsbBus<Usb<'static>>>) {
    let mut serial = SerialPort::new(usb_bus);

    let mut usb_dev = UsbDeviceBuilder::new(usb_bus, UsbVidPid(0x303A, 0x3001))
        .device_class(USB_CLASS_CDC)
        .build();

    let mut ssid = [0u8; 64];
    let mut pass = [0u8; 64];

    // None  = not started (for ssid), or not yet entering (for pass)
    // Some(n) = current length collected so far
    let mut ssid_len: Option<usize> = None;
    let mut pass_len: Option<usize> = None;

    let _ = serial.write(b"Press enter to start...\r\n");

    let mut byte = [0u8; 1];

    loop {
        Timer::after(Duration::from_millis(1)).await;

        if !usb_dev.poll(&mut [&mut serial]) {
            continue;
        }

        if let Ok(1) = serial.read(&mut byte) {
            let b = byte[0];

            // --- Stage 0: waiting for Enter to start ---
            if ssid_len.is_none() {
                if b == b'\r' || b == b'\n' {
                    let _ = serial.write(b"Please enter ssid:\r\n");
                    ssid_len = Some(0);
                }
                continue;
            }

            // --- Stage 1: entering SSID (echo chars) ---
            if pass_len.is_none() {
                let mut n = ssid_len.unwrap();
                if b == b'\r' || b == b'\n' {
                    let _ = serial.write(b"\r\nPlease enter password:\r\n");
                    pass_len = Some(0);
                } else if (b == 0x08 || b == 0x7F) && n > 0 {
                    n -= 1;
                    ssid_len = Some(n);
                    let _ = serial.write(b"\x08 \x08");
                } else if n < ssid.len() {
                    ssid[n] = b;
                    n += 1;
                    ssid_len = Some(n);
                    let _ = serial.write(&[b]); // echo typed char
                }
                continue;
            }

            // --- Stage 2: entering password (echo '*') ---
            let mut m = pass_len.unwrap();
            if b == b'\r' || b == b'\n' {
                let _ = serial.write(b"\r\n");
                break; // finished: ssid[..ssid_len], pass[..pass_len]
            } else if (b == 0x08 || b == 0x7F) && m > 0 {
                m -= 1;
                pass_len = Some(m);
                let _ = serial.write(b"\x08 \x08");
            } else if m < pass.len() {
                pass[m] = b;
                m += 1;
                pass_len = Some(m);
                let _ = serial.write(b"*");
            }
        }
    }

    // You handle the rest from here.
    let s_len = ssid_len.unwrap_or(0).min(ssid.len());
    let p_len = pass_len.unwrap_or(0).min(pass.len());
    let ssid_slice = &ssid[..s_len];
    let pass_slice = &pass[..p_len];

    println!("SSID: {:?}", ssid_slice);
    println!("PASS: {:?}", pass_slice);

    storages::save_to_page(WIFI_ADDRESS, (String::from(core::str::from_utf8(ssid_slice).unwrap()), String::from(core::str::from_utf8(pass_slice).unwrap())));

    serial.write(b"Done.\r\n").unwrap();

    usb_dev.poll(&mut [&mut serial]);

    loop {

        Timer::after(Duration::from_secs(1000)).await;

    }
}

#[embassy_executor::task]
async fn heart_beat(mut beat_output: esp_hal::gpio::Output<'static>) {

    loop {
        beat_output.set_low();
        Timer::after(Duration::from_millis(10)).await;
        beat_output.set_high();
        Timer::after(Duration::from_millis(300)).await;
        beat_output.set_low();
        Timer::after(Duration::from_millis(10)).await;
        beat_output.set_high();
        Timer::after(Duration::from_millis(800)).await;

    }

}


#[esp_hal_embassy::main]
async fn main(spawner: Spawner) {
    // generator version: 0.5.0
    esp_alloc::heap_allocator!(size: 64 * 1024);

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    /* Pin definitions */

    let red_pin = peripherals.GPIO17;
    let green_pin = peripherals.GPIO16;
    let blue_pin = peripherals.GPIO15;

    let mode_button_pin = peripherals.GPIO3;

    let sda_pin = peripherals.GPIO13;
    let scl_pin = peripherals.GPIO14;

    let coil_a1_pin = peripherals.GPIO45; //Red
    let coil_a2_pin = peripherals.GPIO42; //Yellow
    let coil_b1_pin = peripherals.GPIO40; //White
    let coil_b2_pin = peripherals.GPIO41; //Black

    let motor_enable_pin = peripherals.GPIO39;

    let exception_led_pin = peripherals.GPIO34;
    let panic_led_pin = peripherals.GPIO33;
    let beat_led_pin = peripherals.GPIO21;

    let tx_pin = peripherals.GPIO36;
    let rx_pin = peripherals.GPIO35;

    let dp_pin = peripherals.GPIO20;
    let dm_pin = peripherals.GPIO19;

    let _uart = Uart::new(peripherals.UART0, esp_hal::uart::Config::default()).unwrap()
        .with_rx(rx_pin)
        .with_tx(tx_pin);

    print!("{}", crate::backtrace::RESET);

    /* Setup Diagnostic LED pins */

    let _exception_output = Output::new(exception_led_pin, Level::High, esp_hal::gpio::OutputConfig::default());
    let _panic_output = Output::new(panic_led_pin, Level::High, esp_hal::gpio::OutputConfig::default());
    let beat_output = Output::new(beat_led_pin, Level::High, esp_hal::gpio::OutputConfig::default());

    spawner.spawn(heart_beat(beat_output)).unwrap();

    /* Setup RGB LED pins */

    print!("LEDs...");

    let red = Output::new(red_pin, Level::High, OutputConfig::default());
    let green = Output::new(green_pin, Level::High, OutputConfig::default());
    let blue = Output::new(blue_pin, Level::High, OutputConfig::default());

    let mut myledc = Ledc::new(peripherals.LEDC);
    myledc.set_global_slow_clock(LSGlobalClkSource::APBClk);

    static LSTIMER1: StaticCell<esp_hal::ledc::timer::Timer<'_, esp_hal::ledc::LowSpeed>> = StaticCell::new();
    let lstimer1 = LSTIMER1.init(
        {
            let mut t = myledc.timer::<esp_hal::ledc::LowSpeed>(esp_hal::ledc::timer::Number::Timer1);
            t.configure(esp_hal::ledc::timer::config::Config {
                duty: esp_hal::ledc::timer::config::Duty::Duty12Bit,
                clock_source: esp_hal::ledc::timer::LSClockSource::APBClk,
                frequency: esp_hal::time::Rate::from_khz(4),
            }).unwrap(); //Safe
            t
        }
    );

    let red_channel = RED_CH.init(myledc.channel(esp_hal::ledc::channel::Number::Channel0, red));
    red_channel.configure(esp_hal::ledc::channel::config::Config {
        timer: lstimer1,
        duty_pct: 10,
        pin_config: esp_hal::ledc::channel::config::PinConfig::PushPull,
    }).unwrap(); //Safe

    let green_channel = GREEN_CH.init(myledc.channel(esp_hal::ledc::channel::Number::Channel1, green));
    green_channel.configure(esp_hal::ledc::channel::config::Config {
        timer: lstimer1,
        duty_pct: 10,
        pin_config: esp_hal::ledc::channel::config::PinConfig::PushPull,
    }).unwrap(); //Safe

    let blue_channel = BLUE_CH.init(myledc.channel(esp_hal::ledc::channel::Number::Channel2, blue));
    blue_channel.configure(esp_hal::ledc::channel::config::Config {
        timer: lstimer1,
        duty_pct: 10,
        pin_config: esp_hal::ledc::channel::config::PinConfig::PushPull,
    }).unwrap(); //Safe

    unsafe {
        RED_REF = Some(red_channel);
        GREEN_REF = Some(green_channel);
        BLUE_REF = Some(blue_channel);

        RED_REF.as_ref().unwrap().set_duty_hw(DUTY_MAX); //Safe
        GREEN_REF.as_ref().unwrap().set_duty_hw(0); //Safe
        GREEN_REF.as_ref().unwrap().set_duty_hw(0); //Safe
    }

    static LED_CHNL: StaticCell<Channel<NoopRawMutex, crate::rgb::State, 1>> = StaticCell::new();
    let led_channel: & 'static mut Channel<NoopRawMutex, crate::rgb::State, 1>  = LED_CHNL.init(Channel::new());

    static BRIGHT_CHNL: StaticCell<Channel<NoopRawMutex, u8, 1>> = StaticCell::new();
    let brightness_channel: & 'static mut Channel<NoopRawMutex, u8, 1>  = BRIGHT_CHNL.init(Channel::new());


    spawner.spawn(led_task(led_channel, brightness_channel, RGBState::OneFlash(Color { r: 0, g: 255, b:255 }, 100, 100))).unwrap(); //Unrecoverable


    let timer0 = SystemTimer::new(peripherals.SYSTIMER);
    esp_hal_embassy::init(timer0.alarm0);

    println!("Done.");

    /* Configure CPU tsense */

    print!("CPU tsense...");

    //let cpu_tsense = esp_hal::tsens::TemperatureSensor::new(peripherals.TSENS, esp_hal::tsens::Config::default());

    println!("Done.");

    /* Set up button */

    print!("Button...");

    let button_pin = esp_hal::gpio::Input::new(mode_button_pin, InputConfig::default().with_pull(Pull::Up));

    let mut mode_button = crate::mode_button::Mode::new(button_pin);

    println!("Done.");

    /* Set ip dht22 */

    let sensor_i2c = esp_hal::i2c::master::I2c::new(peripherals.I2C0, esp_hal::i2c::master::Config::default()).unwrap()
        .with_scl(scl_pin)
        .with_sda(sda_pin);

    print!("Temperature sensor...");

    let mut aht = aht20_driver::AHT20::new(sensor_i2c, aht20_driver::SENSOR_ADDRESS);

    let mut thermo = Thermometer::new(aht.init(& mut Delay).unwrap());

    let _ = thermo.get_temperature();

    println!("Done.");

    /* Initialise FSM */

    print!("Start FSM...");

    let mut fsm = FSM::new();

    println!("Done.");

    /* Initialise motor */

    print!("Init motor...");

    let mut motor = Motor::new(coil_a1_pin, coil_a2_pin, coil_b1_pin, coil_b2_pin, motor_enable_pin);

    println!("Done.");

    /* Check for calibration */

    print!("Calibration Check...");

    if storages::is_locked() {
        fsm.force_change(fsm::State::Calibrate(CalibrateType::MotorLocked));
    }

    if motor.max_position() == 0 {
        fsm.force_change(fsm::State::Calibrate(CalibrateType::MissingMaxPos));
    }

    if motor.max_position() > ABSOLUTE_MAX_POSITION {
        fsm.force_change(fsm::State::Calibrate(CalibrateType::AbsoluteMaxExceeded));
    }

    println!("Done.");

    /* Set up the wifi and web server */

    print!("Wifi...");

    static RESOURCES: StaticCell<StackResources<4>> = StaticCell::new();
    let resources: & 'static mut StackResources<4>  = RESOURCES.init(StackResources::new());


    static TIME_MUTEX: StaticCell<Mutex<NoopRawMutex, RTC>> = StaticCell::new();
    let time_mutex: & 'static mut Mutex<NoopRawMutex, RTC>  = TIME_MUTEX.init(Mutex::new(RTC::new()));


    led_channel.send(RGBState::OneFlash(Color { r: 0, g: 255, b: 255 }, 50, 50)).await;

    let (ssid, password) = match storages::load_from_page(storages::WIFI_ADDRESS) {
        Some((ssid, password)) => (ssid, password),
        None => { (String::new(), String::new()) }
    };

    let (mut is_static, mut static_ip, mut gateway, mut dns1, mut dns2, mut dns3) = match storages::load_from_page(storages::WIFI_IP) {
        Some((is_static, static_ip, gateway, dns1, dns2, dns3)) => (is_static, static_ip, gateway, dns1, dns2, dns3),
        None => { (false, embassy_net::Ipv4Address::from_bits(0), embassy_net::Ipv4Address::from_bits(0), embassy_net::Ipv4Address::from_bits(0), embassy_net::Ipv4Address::from_bits(0), embassy_net::Ipv4Address::from_bits(0)) }
    };

    if (ssid.is_empty() && password.is_empty()) || mode_button.is_pressed() {
        //Spawn usb_cdc thread

        let usb = Usb::new(peripherals.USB0, dp_pin, dm_pin);
        let usb_bus = UsbBus::new(usb, unsafe { &mut *core::ptr::addr_of_mut!(EP_MEMORY) });


        static USB_BUS_ALLOC: StaticCell<UsbBusAllocator<UsbBus<Usb<'static>>>> = StaticCell::new();
        let usb_bus_alloc: & 'static mut UsbBusAllocator<UsbBus<Usb<'static>>>  = USB_BUS_ALLOC.init(usb_bus);

        spawner.spawn(usb_cdc(usb_bus_alloc)).unwrap();
    }

    let rng = Rng::new(peripherals.RNG);

    let timer1 = TimerGroup::new(peripherals.TIMG0);
    static WIFI_INIT: StaticCell<esp_wifi::EspWifiController<'_>> = StaticCell::new();
    let wifi_init = WIFI_INIT.init(esp_wifi::init(timer1.timer0, rng).unwrap());


    let (mut wifi_controller, interfaces) = esp_wifi::wifi::new(wifi_init, peripherals.WIFI)
        .unwrap();

    static REQ_CHNL: StaticCell<Channel<NoopRawMutex, RequestMain, 1>> = StaticCell::new();
    let request_channel: & 'static mut Channel<NoopRawMutex, RequestMain, 1>  = REQ_CHNL.init(Channel::new());


    static RES_CHNL: StaticCell<Channel<NoopRawMutex, ResponseMain, 1>> = StaticCell::new();
    let response_channel: & 'static mut Channel<NoopRawMutex, ResponseMain, 1>  = RES_CHNL.init(Channel::new());

    let mac_address = interfaces.sta.mac_address();

    {



        wifi_controller
            .set_configuration(&esp_wifi::wifi::Configuration::Client(esp_wifi::wifi::ClientConfiguration {
                ssid: ssid,
                password: password,
                ..Default::default()
            }))
            .unwrap();

        if let Ok(_) = wifi_controller.start() {

            use core::str::FromStr;

            let conf = if is_static {

                let mut dnss = heapless::Vec::new();

                if dns1.to_bits() != 0 {
                    dnss.push(dns1.clone()).unwrap();
                }

                if dns2.to_bits() != 0 {
                    dnss.push(dns2.clone()).unwrap();
                }

                if dns3.to_bits() != 0 {
                    dnss.push(dns3.clone()).unwrap();
                }

                embassy_net::Config::ipv4_static(embassy_net::StaticConfigV4 {
                    address: embassy_net::Ipv4Cidr::new(static_ip.clone(), 24),
                    gateway: if gateway.to_bits() == 0 { None } else { Some(gateway.clone()) },
                    dns_servers: dnss,
                })
            } else {
                embassy_net::Config::dhcpv4(Default::default())
            };

            if let Ok(_) = wifi_controller.connect() {
                let (stack, runner) = embassy_net::new(
                    interfaces.sta,
                    conf,
                    resources,
                    rng.clone().random() as u64,
                );

                println!("Done.");

                print!("Web server...");

                // Spawn the network runner (must be running for sockets to work)
                spawner.spawn(net_task(runner)).unwrap(); //Unrecoverable


                if let First(_) = embassy_futures::select::select(
                    stack.wait_config_up(),
                    Timer::after(Duration::from_secs(15))
                ).await {


                    let ip = stack.config_v4().unwrap().address; //Safe

                    println!("Done.");

                    println!("Network up. IPv4: {}", ip);


                    led_channel.send(RGBState::OneFlash(Color { r: 255, g: 0, b: 50 }, 50, 50)).await;

                    /* Get internet time */

                    print!("Get internet time...");

                    spawner.spawn(world_time_task(stack, time_mutex)).unwrap(); //Unrecoverable

                    led_channel.send(RGBState::OneFlash(Color { r: 255, g: 0, b: 255 }, 200, 200)).await;

                    //Block the main thread until the internet time is resolved

                    let time_api_start = Instant::now();

                    loop {
                        {
                            let guard = time_mutex.lock().await;

                            if guard.date_time().is_some() || time_api_start.elapsed().as_secs() > 3 {
                                break;
                            }
                        }

                        Timer::after(Duration::from_millis(100)).await;
                    }

                    println!("Done.");


                    print!("Start Web server...");
                    spawner.spawn(webserver_task(stack, request_channel, response_channel)).unwrap(); //Unrecoverable

                    println!("Done.");

                }


            }
        }

    }

    if !wifi_controller.is_connected().unwrap_or_else(|_| false) {
        //If wifi is not working, warning
        fsm.force_change(State::Warning(WarningType::WiFiError))
    } else {
        //If the wifi is working but there is no time, warning
        let guard = time_mutex.lock().await;

        if guard.date_time().is_none() {

            fsm.force_change(State::Warning(WarningType::WorldTimeError))
        }
    }




    led_channel.send(RGBState::Solid(Color { r: 0, g: 255, b: 0 })).await;

    Timer::after(Duration::from_millis(500)).await;

    let start_time = Instant::now();

    println!("Main loop started");

    loop {



        match request_channel.try_receive() {
            Ok(request) => {
                match request {
                    RequestMain::ShortBoost => {

                        fsm.request_change(fsm::State::Boost(Instant::now(), esp_hal::time::Duration::from_secs(motor.short_boost() as u64)));
                    }
                    RequestMain::LongBoost => {

                        fsm.request_change(fsm::State::Boost(Instant::now(), esp_hal::time::Duration::from_secs(motor.long_boost() as u64)));
                    }
                    RequestMain::Cancel => {

                        fsm.request_change(fsm::State::Cancel);
                    }
                    RequestMain::SetShortDuration(duration) => {
                        motor.set_short_boost(duration);
                    }
                    RequestMain::SetLongDuration(duration) => {
                        motor.set_long_boost(duration);
                    }
                    RequestMain::Rainbow => {
                        fsm.request_change(fsm::State::Rainbow);
                    }
                    RequestMain::SyncTime(epoch) => {

                        let mut guard = time_mutex.lock().await;

                        guard.update_epoch(epoch);
                    }
                    RequestMain::CalibrateInfo => {
                        response_channel.send(ResponseMain::Motor {
                            max_position: motor.max_position(),
                            is_locked: storages::is_locked(),
                            position: storages::get_position(),
                        }).await;
                    }
                    RequestMain::ModeSafe => {
                        fsm.request_change(fsm::State::SafeMode);
                    }
                    RequestMain::ModeCalibrate => {
                        fsm.request_change(fsm::State::Calibrate(CalibrateType::UserRequested));
                    }
                    RequestMain::SettingsPage => {
                        let settings = ResponseMain::Settings {
                            reset_reason: esp_hal::rom::rtc_get_reset_reason(0),
                            //core_temperature: cpu_tsense.as_ref().ok().map(|x| x.get_temperature().to_celsius()),
                            current_time: {

                                let guard = time_mutex.lock().await;

                                guard.date_time()
                            },
                            brightness: fsm.get_brightness(),

                        };

                        response_channel.send(settings).await;
                    }
                    RequestMain::GetStatus => {



                        let status = ResponseMain::Status {
                            temperature: thermo.get_temperature().ok(),
                            current_state: fsm.state().clone(),
                            max_pos: motor.max_position(),
                            position: storages::get_position(),
                            current_time: {

                                let guard = time_mutex.lock().await;

                                guard.date_time()
                            },
                            up_time: start_time,
                            thermostat: thermo.thermostat(),
                            variant: fsm.variant(),
                        };

                        response_channel.send(status).await;

                    }
                    RequestMain::Reset => {
                        esp_hal::rom::software_reset();
                    }
                    RequestMain::Panic => {
                        panic!("User requested panic via web server")
                    }
                    RequestMain::Exception => {
                        unsafe {
                            core::arch::asm!(".word 0", options(nomem, nostack, noreturn));
                        }
                    }
                    RequestMain::SetMax(pos) => {
                        motor.set_max_position(pos);
                    }
                    RequestMain::Push(pos) => {
                        motor.calibrate_push(pos);
                    }
                    RequestMain::Pull(pos) => {
                        motor.calibrate_pull(pos);
                    }
                    RequestMain::UnlockZero => {
                        storages::unlock_and_set_pos(0);
                    }
                    RequestMain::Lock => {
                        storages::lock();
                    }
                    RequestMain::SetSummer => {
                        fsm.set_variant(Variant::Summer);
                    }
                    RequestMain::SetWinter => {
                        fsm.set_variant(Variant::Winter);
                    }
                    RequestMain::ThermostatInfo => {
                        let thermo = ResponseMain::Thermostat {
                            thermostat: thermo.thermostat(),
                            short_duration: motor.short_boost(),
                            long_duration: motor.long_boost(),
                        };

                        response_channel.send(thermo).await;
                    }
                    RequestMain::SetThermostat(temperature) => {
                        thermo.set_thermostat(temperature);
                    }
                    RequestMain::NetworkInfo => {
                        let network = ResponseMain::Network {
                            mac_addr: mac_address.clone(),
                            is_static,
                            ip_addr: static_ip,
                            gateway,
                            dns1,
                            dns2,
                            dns3,
                        };

                        response_channel.send(network).await;
                    }
                    RequestMain::GetState => {
                        let state = ResponseMain::State {
                            state: fsm.state().clone(),
                        };

                        response_channel.send(state).await;
                    }
                    RequestMain::Descale => {
                        fsm.request_change(fsm::State::Descale);
                    }
                    RequestMain::GetSummerSchedule => {

                        response_channel.send(ResponseMain::SummerSchedule { sched: fsm.summer().to_vec() }).await;

                    }
                    RequestMain::GetWinterSchedule => {

                        response_channel.send(ResponseMain::WinterSchedule { sched: fsm.winter().to_vec() }).await;

                    }
                    RequestMain::GetBrightnessSchedule => {

                        response_channel.send(ResponseMain::BrightnessSchedule { sched: fsm.brightness().to_vec() }).await;

                    }
                    RequestMain::SetSummerSchedule(v) => {
                        fsm.set_summer_schedule(v);
                    }
                    RequestMain::SetWinterSchedule(v) => {
                        fsm.set_winter_schedule(v);
                    }
                    RequestMain::SetBrightnessSchedule(v) => {
                        fsm.set_brightness_schedule(v);
                    }
                    RequestMain::SetStaticIp(_is_static, _static_ip, _gateway, _dns1, _dns2, _dns3 ) => {

                        is_static = _is_static;
                        static_ip = _static_ip;
                        gateway = _gateway;
                        dns1 = _dns1;
                        dns2 = _dns2;
                        dns3 = _dns3;

                        storages::save_to_page(storages::WIFI_IP, (is_static, static_ip, gateway, dns1, dns2, dns3));



                    }
                }
            }
            Err(_) => {

            }
        }

        mode_button.update();

        if let Some(action) = mode_button.released_action() {

            match action {
                mode_button::Action::ShortBoost => {
                    fsm.request_change(fsm::State::Boost(Instant::now(), esp_hal::time::Duration::from_secs(motor.short_boost() as u64)));
                }
                mode_button::Action::LongBoost => {
                    fsm.request_change(fsm::State::Boost(Instant::now(), esp_hal::time::Duration::from_secs(motor.long_boost() as u64)));
                }
                mode_button::Action::Cancel => {
                    fsm.request_change(fsm::State::Cancel);
                }
                mode_button::Action::Calibrate => {
                    fsm.request_change(fsm::State::Calibrate(CalibrateType::UserRequested));
                }
                mode_button::Action::Safe => {
                    fsm.request_change(fsm::State::SafeMode);
                }
                mode_button::Action::Retract => {
                    motor.calibrate_pull(7);
                }
                mode_button::Action::ZeroUnlock => {
                    storages::unlock_and_set_pos(0);
                }
            }
        }

        if mode_button.mode_just_changed() {

            if let Some(action) = mode_button.current_action() {
                led_channel.send(RGBState::Solid(action.color())).await;
            } else {
                led_channel.send(RGBState::Off).await;
            }
        }

        if !mode_button.is_pressed() {
            let state_just_changed = fsm.just_changed();

            let current_time = {
                let guard = time_mutex.lock().await;

                guard.date_time()
            };

            match fsm.state() {
                crate::fsm::State::Start => {
                    fsm.change_from_schedule(current_time);

                    fsm.allow_request();
                }
                crate::fsm::State::Rainbow => {
                    if state_just_changed || mode_button.just_released() {

                        led_channel.send(RGBState::Rainbow(3000)).await;

                        Timer::after(Duration::from_millis(100)).await; //Give the led thread time

                    }

                    if state_just_changed {

                        mode_button.clear_actions();
                        mode_button.push_action(mode_button::Action::Cancel);
                    }

                    fsm.allow_request();
                }
                crate::fsm::State::Regulate => {

                    if state_just_changed || mode_button.just_released() {
                        led_channel.send(RGBState::Fade(Color { r: 255, g: 30, b: 0 }, 3000)).await;

                        Timer::after(Duration::from_millis(100)).await;

                    }

                    if state_just_changed {

                        mode_button.clear_actions();
                        mode_button.push_action(mode_button::Action::ShortBoost);
                        mode_button.push_action(mode_button::Action::LongBoost);
                        mode_button.push_action(mode_button::Action::Safe);
                        mode_button.push_action(mode_button::Action::Calibrate);

                    }

                    let thermostat = thermo.thermostat();

                    match thermo.get_temperature() {
                        Ok(actual) => {
                            fsm.change_from_schedule(current_time);

                            fsm.allow_request();

                            if actual < (thermostat - 0.25) {
                                match motor.open_valve() {
                                    Err(motor::MotorError::MotorLocked) => {
                                        fsm.force_change(fsm::State::Calibrate(CalibrateType::MotorLocked));
                                    },
                                    Err(motor::MotorError::MotorMaxNotSet) => {
                                        fsm.force_change(fsm::State::Calibrate(CalibrateType::MissingMaxPos));
                                    }
                                    Err(motor::MotorError::AbsoluteMaxExceeded) => {
                                        fsm.force_change(fsm::State::Calibrate(CalibrateType::AbsoluteMaxExceeded));
                                    }
                                    Ok(()) => {}
                                }

                            }

                            if actual > (thermostat + 0.25) {
                                match motor.close_valve() {
                                    Err(motor::MotorError::MotorLocked) => {
                                        fsm.force_change(fsm::State::Calibrate(CalibrateType::MotorLocked));
                                    },
                                    Err(motor::MotorError::MotorMaxNotSet) => {
                                        fsm.force_change(fsm::State::Calibrate(CalibrateType::MissingMaxPos));
                                    }
                                    Err(motor::MotorError::AbsoluteMaxExceeded) => {
                                        fsm.force_change(fsm::State::Calibrate(CalibrateType::AbsoluteMaxExceeded));
                                    }
                                    Ok(()) => {}
                                }
                            }


                        }
                        Err(_) => {
                            fsm.force_change(fsm::State::Warning(WarningType::TemperatureSensorError))
                        }
                    }

                }
                crate::fsm::State::Off => {

                    if state_just_changed || mode_button.just_released() {
                        led_channel.send(RGBState::Fade(Color { r: 0, g: 10, b: 255 }, 5000)).await;

                        Timer::after(Duration::from_millis(100)).await; //Give the led thread time

                    }

                    if state_just_changed {
                        match motor.close_valve() {
                            Err(motor::MotorError::MotorLocked) => {
                                fsm.force_change(fsm::State::Calibrate(CalibrateType::MotorLocked));
                            },
                            Err(motor::MotorError::MotorMaxNotSet) => {
                                fsm.force_change(fsm::State::Calibrate(CalibrateType::MissingMaxPos));
                            }
                            Err(motor::MotorError::AbsoluteMaxExceeded) => {
                                fsm.force_change(fsm::State::Calibrate(CalibrateType::AbsoluteMaxExceeded));
                            }
                            Ok(()) => {}
                        }


                        mode_button.clear_actions();
                        mode_button.push_action(mode_button::Action::ShortBoost);
                        mode_button.push_action(mode_button::Action::LongBoost);
                        mode_button.push_action(mode_button::Action::Safe);
                        mode_button.push_action(mode_button::Action::Calibrate);
                    } else {

                        fsm.change_from_schedule(current_time);

                        fsm.allow_request();
                    }

                }
                crate::fsm::State::Boost(time_started, duration) => {

                    if state_just_changed || mode_button.just_released() {
                        led_channel.send(RGBState::Fade(Color { r: 255, g: 10, b: 0 }, 400)).await;

                        Timer::after(Duration::from_millis(100)).await; //Give the led thread time

                    }

                    if time_started.elapsed() > *duration {
                        fsm.request_change(fsm::State::Start);
                    }

                    if state_just_changed {

                        match motor.open_valve() {
                            Err(motor::MotorError::MotorLocked) => {
                                fsm.force_change(fsm::State::Calibrate(CalibrateType::MotorLocked));
                            },
                            Err(motor::MotorError::MotorMaxNotSet) => {
                                fsm.force_change(fsm::State::Calibrate(CalibrateType::MissingMaxPos));
                            }
                            Err(motor::MotorError::AbsoluteMaxExceeded) => {
                                fsm.force_change(fsm::State::Calibrate(CalibrateType::AbsoluteMaxExceeded));
                            }
                            Ok(()) => {}
                        }



                        mode_button.clear_actions();
                        mode_button.push_action(mode_button::Action::Cancel);
                        mode_button.push_action(mode_button::Action::Safe);
                        mode_button.push_action(mode_button::Action::Calibrate);
                    }

                    fsm.allow_request();
                }
                crate::fsm::State::Calibrate(_) => {

                    if state_just_changed || mode_button.just_released() {
                        led_channel.send(RGBState::Fade(Color { r: 255, g: 150, b: 0 }, 400)).await;

                        Timer::after(Duration::from_millis(100)).await; //Give the led thread time

                    }

                    if state_just_changed {

                        mode_button.clear_actions();
                        mode_button.push_action(mode_button::Action::Retract);
                        mode_button.push_action(mode_button::Action::ZeroUnlock);
                        mode_button.push_action(mode_button::Action::Cancel);
                    }

                    //FSM may only leave calibrate mode for warnings
                    fsm.handle_request(|state| state.is_warning() || (state.is_cancel() && motor.max_position() != 0 && !storages::is_locked() && motor.max_position() < ABSOLUTE_MAX_POSITION));
                }
                crate::fsm::State::SafeMode => {

                    if state_just_changed || mode_button.just_released() {
                        led_channel.send(RGBState::Fade(Color { r: 255, g: 255, b: 255 }, 3000)).await;

                        Timer::after(Duration::from_millis(100)).await; //Give the led thread time

                    }

                    if state_just_changed {

                        match motor.open_valve() {
                            Err(motor::MotorError::MotorLocked) => {
                                fsm.force_change(fsm::State::Calibrate(CalibrateType::MotorLocked));
                            },
                            Err(motor::MotorError::MotorMaxNotSet) => {
                                fsm.force_change(fsm::State::Calibrate(CalibrateType::MissingMaxPos));
                            }
                            Err(motor::MotorError::AbsoluteMaxExceeded) => {
                                fsm.force_change(fsm::State::Calibrate(CalibrateType::AbsoluteMaxExceeded));
                            }
                            Ok(()) => {}
                        }


                        mode_button.clear_actions();

                        storages::lock();
                    }


                    //FSM may only leave safe mode for warnings or calibration
                    fsm.handle_request(|state| state.is_warning() || state.is_calibrate());
                }
                crate::fsm::State::Descale => {

                    if state_just_changed || mode_button.just_released() {

                        led_channel.send(RGBState::CrossFade { from: Color { r: 255, g: 255, b: 0 }, to: Color { r: 0, g: 255, b: 0 }, duration: 5000 }).await;

                        Timer::after(Duration::from_millis(100)).await; //Give the led thread time

                    }

                    if state_just_changed {
                        //Whatever position the valve is in, open and close it

                        match motor.close_valve() {
                            Err(motor::MotorError::MotorLocked) => {
                                fsm.force_change(fsm::State::Calibrate(CalibrateType::MotorLocked));
                            },
                            Err(motor::MotorError::MotorMaxNotSet) => {
                                fsm.force_change(fsm::State::Calibrate(CalibrateType::MissingMaxPos));
                            }
                            Err(motor::MotorError::AbsoluteMaxExceeded) => {
                                fsm.force_change(fsm::State::Calibrate(CalibrateType::AbsoluteMaxExceeded));
                            }
                            Ok(()) => {

                                motor.open_valve();
                                motor.close_valve();
                            }
                        }






                        mode_button.clear_actions();
                        mode_button.push_action(mode_button::Action::Cancel);
                        mode_button.push_action(mode_button::Action::ShortBoost);
                        mode_button.push_action(mode_button::Action::LongBoost);
                        mode_button.push_action(mode_button::Action::Safe);
                        mode_button.push_action(mode_button::Action::Calibrate);
                    } else {

                        fsm.change_from_schedule(current_time);

                        fsm.allow_request();
                    }

                }
                crate::fsm::State::Warning(code) => {

                    if state_just_changed || mode_button.just_released() {

                        led_channel.send(RGBState::FadeFlash {
                            color: Color { r: 255, g: 100, b: 0 },
                            fade_duration: 2000,
                            pause: 300,
                            flash_count: code.warning_code(),
                            on_duration: 30,
                            off_duration: 270,
                            final_pause: 1000,
                        }).await;


                        Timer::after(Duration::from_millis(100)).await; //Give the led thread time
                    }

                    if state_just_changed {

                        mode_button.clear_actions();
                        mode_button.push_action(mode_button::Action::Cancel);

                    }


                    //The only way to move past a warning is with the special Ignore state, calibrate or safe mode
                    fsm.handle_request(|state| state.is_cancel() || state.is_calibrate() || state.is_safe_mode());
                }
                crate::fsm::State::Cancel => {
                    //Ignore is a special state that is the only way to exit warnings, leave boost early
                    // and manually get out of descale.


                    fsm.force_change(fsm::State::Start);


                }
            }

            if let Some(brightness) = fsm.get_brightness_changed(current_time) {
                println!("Brightness: {}", brightness);
                brightness_channel.send(brightness).await;
            }
        }


        Timer::after(Duration::from_millis(1)).await;
    }


}

// some smoltcp boilerplate
fn timestamp() -> smoltcp::time::Instant {
    smoltcp::time::Instant::from_micros(
        esp_hal::time::Instant::now()
            .duration_since_epoch()
            .as_micros() as i64,
    )
}

pub fn create_interface(device: &mut esp_wifi::wifi::WifiDevice<'_>) -> smoltcp::iface::Interface {
    smoltcp::iface::Interface::new(
        smoltcp::iface::Config::new(smoltcp::wire::HardwareAddress::Ethernet(
            smoltcp::wire::EthernetAddress::from_bytes(&device.mac_address()),
        )),
        device,
        timestamp(),
    )
}