#![no_std]
#![no_main]
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
use esp_backtrace as _;
use alloc::{format, vec};
use alloc::string::String;
use alloc::vec::Vec;
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
    commands::{Function, Message},

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
use smoltcp::wire::IpEndpoint;
use core::fmt::Write;
use time::OffsetDateTime;
use crate::fsm::{State, WarningType};
use crate::storages::is_locked;

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

        //let world_time_api_dns = "192.168.1.111";
        //let world_time_api_port = 8080;

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


///
/// Server takes clients, reads the message, sends a response then closes the connection
///
/// The message is sent to the main loop, the function is executed, and the return string is
/// sent back to this thread to send to the client.
///
#[embassy_executor::task]
async fn server_task(
    stack: Stack<'static>,
    msg: & 'static Channel<NoopRawMutex, Message, 1>,
    ret:  & 'static Channel<NoopRawMutex, String, 1>,

) -> ! {
    let mut rx_buf = [0u8; 3024];
    let mut tx_buf = [0u8; 3024];

    loop {
        let mut socket = TcpSocket::new(stack, &mut rx_buf, &mut tx_buf);

        socket.set_timeout(Some(Duration::from_secs(10)));

        if let Err(e) = socket.accept(8080).await {
            println!("Accept failed: {:?}", e);
            continue;
        }

        let mut buf = [0u8; 3024];

        if let Ok(n) = socket.read(& mut buf).await {
            if n != 0 {
                if let Ok(s) = core::str::from_utf8(&buf[..n]) {
                    if let Ok(message) = serde_json::from_str::<Message>(s) {
                        use embedded_io_async::Write;
                        println!("Message: {:?}", message);

                        //Send the message to the main thread
                        assert!(!msg.is_full());

                        msg.send(message).await;

                        let return_string = ret.receive().await;

                        let _ = socket.write_all(return_string.as_bytes()).await;

                        let _ = socket.flush().await;

                        let _ = socket.close();

                        //Purge remaining data and wait for eof
                        let mut drain = [0u8; 64];
                        let timeout = Instant::now();
                        loop {
                            if timeout.elapsed() > esp_hal::time::Duration::from_secs(3) {
                                break
                            }
                            match socket.read(&mut drain).await {
                                Ok(0) => break,      // peer closed -> done
                                Ok(_) => continue,   // ignore any extra bytes
                                Err(_) => break,     // error -> just exit
                            }
                        }


                        println!("client disconnected");
                    }
                }
            }
        }

        Timer::after(Duration::from_millis(5)).await;
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
        core_temperature: Option<f32>,
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
    use embedded_io_async::Write;

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

                                    let temperature = temperature.map(|t| format!("{}", t)).unwrap_or_else(|| format!("Error"));
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

                                if let ResponseMain::Settings { reset_reason, core_temperature, current_time, brightness } = response.receive().await {

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

                                    let cpu_tsense = match core_temperature {
                                        Some(value) => format!("{:.1}°C", value),
                                        None => format!("Unknown"),
                                    };

                                    let current_time = current_time.map(|t| format!("{:04}-{:02}-{:02} {:02}:{:02}:{:02}", t.year(), t.month() as u32, t.day(), t.hour(), t.minute(), t.second())).unwrap_or_else(|| format!("Error"));

                                    let brightness = format!("{}", brightness);

                                    let cards = vec![reset_code.as_str(), reset_reason, cpu_tsense.as_str(), brightness.as_str(), current_time.as_str()];


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

                                    _ => send_404(&mut socket).await,
                                }
                            }
                            ("GET", "/calibrate") => {
                                request.send(RequestMain::CalibrateInfo).await;

                                if let ResponseMain::Motor { max_position, is_locked, position } = response.receive().await {

                                    let max = format!("{}", max_position);
                                    let is_locked = format!("{}", if is_locked {"Locked"} else {"Unlocked"});
                                    let pos = if position == 0xffffffff { format!("Undefined") } else { format!("{}", position) };

                                    let cards = vec![max.as_str(), is_locked.as_str(), pos.as_str()];


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
                                            if let fsm::State::Calibrate = state {
                                                request.send(RequestMain::SetMax(max.parse::<u32>().unwrap())).await;
                                                send_200(& mut socket, None).await;
                                            } else {
                                                send_403(& mut socket).await;
                                            }
                                        }
                                        (Some("push"), Some(push)) => {
                                            if let fsm::State::Calibrate = state {
                                                request.send(RequestMain::Push(push.parse::<u32>().unwrap())).await;
                                                send_200(& mut socket, None).await;
                                            } else {
                                                send_403(& mut socket).await;
                                            }

                                        }
                                        (Some("pull"), Some(pull)) => {
                                            if let fsm::State::Calibrate = state {
                                                request.send(RequestMain::Pull(pull.parse::<u32>().unwrap())).await;
                                                send_200(& mut socket, None).await;
                                            } else {
                                                send_403(& mut socket).await;
                                            }
                                        }
                                        (Some("action"), Some("lock")) => {
                                            if let fsm::State::Calibrate = state {
                                                request.send(RequestMain::Lock).await;
                                                send_200(& mut socket, None).await;
                                            } else {
                                                send_403(& mut socket).await;
                                            }
                                        }
                                        (Some("action"), Some("unlock_zero")) => {
                                            if let fsm::State::Calibrate = state {
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

                                if let ResponseMain::Network { mac_addr } = response.receive().await {

                                    let mac = format!("{:02X}:{:02X}:{:02X}:{:02X}:{:02X}:{:02X}",
                                                      mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);

                                    let cards = vec![mac.as_str()];


                                    let _ = send_template("text/html", NETWORK_HTML, & mut socket, &cards).await;
                                } else {
                                    send_404(& mut socket).await;
                                }

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
            let r = (r as u32 * 256) / 255;
            let g = (g as u32 * 256) / 255;
            let b = (b as u32 * 256) / 255;

            red_channel.set_duty_hw(256 - r);
            green_channel.set_duty_hw(256 - g);
            blue_channel.set_duty_hw(256 - b);
        });

        Timer::after(Duration::from_millis(5)).await;

    }

}


#[esp_hal_embassy::main]
async fn main(spawner: Spawner) {
    // generator version: 0.5.0
    esp_alloc::heap_allocator!(size: 64 * 1024);

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    /* Pin definitions */

    let RED_PIN = peripherals.GPIO10;
    let GREEN_PIN = peripherals.GPIO20;
    let BLUE_PIN = peripherals.GPIO21;

    let MODE_BUTTON_PIN = peripherals.GPIO8;

    let ONE_WIRE_PIN = peripherals.GPIO9;

    let COIL_A1_PIN = peripherals.GPIO0;
    let COIL_A2_PIN = peripherals.GPIO3;
    let COIL_B1_PIN = peripherals.GPIO1;
    let COIL_B2_PIN = peripherals.GPIO2;

    /* Setup LED pins */

    print!("LEDs...");

    let red = Output::new(RED_PIN, Level::High, OutputConfig::default());
    let green = Output::new(GREEN_PIN, Level::High, OutputConfig::default());
    let blue = Output::new(BLUE_PIN, Level::High, OutputConfig::default());

    let mut myledc = Ledc::new(peripherals.LEDC);
    myledc.set_global_slow_clock(LSGlobalClkSource::APBClk);

    static LSTIMER1: StaticCell<esp_hal::ledc::timer::Timer<'_, esp_hal::ledc::LowSpeed>> = StaticCell::new();
    let lstimer1 = LSTIMER1.init(
        {
            let mut t = myledc.timer::<esp_hal::ledc::LowSpeed>(esp_hal::ledc::timer::Number::Timer1);
            t.configure(esp_hal::ledc::timer::config::Config {
                duty: esp_hal::ledc::timer::config::Duty::Duty8Bit,
                clock_source: esp_hal::ledc::timer::LSClockSource::APBClk,
                frequency: esp_hal::time::Rate::from_khz(24),
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

        RED_REF.as_ref().unwrap().set_duty_hw(256); //Safe
        GREEN_REF.as_ref().unwrap().set_duty_hw(0); //Safe
        GREEN_REF.as_ref().unwrap().set_duty_hw(0); //Safe
    }

    static LED_CHNL: StaticCell<Channel<NoopRawMutex, crate::rgb::State, 1>> = StaticCell::new();
    let led_channel: & 'static mut Channel<NoopRawMutex, crate::rgb::State, 1>  = LED_CHNL.init(Channel::new());

    static BRIGHT_CHNL: StaticCell<Channel<NoopRawMutex, u8, 1>> = StaticCell::new();
    let brightness_channel: & 'static mut Channel<NoopRawMutex, u8, 1>  = BRIGHT_CHNL.init(Channel::new());

    spawner.spawn(led_task(led_channel, brightness_channel, RGBState::OneFlash(Color { r: 0, g: 255, b:255 }, 100, 100))).unwrap(); //Unrecoverable

    println!("Done.");

    /* Configure CPU tsense */

    print!("CPU tsense...");

    let cpu_tsense = esp_hal::tsens::TemperatureSensor::new(peripherals.TSENS, esp_hal::tsens::Config::default());

    println!("Done.");

    /* Set up button */

    print!("Button...");

    let button_pin = esp_hal::gpio::Input::new(MODE_BUTTON_PIN, InputConfig::default().with_pull(Pull::Up));

    let mut mode_button = crate::mode_button::Mode::new(button_pin);

    println!("Done.");

    /* Set ip dht22 */

    print!("Temperature sensor...");

    let mut thermo = Thermometer::new(ONE_WIRE_PIN);

    let _ = thermo.get_temperature();

    println!("Done.");

    /* Initialise FSM */

    print!("Start FSM...");

    let mut fsm = FSM::new();

    println!("Done.");

    /* Check for calibration */

    print!("Calibration Check...");

    if storages::is_locked() {

        fsm.force_change(fsm::State::Calibrate);


    }
    println!("Done.");

    /* Initialise motor */

    print!("Init motor...");

    let mut motor = Motor::new(COIL_A1_PIN, COIL_A2_PIN, COIL_B1_PIN, COIL_B2_PIN);

    println!("Done.");

    /* Set up the wifi and web server */

    print!("Wifi...");



    static MSG_CHNL: StaticCell<Channel<NoopRawMutex, Message, 1>> = StaticCell::new();
    let message_channel: & 'static mut Channel<NoopRawMutex, Message, 1>  = MSG_CHNL.init(Channel::new());

    static RET_CHNL: StaticCell<Channel<NoopRawMutex, String, 1>> = StaticCell::new();
    let return_channel: & 'static mut Channel<NoopRawMutex, String, 1>  = RET_CHNL.init(Channel::new());


    static RESOURCES: StaticCell<StackResources<4>> = StaticCell::new();
    let resources: & 'static mut StackResources<4>  = RESOURCES.init(StackResources::new());


    static TIME_MUTEX: StaticCell<Mutex<NoopRawMutex, RTC>> = StaticCell::new();
    let time_mutex: & 'static mut Mutex<NoopRawMutex, RTC>  = TIME_MUTEX.init(Mutex::new(RTC::new()));


    led_channel.send(RGBState::OneFlash(Color { r: 0, g: 255, b: 255 }, 50, 50)).await;

    let (ssid, password) = match storages::load_from_page(storages::WIFI_ADDRESS) {
        Some((ssid, password)) => (ssid, password),
        None => (String::new(), String::new())
    };

    let rng = Rng::new(peripherals.RNG);

    let timer0 = SystemTimer::new(peripherals.SYSTIMER);
    esp_hal_embassy::init(timer0.alarm0);

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


            if let Ok(_) = wifi_controller.connect() {
                let (stack, runner) = embassy_net::new(
                    interfaces.sta,
                    embassy_net::Config::dhcpv4(Default::default()),
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
                            let mut guard = time_mutex.lock().await;

                            if guard.date_time().is_some() || time_api_start.elapsed().as_secs() > 7 {
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

    if !wifi_controller.is_connected().unwrap_or_else(|x| false) {
        //If wifi is not working, warning
        fsm.force_change(State::Warning(WarningType::WiFiError))
    } else {
        //If the wifi is working but there is no time, warning
        let mut guard = time_mutex.lock().await;

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
                        fsm.request_change(fsm::State::Calibrate);
                    }
                    RequestMain::SettingsPage => {
                        let settings = ResponseMain::Settings {
                            reset_reason: esp_hal::rom::rtc_get_reset_reason(0),
                            core_temperature: cpu_tsense.as_ref().ok().map(|x| x.get_temperature().to_celsius()),
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
                            #[cfg(target_arch = "riscv32")]
                            core::arch::asm!("unimp", options(noreturn));

                            // Xtensa (ESP32-S2/S3). `ill` is an illegal-instruction trap.
                            // If your toolchain doesn’t accept `ill`, use `.byte 0` repeatedly.
                            core::arch::asm!("ill", options(noreturn))
                            // Fallback for older xtensa assemblers:
                            // core::arch::asm!(".byte 0, 0, 0", options(noreturn));
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
                }
            }
            Err(_) => {

            }
        }


        match message_channel.try_receive() {
            Ok(message) => {
                println!("Loop message");
                let return_string = match message.function {
                    Function::CalibratePush(revolutions) => {
                        if fsm.state().is_calibrate() {
                            motor.calibrate_push(revolutions);
                            format!("Pushed {} revolutions", revolutions)
                        } else {
                            format!("Device must be in calibrate mode for calibrate push to work")
                        }
                    },
                    Function::CalibratePull(revolutions) => {
                        if fsm.state().is_calibrate() {
                            motor.calibrate_pull(revolutions);
                            format!("Pulled {} revolutions", revolutions)
                        } else {
                            format!("Device must be in calibrate mode for calibrate pull to work")
                        }
                    },
                    Function::UnlockAndZero => {
                        if fsm.state().is_calibrate() {
                            if storages::is_locked() {
                                storages::unlock_and_set_pos(0);
                                format!("Lock unlocked and zeroed")
                            } else {
                                format!("Already unlocked")
                            }
                        } else {
                            format!("Device must be in calibrate mode for unlock to work")
                        }
                    },
                    Function::Lock => {
                        if fsm.state().is_calibrate() {
                            storages::lock();
                            format!("Locked motor")
                        } else {
                            format!("Device must be in calibrate mode for lock to work")
                        }
                    }
                    Function::GetLock => {
                        format!("Lock is {}", if storages::is_locked() { "locked" } else { "unlocked" })
                    },
                    Function::GetPosition => {

                        let position = storages::get_position();

                        if position > crate::motor::ABSOLUTE_MAX_POSITION {
                            format!("Cannot get position - device is locked or position is corrupt")
                        } else {
                            format!("Position is at: {}", position)
                        }

                    },
                    Function::GetMax => {
                        format!("Max position is at: {}", motor.max_position())
                    },
                    Function::SetMax(value) => {
                        if fsm.state().is_calibrate() {
                            motor.set_max_position(value);
                            format!("Set max position to {}", value)
                        } else {
                            format!("Device must be in calibrate mode for set max to work")
                        }
                    },
                    Function::GetThermostat => {
                        format!("Thermostat value is {}°C", thermo.thermostat())
                    },
                    Function::SetThermostat(temperature) => {
                        thermo.set_thermostat(temperature);
                        format!("Thermostat set to {}°C", thermo.thermostat())
                    },
                    Function::ReadTemperature => {
                        match thermo.get_temperature() {
                            Ok(temperature) => {
                                format!("Temperature is {}°C", temperature)
                            }
                            Err(e) => {
                                format!("Temperature is unknown, sensor error - {:?}", e)
                            }
                        }


                    },
                    Function::GetMacAddress => {
                        //format!("{:02X}:{:02X}:{:02X}:{:02X}:{:02X}:{:02X}",
                        //        mac_address[0], mac_address[1], mac_address[2], mac_address[3], mac_address[4], mac_address[5])
                        format!("no")
                    },
                    Function::SoftReset => {
                        esp_hal::rom::software_reset();
                    },
                    Function::SyncTime(epoch) => {
                        let mut guard = time_mutex.lock().await;

                        guard.update_epoch(epoch);

                        format!("Time updated")
                    },
                    Function::GetResetReason => {
                        format!("Reset reason: {}", esp_hal::rom::rtc_get_reset_reason(0))
                    },
                    Function::GetTime => {

                        let guard = time_mutex.lock().await;

                        if let Some(date_time) = guard.date_time() {
                            format!("Time is {:04}-{:02}-{:02} {:02}:{:02}:{:02}",
                                   date_time.year(),
                                   date_time.month() as u32,
                                   date_time.day(),
                                   date_time.hour(),
                                   date_time.minute(),
                                   date_time.second(),
                            )
                        } else {
                            format!("Could not get time - rtc not synced")
                        }
                    },
                    Function::Descale => {
                        fsm.request_change(fsm::State::Descale);
                        format!("Requesting descale - NOTE: this might not work if the device is in safe, calibrate or warning mode")
                    },
                    Function::Rainbow => {
                        fsm.request_change(fsm::State::Rainbow);
                        format!("Requesting Rainbow")
                    }
                    Function::Calibrate => {
                        fsm.request_change(fsm::State::Calibrate);
                        format!("Requesting descale - NOTE: this might not work if the device is in safe, calibrate or warning mode")
                    },
                    Function::SafeMode => {
                        fsm.request_change(fsm::State::SafeMode);
                        format!("Requesting descale - NOTE: this might not work if the device is in safe, calibrate or warning mode")
                    },
                    Function::Cancel => {
                        fsm.request_change(fsm::State::Cancel);
                        format!("Cancelling mode (does nothing unless in warning, descale or boost mode)")
                    },
                    Function::ShortBoost => {
                        fsm.request_change(fsm::State::Boost(Instant::now(), esp_hal::time::Duration::from_secs(motor.short_boost() as u64)));
                        format!("Starting short boost")
                    },
                    Function::LongBoost => {
                        fsm.request_change(fsm::State::Boost(Instant::now(), esp_hal::time::Duration::from_secs(motor.long_boost() as u64)));
                        format!("Starting long boost")
                    },
                    Function::CurrentState => {
                        format!("State is: {:?}", fsm.state())
                    },
                    Function::GetBoostDuration => {
                        if let fsm::State::Boost(instant, duration) = fsm.state() {

                            let seconds_left = (*duration - instant.elapsed()).as_secs();

                            format!("Boost has {} minutes and {} seconds left", seconds_left / 60, seconds_left % 60)
                        } else {
                            format!("Radiator is not in boost mode")
                        }
                    }
                    Function::SetShortDuration(duration) => {
                        motor.set_short_boost(duration);
                        format!("Set short boost duration")
                    }
                    Function::SetLongDuration(duration) => {
                        motor.set_long_boost(duration);
                        format!("Set long boost duration")
                    }
                    Function::SetVariant(variant) => {
                        if let Some(v) = Variant::from_str(variant.as_str()) {
                            fsm.set_variant(v);
                            format!("Variant changed to: {}", variant)
                        } else {
                            format!("Invalid variant (must be summer or winter)")
                        }
                    }
                    Function::GetVariant => {
                        format!("Using {:?} schedule", fsm.variant())
                    }
                    Function::GetSummer => {
                        if fsm.summer().len() == 0 {
                            format!("Summer schedule is empty")
                        } else {
                            let mut str = String::new();
                            for slot in fsm.summer() {
                                write!(str, "\n{}", slot).unwrap(); //Safe
                            }
                            str
                        }
                    }
                    Function::GetWinter => {
                        if fsm.winter().len() == 0 {
                            format!("Winter schedule is empty")
                        } else {
                            let mut str = String::new();
                            for slot in fsm.winter() {
                                write!(str, "\n{}", slot).unwrap(); //Safe
                            }
                            str
                        }
                    }
                    Function::GetBrightness => {
                        if fsm.brightness().len() == 0 {
                            format!("Winter schedule is empty")
                        } else {
                            let mut str = String::new();
                            for slot in fsm.brightness() {
                                write!(str, "\n{}", slot).unwrap(); //Safe
                            }
                            str
                        }
                    }
                    Function::AddSummerSlot(slot) => {

                        if let Some(s) = ScheduleEntry::from_str(slot.as_str()) {

                            fsm.add_summer(s);
                            format!("Added summer slot")
                        } else {
                            format!("Invalid schedule - must be weekday,start,finish,state format (WWW,HH:MM,HH:MM,state)")
                        }

                    }
                    Function::ClearSummer(confirm) => {

                        if confirm == "CLEAR" {
                            fsm.clear_summer();
                            format!("Summer schedule cleared")
                        } else {
                            format!("Please send 'CLEAR' confirmation to erase schedule")
                        }
                    }
                    Function::AddWinterSlot(slot) => {

                        if let Some(s) = ScheduleEntry::from_str(slot.as_str()) {

                            fsm.add_winter(s);
                            format!("Added winter slot")
                        } else {
                            format!("Invalid schedule - must be weekday,start,finish,state format (WWW,HH:MM,HH:MM,state)")
                        }

                    }
                    Function::ClearWinter(confirm) => {

                        if confirm == "CLEAR" {
                            fsm.clear_winter();
                            format!("Winter schedule cleared")
                        } else {
                            format!("Please send 'CLEAR' confirmation to erase schedule")
                        }
                    }
                    Function::AddBrightnessSlot(slot) => {

                        if let Some(s) = BrightnessEntry::from_str(slot.as_str()) {

                            fsm.add_brightness(s);
                            format!("Added brightness slot")
                        } else {
                            format!("Invalid schedule - must be weekday,start,finish,state format (WWW,HH:MM,HH:MM,state)")
                        }

                    }
                    Function::ClearBrightness(confirm) => {

                        if confirm == "CLEAR" {
                            fsm.clear_brightness();
                            format!("Brightness schedule cleared")
                        } else {
                            format!("Please send 'CLEAR' confirmation to erase schedule")
                        }


                    }
                    Function::RemoveSummerSlot(slot) => {
                        if let Some(s) = ScheduleEntry::from_str(slot.as_str()) {

                            if fsm.remove_summer(s).is_some() {
                                format!("Removed summer slot")
                            } else {
                                format!("No such slot in summer schedule")
                            }

                        } else {
                            format!("Invalid schedule - must be weekday,start,finish,state format (WWW,HH:MM,HH:MM,state)")
                        }
                    }
                    Function::RemoveWinterSlot(slot) => {
                        if let Some(s) = ScheduleEntry::from_str(slot.as_str()) {

                            if fsm.remove_winter(s).is_some() {
                                format!("Removed winter slot")
                            } else {
                                format!("No such slot in winter schedule")
                            }

                        } else {
                            format!("Invalid schedule - must be weekday,start,finish,state format (WWW,HH:MM,HH:MM,state)")
                        }
                    }
                    Function::RemoveBrightnessSlot(slot) => {
                        if let Some(s) = BrightnessEntry::from_str(slot.as_str()) {

                            if fsm.remove_brightness(s).is_some() {
                                format!("Removed brightness slot")
                            } else {
                                format!("No such slot in brightness schedule")
                            }

                        } else {
                            format!("Invalid schedule - must be weekday,start,finish,state format (WWW,HH:MM,HH:MM,state)")
                        }
                    }
                    Function::StartPanic(confirm) => {
                        if confirm == "PANIC" {
                            panic!("User requested panic (via WiFi commands)");
                        } else {
                            format!("When requesting a panic the 'PANIC' string must be sent to confirm")
                        }
                    }
                    Function::StartException(confirm) => {
                        if confirm == "EXCEPTION" {
                            unsafe {
                                #[cfg(target_arch = "riscv32")]
                                core::arch::asm!("unimp", options(noreturn));

                                // Xtensa (ESP32-S2/S3). `ill` is an illegal-instruction trap.
                                // If your toolchain doesn’t accept `ill`, use `.byte 0` repeatedly.
                                core::arch::asm!("ill", options(noreturn))
                                // Fallback for older xtensa assemblers:
                                // core::arch::asm!(".byte 0, 0, 0", options(noreturn));
                            }
                        } else {
                            format!("When requesting a panic the 'EXCEPTION' string must be sent to confirm")
                        }
                    }
                    Function::GetLEDBrightness => {
                        format!("LED brightness is {}%", fsm.get_brightness())
                    }
                    Function::GetUpTime => {
                        let up_time = start_time.elapsed().as_secs();
                        let dur = time::Duration::seconds(up_time as i64);

                        let days = dur.whole_seconds() / 86400;
                        let hours = (dur.whole_seconds() % 86_400) / 3_600;
                        let minutes = (dur.whole_seconds() % 3_600) / 60;
                        let seconds = dur.whole_seconds() % 60;

                        if days == 0 {
                            if hours == 0 {
                                if minutes == 0 {
                                    format!("Uptime is {} seconds.", seconds)
                                } else {
                                    format!("Uptime is {} minutes and {} seconds.", minutes, seconds)
                                }
                            } else {
                                format!("Uptime is {} hours, {} minutes and {} seconds.", hours, minutes, seconds)
                            }
                        } else {
                            format!("Uptime is {} days, {} hours, {} minutes and {} seconds.", days, hours, minutes, seconds)
                        }

                    }


                };

                return_channel.send(return_string).await;
            },
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
                    fsm.request_change(fsm::State::Calibrate);
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
                                if !motor.open_valve() {
                                    fsm.force_change(fsm::State::Calibrate);
                                }

                            }

                            if actual > (thermostat + 0.25) {
                                if !motor.close_valve() {
                                    fsm.force_change(fsm::State::Calibrate);
                                }
                            }


                        }
                        Err(e) => {
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

                        if !motor.close_valve() {
                            fsm.force_change(State::Calibrate);
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

                        if !motor.open_valve() {
                            fsm.force_change(fsm::State::Calibrate);
                        }


                        mode_button.clear_actions();
                        mode_button.push_action(mode_button::Action::Cancel);
                        mode_button.push_action(mode_button::Action::Safe);
                        mode_button.push_action(mode_button::Action::Calibrate);
                    }

                    fsm.allow_request();
                }
                crate::fsm::State::Calibrate => {

                    if state_just_changed || mode_button.just_released() {
                        led_channel.send(RGBState::Fade(Color { r: 255, g: 150, b: 0 }, 400)).await;

                        Timer::after(Duration::from_millis(100)).await; //Give the led thread time

                    }

                    if state_just_changed {

                        mode_button.clear_actions();
                        mode_button.push_action(mode_button::Action::Retract);
                        mode_button.push_action(mode_button::Action::ZeroUnlock);
                    }

                    //FSM may only leave calibrate mode for warnings
                    fsm.handle_request(|state| state.is_warning());
                }
                crate::fsm::State::SafeMode => {

                    if state_just_changed || mode_button.just_released() {
                        led_channel.send(RGBState::Fade(Color { r: 255, g: 255, b: 255 }, 3000)).await;

                        Timer::after(Duration::from_millis(100)).await; //Give the led thread time

                    }

                    if state_just_changed {

                        if !motor.open_valve() {
                            fsm.force_change(fsm::State::Calibrate);
                        }


                        mode_button.clear_actions();
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

                        if !motor.close_valve() {

                            fsm.force_change(fsm::State::Calibrate);
                        } else {

                            motor.open_valve();
                            motor.close_valve();
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


                    //The only way to move past a warning is with the special Ignore state
                    fsm.handle_request(|state| state.is_cancel());
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