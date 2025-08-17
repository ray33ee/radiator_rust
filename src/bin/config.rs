#![no_std]

use heapless::{String, Vec};
use core::str::FromStr;

type Str = String<64>;

#[derive(Clone, Debug)]
pub struct WifiConfig {
    pub ssid: Str,
    pub password: Str,
    pub static_ip: Str,
    pub dns: Str,
    pub gateway: Str,
    pub subnet: Str,
}

#[derive(Clone, Debug)]
pub struct MotorConfig {
    pub position: i32,
    pub max: i32,
}

#[derive(Clone, Debug)]
pub struct BoostConfig {
    pub short: u32,
    pub long: u32,
}

#[derive(Clone, Debug)]
pub struct Thermostat {
    pub temperature: f32,
}

#[derive(Clone, Debug)]
pub struct TimeConfig {
    pub ntp_servers: Vec<Str, 3>,
    pub timezone: Str,
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Mode {
    Off,
    Regulate,
    Descale,
    Unknown,
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Day {
    Mon, Tue, Wed, Thu, Fri, Sat, Sun, Invalid,
}

#[derive(Clone, Copy, Debug)]
pub struct Time {
    pub hour: u8,
    pub minute: u8,
}

#[derive(Clone, Debug)]
pub struct ScheduleEntry {
    pub day: Day,
    pub start: Time,
    pub end: Time,
    pub mode: Mode,
}

#[derive(Clone, Debug)]
pub struct BrightnessEntry {
    pub day: Day,
    pub start: Time,
    pub end: Time,
    pub level: u8,
}

#[derive(Clone, Debug)]
pub struct Secrets {
    pub key: [u8; 96],
    pub iv: [u8; 24],
    pub key_len: usize,
    pub iv_len: usize,
}

#[derive(Clone, Debug)]
pub struct Config {
    pub wifi: WifiConfig,
    pub motor: MotorConfig,
    pub boost: BoostConfig,
    pub thermostat: Thermostat,
    pub time: TimeConfig,
    pub secrets: Secrets,
    pub schedule_variant: Str,
    pub summer: Vec<ScheduleEntry, 16>,
    pub winter: Vec<ScheduleEntry, 8>,
    pub brightness: Vec<BrightnessEntry, 32>,
}

pub fn parse_day(s: &str) -> Day {
    match s {
        "MON" => Day::Mon,
        "TUE" => Day::Tue,
        "WED" => Day::Wed,
        "THU" => Day::Thu,
        "FRI" => Day::Fri,
        "SAT" => Day::Sat,
        "SUN" => Day::Sun,
        _ => Day::Invalid,
    }
}

pub fn parse_mode(s: &str) -> Mode {
    match s {
        "OFF" => Mode::Off,
        "REGULATE" => Mode::Regulate,
        "DESCALE" => Mode::Descale,
        _ => Mode::Unknown,
    }
}

pub fn parse_time(s: &str) -> Option<Time> {
    let mut parts = s.split(':');
    let hour = parts.next()?.parse::<u8>().ok()?;
    let minute = parts.next()?.parse::<u8>().ok()?;
    if hour < 24 && minute < 60 {
        Some(Time { hour, minute })
    } else {
        None
    }
}

fn parse_u32(s: &str) -> Option<u32> {
    atoi::atoi::<u32>(s.as_bytes())
}

fn parse_i32(s: &str) -> Option<i32> {
    atoi::atoi::<i32>(s.as_bytes())
}

pub fn parse_f32(s: &str) -> Option<f32> {
    let mut int_part = 0f32;
    let mut frac_part = 0f32;
    let mut divisor = 10f32;
    let mut seen_dot = false;
    let mut negative = false;

    let s = s.trim();
    let mut chars = s.bytes();

    if let Some(b'-') = chars.clone().next() {
        negative = true;
        chars.next();
    }

    for b in chars {
        match b {
            b'0'..=b'9' if !seen_dot => {
                int_part = int_part * 10.0 + (b - b'0') as f32;
            }
            b'0'..=b'9' => {
                frac_part += (b - b'0') as f32 / divisor;
                divisor *= 10.0;
            }
            b'.' if !seen_dot => {
                seen_dot = true;
            }
            _ => return None,
        }
    }

    let result = int_part + frac_part;
    Some(if negative { -result } else { result })
}

pub fn parse_schedule_entry(line: &str) -> Option<ScheduleEntry> {
    let mut parts = line.split(',');

    let day = parse_day(parts.next()?.trim());
    let start = parse_time(parts.next()?.trim())?;
    let end = parse_time(parts.next()?.trim())?;
    let mode = parse_mode(parts.next()?.trim());

    Some(ScheduleEntry { day, start, end, mode })
}

pub fn parse_brightness_entry(line: &str) -> Option<BrightnessEntry> {
    let mut parts = line.split(',');

    let day = parse_day(parts.next()?.trim());
    let start = parse_time(parts.next()?.trim())?;
    let end = parse_time(parts.next()?.trim())?;
    let level = parts.next()?.trim().parse::<u8>().ok()?;

    Some(BrightnessEntry { day, start, end, level })
}

fn decode_base64(input: &str, output: &mut [u8]) -> usize {
    use base64::Engine;
    base64::engine::general_purpose::STANDARD.decode_slice(input.as_bytes(), output).unwrap_or_else(|_| 0)
}

pub fn parse_config(input: &[u8]) -> Option<Config> {
    let mut config = Config {
        wifi: WifiConfig {
            ssid: String::new(),
            password: String::new(),
            static_ip: String::new(),
            dns: String::new(),
            gateway: String::new(),
            subnet: String::new(),
        },
        motor: MotorConfig { position: 0, max: 0 },
        boost: BoostConfig { short: 0, long: 0 },
        thermostat: Thermostat { temperature: 0.0 },
        time: TimeConfig {
            ntp_servers: Vec::new(),
            timezone: Str::new(),
        },
        secrets: Secrets {
            key: [0; 96],
            iv: [0; 24],
            key_len: 0,
            iv_len: 0,
        },
        schedule_variant: String::new(),
        summer: Vec::new(),
        winter: Vec::new(),
        brightness: Vec::new(),
    };

    let mut section: Option<&str> = None;

    for line in input.split(|&c| c == b'\n') {
        let line = match core::str::from_utf8(line) {
            Ok(s) => s.trim(),
            Err(_) => continue,
        };

        if line.is_empty() || line.starts_with('#') {
            continue;
        }

        if line.starts_with('[') && line.ends_with(']') {
            section = Some(&line[1..line.len() - 1]);
            continue;
        }

        match section {
            Some("wifi") => {
                if let Some((k, v)) = line.split_once('=') {
                    match k.trim() {
                        "ssid" => config.wifi.ssid = Str::from_str(v.trim()).unwrap_or_default(),
                        "password" => config.wifi.password = Str::from_str(v.trim()).unwrap_or_default(),
                        "static" => config.wifi.static_ip = Str::from_str(v.trim()).unwrap_or_default(),
                        "dns" => config.wifi.dns = Str::from_str(v.trim()).unwrap_or_default(),
                        "gateway" => config.wifi.gateway = Str::from_str(v.trim()).unwrap_or_default(),
                        "subnet" => config.wifi.subnet = Str::from_str(v.trim()).unwrap_or_default(),
                        _ => {}
                    }
                }
            }
            Some("motor") => {
                if let Some((k, v)) = line.split_once('=') {
                    match k.trim() {
                        "position" => {
                            if let Some(n) = parse_i32(v.trim()) {
                                config.motor.position = n;
                            }
                        }
                        "max" => {
                            if let Some(n) = parse_i32(v.trim()) {
                                config.motor.max = n;
                            }
                        }
                        _ => {}
                    }
                }
            }
            Some("boost") => {
                if let Some((k, v)) = line.split_once('=') {
                    match k.trim() {
                        "short" => {
                            if let Some(n) = parse_u32(v.trim()) {
                                config.boost.short = n;
                            }
                        }
                        "long" => {
                            if let Some(n) = parse_u32(v.trim()) {
                                config.boost.long = n;
                            }
                        }
                        _ => {}
                    }
                }
            }
            Some("schedule") => {
                if let Some((k, v)) = line.split_once('=') {
                    if k.trim() == "variant" {
                        config.schedule_variant = Str::from_str(v.trim()).unwrap_or_default();
                    }
                }
            }
            Some("summer") => {
                if let Some(entry) = parse_schedule_entry(line) {
                    config.summer.push(entry).ok();
                }
            }
            Some("winter") => {
                if let Some(entry) = parse_schedule_entry(line) {
                    config.winter.push(entry).ok();
                }
            }
            Some("brightness") => {
                if let Some(entry) = parse_brightness_entry(line) {
                    config.brightness.push(entry).ok();
                }
            }
            Some("secrets") => {
                if let Some((k, v)) = line.split_once('=') {
                    match k.trim() {
                        "key" => {
                            let mut out = [0u8; 96];
                            let len = decode_base64(v.trim(), &mut out);
                            config.secrets.key[..len].copy_from_slice(&out[..len]);
                            config.secrets.key_len = len;
                        }
                        "iv" => {
                            let mut out = [0u8; 24];
                            let len = decode_base64(v.trim(), &mut out);
                            config.secrets.iv[..len].copy_from_slice(&out[..len]);
                            config.secrets.iv_len = len;
                        }
                        _ => {}
                    }
                }
            }
            Some("thermostat") => {
                if let Some((k, v)) = line.split_once('=') {
                    if k.trim() == "temperature" {
                        if let Some(f) = parse_f32(v.trim()) {
                            config.thermostat.temperature = f;
                        }
                    }
                }
            }
            Some("time") => {
                if let Some((k, v)) = line.split_once('=') {
                    let key = k.trim();
                    let val = v.trim();
                    if key.starts_with("ntp") && config.time.ntp_servers.len() < 3 {
                        config.time.ntp_servers.push(Str::from_str(val).unwrap_or_default()).ok();
                    } else if key == "timezone" {
                        config.time.timezone = Str::from_str(val).unwrap_or_default();
                    }
                }
            }
            Some(section) => {
                panic!("Bad config file - Unrecognised section: {}", section);
            }
            None => {
                panic!("Bad config file - entry without a section");
            }
        }
    }

    Some(config)
}
