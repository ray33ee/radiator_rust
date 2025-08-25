
use heapless::{String, Vec};
use core::str::FromStr;
use core::option::Option;
use time::Weekday;
use time::OffsetDateTime;
use crate::fsm;

const KEY_LENGTH: usize = 128;
const IV_LENGTH: usize = 16;

type Str = String<64>;

#[derive(Clone, Debug)]
pub(super) struct WifiConfig {
    pub(super) ssid: Str,
    pub(super) password: Str,
    pub(super) static_ip: Option<Str>,
    pub(super) dns: Option<Str>,
    pub(super) gateway: Option<Str>,
    pub(super) subnet: Option<Str>,
}

#[derive(Clone, Debug)]
pub(super) struct MotorConfig {
    pub(super) max: u32,
}

#[derive(Clone, Debug)]
pub(super) struct BoostConfig {
    pub(super) short: u32,
    pub(super) long: u32,
}

#[derive(Clone, Debug)]
pub(super) struct Thermostat {
    pub(super) temperature: f32,
}

#[derive(Clone, Debug)]
pub(super) struct TimeConfig {
    pub(super) ntp_servers: Vec<Str, 3>,
    pub(super) timezone: Str,
}
/*
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub(super) enum Mode {
    Off,
    Regulate,
    Descale,
    Unknown,
}
*/

#[derive(Clone, Debug)]
pub(super) struct ScheduleEntry {
    pub(super) day: Weekday,
    pub(super) start: u16,
    pub(super) end: u16,
    pub(super) mode: crate::fsm::State,
}

impl ScheduleEntry {

    fn is_in(&self, current_time: OffsetDateTime) -> bool {
        if current_time.weekday() == self.day {
            let current_minutes = current_time.hour() as u16 * 60 + current_time.minute() as u16;
            if current_minutes > self.start && current_minutes < self.end {
                return true;
            }
        }
        false
    }
}

#[derive(Clone, Debug)]
pub(super) struct BrightnessEntry {
    pub(super) day: Weekday,
    pub(super) start: u16,
    pub(super) end: u16,
    pub(super) level: u8,
}

impl BrightnessEntry {

    fn is_in(&self, current_time: OffsetDateTime) -> bool {
        if current_time.weekday() == self.day {
            let current_minutes = current_time.hour() as u16 * 60 + current_time.minute() as u16;
            if current_minutes > self.start && current_minutes < self.end {
                return true;
            }
        }
        false
    }
}

#[derive(Clone, Debug)]
pub(super) struct Secrets {
    pub(super) key: Vec<u8, KEY_LENGTH>,
    pub(super) iv: Vec<u8, IV_LENGTH>,
}

#[derive(Clone, Debug)]
pub(super) enum Variant {
    Summer,
    Winter,
}

#[derive(Clone, Debug)]
pub(super) struct Config {
    pub(super) wifi: WifiConfig,
    pub(super) motor: MotorConfig,
    pub(super) boost: BoostConfig,
    pub(super) thermostat: Thermostat,
    pub(super) time: TimeConfig,
    pub(super) secrets: Secrets,
    pub(super) schedule_variant: Variant,
    pub(super) summer: Vec<ScheduleEntry, 32>,
    pub(super) winter: Vec<ScheduleEntry, 32>,
    pub(super) brightness: Vec<BrightnessEntry, 32>,
}

impl Config {

    fn parse_day(s: &str) -> Weekday {
        match s {
            "MON" => Weekday::Monday,
            "TUE" => Weekday::Tuesday,
            "WED" => Weekday::Wednesday,
            "THU" => Weekday::Thursday,
            "FRI" => Weekday::Friday,
            "SAT" => Weekday::Saturday,
            "SUN" => Weekday::Sunday,
            _ => panic!("Invalid weekday: {}", s),
        }
    }

    fn parse_mode(s: &str) -> crate::fsm::State {
        match s {
            "OFF" => crate::fsm::State::Off,
            "REGULATE" => crate::fsm::State::Regulate,
            "DESCALE" => crate::fsm::State::Descale,
            "RESET" => crate::fsm::State::Reset,
            _ => panic!("Invalid state: {}", s),
        }
    }

    fn parse_time(s: &str) -> Option<u16> {
        let mut parts = s.split(':');
        let hour = parts.next()?.parse::<u8>().ok()?;
        let minute = parts.next()?.parse::<u8>().ok()?;
        if hour <= 24 && minute < 60 {
            Some(hour as u16 * 60 + minute as u16)
        } else {
            None
        }
    }

    fn parse_u32(s: &str) -> Option<u32> {
        atoi::atoi::<u32>(s.as_bytes())
    }


    fn parse_f32(s: &str) -> Option<f32> {
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

    fn parse_schedule_entry(line: &str) -> Option<ScheduleEntry> {
        let mut parts = line.split(',');

        let day = Self::parse_day(parts.next()?.trim());
        let start = Self::parse_time(parts.next()?.trim())?;
        let end = Self::parse_time(parts.next()?.trim())?;
        let mode = Self::parse_mode(parts.next()?.trim());

        Some(ScheduleEntry { day, start, end, mode })
    }

    fn parse_brightness_entry(line: &str) -> Option<BrightnessEntry> {
        let mut parts = line.split(',');

        let day = Self::parse_day(parts.next()?.trim());
        let start = Self::parse_time(parts.next()?.trim())?;
        let end = Self::parse_time(parts.next()?.trim())?;
        let level = parts.next()?.trim().parse::<u8>().ok()?;

        Some(BrightnessEntry { day, start, end, level })
    }

    fn decode_base64(input: &str, output: &mut [u8]) -> usize {
        use base64::Engine;
        base64::engine::general_purpose::STANDARD.decode_slice(input.as_bytes(), output).unwrap_or_else(|_| 0)
    }

    fn parse_variant(input: &str) -> Variant {
        match input {
            "summer" => Variant::Summer,
            "winter" => Variant::Winter,
            _ => panic!("Unknown variant: {}", input),
        }
    }

    pub(super) fn parse_config(input: &[u8]) -> Config {
        let mut config = Config {
            wifi: WifiConfig {
                ssid: String::new(),
                password: String::new(),
                static_ip: None,
                dns: None,
                gateway: None,
                subnet: None,
            },
            motor: MotorConfig { max: 0 },
            boost: BoostConfig { short: 0, long: 0 },
            thermostat: Thermostat { temperature: 0.0 },
            time: TimeConfig {
                ntp_servers: Vec::new(),
                timezone: Str::new(),
            },
            secrets: Secrets {
                key: Vec::new(),
                iv: Vec::new(),
            },
            schedule_variant: Variant::Summer,
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
                            "static" => config.wifi.static_ip = Some(Str::from_str(v.trim()).unwrap_or_default()),
                            "dns" => config.wifi.dns = Some(Str::from_str(v.trim()).unwrap_or_default()),
                            "gateway" => config.wifi.gateway = Some(Str::from_str(v.trim()).unwrap_or_default()),
                            "subnet" => config.wifi.subnet = Some(Str::from_str(v.trim()).unwrap_or_default()),
                            _ => {
                                todo!();
                            }
                        }
                    }
                }
                Some("motor") => {
                    if let Some((k, v)) = line.split_once('=') {
                        match k.trim() {
                            "max" => {
                                if let Some(n) = Self::parse_u32(v.trim()) {
                                    config.motor.max = n;
                                }
                            }
                            _ => {todo!();}
                        }
                    }
                }
                Some("boost") => {
                    if let Some((k, v)) = line.split_once('=') {
                        match k.trim() {
                            "short" => {
                                if let Some(n) = Self::parse_u32(v.trim()) {
                                    config.boost.short = n;
                                }
                            }
                            "long" => {
                                if let Some(n) = Self::parse_u32(v.trim()) {
                                    config.boost.long = n;
                                }
                            }
                            _ => {todo!();}
                        }
                    }
                }
                Some("schedule") => {
                    if let Some((k, v)) = line.split_once('=') {
                        if k.trim() == "variant" {
                            config.schedule_variant = Self::parse_variant(v.trim());
                        }
                    }
                }
                Some("summer") => {
                    if let Some(entry) = Self::parse_schedule_entry(line) {
                        config.summer.push(entry).ok();
                    }
                }
                Some("winter") => {
                    if let Some(entry) = Self::parse_schedule_entry(line) {
                        config.winter.push(entry).ok();
                    }
                }
                Some("brightness") => {
                    if let Some(entry) = Self::parse_brightness_entry(line) {
                        config.brightness.push(entry).ok();
                    }
                }
                Some("secrets") => {
                    if let Some((k, v)) = line.split_once('=') {
                        match k.trim() {
                            "key" => {
                                config.secrets.key.resize_default(KEY_LENGTH).expect("Could not resize vec - resize is larger than cap");
                                assert_eq!(Self::decode_base64(v.trim(), &mut config.secrets.key), KEY_LENGTH);
                            }
                            "iv" => {
                                config.secrets.iv.resize_default(IV_LENGTH).expect("Could not resize vec - resize is larger than cap");
                                assert_eq!(Self::decode_base64(v.trim(), &mut config.secrets.iv), IV_LENGTH);
                            }
                            _ => {todo!();}
                        }
                    }
                }
                Some("thermostat") => {
                    if let Some((k, v)) = line.split_once('=') {
                        if k.trim() == "temperature" {
                            if let Some(f) = Self::parse_f32(v.trim()) {
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

        config
    }


    pub(super) fn get_state(&self, current_time: OffsetDateTime) -> fsm::State {
        let schedule = {
            match self.schedule_variant {
                Variant::Summer => { &self.summer }
                Variant::Winter => { &self.winter }
            }
        };

        for slot in schedule {
            if slot.is_in(current_time) {
                return slot.mode;
            }
        }

        //If there is no schedule entry then the default is Off
        fsm::State::Off
    }
}
