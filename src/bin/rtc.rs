use esp_hal::time::Instant;
use time::{OffsetDateTime, Date, Time};
use core::option::Option;

pub(crate) struct RTC {
    epoch: Option<i64>,
    instant: Option<Instant>,
}

fn to_u8(s: &[u8]) -> u8 {
    (s[0] - b'0') * 10 + (s[1] - b'0')
}

fn to_u16(s: &[u8]) -> u16 {
    (s[0] - b'0') as u16 * 1000 +
    (s[1] - b'0') as u16 * 100 +
    (s[2] - b'0') as u16 * 10 +
    (s[3] - b'0') as u16
}


impl RTC {
    pub(crate) fn new() -> Self {
        Self {
            epoch: None,
            instant: None,
        }
    }


    pub(crate) fn epoch_from_iso(iso_string: &str) -> i64 {
        let iso = iso_string.as_bytes();
        let date = Date::from_calendar_date(
            to_u16(&iso[0..4]) as i32,
            to_u8(&iso[5..7]).try_into().unwrap(),
            to_u8(&iso[8..10]),
        ).unwrap();

        let time = Time::from_hms(
            to_u8(&iso[11..13]),
            to_u8(&iso[14..16]),
            to_u8(&iso[17..19]),
        ).unwrap();

        let date_time = OffsetDateTime::new_utc(date, time);

        date_time.unix_timestamp()
    }

    pub(crate) fn update_epoch(&mut self, epoch: i64) {
        self.epoch = Some(epoch);
        self.instant = Some(Instant::now());
    }

    pub(crate) fn date_time(&self) -> Option<OffsetDateTime> {

        if let Some(epoch) = &self.epoch {
            if let Some(instant) = &self.instant {
                return Some(OffsetDateTime::from_unix_timestamp(*epoch + instant.elapsed().as_secs() as i64).unwrap());
            }
        }

        None
    }
}