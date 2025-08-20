use esp_hal::time::Instant;
use time::OffsetDateTime;
use core::option::Option;

pub(crate) struct RTC {
    epoch: Option<i64>,
    instant: Option<Instant>,
}

impl RTC {
    pub(crate) fn new() -> Self {
        Self {
            epoch: None,
            instant: None,
        }
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