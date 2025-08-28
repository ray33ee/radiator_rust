use alloc::vec::Vec;
use core::fmt;
use core::fmt::{Display, Formatter};
use time::{OffsetDateTime, Weekday};
use esp_hal::time::{Instant, Duration};
use serde::{Serialize, Deserialize};
use crate::{fsm, storages};
use crate::fsm::Variant::{Summer, Winter};
use crate::storages::SCHEDULE_ADDRESS;

#[derive(Debug, Copy, Clone, PartialEq)]
pub(crate) enum State {
    Start,
    Regulate,
    Off,
    Boost(Instant, Duration),
    Calibrate,
    SafeMode,
    Reset,
    Descale,
    Warning(u32),
    Cancel,
}



fn parse_day(s: &str) -> Option<Weekday> {
    match s {
        "MON" | "mon" => Some(Weekday::Monday),
        "TUE" | "tue" => Some(Weekday::Tuesday),
        "WED" | "wed" => Some(Weekday::Wednesday),
        "THU" | "thu" => Some(Weekday::Thursday),
        "FRI" | "fri" => Some(Weekday::Friday),
        "SAT" | "sat" => Some(Weekday::Saturday),
        "SUN" | "sun" => Some(Weekday::Sunday),
        _ => None,
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

#[derive(Clone, Debug, Serialize, Deserialize)]
pub(crate) enum ScheduleState {
    Regulate,
    Off,
    SafeMode,
    Descale,
}

impl ScheduleState {

    fn to_fsm_state(& self) -> State {
        match self {
            ScheduleState::Regulate => {State::Regulate}
            ScheduleState::Off => {State::Off}
            ScheduleState::SafeMode => {State::SafeMode}
            ScheduleState::Descale => {State::Descale}
        }
    }



}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub(super) struct ScheduleEntry {
    pub(super) day: Weekday,
    pub(super) start: u16,
    pub(super) end: u16,
    pub(super) mode: ScheduleState,
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


    fn parse_mode(s: &str) -> Option<crate::fsm::ScheduleState> {
        match s {
            "OFF" | "off" | "Off" => Some(crate::fsm::ScheduleState::Off),
            "REGULATE" | "regulate" | "Regulate" => Some(crate::fsm::ScheduleState::Regulate),
            "DESCALE" | "descale" | "Descale" => Some(crate::fsm::ScheduleState::Descale),
            _ => None,
        }
    }

    fn parse_schedule_entry(line: &str) -> Option<ScheduleEntry> {
        let mut parts = line.split(',');

        let day = parse_day(parts.next()?.trim())?;
        let start = parse_time(parts.next()?.trim())?;
        let end = parse_time(parts.next()?.trim())?;
        let mode = Self::parse_mode(parts.next()?.trim())?;

        Some(ScheduleEntry { day, start, end, mode })
    }

    pub(crate) fn from_str(s: &str) -> Option<ScheduleEntry> {
        Self::parse_schedule_entry(s)
    }
}

impl Display for ScheduleEntry {
    fn fmt(&self, f: &mut Formatter<'_>) -> fmt::Result {

        let day_str = match self.day {
            Weekday::Monday => {"mon"}
            Weekday::Tuesday => {"tue"}
            Weekday::Wednesday => {"wed"}
            Weekday::Thursday => {"thu"}
            Weekday::Friday => {"fri"}
            Weekday::Saturday => {"sat"}
            Weekday::Sunday => {"sun"}
        };

        write!(f, "{},{:02}:{:02},{:02}:{:02},{:?}",
               day_str,
            self.start / 60,
            self.start % 60,
            self.end / 60,
            self.end % 60,
            self.mode
        )
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
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


    fn parse_brightness_entry(line: &str) -> Option<BrightnessEntry> {
        let mut parts = line.split(',');

        let day = parse_day(parts.next()?.trim())?;
        let start = parse_time(parts.next()?.trim())?;
        let end = parse_time(parts.next()?.trim())?;
        let level = parts.next()?.trim().parse::<u8>().ok()?;

        Some(BrightnessEntry { day, start, end, level })
    }


    pub(crate) fn from_str(s: &str) -> Option<BrightnessEntry> {
        Self::parse_brightness_entry(s)
    }
}


impl Display for BrightnessEntry {
    fn fmt(&self, f: &mut Formatter<'_>) -> fmt::Result {

        let day_str = match self.day {
            Weekday::Monday => {"mon"}
            Weekday::Tuesday => {"tue"}
            Weekday::Wednesday => {"wed"}
            Weekday::Thursday => {"thu"}
            Weekday::Friday => {"fri"}
            Weekday::Saturday => {"sat"}
            Weekday::Sunday => {"sun"}
        };

        write!(f, "{},{:02}:{:02},{:02}:{:02},{}",
               day_str,
               self.start / 60,
               self.start % 60,
               self.end / 60,
               self.end % 60,
               self.level,
        )
    }
}


#[derive(Clone, Debug, Copy, Serialize, Deserialize)]
pub(super) enum Variant {
    Summer,
    Winter,
}

impl Variant {
    pub(crate) fn from_str(s: &str) -> Option<Variant> {
        match s {
            "Summer" | "summer" => {
                Some(Summer)
            }
            "Winter" | "winter" => {
                Some(Winter)
            }
            _ => None,
        }
    }
}


impl State {

    pub(crate) fn is_calibrate(&self) -> bool {
        *self == State::Calibrate
    }

    pub(crate) fn is_safe_mode(&self) -> bool {
        *self == State::SafeMode
    }

    pub(crate) fn is_warning(&self) -> bool {
        if let State::Warning(_) = *self {
            true
        } else {
            false
        }
    }

    pub(crate) fn is_cancel(&self) -> bool {
        *self == State::Cancel
    }
}

pub(crate) struct FSM {
    state: State,
    just_changed: bool,
    change_queue: Option<State>,
    stored_brightness: u8,

    variant: Variant,
    summer: Vec<ScheduleEntry>,
    winter: Vec<ScheduleEntry>,

    brightness: Vec<BrightnessEntry>,

}

impl FSM {
    pub(crate) fn new() -> Self {

        let (variant, summer, winter, brightness) = match storages::load_from_page(SCHEDULE_ADDRESS) {
            Some((variant, summer, winter, brightness)) => (variant, summer, winter, brightness),
            None => {
                (Winter, Vec::new(),Vec::new(),Vec::new(),)
            }
        };

        Self {
            state: State::Start,
            just_changed: true,
            change_queue: None,
            stored_brightness: 100,

            variant,
            summer,
            winter,

            brightness,
        }
    }

    pub(crate) fn request_change(&mut self, candidate: State) {
        self.change_queue = Some(candidate);
    }

    pub (crate) fn force_change(&mut self, state: State) {
        self.change_queue = Some(state);
        self.start_queued();
    }

    //States must call this regularly to allow or prevent state changes.
    //Queueing potential state changes allows states to refuse.
    pub(crate) fn handle_request<F: FnOnce(State) -> bool>(& mut self, handler: F) {
        //If there's a queued state change and the current state allows it, change to queued
        if let Some(queued) = self.change_queue {
            if handler(queued) {
                self.start_queued();
            }
        }
    }

    //Most states dont need code to decide whether to change states, so this function is
    //provided as shorthand for accepting any queued state
    pub(crate) fn allow_request(&mut self) {
        if self.change_queue.is_some() {
            self.start_queued();
        }
    }

    fn start_queued(& mut self) {
        self.state = self.change_queue.expect("start_queued called with no queued state");
        self.just_changed = true;
        self.change_queue = None;
    }

    pub(crate) fn just_changed(& mut self) -> bool {
        let changed = self.just_changed;
        self.just_changed = false;
        changed
    }

    pub(crate) fn state(&self) -> &State {
        &self.state
    }

    pub(super) fn get_state(&self, current_time: OffsetDateTime) -> fsm::State {
        let schedule = {
            match self.variant {
                Variant::Summer => { &self.summer }
                Variant::Winter => { &self.winter }
            }
        };

        for slot in schedule {
            if slot.is_in(current_time) {
                return slot.mode.to_fsm_state();
            }
        }

        //If there is no schedule entry then the default is Off
        fsm::State::Off
    }

    pub(super) fn _get_brightness(&self, current_time: OffsetDateTime) -> u8 {
        let schedule = &self.brightness;

        for slot in schedule {
            if slot.is_in(current_time) {
                return slot.level;
            }
        }

        //If there is no schedule entry then the default is max brightness, 100
        100
    }


    pub(crate) fn change_from_schedule(&mut self, current_time: OffsetDateTime) {

        let sched_state = self.get_state(current_time);

        if sched_state != self.state {
            self.request_change(sched_state);
        }


    }


     pub(crate) fn get_brightness(& mut self, current_time: OffsetDateTime) -> Option<u8> {

        let brightness = self._get_brightness(current_time);

        if brightness != self.stored_brightness {
            self.stored_brightness = brightness;
            Some(brightness)
        } else {
            None
        }
    }

    pub(crate) fn variant(&self) -> Variant {
        self.variant
    }

    pub(crate) fn set_variant(&mut self, variant: Variant) {
        self.variant = variant;
        self.save();
    }

    pub(crate) fn add_summer(&mut self, slot: ScheduleEntry) {
        self.summer.push(slot);
        self.save();
    }

    pub(crate) fn clear_summer(&mut self) {
        self.summer.clear();
        self.save();
    }

    pub(crate) fn add_winter(&mut self, slot: ScheduleEntry) {
        self.winter.push(slot);
        self.save();
    }

    pub(crate) fn clear_winter(&mut self) {
        self.winter.clear();
        self.save();
    }

    pub(crate) fn add_brightness(&mut self, slot: BrightnessEntry) {
        self.brightness.push(slot);
        self.save();
    }

    pub(crate) fn clear_brightness(&mut self) {
        self.brightness.clear();
        self.save();
    }

    pub(crate) fn summer(&self) -> &[ScheduleEntry] {
        self.summer.as_slice()
    }

    pub(crate) fn winter(&self) -> &[ScheduleEntry] {
        self.winter.as_slice()
    }

    pub(crate) fn brightness(&self) -> &[BrightnessEntry] {
        self.brightness.as_slice()
    }

    fn save(&self) {
        storages::save_to_page(SCHEDULE_ADDRESS, (&self.variant, &self.summer, &self.winter, &self.brightness));
    }


}