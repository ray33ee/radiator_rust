use time::OffsetDateTime;
use esp_hal::time::{Instant, Duration};

#[derive(Debug, Copy, Clone, PartialEq)]
pub(crate) enum State {
    Empty,
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

pub(crate) struct FSM<'a> {
    state: State,
    just_changed: bool,
    store: & 'a crate::storages::SdInterface<'a>,
    change_queue: Option<State>,
}

impl<'a> FSM<'a> {
    pub(crate) fn new(store: & 'a crate::storages::SdInterface<'a>) -> Self {
        Self {
            state: State::Start,
            just_changed: true,
            store,
            change_queue: None,
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

    pub(crate) fn change_from_schedule(&mut self, current_time: OffsetDateTime) {

        let sched_state = self.store.current_schedule_state(current_time);

        if sched_state != self.state {
            self.request_change(sched_state);
        }
    }

}