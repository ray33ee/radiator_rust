use esp_hal::gpio::Input;
use esp_hal::time::{Instant, Duration};
use esp_hal::gpio::Level;
use esp_println::println;
use heapless::Vec;
use crate::rgb::Color;

//All possible actions from the mode button
#[derive(Copy, Clone, Debug)]
pub(crate) enum Action {
    End,
    Nothing,
    ShortBoost,
    LongBoost,
    Cancel,
    Calibrate,
    Safe,
    Reset,
    Retract,
    ZeroUnlock,
}

impl Action {
    pub(crate) fn color(&self) -> Color {
        match self {
            Action::End => { Color {r: 0, g: 0, b: 0 } },
            Action::Nothing => { Color {r: 0, g: 0, b: 0 } }
            Action::ShortBoost => { Color {r: 255, g: 10, b: 0 } }
            Action::LongBoost => { Color {r: 255, g: 0, b: 0 } }
            Action::Cancel => { Color {r: 0, g: 10, b: 255 } }
            Action::Calibrate => { Color {r: 255, g: 150, b: 0 } }
            Action::Safe => { Color {r: 255, g: 255, b: 255 } }
            Action::Reset => { Color {r: 100, g: 0, b: 255 } }
            Action::Retract => { Color {r: 255, g: 255, b: 0 } }
            Action::ZeroUnlock => { Color {r: 0, g: 255, b: 100 } }
        }
    }
}

pub(crate) struct Mode<'a> {
    pin: Input<'a>,
    last_level: Level,
    last_check: Instant,

    just_pressed: bool,
    just_released: bool,

    pressed_time: Instant,

    index_changed: bool,

    selected_index: Option<u32>,

    actions: Vec<Action, 10>,

    released_action: Option<Action>,
}

impl<'a> Mode<'a> {

    pub(crate) fn new(pin: Input<'a>) -> Self {
        let level = pin.level();
        Self {
            pin,
            last_level: level,
            last_check: Instant::now(),

            just_pressed: false,
            just_released: false,

            pressed_time: Instant::now(),

            index_changed: false,

            selected_index: None,

            actions: Vec::new(),

            released_action: None,
        }
    }

    pub(crate) fn update(&mut self) {

        let level = self.pin.level();

        if self.last_level != level {
            if self.last_check.elapsed() > Duration::from_millis(30) {
                if self.pin.is_low() {
                    //pressed
                    self.just_released = false;
                    self.just_pressed = true;

                    self.selected_index = Some(0);

                    self.index_changed = true;

                    self.pressed_time = Instant::now();
                } else {
                    //released
                    self.just_released = true;
                    self.just_pressed = false;

                    self.released_action = self.current_action();

                    self.selected_index = None;
                }
                self.last_check = Instant::now();
            }
            self.last_level = level;
        }

        if self.is_pressed() {
            let millis = self.pressed_time.elapsed().as_millis();
            let calculated_index = millis / 1000;

            if let Some(index) = self.selected_index {
                if index != calculated_index as u32 {
                    self.selected_index = Some(calculated_index as u32);
                    self.index_changed = true;
                }
            }

        }

    }

    pub(crate) fn just_pressed(& mut self) -> bool {
        let pressed = self.just_pressed;
        self.just_pressed = false;
        pressed
    }

    pub(crate) fn just_released(& mut self) -> bool {
        let released = self.just_released;
        self.just_released = false;
        released
    }

    pub(crate) fn mode_just_changed(& mut self) -> bool {
        let mode_changed = self.index_changed;
        self.index_changed = false;
        mode_changed
    }

    pub(crate) fn is_pressed(&self) -> bool {
        self.pin.level() == Level::Low
    }

    pub(crate) fn index(&self) -> Option<u32> {
        self.selected_index
    }

    pub(crate) fn push_action(&mut self, action: Action) {
        self.actions.push(action).unwrap();
    }

    pub(crate) fn clear_actions(&mut self) {
        self.actions.clear();
    }

    pub(crate) fn current_action(&self) -> Option<Action> {

        match self.selected_index {
            None => {
                None
            }
            Some(index) => {
                if index >= self.actions.len() as u32 {
                    None
                } else {
                    Some(self.actions[index as usize])
                }
            }
        }
    }

    pub(crate) fn released_action(& mut self) -> Option<Action> {
        let action = self.released_action;
        self.released_action = None;
        action
    }

}
