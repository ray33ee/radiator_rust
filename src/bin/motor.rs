use embedded_stepper::{create_stepper_4pin, Stepper4};
use esp_hal::delay::Delay;
use esp_hal::gpio::{Level, Output, OutputConfig};
use esp_hal::gpio::OutputPin;
use crate::storages;


const SPEED: u32 = 625;
const STEPS_PER_REV: u32 = 48;
pub(crate) const ABSOLUTE_MAX_POSITION: u32 = 50;

pub(crate) enum MotorError {
    MotorLocked,
    MotorMaxNotSet,
    AbsoluteMaxExceeded,
}

pub(crate) struct Motor<'a> {
    stepper: Stepper4<Output<'a>, Output<'a>, Output<'a>, Output<'a>, Delay>,
    enable: Output<'a>,
    steps_per_rev: u32,
    position: u32,

    max_position: u32,
    short_boost_duration: u32,
    long_boost_duration: u32,
}

impl<'a> Motor<'a> {
    pub(crate) fn new<P1: OutputPin + 'a, P2: OutputPin + 'a, P3: OutputPin + 'a, P4: OutputPin + 'a, EN: OutputPin + 'a>(p1: P1, p2: P2, p3: P3, p4: P4, en: EN) -> Self {

        let op1 = Output::new(p1, Level::Low, OutputConfig::default());
        let op2 = Output::new(p2, Level::Low, OutputConfig::default());
        let op3 = Output::new(p3, Level::Low, OutputConfig::default());
        let op4 = Output::new(p4, Level::Low, OutputConfig::default());

        let enable = Output::new(en, Level::Low, OutputConfig::default());

        let mut stepper = create_stepper_4pin(op1, op2, op3, op4, Delay::new(), STEPS_PER_REV);

        stepper.set_speed(SPEED);

        let (max_position, short_boost_duration, long_boost_duration) = match crate::storages::load_from_page(crate::storages::MOTOR_ADDRESS) {
            Some((max_position, short_boost_duration, long_boost_duration)) => (max_position, short_boost_duration, long_boost_duration),
            None => {(0u32, 600u32, 1800u32)}
        };

        Self {
            stepper,
            enable,
            steps_per_rev: STEPS_PER_REV,
            position: storages::get_position(),
            max_position,
            short_boost_duration,
            long_boost_duration,
        }
    }

    pub(crate) fn calibrate_push(& mut self, revolutions: u32) {
        self.enable.set_high();
        let _ = self.stepper.step(self.steps_per_rev as i32 * revolutions as i32);
        let _ = self.stepper.deenergise();
        self.enable.set_low();
    }

    pub(crate) fn calibrate_pull(& mut self, revolutions: u32) {
        self.enable.set_high();
        let _ = self.stepper.step(self.steps_per_rev as i32 * -(revolutions as i32));
        let _ = self.stepper.deenergise();
        self.enable.set_low();
    }

    fn move_valve(& mut self, to_position: u32) -> Result<(), MotorError> {

        if storages::is_locked() {
            return Err(MotorError::MotorLocked);
        }

        //Do not move past absolute maximum to protect enclosure
        if to_position > ABSOLUTE_MAX_POSITION {
            return Err(MotorError::AbsoluteMaxExceeded);
        }

        if self.max_position == 0 {
            return Err(MotorError::MotorMaxNotSet);
        }

        if self.position == to_position {
            return Ok(());
        }

        let revs = to_position as i32 - self.position as i32;

        self.enable.set_high();

        storages::lock();

        critical_section::with( |_| {
            let _ = self.stepper.step(self.steps_per_rev as i32 * revs);
        });

        let _ = self.stepper.deenergise();

        storages::unlock_and_set_pos(to_position);

        self.enable.set_low();

        self.position = to_position;

        Ok(())
    }

    pub(crate) fn open_valve(& mut self) -> Result<(), MotorError> {
        self.move_valve(0)
    }

    pub(crate) fn close_valve(& mut self) -> Result<(), MotorError> {
        self.move_valve(self.max_position)
    }

    pub(crate) fn set_max_position(& mut self, max_position: u32) {
        self.max_position = max_position;
        self.save();
    }

    pub(crate) fn max_position(& self) -> u32 {
        self.max_position
    }

    pub(crate) fn short_boost(& self) -> u32 {
        self.short_boost_duration
    }

    pub(crate) fn long_boost(& self) -> u32 {
        self.long_boost_duration
    }

    pub(crate) fn set_short_boost(& mut self, duration: u32) {
        self.short_boost_duration = duration;
        self.save();
    }

    pub(crate) fn set_long_boost(& mut self, duration: u32) {
        self.long_boost_duration = duration;
        self.save();
    }

    fn save(&self) {
        crate::storages::save_to_page(crate::storages::MOTOR_ADDRESS, (self.max_position, self.short_boost_duration, self.long_boost_duration));
    }

}
