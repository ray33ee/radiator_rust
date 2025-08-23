use embedded_stepper::{create_stepper_4pin, Stepper4};
use esp_hal::delay::Delay;
use esp_hal::gpio::{Level, Output, OutputConfig};
use esp_hal::gpio::OutputPin;
use crate::storages::SdInterface;


const SPEED: u32 = 625;
const STEPS_PER_REV: u32 = 48;
const ABSOLUTE_MAX_POSITION: u32 = 50;

pub(crate) struct Motor<'a> {
    stepper: Stepper4<Output<'a>, Output<'a>, Output<'a>, Output<'a>, Delay>,
    steps_per_rev: u32,
    store: & 'a SdInterface<'a>,

    position: u32,
    max_position: u32,
}

impl<'a> Motor<'a> {
    pub(crate) fn new<P1: OutputPin + 'a, P2: OutputPin + 'a, P3: OutputPin + 'a, P4: OutputPin + 'a>(p1: P1, p2: P2, p3: P3, p4: P4, store: &'a SdInterface<'a>) -> Self {

        let op1 = Output::new(p1, Level::Low, OutputConfig::default());
        let op2 = Output::new(p2, Level::Low, OutputConfig::default());
        let op3 = Output::new(p3, Level::Low, OutputConfig::default());
        let op4 = Output::new(p4, Level::Low, OutputConfig::default());

        let mut stepper = create_stepper_4pin(op1, op2, op3, op4, Delay::new(), STEPS_PER_REV);

        stepper.set_speed(SPEED);

        Self {
            stepper,
            steps_per_rev: STEPS_PER_REV,
            store,
            position: store.position(),
            max_position: store.max_position(),
        }
    }

    pub(crate) fn calibrate_push(& mut self, revolutions: u32) {
        let _ = self.stepper.step(self.steps_per_rev as i32 * revolutions as i32);
        let _ = self.stepper.deenergise();
    }

    pub(crate) fn calibrate_pull(& mut self, revolutions: u32) {
        let _ = self.stepper.step(self.steps_per_rev as i32 * -(revolutions as i32));
        let _ = self.stepper.deenergise();
    }

    fn move_valve(& mut self, to_position: u32) {

        //Do not move past absolute maximum to protect enclosure
        if to_position > ABSOLUTE_MAX_POSITION {
            return;
        }

        let revs = to_position as i32 - self.position as i32;

        self.store.lock();

        let _ = self.stepper.step(self.steps_per_rev as i32 * revs);
        let _ = self.stepper.deenergise();

        self.store.set_position(to_position);

        self.store.unlock();

        self.position = to_position;

    }

    pub(crate) fn open_valve(& mut self) {
        self.move_valve(0);
    }

    pub(crate) fn close_valve(& mut self) {
        self.move_valve(self.max_position);
    }

    pub(crate) fn set_max_position(& mut self, max_position: u32) {
        self.max_position = max_position;
        todo!("Must implement the set_max_position function for store");
    }

}
