
use esp_hal::gpio::OutputPin;
use dht22_sensor::{Dht22};
use esp_hal::{
    delay::Delay,
    gpio::{DriveMode, Output, OutputConfig, Pull, Flex},
};

pub(crate) struct Thermometer<'a> {
    sensor: Dht22<Flex<'a>, Delay>,
    last_read: Option<f32>,
    thermostat: f32,
}

impl<'a> Thermometer<'a> {
    pub(crate) fn new<W: OutputPin + 'a>(one_wire: W) -> Self {
        //Setup the pin as input/output
        let od_config = OutputConfig::default()
            .with_drive_mode(DriveMode::OpenDrain)
            .with_pull(Pull::None);

        let od_for_dht22 = Output::new(
            one_wire, esp_hal::gpio::Level::High, od_config
        ).into_flex();

        od_for_dht22.peripheral_input();

        //Create the delay object and make the dht22
        let delay = Delay::new();

        let dht22 = Dht22::new(od_for_dht22, delay);

        let thermostat = crate::storages::load_from_page(crate::storages::THERMO_ADDRESS).unwrap_or_else(|| 23.0);

        Self {
            sensor: dht22,
            last_read: None,
            thermostat,
        }

    }

    pub(crate) fn get_temperature(&mut self) -> Result<f32, ()> {

        match self.sensor.read() {
            Ok(reading) => {
                self.last_read = Some(reading.temperature);
            },
            Err(_) => {
                //If the read failed and there is no backup value:
                if self.last_read.is_none() {
                    return Err(());
                }
            },
        }

        Ok(self.last_read.unwrap())
    }

    pub(crate) fn thermostat(&self) -> f32 {
        self.thermostat
    }

    pub(crate) fn set_thermostat(&mut self, thermostat: f32) {
        self.thermostat = thermostat;
        crate::storages::save_to_page(crate::storages::THERMO_ADDRESS, thermostat);
    }
}
