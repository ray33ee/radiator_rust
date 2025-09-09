use esp_hal::Blocking;
use aht20_driver::AHT20Initialized;
use esp_hal::i2c::master::I2c;

pub(crate) struct Thermometer<'a> {
    //sensor: Dht22<Flex<'a>, Delay>,

    sensor: AHT20Initialized<'a, I2c<'a, Blocking>>,

    last_read: Option<f32>,
    thermostat: f32,
}

impl<'a> Thermometer<'a> {
    pub(crate) fn new(sensor: AHT20Initialized<'a, I2c<'a, Blocking>>) -> Self {

        let thermostat = crate::storages::load_from_page(crate::storages::THERMO_ADDRESS).unwrap_or_else(|| 23.0);

        Self {
            sensor,
            last_read: None,
            thermostat,
        }

    }

    pub(crate) fn get_temperature(&mut self) -> Result<f32, ()> {

        match self.sensor.measure(& mut embassy_time::Delay) {
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
