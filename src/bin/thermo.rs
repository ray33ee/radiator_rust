
use esp_hal::gpio::OutputPin;
use dht22_sensor::Dht22;
use esp_hal::{
    clock::CpuClock,
    delay::Delay,
    gpio::{DriveMode, Output, OutputConfig, Pull, Flex},
    main,
};

pub struct Thermometer<'a> {
    sensor: Dht22<Flex<'a>, Delay>,
    last_read: Option<f32>,
}

impl<'a> Thermometer<'a> {
    pub fn new<W: OutputPin + 'a>(one_wire: W) -> Self {
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

        Self {
            sensor: dht22,
            last_read: None,
        }

    }

    pub fn get_temperature(&mut self) -> f32 {
        if let Ok(reading) = self.sensor.read() {
            self.last_read = Some(reading.temperature);
        }
        self.last_read.unwrap()
    }
}
