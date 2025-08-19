use embedded_sdmmc::{SdCard, VolumeIdx, VolumeManager, TimeSource, Timestamp, Mode, Error, DirEntry};
use esp_hal::{
    gpio::{OutputPin, Output, OutputConfig, Level},
    spi::master::{Spi, Config},
    delay::Delay,
};
use embedded_hal_bus::spi::ExclusiveDevice;
use esp_hal::gpio::InputPin;
use embedded_io::Write;

const CONFIG_NAME: & 'static str = "CONFIG.txt";
const LOCK_NAME: & 'static str = "LOCK";
const POSITION_NAME: & 'static str = "POSITION.txt";

//todo maybe replace this with a real timesource, probs not. I think its only used for file timstamps
struct NullTimeSource;

impl TimeSource for NullTimeSource {
    fn get_timestamp(&self) -> Timestamp {
        Timestamp::from_calendar(2000, 1, 1, 1, 1, 1).unwrap()
    }
}

pub(crate) struct SdInterface<'a>
where
{
    vm: VolumeManager<SdCard<ExclusiveDevice<Spi<'a, esp_hal::Blocking>, Output<'a>, Delay>, Delay>, NullTimeSource>,
    config: super::config::Config,
}

impl<'a> SdInterface<'a> {
    pub(crate) fn new<SCK: OutputPin + 'a, MOSI: OutputPin + 'a, MISO: InputPin + 'a, CS: OutputPin + 'a, SPI: esp_hal::spi::master::Instance + 'a>(
        spi2: SPI,
        sck: SCK,
        mosi: MOSI,
        miso: MISO,
        cs: CS,
    ) -> Self  {
        let spi = Spi::new(spi2, Config::default())
            .unwrap()
            .with_sck(sck)
            .with_mosi(mosi)
            .with_miso(miso);

        let cs = Output::new(cs, Level::Low, OutputConfig::default());

        let spid = ExclusiveDevice::new(spi, cs, Delay::new()).expect("Could not create spi device");

        let sdcard = SdCard::new(spid, Delay::new());

        let vm = VolumeManager::new(sdcard, NullTimeSource);

        let config = {
            let partition = vm.open_volume(VolumeIdx(0)).expect("Could not open volume");

            let root = partition.open_root_dir().expect("Could not open root directory");

            let config = root.open_file_in_dir(CONFIG_NAME, Mode::ReadOnly).expect("Could not open config file");

            let mut buffer = heapless::Vec::<_, 4000>::new();

            buffer.resize_default(config.length() as usize).expect("Could not resize vec");

            config.read(buffer.as_mut_slice()).expect("Could not read config file");

            super::config::Config::parse_config(&buffer)
        };

        Self {
            vm,
            config,
        }
    }

    pub(crate) fn ssid(&self) -> &str {
        self.config.wifi.ssid.as_str()
    }

    pub(crate) fn password(&self) -> &str {
        self.config.wifi.password.as_str()
    }

    pub(crate) fn max_position(&self) -> u32 {
        self.config.motor.max
    }

    pub(crate) fn thermostat(&self) -> f32 {
        self.config.thermostat.temperature
    }

    pub(crate) fn lock(& self) {
        let partition = self.vm.open_volume(VolumeIdx(0)).expect("Could not open volume");

        let root = partition.open_root_dir().expect("Could not open root directory");

        //Create the lock and fail if it exists
        root.open_file_in_dir(LOCK_NAME, Mode::ReadWriteCreate).expect("Create file failed. file might have already existed?");

    }

    pub(crate) fn unlock(& self) {

        let partition = self.vm.open_volume(VolumeIdx(0)).expect("Could not open volume");

        let root = partition.open_root_dir().expect("Could not open root directory");

        root.delete_file_in_dir(LOCK_NAME).expect("Could not delete lock file");

    }

    pub(crate) fn is_locked(& self) -> bool {
        let partition = self.vm.open_volume(VolumeIdx(0)).expect("Could not open volume");

        let root = partition.open_root_dir().expect("Could not open root directory");

        root.find_directory_entry(LOCK_NAME).is_ok()
    }

    pub(crate) fn position(& self) -> u32 {
        let partition = self.vm.open_volume(VolumeIdx(0)).expect("Could not open volume");

        let root = partition.open_root_dir().expect("Could not open root directory");

        let pos = root.open_file_in_dir(POSITION_NAME, Mode::ReadOnly).expect("Could not open config file");

        let mut buffer = heapless::Vec::<_, 20>::new();

        buffer.resize_default(pos.length() as usize).expect("Could not resize vec");

        pos.read(buffer.as_mut_slice()).expect("Could not read config file");

        let i: u32 = atoi::atoi(&buffer[..pos.length() as usize]).expect("Could not convert position file to integer");

        i
    }

    pub(crate) fn set_position(& self, position: u32) {
        let partition = self.vm.open_volume(VolumeIdx(0)).expect("Could not open volume");

        let root = partition.open_root_dir().expect("Could not open root directory");

        let mut pos = root.open_file_in_dir(POSITION_NAME, Mode::ReadWriteTruncate).expect("Could not open position file");

        let mut itoa = itoa::Buffer::new();
        let s = itoa.format(position);

        pos.write(s.as_bytes()).expect("Could not write to position file");

        pos.flush().expect("Could not flush position file");

        pos.close().expect("Could not close position file");
    }

}