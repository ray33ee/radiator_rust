#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]
#![warn(clippy::all)]
#![warn(clippy::pedantic)]
#![warn(clippy::nursery)]
#![warn(clippy::cargo)]
#![warn(missing_docs)] // or deny
#![warn(unsafe_code)]
#![warn(unused_extern_crates)]
#![warn(rust_2018_idioms)]
#![warn(missing_debug_implementations)]
#![warn(missing_copy_implementations)]
#![warn(missing_docs)]
#![warn(unreachable_pub)]
#![warn(unused)]
#![warn(dead_code)]
#![warn(nonstandard_style)]
#![warn(trivial_casts)]
#![warn(trivial_numeric_casts)]
#![warn(variant_size_differences)]

#[panic_handler]
fn panic(_: &core::panic::PanicInfo<'_>) -> ! {
    loop {}
}

mod config;

use esp_hal::{
    clock::CpuClock,
    main,
    time::{
        Duration,
        Instant
    },
    timer::timg::TimerGroup,
};
use esp_println::println;

// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();

#[main]
fn main() -> ! {
    // generator version: 0.5.0

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    esp_alloc::heap_allocator!(size: 64 * 1024);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    let _init = esp_wifi::init(timg0.timer0, esp_hal::rng::Rng::new(peripherals.RNG)).unwrap();

    const CONFIG_TEXT: &[u8] = br#"[wifi]
ssid=name
password=pswd
static=192.168.0.1
dns=192.168.1.255
gateway=192.128.1.255
subnet=255.255.255.0

[motor]
position=0
max=40

[boost]
short=600
long=1800

[schedule]
variant=summer

[summer]
MON,09:00,19:00,REGULATE
TUE,09:00,19:00,REGULATE
WED,09:00,19:00,REGULATE
THU,09:00,19:00,REGULATE
FRI,09:00,19:00,REGULATE
SAT,09:00,19:00,REGULATE
SUN,09:00,19:00,REGULATE
SUN,19:01,19:02,DESCALE

[winter]
SUN,19:01,19:02,DESCALE

[brightness]
MON,00:00,09:00,10
MON,22:00,23:59,10
TUE,00:00,09:00,10
TUE,22:00,23:59,10
WED,00:00,09:00,10
WED,22:00,23:59,10
THU,00:00,09:00,10
THU,22:00,23:59,10
FRI,00:00,09:00,10
FRI,22:00,23:59,10
SAT,00:00,09:00,10
SAT,22:00,23:59,10
SUN,00:00,09:00,10
SUN,22:00,23:59,10

[secrets]
key=mMvvFJFTeND43qOvnL/HPyqpc5Q2cymIYNrfwxGNUkdt4nCJ6HJ93u0rm8WbKp0K+4Es9nY6vobxTM1R3Kzer1EAi+KG+mnio+Q15BAJvgzCY3bpQGFNfeRoe8A/DkkdzDCOkanTBmQgkBat+odan3MdHFgITiOlu+FOBPyfB7I=
iv=SIlsbjf0nBsZg7GxTWeRVQ==

[thermostat]
temperature=23.5

[time]
ntp1=uk.pool.ntp.org
timezone=GMT0BST,M3.5.0/1,M10.5.0/2
"#;

    let config = match config::parse_config(CONFIG_TEXT) {
        Some(cfg) => cfg,
        None => {
            println!("⚠️ Failed to parse CONFIG.txt");
            loop {}
        }
    };

    println!("WiFi SSID: {}", config.wifi.ssid);
    println!("WiFi Password: {}", config.wifi.password);
    println!("Static IP: {}", config.wifi.static_ip);
    println!("DNS: {}", config.wifi.dns);
    println!("Gateway: {}", config.wifi.gateway);
    println!("Subnet: {}", config.wifi.subnet);

    println!("Motor Position: {}", config.motor.position);
    println!("Motor Max: {}", config.motor.max);

    println!("Boost Short: {}", config.boost.short);
    println!("Boost Long: {}", config.boost.long);

    println!("Schedule Variant: {}", config.schedule_variant);

    println!("Summer Schedule:");
    for entry in config.summer.iter() {
        println!("  {:?} {:02}:{:02} - {:02}:{:02} {:?}",
                 entry.day,
                 entry.start.hour, entry.start.minute,
                 entry.end.hour, entry.end.minute,
                 entry.mode
        );
    }

    println!("Winter Schedule:");
    for entry in config.winter.iter() {
        println!("  {:?} {:02}:{:02} - {:02}:{:02} {:?}",
                 entry.day,
                 entry.start.hour, entry.start.minute,
                 entry.end.hour, entry.end.minute,
                 entry.mode
        );
    }

    println!("Brightness Schedule:");
    for entry in config.brightness.iter() {
        println!("  {:?} {:02}:{:02} - {:02}:{:02} Level: {}",
                 entry.day,
                 entry.start.hour, entry.start.minute,
                 entry.end.hour, entry.end.minute,
                 entry.level
        );
    }

    println!("Thermostat Temperature: {}", config.thermostat.temperature);

    println!("Time Servers:");
    for ntp in config.time.ntp_servers.iter() {
        println!("  {}", ntp);
    }

    println!("Timezone: {}", config.time.timezone);

    println!("Secrets:");
    println!("  Key Length: {}", config.secrets.key_len);
    println!("  IV Length: {}", config.secrets.iv_len);
    println!("  Key (first 8): {:02X?}", &config.secrets.key[..8]);
    println!("  IV  (first 8): {:02X?}", &config.secrets.iv[..8]);

    loop {
        let delay_start = Instant::now();
        while delay_start.elapsed() < Duration::from_millis(500) {}
    }

    // for inspiration have a look at the examples at https://github.com/esp-rs/esp-hal/tree/esp-hal-v1.0.0-rc.0/examples/src/bin
}
