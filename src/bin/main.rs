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
#![warn(unsafe_code)]
#![warn(unused_extern_crates)]
#![warn(rust_2018_idioms)]
#![warn(missing_debug_implementations)]
#![warn(missing_copy_implementations)]
#![warn(unreachable_pub)]
#![warn(unused)]
#![warn(dead_code)]
#![warn(nonstandard_style)]
#![warn(trivial_casts)]
#![warn(trivial_numeric_casts)]
#![warn(variant_size_differences)]
extern crate alloc;

mod storages;
mod commands;
mod motor;
mod thermo;
mod rtc;
mod network;
mod rgb;
mod fsm;
mod mode_button;
mod backtrace;

use crate::fsm::FSM;
use crate::rgb::State as RGBState;
use esp_hal::ledc::LSGlobalClkSource;
use esp_hal::ledc::Ledc;
use esp_hal::gpio::{InputConfig, OutputConfig, Pull};
use esp_hal::gpio::Level;
use esp_hal::gpio::Output;
use alloc::format;
use alloc::string::String;
use embassy_time::{Duration, Timer};
use embassy_net::tcp::TcpSocket;
use embassy_net::Stack;
use embassy_net::StackResources;
use static_cell::StaticCell;
use esp_hal::timer::systimer::SystemTimer;
use esp_wifi::wifi::WifiDevice;
use crate::commands::{Function, Message};
use esp_hal::{
    clock::CpuClock,
    main,
    timer::timg::TimerGroup,
    rng::Rng,
    time::Instant,
};
use esp_println::println;
//use esp_backtrace as _;
use motor::Motor;
use crate::thermo::Thermometer;
use embassy_executor::Spawner;
use crate::rtc::RTC;
use embassy_sync::channel::Channel;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::mutex::Mutex;
use smoltcp::wire::IpEndpoint;
use esp_hal::ledc::timer::TimerIFace;
use esp_hal::ledc::channel::ChannelIFace;
use crate::rgb::{RGBLED, Color};
use esp_hal::ledc::channel::ChannelHW;
use backtrace as _;

// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();


#[embassy_executor::task]
async fn net_task(mut runner: embassy_net::Runner<'static, WifiDevice<'static>>) -> ! {
    runner.run().await;
}


#[embassy_executor::task]
async fn world_time_task(
    stack: Stack<'static>,
    //time: & 'static Channel<NoopRawMutex, (), 1>,
    time_mutex: & 'static Mutex<NoopRawMutex, RTC>,

) -> ! {

    let mut rx_buf = [0u8; 2024];
    let mut tx_buf = [0u8; 2024];

    loop {

        let mut socket = TcpSocket::new(stack, &mut rx_buf, &mut tx_buf);

        let world_time_api_dns = "192.168.1.111";

        let time_api_addr = stack.dns_query(world_time_api_dns, smoltcp::wire::DnsQueryType::A).await.unwrap()[0];

        let time_endpoint = IpEndpoint::new(time_api_addr, 8080);

        socket.connect(time_endpoint).await.expect(format!("Could not connect to world time api: {}", world_time_api_dns).as_str());

        println!("Connected to time api...");

        socket.write(b"GET /api/ip HTTP/1.1\r\n\
Host: worldtimeapi.org\r\n\
User-Agent: esp32-rust\r\n\
Connection: close\r\n\
\r\n").await.unwrap();

        let result_got_time = socket.read_with(|bytes| {

            if let Some(body_begin) = bytes.windows(4).position(|window| window == b"\r\n\r\n") {
                if let Some(body_end) = &bytes[body_begin+4..].iter().position(|&b| b == b'}') {
                    let json = core::str::from_utf8(&bytes[body_begin+4..*body_end + body_begin+5]).unwrap();

                    let parsed: serde_json::Value = serde_json::from_str(json).unwrap();

                    let datetime_string = parsed.get("datetime").unwrap().as_str();

                    println!("datetime_string {}", datetime_string.unwrap());


                    return (bytes.len(), Some(RTC::epoch_from_iso(datetime_string.unwrap())));


                }
            }


            (bytes.len(), None)

        }).await;

        if let Ok(got_time) = result_got_time {
            if let Some(epoch) = got_time {
                let mut guard = time_mutex.lock().await;

                guard.update_epoch(epoch);


                socket.close();

                //Put this thread to sleep when the time is found
                break;
            }
        }

        println!("Time not found");

        socket.close();

        Timer::after(Duration::from_secs(3)).await;

    }

    //If the time has been acquired, put the thread to sleep
    loop {
        Timer::after(Duration::from_secs(1)).await;
    }
}


///
/// Server takes clients, reads the message, sends a response then closes the connection
///
/// The message is sent to the main loop, the function is executed, and the return string is
/// sent back to this thread to send to the client.
///
#[embassy_executor::task]
async fn server_task(
    stack: Stack<'static>,
    msg: & 'static Channel<NoopRawMutex, Message, 1>,
    ret:  & 'static Channel<NoopRawMutex, String, 1>,

) -> ! {
    let mut rx_buf = [0u8; 3024];
    let mut tx_buf = [0u8; 3024];

    loop {
        let mut socket = TcpSocket::new(stack, &mut rx_buf, &mut tx_buf);

        socket.set_timeout(Some(Duration::from_secs(10)));

        if let Err(e) = socket.accept(8080).await {
            println!("Accept failed: {:?}", e);
            continue;
        }

        let mut buf = [0u8; 3024];

        if let Ok(n) = socket.read(& mut buf).await {
            if n != 0 {
                if let Ok(s) = core::str::from_utf8(&buf[..n]) {
                    if let Ok(message) = serde_json::from_str::<Message>(s) {
                        use embedded_io_async::Write;
                        println!("Message: {:?}", message);

                        //Send the message to the main thread
                        assert!(!msg.is_full());

                        msg.send(message).await;

                        let return_string = ret.receive().await;

                        let _ = socket.write_all(return_string.as_bytes()).await;

                        let _ = socket.flush().await;

                        let _ = socket.close();

                        //Purge remaining data and wait for eof
                        let mut drain = [0u8; 64];
                        let timeout = Instant::now();
                        loop {
                            if timeout.elapsed() > esp_hal::time::Duration::from_secs(3) {
                                break
                            }
                            match socket.read(&mut drain).await {
                                Ok(0) => break,      // peer closed -> done
                                Ok(_) => continue,   // ignore any extra bytes
                                Err(_) => break,     // error -> just exit
                            }
                        }


                        println!("client disconnected");
                    }
                }
            }
        }

        Timer::after(Duration::from_millis(5)).await;
    }
}



#[embassy_executor::task]
async fn led_task(
    led_state: & 'static Channel<NoopRawMutex, crate::rgb::State, 1>,
    red_channel: esp_hal::ledc::channel::Channel<'static, esp_hal::ledc::LowSpeed>,
    green_channel: esp_hal::ledc::channel::Channel<'static, esp_hal::ledc::LowSpeed>,
    blue_channel: esp_hal::ledc::channel::Channel<'static, esp_hal::ledc::LowSpeed>,
    initial_state: RGBState,
) -> ! {


    let mut rgb_led = RGBLED::new();

    rgb_led.set_state(initial_state);

    loop {


        if let Ok(state) = led_state.try_receive() {
            rgb_led.set_state(state);
        }

        rgb_led.update(|r, g, b| {
            let r = (r as u32 * 256) / 255;
            let g = (g as u32 * 256) / 255;
            let b = (b as u32 * 256) / 255;

            red_channel.set_duty_hw(256 - r);
            green_channel.set_duty_hw(256 - g);
            blue_channel.set_duty_hw(256 - b);
        });

        Timer::after(Duration::from_millis(5)).await;

    }

}

#[esp_hal_embassy::main]
async fn main(spawner: Spawner) {
    // generator version: 0.5.0
    esp_alloc::heap_allocator!(size: 64 * 1024);

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    /*unsafe {
        #[cfg(target_arch = "riscv32")]
        core::arch::asm!("unimp", options(noreturn));

        // Xtensa (ESP32-S2/S3). `ill` is an illegal-instruction trap.
        // If your toolchain doesn’t accept `ill`, use `.byte 0` repeatedly.
        #[cfg(target_arch = "xtensa")]
        core::arch::asm!("ill", options(noreturn));
        // Fallback for older xtensa assemblers:
        // core::arch::asm!(".byte 0, 0, 0", options(noreturn));
    }*/

    /* Setup LED pins */

    let red = Output::new(peripherals.GPIO10, Level::High, OutputConfig::default());
    let green = Output::new(peripherals.GPIO21, Level::High, OutputConfig::default());
    let blue = Output::new(peripherals.GPIO20, Level::High, OutputConfig::default());

    let mut myledc = Ledc::new(peripherals.LEDC);
    myledc.set_global_slow_clock(LSGlobalClkSource::APBClk);

    static LSTIMER1: StaticCell<esp_hal::ledc::timer::Timer<'_, esp_hal::ledc::LowSpeed>> = StaticCell::new();
    let lstimer1 = LSTIMER1.init(
        {
            let mut t = myledc.timer::<esp_hal::ledc::LowSpeed>(esp_hal::ledc::timer::Number::Timer1);
            t.configure(esp_hal::ledc::timer::config::Config {
                duty: esp_hal::ledc::timer::config::Duty::Duty8Bit,
                clock_source: esp_hal::ledc::timer::LSClockSource::APBClk,
                frequency: esp_hal::time::Rate::from_khz(24),
            }).unwrap();
            t
        }
    );

    let mut red_channel = myledc.channel(esp_hal::ledc::channel::Number::Channel0, red);
    red_channel.configure(esp_hal::ledc::channel::config::Config {
        timer: lstimer1,
        duty_pct: 10,
        pin_config: esp_hal::ledc::channel::config::PinConfig::PushPull,
    }).unwrap();

    let mut green_channel = myledc.channel(esp_hal::ledc::channel::Number::Channel1, green);
    green_channel.configure(esp_hal::ledc::channel::config::Config {
        timer: lstimer1,
        duty_pct: 10,
        pin_config: esp_hal::ledc::channel::config::PinConfig::PushPull,
    }).unwrap();

    let mut blue_channel = myledc.channel(esp_hal::ledc::channel::Number::Channel2, blue);
    blue_channel.configure(esp_hal::ledc::channel::config::Config {
        timer: lstimer1,
        duty_pct: 10,
        pin_config: esp_hal::ledc::channel::config::PinConfig::PushPull,
    }).unwrap();

    red_channel.set_duty(100).unwrap();
    green_channel.set_duty(0).unwrap();
    blue_channel.set_duty(0).unwrap();

    static LED_CHNL: StaticCell<Channel<NoopRawMutex, crate::rgb::State, 1>> = StaticCell::new();
    let led_channel: & 'static mut Channel<NoopRawMutex, crate::rgb::State, 1>  = LED_CHNL.init(Channel::new());

    spawner.spawn(led_task(led_channel, red_channel, blue_channel, green_channel, RGBState::OneFlash(Color { r: 0, g: 255, b:255 }, 100, 100))).unwrap();

    /* Set up button */

    let button_pin = esp_hal::gpio::Input::new(peripherals.GPIO8, InputConfig::default().with_pull(Pull::Up));

    let mut mode_button = crate::mode_button::Mode::new(button_pin);

    /* Set ip dht22 */
    let mut thermo = Thermometer::new(peripherals.GPIO9);

    thermo.get_temperature();

    /* Set up the sd storage interface */
    let store = storages::SdInterface::new(peripherals.SPI2, peripherals.GPIO4, peripherals.GPIO6, peripherals.GPIO5, peripherals.GPIO7);


    /* Set up the wifi and web server */

    led_channel.send(RGBState::OneFlash(Color { r: 0, g: 255, b: 255 }, 50, 50)).await;

    let rng = Rng::new(peripherals.RNG);

    let timer0 = SystemTimer::new(peripherals.SYSTIMER);
    esp_hal_embassy::init(timer0.alarm0);

    let timer1 = TimerGroup::new(peripherals.TIMG0);

    static WIFI_INIT: StaticCell<esp_wifi::EspWifiController<'_>> = StaticCell::new();
    let wifi_init = WIFI_INIT
        .init(esp_wifi::init(timer1.timer0, rng).expect("Failed to initialize WIFI/BLE controller"));



    let (mut wifi_controller, interfaces) = esp_wifi::wifi::new(wifi_init, peripherals.WIFI)
        .expect("Failed to initialize WIFI controller");


    wifi_controller
        .set_configuration(&esp_wifi::wifi::Configuration::Client(esp_wifi::wifi::ClientConfiguration {
            ssid: store.ssid().into(),
            password: store.password().into(),
            ..Default::default()
        }))
        .unwrap();


    wifi_controller.start().unwrap();

    wifi_controller.connect().unwrap();

    static RESOURCES: StaticCell<StackResources<4>> = StaticCell::new();
    let resources: & 'static mut StackResources<4>  = RESOURCES.init(StackResources::new());

    let (stack, runner) = embassy_net::new(
        interfaces.sta,
        embassy_net::Config::dhcpv4(Default::default()),
        resources,
        rng.clone().random() as u64,

    );

    // Spawn the network runner (must be running for sockets to work)
    spawner.spawn(net_task(runner)).unwrap();



    //Todo: add a timeout to this code so it doesn't block the code.
    //Using embassy select
    stack.wait_config_up().await;


    let ip = stack.config_v4().unwrap().address;
    println!("network up. IPv4: {}", ip);


    led_channel.send(RGBState::OneFlash(Color { r: 255, g: 0, b: 50 }, 50, 50)).await;

    static MSG_CHNL: StaticCell<Channel<NoopRawMutex, Message, 1>> = StaticCell::new();
    let message_channel: & 'static mut Channel<NoopRawMutex, Message, 1>  = MSG_CHNL.init(Channel::new());



    static RET_CHNL: StaticCell<Channel<NoopRawMutex, String, 1>> = StaticCell::new();
    let return_channel: & 'static mut Channel<NoopRawMutex, String, 1>  = RET_CHNL.init(Channel::new());

    /* Get internet time */
    static TIME_MUTEX: StaticCell<Mutex<NoopRawMutex, RTC>> = StaticCell::new();
    let time_mutex: & 'static mut Mutex<NoopRawMutex, RTC>  = TIME_MUTEX.init(Mutex::new(RTC::new()));

    spawner.spawn(world_time_task(stack, time_mutex)).unwrap();

    led_channel.send(RGBState::OneFlash(Color { r: 255, g: 0, b: 255 }, 200, 200)).await;

    //Block the main thread until the internet time is resolved
    loop {

        {
            let guard = time_mutex.lock().await;

            if guard.date_time().is_some() {
                break;
            }
        }

        Timer::after(Duration::from_millis(100)).await;

    }


    /* Initialise stepper */

    let mut motor = {

        let p1 = peripherals.GPIO0;
        let p2 = peripherals.GPIO2;
        let p3 = peripherals.GPIO1;
        let p4 = peripherals.GPIO3;

        Motor::new(p1, p2, p3, p4, & store)

    };

    spawner.spawn(server_task(stack, message_channel, return_channel)).unwrap();

    led_channel.send(RGBState::Solid(Color { r: 0, g: 255, b: 0 })).await;

    Timer::after(Duration::from_millis(500)).await;

    let mut fsm = FSM::new(& store);


    if store.is_locked() {

        fsm.force_change(fsm::State::Calibrate);

    }

    loop {


        match message_channel.try_receive() {
            Ok(message) => {
                println!("Loop message");
                let return_string = match message.function {
                    Function::CalibratePush(revolutions) => {
                        if fsm.state().is_calibrate() {
                            motor.calibrate_push(revolutions);
                            format!("Pushed {} revolutions", revolutions)
                        } else {
                            format!("Device must be in calibrate mode for calibrate push to work")
                        }
                    },
                    Function::CalibratePull(revolutions) => {
                        if fsm.state().is_calibrate() {
                            motor.calibrate_pull(revolutions);
                            format!("Pulled {} revolutions", revolutions)
                        } else {
                            format!("Device must be in calibrate mode for calibrate pull to work")
                        }
                    },
                    Function::Unlock => {
                        if fsm.state().is_calibrate() {
                            if store.is_locked() {
                                store.unlock();
                                format!("Lock unlocked")
                            } else {
                                format!("Already unlocked")
                            }
                        } else {
                            format!("Device must be in calibrate mode for unlock to work")
                        }
                    },
                    Function::GetLock => {
                        format!("Lock is {}", if store.is_locked() { "locked" } else { "unlocked" })
                    },
                    Function::GetPosition => {
                        format!("Position is at: {}", store.position())
                    },
                    Function::Zero => {
                        if fsm.state().is_calibrate() {
                            store.set_position(0);
                            format!("Position set to 0")
                        } else {
                            format!("Device must be in calibrate mode for zero to work")
                        }
                    },
                    Function::GetMax => {
                        format!("Max position is at: {}", store.max_position())
                    },
                    Function::SetMax(value) => {
                        if fsm.state().is_calibrate() {
                            motor.set_max_position(value);
                            format!("Set max position to {}", value)
                        } else {
                            format!("Device must be in calibrate mode for set max to work")
                        }
                    },
                    Function::GetThermostat => {
                        format!("Thermostat value is {}°C", store.thermostat())
                    },
                    Function::SetThermostat(_) => {
                        format!("Not Implemented")
                    },
                    Function::ReadTemperature => {
                        format!("Temperature is {}°C", thermo.get_temperature())
                    },
                    Function::GetMacAddress => {
                        format!("Not Implemented")
                    },
                    Function::SoftReset => {
                        esp_hal::rom::software_reset();
                    },
                    Function::SyncTime(epoch) => {
                        let mut guard = time_mutex.lock().await;

                        guard.update_epoch(epoch);

                        format!("Time updated")
                    },
                    Function::GetResetReason => {
                        format!("Reset reason: {}", esp_hal::rom::rtc_get_reset_reason(0))
                    },
                    Function::GetTime => {

                        let guard = time_mutex.lock().await;

                        if let Some(date_time) = guard.date_time() {
                            format!("Time is {:04}-{:02}-{:02}T{:02}:{:02}:{:02}",
                                   date_time.year(),
                                   date_time.month() as u32,
                                   date_time.day(),
                                   date_time.hour(),
                                   date_time.minute(),
                                   date_time.second(),
                            )
                        } else {
                            format!("Could not get time - rtc not synced")
                        }
                    },
                    Function::Descale => {
                        fsm.request_change(fsm::State::Descale);
                        format!("Requesting descale - NOTE: this might not work if the device is in safe, calibrate or warning mode")
                    },
                    Function::Calibrate => {
                        fsm.request_change(fsm::State::Calibrate);
                        format!("Requesting descale - NOTE: this might not work if the device is in safe, calibrate or warning mode")
                    },
                    Function::SafeMode => {
                        fsm.request_change(fsm::State::SafeMode);
                        format!("Requesting descale - NOTE: this might not work if the device is in safe, calibrate or warning mode")
                    },
                    Function::Cancel => {
                        fsm.request_change(fsm::State::Cancel);
                        format!("Cancelling mode (does nothing unless in warning, descale or boost mode)")
                    },
                    Function::ShortBoost => {
                        //todo: replace literal with store.short_duration
                        fsm.request_change(fsm::State::Boost(Instant::now(), esp_hal::time::Duration::from_secs(600)));
                        format!("Starting short boost")
                    },
                    Function::LongBoost => {
                        //todo: replace literal with store.long_duration
                        fsm.request_change(fsm::State::Boost(Instant::now(), esp_hal::time::Duration::from_secs(1800)));
                        format!("Starting long boost")
                    },

                };

                return_channel.send(return_string).await;
            },
            Err(_) => {

            }
        }


        mode_button.update();

        if let Some(action) = mode_button.released_action() {

            match action {
                mode_button::Action::End => {}
                mode_button::Action::Nothing => {}
                mode_button::Action::ShortBoost => {
                    //todo: replace literal with store.short_duration
                    fsm.request_change(fsm::State::Boost(Instant::now(), esp_hal::time::Duration::from_secs(600)));
                }
                mode_button::Action::LongBoost => {
                    //todo: replace literal with store.long_duration
                    fsm.request_change(fsm::State::Boost(Instant::now(), esp_hal::time::Duration::from_secs(1800)));
                }
                mode_button::Action::Cancel => {
                    fsm.request_change(fsm::State::Cancel);
                }
                mode_button::Action::Calibrate => {
                    fsm.request_change(fsm::State::Calibrate);
                }
                mode_button::Action::Safe => {
                    fsm.request_change(fsm::State::SafeMode);
                }
                mode_button::Action::Reset => {
                    esp_hal::rom::software_reset();
                }
                mode_button::Action::Retract => {
                    motor.calibrate_pull(7);
                }
                mode_button::Action::ZeroUnlock => {
                    store.set_position(0);
                    if store.is_locked() {
                        store.unlock();
                    }
                }
            }
        }

        if mode_button.mode_just_changed() {

            if let Some(action) = mode_button.current_action() {
                led_channel.send(RGBState::Solid(action.color())).await;
            } else {
                led_channel.send(RGBState::Off).await;
            }
        }

        if !mode_button.is_pressed() {
            let state_just_changed = fsm.just_changed();

            let current_time = {
                let guard = time_mutex.lock().await;

                guard.date_time().unwrap()
            };

            match fsm.state() {
                crate::fsm::State::Empty => {}
                crate::fsm::State::Start => {
                    fsm.change_from_schedule(current_time);

                    fsm.allow_request();
                }
                crate::fsm::State::Regulate => {

                    if state_just_changed || mode_button.just_released() {
                        led_channel.send(RGBState::Fade(Color { r: 255, g: 30, b: 0 }, 3000)).await;

                        Timer::after(Duration::from_millis(100)).await;

                    }

                    if state_just_changed {

                        mode_button.clear_actions();
                        mode_button.push_action(mode_button::Action::ShortBoost);
                        mode_button.push_action(mode_button::Action::LongBoost);
                        mode_button.push_action(mode_button::Action::Calibrate);
                        mode_button.push_action(mode_button::Action::Safe);
                        mode_button.push_action(mode_button::Action::Reset);

                    }

                    let actual = thermo.get_temperature();
                    let thermostat = store.thermostat();

                    if actual < (thermostat - 0.25) {
                        motor.open_valve();
                    }

                    if actual > (thermostat + 0.25) {
                        motor.close_valve();
                    }

                    fsm.change_from_schedule(current_time);

                    fsm.allow_request();
                }
                crate::fsm::State::Off => {

                    if state_just_changed || mode_button.just_released() {
                        led_channel.send(RGBState::Fade(Color { r: 0, g: 10, b: 255 }, 5000)).await;

                        Timer::after(Duration::from_millis(100)).await; //Give the led thread time

                    }

                    if state_just_changed {

                        motor.close_valve();

                        mode_button.clear_actions();
                        mode_button.push_action(mode_button::Action::ShortBoost);
                        mode_button.push_action(mode_button::Action::LongBoost);
                        mode_button.push_action(mode_button::Action::Calibrate);
                        mode_button.push_action(mode_button::Action::Safe);
                        mode_button.push_action(mode_button::Action::Reset);
                    }

                    fsm.change_from_schedule(current_time);

                    fsm.allow_request();
                }
                crate::fsm::State::Boost(time_started, duration) => {

                    if state_just_changed || mode_button.just_released() {
                        led_channel.send(RGBState::Fade(Color { r: 255, g: 10, b: 0 }, 400)).await;

                        Timer::after(Duration::from_millis(100)).await; //Give the led thread time

                    }

                    if state_just_changed {

                        motor.open_valve();


                        mode_button.clear_actions();
                        mode_button.push_action(mode_button::Action::Cancel);
                        mode_button.push_action(mode_button::Action::Calibrate);
                        mode_button.push_action(mode_button::Action::Safe);
                        mode_button.push_action(mode_button::Action::Reset);
                    }

                    if time_started.elapsed() > *duration {
                        fsm.request_change(fsm::State::Start);
                    }

                    fsm.allow_request();
                }
                crate::fsm::State::Calibrate => {

                    if state_just_changed || mode_button.just_released() {
                        led_channel.send(RGBState::Fade(Color { r: 255, g: 150, b: 0 }, 400)).await;

                        Timer::after(Duration::from_millis(100)).await; //Give the led thread time

                    }

                    if state_just_changed {

                        mode_button.clear_actions();
                        mode_button.push_action(mode_button::Action::Retract);
                        mode_button.push_action(mode_button::Action::ZeroUnlock);
                        mode_button.push_action(mode_button::Action::Safe);
                        mode_button.push_action(mode_button::Action::Reset);
                    }

                    //FSM may only leave calibrate mode for warnings
                    fsm.handle_request(|state| state.is_warning());
                }
                crate::fsm::State::SafeMode => {

                    if state_just_changed || mode_button.just_released() {
                        led_channel.send(RGBState::Fade(Color { r: 255, g: 255, b: 255 }, 3000)).await;

                        Timer::after(Duration::from_millis(100)).await; //Give the led thread time

                    }

                    if state_just_changed {

                        motor.open_valve();


                        mode_button.clear_actions();
                        mode_button.push_action(mode_button::Action::Reset);
                    }


                    //FSM may only leave safe mode for warnings
                    fsm.handle_request(|state| state.is_warning());
                }
                crate::fsm::State::Reset => {
                    esp_hal::rom::software_reset();
                }
                crate::fsm::State::Descale => {

                    if state_just_changed || mode_button.just_released() {

                        led_channel.send(RGBState::CrossFade { from: Color { r: 0, g: 255, b: 0 }, to: Color { r: 255, g: 255, b: 0 }, duration: 5000 }).await;

                        Timer::after(Duration::from_millis(100)).await; //Give the led thread time

                    }

                    if state_just_changed {
                        //Whatever position the valve is in, open and close it
                        motor.close_valve();
                        motor.open_valve();
                        motor.close_valve();


                        mode_button.clear_actions();
                        mode_button.push_action(mode_button::Action::Cancel);
                        mode_button.push_action(mode_button::Action::ShortBoost);
                        mode_button.push_action(mode_button::Action::LongBoost);
                        mode_button.push_action(mode_button::Action::Calibrate);
                        mode_button.push_action(mode_button::Action::Safe);
                        mode_button.push_action(mode_button::Action::Reset);
                    }

                    fsm.allow_request();
                }
                crate::fsm::State::Warning(code) => {

                    if state_just_changed || mode_button.just_released() {

                        led_channel.send(RGBState::FadeFlash {
                            color: Color { r: 255, g: 100, b: 0 },
                            fade_duration: 2000,
                            pause: 300,
                            flash_count: *code,
                            on_duration: 30,
                            off_duration: 270,
                            final_pause: 1000,
                        }).await;


                        Timer::after(Duration::from_millis(100)).await; //Give the led thread time
                    }

                    if state_just_changed {

                        mode_button.clear_actions();
                        mode_button.push_action(mode_button::Action::Cancel);

                    }


                    //The only way to move past a warning is with the special Ignore state
                    fsm.handle_request(|state| state.is_cancel());
                }
                crate::fsm::State::Cancel => {
                    //Ignore is a special state that is the only way to exit warnings, leave boost early
                    // and manually get out of descale.
                    fsm.force_change(fsm::State::Start);
                }
            }
        }

        /**/

        Timer::after(Duration::from_millis(1)).await;
    }


}

// some smoltcp boilerplate
fn timestamp() -> smoltcp::time::Instant {
    smoltcp::time::Instant::from_micros(
        esp_hal::time::Instant::now()
            .duration_since_epoch()
            .as_micros() as i64,
    )
}

pub fn create_interface(device: &mut esp_wifi::wifi::WifiDevice<'_>) -> smoltcp::iface::Interface {
    smoltcp::iface::Interface::new(
        smoltcp::iface::Config::new(smoltcp::wire::HardwareAddress::Ethernet(
            smoltcp::wire::EthernetAddress::from_bytes(&device.mac_address()),
        )),
        device,
        timestamp(),
    )
}