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

use esp_hal::ledc::LSGlobalClkSource;
use esp_hal::ledc::Ledc;
use esp_hal::gpio::OutputConfig;
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
use esp_backtrace as _;
use motor::Motor;
use crate::thermo::Thermometer;
use embassy_executor::Spawner;
use crate::rtc::RTC;
use embassy_sync::channel::Channel;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use smoltcp::wire::IpEndpoint;
use esp_hal::ledc::timer::TimerIFace;
use esp_hal::ledc::channel::ChannelIFace;
use crate::rgb::{RGBLED, Color};
use esp_hal::ledc::channel::ChannelHW;

// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();



#[embassy_executor::task]
async fn net_task(mut runner: embassy_net::Runner<'static, WifiDevice<'static>>) -> ! {
    runner.run().await;
}


#[embassy_executor::task]
async fn world_time_task(stack: Stack<'static>, time: & 'static Channel<NoopRawMutex, (), 1>,) -> ! {

    let mut rx_buf = [0u8; 2024];
    let mut tx_buf = [0u8; 2024];

    loop {

        time.receive().await;

        let mut socket = TcpSocket::new(stack, &mut rx_buf, &mut tx_buf);

        let time_api_addr = stack.dns_query("worldtimeapi.org", smoltcp::wire::DnsQueryType::A).await.unwrap()[0];

        let time_endpoint = IpEndpoint::new(time_api_addr, 80);

        socket.connect(time_endpoint).await.unwrap();

        socket.write(b"GET /api/ip HTTP/1.1\r\n\
Host: worldtimeapi.org\r\n\
User-Agent: esp32-rust\r\n\
Connection: close\r\n\
\r\n").await.unwrap();

        let _ = socket.read_with(|bytes| {

            if let Some(body_begin) = bytes.windows(4).position(|window| window == b"\r\n\r\n") {
                if let Some(body_end) = &bytes[body_begin+4..].iter().position(|&b| b == b'}') {
                    let json = core::str::from_utf8(&bytes[body_begin+4..*body_end + body_begin+5]).unwrap();

                    let parsed: serde_json::Value = serde_json::from_str(json).unwrap();

                    let datetime_string = parsed.get("datetime").unwrap().as_str();

                    println!("datetime_string {}", datetime_string.unwrap());

                }
            }


            (bytes.len(), ())

        }).await;


        socket.close();

    }
}


///
/// Server takes clients, reads the message, sends a response then closes the connection
///
/// The message is sent to the main loop, the function is exeuted, and the return string is
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
    blue_channel: esp_hal::ledc::channel::Channel<'static, esp_hal::ledc::LowSpeed>) -> ! {


    let mut rgb_led = RGBLED::new();

    rgb_led.set_state(crate::rgb::State::Rainbow(3000));

    loop {


        if let Ok(state) = led_state.try_receive() {
            println!("Received state");
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

    /* Setup LED pins */

    let red = Output::new(peripherals.GPIO10, Level::High, OutputConfig::default());
    let green = Output::new(peripherals.GPIO20, Level::High, OutputConfig::default());
    let blue = Output::new(peripherals.GPIO21, Level::High, OutputConfig::default());

    let mut myledc = Ledc::new(peripherals.LEDC);
    myledc.set_global_slow_clock(LSGlobalClkSource::APBClk);

    static LSTIMER1: StaticCell<esp_hal::ledc::timer::Timer<esp_hal::ledc::LowSpeed>> = StaticCell::new();
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


    static LED_CHNL: StaticCell<Channel<NoopRawMutex, crate::rgb::State, 1>> = StaticCell::new();
    let led_channel: & 'static mut Channel<NoopRawMutex, crate::rgb::State, 1>  = LED_CHNL.init(Channel::new());

    spawner.spawn(led_task(led_channel, red_channel, blue_channel, green_channel)).unwrap();

    /* Set ip dht22 */
    let mut thermo = Thermometer::new(peripherals.GPIO9);

    thermo.get_temperature();

    /* Set up RTC counter and NTP */
    let mut rtc = RTC::new();

    /* Set up the sd storage interface */
    let store = storages::SdInterface::new(peripherals.SPI2, peripherals.GPIO4, peripherals.GPIO6, peripherals.GPIO5, peripherals.GPIO7);

    if store.is_locked() {
        println!("Motor is locked!");
    }

    /* Set up the wifi and web server */
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


    //Todo: add a timeout to this code so it doesn't block the code
    stack.wait_config_up().await;

    let ip = stack.config_v4().unwrap().address;
    println!("network up. IPv4: {}", ip);



    static MSG_CHNL: StaticCell<Channel<NoopRawMutex, Message, 1>> = StaticCell::new();
    let message_channel: & 'static mut Channel<NoopRawMutex, Message, 1>  = MSG_CHNL.init(Channel::new());



    static RET_CHNL: StaticCell<Channel<NoopRawMutex, String, 1>> = StaticCell::new();
    let return_channel: & 'static mut Channel<NoopRawMutex, String, 1>  = RET_CHNL.init(Channel::new());

    static TIME_CHNL: StaticCell<Channel<NoopRawMutex, (), 1>> = StaticCell::new();
    let time_channel: & 'static mut Channel<NoopRawMutex, (), 1>  = TIME_CHNL.init(Channel::new());


    /* Initialise stepper */

    let mut motor = {

        let p1 = peripherals.GPIO0;
        let p2 = peripherals.GPIO2;
        let p3 = peripherals.GPIO1;
        let p4 = peripherals.GPIO3;

        Motor::new(p1, p2, p3, p4, & store)

    };

    spawner.spawn(server_task(stack, message_channel, return_channel)).unwrap();

    spawner.spawn(world_time_task(stack, time_channel)).unwrap();

    led_channel.send(crate::rgb::State::Fade(crate::rgb::Color { r: 255, g: 255, b: 0 }, 1000)).await;

    loop {



        match message_channel.try_receive() {
            Ok(message) => {
                println!("Loop message");
                let return_string = match message.function {
                    Function::CalibratePush(revolutions) => {
                        motor.calibrate_push(revolutions);
                        format!("Pushed {} revolutions", revolutions)
                    },
                    Function::CalibratePull(revolutions) => {
                        motor.calibrate_pull(revolutions);
                        format!("Pulled {} revolutions", revolutions)
                    },
                    Function::Unlock => {
                        if store.is_locked() {
                            store.unlock();
                            format!("Lock unlocked")
                        } else {
                            format!("Already unlocked")
                        }
                    },
                    Function::DebugSetPosition(pos) => {
                        store.set_position(pos);
                        format!("Position set at {}", pos)
                    },
                    Function::DebugGetPosition => {
                        format!("Position is at: {}", store.position())
                    },
                    Function::DebugOpen => {
                        motor.open_valve();
                        format!("Valve opened")
                    },
                    Function::DebugClose => {
                        motor.close_valve();
                        format!("Valve closed")
                    },
                    Function::GetLock => {
                        format!("Lock is {}", if store.is_locked() { "locked" } else { "unlocked" })
                    },
                    Function::GetPosition => {
                        format!("Position is at: {}", store.position())
                    },
                    Function::Zero => {
                        store.set_position(0);
                        format!("Position set to 0")
                    },
                    Function::GetMax => {
                        format!("Max position is at: {}", store.max_position())
                    },
                    Function::SetMax(value) => {
                        motor.set_max_position(value);
                        format!("Set max position to {}", value)
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
                        rtc.update_epoch(epoch);
                        format!("Time updated")
                    },
                    Function::GetResetReason => {
                        format!("Reset reason: {}", esp_hal::rom::rtc_get_reset_reason(0))
                    },
                    Function::GetTime => {
                        if let Some(date_time) = rtc.date_time() {
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
                    Function::DebugTimeAPI => {
                        time_channel.send(()).await;
                        format!("Trying to get time api")
                    },
                };

                return_channel.send(return_string).await;
            },
            Err(_) => {

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