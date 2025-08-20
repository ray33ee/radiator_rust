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

mod storage;
mod commands;
mod motor;
mod thermo;
mod rtc;
mod network;

use crate::commands::{Function, Message};
use esp_hal::{
    clock::CpuClock,
    main,
    timer::timg::TimerGroup,
    delay::Delay,
    rng::Rng,
    time::Instant,
};
use smoltcp::{
    iface::{
        SocketStorage,
        SocketSet,
    },
    wire::{
        DhcpOption,
    },
};
use esp_println::println;
use esp_backtrace as _;
use esp_wifi::wifi::{ClientConfiguration, Configuration};
use motor::Motor;
use crate::thermo::Thermometer;
use core::fmt::Write;
use core::ops::Deref;
use crate::network::Network;
use crate::rtc::RTC;


/*#[panic_handler]
fn panic(_: &core::panic::PanicInfo<'_>) -> ! {
    loop {}
}*/

// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();

#[main]
fn main() -> ! {
    // generator version: 0.5.0
    esp_alloc::heap_allocator!(size: 64 * 1024);

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    /* Set ip dht22 */
    let mut thermo = Thermometer::new(peripherals.GPIO9);

    thermo.get_temperature();

    /* Set up RTC counter and NTP */
    let mut rtc = RTC::new();

    /* Set up the sd storage interface */
    let store = storage::SdInterface::new(peripherals.SPI2, peripherals.GPIO4, peripherals.GPIO6, peripherals.GPIO5, peripherals.GPIO7);

    if store.is_locked() {
        println!("Motor is locked!");
    }

    /* Set up the wifi and web server */
    let rng = Rng::new(peripherals.RNG);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    let esp_wifi_ctrl = esp_wifi::init(timg0.timer0, rng.clone()).unwrap();

    let (mut controller, interfaces) =
        esp_wifi::wifi::new(&esp_wifi_ctrl, peripherals.WIFI).unwrap();

    let mut device = interfaces.sta;
    let mut iface = create_interface(&mut device);

    let mut socket_set_entries: [SocketStorage<'_>; 4] = Default::default();
    let mut socket_set = SocketSet::new(&mut socket_set_entries[..]);
    let mut dhcp_socket = smoltcp::socket::dhcpv4::Socket::new();
    // we can set a hostname here (or add other DHCP options)
    dhcp_socket.set_outgoing_options(&[DhcpOption {
        kind: 12,
        data: b"esp-wifi",
    }]);
    let dhcp_handle = socket_set.add(dhcp_socket);

    controller
        .set_power_saving(esp_wifi::config::PowerSaveMode::None)
        .unwrap();

    let client_config = Configuration::Client(ClientConfiguration {
        ssid: store.ssid().into(),
        password: store.password().into(),
        ..Default::default()
    });
    let res = controller.set_configuration(&client_config);
    println!("wifi_set_configuration returned {:?}", res);

    controller.start().unwrap();
    println!("is wifi started: {:?}", controller.is_started());

    println!("Start Wifi Scan");
    let res = controller.scan_n(10).unwrap();
    for ap in res {
        println!("{:?}", ap);
    }

    println!("{:?}", controller.capabilities());
    println!("wifi_connect {:?}", controller.connect());

    // wait to get connected
    println!("Wait to get connected");
    loop {
        match controller.is_connected() {
            Ok(true) => break,
            Ok(false) => {}
            Err(err) => {
                println!("{:?}", err);
                loop {}
            }
        }
    }
    println!("Connected? {:?}", controller.is_connected());

    let mut server_rx_storage = [0u8; 1024];
    let mut server_tx_storage = [0u8; 1024];

    let server_rx_buffer = smoltcp::storage::RingBuffer::<u8>::new::<&mut [u8]>(&mut server_rx_storage[..]);
    let server_tx_buffer = smoltcp::storage::RingBuffer::<u8>::new::<&mut [u8]>(&mut server_tx_storage[..]);

    let server_socket = smoltcp::socket::tcp::Socket::new(server_rx_buffer, server_tx_buffer);

    let server_handle = socket_set.add(server_socket);

    let mut time_rx_storage = [0u8; 3024];
    let mut time_tx_storage = [0u8; 3024];

    let time_rx_buffer = smoltcp::storage::RingBuffer::<u8>::new::<&mut [u8]>(&mut time_rx_storage[..]);
    let time_tx_buffer = smoltcp::storage::RingBuffer::<u8>::new::<&mut [u8]>(&mut time_tx_storage[..]);

    let time_socket = smoltcp::socket::tcp::Socket::new(time_rx_buffer, time_tx_buffer);

    let time_handle = socket_set.add(time_socket);



    /* Initialise stepper */

    let mut motor = {

        let p1 = peripherals.GPIO0;
        let p2 = peripherals.GPIO2;
        let p3 = peripherals.GPIO1;
        let p4 = peripherals.GPIO3;

        Motor::new(p1, p2, p3, p4, & store)

    };

    //let mut message = heapless::Deque::<_, 10>::new();


    let mut must_close = false;

    let mut st = heapless::String::<1000>::new();

    let get_time = false;

    let mut nal = Network::new(rng.clone());

    loop {


        iface.poll(smoltcp::time::Instant::from_micros(Instant::now().duration_since_epoch().as_micros() as i64), &mut device, &mut socket_set);



        let mut do_thing = false;

        nal.update(& mut iface, & mut socket_set, server_handle,time_handle, dhcp_handle, |message| {

            st.clear();

            match message.function {
                Function::CalibratePush(revolutions) => {
                    motor.calibrate_push(revolutions);
                    write!(&mut st, "Pushed {} revolutions", revolutions).unwrap();
                },
                Function::CalibratePull(revolutions) => {
                    motor.calibrate_pull(revolutions);
                    write!(&mut st, "Pulled {} revolutions", revolutions).unwrap();
                },
                Function::Unlock => {
                    if store.is_locked() {
                        store.unlock();
                        write!(&mut st, "Lock unlocked").unwrap();
                    } else {
                        write!(&mut st, "Already unlocked").unwrap();
                    }
                },
                Function::DebugSetPosition(pos) => {
                    store.set_position(pos);
                    write!(&mut st, "Position set at {}", pos).unwrap();
                },
                Function::DebugGetPosition => {
                    write!(&mut st, "Position is at: {}", store.position()).unwrap();
                },
                Function::DebugOpen => {
                    motor.open_valve();
                    write!(&mut st, "Valve opened").unwrap();
                },
                Function::DebugClose => {
                    motor.close_valve();
                    write!(&mut st, "Valve closed").unwrap();
                },
                Function::GetLock => {
                    write!(&mut st, "Lock is {}", if store.is_locked() { "locked" } else { "unlocked" }).unwrap();
                },
                Function::GetPosition => {
                    write!(&mut st, "Position is at: {}", store.position()).unwrap();
                },
                Function::Zero => {
                    store.set_position(0);
                    write!(&mut st, "Position set to 0").unwrap();
                },
                Function::GetMax => {
                    write!(&mut st, "Max position is at: {}", store.max_position()).unwrap();
                },
                Function::SetMax(value) => {
                    motor.set_max_position(value);
                    write!(&mut st, "Set max position to {}", value).unwrap();
                },
                Function::GetThermostat => {
                    write!(&mut st, "Thermostat value is {}°C", store.thermostat()).unwrap();
                },
                Function::SetThermostat(_) => {
                    write!(&mut st, "Not Implemented").unwrap();
                },
                Function::ReadTemperature => {
                    write!(&mut st, "Temperature is {}°C", thermo.get_temperature()).unwrap();
                },
                Function::GetMacAddress => {
                    write!(&mut st, "Not Implemented").unwrap();
                },
                Function::SoftReset => {
                    esp_hal::rom::software_reset();
                },
                Function::SyncTime(epoch) => {
                    rtc.update_epoch(epoch);
                    write!(&mut st, "Time updated").unwrap();
                },
                Function::GetResetReason => {
                    write!(&mut st, "Reset reason: {}", esp_hal::rom::rtc_get_reset_reason(0)).unwrap();
                },
                Function::GetTime => {
                    if let Some(date_time) = rtc.date_time() {
                        write!(&mut st, "Time is {:04}-{:02}-{:02}T{:02}:{:02}:{:02}",
                               date_time.year(),
                               date_time.month() as u32,
                               date_time.day(),
                               date_time.hour(),
                               date_time.minute(),
                               date_time.second(),
                        ).unwrap();
                    } else {
                        write!(&mut st, "Could not get time - rtc not synced").unwrap();
                    }
                },
                Function::DebugTimeAPI => {
                    do_thing = true;
                    write!(&mut st, "Trying to get time api").unwrap();
                },
            }

            st.deref()
        });

        if do_thing {
            println!("Do thing");
            nal.try_time_api();
            do_thing = false;
        }


        Delay::new().delay_millis(5);
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