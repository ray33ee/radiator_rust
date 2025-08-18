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
mod server;
mod commands;
mod motor;

use crate::commands::Function;
use server::Server;
use esp_hal::{
    clock::CpuClock,
    main,
    timer::timg::TimerGroup,
    delay::Delay,
    rng::Rng,
    time::Instant,
    gpio::{
        Output,
        Level,
        OutputConfig,
    },
};
use smoltcp::{
    iface::{
        SocketStorage,
        SocketSet,
        Interface,
    },
    wire::{
        DhcpOption,
        IpAddress
    },
    socket::Socket,
};
use esp_println::{print, println};
use esp_backtrace as _;
use blocking_network_stack::{Stack, ipv4::Ipv4Addr};
use esp_wifi::wifi::{ClientConfiguration, Configuration};
use serde_json::to_string;
use motor::Motor;


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

    /* Set up the sd storage interface */
    let mut store = storage::SdInterface::new(peripherals.SPI2, peripherals.GPIO4, peripherals.GPIO6, peripherals.GPIO5, peripherals.GPIO7);

    if store.is_locked() {
        println!("Motor is locked!");
    }

    /* Set up the wifi and web server */

    let mut rng = Rng::new(peripherals.RNG);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    let esp_wifi_ctrl = esp_wifi::init(timg0.timer0, rng.clone()).unwrap();

    let (mut controller, interfaces) =
        esp_wifi::wifi::new(&esp_wifi_ctrl, peripherals.WIFI).unwrap();

    let mut device = interfaces.sta;
    let iface = create_interface(&mut device);

    let mut socket_set_entries: [SocketStorage; 3] = Default::default();
    let mut socket_set = SocketSet::new(&mut socket_set_entries[..]);
    let mut dhcp_socket = smoltcp::socket::dhcpv4::Socket::new();
    // we can set a hostname here (or add other DHCP options)
    dhcp_socket.set_outgoing_options(&[DhcpOption {
        kind: 12,
        data: b"esp-wifi",
    }]);
    socket_set.add(dhcp_socket);

    let now = || Instant::now().duration_since_epoch().as_millis();
    let stack = Stack::new(iface, device, socket_set, now, rng.random());

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
    println!("{:?}", controller.is_connected());

    // wait for getting an ip address
    println!("Wait to get an ip address");
    loop {
        stack.work();

        if stack.is_iface_up() {
            println!("got ip {:?}", stack.get_ip_info());
            break;
        }
    }

    println!("Start busy loop on main");

    let mut rx_buffer = [0u8; 1536];
    let mut tx_buffer = [0u8; 1536];
    let mut socket = stack.get_socket(&mut rx_buffer, &mut tx_buffer);

    let mut server = Server::new(socket);

    server.start();

    /* Initialise stepper */

    let mut motor = {

        let p1 = peripherals.GPIO0;
        let p2 = peripherals.GPIO2;
        let p3 = peripherals.GPIO1;
        let p4 = peripherals.GPIO3;

        Motor::new(p1, p2, p3, p4, & store)

    };

    loop {
        stack.work();

        if let Some(message) = server.work() {
            match message.function {
                Function::Test => {
                    println!("Test function");
                },
                Function::CalibratePush(revolutions) => {
                    motor.calibrate_push(revolutions);
                },
                Function::CalibratePull(revolutions) => {
                    motor.calibrate_pull(revolutions);
                },
                Function::Unlock => {
                    store.unlock();

                },
                Function::DebugSetPosition(pos) => {
                    store.set_position(pos);
                },
                Function::DebugGetPosition => {
                    println!("Get Pos: {}", store.position());
                },
            }
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

pub fn create_interface(device: &mut esp_wifi::wifi::WifiDevice) -> smoltcp::iface::Interface {
    // users could create multiple instances but since they only have one WifiDevice
    // they probably can't do anything bad with that
    smoltcp::iface::Interface::new(
        smoltcp::iface::Config::new(smoltcp::wire::HardwareAddress::Ethernet(
            smoltcp::wire::EthernetAddress::from_bytes(&device.mac_address()),
        )),
        device,
        timestamp(),
    )
}