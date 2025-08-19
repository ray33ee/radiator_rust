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

use crate::commands::{Function, Message};
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
use esp_wifi::wifi::{ClientConfiguration, Configuration};
use serde_json::to_string;
use motor::Motor;
use crate::thermo::Thermometer;
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

    let mut socket_set_entries: [SocketStorage; 3] = Default::default();
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

    let mut rx_storage = [0u8; 1024];
    let mut tx_storage = [0u8; 1024];

    let rx_buffer = smoltcp::storage::RingBuffer::<u8>::new::<&mut [u8]>(&mut rx_storage[..]);
    let tx_buffer = smoltcp::storage::RingBuffer::<u8>::new::<&mut [u8]>(&mut tx_storage[..]);

    let tcp_socket = smoltcp::socket::tcp::Socket::new(rx_buffer, tx_buffer);

    let tcp_handle = socket_set.add(tcp_socket);

    /* Initialise stepper */

    let mut motor = {

        let p1 = peripherals.GPIO0;
        let p2 = peripherals.GPIO2;
        let p3 = peripherals.GPIO1;
        let p4 = peripherals.GPIO3;

        Motor::new(p1, p2, p3, p4, & store)

    };

    let mut message_queue = heapless::Deque::<_, 10>::new();

    let mut end = false;

    loop {


        iface.poll(smoltcp::time::Instant::from_millis(Instant::now().duration_since_epoch().as_millis() as i64), &mut device, &mut socket_set);
        let dhcp_socket = socket_set.get_mut::<smoltcp::socket::dhcpv4::Socket>(dhcp_handle);
        if let Some(event) = dhcp_socket.poll() {
            match event {
                smoltcp::socket::dhcpv4::Event::Configured(config) => {
                    iface.update_ip_addrs(|addrs| {
                        println!("IP addr: {:?}", config.address);
                        addrs.push(smoltcp::wire::IpCidr::Ipv4(config.address)).unwrap();
                    });

                }
                smoltcp::socket::dhcpv4::Event::Deconfigured => {

                }
            }
        }



        let socket = socket_set.get_mut::<smoltcp::socket::tcp::Socket>(tcp_handle);



        if !socket.is_open() {
            socket.listen(8080).unwrap(); // server
        }

        if end {
            socket.close();
            end = false;
        }

        if socket.can_recv() {
            socket.recv(|buf| {
                if let Some(first_nul) = buf.iter().position(|&b| b == 0) {
                    let frame = &buf[..first_nul]; // bytes before the first NUL
                    if !frame.is_empty() {
                        if let Ok(s) = core::str::from_utf8(frame) {
                            if let Ok(message) = serde_json::from_str::<Message>(s) {
                                let _ = message_queue.push_back(message);
                            }
                        }
                    }
                    // Consume up to and including that first NUL. Anything after stays in the buffer.
                    (first_nul + 1, ())
                } else {
                    // No complete message yet; keep all bytes.
                    (0, ())
                }

            }).unwrap();

        }

        if !message_queue.is_empty() {
            let message = message_queue.pop_front().unwrap();

            match message.function {
                Function::Test => {
                    println!("Test function");
                    socket.send_slice(b"hello\0").unwrap();
                    end = true;
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
                Function::DebugOpen => {
                    motor.open_valve();
                },
                Function::DebugClose => {
                    motor.close_valve();
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