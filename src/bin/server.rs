
use esp_hal::{
    time::{Instant, Duration},
};
use esp_wifi::{
    EspWifiController,
    wifi::{
        ClientConfiguration,
        Configuration
    },
};
use smoltcp::{
    iface::{
        SocketStorage,
        SocketSet,
        Interface,
    },
    wire::{DhcpOption, IpAddress},
    socket::Socket,
};
use blocking_network_stack::{Stack, ipv4::Ipv4Addr};
use embedded_io::Read;
use esp_println::println;
use esp_hal::peripherals::WIFI;
use crate::commands::Message;

pub struct Server<'a> {
    socket: blocking_network_stack::Socket<'a, 'a, esp_wifi::wifi::WifiDevice<'a>>,
    buffer: [u8; 2048],
    pos: usize,
    deadline: Option<Instant>,
}

impl<'a> Server<'a> {
    pub fn new(socket: blocking_network_stack::Socket<'a, 'a, esp_wifi::wifi::WifiDevice<'a>>) -> Self {
        Self {
            socket,
            buffer: [0; 2048],
            pos: 0,
            deadline: None,
        }
    }

    pub fn start(&mut self) {
        self.socket.listen(8080).unwrap();
    }

    pub fn work(& mut self) -> Option<Message> {
        self.socket.work();

        // If socket not open, start listening again
        if !self.socket.is_open() {
            self.socket.listen(8080).unwrap();
            self.pos = 0;
            self.deadline = None;
        }

        // If connected, handle input one tick at a time
        if self.socket.is_connected() {
            if self.deadline.is_none() {
                // first tick of this connection
                self.deadline = Some(Instant::now() + Duration::from_secs(20));
                self.pos = 0;
            }

            // Try to read a chunk
            if let Ok(len) = self.socket.read(&mut self.buffer[self.pos..]) {
                if len > 0 {
                    self.pos += len;
                    if self.buffer[self.pos-1] == b'\0' {

                        let to_print = unsafe { core::str::from_utf8_unchecked(&self.buffer[..self.pos-1]) };


                        let message = serde_json::from_str::<Message>(to_print).unwrap();


                        self.socket.close();
                        self.pos = 0;
                        self.deadline = None;

                        return Some(message);
                    }
                }
            }

            // Check for timeout
            if let Some(d) = self.deadline {
                if Instant::now() > d {
                    println!("Timeout");
                    self.socket.close();
                    self.pos = 0;
                    self.deadline = None;
                }
            }
        }
        None
    }

}
