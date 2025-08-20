use esp_hal::rng::Rng;
use smoltcp::wire::IpAddress;
use esp_println::println;
use smoltcp::iface::{Context, Interface, SocketHandle, SocketSet};
use smoltcp::socket::tcp::Socket;
use esp_hal::time::Instant;
use smoltcp::wire::IpListenEndpoint;
use crate::commands::Message;

#[derive(PartialEq)]
enum State {
    Initialising,
    Restart,
    ConnectToTimeServer(Instant),
    SendTimeRequest,
    ReceiveTime,
    AwaitMessage,
    CloseClient,
}

struct Ephemeral(bool, Rng);

impl Ephemeral {
    pub fn new(rng: Rng) -> Self {
        Ephemeral(false, rng)
    }

    pub fn next_port(& mut self) -> u16 {
        let next = self.1.random();

        let next = if self.0 {
            (next % 5000 + 50000) as u16
        } else {
            (next % 5000 + 55000) as u16
        };

        self.0 = !self.0;

        next
    }

}

pub(crate) struct Network {
    state: State,
    queue: Option<State>,
    port_generator: Ephemeral,
}

impl Network {

    pub fn new(rng: Rng) -> Self {
        Self {
            state: State::Initialising,
            queue: None,
            port_generator: Ephemeral::new(rng),
        }

    }

    pub fn try_time_api(& mut self) {
        self.queue = Some(State::ConnectToTimeServer(Instant::now()));
    }

    pub fn update<'a, F: FnOnce(Message) -> & 'a str >(& mut self,
                                                       interface: & mut Interface,
                                                       socket_set: &mut SocketSet,
                                                       server_handle: SocketHandle,
                                                       time_handle: SocketHandle,
                                                       dhcp_handle: SocketHandle,
                                                       processor: F) {

        let dhcp_socket = socket_set.get_mut::<smoltcp::socket::dhcpv4::Socket<'_>>(dhcp_handle);
        if let Some(event) = dhcp_socket.poll() {
            match event {
                smoltcp::socket::dhcpv4::Event::Configured(config) => {
                    interface.update_ip_addrs(|addrs| {
                        println!("IP addr: {:?}", config.address);
                        addrs.clear();
                        addrs.push(smoltcp::wire::IpCidr::Ipv4(config.address)).unwrap();
                    });

                    if let Some(router) = config.router {
                        println!("Router: {:?}", router);
                        interface.routes_mut().add_default_ipv4_route(router).unwrap();
                    }

                    if self.state == State::Initialising {
                        self.state = State::Restart;
                        return
                    }
                }
                smoltcp::socket::dhcpv4::Event::Deconfigured => {

                }
            }
        }


        match self.state {
            State::Initialising => {
                //Do nothing while the server is resolving the dhcp ip
            },
            State::Restart => {
                println!("Restarting fsm");
                //self.state = State::ConnectToTimeServer;
                //self.state = State::AwaitMessage;
                self.state = State::AwaitMessage;
            },
            State::ConnectToTimeServer(instant) => {

                let remote_endpoint = smoltcp::wire::IpEndpoint::new(IpAddress::v4(213,188,196,246), 80);
                //let remote_endpoint = smoltcp::wire::IpEndpoint::new(IpAddress::v4(23,215,0,136), 80);

                let src = interface.ipv4_addr().unwrap().into();
                let ephemeral = self.port_generator.next_port();
                let local_endpoint = IpListenEndpoint {
                    addr: Some(src),
                    port: ephemeral, // ephemeral
                };

                use esp_println::println;

                let time_socket = socket_set.get_mut::<smoltcp::socket::tcp::Socket<'_>>(time_handle);

                if !time_socket.is_open() {

                    println!("remote is zero? {}", remote_endpoint.port);
                    println!("remote is unspecified? {}", remote_endpoint.addr.is_unspecified() );
                    println!("ipv4: {:?}", interface.ipv4_addr());
                    println!("routes: {:?}", interface.routes());


                    let mut cx = interface.context();         // take a fresh, mutable Context

                    time_socket.connect(& mut cx, remote_endpoint, local_endpoint).unwrap();

                    println!("connected");
                }

                // Wait until handshake completes
                if time_socket.state() == smoltcp::socket::tcp::State::Established {
                    self.state = State::SendTimeRequest;
                }

                if instant.elapsed().as_millis() > 3000 {
                    println!("Connection failed. try again later.");
                    self.state = State::Restart;
                }

            },
            State::SendTimeRequest => {
                let time_socket = socket_set.get_mut::<smoltcp::socket::tcp::Socket<'_>>(time_handle);
                println!("ddsfds");
                if time_socket.can_send() {
                    time_socket.send_slice(
                        b"GET /api/ip HTTP/1.1\r\n\
    Host: worldtimeapi.org\r\n\r\n",
                    ).unwrap();

                }
                self.state = State::ReceiveTime;
            },
            State::ReceiveTime => {
                let time_socket = socket_set.get_mut::<Socket<'_>>(time_handle);
                if time_socket.can_recv() {
                    time_socket.recv(|buf| {

                        let mut headers = [httparse::EMPTY_HEADER; 100];
                        let mut res = httparse::Response::new(&mut headers);

                        match res.parse(buf) {
                            Ok(httparse::Status::Complete(offset)) => {

                                let body = core::str::from_utf8(&buf[offset..]).unwrap();


                                let v: serde_json::Value = serde_json::from_str(body).unwrap();

                                println!("\n\n\ndatetime: {}\n\n\n", v["datetime"].as_str().unwrap());

                                (buf.len(), ())
                            }
                            Ok(httparse::Status::Partial) => {
                                (0, ())
                            }
                            Err(e) => {
                                (0, ())
                            }
                        }



                    }).unwrap();
                }

                // If the remote closed connection and no more to receive
                if !time_socket.may_recv() {
                    time_socket.close();
                    self.state = State::Restart;
                }


            },
            State::AwaitMessage => {
                let server_socket = socket_set.get_mut::<smoltcp::socket::tcp::Socket<'_>>(server_handle);
                if !server_socket.is_open() {
                    server_socket.listen(8080).unwrap(); // server
                }

                let mut return_string = None;

                if server_socket.can_recv() {
                    server_socket.recv(|buf| {
                        if let Ok(s) = core::str::from_utf8(buf) {
                            if let Ok(message) = serde_json::from_str::<Message>(s) {
                                return_string = Some(processor(message));
                            }
                        }
                        (buf.len(), ())
                    }).unwrap();
                }

                if let Some(return_string) = return_string {
                    server_socket.send_slice(return_string.as_bytes()).unwrap();
                    self.state = State::CloseClient;
                } else {
                    if let Some(state) = self.queue.take() {
                        println!("Change to scheduled state");
                        self.state = state;
                    }
                }


            }
            State::CloseClient => {
                let server_socket = socket_set.get_mut::<smoltcp::socket::tcp::Socket<'_>>(server_handle);
                server_socket.close();
                self.state = State::Restart;
            },
        }
    }



}
