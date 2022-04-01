// Copyright (c) 2021 Marco Boneberger
// Licensed under the EUPL-1.2-or-later
#![allow(dead_code)]

extern crate libc;
extern crate nix;

use std::collections::HashMap;
use std::error::Error;
use std::fmt::Debug;
use std::io::{Read, Write};
use std::mem::size_of;
use std::net::TcpStream as StdTcpStream;
use std::net::{IpAddr, SocketAddr, ToSocketAddrs};
use std::os::unix::io::AsRawFd;
use std::str::FromStr;
use std::time::Duration;

use mio::net::{TcpStream, UdpSocket};
use mio::{Events, Interest, Poll, Token};

use nix::sys::socket::setsockopt;
use nix::sys::socket::sockopt::{KeepAlive, TcpKeepCount, TcpKeepIdle, TcpKeepInterval};

use serde::de::DeserializeOwned;
use serde::Serialize;

use crate::exception::{FrankaException, FrankaResult};
use crate::gripper;
use crate::gripper::types::CommandHeader;
use crate::robot::service_types::{
    LoadModelLibraryResponse, LoadModelLibraryStatus, RobotCommandEnum, RobotCommandHeader,
};

const CLIENT: Token = Token(1);

pub enum NetworkType {
    Robot,
    Gripper,
}

pub trait MessageCommand {
    fn get_command_message_id(&self) -> u32;
}

pub struct Network {
    network_type: NetworkType,
    tcp_socket: TcpStream,
    udp_socket: UdpSocket,
    udp_server_address: SocketAddr,
    udp_port: u16,
    udp_timeout: Duration,
    pub command_id: u32,
    pending_response: Vec<u8>,
    pending_response_offset: usize,
    pending_response_len: usize,
    pending_command_id: u32,
    received_responses: HashMap<u32, Vec<u8>>,
    poll_read: Poll,
    events: Events,
    poll_read_udp: Poll,
    events_udp: Events,
}

impl Network {
    pub fn new(
        network_type: NetworkType,
        franka_address: &str,
        franka_port: u16,
    ) -> Result<Network, Box<dyn Error>> {
        let address_str: String = format!("{}:{}", franka_address, franka_port);
        let sock_address = address_str.to_socket_addrs().unwrap().next().unwrap();
        let mut tcp_socket = TcpStream::from_std(StdTcpStream::connect(sock_address)?);
        let fd = tcp_socket.as_raw_fd();

        setsockopt(fd, KeepAlive, &true)?;
        setsockopt(fd, TcpKeepIdle, &1)?;
        setsockopt(fd, TcpKeepCount, &3)?;
        setsockopt(fd, TcpKeepInterval, &1)?;

        let udp_timeout = Duration::from_secs(1); // TODO: offer in constructor
        let ip_addr = IpAddr::from_str("0.0.0.0")?;
        let udp_server_address = SocketAddr::new(ip_addr, 0);

        let mut udp_socket = UdpSocket::bind(udp_server_address)?;
        let udp_port = udp_socket.local_addr()?.port();

        let command_id = 0;
        let pending_response: Vec<u8> = Vec::new();
        let pending_response_offset = 0;
        let pending_command_id = 0;
        let received_responses = HashMap::new();
        let poll_read = Poll::new()?;
        poll_read
            .registry()
            .register(&mut tcp_socket, CLIENT, Interest::READABLE)?;
        let poll_read_udp = Poll::new()?;
        poll_read_udp
            .registry()
            .register(&mut udp_socket, CLIENT, Interest::READABLE)?;
        let events = Events::with_capacity(128);
        let events_udp = Events::with_capacity(1);
        Ok(Network {
            network_type,
            tcp_socket,
            udp_socket,
            udp_server_address,
            udp_port,
            udp_timeout,
            command_id,
            pending_response,
            pending_response_offset,
            pending_command_id,
            pending_response_len: 0,
            received_responses,
            poll_read,
            events,
            poll_read_udp,
            events_udp,
        })
    }
    pub fn create_header_for_gripper(
        &mut self,
        command: gripper::types::Command,
        size: usize,
    ) -> CommandHeader {
        let header = gripper::types::CommandHeader::new(command, self.command_id, size as u32);
        self.command_id += 1;
        header
    }
    pub fn create_header_for_robot(
        &mut self,
        command: RobotCommandEnum,
        size: usize,
    ) -> RobotCommandHeader {
        let header = RobotCommandHeader::new(command, self.command_id, size as u32);
        self.command_id += 1;
        header
    }

    pub fn tcp_send_request<T: Serialize + DeserializeOwned + MessageCommand + Debug>(
        &mut self,
        request: T,
    ) -> u32 {
        let encoded_request = serialize(&request);
        self.tcp_socket.write_all(&encoded_request).unwrap();
        request.get_command_message_id()
    }
    /// Blocks until a Response message with the given command ID has been received and returns this
    /// response.
    ///
    /// # Arguments
    /// * `command_id` - Expected command ID of the Response.
    pub fn tcp_blocking_receive_response<T: DeserializeOwned + Debug + 'static>(
        &mut self,
        command_id: u32,
    ) -> T {
        let response_bytes = self.wait_for_response_to_arrive(&command_id);
        deserialize(&response_bytes)
    }
    /// Blocks until a LoadModelLibraryResponse message with the given command ID has been received
    /// and returns this LoadModelLibraryResponse.
    /// # Arguments
    /// * `command_id` - Expected command ID of the Response.
    /// * `buffer` -  variable-length data for the expected LoadModelLibraryResponse message (if
    /// any has been received) is copied into it.
    ///
    /// # Error
    /// * [`ModelException`](`crate::exception::FrankaException::ModelException`) - if the
    /// model could not be downloaded successfully.
    ///
    pub fn tcp_blocking_receive_load_library_response(
        &mut self,
        command_id: u32,
        buffer: &mut Vec<u8>,
    ) -> FrankaResult<LoadModelLibraryResponse> {
        let response_bytes = self.wait_for_response_to_arrive(&command_id);
        let lib_response: LoadModelLibraryResponse =
            deserialize(&response_bytes[0..size_of::<LoadModelLibraryResponse>()]);
        match lib_response.status {
            LoadModelLibraryStatus::Success => {}
            LoadModelLibraryStatus::Error => {
                return Err(FrankaException::ModelException {
                    message: "libfranka-rs: Server reports error when loading model library."
                        .to_string(),
                });
            }
        }
        assert_ne!(
            lib_response.header.size as usize,
            size_of::<LoadModelLibraryResponse>()
        );
        buffer.append(&mut Vec::from(
            &response_bytes[size_of::<LoadModelLibraryResponse>()..],
        ));
        Ok(lib_response)
    }
    /// Tries to receive a Response message with the given command ID (non-blocking).
    ///
    /// # Arguments
    /// * `command_id` - Expected command ID of the response
    /// * `handler` -  Callback to be invoked if the expected response has been received.
    ///
    /// # Return
    /// * true - if everything worked as expected
    /// * false - if the message could not be received
    ///
    /// # Error
    /// * [`FrankaException`](`crate::exception::FrankaException`) - if the handler returns an exception
    pub fn tcp_receive_response<T, F>(
        &mut self,
        command_id: u32,
        handler: F,
    ) -> Result<bool, FrankaException>
    where
        F: FnOnce(T) -> Result<(), FrankaException>,
        T: DeserializeOwned + Debug + 'static,
    {
        self.tcp_read_from_buffer(Duration::from_micros(0));
        let message = self.received_responses.get(&command_id);
        if message.is_none() {
            return Ok(false);
        }
        if message.unwrap().len() != size_of::<T>() {
            panic!("libfranka-rs: Incorrect TCP message size.");
        }
        let message: T = deserialize(&message.unwrap());
        let result = handler(message);
        match result {
            Ok(_) => {
                self.received_responses.remove(&command_id);
                Ok(true)
            }
            Err(e) => Err(e),
        }
    }

    fn wait_for_response_to_arrive(&mut self, command_id: &u32) -> Vec<u8> {
        let mut response_bytes: Option<Vec<u8>> = None;
        while response_bytes == None {
            {
                self.tcp_read_from_buffer(Duration::from_millis(10));
                response_bytes = self.received_responses.remove(&command_id);
            }
            std::thread::yield_now();
        }
        response_bytes.unwrap()
    }
    pub fn udp_receive<T: Debug + Serialize + DeserializeOwned + 'static>(&mut self) -> Option<T> {
        // TODO replace Vec<u8> with array when https://github.com/rust-lang/rust/issues/43408
        // is fixed
        let mut buffer: Vec<u8> = vec![0; size_of::<T>()];
        let available_bytes = self.udp_socket.peek(&mut buffer).ok()?;
        if available_bytes >= size_of::<T>() {
            let object: Option<T> = match self.udp_blocking_receive() {
                Ok(o) => Some(o),
                Err(_) => None,
            };
            return object;
        }
        None
    }
    pub fn udp_blocking_receive<T: Debug + Serialize + DeserializeOwned + 'static>(
        &mut self,
    ) -> FrankaResult<T> {
        self.poll_read_udp
            .poll(&mut self.events_udp, Some(self.udp_timeout))
            .unwrap();
        for event in self.events_udp.iter() {
            match event.token() {
                CLIENT => {
                    if event.is_readable() {
                        let mut buffer: Vec<u8> = vec![0; size_of::<T>()];
                        let read_bytes_and_address = self.udp_socket.recv_from(&mut buffer);
                        while self.udp_socket.recv_from(&mut buffer).is_ok() {}
                        let read_bytes = match read_bytes_and_address {
                            Ok(res) => {
                                self.udp_server_address = res.1;
                                res.0
                            }
                            Err(e) => {
                                return Err(FrankaException::NetworkException {
                                    message: e.to_string(),
                                });
                            }
                        };
                        if read_bytes != size_of::<T>() {
                            return Err(FrankaException::NetworkException { message: format!("UDP object could not be received: object has {} bytes but it should have {} bytes", read_bytes, size_of::<T>()) });
                        }
                        return Ok(deserialize(&buffer));
                    }
                }
                _ => unreachable!(),
            }
        }
        Err(FrankaException::NetworkException {
            message: "libfranka-rs: UDP receive: timeout".to_string(),
        })
    }
    pub fn udp_send<T: Debug + Serialize + DeserializeOwned>(
        &mut self,
        data: &T,
    ) -> FrankaResult<()> {
        let bytes_send = self
            .udp_socket
            .send_to(&serialize(data), self.udp_server_address)
            .map_err(|e| FrankaException::NetworkException {
                message: e.to_string(),
            })?;
        if bytes_send != size_of::<T>() {
            return Err(FrankaException::NetworkException {
                message: "libfranka-rs: UDP object could not be send".to_string(),
            });
        }
        Ok(())
    }

    fn tcp_read_from_buffer(&mut self, timeout: Duration) {
        self.poll_read
            .poll(&mut self.events, Some(timeout))
            .unwrap();
        for event in self.events.iter() {
            match event.token() {
                CLIENT => {
                    if event.is_readable() {
                        let mut buffer = [0_u8; 70000];
                        let available_bytes = self.tcp_socket.peek(&mut buffer);
                        let available_bytes = match available_bytes {
                            Ok(a) => a,
                            Err(e) => {
                                eprintln!("{}", e.to_string());
                                return;
                            }
                        };

                        if self.pending_response.is_empty() {
                            let header_mem_size = match self.network_type {
                                NetworkType::Gripper => size_of::<CommandHeader>(),
                                NetworkType::Robot => size_of::<RobotCommandHeader>(),
                            };
                            if available_bytes >= header_mem_size {
                                let mut header_bytes: Vec<u8> = vec![0; header_mem_size];
                                self.tcp_socket.read_exact(&mut header_bytes).unwrap();
                                self.pending_response.append(&mut header_bytes.clone());
                                self.pending_response_offset = header_mem_size as usize;
                                match self.network_type {
                                    NetworkType::Gripper => {
                                        let header: CommandHeader = deserialize(&header_bytes);
                                        self.pending_response_len = header.size as usize;
                                        self.pending_command_id = header.command_id;
                                    }
                                    NetworkType::Robot => {
                                        let header: RobotCommandHeader = deserialize(&header_bytes);
                                        self.pending_response_len = header.size as usize;
                                        self.pending_command_id = header.command_id;
                                    }
                                };
                            }
                        }
                        if !self.pending_response.is_empty() && available_bytes > 0 {
                            let number_of_bytes_to_read = usize::min(
                                available_bytes,
                                self.pending_response_len - self.pending_response_offset as usize,
                            );
                            let mut response_buffer: Vec<u8> = vec![0; number_of_bytes_to_read];
                            self.tcp_socket.read_exact(&mut response_buffer).unwrap();
                            self.pending_response.append(&mut response_buffer);
                            self.pending_response_offset += number_of_bytes_to_read;
                            if self.pending_response_offset == self.pending_response_len as usize {
                                self.received_responses
                                    .insert(self.pending_command_id, self.pending_response.clone());
                                self.pending_response.clear();
                                self.pending_response_offset = 0;
                                self.pending_command_id = 0;
                                self.pending_response_len = 0;
                            }
                        }
                    }
                    if event.is_writable() {
                        eprintln!("There should not be any writable events")
                    }
                }
                _ => unreachable!(),
            }
        }
    }
    pub fn get_udp_port(&self) -> u16 {
        self.udp_port
    }
}

fn serialize<T: Serialize>(s: &T) -> Vec<u8> {
    bincode::serialize(s).unwrap()
}

fn deserialize<T: Debug + DeserializeOwned + 'static>(encoded: &[u8]) -> T {
    bincode::deserialize(&encoded).unwrap()
}

#[cfg(test)]
mod tests {
    use crate::network::{deserialize, serialize};
    use crate::robot::types::RobotStateIntern;

    #[test]
    fn can_serialize_and_deserialize() {
        let state = RobotStateIntern::dummy();
        let state2: RobotStateIntern = deserialize(&serialize(&state));
        assert_eq!(state, state2);
    }
}
