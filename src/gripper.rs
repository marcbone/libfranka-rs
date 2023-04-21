// Copyright (c) 2021 Marco Boneberger
// Licensed under the EUPL-1.2-or-later

//!  Contains the franka::Gripper type.

use std::mem::size_of;

use crate::exception::{create_command_exception, FrankaException, FrankaResult};
use crate::gripper::gripper_state::GripperState;
use crate::gripper::types::{
    ConnectRequest, ConnectRequestWithHeader, ConnectResponse, GraspRequest,
    GraspRequestWithHeader, GripperCommandEnum, GripperCommandHeader, GripperStateIntern,
    MoveRequest, MoveRequestWithHeader, Status, COMMAND_PORT, GRIPPER_VERSION,
};
use crate::network::{DeviceData, Network};

pub mod gripper_state;
pub(crate) mod types;

/// Maintains a network connection to the gripper, provides the current gripper state,
/// and allows the execution of commands.
pub struct Gripper {
    network: Network<GripperData>,
    ri_version: Option<u16>,
}

impl Gripper {
    ///  Establishes a connection with a gripper connected to a robot.
    /// # Arguments
    /// * `franka_address` - IP/hostname of the robot the gripper is connected to.
    /// # Errors
    /// * [`NetworkException`](`crate::exception::FrankaException::NetworkException`) if the connection is lost, e.g. after a timeout.
    /// * [`IncompatibleLibraryVersionError`](`crate::exception::FrankaException::IncompatibleLibraryVersionError`) if this version of libfranka-rs is not supported
    pub fn new(franka_address: &str) -> FrankaResult<Gripper> {
        let mut gripper = Gripper {
            network: Network::new(franka_address, COMMAND_PORT).map_err(|e| {
                FrankaException::NetworkException {
                    message: e.to_string(),
                }
            })?,
            ri_version: None,
        };
        gripper.connect_gripper(&GRIPPER_VERSION)?;
        Ok(gripper)
    }
    fn connect_gripper(&mut self, ri_version: &u16) -> FrankaResult<()> {
        let connect_command = ConnectRequestWithHeader {
            header: self.network.create_header(
                GripperCommandEnum::Connect,
                size_of::<ConnectRequestWithHeader>(),
            ),
            request: ConnectRequest::new(self.network.get_udp_port()),
        };
        let command_id: u32 = self.network.tcp_send_request(connect_command);
        let connect_response: ConnectResponse =
            self.network.tcp_blocking_receive_response(command_id);
        match connect_response.status {
            Status::Success => {
                self.ri_version = Some(connect_response.version);
                Ok(())
            }
            _ => Err(FrankaException::IncompatibleLibraryVersionError {
                server_version: connect_response.version,
                library_version: *ri_version,
            }),
        }
    }
    /// Moves the gripper fingers to a specified width.
    /// # Arguments
    /// * `width` - Intended opening width. \[m\]
    /// * `speed` - Closing speed. \[m/s\]
    /// # Errors
    /// * [`NetworkException`](`crate::exception::FrankaException::NetworkException`) if the connection is lost, e.g. after a timeout.
    /// * [`CommandException`](`crate::exception::FrankaException::CommandException`) if an error occurred
    /// # Return
    /// True if command was successful, false otherwise.
    pub fn move_gripper(&mut self, width: f64, speed: f64) -> FrankaResult<bool> {
        let command = MoveRequestWithHeader {
            header: self
                .network
                .create_header(GripperCommandEnum::Move, size_of::<MoveRequestWithHeader>()),
            request: MoveRequest::new(width, speed),
        };
        let command_id: u32 = self.network.tcp_send_request(command);
        let status: Status = self.network.tcp_blocking_receive_status(command_id);
        handle_response_status(&status)
    }
    /// Performs homing of the gripper.
    ///
    /// After changing the gripper fingers, a homing needs to be done.
    /// This is needed to estimate the maximum grasping width.
    ///
    /// # Errors
    /// * [`NetworkException`](`crate::exception::FrankaException::NetworkException`) if the connection is lost, e.g. after a timeout.
    /// [`CommandException`](`crate::exception::FrankaException::CommandException`) if an error occurred
    /// # Return
    /// True if command was successful, false otherwise.
    pub fn homing(&mut self) -> FrankaResult<bool> {
        let command: GripperCommandHeader = self.network.create_header(
            GripperCommandEnum::Homing,
            size_of::<GripperCommandHeader>(),
        );
        let command_id: u32 = self.network.tcp_send_request(command);
        let status: Status = self.network.tcp_blocking_receive_status(command_id);
        handle_response_status(&status)
    }

    /// Stops a currently running gripper move or grasp.
    /// # Errors
    /// * [`NetworkException`](`crate::exception::FrankaException::NetworkException`) if the connection is lost, e.g. after a timeout.
    /// * [`CommandException`](`crate::exception::FrankaException::CommandException`) if an error occurred
    /// # Return
    /// True if command was successful, false otherwise.
    pub fn stop(&mut self) -> FrankaResult<bool> {
        let command: GripperCommandHeader = self
            .network
            .create_header(GripperCommandEnum::Stop, size_of::<GripperCommandHeader>());
        let command_id: u32 = self.network.tcp_send_request(command);
        let status: Status = self.network.tcp_blocking_receive_status(command_id);
        handle_response_status(&status)
    }

    /// Grasps an object.
    ///
    /// An object is considered grasped if the distance `d` between the gripper fingers satisfies
    /// ![a](https://latex.codecogs.com/png.latex?%5Ctext%7Bwidth%7D%20-%20%5Ctext%7Bepsilon%5C_inner%7D%20%3C%20d%20%3C%20%5Ctext%7Bwidth%7D%20&plus;%20%5Ctext%7Bepsilon%5C_outer%7D)
    /// # Arguments
    /// * `width` - Size of the object to grasp. \[m\]
    /// * `speed` - Closing speed. \[m/s\]
    /// * `force` - Grasping force. \[N\]
    /// * `epsilon_inner` - Maximum tolerated deviation when the actual grasped width is smaller
    /// than the commanded grasp width. Default is 0.005.
    /// * `epsilon_outer` -  Maximum tolerated deviation when the actual grasped width is larger than the commanded grasp width.
    /// Default is 0.005.
    /// # Errors
    /// * [`NetworkException`](`crate::exception::FrankaException::NetworkException`) if the connection is lost, e.g. after a timeout.
    /// * [`CommandException`](`crate::exception::FrankaException::CommandException`) if an error occurred
    /// # Return
    /// True if an object has been grasped, false otherwise.
    pub fn grasp(
        &mut self,
        width: f64,
        speed: f64,
        force: f64,
        epsilon_inner: Option<f64>,
        epsilon_outer: Option<f64>,
    ) -> FrankaResult<bool> {
        let epsilon_inner = epsilon_inner.unwrap_or(0.005);
        let epsilon_outer = epsilon_outer.unwrap_or(0.005);
        let command = GraspRequestWithHeader {
            header: self.network.create_header(
                GripperCommandEnum::Grasp,
                size_of::<GraspRequestWithHeader>(),
            ),
            request: GraspRequest::new(width, speed, force, epsilon_inner, epsilon_outer),
        };
        let command_id: u32 = self.network.tcp_send_request(command);
        let status: Status = self.network.tcp_blocking_receive_status(command_id);
        handle_response_status(&status)
    }
    /// Waits for a gripper state update and returns it.
    /// # Errors
    /// * [`NetworkException`](`crate::exception::FrankaException::NetworkException`) if the connection is lost, e.g. after a timeout.
    /// * [`CommandException`](`crate::exception::FrankaException::CommandException`) if an error occurred
    /// # Return
    ///  Current gripper state.
    pub fn read_once(&mut self) -> FrankaResult<GripperState> {
        while self.network.udp_receive::<GripperStateIntern>().is_some() {}
        Ok(self
            .network
            .udp_blocking_receive::<GripperStateIntern>()?
            .into())
    }
    /// Returns the software version reported by the connected server.
    ///
    /// # Return
    /// Software version of the connected server.
    pub fn server_version(&self) -> u16 {
        self.ri_version.unwrap()
    }
}

fn handle_response_status(status: &Status) -> FrankaResult<bool> {
    match status {
        Status::Success => Ok(true),
        Status::Fail => Err(create_command_exception(
            "libfranka-rs gripper: Command failed!",
        )),
        Status::Unsuccessful => Ok(false),
        Status::Aborted => Err(create_command_exception(
            "libfranka-rs gripper: Command aborted!",
        )),
    }
}

#[cfg(test)]
mod tests {
    use crate::gripper::types::{
        ConnectRequestWithHeader, ConnectResponse, GraspRequest, GraspRequestWithHeader,
        GripperCommandEnum, GripperCommandHeader, GripperStateIntern, MoveRequest,
        MoveRequestWithHeader, Status, COMMAND_PORT, GRIPPER_VERSION,
    };
    use crate::gripper::Gripper;
    use crate::FrankaResult;
    use bincode::{deserialize, serialize, serialized_size};
    use mio::net::UdpSocket;
    use mockall::{automock, predicate::*};
    use std::io::{Read, Write};
    use std::net::TcpListener;
    use std::net::ToSocketAddrs;
    use std::rc::Rc;
    use std::sync::{Arc, Mutex};
    use std::time::{Duration, Instant};

    use crate::exception::FrankaException;
    use crate::gripper::types::Status::{Fail, Success};
    use crate::network::MessageCommand;
    use serde::{Deserialize, Serialize};
    use std::iter::FromIterator;
    use std::mem::size_of;

    #[derive(Serialize, Deserialize, Clone, Copy)]
    #[repr(packed)]
    struct GripperResponse {
        pub header: GripperCommandHeader,
        pub status: Status,
    }
    struct Socket<F: Fn(&Vec<u8>), G: Fn(&mut Vec<u8>)> {
        pub send_bytes: F,
        pub receive_bytes: G,
    }

    struct GripperMockServer {
        server_version: u16,
    }

    pub struct ServerReaction {}

    #[automock]
    #[allow(unused)]
    impl ServerReaction {
        fn process_received_bytes(&self, bytes: &mut Vec<u8>) -> Vec<u8> {
            Vec::new()
        }
        fn number_of_reactions(&self) -> usize {
            0
        }
    }

    impl GripperMockServer {
        pub fn new(server_version: u16) -> Self {
            GripperMockServer { server_version }
        }

        pub fn server_thread(&mut self, reaction: &mut MockServerReaction) {
            let hostname: &str = "127.0.0.1";
            let address = format!("{}:{}", hostname, COMMAND_PORT)
                .to_socket_addrs()
                .unwrap()
                .next()
                .unwrap();
            let srv_sock = TcpListener::bind(address).unwrap();
            let (tcp_socket, _remote_address) = srv_sock.accept().unwrap();
            tcp_socket.set_nonblocking(false).unwrap();
            tcp_socket.set_nodelay(true).unwrap();
            let tcp_socket = Rc::new(Mutex::new(tcp_socket));

            let mut tcp_socket_wrapper = Socket {
                send_bytes: |bytes| {
                    let mut soc = tcp_socket.lock().unwrap();
                    soc.write(bytes.as_slice()).unwrap();
                },
                receive_bytes: |bytes| {
                    let mut soc = tcp_socket.lock().unwrap();
                    let mut buffer = vec![0 as u8; 100];
                    let num_bytes = soc.read(&mut buffer).unwrap();
                    buffer.resize(num_bytes, 0);
                    assert_eq!(buffer.len(), num_bytes);
                    *bytes = buffer;
                },
            };
            let request = self.receive_gripper_connect_request(&mut tcp_socket_wrapper);
            let udp_port = request.request.udp_port;
            self.send_gripper_connect_response(request, &mut tcp_socket_wrapper);

            let udp_socket = UdpSocket::bind(
                format!("{}:1833", hostname)
                    .to_socket_addrs()
                    .unwrap()
                    .next()
                    .unwrap(),
            )
            .unwrap();
            udp_socket
                .connect(
                    format!("{}:{}", hostname, udp_port)
                        .to_socket_addrs()
                        .unwrap()
                        .next()
                        .unwrap(),
                )
                .unwrap();
            let udp_socket_wrapper = Socket {
                send_bytes: move |bytes| {
                    let res = udp_socket.send(bytes.as_slice());
                    if res.is_err() {
                        println!("could not send upd bytes");
                        return;
                    }
                    let num_bytes = res.unwrap();
                    assert_eq!(num_bytes, bytes.len());
                },
                receive_bytes: |_bytes| unimplemented!(),
            };

            let udp_thread = std::thread::spawn(move || {
                let mut counter = 1;
                let start = Instant::now();
                while (Instant::now() - start).as_secs_f64() < 0.01 {
                    let bytes = serialize(&GripperStateIntern {
                        message_id: counter,
                        width: 0.0,
                        max_width: 0.0,
                        is_grasped: false,
                        temperature: 0,
                    })
                    .unwrap();
                    counter += 1;
                    (udp_socket_wrapper.send_bytes)(&bytes);
                    std::thread::sleep(Duration::from_millis(10));
                }
            });
            let mut wrapper = Box::new(tcp_socket_wrapper);

            for _ in 0..reaction.number_of_reactions() {
                self.handle_receive(&mut wrapper, reaction);
                std::thread::sleep(Duration::from_millis(10));
            }
            udp_thread.join().unwrap();
        }

        fn handle_receive<F, G>(
            &self,
            tcp_socket: &mut Box<Socket<F, G>>,
            reaction: &mut MockServerReaction,
        ) where
            F: Fn(&Vec<u8>),
            G: Fn(&mut Vec<u8>),
        {
            let mut bytes = vec![0 as u8; 100];
            (tcp_socket.receive_bytes)(&mut bytes);
            let response = reaction.process_received_bytes(&mut bytes);
            (tcp_socket.send_bytes)(&response);
        }
        fn receive_gripper_connect_request<F: Fn(&Vec<u8>), G: Fn(&mut Vec<u8>)>(
            &self,
            tcp_socket: &mut Socket<F, G>,
        ) -> ConnectRequestWithHeader {
            let mut bytes = vec![0 as u8; 100];
            (tcp_socket.receive_bytes)(&mut bytes);
            let request: ConnectRequestWithHeader = deserialize(bytes.as_slice()).unwrap();
            return request;
        }
        fn send_gripper_connect_response<F: Fn(&Vec<u8>), G: Fn(&mut Vec<u8>)>(
            &self,
            request: ConnectRequestWithHeader,
            tcp_socket: &mut Socket<F, G>,
        ) {
            let mut response = ConnectResponse {
                header: GripperCommandHeader {
                    command: GripperCommandEnum::Connect,
                    command_id: request.get_command_message_id(),
                    size: 0,
                },
                status: match self.server_version == request.request.version {
                    true => Success,
                    false => Fail,
                },
                version: self.server_version,
            };
            let response_size = serialized_size(&response).unwrap();
            response.header.size = response_size as u32;
            let serialized_response = serialize(&response).unwrap();
            (tcp_socket.send_bytes)(&serialized_response);
        }
    }

    #[test]
    fn gripper_move_test() -> FrankaResult<()> {
        let mut counter = 0;
        let move_request_values = [(0.1, 0.5), (0.3, 0.1)];
        let mut generate_move_request = move |width: f64, speed: f64| -> MoveRequestWithHeader {
            counter += 1;
            MoveRequestWithHeader {
                header: GripperCommandHeader::new(
                    GripperCommandEnum::Move,
                    counter,
                    size_of::<MoveRequestWithHeader>() as u32,
                ),
                request: MoveRequest::new(width, speed),
            }
        };
        let requests = Arc::new(Vec::from_iter(
            move_request_values
                .iter()
                .map(|(x, y)| generate_move_request(*x, *y)),
        ));

        let requests_server = requests.clone();
        let thread = std::thread::spawn(|| {
            let mut mock_gripper = GripperMockServer::new(GRIPPER_VERSION);
            let mut mock = MockServerReaction::default();
            let num_requests = requests_server.len();
            let mut counter = 0;
            mock.expect_process_received_bytes()
                .returning(move |bytes: &mut Vec<u8>| -> Vec<u8> {
                    let expected_request = requests_server.get(counter).unwrap();
                    let serialized_expected_request = serialize(expected_request).unwrap();
                    assert_eq!(bytes.len(), serialized_expected_request.len());
                    bytes
                        .iter()
                        .zip(serialized_expected_request.iter())
                        .for_each(|(x, y)| assert_eq!(x, y));
                    let req: MoveRequestWithHeader = deserialize(&bytes).unwrap();
                    counter += 1;
                    let mut response = GripperResponse {
                        header: GripperCommandHeader::new(
                            GripperCommandEnum::Move,
                            req.header.command_id,
                            0,
                        ),
                        status: Status::Success,
                    };
                    response.header.size = serialized_size(&response).unwrap() as u32;
                    serialize(&response).unwrap()
                })
                .times(num_requests);
            mock.expect_number_of_reactions().return_const(num_requests);
            mock_gripper.server_thread(&mut mock);
        });
        {
            std::thread::sleep(Duration::from_secs_f64(0.01));
            let mut gripper = Gripper::new("127.0.0.1").expect("gripper failure");
            assert_eq!(gripper.server_version(), GRIPPER_VERSION);
            for (width, speed) in move_request_values.iter() {
                gripper.move_gripper(*width, *speed).unwrap();
            }
        }
        thread.join().unwrap();
        Ok(())
    }

    #[test]
    fn gripper_stop_test() -> FrankaResult<()> {
        let thread = std::thread::spawn(|| {
            let mut mock_gripper = GripperMockServer::new(GRIPPER_VERSION);
            let mut mock = MockServerReaction::default();
            mock.expect_process_received_bytes()
                .returning(move |bytes: &mut Vec<u8>| -> Vec<u8> {
                    let req: GripperCommandHeader = deserialize(&bytes).unwrap();
                    match req.command {
                        GripperCommandEnum::Stop => {}
                        _ => {
                            assert!(false)
                        }
                    }
                    let mut response = GripperResponse {
                        header: GripperCommandHeader::new(
                            GripperCommandEnum::Stop,
                            req.command_id,
                            0,
                        ),
                        status: Status::Success,
                    };
                    response.header.size = serialized_size(&response).unwrap() as u32;
                    serialize(&response).unwrap()
                })
                .times(1);
            mock.expect_number_of_reactions().return_const(1 as usize);
            mock_gripper.server_thread(&mut mock);
        });
        {
            std::thread::sleep(Duration::from_secs_f64(0.01));
            let mut gripper = Gripper::new("127.0.0.1").expect("gripper failure");
            gripper.stop().unwrap();
        }
        thread.join().unwrap();
        Ok(())
    }

    #[test]
    fn gripper_homing_test() -> FrankaResult<()> {
        let thread = std::thread::spawn(|| {
            let mut mock_gripper = GripperMockServer::new(GRIPPER_VERSION);
            let mut mock = MockServerReaction::default();
            mock.expect_process_received_bytes()
                .returning(move |bytes: &mut Vec<u8>| -> Vec<u8> {
                    let req: GripperCommandHeader = deserialize(&bytes).unwrap();
                    match req.command {
                        GripperCommandEnum::Homing => {}
                        _ => {
                            assert!(false)
                        }
                    }
                    let mut response = GripperResponse {
                        header: GripperCommandHeader::new(
                            GripperCommandEnum::Homing,
                            req.command_id,
                            0,
                        ),
                        status: Status::Success,
                    };
                    response.header.size = serialized_size(&response).unwrap() as u32;
                    serialize(&response).unwrap()
                })
                .times(1);
            mock.expect_number_of_reactions().return_const(1 as usize);
            mock_gripper.server_thread(&mut mock);
        });
        {
            std::thread::sleep(Duration::from_secs_f64(0.01));
            let mut gripper = Gripper::new("127.0.0.1").expect("gripper failure");
            gripper.homing().unwrap();
        }
        thread.join().unwrap();
        Ok(())
    }

    #[test]
    fn gripper_grasp_test() -> FrankaResult<()> {
        let mut counter = 0;
        let grasp_request_values = [(0.05, 0.1, 400., 0.004, 0.005)];
        let mut generate_grasp_request = move |width: f64,
                                               speed: f64,
                                               force: f64,
                                               epsilon_inner: f64,
                                               epsilon_outer: f64|
              -> GraspRequestWithHeader {
            counter += 1;
            GraspRequestWithHeader {
                header: GripperCommandHeader::new(
                    GripperCommandEnum::Grasp,
                    counter,
                    size_of::<GraspRequestWithHeader>() as u32,
                ),
                request: GraspRequest::new(width, speed, force, epsilon_inner, epsilon_outer),
            }
        };
        let requests =
            Arc::new(Vec::from_iter(grasp_request_values.iter().map(
                |(a, b, c, d, e)| generate_grasp_request(*a, *b, *c, *d, *e),
            )));

        let requests_server = requests.clone();
        let thread = std::thread::spawn(|| {
            let mut mock_gripper = GripperMockServer::new(GRIPPER_VERSION);
            let mut mock = MockServerReaction::default();
            let num_requests = requests_server.len();
            let mut counter = 0;
            mock.expect_process_received_bytes()
                .returning(move |bytes: &mut Vec<u8>| -> Vec<u8> {
                    let expected_request = requests_server.get(counter).unwrap();
                    let serialized_expected_request = serialize(expected_request).unwrap();
                    assert_eq!(bytes.len(), serialized_expected_request.len());
                    bytes
                        .iter()
                        .zip(serialized_expected_request.iter())
                        .for_each(|(x, y)| assert_eq!(x, y));
                    let req: MoveRequestWithHeader = deserialize(&bytes).unwrap();
                    counter += 1;
                    let mut response = GripperResponse {
                        header: GripperCommandHeader::new(
                            GripperCommandEnum::Grasp,
                            req.header.command_id,
                            0,
                        ),
                        status: Status::Success,
                    };
                    response.header.size = serialized_size(&response).unwrap() as u32;
                    serialize(&response).unwrap()
                })
                .times(num_requests);
            mock.expect_number_of_reactions().return_const(num_requests);
            mock_gripper.server_thread(&mut mock);
        });
        {
            std::thread::sleep(Duration::from_secs_f64(0.01));
            let mut gripper = Gripper::new("127.0.0.1").expect("gripper failure");
            for (width, speed, force, epsilon_inner, epsilon_outer) in grasp_request_values.iter() {
                gripper
                    .grasp(
                        *width,
                        *speed,
                        *force,
                        Some(*epsilon_inner),
                        Some(*epsilon_outer),
                    )
                    .unwrap();
            }
        }
        thread.join().unwrap();
        Ok(())
    }

    #[test]
    fn incompatible_library() {
        let thread = std::thread::spawn(|| {
            let mut mock_gripper = GripperMockServer::new(GRIPPER_VERSION + 1);
            let mut mock = MockServerReaction::default();
            mock.expect_process_received_bytes()
                .returning(|_bytes| Vec::<u8>::new());
            mock.expect_number_of_reactions().return_const(0 as usize);
            mock_gripper.server_thread(&mut mock);
        });
        let gripper_result;
        {
            std::thread::sleep(Duration::from_secs_f64(0.01));
            gripper_result = Gripper::new("127.0.0.1")
        }
        thread.join().unwrap();
        match gripper_result {
            Ok(_) => {
                panic!("Expected incompatible library version")
            }
            Err(error) => match error {
                FrankaException::IncompatibleLibraryVersionError { .. } => {}
                e => {
                    panic!("Expected incompatible library version but found {:?}", e)
                }
            },
        };
    }

    #[test]
    fn gripper_read_once() {
        let thread = std::thread::spawn(|| {
            let mut server = GripperMockServer::new(GRIPPER_VERSION);
            let mut mock = MockServerReaction::default();
            mock.expect_process_received_bytes()
                .returning(|_bytes| Vec::<u8>::new());
            mock.expect_number_of_reactions().return_const(0 as usize);
            server.server_thread(&mut mock);
        });
        {
            std::thread::sleep(Duration::from_secs_f64(0.01));
            let mut gripper = Gripper::new("127.0.0.1").expect("gripper failure");
            let _state = gripper.read_once().expect("could not read gripper state");
        }
        thread.join().unwrap();
    }
}

pub struct GripperData {}

impl DeviceData for GripperData {
    type CommandHeader = GripperCommandHeader;
    type CommandEnum = GripperCommandEnum;

    fn create_header(
        command_id: &mut u32,
        command: Self::CommandEnum,
        size: usize,
    ) -> Self::CommandHeader {
        let header = GripperCommandHeader::new(command, *command_id, size as u32);
        *command_id += 1;
        header
    }

    fn get_library_version() -> u16 {
        GRIPPER_VERSION
    }
}
