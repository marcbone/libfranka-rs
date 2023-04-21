// Copyright (c) 2021 Marco Boneberger
// Licensed under the EUPL-1.2-or-later

//! Contains the franka::Robot type.

mod control_loop;
mod control_tools;
pub mod control_types;
pub mod error;
pub mod errors;
pub mod fr3;
pub mod logger;
pub mod low_pass_filter;
mod motion_generator_traits;
pub mod panda;
mod rate_limiting;
pub mod robot;
mod robot_control;
mod robot_impl;
pub mod robot_state;
pub mod robot_wrapper;
pub(crate) mod service_types;
pub(crate) mod types;
pub mod virtual_wall_cuboid;

#[cfg(test)]
mod tests {
    use mockall::{automock, predicate::*};
    use std::io::{Read, Write};
    use std::net::TcpListener;
    use std::net::ToSocketAddrs;
    use std::rc::Rc;
    use std::sync::{Arc, Mutex};

    use crate::exception::FrankaException;
    use crate::network::MessageCommand;
    use crate::robot::service_types::{
        ConnectRequestWithPandaHeader, ConnectResponsePanda, ConnectStatus, Fr3CommandEnum,
        Fr3CommandHeader, GetterSetterStatusPanda, MoveControllerMode, MoveDeviation,
        MoveMotionGeneratorMode, MoveRequest, MoveRequestWithPandaHeader, MoveStatusPanda,
        PandaCommandEnum, PandaCommandHeader, SetCollisionBehaviorRequest,
        SetCollisionBehaviorRequestWithFr3Header, SetCollisionBehaviorRequestWithPandaHeader,
        SetterResponseFr3, COMMAND_PORT, FR3_VERSION,
    };
    use crate::robot::types::PandaStateIntern;
    use crate::{Fr3, RobotWrapper};
    use crate::{FrankaResult, JointPositions, MotionFinished, Panda, RealtimeConfig, RobotState};
    use bincode::{deserialize, serialize, serialized_size};
    use std::iter::FromIterator;
    use std::mem::size_of;
    use std::time::{Duration, Instant};

    struct Socket<F: Fn(&Vec<u8>), G: Fn(&mut Vec<u8>)> {
        pub send_bytes: F,
        pub receive_bytes: G,
    }

    struct RobotMockServer {
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

    impl RobotMockServer {
        pub fn new(server_version: u16) -> Self {
            RobotMockServer { server_version }
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
                    println!("send bytes");
                },
                receive_bytes: |bytes| {
                    let mut soc = tcp_socket.lock().unwrap();
                    let mut buffer = vec![0 as u8; 3000];
                    let num_bytes = soc.read(&mut buffer).unwrap();
                    buffer.resize(num_bytes, 0);
                    assert_eq!(buffer.len(), num_bytes);
                    *bytes = buffer;
                },
            };
            let request = self.receive_robot_connect_request(&mut tcp_socket_wrapper);
            let udp_port = request.request.udp_port;
            self.send_robot_connect_response(request, &mut tcp_socket_wrapper);

            let udp_socket = std::net::UdpSocket::bind(
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
                while (Instant::now() - start).as_secs_f64() < 0.1 {
                    let mut state = PandaStateIntern::dummy();
                    state.message_id = counter;
                    let bytes = serialize(&state).unwrap();
                    counter += 1;
                    (udp_socket_wrapper.send_bytes)(&bytes);
                    std::thread::sleep(Duration::from_millis(5));
                }
            });
            let mut wrapper = Box::new(tcp_socket_wrapper);

            for _ in 0..reaction.number_of_reactions() {
                self.handle_receive(&mut wrapper, reaction);
                std::thread::sleep(Duration::from_millis(5));
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
        fn receive_robot_connect_request<F: Fn(&Vec<u8>), G: Fn(&mut Vec<u8>)>(
            &self,
            tcp_socket: &mut Socket<F, G>,
        ) -> ConnectRequestWithPandaHeader {
            let mut bytes = vec![0 as u8; 100];
            (tcp_socket.receive_bytes)(&mut bytes);
            let request: ConnectRequestWithPandaHeader = deserialize(bytes.as_slice()).unwrap();
            return request;
        }
        fn send_robot_connect_response<F: Fn(&Vec<u8>), G: Fn(&mut Vec<u8>)>(
            &self,
            request: ConnectRequestWithPandaHeader,
            tcp_socket: &mut Socket<F, G>,
        ) {
            let mut response = ConnectResponsePanda {
                header: PandaCommandHeader {
                    command: PandaCommandEnum::Connect,
                    command_id: request.get_command_message_id(),
                    size: 0,
                },
                status: match self.server_version == request.request.version {
                    true => ConnectStatus::Success,
                    false => ConnectStatus::IncompatibleLibraryVersion,
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
    fn set_collision_behavior_test() -> FrankaResult<()> {
        let mut counter = 0;
        let collision_behavior_request_values = [(
            [20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0],
            [20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0],
            [20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0],
            [20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0],
            [20.0, 20.0, 20.0, 25.0, 25.0, 25.0],
            [20.0, 20.0, 20.0, 25.0, 25.0, 25.0],
            [20.0, 20.0, 20.0, 25.0, 25.0, 25.0],
            [20.0, 20.0, 20.0, 25.0, 25.0, 25.0],
        )];
        let mut generate_collision_behavior_request =
            move |lower_torque_thresholds_acceleration: [f64; 7],
                  upper_torque_thresholds_acceleration: [f64; 7],
                  lower_torque_thresholds_nominal: [f64; 7],
                  upper_torque_thresholds_nominal: [f64; 7],
                  lower_force_thresholds_acceleration: [f64; 6],
                  upper_force_thresholds_acceleration: [f64; 6],
                  lower_force_thresholds_nominal: [f64; 6],
                  upper_force_thresholds_nominal: [f64; 6]| {
                counter += 1;
                SetCollisionBehaviorRequestWithFr3Header {
                    header: Fr3CommandHeader::new(
                        Fr3CommandEnum::SetCollisionBehavior,
                        counter,
                        size_of::<SetCollisionBehaviorRequestWithFr3Header>() as u32,
                    ),
                    request: SetCollisionBehaviorRequest::new(
                        lower_torque_thresholds_acceleration,
                        upper_torque_thresholds_acceleration,
                        lower_torque_thresholds_nominal,
                        upper_torque_thresholds_nominal,
                        lower_force_thresholds_acceleration,
                        upper_force_thresholds_acceleration,
                        lower_force_thresholds_nominal,
                        upper_force_thresholds_nominal,
                    ),
                }
            };
        let requests = Arc::new(Vec::from_iter(
            collision_behavior_request_values
                .iter()
                .map(|(a, b, c, d, e, f, g, h)| {
                    generate_collision_behavior_request(*a, *b, *c, *d, *e, *f, *g, *h)
                }),
        ));
        let requests_server = requests.clone();
        let thread = std::thread::spawn(|| {
            let mut robot_server = RobotMockServer::new(FR3_VERSION);
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
                    let req: SetCollisionBehaviorRequestWithPandaHeader =
                        deserialize(&bytes).unwrap();
                    counter += 1;
                    let mut response = SetterResponseFr3 {
                        header: Fr3CommandHeader::new(
                            Fr3CommandEnum::SetCollisionBehavior,
                            req.header.command_id,
                            0,
                        ),
                        status: GetterSetterStatusPanda::Success,
                    };
                    response.header.size = serialized_size(&response).unwrap() as u32;
                    serialize(&response).unwrap()
                })
                .times(num_requests);
            mock.expect_number_of_reactions().return_const(num_requests);
            robot_server.server_thread(&mut mock);
        });
        {
            std::thread::sleep(Duration::from_secs_f64(0.01));
            let mut robot = Fr3::new("127.0.0.1", None, None).expect("robot failure");
            assert_eq!(robot.server_version(), FR3_VERSION);
            for (a, b, c, d, e, f, g, h) in collision_behavior_request_values.iter() {
                robot
                    .set_collision_behavior(*a, *b, *c, *d, *e, *f, *g, *h)
                    .unwrap();
            }
        }
        thread.join().unwrap();

        Ok(())
    }

    #[test]
    fn fail_start_motion_test() {
        let requests = Arc::new(vec![MoveRequestWithPandaHeader {
            header: PandaCommandHeader::new(
                PandaCommandEnum::Move,
                1,
                size_of::<MoveRequestWithPandaHeader>() as u32,
            ),
            request: MoveRequest::new(
                MoveControllerMode::JointImpedance,
                MoveMotionGeneratorMode::JointPosition,
                MoveDeviation {
                    translation: 10.,
                    rotation: 3.12,
                    elbow: 2. * std::f64::consts::PI,
                },
                MoveDeviation {
                    translation: 10.,
                    rotation: 3.12,
                    elbow: 2. * std::f64::consts::PI,
                },
            ),
        }]);
        let requests_server = requests.clone();
        let thread = std::thread::spawn(|| {
            let mut robot_server = RobotMockServer::new(FR3_VERSION);
            let mut mock = MockServerReaction::default();
            let num_requests = requests_server.len();
            let mut counter = 0;
            mock.expect_process_received_bytes()
                .returning(move |bytes: &mut Vec<u8>| -> Vec<u8> {
                    let expected_request = requests_server.get(counter).unwrap();
                    let serialized_expected_request = serialize(expected_request).unwrap();
                    let req: MoveRequestWithPandaHeader = deserialize(&bytes).unwrap();
                    assert_eq!(bytes.len(), serialized_expected_request.len());
                    bytes
                        .iter()
                        .zip(serialized_expected_request.iter())
                        .for_each(|(x, y)| assert_eq!(x, y));
                    counter += 1;

                    let mut response = (
                        PandaCommandHeader::new(PandaCommandEnum::Move, req.header.command_id, 0),
                        MoveStatusPanda::Aborted,
                    );
                    response.0.size = serialized_size(&response).unwrap() as u32;
                    serialize(&response).unwrap()
                })
                .times(num_requests);
            mock.expect_number_of_reactions().return_const(num_requests);
            robot_server.server_thread(&mut mock);
        });
        {
            std::thread::sleep(Duration::from_secs_f64(0.01));
            let mut robot =
                Fr3::new("127.0.0.1", RealtimeConfig::Ignore, None).expect("robot failure");
            let mut counter = 0;
            let result = robot.control_joint_positions(
                |_, _| {
                    counter += 1;
                    if counter > 2 {
                        return JointPositions::new([0.; 7]).motion_finished();
                    }
                    JointPositions::new([0.; 7])
                },
                None,
                None,
                None,
            );
            match result {
                Err(FrankaException::CommandException { message: _ }) => {
                    thread.join().unwrap();
                }
                _ => {
                    panic!("did not receive a command exception")
                }
            }
        }
    }

    #[test]
    fn incompatible_library() {
        let thread = std::thread::spawn(|| {
            let mut robot_server = RobotMockServer::new(FR3_VERSION + 1);
            let mut mock = MockServerReaction::default();
            mock.expect_process_received_bytes()
                .returning(|_bytes| Vec::<u8>::new());
            mock.expect_number_of_reactions().return_const(0 as usize);
            robot_server.server_thread(&mut mock);
        });
        std::thread::sleep(Duration::from_secs_f64(0.01));
        let robot_result = Panda::new("127.0.0.1", None, None);

        thread.join().unwrap();
        match robot_result {
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
    fn robot_read_once() -> FrankaResult<()> {
        let thread = std::thread::spawn(|| {
            let mut robot_server = RobotMockServer::new(FR3_VERSION);
            let mut mock = MockServerReaction::default();
            mock.expect_process_received_bytes()
                .returning(|_bytes| Vec::<u8>::new());
            mock.expect_number_of_reactions().return_const(0 as usize);
            robot_server.server_thread(&mut mock);
        });

        {
            std::thread::sleep(Duration::from_secs_f64(0.01));
            let mut robot = Fr3::new("127.0.0.1", None, None)?;
            let _state = robot.read_once().unwrap();
        }
        thread.join().unwrap();
        Ok(())
    }

    #[test]
    fn robot_read() -> FrankaResult<()> {
        let thread = std::thread::spawn(|| {
            let mut robot_server = RobotMockServer::new(FR3_VERSION);
            let mut mock = MockServerReaction::default();
            mock.expect_process_received_bytes()
                .returning(|_bytes| Vec::<u8>::new());
            mock.expect_number_of_reactions().return_const(0 as usize);
            robot_server.server_thread(&mut mock);
        });
        {
            std::thread::sleep(Duration::from_secs_f64(0.01));
            let mut robot = Fr3::new("127.0.0.1", None, None)?;
            let mut counter = 0;
            let mut first_time = true;
            let mut start_counter = 0;
            robot
                .read(|state: &RobotState| {
                    if first_time {
                        first_time = false;
                        counter = state.time.as_millis();
                        start_counter = counter;
                    }
                    assert_eq!(state.time.as_millis(), counter);
                    counter += 1;
                    counter < start_counter + 10
                })
                .unwrap();
        }
        thread.join().unwrap();
        Ok(())
    }
}
