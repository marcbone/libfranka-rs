// Copyright (c) 2021 Marco Boneberger
// Licensed under the EUPL-1.2-or-later
use crate::exception::{FrankaException, FrankaResult};
use crate::model::library_downloader::{LibraryDownloader, LibraryDownloaderGeneric};
use crate::network::{Network, RobotData};
use crate::robot::control_types::RealtimeConfig;
use crate::robot::logger::Logger;

use crate::robot::robot_control::RobotControl;
use crate::robot::robot_state::AbstractRobotState;
use crate::robot::service_types::{
    ConnectResponseWithoutHeader, ConnectStatus, MoveControllerMode, MoveDeviation,
    MoveMotionGeneratorMode, MoveRequest,
};
use crate::robot::types::{
    AbstractRobotStateIntern, ControllerCommand, ControllerMode, MotionGeneratorCommand,
    MotionGeneratorMode, RobotCommand,
};
use crate::RobotModel;
use std::fs::remove_file;
use std::path::Path;

pub trait RobotImplementation:
    RobotControl<State = <<Self as RobotImplementation>::Data as RobotData>::State>
{
    type Data: RobotData;
    fn read_once(
        &mut self,
    ) -> FrankaResult<<<Self as RobotImplementation>::Data as RobotData>::State>;
    fn load_model(
        &mut self,
        persistent: bool,
    ) -> FrankaResult<<<Self as RobotImplementation>::Data as RobotData>::Model>;
    fn server_version(&self) -> u16;
}

pub struct RobotImplGeneric<Data: RobotData> {
    pub network: Network<Data::DeviceData>,
    logger: Logger<Data::State>,
    realtime_config: RealtimeConfig,
    ri_version: Option<u16>,
    motion_generator_mode: Option<MotionGeneratorMode>,
    current_move_motion_generator_mode: MotionGeneratorMode,
    controller_mode: ControllerMode,
    current_move_controller_mode: Option<ControllerMode>,
    message_id: u64,
}

impl<Data: RobotData> RobotControl for RobotImplGeneric<Data> {
    type State = Data::State;

    fn start_motion(
        &mut self,
        controller_mode: MoveControllerMode,
        motion_generator_mode: MoveMotionGeneratorMode,
        maximum_path_deviation: &MoveDeviation,
        maximum_goal_deviation: &MoveDeviation,
    ) -> FrankaResult<u32> {
        if self.motion_generator_running() || self.controller_running() {
            panic!("libfranka-rs robot: Attempted to start multiple motions!");
        }
        self.current_move_motion_generator_mode = match motion_generator_mode {
            MoveMotionGeneratorMode::JointPosition => MotionGeneratorMode::JointPosition,
            MoveMotionGeneratorMode::JointVelocity => MotionGeneratorMode::JointVelocity,
            MoveMotionGeneratorMode::CartesianPosition => MotionGeneratorMode::CartesianPosition,
            MoveMotionGeneratorMode::CartesianVelocity => MotionGeneratorMode::CartesianVelocity,
        };
        self.current_move_controller_mode = match controller_mode {
            MoveControllerMode::JointImpedance => Some(ControllerMode::JointImpedance),
            MoveControllerMode::ExternalController => Some(ControllerMode::ExternalController),
            MoveControllerMode::CartesianImpedance => Some(ControllerMode::CartesianImpedance),
        };
        let move_command_id = self.execute_move_command(
            &controller_mode,
            &motion_generator_mode,
            maximum_path_deviation,
            maximum_goal_deviation,
        )?;

        while self.motion_generator_mode.unwrap() != self.current_move_motion_generator_mode
            || Some(self.controller_mode) != self.current_move_controller_mode
        {
            match self
                .network
                .tcp_receive_response(move_command_id, |x| Data::handle_command_move_status(x))
            {
                Ok(received_message) => {
                    if received_message {
                        break;
                    }
                }
                Err(FrankaException::CommandException { message }) => {
                    return Err(FrankaException::ControlException {
                        log: None,
                        error: message,
                    });
                }
                _ => {
                    panic!("this should be an command exception but it is not")
                }
            }

            let _robot_state = self.update(None, None)?;
        }
        self.logger.flush();
        Ok(move_command_id)
    }
    #[allow(unused_must_use)]
    fn finish_motion(
        &mut self,
        motion_id: u32,
        motion_command: Option<&MotionGeneratorCommand>,
        control_command: Option<&ControllerCommand>,
    ) -> FrankaResult<()> {
        if !self.motion_generator_running() && !self.controller_running() {
            self.current_move_motion_generator_mode = MotionGeneratorMode::Idle;
            self.current_move_controller_mode = Some(ControllerMode::Other);
            return Ok(());
        }
        if motion_command.is_none() {
            return Err(FrankaException::ControlException {
                log: None,
                error: String::from("libfranka-rs robot: No motion generator command given!"),
            });
        }
        let mut motion_finished_command = *motion_command.unwrap();
        motion_finished_command.motion_generation_finished = true;
        let mut robot_state = None;
        while self.motion_generator_running() || self.controller_running() {
            robot_state = Some(self.update(Some(&motion_finished_command), control_command)?);
        }
        let robot_state = robot_state.unwrap();

        let status = self.network.tcp_blocking_receive_status(motion_id);

        Data::create_control_exception_if_reflex_aborted(
            String::from("Motion finished commanded, but the robot is still moving!"),
            status,
            robot_state.get_last_motion_errors(),
            self.logger.flush(),
        )?;

        match Data::handle_command_move_status(status) {
            Ok(_) => {}
            Err(FrankaException::CommandException { message }) => {
                return Err(Data::create_control_exception(
                    message,
                    status,
                    robot_state.get_last_motion_errors(),
                    self.logger.flush(),
                ));
            }
            _ => {
                panic!("this should be an command exception but it is not")
            }
        }
        self.current_move_motion_generator_mode = MotionGeneratorMode::Idle;
        self.current_move_controller_mode = Some(ControllerMode::Other);
        Ok(())
    }

    // TODO exception handling
    fn cancel_motion(&mut self, motion_id: u32) {
        self.execute_stop_command()
            .expect("error while canceling motion"); //todo handle Network exception. But it is not that important since we already have an error anyways
        let mut _robot_state = self.receive_robot_state();
        while self.motion_generator_running() || self.controller_running() {
            _robot_state = self.receive_robot_state();
        }

        // comment from libfranka devs:
        // Ignore Move response.
        // TODO (FWA): It is not guaranteed that the Move response won't come later

        self.network
            .tcp_receive_response(motion_id, |_x: Data::MoveStatus| Ok(()))
            .expect("This should be impossible as the handler always returns Ok(())");
        self.current_move_motion_generator_mode = MotionGeneratorMode::Idle;
        self.current_move_controller_mode = Some(ControllerMode::Other);
    }

    fn update(
        &mut self,
        motion_command: Option<&MotionGeneratorCommand>,
        control_command: Option<&ControllerCommand>,
    ) -> FrankaResult<Data::State> {
        let robot_command = self.send_robot_command(motion_command, control_command)?;
        let state = Data::State::from(self.receive_robot_state()?);
        if let Some(command) = robot_command {
            self.logger.log(&state, &command);
        }
        Ok(state)
    }

    fn realtime_config(&self) -> RealtimeConfig {
        self.realtime_config
    }

    fn throw_on_motion_error(
        &mut self,
        robot_state: &Data::State,
        motion_id: u32,
    ) -> FrankaResult<()> {
        if !robot_state.is_moving()
            || self.motion_generator_mode.unwrap() != self.current_move_motion_generator_mode
            || self.controller_mode != self.current_move_controller_mode.unwrap()
        {
            let status = self.network.tcp_blocking_receive_status(motion_id);
            let result = Data::handle_command_move_status(status);
            return match result {
                Err(error) => Err(Data::create_control_exception(
                    error.to_string(),
                    status,
                    robot_state.get_last_motion_errors(),
                    self.logger.flush(),
                )),
                Ok(_) => panic!("Unexpected reply to a Move command"),
            };
        }
        Ok(())
    }
}

impl<Data: RobotData + RobotData<DeviceData = Data>> RobotImplementation
    for RobotImplGeneric<Data>
{
    type Data = Data;

    fn read_once(
        &mut self,
    ) -> FrankaResult<<<Self as RobotImplementation>::Data as RobotData>::State> {
        while self.network.udp_receive::<Data::StateIntern>().is_some() {}
        Ok(Data::State::from(self.receive_robot_state()?))
    }
    fn load_model(&mut self, persistent: bool) -> FrankaResult<Data::Model> {
        let model_file = Path::new("/tmp/model.so"); // TODO when we load from file we need to distinguish between FR3 and panda
        let model_already_downloaded = model_file.exists();
        if !model_already_downloaded {
            LibraryDownloaderGeneric::<Data>::download(&mut self.network, model_file)?;
        }
        let model = Data::Model::new(model_file, None)?;
        if !persistent && model_already_downloaded {
            remove_file(model_file).expect("Could not delete file");
        }
        Ok(model)
    }

    fn server_version(&self) -> u16 {
        self.ri_version.unwrap()
    }
}

// fn handle_command_move_status(status: &MoveStatusPanda) -> Result<(), FrankaException> {
//     match status {
//         MoveStatusPanda::Success => Ok(()),
//         MoveStatusPanda::MotionStarted => {
//             //todo handle motion_generator_running == true
//             Ok(())
//         }
//         MoveStatusPanda::EmergencyAborted => Err(create_command_exception(
//             "libfranka-rs: Move command aborted: User Stop pressed!",
//         )),
//         MoveStatusPanda::ReflexAborted => Err(create_command_exception(
//             "libfranka-rs: Move command aborted: motion aborted by reflex!",
//         )),
//         MoveStatusPanda::InputErrorAborted => Err(create_command_exception(
//             "libfranka-rs: Move command aborted: invalid input provided!",
//         )),
//         MoveStatusPanda::CommandNotPossibleRejected => Err(create_command_exception(
//             "libfranka-rs: Move command rejected: command not possible in the current mode!",
//         )),
//         MoveStatusPanda::StartAtSingularPoseRejected => Err(create_command_exception(
//             "libfranka-rs: Move command rejected: cannot start at singular pose!",
//         )),
//         MoveStatusPanda::InvalidArgumentRejected => Err(create_command_exception(
//             "libfranka-rs: Move command rejected: maximum path deviation out of range!",
//         )),
//         MoveStatusPanda::Preempted => Err(create_command_exception(
//             "libfranka-rs: Move command preempted!",
//         )),
//         MoveStatusPanda::Aborted => Err(create_command_exception(
//             "libfranka-rs: Move command aborted!",
//         )),
//     }
// }

impl<Data: RobotData> RobotImplGeneric<Data> {
    pub fn new(
        network: Network<Data::DeviceData>,
        log_size: usize,
        realtime_config: RealtimeConfig,
    ) -> FrankaResult<Self> {
        let current_move_generator_mode = MotionGeneratorMode::Idle;
        let controller_mode = ControllerMode::Other;
        let logger: Logger<Data::State> = Logger::new(log_size);
        let mut robot_impl = RobotImplGeneric {
            network,
            logger,
            realtime_config,
            ri_version: None,
            motion_generator_mode: None,
            current_move_motion_generator_mode: current_move_generator_mode,
            controller_mode,
            current_move_controller_mode: None,
            message_id: 0,
        };
        robot_impl.connect_robot()?;
        let state = robot_impl
            .network
            .udp_blocking_receive::<Data::StateIntern>()
            .unwrap();
        robot_impl.update_state_with(&state);
        Ok(robot_impl)
    }

    fn connect_robot(&mut self) -> Result<(), FrankaException> {
        let udp_port = self.network.get_udp_port();
        let connect_command = Data::create_connect_request(&mut self.network.command_id, udp_port);
        let command_id: u32 = self.network.tcp_send_request(connect_command);
        let connect_response: ConnectResponseWithoutHeader =
            self.network.tcp_blocking_receive_status(command_id);
        match connect_response.status {
            ConnectStatus::Success => {
                self.ri_version = Some(connect_response.version);
                Ok(())
            }
            _ => Err(FrankaException::IncompatibleLibraryVersionError {
                server_version: connect_response.version,
                library_version: Data::get_library_version(),
            }),
        }
    }
    fn update_state_with(&mut self, robot_state: &Data::StateIntern) {
        self.motion_generator_mode = Some(robot_state.get_motion_generator_mode());
        self.controller_mode = robot_state.get_controller_mode();
        self.message_id = robot_state.get_message_id();
    }
    pub fn read_once(&mut self) -> FrankaResult<Data::State> {
        while self.network.udp_receive::<Data::StateIntern>().is_some() {}
        Ok(Data::State::from(self.receive_robot_state()?))
    }
    fn receive_robot_state(&mut self) -> FrankaResult<Data::StateIntern> {
        let mut latest_accepted_state: Option<Data::StateIntern> = None;
        let mut received_state = self.network.udp_receive::<Data::StateIntern>();
        while received_state.is_some() {
            if received_state.unwrap().get_message_id()
                > match latest_accepted_state {
                    Some(s) => s.get_message_id(),
                    None => self.message_id,
                }
            {
                latest_accepted_state = Some(received_state.unwrap());
            }
            received_state = self.network.udp_receive::<Data::StateIntern>();
        }
        while latest_accepted_state.is_none() {
            received_state = Some(self.network.udp_blocking_receive()?);
            if received_state.unwrap().get_message_id() > self.message_id {
                latest_accepted_state = received_state;
            }
        }
        self.update_state_with(&latest_accepted_state.unwrap());
        Ok(latest_accepted_state.unwrap())
    }
    fn send_robot_command(
        &mut self,
        motion_command: Option<&MotionGeneratorCommand>,
        control_command: Option<&ControllerCommand>,
    ) -> FrankaResult<Option<RobotCommand>> {
        let motion_is_some = motion_command.is_some();
        let controller_is_some = control_command.is_some();
        if motion_is_some || controller_is_some {
            if motion_is_some
                && self.current_move_motion_generator_mode == MotionGeneratorMode::Idle
            {
                return Err(FrankaException::NoMotionGeneratorRunningError);
            }
            if controller_is_some
                && self.current_move_controller_mode.unwrap() != ControllerMode::ExternalController
            {
                return Err(FrankaException::NoControllerRunningError);
            }
            if self.current_move_motion_generator_mode != MotionGeneratorMode::Idle
                && self.current_move_controller_mode.unwrap() == ControllerMode::ExternalController
                && (!motion_is_some || !controller_is_some)
            {
                return Err(FrankaException::PartialCommandError);
            }
            if motion_is_some && controller_is_some {
                let command = RobotCommand {
                    message_id: self.message_id,
                    motion: (*motion_command.unwrap()).pack(),
                    control: (*control_command.unwrap()).pack(),
                };
                return match self.network.udp_send(&command) {
                    Ok(_) => Ok(Some(command)),
                    Err(e) => Err(FrankaException::NetworkException {
                        message: format!("libfranka-rs: UDP send: {}", e),
                    }),
                };
            } else if motion_is_some {
                let command = RobotCommand {
                    message_id: self.message_id,
                    motion: (*motion_command.unwrap()).pack(),
                    control: ControllerCommand { tau_J_d: [0.; 7] }.pack(),
                };
                return match self.network.udp_send(&command) {
                    Ok(_) => Ok(Some(command)),
                    Err(e) => Err(FrankaException::NetworkException {
                        message: format!("libfranka-rs: UDP send: {}", e),
                    }),
                };
            }
        }

        Ok(None)
    }
    fn motion_generator_running(&self) -> bool {
        self.motion_generator_mode.unwrap() != MotionGeneratorMode::Idle
    }
    fn controller_running(&self) -> bool {
        self.controller_mode == ControllerMode::ExternalController
    }
    pub fn server_version(&self) -> u16 {
        self.ri_version.unwrap()
    }
    fn execute_move_command(
        &mut self,
        controller_mode: &MoveControllerMode,
        motion_generator_mode: &MoveMotionGeneratorMode,
        maximum_path_deviation: &MoveDeviation,
        maximum_goal_deviation: &MoveDeviation,
    ) -> FrankaResult<u32> {
        // let connect_command = MoveRequestWithPandaHeader {
        //     header: self.network.create_header_for_panda(
        //         PandaCommandEnum::Move,
        //         size_of::<MoveRequestWithPandaHeader>(),
        //     ),
        //     request: MoveRequest::new(
        //         *controller_mode,
        //         *motion_generator_mode,
        //         *maximum_path_deviation,
        //         *maximum_goal_deviation,
        //     ),
        // };
        let command = Data::create_move_request(
            &mut self.network.command_id,
            MoveRequest::new(
                *controller_mode,
                *motion_generator_mode,
                *maximum_path_deviation,
                *maximum_goal_deviation,
            ),
        );
        let command_id: u32 = self.network.tcp_send_request(command);
        let status = self.network.tcp_blocking_receive_status(command_id);
        Data::handle_command_move_status(status)?;
        Ok(command_id)
    }
    fn execute_stop_command(&mut self) -> FrankaResult<u32> {
        let command = Data::create_stop_request(&mut self.network.command_id);
        let command_id: u32 = self.network.tcp_send_request(command);
        let status = self.network.tcp_blocking_receive_status(command_id);
        Data::handle_command_stop_move_status(status)?;
        Ok(command_id)
    }
}
