// Copyright (c) 2021 Marco Boneberger
// Licensed under the EUPL-1.2-or-later
use std::mem::size_of;

use crate::exception::{create_command_exception, FrankaException, FrankaResult};
use crate::model::library_downloader::LibraryDownloader;
use crate::model::Model;
use crate::network::{FR3Data, Network, PandaData, RobotData};
use crate::robot::control_types::RealtimeConfig;
use crate::robot::errors::FrankaErrors;
use crate::robot::logger::{Logger, Record};

use crate::robot::robot_control::RobotControl;
use crate::robot::robot_state::{FR3State, PandaState, RobotState};
use crate::robot::service_types::{ConnectRequest, ConnectRequestWithHeader, ConnectResponse, ConnectStatus, MoveControllerMode, MoveDeviation, MoveMotionGeneratorMode, MoveRequest, MoveRequestWithHeader, MoveResponse, MoveStatus, PandaCommandEnum, PandaCommandHeader, StopMoveRequestWithHeader, StopMoveResponse, StopMoveStatus, VERSION};
use crate::robot::types::{
    ControllerCommand, ControllerMode, MotionGeneratorCommand, MotionGeneratorMode, RobotCommand,
    RobotMode, PandaStateIntern,
};
use std::fs::remove_file;
use std::path::Path;

pub trait RobotImplementation : RobotControl<State2=Self::State> {
    type State:RobotState;
    fn update2(
        &mut self,
        motion_command: Option<&MotionGeneratorCommand>,
        control_command: Option<&ControllerCommand>,
    ) -> FrankaResult<Self::State>;
    fn read_once2(&mut self) -> FrankaResult<Self::State>;
    fn load_model2(&mut self, persistent: bool) -> FrankaResult<Model>;
    fn server_version2(&self) -> u16;
}

impl RobotImplementation for RobotImplPanda {
    type State = PandaState;

    fn update2(&mut self, motion_command: Option<&MotionGeneratorCommand>, control_command: Option<&ControllerCommand>) -> FrankaResult<Self::State> {
        let robot_command = self.send_robot_command(motion_command, control_command)?;
        let state = PandaState::from(self.receive_robot_state()?);
        if let Some(command) = robot_command {
            self.logger.log(&state, &command);
        }
        Ok(state)
    }

    fn read_once2(&mut self) -> FrankaResult<Self::State> {
        while self.network.udp_receive::<PandaStateIntern>().is_some() {}
        Ok(PandaState::from(self.receive_robot_state()?))
    }
    fn load_model2(&mut self, persistent: bool) -> FrankaResult<Model> {
        let model_file = Path::new("/tmp/model.so");
        let model_already_downloaded = model_file.exists();
        if !model_already_downloaded {
            LibraryDownloader::download(&mut self.network, model_file)?;
        }
        let model = Model::new(model_file, None)?;
        if !persistent && model_already_downloaded {
            remove_file(model_file).expect("Could not delete file");
        }
        Ok(model)
    }
    fn server_version2(&self) -> u16 {
        self.ri_version.unwrap()
    }
}

impl RobotImplementation for RobotImplFR3 {
    type State = FR3State;

    fn update2(&mut self, motion_command: Option<&MotionGeneratorCommand>, control_command: Option<&ControllerCommand>) -> FrankaResult<Self::State> {
        let robot_command = self.send_robot_command(motion_command, control_command)?;
        let state = FR3State::from(self.receive_robot_state()?);
        if let Some(command) = robot_command {
            // self.logger.log(&state, &command); // todo fix logger
        }
        Ok(state)
    }

    fn read_once2(&mut self) -> FrankaResult<Self::State> {
        while self.network.udp_receive::<PandaStateIntern>().is_some() {}
        Ok(FR3State::from(self.receive_robot_state()?))
    }
    fn load_model2(&mut self, persistent: bool) -> FrankaResult<Model> {
        let model_file = Path::new("/tmp/model.so");
        let model_already_downloaded = model_file.exists();
        if !model_already_downloaded {
            LibraryDownloader::download2(&mut self.network, model_file)?;
        }
        let model = Model::new(model_file, None)?;
        if !persistent && model_already_downloaded {
            remove_file(model_file).expect("Could not delete file");
        }
        Ok(model)
    }
    fn server_version2(&self) -> u16 {
        self.ri_version.unwrap()
    }
}

pub struct RobotImplPanda {
    pub network: Network<PandaData>,
    logger: Logger,
    realtime_config: RealtimeConfig,
    ri_version: Option<u16>,
    motion_generator_mode: Option<MotionGeneratorMode>,
    current_move_motion_generator_mode: MotionGeneratorMode,
    controller_mode: ControllerMode,
    current_move_controller_mode: Option<ControllerMode>,
    message_id: u64,
}

pub struct RobotImplFR3 {
    pub network: Network<FR3Data>,
    logger: Logger,
    realtime_config: RealtimeConfig,
    ri_version: Option<u16>,
    motion_generator_mode: Option<MotionGeneratorMode>,
    current_move_motion_generator_mode: MotionGeneratorMode,
    controller_mode: ControllerMode,
    current_move_controller_mode: Option<ControllerMode>,
    message_id: u64,
}

fn handle_command_response_move(response: &MoveResponse) -> Result<(), FrankaException> {
    match response.status {
        MoveStatus::Success => Ok(()),
        MoveStatus::MotionStarted => {
            //todo handle motion_generator_running == true
            Ok(())
        }
        MoveStatus::EmergencyAborted => Err(create_command_exception(
            "libfranka-rs: Move command aborted: User Stop pressed!",
        )),
        MoveStatus::ReflexAborted => Err(create_command_exception(
            "libfranka-rs: Move command aborted: motion aborted by reflex!",
        )),
        MoveStatus::InputErrorAborted => Err(create_command_exception(
            "libfranka-rs: Move command aborted: invalid input provided!",
        )),
        MoveStatus::CommandNotPossibleRejected => Err(create_command_exception(
            "libfranka-rs: Move command rejected: command not possible in the current mode!",
        )),
        MoveStatus::StartAtSingularPoseRejected => Err(create_command_exception(
            "libfranka-rs: Move command rejected: cannot start at singular pose!",
        )),
        MoveStatus::InvalidArgumentRejected => Err(create_command_exception(
            "libfranka-rs: Move command rejected: maximum path deviation out of range!",
        )),
        MoveStatus::Preempted => Err(create_command_exception(
            "libfranka-rs: Move command preempted!",
        )),
        MoveStatus::Aborted => Err(create_command_exception(
            "libfranka-rs: Move command aborted!",
        )),
    }
}

fn handle_command_response_stop(response: &StopMoveResponse) -> Result<(), FrankaException> {
    match response.status {
        StopMoveStatus::Success => Ok(()),
        StopMoveStatus::EmergencyAborted => Err(create_command_exception(
            "libfranka-rs: Stop command aborted: User Stop pressed!",
        )),
        StopMoveStatus::ReflexAborted => Err(create_command_exception(
            "libfranka-rs: Stop command aborted: motion aborted by reflex!",
        )),
        StopMoveStatus::CommandNotPossibleRejected => Err(create_command_exception(
            "libfranka-rs: Stop command rejected: command not possible in the current mode!",
        )),
        StopMoveStatus::Aborted => Err(create_command_exception(
            "libfranka-rs: Stop command aborted!",
        )),
    }
}

impl RobotImplPanda {
    pub fn new(
        network: Network<PandaData>,
        log_size: usize,
        realtime_config: RealtimeConfig,
    ) -> FrankaResult<Self> {
        let current_move_generator_mode = MotionGeneratorMode::Idle;
        let controller_mode = ControllerMode::Other;
        let logger = Logger::new(log_size);
        let mut robot_impl = RobotImplPanda {
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
            .udp_blocking_receive::<PandaStateIntern>()
            .unwrap();
        robot_impl.update_state_with(&state);
        Ok(robot_impl)
    }
    fn connect_robot(&mut self) -> Result<(), FrankaException> {
        let connect_command = ConnectRequestWithHeader {
            header:PandaData::create_header(&mut self.network.command_id,
                PandaCommandEnum::Connect,
                size_of::<ConnectRequestWithHeader>(),
            ),
            request: ConnectRequest::new(self.network.get_udp_port()),
        };
        let command_id: u32 = self.network.tcp_send_request(connect_command);
        let connect_response: ConnectResponse =
            self.network.tcp_blocking_receive_response(command_id);
        match connect_response.status {
            ConnectStatus::Success => {
                self.ri_version = Some(connect_response.version);
                Ok(())
            }
            _ => Err(FrankaException::IncompatibleLibraryVersionError {
                server_version: connect_response.version,
                library_version: VERSION,
            }),
        }
    }
    fn update_state_with(&mut self, robot_state: &PandaStateIntern) {
        self.motion_generator_mode = Some(robot_state.motion_generator_mode);
        self.controller_mode = robot_state.controller_mode;
        self.message_id = robot_state.message_id;
    }
    pub fn read_once(&mut self) -> FrankaResult<PandaState> {
        while self.network.udp_receive::<PandaStateIntern>().is_some() {}
        Ok(PandaState::from(self.receive_robot_state()?))
    }
    fn receive_robot_state(&mut self) -> FrankaResult<PandaStateIntern> {
        let mut latest_accepted_state: Option<PandaStateIntern> = None;
        let mut received_state = self.network.udp_receive::<PandaStateIntern>();
        while received_state.is_some() {
            if received_state.unwrap().message_id
                > match latest_accepted_state {
                    Some(s) => s.message_id,
                    None => self.message_id,
                }
            {
                latest_accepted_state = Some(received_state.unwrap());
            }
            received_state = self.network.udp_receive::<PandaStateIntern>();
        }
        while latest_accepted_state.is_none() {
            received_state = Some(self.network.udp_blocking_receive()?);
            if received_state.unwrap().message_id > self.message_id {
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
    pub fn load_model(&mut self, persistent: bool) -> FrankaResult<Model> {
        let model_file = Path::new("/tmp/model.so");
        let model_already_downloaded = model_file.exists();
        if !model_already_downloaded {
            LibraryDownloader::download(&mut self.network, model_file)?;
        }
        let model = Model::new(model_file, None)?;
        if !persistent && model_already_downloaded {
            remove_file(model_file).expect("Could not delete file");
        }
        Ok(model)
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
        let connect_command = MoveRequestWithHeader {
            header: self.network.create_header_for_panda(
                PandaCommandEnum::Move,
                size_of::<MoveRequestWithHeader>(),
            ),
            request: MoveRequest::new(
                *controller_mode,
                *motion_generator_mode,
                *maximum_path_deviation,
                *maximum_goal_deviation,
            ),
        };
        let command_id: u32 = self.network.tcp_send_request(connect_command);
        let response: MoveResponse = self.network.tcp_blocking_receive_response(command_id);
        handle_command_response_move(&response)?;
        Ok(command_id)
    }
    fn execute_stop_command(&mut self) -> FrankaResult<u32> {
        let command = StopMoveRequestWithHeader {
            header: self.network.create_header_for_panda(
                PandaCommandEnum::StopMove,
                size_of::<StopMoveRequestWithHeader>(),
            ),
        };
        let command_id: u32 = self.network.tcp_send_request(command);
        let response: StopMoveResponse = self.network.tcp_blocking_receive_response(command_id);
        handle_command_response_stop(&response)?;
        Ok(command_id)
    }
}

impl RobotImplFR3 {
    pub fn new(
        network: Network<FR3Data>,
        log_size: usize,
        realtime_config: RealtimeConfig,
    ) -> FrankaResult<Self> {
        let current_move_generator_mode = MotionGeneratorMode::Idle;
        let controller_mode = ControllerMode::Other;
        let logger = Logger::new(log_size);
        let mut robot_impl = RobotImplFR3 {
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
            .udp_blocking_receive::<PandaStateIntern>()
            .unwrap();
        robot_impl.update_state_with(&state);
        Ok(robot_impl)
    }
    fn connect_robot(&mut self) -> Result<(), FrankaException> {
        let connect_command = ConnectRequestWithHeader {
            header: self.network.create_header_for_panda(
                PandaCommandEnum::Connect,
                size_of::<ConnectRequestWithHeader>(),
            ),
            request: ConnectRequest::new(self.network.get_udp_port()),
        };
        let command_id: u32 = self.network.tcp_send_request(connect_command);
        let connect_response: ConnectResponse =
            self.network.tcp_blocking_receive_response(command_id);
        match connect_response.status {
            ConnectStatus::Success => {
                self.ri_version = Some(connect_response.version);
                Ok(())
            }
            _ => Err(FrankaException::IncompatibleLibraryVersionError {
                server_version: connect_response.version,
                library_version: VERSION,
            }),
        }
    }
    fn update_state_with(&mut self, robot_state: &PandaStateIntern) {
        self.motion_generator_mode = Some(robot_state.motion_generator_mode);
        self.controller_mode = robot_state.controller_mode;
        self.message_id = robot_state.message_id;
    }
    pub fn read_once(&mut self) -> FrankaResult<FR3State> {
        while self.network.udp_receive::<PandaStateIntern>().is_some() {}
        Ok(FR3State::from(self.receive_robot_state()?))
    }
    fn receive_robot_state(&mut self) -> FrankaResult<PandaStateIntern> {
        let mut latest_accepted_state: Option<PandaStateIntern> = None;
        let mut received_state = self.network.udp_receive::<PandaStateIntern>();
        while received_state.is_some() {
            if received_state.unwrap().message_id
                > match latest_accepted_state {
                Some(s) => s.message_id,
                None => self.message_id,
            }
            {
                latest_accepted_state = Some(received_state.unwrap());
            }
            received_state = self.network.udp_receive::<PandaStateIntern>();
        }
        while latest_accepted_state.is_none() {
            received_state = Some(self.network.udp_blocking_receive()?);
            if received_state.unwrap().message_id > self.message_id {
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
    pub fn load_model(&mut self, persistent: bool) -> FrankaResult<Model> {
        let model_file = Path::new("/tmp/model.so");
        let model_already_downloaded = model_file.exists();
        if !model_already_downloaded {
            LibraryDownloader::download2(&mut self.network, model_file)?;
        }
        let model = Model::new(model_file, None)?;
        if !persistent && model_already_downloaded {
            remove_file(model_file).expect("Could not delete file");
        }
        Ok(model)
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
        let connect_command = MoveRequestWithHeader {
            header: self.network.create_header_for_panda(
                PandaCommandEnum::Move,
                size_of::<MoveRequestWithHeader>(),
            ),
            request: MoveRequest::new(
                *controller_mode,
                *motion_generator_mode,
                *maximum_path_deviation,
                *maximum_goal_deviation,
            ),
        };
        let command_id: u32 = self.network.tcp_send_request(connect_command);
        let response: MoveResponse = self.network.tcp_blocking_receive_response(command_id);
        handle_command_response_move(&response)?;
        Ok(command_id)
    }
    fn execute_stop_command(&mut self) -> FrankaResult<u32> {
        let command = StopMoveRequestWithHeader {
            header: self.network.create_header_for_panda(
                PandaCommandEnum::StopMove,
                size_of::<StopMoveRequestWithHeader>(),
            ),
        };
        let command_id: u32 = self.network.tcp_send_request(command);
        let response: StopMoveResponse = self.network.tcp_blocking_receive_response(command_id);
        handle_command_response_stop(&response)?;
        Ok(command_id)
    }
}

impl RobotControl for RobotImplPanda {
    type State2 = PandaState;

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
                .tcp_receive_response(move_command_id, |x| handle_command_response_move(&x))
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
        let response: MoveResponse = self.network.tcp_blocking_receive_response(motion_id);
        if response.status == MoveStatus::ReflexAborted {
            return Err(create_control_exception(
                String::from("Motion finished commanded, but the robot is still moving!"),
                &response.status,
                &robot_state.last_motion_errors,
                self.logger.flush(),
            ));
        }
        match handle_command_response_move(&response) {
            Ok(_) => {}
            Err(FrankaException::CommandException { message }) => {
                return Err(create_control_exception(
                    message,
                    &response.status,
                    &robot_state.last_motion_errors,
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
            .tcp_receive_response(motion_id, |_x: MoveResponse| Ok(()))
            .expect("This should be impossible as the handler always returns Ok(())");
        self.current_move_motion_generator_mode = MotionGeneratorMode::Idle;
        self.current_move_controller_mode = Some(ControllerMode::Other);
    }

    fn update(
        &mut self,
        motion_command: Option<&MotionGeneratorCommand>,
        control_command: Option<&ControllerCommand>,
    ) -> FrankaResult<PandaState> {
        let robot_command = self.send_robot_command(motion_command, control_command)?;
        let state = PandaState::from(self.receive_robot_state()?);
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
        robot_state: &PandaState,
        motion_id: u32,
    ) -> FrankaResult<()> {
        if robot_state.robot_mode != RobotMode::Move
            || self.motion_generator_mode.unwrap() != self.current_move_motion_generator_mode
            || self.controller_mode != self.current_move_controller_mode.unwrap()
        {
            let response = self.network.tcp_blocking_receive_response(motion_id);
            let result = handle_command_response_move(&response);
            return match result {
                Err(error) => Err(create_control_exception(
                    error.to_string(),
                    &response.status,
                    &robot_state.last_motion_errors,
                    self.logger.flush(),
                )),
                Ok(_) => panic!("Unexpected reply to a Move command"),
            };
        }
        Ok(())
    }
}

impl RobotControl for RobotImplFR3 {
    type State2 = FR3State;

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
                .tcp_receive_response(move_command_id, |x| handle_command_response_move(&x))
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
        let response: MoveResponse = self.network.tcp_blocking_receive_response(motion_id);
        if response.status == MoveStatus::ReflexAborted {
            return Err(create_control_exception(
                String::from("Motion finished commanded, but the robot is still moving!"),
                &response.status,
                &robot_state.last_motion_errors,
                self.logger.flush(),
            ));
        }
        match handle_command_response_move(&response) {
            Ok(_) => {}
            Err(FrankaException::CommandException { message }) => {
                return Err(create_control_exception(
                    message,
                    &response.status,
                    &robot_state.last_motion_errors,
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
            .tcp_receive_response(motion_id, |_x: MoveResponse| Ok(()))
            .expect("This should be impossible as the handler always returns Ok(())");
        self.current_move_motion_generator_mode = MotionGeneratorMode::Idle;
        self.current_move_controller_mode = Some(ControllerMode::Other);
    }

    fn update(
        &mut self,
        motion_command: Option<&MotionGeneratorCommand>,
        control_command: Option<&ControllerCommand>,
    ) -> FrankaResult<FR3State> {
        let robot_command = self.send_robot_command(motion_command, control_command)?;
        let state = FR3State::from(self.receive_robot_state()?);
        if let Some(command) = robot_command {
            // self.logger.log(&state, &command); TODO(care about logger)
        }
        Ok(state)
    }

    fn realtime_config(&self) -> RealtimeConfig {
        self.realtime_config
    }

    fn throw_on_motion_error(
        &mut self,
        robot_state: &Self::State2,
        motion_id: u32,
    ) -> FrankaResult<()> {
        if robot_state.robot_mode != RobotMode::Move
            || self.motion_generator_mode.unwrap() != self.current_move_motion_generator_mode
            || self.controller_mode != self.current_move_controller_mode.unwrap()
        {
            let response = self.network.tcp_blocking_receive_response(motion_id);
            let result = handle_command_response_move(&response);
            return match result {
                Err(error) => Err(create_control_exception(
                    error.to_string(),
                    &response.status,
                    &robot_state.last_motion_errors,
                    self.logger.flush(),
                )),
                Ok(_) => panic!("Unexpected reply to a Move command"),
            };
        }
        Ok(())
    }
}

// impl MotionGeneratorTrait for RobotImpl {
//     fn get_motion_generator_mode() -> MoveMotionGeneratorMode {
//         MoveMotionGeneratorMode::JointVelocity
//     }
// }

fn create_control_exception(
    message: String,
    move_status: &MoveStatus,
    reflex_reasons: &FrankaErrors,
    log: Vec<Record>,
) -> FrankaException {
    let mut exception_string = String::from(&message);
    if move_status == &MoveStatus::ReflexAborted {
        exception_string += " ";
        exception_string += reflex_reasons.to_string().as_str();
        if log.len() >= 2 {
            let lost_packets: u128 =
                (log.last().unwrap().state.time - log[log.len() - 2].state.time).as_millis() - 1;
            exception_string += format!(
                "\ncontrol_command_success_rate: {}",
                log[log.len() - 2].state.control_command_success_rate
                    * (1. - lost_packets as f64 / 100.)
            )
            .as_str();
            if lost_packets > 0 {
                exception_string += format!(
                    " packets lost in a row in the last sample: {}",
                    lost_packets
                )
                .as_str();
            }
        }
    }
    FrankaException::ControlException {
        error: exception_string,
        log: Some(log),
    }
}
