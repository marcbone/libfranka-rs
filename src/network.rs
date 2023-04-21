// Copyright (c) 2021 Marco Boneberger
// Licensed under the EUPL-1.2-or-later
#![allow(dead_code)]

extern crate libc;
extern crate nix;

use std::collections::HashMap;
use std::error::Error;
use std::fmt::Debug;
use std::io::{Read, Write};
use std::marker::PhantomData;
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

use crate::exception::{create_command_exception, FrankaException, FrankaResult};
use crate::gripper::types::{
    CommandHeader, GripperCommandEnum, GripperCommandHeader, GRIPPER_VERSION,
};
use crate::robot::errors::FrankaErrors;
use crate::robot::logger::Record;
use crate::robot::robot_state::AbstractRobotState;
use crate::robot::service_types::{
    AutomaticErrorRecoveryStatusFR3, AutomaticErrorRecoveryStatusPanda, ConnectRequest,
    ConnectRequestWithFR3Header, ConnectRequestWithPandaHeader, FR3CommandEnum, FR3CommandHeader,
    GetterSetterStatusFR3, GetterSetterStatusPanda, LoadModelLibraryRequest,
    LoadModelLibraryRequestWithFR3Header, LoadModelLibraryRequestWithPandaHeader,
    LoadModelLibraryStatus, MoveRequest, MoveRequestWithFR3Header, MoveRequestWithPandaHeader,
    MoveStatusFR3, MoveStatusPanda, PandaCommandEnum, PandaCommandHeader, RobotHeader,
    SetCartesianImpedanceRequest, SetCartesianImpedanceRequestWithFR3Header,
    SetCartesianImpedanceRequestWithPandaHeader, SetCollisionBehaviorRequest,
    SetCollisionBehaviorRequestWithFR3Header, SetCollisionBehaviorRequestWithPandaHeader,
    SetEeToKRequest, SetEeToKRequestWithFR3Header, SetEeToKRequestWithPandaHeader,
    SetGuidingModeRequest, SetGuidingModeRequestWithFR3Header,
    SetGuidingModeRequestWithPandaHeader, SetJointImpedanceRequest,
    SetJointImpedanceRequestWithFR3Header, SetJointImpedanceRequestWithPandaHeader, SetLoadRequest,
    SetLoadRequestWithFR3Header, SetLoadRequestWithPandaHeader, SetNeToEeRequest,
    SetNeToEeRequestWithFR3Header, SetNeToEeRequestWithPandaHeader, StopMoveStatusFR3,
    StopMoveStatusPanda, FR3_VERSION, PANDA_VERSION,
};
use crate::robot::types::{AbstractRobotStateIntern, FR3StateIntern, PandaStateIntern};
use crate::{FR3Model, PandaModel, RobotModel, RobotState};

const CLIENT: Token = Token(1);

pub enum NetworkType {
    Panda,
    FR3,
    Gripper,
}

pub trait DeviceData {
    type CommandHeader: CommandHeader;
    type CommandEnum;
    fn create_header(
        command_id: &mut u32,
        command: Self::CommandEnum,
        size: usize,
    ) -> Self::CommandHeader;
    fn get_library_version() -> u16;
}

pub trait RobotData: DeviceData {
    type DeviceData: DeviceData;
    type Header: RobotHeader;
    type State: AbstractRobotState + From<Self::StateIntern> + From<RobotState>;
    type StateIntern: Debug + DeserializeOwned + Serialize + AbstractRobotStateIntern + 'static;
    type Model: RobotModel;
    type LoadModelRequestWithHeader: MessageCommand
        + Serialize
        + From<(u32, LoadModelLibraryRequest)>;
    type SetCollisionBehaviorRequestWithHeader: MessageCommand
        + Serialize
        + From<(u32, SetCollisionBehaviorRequest)>;
    type SetLoadRequestWithHeader: MessageCommand + Serialize + From<(u32, SetLoadRequest)>;
    type SetJointImpedanceRequestWithHeader: MessageCommand
        + Serialize
        + From<(u32, SetJointImpedanceRequest)>;
    type SetCartesianImpedanceRequestWithHeader: MessageCommand
        + Serialize
        + From<(u32, SetCartesianImpedanceRequest)>;
    type SetGuidingModeRequestWithHeader: MessageCommand
        + Serialize
        + From<(u32, SetGuidingModeRequest)>;
    type ConnectRequestWithHeader: MessageCommand + Serialize + From<(u32, ConnectRequest)>;
    type SetEeToKRequestWithHeader: MessageCommand + Serialize + From<(u32, SetEeToKRequest)>;
    type SetNeToEeRequestWithHeader: MessageCommand + Serialize + From<(u32, SetNeToEeRequest)>;
    type MoveRequestWithHeader: MessageCommand + Serialize + From<(u32, MoveRequest)>;
    type MoveStatus: DeserializeOwned + PartialEq<Self::MoveStatus> + Copy + Clone + 'static; // todo is this static fine here?
    type GetterSetterStatus: DeserializeOwned + Copy + Clone + 'static; // todo is this static fine here?
    type StopMoveStatus: DeserializeOwned + Copy + Clone + 'static; // todo is this static fine here?
    type AutomaticErrorRecoveryStatus: DeserializeOwned + Copy + Clone + 'static; // todo is this static fine here?

    fn create_model_library_request(
        command_id: &mut u32,
        request: LoadModelLibraryRequest,
    ) -> Self::LoadModelRequestWithHeader {
        *command_id += 1;
        (*command_id - 1, request).into()
    }
    fn create_set_collision_behavior_request(
        command_id: &mut u32,
        request: SetCollisionBehaviorRequest,
    ) -> Self::SetCollisionBehaviorRequestWithHeader {
        *command_id += 1;
        (*command_id - 1, request).into()
    }
    fn create_set_load_request(
        command_id: &mut u32,
        request: SetLoadRequest,
    ) -> Self::SetLoadRequestWithHeader {
        *command_id += 1;
        (*command_id - 1, request).into()
    }

    fn create_set_joint_impedance_request(
        command_id: &mut u32,
        request: SetJointImpedanceRequest,
    ) -> Self::SetJointImpedanceRequestWithHeader {
        *command_id += 1;
        (*command_id - 1, request).into()
    }

    fn create_set_cartesian_impedance_request(
        command_id: &mut u32,
        request: SetCartesianImpedanceRequest,
    ) -> Self::SetCartesianImpedanceRequestWithHeader {
        *command_id += 1;
        (*command_id - 1, request).into()
    }

    fn create_set_guiding_mode_request(
        command_id: &mut u32,
        request: SetGuidingModeRequest,
    ) -> Self::SetGuidingModeRequestWithHeader {
        *command_id += 1;
        (*command_id - 1, request).into()
    }

    fn create_set_ee_to_k_request(
        command_id: &mut u32,
        request: SetEeToKRequest,
    ) -> Self::SetEeToKRequestWithHeader {
        *command_id += 1;
        (*command_id - 1, request).into()
    }

    fn create_set_ne_to_ee_request(
        command_id: &mut u32,
        request: SetNeToEeRequest,
    ) -> Self::SetNeToEeRequestWithHeader {
        *command_id += 1;
        (*command_id - 1, request).into()
    }

    fn create_move_request(
        command_id: &mut u32,
        request: MoveRequest,
    ) -> Self::MoveRequestWithHeader {
        *command_id += 1;
        (*command_id - 1, request).into()
    }

    fn create_connect_request(
        command_id: &mut u32,
        udp_port: u16,
    ) -> Self::ConnectRequestWithHeader;

    fn create_automatic_error_recovery_request(command_id: &mut u32) -> Self::CommandHeader;
    fn create_stop_request(command_id: &mut u32) -> Self::CommandHeader;

    fn handle_command_move_status(status: Self::MoveStatus) -> Result<(), FrankaException>;
    fn create_control_exception(
        message: String,
        move_status: Self::MoveStatus,
        reflex_reasons: &FrankaErrors,
        log: Vec<Record<Self::State>>,
    ) -> FrankaException;
    fn create_control_exception_if_reflex_aborted(
        message: String,
        move_status: Self::MoveStatus,
        reflex_reasons: &FrankaErrors,
        log: Vec<Record<Self::State>>,
    ) -> FrankaResult<()>;

    fn handle_getter_setter_status(status: Self::GetterSetterStatus) -> FrankaResult<()>;
    fn handle_automatic_error_recovery_status(
        status: Self::AutomaticErrorRecoveryStatus,
    ) -> FrankaResult<()>;
    fn handle_command_stop_move_status(status: Self::StopMoveStatus)
        -> Result<(), FrankaException>;
}

pub struct PandaData {}

pub struct FR3Data {}

pub struct GripperData {}

impl DeviceData for PandaData {
    type CommandHeader = PandaCommandHeader;
    type CommandEnum = PandaCommandEnum;
    fn create_header(
        command_id: &mut u32,
        command: Self::CommandEnum,
        size: usize,
    ) -> Self::CommandHeader {
        let header = PandaCommandHeader::new(command, *command_id, size as u32);
        *command_id += 1;
        header
    }

    fn get_library_version() -> u16 {
        PANDA_VERSION
    }
}

impl RobotData for PandaData {
    type DeviceData = Self;
    type Header = PandaCommandHeader;
    type State = RobotState;
    type StateIntern = PandaStateIntern;
    type Model = PandaModel;
    type LoadModelRequestWithHeader = LoadModelLibraryRequestWithPandaHeader;
    type SetCollisionBehaviorRequestWithHeader = SetCollisionBehaviorRequestWithPandaHeader;
    type SetLoadRequestWithHeader = SetLoadRequestWithPandaHeader;
    type SetJointImpedanceRequestWithHeader = SetJointImpedanceRequestWithPandaHeader;
    type SetCartesianImpedanceRequestWithHeader = SetCartesianImpedanceRequestWithPandaHeader;
    type SetGuidingModeRequestWithHeader = SetGuidingModeRequestWithPandaHeader;
    type ConnectRequestWithHeader = ConnectRequestWithPandaHeader;
    type SetEeToKRequestWithHeader = SetEeToKRequestWithPandaHeader;
    type SetNeToEeRequestWithHeader = SetNeToEeRequestWithPandaHeader;

    type MoveRequestWithHeader = MoveRequestWithPandaHeader;

    type MoveStatus = MoveStatusPanda;

    type GetterSetterStatus = GetterSetterStatusPanda;

    type StopMoveStatus = StopMoveStatusPanda;

    type AutomaticErrorRecoveryStatus = AutomaticErrorRecoveryStatusPanda;

    fn create_connect_request(
        command_id: &mut u32,
        udp_port: u16,
    ) -> Self::ConnectRequestWithHeader {
        let request = ConnectRequest::new(udp_port, PANDA_VERSION);
        ConnectRequestWithPandaHeader {
            header: Self::create_header(
                command_id,
                PandaCommandEnum::Connect,
                size_of::<Self::ConnectRequestWithHeader>(),
            ),
            request,
        }
    }

    fn create_automatic_error_recovery_request(command_id: &mut u32) -> Self::CommandHeader {
        Self::create_header(
            command_id,
            PandaCommandEnum::AutomaticErrorRecovery,
            size_of::<PandaCommandHeader>(),
        )
    }

    fn create_stop_request(command_id: &mut u32) -> Self::CommandHeader {
        Self::create_header(
            command_id,
            PandaCommandEnum::StopMove,
            size_of::<PandaCommandHeader>(),
        )
    }

    fn handle_command_move_status(status: Self::MoveStatus) -> Result<(), FrankaException> {
        match status {
            MoveStatusPanda::Success => Ok(()),
            MoveStatusPanda::MotionStarted => {
                //todo handle motion_generator_running == true
                Ok(())
            }
            MoveStatusPanda::EmergencyAborted => Err(create_command_exception(
                "libfranka-rs: Move command aborted: User Stop pressed!",
            )),
            MoveStatusPanda::ReflexAborted => Err(create_command_exception(
                "libfranka-rs: Move command aborted: motion aborted by reflex!",
            )),
            MoveStatusPanda::InputErrorAborted => Err(create_command_exception(
                "libfranka-rs: Move command aborted: invalid input provided!",
            )),
            MoveStatusPanda::CommandNotPossibleRejected => Err(create_command_exception(
                "libfranka-rs: Move command rejected: command not possible in the current mode!",
            )),
            MoveStatusPanda::StartAtSingularPoseRejected => Err(create_command_exception(
                "libfranka-rs: Move command rejected: cannot start at singular pose!",
            )),
            MoveStatusPanda::InvalidArgumentRejected => Err(create_command_exception(
                "libfranka-rs: Move command rejected: maximum path deviation out of range!",
            )),
            MoveStatusPanda::Preempted => Err(create_command_exception(
                "libfranka-rs: Move command preempted!",
            )),
            MoveStatusPanda::Aborted => Err(create_command_exception(
                "libfranka-rs: Move command aborted!",
            )),
        }
    }

    fn create_control_exception(
        message: String,
        move_status: Self::MoveStatus,
        reflex_reasons: &FrankaErrors,
        log: Vec<Record<Self::State>>,
    ) -> FrankaException {
        let mut exception_string = String::from(&message);
        if move_status == MoveStatusPanda::ReflexAborted {
            exception_string += " ";
            exception_string += reflex_reasons.to_string().as_str();
            if log.len() >= 2 {
                let lost_packets: u128 =
                    (log.last().unwrap().state.time - log[log.len() - 2].state.time).as_millis()
                        - 1;
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
    fn create_control_exception_if_reflex_aborted(
        message: String,
        move_status: Self::MoveStatus,
        reflex_reasons: &FrankaErrors,
        log: Vec<Record<Self::State>>,
    ) -> FrankaResult<()> {
        // todo think about if option is a good return type
        if move_status == MoveStatusPanda::ReflexAborted {
            return Err(Self::create_control_exception(
                message,
                move_status,
                reflex_reasons,
                log,
            ));
        }
        Ok(())
    }

    fn handle_getter_setter_status(status: Self::GetterSetterStatus) -> FrankaResult<()> {
        match status {
            GetterSetterStatusPanda::Success => Ok(()),
            GetterSetterStatusPanda::CommandNotPossibleRejected => Err(create_command_exception(
                "libfranka-rs: command rejected: command not possible in current mode",
            )),
            GetterSetterStatusPanda::InvalidArgumentRejected => Err(create_command_exception(
                "libfranka-rs: command rejected: invalid argument!",
            )),
        }
    }

    fn handle_automatic_error_recovery_status(
        status: Self::AutomaticErrorRecoveryStatus,
    ) -> FrankaResult<()> {
        match status {
            AutomaticErrorRecoveryStatusPanda::Success => Ok(()),
            AutomaticErrorRecoveryStatusPanda::EmergencyAborted => Err(create_command_exception(
                "libfranka-rs: command aborted: User Stop pressed!",
            )),
            AutomaticErrorRecoveryStatusPanda::ReflexAborted => Err(create_command_exception(
                "libfranka-rs: command aborted: motion aborted by reflex!",
            )),
            AutomaticErrorRecoveryStatusPanda::CommandNotPossibleRejected => {
                Err(create_command_exception(
                    "libfranka-rs: command rejected: command not possible in current mode",
                ))
            }
            AutomaticErrorRecoveryStatusPanda::ManualErrorRecoveryRequiredRejected => {
                Err(create_command_exception(
                    "libfranka-rs: command rejected: manual error recovery required!",
                ))
            }
            AutomaticErrorRecoveryStatusPanda::Aborted => {
                Err(create_command_exception("libfranka-rs: command aborted!"))
            }
        }
    }

    fn handle_command_stop_move_status(
        status: Self::StopMoveStatus,
    ) -> Result<(), FrankaException> {
        match status {
            StopMoveStatusPanda::Success => Ok(()),
            StopMoveStatusPanda::EmergencyAborted => Err(create_command_exception(
                "libfranka-rs: Stop command aborted: User Stop pressed!",
            )),
            StopMoveStatusPanda::ReflexAborted => Err(create_command_exception(
                "libfranka-rs: Stop command aborted: motion aborted by reflex!",
            )),
            StopMoveStatusPanda::CommandNotPossibleRejected => Err(create_command_exception(
                "libfranka-rs: Stop command rejected: command not possible in the current mode!",
            )),
            StopMoveStatusPanda::Aborted => Err(create_command_exception(
                "libfranka-rs: Stop command aborted!",
            )),
        }
    }
}
impl DeviceData for FR3Data {
    type CommandHeader = FR3CommandHeader;
    type CommandEnum = FR3CommandEnum;
    fn create_header(
        command_id: &mut u32,
        command: Self::CommandEnum,
        size: usize,
    ) -> Self::CommandHeader {
        let header = FR3CommandHeader::new(command, *command_id, size as u32);
        *command_id += 1;
        header
    }

    fn get_library_version() -> u16 {
        FR3_VERSION
    }
}
impl RobotData for FR3Data {
    type DeviceData = Self;
    type Header = FR3CommandHeader;
    type State = RobotState;
    type StateIntern = FR3StateIntern;
    type Model = FR3Model;
    type LoadModelRequestWithHeader = LoadModelLibraryRequestWithFR3Header;
    type SetCollisionBehaviorRequestWithHeader = SetCollisionBehaviorRequestWithFR3Header;
    type SetLoadRequestWithHeader = SetLoadRequestWithFR3Header;
    type SetJointImpedanceRequestWithHeader = SetJointImpedanceRequestWithFR3Header;
    type SetCartesianImpedanceRequestWithHeader = SetCartesianImpedanceRequestWithFR3Header;
    type SetGuidingModeRequestWithHeader = SetGuidingModeRequestWithFR3Header;
    type ConnectRequestWithHeader = ConnectRequestWithFR3Header;
    type SetEeToKRequestWithHeader = SetEeToKRequestWithFR3Header;
    type SetNeToEeRequestWithHeader = SetNeToEeRequestWithFR3Header;
    type MoveRequestWithHeader = MoveRequestWithFR3Header;
    type MoveStatus = MoveStatusFR3;

    type GetterSetterStatus = GetterSetterStatusFR3;

    type StopMoveStatus = StopMoveStatusFR3;

    type AutomaticErrorRecoveryStatus = AutomaticErrorRecoveryStatusFR3;
    fn create_connect_request(
        command_id: &mut u32,
        udp_port: u16,
    ) -> Self::ConnectRequestWithHeader {
        let request = ConnectRequest {
            version: FR3_VERSION,
            udp_port,
        };
        ConnectRequestWithFR3Header {
            header: Self::create_header(
                command_id,
                FR3CommandEnum::Connect,
                size_of::<Self::ConnectRequestWithHeader>(),
            ),
            request,
        }
    }

    fn create_automatic_error_recovery_request(command_id: &mut u32) -> Self::CommandHeader {
        Self::create_header(
            command_id,
            FR3CommandEnum::AutomaticErrorRecovery,
            size_of::<FR3CommandHeader>(),
        )
    }

    fn create_stop_request(command_id: &mut u32) -> Self::CommandHeader {
        Self::create_header(
            command_id,
            FR3CommandEnum::StopMove,
            size_of::<FR3CommandHeader>(),
        )
    }

    fn handle_command_move_status(status: Self::MoveStatus) -> Result<(), FrankaException> {
        match status {
            MoveStatusFR3::Success => Ok(()),
            MoveStatusFR3::MotionStarted => {
            //todo handle motion_generator_running == true
            Ok(())
            }
            MoveStatusFR3::EmergencyAborted => Err(create_command_exception(
            "libfranka-rs: Move command aborted: User Stop pressed!",
            )),
            MoveStatusFR3::ReflexAborted => Err(create_command_exception(
            "libfranka-rs: Move command aborted: motion aborted by reflex!",
            )),
            MoveStatusFR3::InputErrorAborted => Err(create_command_exception(
            "libfranka-rs: Move command aborted: invalid input provided!",
            )),
            MoveStatusFR3::CommandNotPossibleRejected => Err(create_command_exception(
            "libfranka-rs: Move command rejected: command not possible in the current mode!",
            )),
            MoveStatusFR3::StartAtSingularPoseRejected => Err(create_command_exception(
            "libfranka-rs: Move command rejected: cannot start at singular pose!",
            )),
            MoveStatusFR3::InvalidArgumentRejected => Err(create_command_exception(
            "libfranka-rs: Move command rejected: maximum path deviation out of range!",
            )),
            MoveStatusFR3::Preempted => Err(create_command_exception(
            "libfranka-rs: Move command preempted!",
            )),
            MoveStatusFR3::Aborted => Err(create_command_exception(
            "libfranka-rs: Move command aborted!",
            )),
            MoveStatusFR3::PreemptedDueToActivatedSafetyFunctions =>Err(create_command_exception(
                "libfranka-rs: Move command preempted due to activated safety function! Please disable all safety functions.",
            )),
            MoveStatusFR3::CommandRejectedDueToActivatedSafetyFunctions =>
                Err(create_command_exception(
                    "libfranka-rs: Move command rejected due to activated safety function! Please disable all safety functions.",
                ))


        }
    }

    fn create_control_exception(
        message: String,
        move_status: Self::MoveStatus,
        reflex_reasons: &FrankaErrors,
        log: Vec<Record<Self::State>>,
    ) -> FrankaException {
        let mut exception_string = String::from(&message);
        if move_status == MoveStatusFR3::ReflexAborted {
            exception_string += " ";
            exception_string += reflex_reasons.to_string().as_str();
            if log.len() >= 2 {
                let lost_packets: u128 =
                    (log.last().unwrap().state.time - log[log.len() - 2].state.time).as_millis()
                        - 1;
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
    fn create_control_exception_if_reflex_aborted(
        message: String,
        move_status: Self::MoveStatus,
        reflex_reasons: &FrankaErrors,
        log: Vec<Record<Self::State>>,
    ) -> FrankaResult<()> {
        if move_status == MoveStatusFR3::ReflexAborted {
            return Err(Self::create_control_exception(
                message,
                move_status,
                reflex_reasons,
                log,
            ));
        }
        Ok(())
    }

    fn handle_getter_setter_status(status: Self::GetterSetterStatus) -> FrankaResult<()> {
        match status {
                GetterSetterStatusFR3::Success => Ok(()),
                GetterSetterStatusFR3::CommandNotPossibleRejected => Err(create_command_exception(
                    "libfranka-rs: command rejected: command not possible in current mode",
                )),
                GetterSetterStatusFR3::InvalidArgumentRejected => Err(create_command_exception(
                    "libfranka-rs: command rejected: invalid argument!",
                )),
                GetterSetterStatusFR3::CommandRejectedDueToActivatedSafetyFunctions => Err(create_command_exception(
                    "libfranka-rs: command rejected due to activated safety function! Please disable all safety functions.",
                )),
            }
    }

    fn handle_automatic_error_recovery_status(
        status: Self::AutomaticErrorRecoveryStatus,
    ) -> FrankaResult<()> {
        match &status {
            AutomaticErrorRecoveryStatusFR3::Success => Ok(()),
            AutomaticErrorRecoveryStatusFR3::EmergencyAborted => Err(create_command_exception(
                "libfranka-rs: command aborted: User Stop pressed!",
            )),
            AutomaticErrorRecoveryStatusFR3::ReflexAborted => Err(create_command_exception(
                "libfranka-rs: command aborted: motion aborted by reflex!",
            )),
            AutomaticErrorRecoveryStatusFR3::CommandNotPossibleRejected => {
                Err(create_command_exception(
                    "libfranka-rs: command rejected: command not possible in current mode",
                ))
            }
            AutomaticErrorRecoveryStatusFR3::ManualErrorRecoveryRequiredRejected => {
                Err(create_command_exception(
                    "libfranka-rs: command rejected: manual error recovery required!",
                ))
            }
            AutomaticErrorRecoveryStatusFR3::Aborted => {
                Err(create_command_exception("libfranka-rs: command aborted!"))
            }
            AutomaticErrorRecoveryStatusFR3::CommandRejectedDueToActivatedSafetyFunctions => Err(create_command_exception(
                "libfranka-rs: command rejected due to activated safety function! Please disable all safety functions.",
            )),
        }
    }

    fn handle_command_stop_move_status(
        status: Self::StopMoveStatus,
    ) -> Result<(), FrankaException> {
        match status {
            StopMoveStatusFR3::Success => Ok(()),
            StopMoveStatusFR3::EmergencyAborted => Err(create_command_exception(
                "libfranka-rs: Stop command aborted: User Stop pressed!",
            )),
            StopMoveStatusFR3::ReflexAborted => Err(create_command_exception(
                "libfranka-rs: Stop command aborted: motion aborted by reflex!",
            )),
            StopMoveStatusFR3::CommandNotPossibleRejected => Err(create_command_exception(
                "libfranka-rs: Stop command rejected: command not possible in the current mode!",
            )),
            StopMoveStatusFR3::Aborted => Err(create_command_exception(
                "libfranka-rs: Stop command aborted!",
            )),
            StopMoveStatusFR3::CommandRejectedDueToActivatedSafetyFunctions => Err(create_command_exception(
                "libfranka-rs: Move command rejected due to activated safety function! Please disable all safety functions.",
            ))
        }
    }
}

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

pub trait MessageCommand {
    fn get_command_message_id(&self) -> u32;
}

pub struct Network<Data: DeviceData> {
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
    data: PhantomData<Data>,
}

impl<Data: DeviceData> Network<Data> {
    pub fn new(franka_address: &str, franka_port: u16) -> Result<Network<Data>, Box<dyn Error>> {
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
            data: PhantomData,
        })
    }

    pub fn create_header_for_panda(
        &mut self,
        command: PandaCommandEnum,
        size: usize,
    ) -> PandaCommandHeader {
        let header = PandaCommandHeader::new(command, self.command_id, size as u32);
        self.command_id += 1;
        header
    }

    pub fn create_header(
        &mut self,
        command: Data::CommandEnum,
        size: usize,
    ) -> Data::CommandHeader {
        Data::create_header(&mut self.command_id, command, size)
    }

    pub fn tcp_send_request<T: Serialize + MessageCommand>(&mut self, request: T) -> u32 {
        let encoded_request = serialize(&request);
        self.tcp_socket.write_all(&encoded_request).unwrap();
        request.get_command_message_id()
    }
    /// Blocks until a Response message with the given command ID has been received and returns this
    /// response.
    ///
    /// # Arguments
    /// * `command_id` - Expected command ID of the Response.
    pub fn tcp_blocking_receive_response<T: DeserializeOwned + 'static>(
        &mut self,
        command_id: u32,
    ) -> T {
        let response_bytes = self.wait_for_response_to_arrive(&command_id);
        deserialize(&response_bytes)
    }
    /// Blocks until a Response message with the given command ID has been received and returns this
    /// response.
    ///
    /// # Arguments
    /// * `command_id` - Expected command ID of the Response.
    pub fn tcp_blocking_receive_status<T: DeserializeOwned + 'static>(
        &mut self,
        command_id: u32,
    ) -> T {
        let response_bytes = self.wait_for_response_to_arrive(&command_id);
        let (_, out): (Data::CommandHeader, T) = deserialize(&response_bytes);
        out
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
    ) -> FrankaResult<LoadModelLibraryStatus> {
        let response_bytes = self.wait_for_response_to_arrive(&command_id);
        let (header, status): (Data::CommandHeader, LoadModelLibraryStatus) = deserialize(
            &response_bytes
                [0..size_of::<LoadModelLibraryStatus>() + size_of::<Data::CommandHeader>()],
        );
        match status {
            LoadModelLibraryStatus::Success => {}
            LoadModelLibraryStatus::Error => {
                return Err(FrankaException::ModelException {
                    message: "libfranka-rs: Server reports error when loading model library."
                        .to_string(),
                });
            }
        }
        assert_ne!(
            header.get_size() as usize,
            size_of::<LoadModelLibraryStatus>() + size_of::<Data::CommandHeader>()
        );
        buffer.append(&mut Vec::from(
            &response_bytes
                [size_of::<LoadModelLibraryStatus>() + size_of::<Data::CommandHeader>()..],
        ));
        Ok(status)
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
        T: DeserializeOwned + 'static,
    {
        self.tcp_read_from_buffer(Duration::from_micros(0));
        let message = self.received_responses.get(&command_id);
        if message.is_none() {
            return Ok(false);
        }
        if message.unwrap().len() != size_of::<T>() + size_of::<Data::CommandHeader>() {
            panic!("libfranka-rs: Incorrect TCP message size.");
        }
        let message: (Data::CommandHeader, T) = deserialize(message.unwrap());
        let result = handler(message.1);
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
        while response_bytes.is_none() {
            {
                self.tcp_read_from_buffer(Duration::from_millis(10));
                response_bytes = self.received_responses.remove(command_id);
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
                        let mut buffer = [0_u8; 150000];
                        let available_bytes = self.tcp_socket.peek(&mut buffer);
                        let available_bytes = match available_bytes {
                            Ok(a) => a,
                            Err(e) => {
                                eprintln!("{}", e);
                                return;
                            }
                        };

                        if self.pending_response.is_empty() {
                            let header_mem_size = size_of::<Data::CommandHeader>();
                            if available_bytes >= header_mem_size {
                                let mut header_bytes: Vec<u8> = vec![0; header_mem_size];
                                self.tcp_socket.read_exact(&mut header_bytes).unwrap();
                                self.pending_response.append(&mut header_bytes.clone());
                                self.pending_response_offset = header_mem_size;
                                let header: Data::CommandHeader = deserialize(&header_bytes);
                                self.pending_response_len = header.get_size() as usize;
                                self.pending_command_id = header.get_command_id();
                            }
                        }
                        if !self.pending_response.is_empty() && available_bytes > 0 {
                            let number_of_bytes_to_read = usize::min(
                                available_bytes,
                                self.pending_response_len - self.pending_response_offset,
                            );
                            let mut response_buffer: Vec<u8> = vec![0; number_of_bytes_to_read];
                            self.tcp_socket.read_exact(&mut response_buffer).unwrap();
                            self.pending_response.append(&mut response_buffer);
                            self.pending_response_offset += number_of_bytes_to_read;
                            if self.pending_response_offset == self.pending_response_len {
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

fn deserialize<T: DeserializeOwned + 'static>(encoded: &[u8]) -> T {
    bincode::deserialize(encoded).unwrap()
}

#[cfg(test)]
mod tests {
    use crate::network::{deserialize, serialize};
    use crate::robot::types::PandaStateIntern;

    #[test]
    fn can_serialize_and_deserialize() {
        let state = PandaStateIntern::dummy();
        let state2: PandaStateIntern = deserialize(&serialize(&state));
        assert_eq!(state, state2);
    }
}
