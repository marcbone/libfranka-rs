// Copyright (c) 2021 Marco Boneberger
// Licensed under the EUPL-1.2-or-later
#![allow(non_upper_case_globals)]

use std::fmt::Debug;

use serde::Deserialize;
use serde::Serialize;
use serde_repr::{Deserialize_repr, Serialize_repr};

use crate::network::MessageCommand;

static kVersion: u16 = 4;
pub static kCommandPort: u16 = 1337;

#[derive(Serialize_repr, Deserialize_repr, Debug, Copy, Clone)]
#[repr(u32)]
#[allow(non_camel_case_types)]
pub enum RobotCommandEnum {
    kConnect,
    kMove,
    kStopMove,
    kGetCartesianLimit,
    kSetCollisionBehavior,
    kSetJointImpedance,
    kSetCartesianImpedance,
    kSetGuidingMode,
    kSetEeToK,
    kSetNeToEe,
    kSetLoad,
    kSetFilters,
    kAutomaticErrorRecovery,
    kLoadModelLibrary,
}

#[derive(Serialize_repr, Deserialize_repr, Debug, Copy, Clone)]
#[repr(u8)]
#[allow(non_camel_case_types)]
pub enum DefaultStatus {
    kSuccess,
    kCommandNotPossibleRejected,
}

#[derive(Serialize_repr, Deserialize_repr, Debug, Copy, Clone)]
#[repr(u8)]
#[allow(non_camel_case_types)]
pub enum ConnectStatus {
    kSuccess,
    kIncompatibleLibraryVersion,
}

#[derive(Serialize_repr, Deserialize_repr, Debug, Copy, Clone, PartialEq)]
#[repr(u8)]
#[allow(non_camel_case_types)]
pub enum MoveStatus {
    kSuccess,
    kMotionStarted,
    kPreempted,
    kCommandNotPossibleRejected,
    kStartAtSingularPoseRejected,
    kInvalidArgumentRejected,
    kReflexAborted,
    kEmergencyAborted,
    kInputErrorAborted,
    kAborted,
}

#[derive(Serialize_repr, Deserialize_repr, Debug, Copy, Clone)]
#[repr(u8)]
#[allow(non_camel_case_types)]
pub enum StopMoveStatus {
    kSuccess,
    kCommandNotPossibleRejected,
    kEmergencyAborted,
    kReflexAborted,
    kAborted,
}

#[derive(Serialize_repr, Deserialize_repr, Debug, Copy, Clone)]
#[repr(u8)]
#[allow(non_camel_case_types)]
pub enum AutomaticErrorRecoveryStatus {
    kSuccess,
    kCommandNotPossibleRejected,
    kManualErrorRecoveryRequiredRejected,
    kReflexAborted,
    kEmergencyAborted,
    kAborted,
}

#[derive(Serialize_repr, Deserialize_repr, Debug, Copy, Clone)]
#[repr(u8)]
#[allow(non_camel_case_types)]
pub enum LoadModelLibraryStatus {
    kSuccess,
    kError,
}

#[derive(Serialize_repr, Deserialize_repr, Debug, Copy, Clone)]
#[repr(u8)]
#[allow(non_camel_case_types)]
pub enum GetterSetterStatus {
    kSuccess,
    kCommandNotPossibleRejected,
    kInvalidArgumentRejected,
}

#[derive(Serialize, Deserialize, Debug, Copy, Clone)]
#[repr(packed)]
pub struct ConnectRequest {
    pub version: u16,
    pub udp_port: u16,
}

impl ConnectRequest {
    pub fn new(udp_port: u16) -> Self {
        ConnectRequest {
            version: kVersion,
            udp_port,
        }
    }
}

#[derive(Serialize, Deserialize, Debug, Copy, Clone)]
#[repr(packed)]
pub struct ConnectRequestWithHeader {
    pub header: RobotCommandHeader,
    pub request: ConnectRequest,
}

impl MessageCommand for ConnectRequestWithHeader {
    fn get_command_message_id(&self) -> u32 {
        self.header.get_command_message_id()
    }
}

#[derive(Serialize, Deserialize, Debug)]
pub struct ConnectResponse {
    pub header: RobotCommandHeader,
    pub status: ConnectStatus,
    pub version: u16,
}

#[derive(Serialize_repr, Deserialize_repr, Debug, Copy, Clone)]
#[repr(u32)]
#[allow(non_camel_case_types)]
pub enum MoveControllerMode {
    kJointImpedance,
    kCartesianImpedance,
    kExternalController,
}

#[derive(Serialize_repr, Deserialize_repr, Debug, Copy, Clone)]
#[repr(u32)]
#[allow(non_camel_case_types)]
pub enum MoveMotionGeneratorMode {
    kJointPosition,
    kJointVelocity,
    kCartesianPosition,
    kCartesianVelocity,
}

#[derive(Serialize, Deserialize, Debug, Copy, Clone)]
#[repr(packed)]
pub struct MoveDeviation {
    pub(crate) translation: f64,
    pub(crate) rotation: f64,
    pub(crate) elbow: f64,
}

#[derive(Serialize, Deserialize, Debug, Copy, Clone)]
#[repr(packed)]
pub struct MoveRequest {
    controller_mode: MoveControllerMode,
    motion_generator_mode: MoveMotionGeneratorMode,
    maximum_path_deviation: MoveDeviation,
    maximum_goal_deviation: MoveDeviation,
}

impl MoveRequest {
    pub fn new(
        controller_mode: MoveControllerMode,
        motion_generator_mode: MoveMotionGeneratorMode,
        maximum_path_deviation: MoveDeviation,
        maximum_goal_deviation: MoveDeviation,
    ) -> Self {
        MoveRequest {
            controller_mode,
            motion_generator_mode,
            maximum_path_deviation,
            maximum_goal_deviation,
        }
    }
}

#[derive(Serialize, Deserialize, Debug, Copy, Clone)]
#[repr(packed)]
pub struct MoveRequestWithHeader {
    pub header: RobotCommandHeader,
    pub request: MoveRequest,
}

impl MessageCommand for MoveRequestWithHeader {
    fn get_command_message_id(&self) -> u32 {
        self.header.get_command_message_id()
    }
}

#[derive(Serialize, Deserialize, Debug)]
pub struct MoveResponse {
    pub header: RobotCommandHeader,
    pub status: MoveStatus,
}

#[derive(Serialize, Deserialize, Debug, Copy, Clone)]
#[repr(packed)]
pub struct StopMoveRequestWithHeader {
    pub header: RobotCommandHeader,
}

impl MessageCommand for StopMoveRequestWithHeader {
    fn get_command_message_id(&self) -> u32 {
        self.header.get_command_message_id()
    }
}

#[derive(Serialize, Deserialize, Debug)]
pub struct StopMoveResponse {
    pub header: RobotCommandHeader,
    pub status: StopMoveStatus,
}

#[derive(Serialize, Deserialize, Debug, Copy, Clone)]
#[repr(packed)]
pub struct GetCartesianLimitRequest {
    pub id: i32,
}

impl GetCartesianLimitRequest {
    pub fn new(id: i32) -> Self {
        GetCartesianLimitRequest { id }
    }
}

#[derive(Serialize, Deserialize, Debug, Copy, Clone)]
#[repr(packed)]
pub struct GetCartesianLimitRequestWithHeader {
    pub header: RobotCommandHeader,
    pub request: GetCartesianLimitRequest,
}

impl MessageCommand for GetCartesianLimitRequestWithHeader {
    fn get_command_message_id(&self) -> u32 {
        self.header.get_command_message_id()
    }
}

#[derive(Serialize, Deserialize, Debug)]
pub struct GetCartesianLimitResponse {
    pub header: RobotCommandHeader,
    pub status: GetterSetterStatus,
    pub object_world_size: [f64; 3],
    pub object_frame: [f64; 16],
    pub object_activation: bool,
}

#[derive(Serialize, Deserialize, Debug, Copy, Clone)]
#[repr(packed)]
pub struct SetCollisionBehaviorRequest {
    pub lower_torque_thresholds_acceleration: [f64; 7],
    pub upper_torque_thresholds_acceleration: [f64; 7],
    pub lower_torque_thresholds_nominal: [f64; 7],
    pub upper_torque_thresholds_nominal: [f64; 7],
    pub lower_force_thresholds_acceleration: [f64; 6],
    pub upper_force_thresholds_acceleration: [f64; 6],
    pub lower_force_thresholds_nominal: [f64; 6],
    pub upper_force_thresholds_nominal: [f64; 6],
}

impl SetCollisionBehaviorRequest {
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        lower_torque_thresholds_acceleration: [f64; 7],
        upper_torque_thresholds_acceleration: [f64; 7],
        lower_torque_thresholds_nominal: [f64; 7],
        upper_torque_thresholds_nominal: [f64; 7],
        lower_force_thresholds_acceleration: [f64; 6],
        upper_force_thresholds_acceleration: [f64; 6],
        lower_force_thresholds_nominal: [f64; 6],
        upper_force_thresholds_nominal: [f64; 6],
    ) -> Self {
        SetCollisionBehaviorRequest {
            lower_torque_thresholds_acceleration,
            upper_torque_thresholds_acceleration,
            lower_torque_thresholds_nominal,
            upper_torque_thresholds_nominal,
            lower_force_thresholds_acceleration,
            upper_force_thresholds_acceleration,
            lower_force_thresholds_nominal,
            upper_force_thresholds_nominal,
        }
    }
}

#[derive(Serialize, Deserialize, Debug, Copy, Clone)]
#[repr(packed)]
pub struct SetCollisionBehaviorRequestWithHeader {
    pub header: RobotCommandHeader,
    pub request: SetCollisionBehaviorRequest,
}

impl MessageCommand for SetCollisionBehaviorRequestWithHeader {
    fn get_command_message_id(&self) -> u32 {
        self.header.get_command_message_id()
    }
}

pub type SetCollisionBehaviorResponse = SetterResponse;

#[derive(Serialize, Deserialize, Debug, Copy, Clone)]
#[repr(packed)]
#[allow(non_snake_case)]
pub struct SetJointImpedanceRequest {
    pub K_theta: [f64; 7],
}

#[allow(non_snake_case)]
impl SetJointImpedanceRequest {
    pub fn new(K_theta: [f64; 7]) -> Self {
        SetJointImpedanceRequest { K_theta }
    }
}

#[derive(Serialize, Deserialize, Debug, Copy, Clone)]
#[repr(packed)]
pub struct SetJointImpedanceRequestWithHeader {
    pub header: RobotCommandHeader,
    pub request: SetJointImpedanceRequest,
}

impl MessageCommand for SetJointImpedanceRequestWithHeader {
    fn get_command_message_id(&self) -> u32 {
        self.header.get_command_message_id()
    }
}

pub type SetJointImpedanceResponse = SetterResponse;

#[derive(Serialize, Deserialize, Debug, Copy, Clone)]
#[repr(packed)]
#[allow(non_snake_case)]
pub struct SetCartesianImpedanceRequest {
    pub K_x: [f64; 6],
}

#[allow(non_snake_case)]
impl SetCartesianImpedanceRequest {
    pub fn new(K_x: [f64; 6]) -> Self {
        SetCartesianImpedanceRequest { K_x }
    }
}

#[derive(Serialize, Deserialize, Debug, Copy, Clone)]
#[repr(packed)]
pub struct SetCartesianImpedanceRequestWithHeader {
    pub header: RobotCommandHeader,
    pub request: SetCartesianImpedanceRequest,
}

impl MessageCommand for SetCartesianImpedanceRequestWithHeader {
    fn get_command_message_id(&self) -> u32 {
        self.header.get_command_message_id()
    }
}

pub type SetCartesianImpedanceResponse = SetterResponse;

#[derive(Serialize, Deserialize, Debug, Copy, Clone)]
#[repr(packed)]
#[allow(non_snake_case)]
pub struct SetGuidingModeRequest {
    pub guiding_mode: [bool; 6],
    pub nullspace: bool,
}

#[allow(non_snake_case)]
impl SetGuidingModeRequest {
    pub fn new(guiding_mode: [bool; 6], nullspace: bool) -> Self {
        SetGuidingModeRequest {
            guiding_mode,
            nullspace,
        }
    }
}

#[derive(Serialize, Deserialize, Debug, Copy, Clone)]
#[repr(packed)]
pub struct SetGuidingModeRequestWithHeader {
    pub header: RobotCommandHeader,
    pub request: SetGuidingModeRequest,
}

impl MessageCommand for SetGuidingModeRequestWithHeader {
    fn get_command_message_id(&self) -> u32 {
        self.header.get_command_message_id()
    }
}

pub type SetGuidingModeResponse = SetterResponse;

#[derive(Serialize, Deserialize, Debug, Copy, Clone)]
#[repr(packed)]
#[allow(non_snake_case)]
pub struct SetEeToKRequest {
    pub EE_T_K: [f64; 16],
}

#[allow(non_snake_case)]
impl SetEeToKRequest {
    pub fn new(EE_T_K: [f64; 16]) -> Self {
        SetEeToKRequest { EE_T_K }
    }
}

#[derive(Serialize, Deserialize, Debug, Copy, Clone)]
#[repr(packed)]
pub struct SetEeToKRequestWithHeader {
    pub header: RobotCommandHeader,
    pub request: SetEeToKRequest,
}

impl MessageCommand for SetEeToKRequestWithHeader {
    fn get_command_message_id(&self) -> u32 {
        self.header.get_command_message_id()
    }
}

pub type SetEeToKResponse = SetterResponse;

#[derive(Serialize, Deserialize, Debug, Copy, Clone)]
#[repr(packed)]
#[allow(non_snake_case)]
pub struct SetNeToEeRequest {
    pub NE_T_EE: [f64; 16],
}

#[allow(non_snake_case)]
impl SetNeToEeRequest {
    pub fn new(NE_T_EE: [f64; 16]) -> Self {
        SetNeToEeRequest { NE_T_EE }
    }
}

#[derive(Serialize, Deserialize, Debug, Copy, Clone)]
#[repr(packed)]
pub struct SetNeToEeRequestWithHeader {
    pub header: RobotCommandHeader,
    pub request: SetNeToEeRequest,
}

impl MessageCommand for SetNeToEeRequestWithHeader {
    fn get_command_message_id(&self) -> u32 {
        self.header.get_command_message_id()
    }
}

pub type SetNeToEeResponse = SetterResponse;

#[derive(Serialize, Deserialize, Debug, Copy, Clone)]
#[repr(packed)]
#[allow(non_snake_case)]
pub struct SetLoadRequest {
    pub m_load: f64,
    pub F_x_Cload: [f64; 3],
    pub I_Load: [f64; 9],
}

#[allow(non_snake_case)]
impl SetLoadRequest {
    pub fn new(m_load: f64, F_x_Cload: [f64; 3], I_Load: [f64; 9]) -> Self {
        SetLoadRequest {
            m_load,
            F_x_Cload,
            I_Load,
        }
    }
}

#[derive(Serialize, Deserialize, Debug, Copy, Clone)]
#[repr(packed)]
pub struct SetLoadRequestWithHeader {
    pub header: RobotCommandHeader,
    pub request: SetLoadRequest,
}

impl MessageCommand for SetLoadRequestWithHeader {
    fn get_command_message_id(&self) -> u32 {
        self.header.get_command_message_id()
    }
}

pub type SetLoadResponse = SetterResponse;

#[derive(Serialize, Deserialize, Debug, Copy, Clone)]
#[repr(packed)]
#[allow(non_snake_case)]
pub struct SetFiltersRequest {
    pub joint_position_filter_frequency: f64,
    pub joint_velocity_filter_frequency: f64,
    pub cartesian_position_filter_frequency: f64,
    pub cartesian_velocity_filter_frequency: f64,
    pub controller_filter_frequency: f64,
}

#[allow(non_snake_case)]
impl SetFiltersRequest {
    pub fn new(
        joint_position_filter_frequency: f64,
        joint_velocity_filter_frequency: f64,
        cartesian_position_filter_frequency: f64,
        cartesian_velocity_filter_frequency: f64,
        controller_filter_frequency: f64,
    ) -> Self {
        SetFiltersRequest {
            joint_position_filter_frequency,
            joint_velocity_filter_frequency,
            cartesian_position_filter_frequency,
            cartesian_velocity_filter_frequency,
            controller_filter_frequency,
        }
    }
}

#[derive(Serialize, Deserialize, Debug, Copy, Clone)]
#[repr(packed)]
pub struct SetFiltersRequestWithHeader {
    pub header: RobotCommandHeader,
    pub request: SetFiltersRequest,
}

impl MessageCommand for SetFiltersRequestWithHeader {
    fn get_command_message_id(&self) -> u32 {
        self.header.get_command_message_id()
    }
}

pub type SetFiltersResponse = SetterResponse;

#[derive(Serialize, Deserialize, Debug)]
pub struct SetterResponse {
    pub header: RobotCommandHeader,
    pub status: GetterSetterStatus,
}

pub type AutomaticErrorRecoveryRequestWithHeader = RobotCommandHeader;
// #[derive(Serialize, Deserialize, Debug, Copy, Clone)]
// #[repr(packed)]
// pub struct AutomaticErrorRecoveryRequestWithHeader {
//     pub header: RobotCommandHeader,
// }
//
// impl MessageCommand for AutomaticErrorRecoveryRequestWithHeader {
//     fn get_command_message_id(&self) -> u32 {
//         self.header.get_command_message_id()
//     }
// }

#[derive(Serialize, Deserialize, Debug)]
pub struct AutomaticErrorRecoveryResponse {
    pub header: RobotCommandHeader,
    pub status: AutomaticErrorRecoveryStatus,
}

#[derive(Serialize_repr, Deserialize_repr, Debug, Copy, Clone)]
#[repr(u8)]
#[allow(non_camel_case_types)]
pub enum LoadModelLibraryArchitecture {
    kX64,
    kX86,
    kArm,
    kArm64,
}

#[derive(Serialize_repr, Deserialize_repr, Debug, Copy, Clone)]
#[repr(u8)]
#[allow(non_camel_case_types)]
pub enum LoadModelLibrarySystem {
    kLinux,
    kWindow,
}

#[derive(Serialize, Deserialize, Debug, Copy, Clone)]
#[repr(packed)]
pub struct LoadModelLibraryRequest {
    pub architecture: LoadModelLibraryArchitecture,
    pub system: LoadModelLibrarySystem,
}

#[derive(Serialize, Deserialize, Debug, Copy, Clone)]
#[repr(packed)]
pub struct LoadModelLibraryRequestWithHeader {
    pub header: RobotCommandHeader,
    pub request: LoadModelLibraryRequest,
}

impl MessageCommand for LoadModelLibraryRequestWithHeader {
    fn get_command_message_id(&self) -> u32 {
        self.header.get_command_message_id()
    }
}

#[derive(Serialize, Deserialize, Debug)]
pub struct LoadModelLibraryResponse {
    pub header: RobotCommandHeader,
    pub status: LoadModelLibraryStatus,
}

#[derive(Serialize, Deserialize, Debug, Copy, Clone)]
#[repr(packed)]
pub struct RobotCommandHeader {
    pub command: RobotCommandEnum,
    pub command_id: u32,
    pub size: u32,
}

impl RobotCommandHeader {
    pub fn new(command: RobotCommandEnum, command_id: u32, size: u32) -> RobotCommandHeader {
        RobotCommandHeader {
            command,
            command_id,
            size,
        }
    }
}

impl MessageCommand for RobotCommandHeader {
    fn get_command_message_id(&self) -> u32 {
        self.command_id
    }
}
