// Copyright (c) 2021 Marco Boneberger
// Licensed under the EUPL-1.2-or-later

macro_rules! define_panda_request_with_header {
    ($name: ident, $request: ty, $command: expr) => {
        #[derive(Serialize, Deserialize, Debug, Copy, Clone)]
        #[repr(packed)]
        pub struct $name {
            pub header: PandaCommandHeader,
            pub request: $request,
        }
        impl MessageCommand for $name {
            fn get_command_message_id(&self) -> u32 {
                self.header.get_command_message_id()
            }
        }
        impl From<(u32, $request)> for $name {
            fn from(tuple: (u32, $request)) -> Self {
                let command_id = tuple.0;
                $name {
                    header: PandaCommandHeader::new(
                        $command,
                        command_id,
                        std::mem::size_of::<Self>() as u32,
                    ),
                    request: tuple.1,
                }
            }
        }
    };
}
macro_rules! define_fr3_request_with_header {
    ($name: ident, $request: ty, $command: expr) => {
        #[derive(Serialize, Deserialize, Debug, Copy, Clone)]
        #[repr(packed)]
        pub struct $name {
            pub header: FR3CommandHeader,
            pub request: $request,
        }
        impl MessageCommand for $name {
            fn get_command_message_id(&self) -> u32 {
                self.header.get_command_message_id()
            }
        }
        impl From<(u32, $request)> for $name {
            fn from(tuple: (u32, $request)) -> Self {
                let command_id = tuple.0;
                $name {
                    header: FR3CommandHeader::new(
                        $command,
                        command_id,
                        std::mem::size_of::<Self>() as u32,
                    ),
                    request: tuple.1,
                }
            }
        }
    };
}
use std::fmt::Debug;

use crate::gripper::types::CommandHeader;
use serde::Deserialize;
use serde::Serialize;
use serde_repr::{Deserialize_repr, Serialize_repr};

use crate::network::MessageCommand;

pub static PANDA_VERSION: u16 = 5;
pub static FR3_VERSION: u16 = 6;
pub static COMMAND_PORT: u16 = 1337;

pub trait RobotHeader: MessageCommand + Serialize + Debug + Copy + Clone {}

impl RobotHeader for PandaCommandHeader {}

impl MessageCommand for FR3CommandHeader {
    fn get_command_message_id(&self) -> u32 {
        self.command_id
    }
}

impl RobotHeader for FR3CommandHeader {}

impl CommandHeader for FR3CommandHeader {
    fn get_command_id(&self) -> u32 {
        self.command_id
    }

    fn get_size(&self) -> u32 {
        self.size
    }
}

impl CommandHeader for PandaCommandHeader {
    fn get_command_id(&self) -> u32 {
        self.command_id
    }

    fn get_size(&self) -> u32 {
        self.size
    }
}

#[derive(Serialize_repr, Deserialize_repr, Debug, Copy, Clone)]
#[repr(u32)]
pub enum PandaCommandEnum {
    Connect,
    Move,
    StopMove,
    GetCartesianLimit,
    SetCollisionBehavior,
    SetJointImpedance,
    SetCartesianImpedance,
    SetGuidingMode,
    SetEeToK,
    SetNeToEe,
    SetLoad,
    SetFilters,
    AutomaticErrorRecovery,
    LoadModelLibrary,
}

#[derive(Serialize_repr, Deserialize_repr, Debug, Copy, Clone)]
#[repr(u32)]
pub enum FR3CommandEnum {
    Connect,
    Move,
    StopMove,
    SetCollisionBehavior,
    SetJointImpedance,
    SetCartesianImpedance,
    SetGuidingMode,
    SetEeToK,
    SetNeToEe,
    SetLoad,
    AutomaticErrorRecovery,
    LoadModelLibrary,
}

#[derive(Serialize_repr, Deserialize_repr, Debug, Copy, Clone)]
#[repr(u8)]
pub enum DefaultStatus {
    Success,
    CommandNotPossibleRejected,
}

#[derive(Serialize_repr, Deserialize_repr, Debug, Copy, Clone)]
#[repr(u8)]
pub enum ConnectStatus {
    Success,
    IncompatibleLibraryVersion,
}

#[derive(Serialize_repr, Deserialize_repr, Debug, Copy, Clone, PartialEq)]
#[repr(u8)]
pub enum MoveStatusPanda {
    Success,
    MotionStarted,
    Preempted,
    CommandNotPossibleRejected,
    StartAtSingularPoseRejected,
    InvalidArgumentRejected,
    ReflexAborted,
    EmergencyAborted,
    InputErrorAborted,
    Aborted,
}

#[derive(Serialize_repr, Deserialize_repr, Debug, Copy, Clone, PartialEq)]
#[repr(u8)]
pub enum MoveStatusFR3 {
    Success,
    MotionStarted,
    Preempted,
    PreemptedDueToActivatedSafetyFunctions,
    CommandRejectedDueToActivatedSafetyFunctions,
    CommandNotPossibleRejected,
    StartAtSingularPoseRejected,
    InvalidArgumentRejected,
    ReflexAborted,
    EmergencyAborted,
    InputErrorAborted,
    Aborted,
}

#[derive(Serialize_repr, Deserialize_repr, Debug, Copy, Clone)]
#[repr(u8)]
pub enum StopMoveStatusPanda {
    Success,
    CommandNotPossibleRejected,
    EmergencyAborted,
    ReflexAborted,
    Aborted,
}

#[derive(Serialize_repr, Deserialize_repr, Debug, Copy, Clone)]
#[repr(u8)]
pub enum StopMoveStatusFR3 {
    Success,
    CommandNotPossibleRejected,
    CommandRejectedDueToActivatedSafetyFunctions,
    EmergencyAborted,
    ReflexAborted,
    Aborted,
}

#[derive(Serialize_repr, Deserialize_repr, Debug, Copy, Clone)]
#[repr(u8)]
pub enum AutomaticErrorRecoveryStatusPanda {
    Success,
    CommandNotPossibleRejected,
    ManualErrorRecoveryRequiredRejected,
    ReflexAborted,
    EmergencyAborted,
    Aborted,
}

#[derive(Serialize_repr, Deserialize_repr, Debug, Copy, Clone)]
#[repr(u8)]
pub enum AutomaticErrorRecoveryStatusFR3 {
    Success,
    CommandNotPossibleRejected,
    CommandRejectedDueToActivatedSafetyFunctions,
    ManualErrorRecoveryRequiredRejected,
    ReflexAborted,
    EmergencyAborted,
    Aborted,
}

#[derive(Serialize_repr, Deserialize_repr, Debug, Copy, Clone)]
#[repr(u8)]
pub enum LoadModelLibraryStatus {
    Success,
    Error,
}

#[derive(Serialize_repr, Deserialize_repr, Debug, Copy, Clone)]
#[repr(u8)]
pub enum GetterSetterStatusPanda {
    Success,
    CommandNotPossibleRejected,
    InvalidArgumentRejected,
}

#[derive(Serialize_repr, Deserialize_repr, Debug, Copy, Clone)]
#[repr(u8)]
pub enum GetterSetterStatusFR3 {
    Success,
    CommandNotPossibleRejected,
    InvalidArgumentRejected,
    CommandRejectedDueToActivatedSafetyFunctions,
}

#[derive(Serialize, Deserialize, Debug, Copy, Clone)]
#[repr(packed)]
pub struct ConnectRequest {
    pub version: u16,
    pub udp_port: u16,
}

impl ConnectRequest {
    pub fn new(udp_port: u16, version: u16) -> Self {
        ConnectRequest { version, udp_port }
    }
}

define_panda_request_with_header!(
    ConnectRequestWithPandaHeader,
    ConnectRequest,
    PandaCommandEnum::Connect
);
define_fr3_request_with_header!(
    ConnectRequestWithFR3Header,
    ConnectRequest,
    FR3CommandEnum::Connect
);

#[derive(Serialize, Deserialize)]
#[repr(packed)]
pub struct ConnectResponsePanda {
    pub header: PandaCommandHeader,
    pub status: ConnectStatus,
    pub version: u16,
}

#[derive(Serialize, Deserialize)]
#[repr(packed)]
pub struct ConnectResponseWithoutHeader {
    pub status: ConnectStatus,
    pub version: u16,
}

#[derive(Serialize_repr, Deserialize_repr, Debug, Copy, Clone)]
#[repr(u32)]
pub enum MoveControllerMode {
    JointImpedance,
    CartesianImpedance,
    ExternalController,
}

#[derive(Serialize_repr, Deserialize_repr, Debug, Copy, Clone)]
#[repr(u32)]
pub enum MoveMotionGeneratorMode {
    JointPosition,
    JointVelocity,
    CartesianPosition,
    CartesianVelocity,
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

define_panda_request_with_header!(
    MoveRequestWithPandaHeader,
    MoveRequest,
    PandaCommandEnum::Move
);
define_fr3_request_with_header!(MoveRequestWithFR3Header, MoveRequest, FR3CommandEnum::Move);

#[derive(Serialize, Deserialize, Debug, Copy, Clone)]
#[repr(packed)]
pub struct StopMoveRequestWithPandaHeader {
    pub header: PandaCommandHeader,
}

#[derive(Serialize, Deserialize, Debug, Copy, Clone)]
#[repr(packed)]
pub struct StopMoveRequestWithFR3Header {
    pub header: FR3CommandHeader,
}

impl MessageCommand for StopMoveRequestWithPandaHeader {
    fn get_command_message_id(&self) -> u32 {
        self.header.get_command_message_id()
    }
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
define_panda_request_with_header!(
    GetCartesianLimitRequestWithPandaHeader,
    GetCartesianLimitRequest,
    PandaCommandEnum::GetCartesianLimit
);

#[derive(Serialize, Deserialize, Debug)]
pub struct GetCartesianLimitResponse {
    pub header: PandaCommandHeader,
    pub status: GetterSetterStatusPanda,
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
define_panda_request_with_header!(
    SetCollisionBehaviorRequestWithPandaHeader,
    SetCollisionBehaviorRequest,
    PandaCommandEnum::SetCollisionBehavior
);
define_fr3_request_with_header!(
    SetCollisionBehaviorRequestWithFR3Header,
    SetCollisionBehaviorRequest,
    FR3CommandEnum::SetCollisionBehavior
);

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

define_panda_request_with_header!(
    SetJointImpedanceRequestWithPandaHeader,
    SetJointImpedanceRequest,
    PandaCommandEnum::SetJointImpedance
);
define_fr3_request_with_header!(
    SetJointImpedanceRequestWithFR3Header,
    SetJointImpedanceRequest,
    FR3CommandEnum::SetJointImpedance
);

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

define_panda_request_with_header!(
    SetCartesianImpedanceRequestWithPandaHeader,
    SetCartesianImpedanceRequest,
    PandaCommandEnum::SetCartesianImpedance
);
define_fr3_request_with_header!(
    SetCartesianImpedanceRequestWithFR3Header,
    SetCartesianImpedanceRequest,
    FR3CommandEnum::SetCartesianImpedance
);

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

define_panda_request_with_header!(
    SetGuidingModeRequestWithPandaHeader,
    SetGuidingModeRequest,
    PandaCommandEnum::SetGuidingMode
);
define_fr3_request_with_header!(
    SetGuidingModeRequestWithFR3Header,
    SetGuidingModeRequest,
    FR3CommandEnum::SetGuidingMode
);

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

define_panda_request_with_header!(
    SetEeToKRequestWithPandaHeader,
    SetEeToKRequest,
    PandaCommandEnum::SetEeToK
);
define_fr3_request_with_header!(
    SetEeToKRequestWithFR3Header,
    SetEeToKRequest,
    FR3CommandEnum::SetEeToK
);

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

define_panda_request_with_header!(
    SetNeToEeRequestWithPandaHeader,
    SetNeToEeRequest,
    PandaCommandEnum::SetNeToEe
);
define_fr3_request_with_header!(
    SetNeToEeRequestWithFR3Header,
    SetNeToEeRequest,
    FR3CommandEnum::SetNeToEe
);

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

define_panda_request_with_header!(
    SetLoadRequestWithPandaHeader,
    SetLoadRequest,
    PandaCommandEnum::SetLoad
);
define_fr3_request_with_header!(
    SetLoadRequestWithFR3Header,
    SetLoadRequest,
    FR3CommandEnum::SetLoad
);

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

define_panda_request_with_header!(
    SetFiltersRequestWithPandaHeader,
    SetFiltersRequest,
    PandaCommandEnum::SetFilters
);

pub type SetFiltersResponse = SetterResponse;

#[derive(Serialize, Deserialize, Debug)]
pub struct SetterResponse {
    pub header: PandaCommandHeader,
    pub status: GetterSetterStatusPanda,
}

#[derive(Serialize, Deserialize, Debug)]
pub struct SetterResponseFR3 {
    pub header: FR3CommandHeader,
    pub status: GetterSetterStatusPanda,
}

#[derive(Serialize_repr, Deserialize_repr, Debug, Copy, Clone)]
#[repr(u8)]
pub enum LoadModelLibraryArchitecture {
    X64,
    X86,
    Arm,
    Arm64,
}

#[derive(Serialize_repr, Deserialize_repr, Debug, Copy, Clone)]
#[repr(u8)]
pub enum LoadModelLibrarySystem {
    Linux,
    Windows,
}

#[derive(Serialize, Deserialize, Debug, Copy, Clone)]
#[repr(packed)]
pub struct LoadModelLibraryRequest {
    pub architecture: LoadModelLibraryArchitecture,
    pub system: LoadModelLibrarySystem,
}

define_panda_request_with_header!(
    LoadModelLibraryRequestWithPandaHeader,
    LoadModelLibraryRequest,
    PandaCommandEnum::LoadModelLibrary
);
define_fr3_request_with_header!(
    LoadModelLibraryRequestWithFR3Header,
    LoadModelLibraryRequest,
    FR3CommandEnum::LoadModelLibrary
);

#[derive(Serialize, Deserialize, Debug, Copy, Clone)]
#[repr(packed)]
pub struct PandaCommandHeader {
    pub command: PandaCommandEnum,
    pub command_id: u32,
    pub size: u32,
}

impl Default for PandaCommandHeader {
    fn default() -> Self {
        PandaCommandHeader {
            command: PandaCommandEnum::Connect,
            command_id: 0,
            size: 0,
        }
    }
}

impl PandaCommandHeader {
    pub fn new(command: PandaCommandEnum, command_id: u32, size: u32) -> PandaCommandHeader {
        PandaCommandHeader {
            command,
            command_id,
            size,
        }
    }
}

#[derive(Serialize, Deserialize, Debug, Copy, Clone)]
#[repr(packed)]
pub struct FR3CommandHeader {
    pub command: FR3CommandEnum,
    pub command_id: u32,
    pub size: u32,
}

impl FR3CommandHeader {
    pub fn new(command: FR3CommandEnum, command_id: u32, size: u32) -> FR3CommandHeader {
        FR3CommandHeader {
            command,
            command_id,
            size,
        }
    }
}

impl MessageCommand for PandaCommandHeader {
    fn get_command_message_id(&self) -> u32 {
        self.command_id
    }
}
