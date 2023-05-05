use crate::device_data::DeviceData;
use crate::exception::FrankaException;
use crate::network::MessageCommand;
use crate::robot::errors::FrankaErrors;
use crate::robot::logger::Record;
use crate::robot::robot_state::AbstractRobotState;
use crate::robot::service_types::{
    ConnectRequest, LoadModelLibraryRequest, MoveRequest, RobotHeader,
    SetCartesianImpedanceRequest, SetCollisionBehaviorRequest, SetEeToKRequest,
    SetGuidingModeRequest, SetJointImpedanceRequest, SetLoadRequest, SetNeToEeRequest,
};
use crate::robot::types::AbstractRobotStateIntern;
use crate::{FrankaResult, RobotModel, RobotState};
use serde::de::DeserializeOwned;
use serde::Serialize;
use std::fmt::Debug;

pub trait RobotData {
   type Model: RobotModel;
   type StateIntern: Debug + DeserializeOwned + Serialize + AbstractRobotStateIntern + 'static;
   type State: AbstractRobotState + From<Self::StateIntern> + From<RobotState>;
}

pub(crate) trait PrivateRobotData: DeviceData + RobotData {
    // type DeviceData: DeviceData;
    type Header: RobotHeader;
    // type State: AbstractRobotState + From<Self::StateIntern> + From<RobotState>;
    // type StateIntern: Debug + DeserializeOwned + Serialize + AbstractRobotStateIntern + 'static;
    // type Model: RobotModel;
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
