use crate::device_data::DeviceData;
use crate::exception::{create_command_exception, FrankaException};
use crate::network::Network;
use crate::robot::errors::FrankaErrors;
use crate::robot::logger::Record;
use crate::robot::private_robot::PrivateRobot;
use crate::robot::rate_limiting::{RateLimiter, DELTA_T, LIMIT_EPS};
use crate::robot::robot_data::{PrivateRobotData, RobotData};
use crate::robot::robot_impl::RobotImplGeneric;
use crate::robot::service_types;
use crate::robot::service_types::{
    AutomaticErrorRecoveryStatusPanda, ConnectRequest, ConnectRequestWithPandaHeader,
    GetCartesianLimitRequest, GetCartesianLimitRequestWithPandaHeader, GetCartesianLimitResponse,
    GetterSetterStatusPanda, LoadModelLibraryRequestWithPandaHeader, MoveRequestWithPandaHeader,
    MoveStatusPanda, PandaCommandEnum, PandaCommandHeader,
    SetCartesianImpedanceRequestWithPandaHeader, SetCollisionBehaviorRequestWithPandaHeader,
    SetEeToKRequestWithPandaHeader, SetFiltersRequest, SetFiltersRequestWithPandaHeader,
    SetFiltersResponse, SetGuidingModeRequestWithPandaHeader,
    SetJointImpedanceRequestWithPandaHeader, SetLoadRequestWithPandaHeader,
    SetNeToEeRequestWithPandaHeader, StopMoveStatusPanda, PANDA_VERSION,
};
use crate::robot::types::PandaStateIntern;
use crate::robot::virtual_wall_cuboid::VirtualWallCuboid;
use crate::{FrankaResult, PandaModel, RealtimeConfig, RobotState};
use std::mem::size_of;

/// Maintains a network connection to the robot, provides the current robot state, gives access to
/// the model library and allows to control the robot. See the [`Robot`](crate::Robot) trait for
/// methods to control the robot.
///
/// # Rate Limiting
/// For Panda the rate limiter is enabled by default. You can disable it by setting the `limit_rate`
/// argument in the control methods to `false`.
/// The implementation is based on the rate limiter in the original libfranka library
/// that was used for Panda robots (version 0.9.2 and lower).
/// Here, the joint position is completely ignored for calculating the maximum and minimum
/// joint velocities.
pub struct Panda(RobotImplGeneric<Self>);

impl Panda {
    /// Establishes a connection with a Panda robot.
    ///
    /// # Arguments
    /// * `franka_address` - IP/hostname of the robot.
    /// * `realtime_config` - if set to Enforce, an exception will be thrown if realtime priority
    /// cannot be set when required. Setting realtime_config to Ignore disables this behavior. Default is Enforce
    /// * `log_size` - sets how many last states should be kept for logging purposes.
    /// The log is provided when a [`ControlException`](`crate::exception::FrankaException::ControlException`) is thrown.
    /// # Example
    /// ```no_run
    /// use franka::{FrankaResult, RealtimeConfig, Panda};
    /// fn main() -> FrankaResult<()> {
    ///     // connects to the robot using real-time scheduling and a default log size of 50.
    ///     let mut robot = Panda::new("robotik-bs.de", None, None)?;
    ///     // connects to the robot without using real-time scheduling and a log size of 1.
    ///     let mut robot = Panda::new("robotik-bs.de", RealtimeConfig::Ignore, 1)?;
    ///     Ok(())
    /// }
    /// ```
    /// # Errors
    /// * [`NetworkException`](FrankaException::NetworkException) if the connection is unsuccessful.
    /// * [`IncompatibleLibraryVersionError`](FrankaException::IncompatibleLibraryVersionError) if this version of `libfranka-rs` is not supported or
    /// if the robot is not a Panda.
    pub fn new<RtConfig: Into<Option<RealtimeConfig>>, LogSize: Into<Option<usize>>>(
        franka_address: &str,
        realtime_config: RtConfig,
        log_size: LogSize,
    ) -> FrankaResult<Panda> {
        let realtime_config = realtime_config.into().unwrap_or(RealtimeConfig::Enforce);
        let log_size = log_size.into().unwrap_or(50);
        let network = Network::new(franka_address, service_types::COMMAND_PORT).map_err(|_| {
            FrankaException::NetworkException {
                message: "Connection could not be established".to_string(),
            }
        })?;
        Ok(Panda(RobotImplGeneric::new(
            network,
            log_size,
            realtime_config,
        )?))
    }

    /// Sets the cut off frequency for the given motion generator or controller.
    ///
    /// Allowed input range for all the filters is between 1.0 Hz and 1000.0 Hz.
    /// If the value is set to maximum (1000Hz) then no filtering is done.
    /// # Arguments
    /// * `joint_position_filter_frequency` - Frequency at which the commanded joint
    /// position is cut off.
    /// * `joint_velocity_filter_frequency` - TFrequency at which the commanded joint
    ///  velocity is cut off.
    /// * `cartesian_position_filter_frequency` - Frequency at which the commanded
    /// Cartesian position is cut off.
    /// * `cartesian_velocity_filter_frequency` - Frequency at which the commanded
    /// Cartesian velocity is cut off.
    /// * `controller_filter_frequency` - Frequency at which the commanded torque is cut off
    /// # Errors
    /// * [`CommandException`](`crate::exception::FrankaException::CommandException`) if the Control reports an error.
    /// * [`NetworkException`](`crate::exception::FrankaException::NetworkException`) if the connection is lost, e.g. after a timeout.
    #[deprecated(note = "please use `low_pass_filter` instead")]
    pub fn set_filters(
        &mut self,
        joint_position_filter_frequency: f64,
        joint_velocity_filter_frequency: f64,
        cartesian_position_filter_frequency: f64,
        cartesian_velocity_filter_frequency: f64,
        controller_filter_frequency: f64,
    ) -> FrankaResult<()> {
        let command = SetFiltersRequestWithPandaHeader {
            header: self.0.network.create_header_for_panda(
                PandaCommandEnum::SetFilters,
                size_of::<SetFiltersRequestWithPandaHeader>(),
            ),
            request: SetFiltersRequest::new(
                joint_position_filter_frequency,
                joint_velocity_filter_frequency,
                cartesian_position_filter_frequency,
                cartesian_velocity_filter_frequency,
                controller_filter_frequency,
            ),
        };
        let command_id: u32 = self.0.network.tcp_send_request(command);
        let response: SetFiltersResponse = self.0.network.tcp_blocking_receive_response(command_id);
        Panda::handle_getter_setter_status(response.status)
    }

    /// Returns the parameters of a virtual wall.
    /// # Arguments
    /// * `id` - ID of the virtual wall.
    /// # Return
    /// Parameters of virtual wall.
    /// # Errors
    /// * [`CommandException`](`crate::exception::FrankaException::CommandException`) if the Control reports an error.
    /// * [`NetworkException`](`crate::exception::FrankaException::NetworkException`) if the connection is lost, e.g. after a timeout.
    pub fn get_virtual_wall(&mut self, id: i32) -> FrankaResult<VirtualWallCuboid> {
        let command = GetCartesianLimitRequestWithPandaHeader {
            header: self.0.network.create_header_for_panda(
                PandaCommandEnum::GetCartesianLimit,
                size_of::<GetCartesianLimitRequestWithPandaHeader>(),
            ),
            request: GetCartesianLimitRequest::new(id),
        };
        let command_id: u32 = self.0.network.tcp_send_request(command);
        let response: GetCartesianLimitResponse =
            self.0.network.tcp_blocking_receive_response(command_id);
        match &response.status {
            GetterSetterStatusPanda::Success => Ok(VirtualWallCuboid::new(id, response)),
            GetterSetterStatusPanda::CommandNotPossibleRejected => Err(create_command_exception(
                "libfranka-rs: command rejected: command not possible in current mode",
            )),
            GetterSetterStatusPanda::InvalidArgumentRejected => Err(create_command_exception(
                "libfranka-rs: command rejected: invalid argument!",
            )),
        }
    }
}

impl PrivateRobot for Panda {
    type Rob = RobotImplGeneric<Self>;

    fn get_rob_mut(&mut self) -> &mut Self::Rob {
        &mut self.0
    }
    fn get_rob(&self) -> &Self::Rob {
        &self.0
    }

    fn get_net(&mut self) -> &mut Network<Self> {
        &mut self.0.network
    }
}

impl DeviceData for Panda {
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

impl RobotData for Panda {
    type Model = PandaModel;
    type StateIntern = PandaStateIntern;
    type State = RobotState;
}

impl Panda {
    /// Maximum joint velocity
    pub const MAX_JOINT_VELOCITY: [f64; 7] = [
        2.1750
            - LIMIT_EPS
            - Self::TOL_NUMBER_PACKETS_LOST * DELTA_T * Self::MAX_JOINT_ACCELERATION[0],
        2.1750
            - LIMIT_EPS
            - Self::TOL_NUMBER_PACKETS_LOST * DELTA_T * Self::MAX_JOINT_ACCELERATION[1],
        2.1750
            - LIMIT_EPS
            - Self::TOL_NUMBER_PACKETS_LOST * DELTA_T * Self::MAX_JOINT_ACCELERATION[2],
        2.1750
            - LIMIT_EPS
            - Self::TOL_NUMBER_PACKETS_LOST * DELTA_T * Self::MAX_JOINT_ACCELERATION[3],
        2.6100
            - LIMIT_EPS
            - Self::TOL_NUMBER_PACKETS_LOST * DELTA_T * Self::MAX_JOINT_ACCELERATION[4],
        2.6100
            - LIMIT_EPS
            - Self::TOL_NUMBER_PACKETS_LOST * DELTA_T * Self::MAX_JOINT_ACCELERATION[5],
        2.6100
            - LIMIT_EPS
            - Self::TOL_NUMBER_PACKETS_LOST * DELTA_T * Self::MAX_JOINT_ACCELERATION[6],
    ];

    /// Minimum joint velocity
    pub const MIN_JOINT_VELOCITY: [f64; 7] = [
        -Self::MAX_JOINT_VELOCITY[0],
        -Self::MAX_JOINT_VELOCITY[1],
        -Self::MAX_JOINT_VELOCITY[2],
        -Self::MAX_JOINT_VELOCITY[3],
        -Self::MAX_JOINT_VELOCITY[4],
        -Self::MAX_JOINT_VELOCITY[5],
        -Self::MAX_JOINT_VELOCITY[6],
    ];
}

impl RateLimiter for Panda {
    const RATE_LIMITING_ON_PER_DEFAULT: bool = true;
    const TOL_NUMBER_PACKETS_LOST: f64 = 3.;
    const MAX_JOINT_JERK: [f64; 7] = [
        7500.0 - LIMIT_EPS,
        3750.0 - LIMIT_EPS,
        5000.0 - LIMIT_EPS,
        6250.0 - LIMIT_EPS,
        7500.0 - LIMIT_EPS,
        10000.0 - LIMIT_EPS,
        10000.0 - LIMIT_EPS,
    ];
    const MAX_JOINT_ACCELERATION: [f64; 7] = [
        15.0000 - LIMIT_EPS,
        7.500 - LIMIT_EPS,
        10.0000 - LIMIT_EPS,
        12.5000 - LIMIT_EPS,
        15.0000 - LIMIT_EPS,
        20.0000 - LIMIT_EPS,
        20.0000 - LIMIT_EPS,
    ];
    const MAX_TRANSLATIONAL_JERK: f64 = 6500.0 - LIMIT_EPS;
    const MAX_TRANSLATIONAL_ACCELERATION: f64 = 13.0000 - LIMIT_EPS;
    const MAX_TRANSLATIONAL_VELOCITY: f64 = 2.0000
        - LIMIT_EPS
        - Self::TOL_NUMBER_PACKETS_LOST * DELTA_T * Self::MAX_TRANSLATIONAL_ACCELERATION;
    const MAX_ROTATIONAL_JERK: f64 = 12500.0 - LIMIT_EPS;
    const MAX_ROTATIONAL_ACCELERATION: f64 = 25.0000 - LIMIT_EPS;
    const MAX_ROTATIONAL_VELOCITY: f64 = 2.5000
        - LIMIT_EPS
        - Self::TOL_NUMBER_PACKETS_LOST * DELTA_T * Self::MAX_ROTATIONAL_ACCELERATION;
    const MAX_ELBOW_JERK: f64 = 5000. - LIMIT_EPS;
    const MAX_ELBOW_ACCELERATION: f64 = 10.0000 - LIMIT_EPS;
    const MAX_ELBOW_VELOCITY: f64 =
        2.1750 - LIMIT_EPS - Self::TOL_NUMBER_PACKETS_LOST * DELTA_T * Self::MAX_ELBOW_ACCELERATION;

    fn compute_upper_limits_joint_velocity(_q: &[f64; 7]) -> [f64; 7] {
        Self::MAX_JOINT_VELOCITY
    }

    fn compute_lower_limits_joint_velocity(_q: &[f64; 7]) -> [f64; 7] {
        Self::MIN_JOINT_VELOCITY
    }
}

impl PrivateRobotData for Panda {
    const MODEL_NAME: &'static str = "panda";
    type Header = PandaCommandHeader;
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
