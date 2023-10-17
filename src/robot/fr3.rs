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
    AutomaticErrorRecoveryStatusFr3, ConnectRequest, ConnectRequestWithFr3Header, Fr3CommandEnum,
    Fr3CommandHeader, GetterSetterStatusFr3, LoadModelLibraryRequestWithFr3Header,
    MoveRequestWithFr3Header, MoveStatusFr3, SetCartesianImpedanceRequestWithFr3Header,
    SetCollisionBehaviorRequestWithFr3Header, SetEeToKRequestWithFr3Header,
    SetGuidingModeRequestWithFr3Header, SetJointImpedanceRequestWithFr3Header,
    SetLoadRequestWithFr3Header, SetNeToEeRequestWithFr3Header, StopMoveStatusFr3, FR3_VERSION,
};
use crate::robot::types::Fr3StateIntern;
use crate::{Fr3Model, FrankaResult, RealtimeConfig, RobotState};
use std::mem::size_of;

/// Maintains a network connection to the FR3 robot, provides the current robot state, gives access to
/// the model library and allows to control the robot. See the [`Robot`](crate::Robot) trait for
/// methods to control the robot.
///
/// # Rate Limiting
/// For FR3 the rate limiter is turned off by default. You can enable it by setting the `limit_rate`
/// argument in the control methods to `true`. The implementation is based on
/// [https://frankaemika.github.io/docs/control_parameters.html#limits-for-franka-research-3](https://frankaemika.github.io/docs/control_parameters.html#limits-for-franka-research-3).
/// That means the maximum velocity depends on the current joint configuration of the robot.
pub struct Fr3(RobotImplGeneric<Self>);

impl RobotData for Fr3 {
    type Model = Fr3Model;
    type StateIntern = Fr3StateIntern;
    type State = RobotState;
}

impl PrivateRobot for Fr3 {
    type Rob = RobotImplGeneric<Fr3>;

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

impl Fr3 {
    pub fn new<RtConfig: Into<Option<RealtimeConfig>>, LogSize: Into<Option<usize>>>(
        franka_address: &str,
        realtime_config: RtConfig,
        log_size: LogSize,
    ) -> FrankaResult<Fr3> {
        let realtime_config = realtime_config.into().unwrap_or(RealtimeConfig::Enforce);
        let log_size = log_size.into().unwrap_or(50);
        let network = Network::new(franka_address, service_types::COMMAND_PORT).map_err(|_| {
            FrankaException::NetworkException {
                message: "Connection could not be established".to_string(),
            }
        })?;
        Ok(Fr3(RobotImplGeneric::new(
            network,
            log_size,
            realtime_config,
        )?))
    }
}

impl DeviceData for Fr3 {
    type CommandHeader = Fr3CommandHeader;
    type CommandEnum = Fr3CommandEnum;
    fn create_header(
        command_id: &mut u32,
        command: Self::CommandEnum,
        size: usize,
    ) -> Self::CommandHeader {
        let header = Fr3CommandHeader::new(command, *command_id, size as u32);
        *command_id += 1;
        header
    }

    fn get_library_version() -> u16 {
        FR3_VERSION
    }
}

impl PrivateRobotData for Fr3 {
    const MODEL_NAME: &'static str = "fr3";
    type Header = Fr3CommandHeader;
    type LoadModelRequestWithHeader = LoadModelLibraryRequestWithFr3Header;
    type SetCollisionBehaviorRequestWithHeader = SetCollisionBehaviorRequestWithFr3Header;
    type SetLoadRequestWithHeader = SetLoadRequestWithFr3Header;
    type SetJointImpedanceRequestWithHeader = SetJointImpedanceRequestWithFr3Header;
    type SetCartesianImpedanceRequestWithHeader = SetCartesianImpedanceRequestWithFr3Header;
    type SetGuidingModeRequestWithHeader = SetGuidingModeRequestWithFr3Header;
    type ConnectRequestWithHeader = ConnectRequestWithFr3Header;
    type SetEeToKRequestWithHeader = SetEeToKRequestWithFr3Header;
    type SetNeToEeRequestWithHeader = SetNeToEeRequestWithFr3Header;
    type MoveRequestWithHeader = MoveRequestWithFr3Header;
    type MoveStatus = MoveStatusFr3;
    type GetterSetterStatus = GetterSetterStatusFr3;
    type StopMoveStatus = StopMoveStatusFr3;
    type AutomaticErrorRecoveryStatus = AutomaticErrorRecoveryStatusFr3;

    fn create_connect_request(
        command_id: &mut u32,
        udp_port: u16,
    ) -> Self::ConnectRequestWithHeader {
        let request = ConnectRequest {
            version: FR3_VERSION,
            udp_port,
        };
        ConnectRequestWithFr3Header {
            header: Self::create_header(
                command_id,
                Fr3CommandEnum::Connect,
                size_of::<Self::ConnectRequestWithHeader>(),
            ),
            request,
        }
    }

    fn create_automatic_error_recovery_request(command_id: &mut u32) -> Self::CommandHeader {
        Self::create_header(
            command_id,
            Fr3CommandEnum::AutomaticErrorRecovery,
            size_of::<Fr3CommandHeader>(),
        )
    }

    fn create_stop_request(command_id: &mut u32) -> Self::CommandHeader {
        Self::create_header(
            command_id,
            Fr3CommandEnum::StopMove,
            size_of::<Fr3CommandHeader>(),
        )
    }

    fn handle_command_move_status(status: Self::MoveStatus) -> Result<(), FrankaException> {
        match status {
            MoveStatusFr3::Success => Ok(()),
            MoveStatusFr3::MotionStarted => {
            //todo handle motion_generator_running == true
            Ok(())
            }
            MoveStatusFr3::EmergencyAborted => Err(create_command_exception(
            "libfranka-rs: Move command aborted: User Stop pressed!",
            )),
            MoveStatusFr3::ReflexAborted => Err(create_command_exception(
            "libfranka-rs: Move command aborted: motion aborted by reflex!",
            )),
            MoveStatusFr3::InputErrorAborted => Err(create_command_exception(
            "libfranka-rs: Move command aborted: invalid input provided!",
            )),
            MoveStatusFr3::CommandNotPossibleRejected => Err(create_command_exception(
            "libfranka-rs: Move command rejected: command not possible in the current mode!",
            )),
            MoveStatusFr3::StartAtSingularPoseRejected => Err(create_command_exception(
            "libfranka-rs: Move command rejected: cannot start at singular pose!",
            )),
            MoveStatusFr3::InvalidArgumentRejected => Err(create_command_exception(
            "libfranka-rs: Move command rejected: maximum path deviation out of range!",
            )),
            MoveStatusFr3::Preempted => Err(create_command_exception(
            "libfranka-rs: Move command preempted!",
            )),
            MoveStatusFr3::Aborted => Err(create_command_exception(
            "libfranka-rs: Move command aborted!",
            )),
            MoveStatusFr3::PreemptedDueToActivatedSafetyFunctions =>Err(create_command_exception(
                "libfranka-rs: Move command preempted due to activated safety function! Please disable all safety functions.",
            )),
            MoveStatusFr3::CommandRejectedDueToActivatedSafetyFunctions =>
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
        if move_status == MoveStatusFr3::ReflexAborted {
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
        if move_status == MoveStatusFr3::ReflexAborted {
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
                GetterSetterStatusFr3::Success => Ok(()),
                GetterSetterStatusFr3::CommandNotPossibleRejected => Err(create_command_exception(
                    "libfranka-rs: command rejected: command not possible in current mode",
                )),
                GetterSetterStatusFr3::InvalidArgumentRejected => Err(create_command_exception(
                    "libfranka-rs: command rejected: invalid argument!",
                )),
                GetterSetterStatusFr3::CommandRejectedDueToActivatedSafetyFunctions => Err(create_command_exception(
                    "libfranka-rs: command rejected due to activated safety function! Please disable all safety functions.",
                )),
            }
    }

    fn handle_automatic_error_recovery_status(
        status: Self::AutomaticErrorRecoveryStatus,
    ) -> FrankaResult<()> {
        match &status {
            AutomaticErrorRecoveryStatusFr3::Success => Ok(()),
            AutomaticErrorRecoveryStatusFr3::EmergencyAborted => Err(create_command_exception(
                "libfranka-rs: command aborted: User Stop pressed!",
            )),
            AutomaticErrorRecoveryStatusFr3::ReflexAborted => Err(create_command_exception(
                "libfranka-rs: command aborted: motion aborted by reflex!",
            )),
            AutomaticErrorRecoveryStatusFr3::CommandNotPossibleRejected => {
                Err(create_command_exception(
                    "libfranka-rs: command rejected: command not possible in current mode",
                ))
            }
            AutomaticErrorRecoveryStatusFr3::ManualErrorRecoveryRequiredRejected => {
                Err(create_command_exception(
                    "libfranka-rs: command rejected: manual error recovery required!",
                ))
            }
            AutomaticErrorRecoveryStatusFr3::Aborted => {
                Err(create_command_exception("libfranka-rs: command aborted!"))
            }
            AutomaticErrorRecoveryStatusFr3::CommandRejectedDueToActivatedSafetyFunctions => Err(create_command_exception(
                "libfranka-rs: command rejected due to activated safety function! Please disable all safety functions.",
            )),
        }
    }

    fn handle_command_stop_move_status(
        status: Self::StopMoveStatus,
    ) -> Result<(), FrankaException> {
        match status {
            StopMoveStatusFr3::Success => Ok(()),
            StopMoveStatusFr3::EmergencyAborted => Err(create_command_exception(
                "libfranka-rs: Stop command aborted: User Stop pressed!",
            )),
            StopMoveStatusFr3::ReflexAborted => Err(create_command_exception(
                "libfranka-rs: Stop command aborted: motion aborted by reflex!",
            )),
            StopMoveStatusFr3::CommandNotPossibleRejected => Err(create_command_exception(
                "libfranka-rs: Stop command rejected: command not possible in the current mode!",
            )),
            StopMoveStatusFr3::Aborted => Err(create_command_exception(
                "libfranka-rs: Stop command aborted!",
            )),
            StopMoveStatusFr3::CommandRejectedDueToActivatedSafetyFunctions => Err(create_command_exception(
                "libfranka-rs: Move command rejected due to activated safety function! Please disable all safety functions.",
            ))
        }
    }
}

/// This is the position based velocity rate limiter for Fr3. The implementation is based on
/// [https://frankaemika.github.io/docs/control_parameters.html#limits-for-franka-research-3](https://frankaemika.github.io/docs/control_parameters.html#limits-for-franka-research-3).
impl Fr3 {
    /// Tolerance value for joint velocity limits to deal with numerical errors and data losses.
    pub const JOINT_VELOCITY_LIMITS_TOLERANCE: [f64; 7] = [
        LIMIT_EPS + Self::TOL_NUMBER_PACKETS_LOST * DELTA_T * Self::MAX_JOINT_ACCELERATION[0],
        LIMIT_EPS + Self::TOL_NUMBER_PACKETS_LOST * DELTA_T * Self::MAX_JOINT_ACCELERATION[1],
        LIMIT_EPS + Self::TOL_NUMBER_PACKETS_LOST * DELTA_T * Self::MAX_JOINT_ACCELERATION[2],
        LIMIT_EPS + Self::TOL_NUMBER_PACKETS_LOST * DELTA_T * Self::MAX_JOINT_ACCELERATION[3],
        LIMIT_EPS + Self::TOL_NUMBER_PACKETS_LOST * DELTA_T * Self::MAX_JOINT_ACCELERATION[4],
        LIMIT_EPS + Self::TOL_NUMBER_PACKETS_LOST * DELTA_T * Self::MAX_JOINT_ACCELERATION[5],
        LIMIT_EPS + Self::TOL_NUMBER_PACKETS_LOST * DELTA_T * Self::MAX_JOINT_ACCELERATION[6],
    ];

    fn upper_limits(a: f64, b: f64, c: f64, d: f64, e: f64, tolerance: f64) -> f64 {
        f64::min(a, f64::max(0., b + f64::sqrt(f64::max(0., c * (d - e))))) - tolerance
    }

    fn lower_limits(a: f64, b: f64, c: f64, d: f64, e: f64, tolerance: f64) -> f64 {
        f64::max(a, f64::min(0., b - f64::sqrt(f64::max(0., c * (d + e))))) + tolerance
    }
}

impl RateLimiter for Fr3 {
    const RATE_LIMITING_ON_PER_DEFAULT: bool = false;
    const TOL_NUMBER_PACKETS_LOST: f64 = 0.;
    const MAX_JOINT_JERK: [f64; 7] = [5000.0 - LIMIT_EPS; 7];
    const MAX_JOINT_ACCELERATION: [f64; 7] = [10.0000 - LIMIT_EPS; 7];
    const MAX_TRANSLATIONAL_JERK: f64 = 4500.0 - LIMIT_EPS;
    const MAX_TRANSLATIONAL_ACCELERATION: f64 = 9.0000 - LIMIT_EPS;
    const MAX_TRANSLATIONAL_VELOCITY: f64 = 3.0000
        - LIMIT_EPS
        - Self::TOL_NUMBER_PACKETS_LOST * DELTA_T * Self::MAX_TRANSLATIONAL_ACCELERATION;
    const MAX_ROTATIONAL_JERK: f64 = 8500.0 - LIMIT_EPS;
    const MAX_ROTATIONAL_ACCELERATION: f64 = 17.0000 - LIMIT_EPS;
    const MAX_ROTATIONAL_VELOCITY: f64 = 2.5000
        - LIMIT_EPS
        - Self::TOL_NUMBER_PACKETS_LOST * DELTA_T * Self::MAX_ROTATIONAL_ACCELERATION;
    const MAX_ELBOW_JERK: f64 = 5000. - LIMIT_EPS;
    const MAX_ELBOW_ACCELERATION: f64 = 10.0000 - LIMIT_EPS;
    const MAX_ELBOW_VELOCITY: f64 =
        1.5 - LIMIT_EPS - Self::TOL_NUMBER_PACKETS_LOST * DELTA_T * Self::MAX_ELBOW_ACCELERATION;

    fn compute_upper_limits_joint_velocity(q: &[f64; 7]) -> [f64; 7] {
        [
            Self::upper_limits(
                2.62,
                -0.3,
                12.,
                2.75010,
                q[0],
                Self::JOINT_VELOCITY_LIMITS_TOLERANCE[0],
            ),
            Self::upper_limits(
                2.62,
                -0.2,
                5.17,
                1.79180,
                q[1],
                Self::JOINT_VELOCITY_LIMITS_TOLERANCE[1],
            ),
            Self::upper_limits(
                2.62,
                -0.2,
                7.,
                2.90650,
                q[2],
                Self::JOINT_VELOCITY_LIMITS_TOLERANCE[2],
            ),
            Self::upper_limits(
                2.62,
                -0.3,
                8.,
                -0.1458,
                q[3],
                Self::JOINT_VELOCITY_LIMITS_TOLERANCE[3],
            ),
            Self::upper_limits(
                5.26,
                -0.35,
                34.,
                2.81010,
                q[4],
                Self::JOINT_VELOCITY_LIMITS_TOLERANCE[4],
            ),
            Self::upper_limits(
                4.18,
                -0.35,
                11.,
                4.52050,
                q[5],
                Self::JOINT_VELOCITY_LIMITS_TOLERANCE[5],
            ),
            Self::upper_limits(
                5.26,
                -0.35,
                34.,
                3.01960,
                q[6],
                Self::JOINT_VELOCITY_LIMITS_TOLERANCE[6],
            ),
        ]
    }

    fn compute_lower_limits_joint_velocity(q: &[f64; 7]) -> [f64; 7] {
        [
            Self::lower_limits(
                -2.62,
                -0.3,
                12.,
                2.75010,
                q[0],
                Self::JOINT_VELOCITY_LIMITS_TOLERANCE[0],
            ),
            Self::lower_limits(
                -2.62,
                -0.2,
                5.17,
                1.791800,
                q[1],
                Self::JOINT_VELOCITY_LIMITS_TOLERANCE[1],
            ),
            Self::lower_limits(
                -2.62,
                -0.2,
                7.,
                2.906500,
                q[2],
                Self::JOINT_VELOCITY_LIMITS_TOLERANCE[2],
            ),
            Self::lower_limits(
                -2.62,
                -0.3,
                8.,
                3.048100,
                q[3],
                Self::JOINT_VELOCITY_LIMITS_TOLERANCE[3],
            ),
            Self::lower_limits(
                -5.26,
                -0.35,
                34.,
                2.810100,
                q[4],
                Self::JOINT_VELOCITY_LIMITS_TOLERANCE[4],
            ),
            Self::lower_limits(
                -4.18,
                -0.35,
                11.,
                -0.54092,
                q[5],
                Self::JOINT_VELOCITY_LIMITS_TOLERANCE[5],
            ),
            Self::lower_limits(
                -5.26,
                -0.35,
                34.,
                3.019600,
                q[6],
                Self::JOINT_VELOCITY_LIMITS_TOLERANCE[6],
            ),
        ]
    }
}
