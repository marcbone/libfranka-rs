// Copyright (c) 2021 Marco Boneberger
// Licensed under the EUPL-1.2-or-later

//! Contains the franka::Robot type.
use std::mem::size_of;
use std::time::Duration;

use std::fmt::Debug;

use crate::exception::{create_command_exception, FrankaException, FrankaResult};
use crate::model::Model;
use crate::network::{Network, NetworkType};
use crate::robot::control_loop::ControlLoop;
use crate::robot::control_types::{
    CartesianPose, CartesianVelocities, ControllerMode, Finishable, JointPositions,
    JointVelocities, RealtimeConfig, Torques,
};
use crate::robot::low_pass_filter::{DEFAULT_CUTOFF_FREQUENCY, MAX_CUTOFF_FREQUENCY};
use crate::robot::motion_generator_traits::MotionGeneratorTrait;
use crate::robot::robot_control::RobotControl;
use crate::robot::robot_impl::RobotImpl;
use crate::robot::service_types::{
    AutomaticErrorRecoveryRequestWithHeader, AutomaticErrorRecoveryResponse,
    AutomaticErrorRecoveryStatus, GetCartesianLimitRequest, GetCartesianLimitRequestWithHeader,
    GetCartesianLimitResponse, GetterSetterStatus, RobotCommandEnum, SetCartesianImpedanceRequest,
    SetCartesianImpedanceRequestWithHeader, SetCartesianImpedanceResponse,
    SetCollisionBehaviorRequest, SetCollisionBehaviorRequestWithHeader,
    SetCollisionBehaviorResponse, SetEeToKRequest, SetEeToKRequestWithHeader, SetEeToKResponse,
    SetFiltersRequest, SetFiltersRequestWithHeader, SetFiltersResponse, SetGuidingModeRequest,
    SetGuidingModeRequestWithHeader, SetGuidingModeResponse, SetJointImpedanceRequest,
    SetJointImpedanceRequestWithHeader, SetJointImpedanceResponse, SetLoadRequest,
    SetLoadRequestWithHeader, SetLoadResponse, SetNeToEeRequest, SetNeToEeRequestWithHeader,
    SetNeToEeResponse, StopMoveRequestWithHeader, StopMoveResponse, StopMoveStatus,
};
use crate::robot::virtual_wall_cuboid::VirtualWallCuboid;
use crate::utils::MotionGenerator;
use robot_state::RobotState;

mod control_loop;
mod control_tools;
pub mod control_types;
pub mod error;
pub mod errors;
pub mod logger;
pub mod low_pass_filter;
mod motion_generator_traits;
mod rate_limiting;
mod robot_control;
mod robot_impl;
pub mod robot_state;
pub(crate) mod service_types;
pub(crate) mod types;
pub mod virtual_wall_cuboid;

/// Maintains a network connection to the robot, provides the current robot state, gives access to
/// the model library and allows to control the robot.
/// # Nominal end effector frame NE
/// The nominal end effector frame is configured outside of libfranka-rs and cannot be changed here.
/// # End effector frame EE
/// By default, the end effector frame EE is the same as the nominal end effector frame NE
/// (i.e. the transformation between NE and EE is the identity transformation).
/// With [`set_EE`](Self::set_EE), a custom transformation matrix can be set.
/// # Stiffness frame K
/// The stiffness frame is used for Cartesian impedance control, and for measuring and applying
/// forces.
/// It can be set with [`set_K`](`Self::set_K`).
///
///
/// # Motion generation and joint-level torque commands
///
/// The following methods allow to perform motion generation and/or send joint-level torque
/// commands without gravity and friction by providing callback functions.
///
/// Only one of these methods can be active at the same time; a
/// [`ControlException`](`crate::exception::FrankaException::ControlException`) is thrown otherwise.
///
/// When a robot state is received, the callback function is used to calculate the response: the
/// desired values for that time step. After sending back the response, the callback function will
/// be called again with the most recently received robot state. Since the robot is controlled with
/// a 1 kHz frequency, the callback functions have to compute their result in a short time frame
/// in order to be accepted. Callback functions take two parameters:
///
/// * A &franka::RobotState showing the current robot state.
/// * A &std::time::Duration to indicate the time since the last callback invocation. Thus, the
///   duration is zero on the first invocation of the callback function!
///
/// The following incomplete example shows the general structure of a callback function:
///
/// ```no_run
/// use franka::robot::robot_state::RobotState;
/// use franka::robot::control_types::{JointPositions, MotionFinished};
/// use std::time::Duration;
/// # fn your_function_which_generates_joint_positions(time:f64) -> JointPositions {JointPositions::new([0.;7])}
/// let mut time = 0.;
/// let callback = |state: &RobotState, time_step: &Duration| -> JointPositions {
///     time += time_step.as_secs_f64();
///     let out: JointPositions = your_function_which_generates_joint_positions(time);
///     if time >= 5.0 {
///         return out.motion_finished();
///     }
///     return out;
///     };
/// ```
/// # Commands
///
/// Commands are executed by communicating with the robot over the network.
/// These functions should therefore not be called from within control or motion generator loops.
pub struct Robot {
    robimpl: RobotImpl,
}

impl Robot {
    /// Establishes a connection with the robot.
    ///
    /// # Arguments
    /// * `franka_address` - IP/hostname of the robot.
    /// * `realtime_config` - if set to Enforce, an exception will be thrown if realtime priority
    /// cannot be set when required. Setting realtime_config to Ignore disables this behavior. Default is Enforce
    /// * `log_size` - sets how many last states should be kept for logging purposes.
    /// The log is provided when a [`ControlException`](`crate::exception::FrankaException::ControlException`) is thrown.
    /// # Example
    /// ```no_run
    /// use franka::{FrankaResult, RealtimeConfig, Robot};
    /// fn main() -> FrankaResult<()> {
    ///     // connects to the robot using real-time scheduling and a default log size of 50.
    ///     let mut robot = Robot::new("robotik-bs.de", None, None)?;
    ///     // connects to the robot without using real-time scheduling and a log size of 1.
    ///     let mut robot = Robot::new("robotik-bs.de", RealtimeConfig::Ignore, 1)?;
    ///     Ok(())
    /// }
    /// ```
    /// # Errors
    /// * [`NetworkException`](`crate::exception::FrankaException::NetworkException`) if the connection is unsuccessful.
    /// * IncompatibleVersionException if this version of `libfranka-rs` is not supported.
    pub fn new<RtConfig: Into<Option<RealtimeConfig>>, LogSize: Into<Option<usize>>>(
        franka_address: &str,
        realtime_config: RtConfig,
        log_size: LogSize,
    ) -> FrankaResult<Robot> {
        let realtime_config = realtime_config.into().unwrap_or(RealtimeConfig::Enforce);
        let log_size = log_size.into().unwrap_or(50);
        let network = Network::new(
            NetworkType::Robot,
            franka_address,
            service_types::COMMAND_PORT,
        )
        .map_err(|_| FrankaException::NetworkException {
            message: "Connection could not be established".to_string(),
        })?;
        Ok(Robot {
            robimpl: RobotImpl::new(network, log_size, realtime_config)?,
        })
    }
    /// Starts a loop for reading the current robot state.
    ///
    /// Cannot be executed while a control or motion generator loop is running.
    ///
    /// This minimal example will print the robot state 100 times:
    /// ```no_run
    /// use franka::{Robot, RobotState, FrankaResult};
    /// fn main() -> FrankaResult<()> {
    ///     let mut robot = Robot::new("robotik-bs.de",None,None)?;
    ///     let mut count = 0;
    ///     robot.read(| robot_state:&RobotState | -> bool {
    ///         println!("{:?}", robot_state);
    ///         count += 1;
    ///         count <= 100
    ///     })
    /// }
    /// ```
    /// # Arguments
    /// * `read_callback` - Callback function for robot state reading. The callback hast to return true as long
    /// as it wants to receive further robot states.
    /// # Error
    /// * [`NetworkException`](`crate::exception::FrankaException::NetworkException`) if the connection is lost, e.g. after a timeout.
    pub fn read<F: FnMut(&RobotState) -> bool>(
        &mut self,
        mut read_callback: F,
    ) -> FrankaResult<()> {
        loop {
            let state = self.robimpl.update(None, None)?;
            if !read_callback(&state) {
                break;
            }
        }
        Ok(())
    }
    /// Waits for a robot state update and returns it.
    ///
    /// # Return
    /// Current robot state.
    /// # Error
    /// * [`NetworkException`](`crate::exception::FrankaException::NetworkException`) if the connection is lost, e.g. after a timeout.
    ///
    /// See [`Robot::read`](`Self::read`) for a way to repeatedly receive the robot state.
    pub fn read_once(&mut self) -> FrankaResult<RobotState> {
        self.robimpl.read_once()
    }

    /// Changes the collision behavior.
    ///
    /// Set separate torque and force boundaries for acceleration/deceleration and constant velocity
    /// movement phases.
    ///
    /// Forces or torques between lower and upper threshold are shown as contacts in the RobotState.
    /// Forces or torques above the upper threshold are registered as collision and cause the robot to
    /// stop moving.
    ///
    /// # Arguments
    /// * `lower_torque_thresholds_acceleration` - Contact torque thresholds during
    ///  acceleration/deceleration for each joint in \[Nm\]
    /// * `upper_torque_thresholds_acceleration` - Collision torque thresholds during
    ///  acceleration/deceleration for each joint in \[Nm\]
    /// * `lower_torque_thresholds_nominal` - Contact torque thresholds for each joint in \[Nm\]
    /// * `upper_torque_thresholds_nominal` - Collision torque thresholds for each joint in \[Nm\]
    /// * `lower_force_thresholds_acceleration` -Contact force thresholds during
    /// acceleration/deceleration for (x,y,z,R,P,Y) in \[N\].
    /// * `upper_force_thresholds_acceleration` - Collision force thresholds during
    /// acceleration/deceleration for (x,y,z,R,P,Y) in \[N\].
    /// * `lower_force_thresholds_nominal` - Contact force thresholds for (x,y,z,R,P,Y) in \[N\]
    /// * `upper_force_thresholds_nominal` - Collision force thresholds for (x,y,z,R,P,Y) in \[N\]
    /// # Errors
    /// * [`CommandException`](`crate::exception::FrankaException::CommandException`) if the Control reports an error.
    /// * [`NetworkException`](`crate::exception::FrankaException::NetworkException`) if the connection is lost, e.g. after a timeout.
    /// # See also
    /// * [`RobotState::cartesian_contact`](`crate::RobotState::cartesian_contact`)
    /// * [`RobotState::cartesian_collision`](`crate::RobotState::cartesian_collision`)
    /// * [`RobotState::joint_contact`](`crate::RobotState::joint_contact`)
    /// * [`RobotState::joint_collision`](`crate::RobotState::joint_collision`)
    /// * [`automatic_error_recovery`](`Self::automatic_error_recovery`) for performing a reset after a collision.
    #[allow(clippy::too_many_arguments)]
    pub fn set_collision_behavior(
        &mut self,
        lower_torque_thresholds_acceleration: [f64; 7],
        upper_torque_thresholds_acceleration: [f64; 7],
        lower_torque_thresholds_nominal: [f64; 7],
        upper_torque_thresholds_nominal: [f64; 7],
        lower_force_thresholds_acceleration: [f64; 6],
        upper_force_thresholds_acceleration: [f64; 6],
        lower_force_thresholds_nominal: [f64; 6],
        upper_force_thresholds_nominal: [f64; 6],
    ) -> FrankaResult<()> {
        let command = SetCollisionBehaviorRequestWithHeader {
            header: self.robimpl.network.create_header_for_robot(
                RobotCommandEnum::SetCollisionBehavior,
                size_of::<SetCollisionBehaviorRequestWithHeader>(),
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
        };
        let command_id: u32 = self.robimpl.network.tcp_send_request(command);
        let response: SetCollisionBehaviorResponse = self
            .robimpl
            .network
            .tcp_blocking_receive_response(command_id);
        handle_getter_setter_status(response.status)
    }

    /// Sets the impedance for each joint in the internal controller.
    ///
    /// User-provided torques are not affected by this setting.
    /// # Arguments
    /// * `K_theta` - Joint impedance values ![K_{\theta}](https://latex.codecogs.com/png.latex?K_{\theta}).
    /// # Errors
    /// * [`CommandException`](`crate::exception::FrankaException::CommandException`) if the Control reports an error.
    /// * [`NetworkException`](`crate::exception::FrankaException::NetworkException`) if the connection is lost, e.g. after a timeout.
    #[allow(non_snake_case)]
    pub fn set_joint_impedance(&mut self, K_theta: [f64; 7]) -> FrankaResult<()> {
        let command = SetJointImpedanceRequestWithHeader {
            header: self.robimpl.network.create_header_for_robot(
                RobotCommandEnum::SetJointImpedance,
                size_of::<SetJointImpedanceRequestWithHeader>(),
            ),
            request: SetJointImpedanceRequest::new(K_theta),
        };
        let command_id: u32 = self.robimpl.network.tcp_send_request(command);
        let response: SetJointImpedanceResponse = self
            .robimpl
            .network
            .tcp_blocking_receive_response(command_id);
        handle_getter_setter_status(response.status)
    }

    /// Sets the Cartesian impedance for (x, y, z, roll, pitch, yaw) in the internal controller.
    ///
    /// User-provided torques are not affected by this setting.
    /// # Arguments
    /// * `K_x` - Cartesian impedance values
    ///
    /// ![K_x=(x \in \[10,3000\] \frac{N}{m}, y \in \[10,3000\] \frac{N}{m}, z \in \[10,3000\] \frac{N}{m}, R \in \[1,300\] \frac{Nm}{rad}, P \in \[1,300\] \frac{Nm}{rad}, Y \in \[1,300\]  \frac{Nm}{rad})](https://latex.codecogs.com/png.latex?K_x=(x&space;\in&space;[10,3000]&space;\frac{N}{m},&space;y&space;\in&space;[10,3000]&space;\frac{N}{m},&space;z&space;\in&space;[10,3000]&space;\frac{N}{m},&space;R&space;\in&space;[1,300]&space;\frac{Nm}{rad},&space;P&space;\in&space;[1,300]&space;\frac{Nm}{rad},&space;Y&space;\in&space;[1,300]&space;\frac{Nm}{rad}))
    /// # Errors
    /// * [`CommandException`](`crate::exception::FrankaException::CommandException`) if the Control reports an error.
    /// * [`NetworkException`](`crate::exception::FrankaException::NetworkException`) if the connection is lost, e.g. after a timeout.
    #[allow(non_snake_case)]
    pub fn set_cartesian_impedance(&mut self, K_x: [f64; 6]) -> FrankaResult<()> {
        let command = SetCartesianImpedanceRequestWithHeader {
            header: self.robimpl.network.create_header_for_robot(
                RobotCommandEnum::SetCartesianImpedance,
                size_of::<SetCartesianImpedanceRequestWithHeader>(),
            ),
            request: SetCartesianImpedanceRequest::new(K_x),
        };
        let command_id: u32 = self.robimpl.network.tcp_send_request(command);
        let response: SetCartesianImpedanceResponse = self
            .robimpl
            .network
            .tcp_blocking_receive_response(command_id);
        handle_getter_setter_status(response.status)
    }
    /// Locks or unlocks guiding mode movement in (x, y, z, roll, pitch, yaw).
    ///
    /// If a flag is set to true, movement is unlocked.
    /// # Note
    /// Guiding mode can be enabled by pressing the two opposing buttons near the robot's flange.
    /// # Arguments
    /// * `guiding_mode` - Unlocked movement in (x, y, z, R, P, Y) in guiding mode.
    /// * `elbow` - True if the elbow is free in guiding mode, false otherwise.
    /// # Errors
    /// * [`CommandException`](`crate::exception::FrankaException::CommandException`) if the Control reports an error.
    /// * [`NetworkException`](`crate::exception::FrankaException::NetworkException`) if the connection is lost, e.g. after a timeout.
    pub fn set_guiding_mode(&mut self, guiding_mode: [bool; 6], elbow: bool) -> FrankaResult<()> {
        let command = SetGuidingModeRequestWithHeader {
            header: self.robimpl.network.create_header_for_robot(
                RobotCommandEnum::SetGuidingMode,
                size_of::<SetGuidingModeRequestWithHeader>(),
            ),
            request: SetGuidingModeRequest::new(guiding_mode, elbow),
        };
        let command_id: u32 = self.robimpl.network.tcp_send_request(command);
        let response: SetGuidingModeResponse = self
            .robimpl
            .network
            .tcp_blocking_receive_response(command_id);
        handle_getter_setter_status(response.status)
    }

    /// Sets the transformation ![^{EE}T_K](https://latex.codecogs.com/png.latex?^{EE}T_K) from end effector frame to stiffness frame.
    ///
    /// The transformation matrix is represented as a vectorized 4x4 matrix in column-major format.
    /// # Arguments
    /// * `EE_T_K` - Vectorized EE-to-K transformation matrix ![^{EE}T_K](https://latex.codecogs.com/png.latex?^{EE}T_K), column-major.
    /// # Errors
    /// * [`CommandException`](`crate::exception::FrankaException::CommandException`) if the Control reports an error.
    /// * [`NetworkException`](`crate::exception::FrankaException::NetworkException`) if the connection is lost, e.g. after a timeout.
    ///
    /// See [Stiffness frame K](#stiffness-frame-k) for an explanation of the stiffness frame.
    #[allow(non_snake_case)]
    pub fn set_K(&mut self, EE_T_K: [f64; 16]) -> FrankaResult<()> {
        let command = SetEeToKRequestWithHeader {
            header: self.robimpl.network.create_header_for_robot(
                RobotCommandEnum::SetEeToK,
                size_of::<SetEeToKRequestWithHeader>(),
            ),
            request: SetEeToKRequest::new(EE_T_K),
        };
        let command_id: u32 = self.robimpl.network.tcp_send_request(command);
        let response: SetEeToKResponse = self
            .robimpl
            .network
            .tcp_blocking_receive_response(command_id);
        handle_getter_setter_status(response.status)
    }

    /// Sets the transformation ![^{NE}T_{EE}](https://latex.codecogs.com/png.latex?^{NE}T_{EE}) from nominal end effector to end effector frame.
    ///
    /// The transformation matrix is represented as a vectorized 4x4 matrix in column-major format.
    /// # Arguments
    /// * `NE_T_EE` - Vectorized NE-to-EE transformation matrix ![^{NE}T_{EE}](https://latex.codecogs.com/png.latex?^{NE}T_{EE}), column-major.
    /// # Errors
    /// * [`CommandException`](`crate::exception::FrankaException::CommandException`) if the Control reports an error.
    /// * [`NetworkException`](`crate::exception::FrankaException::NetworkException`) if the connection is lost, e.g. after a timeout.
    ///
    /// # See also
    /// * [`RobotState::NE_T_EE`](`crate::RobotState::NE_T_EE`)
    /// * [`RobotState::O_T_EE`](`crate::RobotState::O_T_EE`)
    /// * [`RobotState::F_T_EE`](`crate::RobotState::F_T_EE`)
    /// * [NE](#nominal-end-effector-frame-ne) and [EE](#end-effector-frame-ee) for an explanation of those frames
    #[allow(non_snake_case)]
    pub fn set_EE(&mut self, NE_T_EE: [f64; 16]) -> FrankaResult<()> {
        let command = SetNeToEeRequestWithHeader {
            header: self.robimpl.network.create_header_for_robot(
                RobotCommandEnum::SetNeToEe,
                size_of::<SetNeToEeRequestWithHeader>(),
            ),
            request: SetNeToEeRequest::new(NE_T_EE),
        };
        let command_id: u32 = self.robimpl.network.tcp_send_request(command);
        let response: SetNeToEeResponse = self
            .robimpl
            .network
            .tcp_blocking_receive_response(command_id);
        handle_getter_setter_status(response.status)
    }
    /// Sets dynamic parameters of a payload.
    ///
    /// The transformation matrix is represented as a vectorized 4x4 matrix in column-major format.
    /// # Note
    /// This is not for setting end effector parameters, which have to be set in the administrator's
    /// interface.
    /// # Arguments
    /// * `load_mass` - Mass of the load in \[kg\]
    /// * `F_x_Cload` - Translation from flange to center of mass of load
    ///  ![^Fx_{C_\text{load}}](https://latex.codecogs.com/png.latex?^Fx_{C_\text{load}}) in \[m\]
    /// * `load_inertia` - Inertia matrix ![I_\text{load}](https://latex.codecogs.com/png.latex?I_\text{load}) in [kg \times m^2], column-major.
    /// # Errors
    /// * [`CommandException`](`crate::exception::FrankaException::CommandException`) if the Control reports an error.
    /// * [`NetworkException`](`crate::exception::FrankaException::NetworkException`) if the connection is lost, e.g. after a timeout.
    #[allow(non_snake_case)]
    pub fn set_load(
        &mut self,
        load_mass: f64,
        F_x_Cload: [f64; 3],
        load_inertia: [f64; 9],
    ) -> FrankaResult<()> {
        let command = SetLoadRequestWithHeader {
            header: self.robimpl.network.create_header_for_robot(
                RobotCommandEnum::SetLoad,
                size_of::<SetLoadRequestWithHeader>(),
            ),
            request: SetLoadRequest::new(load_mass, F_x_Cload, load_inertia),
        };
        let command_id: u32 = self.robimpl.network.tcp_send_request(command);
        let response: SetLoadResponse = self
            .robimpl
            .network
            .tcp_blocking_receive_response(command_id);
        handle_getter_setter_status(response.status)
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
        let command = SetFiltersRequestWithHeader {
            header: self.robimpl.network.create_header_for_robot(
                RobotCommandEnum::SetFilters,
                size_of::<SetFiltersRequestWithHeader>(),
            ),
            request: SetFiltersRequest::new(
                joint_position_filter_frequency,
                joint_velocity_filter_frequency,
                cartesian_position_filter_frequency,
                cartesian_velocity_filter_frequency,
                controller_filter_frequency,
            ),
        };
        let command_id: u32 = self.robimpl.network.tcp_send_request(command);
        let response: SetFiltersResponse = self
            .robimpl
            .network
            .tcp_blocking_receive_response(command_id);
        handle_getter_setter_status(response.status)
    }

    ///Runs automatic error recovery on the robot.
    ///
    /// Automatic error recovery e.g. resets the robot after a collision occurred.
    /// # Errors
    /// * [`CommandException`](`crate::exception::FrankaException::CommandException`) if the Control reports an error.
    /// * [`NetworkException`](`crate::exception::FrankaException::NetworkException`) if the connection is lost, e.g. after a timeout.
    pub fn automatic_error_recovery(&mut self) -> FrankaResult<()> {
        let command = self.robimpl.network.create_header_for_robot(
            RobotCommandEnum::AutomaticErrorRecovery,
            size_of::<AutomaticErrorRecoveryRequestWithHeader>(),
        );
        let command_id: u32 = self.robimpl.network.tcp_send_request(command);
        let response: AutomaticErrorRecoveryResponse = self
            .robimpl
            .network
            .tcp_blocking_receive_response(command_id);
        match &response.status {
            AutomaticErrorRecoveryStatus::Success => Ok(()),
            AutomaticErrorRecoveryStatus::EmergencyAborted => Err(create_command_exception(
                "libfranka-rs: command aborted: User Stop pressed!",
            )),
            AutomaticErrorRecoveryStatus::ReflexAborted => Err(create_command_exception(
                "libfranka-rs: command aborted: motion aborted by reflex!",
            )),
            AutomaticErrorRecoveryStatus::CommandNotPossibleRejected => {
                Err(create_command_exception(
                    "libfranka-rs: command rejected: command not possible in current mode",
                ))
            }
            AutomaticErrorRecoveryStatus::ManualErrorRecoveryRequiredRejected => {
                Err(create_command_exception(
                    "libfranka-rs: command rejected: manual error recovery required!",
                ))
            }
            AutomaticErrorRecoveryStatus::Aborted => {
                Err(create_command_exception("libfranka-rs: command aborted!"))
            }
        }
    }

    /// Stops all currently running motions.
    ///
    /// If a control or motion generator loop is running in another thread, it will be preempted
    /// with a [`ControlException`](`crate::exception::FrankaException::ControlException`).
    /// # Errors
    /// * [`CommandException`](`crate::exception::FrankaException::CommandException`) if the Control reports an error.
    /// * [`NetworkException`](`crate::exception::FrankaException::NetworkException`) if the connection is lost, e.g. after a timeout.
    pub fn stop(&mut self) -> FrankaResult<()> {
        let command = StopMoveRequestWithHeader {
            header: self.robimpl.network.create_header_for_robot(
                RobotCommandEnum::StopMove,
                size_of::<StopMoveRequestWithHeader>(),
            ),
        };
        let command_id: u32 = self.robimpl.network.tcp_send_request(command);
        let response: StopMoveResponse = self
            .robimpl
            .network
            .tcp_blocking_receive_response(command_id);
        match response.status {
            StopMoveStatus::Success => Ok(()),
            StopMoveStatus::CommandNotPossibleRejected | StopMoveStatus::Aborted => {
                Err(create_command_exception(
                    "libfranka-rs: command rejected: command not possible in current mode",
                ))
            }
            StopMoveStatus::EmergencyAborted => Err(create_command_exception(
                "libfranka-rs: command aborted: User Stop pressed!",
            )),
            StopMoveStatus::ReflexAborted => Err(create_command_exception(
                "libfranka-rs: command aborted: motion aborted by reflex!",
            )),
        }
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
        let command = GetCartesianLimitRequestWithHeader {
            header: self.robimpl.network.create_header_for_robot(
                RobotCommandEnum::GetCartesianLimit,
                size_of::<GetCartesianLimitRequestWithHeader>(),
            ),
            request: GetCartesianLimitRequest::new(id),
        };
        let command_id: u32 = self.robimpl.network.tcp_send_request(command);
        let response: GetCartesianLimitResponse = self
            .robimpl
            .network
            .tcp_blocking_receive_response(command_id);
        match &response.status {
            GetterSetterStatus::Success => Ok(VirtualWallCuboid::new(id, response)),
            GetterSetterStatus::CommandNotPossibleRejected => Err(create_command_exception(
                "libfranka-rs: command rejected: command not possible in current mode",
            )),
            GetterSetterStatus::InvalidArgumentRejected => Err(create_command_exception(
                "libfranka-rs: command rejected: invalid argument!",
            )),
        }
    }
    /// Starts a control loop for a joint position motion generator with a given controller mode.
    ///
    /// Sets realtime priority for the current thread.
    /// Cannot be executed while another control or motion generator loop is active.
    ///
    /// # Arguments
    /// * `motion_generator_callback` Callback function for motion generation.
    /// See [here](#motion-generation-and-joint-level-torque-commands) for more details.
    /// * `controller_mode` Controller to use to execute the motion. Default is joint impedance
    /// * `limit_rate` True if rate limiting should be activated. True by default.
    /// This could distort your motion!
    /// * `cutoff_frequency` Cutoff frequency for a first order low-pass filter applied on
    /// the user commanded signal.
    /// Set to [`MAX_CUTOFF_FREQUENCY`](`crate::robot::low_pass_filter::MAX_CUTOFF_FREQUENCY`) to disable.
    /// Default is 100 Hz
    ///
    /// # Errors
    /// * [`ControlException`](`crate::exception::FrankaException::ControlException`) if an error related to torque control or motion generation occurred.
    /// * [`NetworkException`](`crate::exception::FrankaException::NetworkException`) if the connection is lost, e.g. after a timeout.
    /// * [`RealTimeException`](`crate::exception::FrankaException::RealTimeException`) if realtime priority cannot be set for the current thread.
    /// # Panics
    /// * if joint position commands are NaN or infinity.
    ///
    /// See [`new`](`Self::new`) to change behavior if realtime priority cannot be set.
    pub fn control_joint_positions<
        F: FnMut(&RobotState, &Duration) -> JointPositions,
        CM: Into<Option<ControllerMode>>,
        L: Into<Option<bool>>,
        CF: Into<Option<f64>>,
    >(
        &mut self,
        motion_generator_callback: F,
        controller_mode: CM,
        limit_rate: L,
        cutoff_frequency: CF,
    ) -> FrankaResult<()> {
        self.control_motion_intern(
            motion_generator_callback,
            controller_mode.into(),
            limit_rate.into(),
            cutoff_frequency.into(),
        )
    }
    /// Starts a control loop for a joint velocity motion generator with a given controller mode.
    ///
    /// Sets realtime priority for the current thread.
    /// Cannot be executed while another control or motion generator loop is active.
    ///
    /// # Arguments
    /// * `motion_generator_callback` Callback function for motion generation.
    /// See [here](#motion-generation-and-joint-level-torque-commands) for more details.
    /// * `controller_mode` Controller to use to execute the motion. Default is joint impedance
    /// * `limit_rate` True if rate limiting should be activated. True by default.
    /// This could distort your motion!
    /// * `cutoff_frequency` Cutoff frequency for a first order low-pass filter applied on
    /// the user commanded signal.
    /// Set to [`MAX_CUTOFF_FREQUENCY`](`crate::robot::low_pass_filter::MAX_CUTOFF_FREQUENCY`) to disable.
    /// Default is 100 Hz
    /// # Errors
    /// * [`ControlException`](`crate::exception::FrankaException::ControlException`) if an error related to torque control or motion generation occurred.
    /// * [`NetworkException`](`crate::exception::FrankaException::NetworkException`) if the connection is lost, e.g. after a timeout.
    /// * [`RealTimeException`](`crate::exception::FrankaException::RealTimeException`) if realtime priority cannot be set for the current thread.
    /// # Panics
    /// * if joint velocity commands are NaN or infinity.
    ///
    /// See [`new`](`Self::new`) to change behavior if realtime priority cannot be set.
    pub fn control_joint_velocities<
        F: FnMut(&RobotState, &Duration) -> JointVelocities,
        CM: Into<Option<ControllerMode>>,
        L: Into<Option<bool>>,
        CF: Into<Option<f64>>,
    >(
        &mut self,
        motion_generator_callback: F,
        controller_mode: CM,
        limit_rate: L,
        cutoff_frequency: CF,
    ) -> FrankaResult<()> {
        self.control_motion_intern(
            motion_generator_callback,
            controller_mode.into(),
            limit_rate.into(),
            cutoff_frequency.into(),
        )
    }
    /// Starts a control loop for a Cartesian pose motion generator with a given controller mode.
    ///
    /// Sets realtime priority for the current thread.
    /// Cannot be executed while another control or motion generator loop is active.
    ///
    /// # Arguments
    /// * `motion_generator_callback` Callback function for motion generation.
    /// See [here](#motion-generation-and-joint-level-torque-commands) for more details.
    /// * `controller_mode` Controller to use to execute the motion. Default is joint impedance
    /// * `limit_rate` True if rate limiting should be activated. True by default.
    /// This could distort your motion!
    /// * `cutoff_frequency` Cutoff frequency for a first order low-pass filter applied on
    /// the user commanded signal.
    /// Set to [`MAX_CUTOFF_FREQUENCY`](`crate::robot::low_pass_filter::MAX_CUTOFF_FREQUENCY`) to disable.
    /// Default is 100 Hz
    ///
    /// # Errors
    /// * [`ControlException`](`crate::exception::FrankaException::ControlException`) if an error related to torque control or motion generation occurred.
    /// * [`NetworkException`](`crate::exception::FrankaException::NetworkException`) if the connection is lost, e.g. after a timeout.
    /// * [`RealTimeException`](`crate::exception::FrankaException::RealTimeException`) if realtime priority cannot be set for the current thread.
    /// # Panics
    /// * if Cartesian pose command elements are NaN or infinity.
    ///
    /// See [`new`](`Self::new`) to change behavior if realtime priority cannot be set.
    pub fn control_cartesian_pose<
        F: FnMut(&RobotState, &Duration) -> CartesianPose,
        CM: Into<Option<ControllerMode>>,
        L: Into<Option<bool>>,
        CF: Into<Option<f64>>,
    >(
        &mut self,
        motion_generator_callback: F,
        controller_mode: CM,
        limit_rate: L,
        cutoff_frequency: CF,
    ) -> FrankaResult<()> {
        self.control_motion_intern(
            motion_generator_callback,
            controller_mode.into(),
            limit_rate.into(),
            cutoff_frequency.into(),
        )
    }
    /// Starts a control loop for a Cartesian velocity motion generator with a given controller mode.
    ///
    /// Sets realtime priority for the current thread.
    /// Cannot be executed while another control or motion generator loop is active.
    ///
    /// # Arguments
    /// * `motion_generator_callback` Callback function for motion generation.
    /// See [here](#motion-generation-and-joint-level-torque-commands) for more details.
    /// * `controller_mode` Controller to use to execute the motion. Default is joint impedance
    /// * `limit_rate` True if rate limiting should be activated. True by default.
    /// This could distort your motion!
    /// * `cutoff_frequency` Cutoff frequency for a first order low-pass filter applied on
    /// the user commanded signal.
    /// Set to [`MAX_CUTOFF_FREQUENCY`](`crate::robot::low_pass_filter::MAX_CUTOFF_FREQUENCY`) to disable.
    /// Default is 100 Hz
    ///
    /// # Errors
    /// * [`ControlException`](`crate::exception::FrankaException::ControlException`) if an error related to torque control or motion generation occurred.
    /// * [`NetworkException`](`crate::exception::FrankaException::NetworkException`) if the connection is lost, e.g. after a timeout.
    /// * [`RealTimeException`](`crate::exception::FrankaException::RealTimeException`) if realtime priority cannot be set for the current thread.
    /// # Panics
    /// * if Cartesian velocity command elements are NaN or infinity.
    ///
    /// See [`new`](`Self::new`) to change behavior if realtime priority cannot be set.
    pub fn control_cartesian_velocities<
        F: FnMut(&RobotState, &Duration) -> CartesianVelocities,
        CM: Into<Option<ControllerMode>>,
        L: Into<Option<bool>>,
        CF: Into<Option<f64>>,
    >(
        &mut self,
        motion_generator_callback: F,
        controller_mode: CM,
        limit_rate: L,
        cutoff_frequency: CF,
    ) -> FrankaResult<()> {
        self.control_motion_intern(
            motion_generator_callback,
            controller_mode.into(),
            limit_rate.into(),
            cutoff_frequency.into(),
        )
    }
    /// Starts a control loop for sending joint-level torque commands and joint velocities.
    ///
    /// Sets realtime priority for the current thread.
    /// Cannot be executed while another control or motion generator loop is active.
    ///
    /// # Arguments
    /// * `control_callback` Callback function providing joint-level torque commands.
    /// See [here](#motion-generation-and-joint-level-torque-commands) for more details.
    /// * `motion_generator_callback` Callback function for motion generation.
    /// See [here](#motion-generation-and-joint-level-torque-commands) for more details.
    /// * `limit_rate` True if rate limiting should be activated. True by default.
    /// This could distort your motion!
    /// * `cutoff_frequency` Cutoff frequency for a first order low-pass filter applied on
    /// the user commanded signal.
    /// Set to [`MAX_CUTOFF_FREQUENCY`](`crate::robot::low_pass_filter::MAX_CUTOFF_FREQUENCY`) to disable.
    /// Default is 100 Hz
    /// # Errors
    /// * [`ControlException`](`crate::exception::FrankaException::ControlException`) if an error related to torque control or motion generation occurred.
    /// * [`NetworkException`](`crate::exception::FrankaException::NetworkException`) if the connection is lost, e.g. after a timeout.
    /// * [`RealTimeException`](`crate::exception::FrankaException::RealTimeException`) if realtime priority cannot be set for the current thread.
    /// # Panics
    /// * if joint-level torque or joint velocity commands are NaN or infinity.
    ///
    /// See [`new`](`Self::new`) to change behavior if realtime priority cannot be set.
    pub fn control_torques_and_joint_velocities<
        F: FnMut(&RobotState, &Duration) -> JointVelocities,
        T: FnMut(&RobotState, &Duration) -> Torques,
        L: Into<Option<bool>>,
        CF: Into<Option<f64>>,
    >(
        &mut self,
        mut control_callback: T,
        motion_generator_callback: F,
        limit_rate: L,
        cutoff_frequency: CF,
    ) -> FrankaResult<()> {
        self.control_torques_intern(
            motion_generator_callback,
            &mut control_callback,
            limit_rate.into(),
            cutoff_frequency.into(),
        )
    }
    /// Starts a control loop for sending joint-level torque commands and joint positions.
    ///
    /// Sets realtime priority for the current thread.
    /// Cannot be executed while another control or motion generator loop is active.
    ///
    /// # Arguments
    /// * `control_callback` Callback function providing joint-level torque commands.
    /// See [here](#motion-generation-and-joint-level-torque-commands) for more details.
    /// * `motion_generator_callback` Callback function for motion generation.
    /// See [here](#motion-generation-and-joint-level-torque-commands) for more details.
    /// * `limit_rate` True if rate limiting should be activated. True by default.
    /// This could distort your motion!
    /// * `cutoff_frequency` Cutoff frequency for a first order low-pass filter applied on
    /// the user commanded signal.
    /// Set to [`MAX_CUTOFF_FREQUENCY`](`crate::robot::low_pass_filter::MAX_CUTOFF_FREQUENCY`) to disable.
    /// Default is 100 Hz
    /// # Errors
    /// * [`ControlException`](`crate::exception::FrankaException::ControlException`) if an error related to torque control or motion generation occurred.
    /// * [`NetworkException`](`crate::exception::FrankaException::NetworkException`) if the connection is lost, e.g. after a timeout.
    /// * [`RealTimeException`](`crate::exception::FrankaException::RealTimeException`) if realtime priority cannot be set for the current thread.
    /// # Panics
    /// * if joint-level torque or joint position commands are NaN or infinity.
    ///
    /// See [`new`](`Self::new`) to change behavior if realtime priority cannot be set.
    pub fn control_torques_and_joint_positions<
        F: FnMut(&RobotState, &Duration) -> JointPositions,
        T: FnMut(&RobotState, &Duration) -> Torques,
        L: Into<Option<bool>>,
        CF: Into<Option<f64>>,
    >(
        &mut self,
        mut control_callback: T,
        motion_generator_callback: F,
        limit_rate: L,
        cutoff_frequency: CF,
    ) -> FrankaResult<()> {
        self.control_torques_intern(
            motion_generator_callback,
            &mut control_callback,
            limit_rate.into(),
            cutoff_frequency.into(),
        )
    }

    /// Starts a control loop for sending joint-level torque commands and Cartesian poses.
    ///
    /// Sets realtime priority for the current thread.
    /// Cannot be executed while another control or motion generator loop is active.
    ///
    /// # Arguments
    /// * `control_callback` Callback function providing joint-level torque commands.
    /// See [here](#motion-generation-and-joint-level-torque-commands) for more details.
    /// * `motion_generator_callback` Callback function for motion generation.
    /// See [here](#motion-generation-and-joint-level-torque-commands) for more details.
    /// * `limit_rate` True if rate limiting should be activated. True by default.
    /// This could distort your motion!
    /// * `cutoff_frequency` Cutoff frequency for a first order low-pass filter applied on
    /// the user commanded signal.
    /// Set to [`MAX_CUTOFF_FREQUENCY`](`crate::robot::low_pass_filter::MAX_CUTOFF_FREQUENCY`) to disable.
    /// Default is 100 Hz
    /// # Errors
    /// * [`ControlException`](`crate::exception::FrankaException::ControlException`) if an error related to torque control or motion generation occurred.
    /// * [`NetworkException`](`crate::exception::FrankaException::NetworkException`) if the connection is lost, e.g. after a timeout.
    /// * [`RealTimeException`](`crate::exception::FrankaException::RealTimeException`) if realtime priority cannot be set for the current thread.
    /// # Panics
    /// * if joint-level torque or Cartesian pose command elements are NaN or infinity.
    ///
    /// See [`new`](`Self::new`) to change behavior if realtime priority cannot be set.
    pub fn control_torques_and_cartesian_pose<
        F: FnMut(&RobotState, &Duration) -> CartesianPose,
        T: FnMut(&RobotState, &Duration) -> Torques,
        L: Into<Option<bool>>,
        CF: Into<Option<f64>>,
    >(
        &mut self,
        mut control_callback: T,
        motion_generator_callback: F,
        limit_rate: L,
        cutoff_frequency: CF,
    ) -> FrankaResult<()> {
        self.control_torques_intern(
            motion_generator_callback,
            &mut control_callback,
            limit_rate.into(),
            cutoff_frequency.into(),
        )
    }

    /// Starts a control loop for sending joint-level torque commands and Cartesian velocities.
    ///
    /// Sets realtime priority for the current thread.
    /// Cannot be executed while another control or motion generator loop is active.
    ///
    /// # Arguments
    /// * `control_callback` Callback function providing joint-level torque commands.
    /// See [here](#motion-generation-and-joint-level-torque-commands) for more details.
    /// * `motion_generator_callback` Callback function for motion generation.
    /// See [here](#motion-generation-and-joint-level-torque-commands) for more details.
    /// * `limit_rate` True if rate limiting should be activated. True by default.
    /// This could distort your motion!
    /// * `cutoff_frequency` Cutoff frequency for a first order low-pass filter applied on
    /// the user commanded signal.
    /// Set to [`MAX_CUTOFF_FREQUENCY`](`crate::robot::low_pass_filter::MAX_CUTOFF_FREQUENCY`) to disable.
    /// Default is 100 Hz
    /// # Errors
    /// * [`ControlException`](`crate::exception::FrankaException::ControlException`) if an error related to torque control or motion generation occurred.
    /// * [`NetworkException`](`crate::exception::FrankaException::NetworkException`) if the connection is lost, e.g. after a timeout.
    /// * [`RealTimeException`](`crate::exception::FrankaException::RealTimeException`) if realtime priority cannot be set for the current thread.
    /// # Panics
    /// * if joint-level torque or Cartesian velocity command elements are NaN or infinity.
    ///
    /// See [`new`](`Self::new`) to change behavior if realtime priority cannot be set.
    pub fn control_torques_and_cartesian_velocities<
        F: FnMut(&RobotState, &Duration) -> CartesianVelocities,
        T: FnMut(&RobotState, &Duration) -> Torques,
        L: Into<Option<bool>>,
        CF: Into<Option<f64>>,
    >(
        &mut self,
        mut control_callback: T,
        motion_generator_callback: F,
        limit_rate: L,
        cutoff_frequency: CF,
    ) -> FrankaResult<()> {
        self.control_torques_intern(
            motion_generator_callback,
            &mut control_callback,
            limit_rate.into(),
            cutoff_frequency.into(),
        )
    }
    /// Starts a control loop for sending joint-level torque commands.
    ///
    /// Sets real-time priority for the current thread.
    /// Cannot be executed while another control or motion generator loop is active.
    ///
    /// # Arguments
    /// * `control_callback` - Callback function providing joint-level torque commands.
    /// See [here](#motion-generation-and-joint-level-torque-commands) for more details.
    /// * `limit_rate` - True if rate limiting should be activated. True by default.
    /// This could distort your motion!
    /// * `cutoff_frequency` - Cutoff frequency for a first order low-pass filter applied on
    /// the user commanded signal. Set to
    /// [`MAX_CUTOFF_FREQUENCY`](`crate::robot::low_pass_filter::MAX_CUTOFF_FREQUENCY`)
    /// to disable. Default is 100 Hz
    ///
    /// # Errors
    /// * [`ControlException`](`crate::exception::FrankaException::ControlException`)
    /// if an error related to torque control or motion generation occurred.
    /// * [`NetworkException`](`crate::exception::FrankaException::NetworkException`)
    /// if the connection is lost, e.g. after a timeout.
    /// * [`RealTimeException`](`crate::exception::FrankaException::RealTimeException`)
    /// if real-time priority cannot be set for the current thread.
    /// # Panics
    /// * if joint-level torque commands are NaN or infinity.
    ///
    /// See [`new`](`Self::new`) to change behavior if real-time priority cannot be set.
    pub fn control_torques<
        T: FnMut(&RobotState, &Duration) -> Torques,
        L: Into<Option<bool>>,
        CF: Into<Option<f64>>,
    >(
        &mut self,
        mut control_callback: T,
        limit_rate: L,
        cutoff_frequency: CF,
    ) -> FrankaResult<()> {
        let motion_generator_callback =
            |_state: &RobotState, _time_step: &Duration| JointVelocities::new([0.; 7]);
        self.control_torques_intern(
            &motion_generator_callback,
            &mut control_callback,
            limit_rate.into(),
            cutoff_frequency.into(),
        )
    }
    fn control_torques_intern<
        F: FnMut(&RobotState, &Duration) -> U,
        U: Finishable + Debug + MotionGeneratorTrait,
    >(
        &mut self,
        motion_generator_callback: F,
        control_callback: &mut dyn FnMut(&RobotState, &Duration) -> Torques,
        limit_rate: Option<bool>,
        cutoff_frequency: Option<f64>,
    ) -> FrankaResult<()> {
        let limit_rate = limit_rate.unwrap_or(true);
        let cutoff_frequency = cutoff_frequency.unwrap_or(DEFAULT_CUTOFF_FREQUENCY);
        let mut control_loop = ControlLoop::new(
            &mut self.robimpl,
            control_callback,
            motion_generator_callback,
            limit_rate,
            cutoff_frequency,
        )?;
        control_loop.run()
    }
    fn control_motion_intern<
        F: FnMut(&RobotState, &Duration) -> U,
        U: Finishable + Debug + MotionGeneratorTrait,
    >(
        &mut self,
        motion_generator_callback: F,
        controller_mode: Option<ControllerMode>,
        limit_rate: Option<bool>,
        cutoff_frequency: Option<f64>,
    ) -> FrankaResult<()> {
        let controller_mode = controller_mode.unwrap_or(ControllerMode::JointImpedance);
        let limit_rate = limit_rate.unwrap_or(true);
        let cutoff_frequency = cutoff_frequency.unwrap_or(DEFAULT_CUTOFF_FREQUENCY);
        let mut control_loop = ControlLoop::from_control_mode(
            &mut self.robimpl,
            controller_mode,
            motion_generator_callback,
            limit_rate,
            cutoff_frequency,
        )?;
        control_loop.run()
    }
    /// Sets a default collision behavior, joint impedance and Cartesian impedance.
    /// # Errors
    /// * [`CommandException`](`crate::exception::FrankaException::CommandException`) if the Control reports an error.
    /// * [`NetworkException`](`crate::exception::FrankaException::NetworkException`) if the connection is lost, e.g. after a timeout.
    pub fn set_default_behavior(&mut self) -> FrankaResult<()> {
        self.set_collision_behavior(
            [20.; 7], [20.; 7], [10.; 7], [10.; 7], [20.; 6], [20.; 6], [10.; 6], [10.; 6],
        )?;
        self.set_joint_impedance([3000., 3000., 3000., 2500., 2500., 2000., 2000.])?;
        self.set_cartesian_impedance([3000., 3000., 3000., 300., 300., 300.])?;
        Ok(())
    }
    /// Executes a joint pose motion to a goal position. Adapted from:
    /// Wisama Khalil and Etienne Dombre. 2002. Modeling, Identification and Control of Robots
    /// (Kogan Page Science Paper edition).
    /// # Arguments
    /// * `speed_factor` - General speed factor in range [0, 1].
    /// * `q_goal` - Target joint positions.
    pub fn joint_motion(&mut self, speed_factor: f64, q_goal: &[f64; 7]) -> FrankaResult<()> {
        let mut motion_generator = MotionGenerator::new(speed_factor, q_goal);
        self.control_joint_positions(
            |state, time| motion_generator.generate_motion(state, time),
            Some(ControllerMode::JointImpedance),
            Some(true),
            Some(MAX_CUTOFF_FREQUENCY),
        )
    }
    /// Loads the model library from the robot.
    /// # Arguments
    /// * `persistent` - If set to true the model will be stored at `/tmp/model.so`
    /// # Return
    /// Model instance.
    /// # Errors
    /// * [`ModelException`](`crate::exception::FrankaException::ModelException`) if the model library cannot be loaded.
    /// * [`NetworkException`](`crate::exception::FrankaException::NetworkException`) if the connection is lost, e.g. after a timeout.
    pub fn load_model(&mut self, persistent: bool) -> FrankaResult<Model> {
        self.robimpl.load_model(persistent)
    }
    /// Returns the software version reported by the connected server.
    ///
    /// # Return
    /// Software version of the connected server.
    pub fn server_version(&self) -> u16 {
        self.robimpl.server_version()
    }
}

fn handle_getter_setter_status(status: GetterSetterStatus) -> FrankaResult<()> {
    match status {
        GetterSetterStatus::Success => Ok(()),
        GetterSetterStatus::CommandNotPossibleRejected => Err(create_command_exception(
            "libfranka-rs: command rejected: command not possible in current mode",
        )),
        GetterSetterStatus::InvalidArgumentRejected => Err(create_command_exception(
            "libfranka-rs: command rejected: invalid argument!",
        )),
    }
}
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
    use crate::robot::robot_impl::VERSION;
    use crate::robot::service_types::{
        ConnectRequestWithHeader, ConnectResponse, ConnectStatus, GetterSetterStatus,
        MoveControllerMode, MoveDeviation, MoveMotionGeneratorMode, MoveRequest,
        MoveRequestWithHeader, MoveResponse, MoveStatus, RobotCommandEnum, RobotCommandHeader,
        SetCollisionBehaviorRequest, SetCollisionBehaviorRequestWithHeader,
        SetCollisionBehaviorResponse, COMMAND_PORT,
    };
    use crate::robot::types::RobotStateIntern;
    use crate::{FrankaResult, JointPositions, MotionFinished, RealtimeConfig, Robot, RobotState};
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
                    let mut state = RobotStateIntern::dummy();
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
        ) -> ConnectRequestWithHeader {
            let mut bytes = vec![0 as u8; 100];
            (tcp_socket.receive_bytes)(&mut bytes);
            let request: ConnectRequestWithHeader = deserialize(bytes.as_slice()).unwrap();
            return request;
        }
        fn send_robot_connect_response<F: Fn(&Vec<u8>), G: Fn(&mut Vec<u8>)>(
            &self,
            request: ConnectRequestWithHeader,
            tcp_socket: &mut Socket<F, G>,
        ) {
            let mut response = ConnectResponse {
                header: RobotCommandHeader {
                    command: RobotCommandEnum::Connect,
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
                SetCollisionBehaviorRequestWithHeader {
                    header: RobotCommandHeader::new(
                        RobotCommandEnum::SetCollisionBehavior,
                        counter,
                        size_of::<SetCollisionBehaviorRequestWithHeader>() as u32,
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
            let mut robot_server = RobotMockServer::new(VERSION);
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
                    let req: SetCollisionBehaviorRequestWithHeader = deserialize(&bytes).unwrap();
                    counter += 1;
                    let mut response = SetCollisionBehaviorResponse {
                        header: RobotCommandHeader::new(
                            RobotCommandEnum::SetCollisionBehavior,
                            req.header.command_id,
                            0,
                        ),
                        status: GetterSetterStatus::Success,
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
            let mut robot = Robot::new("127.0.0.1", None, None).expect("robot failure");
            assert_eq!(robot.server_version(), VERSION);
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
        let requests = Arc::new(vec![MoveRequestWithHeader {
            header: RobotCommandHeader::new(
                RobotCommandEnum::Move,
                1,
                size_of::<MoveRequestWithHeader>() as u32,
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
            let mut robot_server = RobotMockServer::new(VERSION);
            let mut mock = MockServerReaction::default();
            let num_requests = requests_server.len();
            let mut counter = 0;
            mock.expect_process_received_bytes()
                .returning(move |bytes: &mut Vec<u8>| -> Vec<u8> {
                    let expected_request = requests_server.get(counter).unwrap();
                    let serialized_expected_request = serialize(expected_request).unwrap();
                    let req: MoveRequestWithHeader = deserialize(&bytes).unwrap();
                    assert_eq!(bytes.len(), serialized_expected_request.len());
                    bytes
                        .iter()
                        .zip(serialized_expected_request.iter())
                        .for_each(|(x, y)| assert_eq!(x, y));
                    counter += 1;
                    let mut response = MoveResponse {
                        header: RobotCommandHeader::new(
                            RobotCommandEnum::Move,
                            req.header.command_id,
                            0,
                        ),
                        status: MoveStatus::Aborted,
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
            let mut robot =
                Robot::new("127.0.0.1", RealtimeConfig::Ignore, None).expect("robot failure");
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
            let mut robot_server = RobotMockServer::new(VERSION + 1);
            let mut mock = MockServerReaction::default();
            mock.expect_process_received_bytes()
                .returning(|_bytes| Vec::<u8>::new());
            mock.expect_number_of_reactions().return_const(0 as usize);
            robot_server.server_thread(&mut mock);
        });
        std::thread::sleep(Duration::from_secs_f64(0.01));
        let robot_result = Robot::new("127.0.0.1", None, None);

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
            let mut robot_server = RobotMockServer::new(VERSION);
            let mut mock = MockServerReaction::default();
            mock.expect_process_received_bytes()
                .returning(|_bytes| Vec::<u8>::new());
            mock.expect_number_of_reactions().return_const(0 as usize);
            robot_server.server_thread(&mut mock);
        });

        {
            std::thread::sleep(Duration::from_secs_f64(0.01));
            let mut robot = Robot::new("127.0.0.1", None, None)?;
            let _state = robot.read_once().unwrap();
        }
        thread.join().unwrap();
        Ok(())
    }

    #[test]
    fn robot_read() -> FrankaResult<()> {
        let thread = std::thread::spawn(|| {
            let mut robot_server = RobotMockServer::new(VERSION);
            let mut mock = MockServerReaction::default();
            mock.expect_process_received_bytes()
                .returning(|_bytes| Vec::<u8>::new());
            mock.expect_number_of_reactions().return_const(0 as usize);
            robot_server.server_thread(&mut mock);
        });
        {
            std::thread::sleep(Duration::from_secs_f64(0.01));
            let mut robot = Robot::new("127.0.0.1", None, None)?;
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
