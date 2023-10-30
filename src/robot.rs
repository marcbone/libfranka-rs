// Copyright (c) 2021 Marco Boneberger
// Licensed under the EUPL-1.2-or-later

//! Contains the Robot trait and all modules that are necessary to use a robot.

use crate::exception::create_command_exception;
use crate::robot::private_robot::PrivateRobot;
use crate::robot::robot_control::RobotControl;
pub(crate) use crate::robot::robot_data::PrivateRobotData;
use crate::robot::robot_impl::RobotImplementation;
use crate::robot::service_types::{
    SetCartesianImpedanceRequest, SetCollisionBehaviorRequest, SetEeToKRequest,
    SetGuidingModeRequest, SetJointImpedanceRequest, SetLoadRequest, SetNeToEeRequest,
    StopMoveStatusPanda,
};
use crate::{
    CartesianPose, CartesianVelocities, ControllerMode, FrankaResult, JointPositions,
    JointVelocities, MotionGenerator, RobotModel, Torques, MAX_CUTOFF_FREQUENCY,
};
use std::time::Duration;

mod control_loop;
mod control_tools;
pub mod control_types;
pub mod error;
pub mod errors;
mod fr3;
pub mod logger;
pub mod low_pass_filter;
mod motion_generator_traits;
mod panda;
mod private_robot;
pub mod rate_limiting;
mod robot_control;
mod robot_data;
mod robot_impl;
pub(crate) mod robot_state;
pub(crate) mod service_types;
pub(crate) mod types;
pub mod virtual_wall_cuboid;

pub use fr3::Fr3;
pub use panda::Panda;
pub use robot_data::RobotData;
pub use robot_state::RobotState;

/// Contains all methods that are available for all robots.
/// # Nominal end effector frame NE
/// The nominal end effector frame is configured outside of libfranka-rs and cannot be changed here.
/// # End effector frame EE
/// By default, the end effector frame EE is the same as the nominal end effector frame NE
/// (i.e. the transformation between NE and EE is the identity transformation).
/// With [`set_EE`](`Robot::set_EE), a custom transformation matrix can be set.
/// # Stiffness frame K
/// The stiffness frame is used for Cartesian impedance control, and for measuring and applying
/// forces.
/// It can be set with [`set_K`](Robot::set_K).
///
///
/// # Motion generation and joint-level torque commands
///
/// The following methods allow to perform motion generation and/or send joint-level torque
/// commands without gravity and friction by providing callback functions.
///
/// Only one of these methods can be active at the same time; a
/// [`ControlException`](crate::exception::FrankaException::ControlException) is thrown otherwise.
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
///
pub trait Robot {
    /// Waits for a robot state update and returns it.
    ///
    /// # Return
    /// Current robot state.
    /// # Error
    /// * [`NetworkException`](`crate::exception::FrankaException::NetworkException`) if the connection is lost, e.g. after a timeout.
    ///
    /// See [`Robot::read`](`Self::read`) for a way to repeatedly receive the robot state.
    fn read_once(&mut self) -> FrankaResult<RobotState>;

    /// Starts a loop for reading the current robot state.
    ///
    /// Cannot be executed while a control or motion generator loop is running.
    ///
    /// This minimal example will print the robot state 100 times:
    /// ```no_run
    /// use franka::{Fr3, RobotState, Robot, FrankaResult};
    /// fn main() -> FrankaResult<()> {
    ///     let mut robot = Fr3::new("robotik-bs.de",None,None)?;
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
    fn read<F: FnMut(&RobotState) -> bool>(&mut self, read_callback: F) -> FrankaResult<()>;

    /// Executes a joint pose motion to a goal position. Adapted from:
    /// Wisama Khalil and Etienne Dombre. 2002. Modeling, Identification and Control of Robots
    /// (Kogan Page Science Paper edition).
    /// # Arguments
    /// * `speed_factor` - General speed factor in range [0, 1].
    /// * `q_goal` - Target joint positions.
    fn joint_motion(&mut self, speed_factor: f64, q_goal: &[f64; 7]) -> FrankaResult<()>;

    /// Starts a control loop for a joint position motion generator with a given controller mode.
    ///
    /// Sets realtime priority for the current thread.
    /// Cannot be executed while another control or motion generator loop is active.
    ///
    /// # Arguments
    /// * `motion_generator_callback` - Callback function for motion generation.
    /// See [here](#motion-generation-and-joint-level-torque-commands) for more details.
    /// * `controller_mode` - Controller to use to execute the motion. Default is joint impedance
    /// * `limit_rate` - Set to true to activate the rate limiter.
    /// The default value is defined for each robot type over
    /// [`RATE_LIMITING_ON_PER_DEFAULT`](`crate::robot::rate_limiting::RateLimiter::RATE_LIMITING_ON_PER_DEFAULT`).
    /// Enabling the rate limiter can distort your motion!
    /// * `cutoff_frequency` - Cutoff frequency for a first order low-pass filter applied on
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
    fn control_joint_positions<
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
    ) -> FrankaResult<()>;

    /// Starts a control loop for a joint velocity motion generator with a given controller mode.
    ///
    /// Sets realtime priority for the current thread.
    /// Cannot be executed while another control or motion generator loop is active.
    ///
    /// # Arguments
    /// * `motion_generator_callback` - Callback function for motion generation.
    /// See [here](#motion-generation-and-joint-level-torque-commands) for more details.
    /// * `controller_mode` - Controller to use to execute the motion. Default is joint impedance
    /// * `limit_rate` - Set to true to activate the rate limiter.
    /// The default value is defined for each robot type over
    /// [`RATE_LIMITING_ON_PER_DEFAULT`](`crate::robot::rate_limiting::RateLimiter::RATE_LIMITING_ON_PER_DEFAULT`).
    /// Enabling the rate limiter can distort your motion!
    /// * `cutoff_frequency` - Cutoff frequency for a first order low-pass filter applied on
    /// the user commanded signal.
    /// Set to [`MAX_CUTOFF_FREQUENCY`](`crate::robot::low_pass_filter::MAX_CUTOFF_FREQUENCY`) to disable.
    /// Default is 100 Hz
    /// # Errors
    /// * [`ControlException`](`crate::exception::FrankaException::ControlException`) if an error related to torque control or motion generation occurred.
    /// * [`NetworkException`](`crate::exception::FrankaException::NetworkException`) if the connection is lost, e.g. after a timeout.
    /// * [`RealTimeException`](`crate::exception::FrankaException::RealTimeException`) if realtime priority cannot be set for the current thread.
    /// # Panics
    /// * if joint velocity commands are NaN or infinity.
    fn control_joint_velocities<
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
    ) -> FrankaResult<()>;

    /// Starts a control loop for a Cartesian pose motion generator with a given controller mode.
    ///
    /// Sets realtime priority for the current thread.
    /// Cannot be executed while another control or motion generator loop is active.
    ///
    /// # Arguments
    /// * `motion_generator_callback` - Callback function for motion generation.
    /// See [here](#motion-generation-and-joint-level-torque-commands) for more details.
    /// * `controller_mode` - Controller to use to execute the motion. Default is joint impedance
    /// * `limit_rate` - Set to true to activate the rate limiter.
    /// The default value is defined for each robot type over
    /// [`RATE_LIMITING_ON_PER_DEFAULT`](`crate::robot::rate_limiting::RateLimiter::RATE_LIMITING_ON_PER_DEFAULT`).
    /// Enabling the rate limiter can distort your motion!
    /// * `cutoff_frequency` - Cutoff frequency for a first order low-pass filter applied on
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
    fn control_cartesian_pose<
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
    ) -> FrankaResult<()>;

    /// Starts a control loop for a Cartesian velocity motion generator with a given controller mode.
    ///
    /// Sets realtime priority for the current thread.
    /// Cannot be executed while another control or motion generator loop is active.
    ///
    /// # Arguments
    /// * `motion_generator_callback` - Callback function for motion generation.
    /// See [here](#motion-generation-and-joint-level-torque-commands) for more details.
    /// * `controller_mode` - Controller to use to execute the motion. Default is joint impedance
    /// * `limit_rate` - Set to true to activate the rate limiter.
    /// The default value is defined for each robot type over
    /// [`RATE_LIMITING_ON_PER_DEFAULT`](`crate::robot::rate_limiting::RateLimiter::RATE_LIMITING_ON_PER_DEFAULT`).
    /// Enabling the rate limiter can distort your motion!
    /// * `cutoff_frequency` - Cutoff frequency for a first order low-pass filter applied on
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
    fn control_cartesian_velocities<
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
    ) -> FrankaResult<()>;

    /// Starts a control loop for sending joint-level torque commands.
    ///
    /// Sets real-time priority for the current thread.
    /// Cannot be executed while another control or motion generator loop is active.
    ///
    /// # Arguments
    /// * `control_callback` - Callback function providing joint-level torque commands.
    /// See [here](#motion-generation-and-joint-level-torque-commands) for more details.
    /// * `limit_rate` - True if rate limiting should be activated. The default value is defined for each robot type over
    /// [`RATE_LIMITING_ON_PER_DEFAULT`](`crate::robot::rate_limiting::RateLimiter::RATE_LIMITING_ON_PER_DEFAULT`)
    /// Enabling the rate limiter can distort your motion!
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
    fn control_torques<
        T: FnMut(&RobotState, &Duration) -> Torques,
        L: Into<Option<bool>>,
        CF: Into<Option<f64>>,
    >(
        &mut self,
        control_callback: T,
        limit_rate: L,
        cutoff_frequency: CF,
    ) -> FrankaResult<()>;

    /// Starts a control loop for sending joint-level torque commands and joint positions.
    ///
    /// Sets realtime priority for the current thread.
    /// Cannot be executed while another control or motion generator loop is active.
    ///
    /// # Arguments
    /// * `control_callback` - Callback function providing joint-level torque commands.
    /// See [here](#motion-generation-and-joint-level-torque-commands) for more details.
    /// * `motion_generator_callback` - Callback function for motion generation.
    /// See [here](#motion-generation-and-joint-level-torque-commands) for more details.
    /// * `limit_rate` - Set to true to activate the rate limiter.
    /// The default value is defined for each robot type over
    /// [`RATE_LIMITING_ON_PER_DEFAULT`](`crate::robot::rate_limiting::RateLimiter::RATE_LIMITING_ON_PER_DEFAULT`).
    /// Enabling the rate limiter can distort your motion!
    /// * `cutoff_frequency` - Cutoff frequency for a first order low-pass filter applied on
    /// the user commanded signal.
    /// Set to [`MAX_CUTOFF_FREQUENCY`](`crate::robot::low_pass_filter::MAX_CUTOFF_FREQUENCY`) to disable.
    /// Default is 100 Hz
    /// # Errors
    /// * [`ControlException`](`crate::exception::FrankaException::ControlException`) if an error related to torque control or motion generation occurred.
    /// * [`NetworkException`](`crate::exception::FrankaException::NetworkException`) if the connection is lost, e.g. after a timeout.
    /// * [`RealTimeException`](`crate::exception::FrankaException::RealTimeException`) if realtime priority cannot be set for the current thread.
    /// # Panics
    /// * if joint-level torque or joint position commands are NaN or infinity.
    fn control_torques_and_joint_positions<
        F: FnMut(&RobotState, &Duration) -> JointPositions,
        T: FnMut(&RobotState, &Duration) -> Torques,
        L: Into<Option<bool>>,
        CF: Into<Option<f64>>,
    >(
        &mut self,
        control_callback: T,
        motion_generator_callback: F,
        limit_rate: L,
        cutoff_frequency: CF,
    ) -> FrankaResult<()>;

    /// Starts a control loop for sending joint-level torque commands and joint velocities.
    ///
    /// Sets realtime priority for the current thread.
    /// Cannot be executed while another control or motion generator loop is active.
    ///
    /// # Arguments
    /// * `control_callback` - Callback function providing joint-level torque commands.
    /// See [here](#motion-generation-and-joint-level-torque-commands) for more details.
    /// * `motion_generator_callback` - Callback function for motion generation.
    /// See [here](#motion-generation-and-joint-level-torque-commands) for more details.
    /// * `limit_rate` - Set to true to activate the rate limiter.
    /// The default value is defined for each robot type over
    /// [`RATE_LIMITING_ON_PER_DEFAULT`](`crate::robot::rate_limiting::RateLimiter::RATE_LIMITING_ON_PER_DEFAULT`).
    /// Enabling the rate limiter can distort your motion!
    /// * `cutoff_frequency` - Cutoff frequency for a first order low-pass filter applied on
    /// the user commanded signal.
    /// Set to [`MAX_CUTOFF_FREQUENCY`](`crate::robot::low_pass_filter::MAX_CUTOFF_FREQUENCY`) to disable.
    /// Default is 100 Hz
    /// # Errors
    /// * [`ControlException`](`crate::exception::FrankaException::ControlException`) if an error related to torque control or motion generation occurred.
    /// * [`NetworkException`](`crate::exception::FrankaException::NetworkException`) if the connection is lost, e.g. after a timeout.
    /// * [`RealTimeException`](`crate::exception::FrankaException::RealTimeException`) if realtime priority cannot be set for the current thread.
    /// # Panics
    /// * if joint-level torque or joint velocity commands are NaN or infinity.
    fn control_torques_and_joint_velocities<
        F: FnMut(&RobotState, &Duration) -> JointVelocities,
        T: FnMut(&RobotState, &Duration) -> Torques,
        L: Into<Option<bool>>,
        CF: Into<Option<f64>>,
    >(
        &mut self,
        control_callback: T,
        motion_generator_callback: F,
        limit_rate: L,
        cutoff_frequency: CF,
    ) -> FrankaResult<()>;

    /// Starts a control loop for sending joint-level torque commands and Cartesian poses.
    ///
    /// Sets realtime priority for the current thread.
    /// Cannot be executed while another control or motion generator loop is active.
    ///
    /// # Arguments
    /// * `control_callback` - Callback function providing joint-level torque commands.
    /// See [here](#motion-generation-and-joint-level-torque-commands) for more details.
    /// * `motion_generator_callback` - Callback function for motion generation.
    /// See [here](#motion-generation-and-joint-level-torque-commands) for more details.
    /// * `limit_rate` - Set to true to activate the rate limiter.
    /// The default value is defined for each robot type over
    /// [`RATE_LIMITING_ON_PER_DEFAULT`](`crate::robot::rate_limiting::RateLimiter::RATE_LIMITING_ON_PER_DEFAULT`).
    /// Enabling the rate limiter can distort your motion!
    /// * `cutoff_frequency` - Cutoff frequency for a first order low-pass filter applied on
    /// the user commanded signal.
    /// Set to [`MAX_CUTOFF_FREQUENCY`](`crate::robot::low_pass_filter::MAX_CUTOFF_FREQUENCY`) to disable.
    /// Default is 100 Hz
    /// # Errors
    /// * [`ControlException`](`crate::exception::FrankaException::ControlException`) if an error related to torque control or motion generation occurred.
    /// * [`NetworkException`](`crate::exception::FrankaException::NetworkException`) if the connection is lost, e.g. after a timeout.
    /// * [`RealTimeException`](`crate::exception::FrankaException::RealTimeException`) if realtime priority cannot be set for the current thread.
    /// # Panics
    /// * if joint-level torque or Cartesian pose command elements are NaN or infinity.
    fn control_torques_and_cartesian_pose<
        F: FnMut(&RobotState, &Duration) -> CartesianPose,
        T: FnMut(&RobotState, &Duration) -> Torques,
        L: Into<Option<bool>>,
        CF: Into<Option<f64>>,
    >(
        &mut self,
        control_callback: T,
        motion_generator_callback: F,
        limit_rate: L,
        cutoff_frequency: CF,
    ) -> FrankaResult<()>;

    /// Starts a control loop for sending joint-level torque commands and Cartesian velocities.
    ///
    /// Sets realtime priority for the current thread.
    /// Cannot be executed while another control or motion generator loop is active.
    ///
    /// # Arguments
    /// * `control_callback` - Callback function providing joint-level torque commands.
    /// See [here](#motion-generation-and-joint-level-torque-commands) for more details.
    /// * `motion_generator_callback` - Callback function for motion generation.
    /// See [here](#motion-generation-and-joint-level-torque-commands) for more details.
    /// * `limit_rate` - Set to true to activate the rate limiter.
    /// The default value is defined for each robot type over
    /// [`RATE_LIMITING_ON_PER_DEFAULT`](`crate::robot::rate_limiting::RateLimiter::RATE_LIMITING_ON_PER_DEFAULT`).
    /// Enabling the rate limiter can distort your motion!
    /// * `cutoff_frequency` - Cutoff frequency for a first order low-pass filter applied on
    /// the user commanded signal.
    /// Set to [`MAX_CUTOFF_FREQUENCY`](`crate::robot::low_pass_filter::MAX_CUTOFF_FREQUENCY`) to disable.
    /// Default is 100 Hz
    /// # Errors
    /// * [`ControlException`](`crate::exception::FrankaException::ControlException`) if an error related to torque control or motion generation occurred.
    /// * [`NetworkException`](`crate::exception::FrankaException::NetworkException`) if the connection is lost, e.g. after a timeout.
    /// * [`RealTimeException`](`crate::exception::FrankaException::RealTimeException`) if realtime priority cannot be set for the current thread.
    /// # Panics
    /// * if joint-level torque or Cartesian velocity command elements are NaN or infinity.
    fn control_torques_and_cartesian_velocities<
        F: FnMut(&RobotState, &Duration) -> CartesianVelocities,
        T: FnMut(&RobotState, &Duration) -> Torques,
        L: Into<Option<bool>>,
        CF: Into<Option<f64>>,
    >(
        &mut self,
        control_callback: T,
        motion_generator_callback: F,
        limit_rate: L,
        cutoff_frequency: CF,
    ) -> FrankaResult<()>;

    /// Loads the model library from the robot.
    /// # Arguments
    /// * `persistent` - If set to true the model will be stored at `/tmp/model.so`
    /// # Return
    /// Model instance.
    /// # Errors
    /// * [`ModelException`](`crate::exception::FrankaException::ModelException`) if the model library cannot be loaded.
    /// * [`NetworkException`](`crate::exception::FrankaException::NetworkException`) if the connection is lost, e.g. after a timeout.
    fn load_model(&mut self, persistent: bool) -> FrankaResult<RobotModel>;

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
    fn set_collision_behavior(
        &mut self,
        lower_torque_thresholds_acceleration: [f64; 7],
        upper_torque_thresholds_acceleration: [f64; 7],
        lower_torque_thresholds_nominal: [f64; 7],
        upper_torque_thresholds_nominal: [f64; 7],
        lower_force_thresholds_acceleration: [f64; 6],
        upper_force_thresholds_acceleration: [f64; 6],
        lower_force_thresholds_nominal: [f64; 6],
        upper_force_thresholds_nominal: [f64; 6],
    ) -> FrankaResult<()>;

    /// Sets a default collision behavior, joint impedance and Cartesian impedance.
    /// # Errors
    /// * [`CommandException`](`crate::exception::FrankaException::CommandException`) if the Control reports an error.
    /// * [`NetworkException`](`crate::exception::FrankaException::NetworkException`) if the connection is lost, e.g. after a timeout.
    fn set_default_behavior(&mut self) -> FrankaResult<()>;

    /// Sets the impedance for each joint in the internal controller.
    ///
    /// User-provided torques are not affected by this setting.
    /// # Arguments
    /// * `K_theta` - Joint impedance values ![K_{\theta}](https://latex.codecogs.com/png.latex?K_{\theta}).
    /// # Errors
    /// * [`CommandException`](`crate::exception::FrankaException::CommandException`) if the Control reports an error.
    /// * [`NetworkException`](`crate::exception::FrankaException::NetworkException`) if the connection is lost, e.g. after a timeout.
    #[allow(non_snake_case)]
    fn set_joint_impedance(&mut self, K_theta: [f64; 7]) -> FrankaResult<()>;

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
    fn set_cartesian_impedance(&mut self, K_x: [f64; 6]) -> FrankaResult<()>;

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
    fn set_load(
        &mut self,
        load_mass: f64,
        F_x_Cload: [f64; 3],
        load_inertia: [f64; 9],
    ) -> FrankaResult<()>;

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
    fn set_guiding_mode(&mut self, guiding_mode: [bool; 6], elbow: bool) -> FrankaResult<()>;

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
    fn set_K(&mut self, EE_T_K: [f64; 16]) -> FrankaResult<()>;

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
    fn set_EE(&mut self, NE_T_EE: [f64; 16]) -> FrankaResult<()>;

    ///Runs automatic error recovery on the robot.
    ///
    /// Automatic error recovery e.g. resets the robot after a collision occurred.
    /// # Errors
    /// * [`CommandException`](`crate::exception::FrankaException::CommandException`) if the Control reports an error.
    /// * [`NetworkException`](`crate::exception::FrankaException::NetworkException`) if the connection is lost, e.g. after a timeout.
    fn automatic_error_recovery(&mut self) -> FrankaResult<()>;

    /// Stops all currently running motions.
    ///
    /// If a control or motion generator loop is running in another thread, it will be preempted
    /// with a [`ControlException`](`crate::exception::FrankaException::ControlException`).
    /// # Errors
    /// * [`CommandException`](`crate::exception::FrankaException::CommandException`) if the Control reports an error.
    /// * [`NetworkException`](`crate::exception::FrankaException::NetworkException`) if the connection is lost, e.g. after a timeout.
    fn stop(&mut self) -> FrankaResult<()>;

    /// Returns the software version reported by the connected server.
    ///
    /// # Return
    /// Software version of the connected server.
    fn server_version(&self) -> u16;
}

impl<R: PrivateRobot> Robot for R
where
    RobotState: From<<R as RobotData>::State>,
    CartesianPose: crate::ConvertMotion<<R as RobotData>::State>,
    JointVelocities: crate::ConvertMotion<<R as RobotData>::State>,
    JointPositions: crate::ConvertMotion<<R as RobotData>::State>,
    CartesianVelocities: crate::ConvertMotion<<R as RobotData>::State>,
{
    fn read_once(&mut self) -> FrankaResult<RobotState> {
        match <Self as PrivateRobot>::get_rob_mut(self).read_once() {
            Ok(state) => Ok(state.into()),
            Err(e) => Err(e),
        }
    }

    fn read<F: FnMut(&RobotState) -> bool>(&mut self, mut read_callback: F) -> FrankaResult<()> {
        loop {
            let state = <R as PrivateRobot>::get_rob_mut(self).update(None, None)?;
            if !read_callback(&state.into()) {
                break;
            }
        }
        Ok(())
    }

    fn joint_motion(&mut self, speed_factor: f64, q_goal: &[f64; 7]) -> FrankaResult<()> {
        let mut motion_generator = MotionGenerator::new(speed_factor, q_goal);
        self.control_joint_positions(
            |state, time| motion_generator.generate_motion(state, time),
            Some(ControllerMode::JointImpedance),
            Some(true),
            Some(MAX_CUTOFF_FREQUENCY),
        )
    }

    fn control_joint_positions<
        F: FnMut(&RobotState, &Duration) -> JointPositions,
        CM: Into<Option<ControllerMode>>,
        L: Into<Option<bool>>,
        CF: Into<Option<f64>>,
    >(
        &mut self,
        mut motion_generator_callback: F,
        controller_mode: CM,
        limit_rate: L,
        cutoff_frequency: CF,
    ) -> FrankaResult<()> {
        let cb = |state: &<Self as RobotData>::State, duration: &Duration| {
            motion_generator_callback(&(state.clone().into()), duration)
        };
        <Self as PrivateRobot>::control_motion_intern(
            self,
            cb,
            controller_mode.into(),
            limit_rate.into(),
            cutoff_frequency.into(),
        )
    }

    fn control_joint_velocities<
        F: FnMut(&RobotState, &Duration) -> JointVelocities,
        CM: Into<Option<ControllerMode>>,
        L: Into<Option<bool>>,
        CF: Into<Option<f64>>,
    >(
        &mut self,
        mut motion_generator_callback: F,
        controller_mode: CM,
        limit_rate: L,
        cutoff_frequency: CF,
    ) -> FrankaResult<()> {
        let cb = |state: &<Self as RobotData>::State, duration: &Duration| {
            motion_generator_callback(&(state.clone().into()), duration) // todo make this without cloning
        };
        <Self as PrivateRobot>::control_motion_intern(
            self,
            cb,
            controller_mode.into(),
            limit_rate.into(),
            cutoff_frequency.into(),
        )
    }

    fn control_cartesian_pose<
        F: FnMut(&RobotState, &Duration) -> CartesianPose,
        CM: Into<Option<ControllerMode>>,
        L: Into<Option<bool>>,
        CF: Into<Option<f64>>,
    >(
        &mut self,
        mut motion_generator_callback: F,
        controller_mode: CM,
        limit_rate: L,
        cutoff_frequency: CF,
    ) -> FrankaResult<()> {
        let cb = |state: &<Self as RobotData>::State, duration: &Duration| {
            motion_generator_callback(&(state.clone().into()), duration)
        };
        <Self as PrivateRobot>::control_motion_intern(
            self,
            cb,
            controller_mode.into(),
            limit_rate.into(),
            cutoff_frequency.into(),
        )
    }

    fn control_cartesian_velocities<
        F: FnMut(&RobotState, &Duration) -> CartesianVelocities,
        CM: Into<Option<ControllerMode>>,
        L: Into<Option<bool>>,
        CF: Into<Option<f64>>,
    >(
        &mut self,
        mut motion_generator_callback: F,
        controller_mode: CM,
        limit_rate: L,
        cutoff_frequency: CF,
    ) -> FrankaResult<()> {
        let cb = |state: &<Self as RobotData>::State, duration: &Duration| {
            motion_generator_callback(&(state.clone().into()), duration)
        };
        self.control_motion_intern(
            cb,
            controller_mode.into(),
            limit_rate.into(),
            cutoff_frequency.into(),
        )
    }

    fn control_torques<
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
            |_state: &<Self as RobotData>::State, _time_step: &Duration| {
                JointVelocities::new([0.; 7])
            };
        let mut cb = |state: &<Self as RobotData>::State, duration: &Duration| {
            control_callback(&(state.clone().into()), duration)
        };
        self.control_torques_intern(
            &motion_generator_callback,
            &mut cb,
            limit_rate.into(),
            cutoff_frequency.into(),
        )
    }

    fn control_torques_and_joint_positions<
        F: FnMut(&RobotState, &Duration) -> JointPositions,
        T: FnMut(&RobotState, &Duration) -> Torques,
        L: Into<Option<bool>>,
        CF: Into<Option<f64>>,
    >(
        &mut self,
        mut control_callback: T,
        mut motion_generator_callback: F,
        limit_rate: L,
        cutoff_frequency: CF,
    ) -> FrankaResult<()> {
        let motion_generator_cb = |state: &<Self as RobotData>::State, duration: &Duration| {
            motion_generator_callback(&(state.clone().into()), duration)
        };
        let mut control_cb = |state: &<Self as RobotData>::State, duration: &Duration| {
            control_callback(&(state.clone().into()), duration)
        };
        self.control_torques_intern(
            motion_generator_cb,
            &mut control_cb,
            limit_rate.into(),
            cutoff_frequency.into(),
        )
    }

    fn control_torques_and_joint_velocities<
        F: FnMut(&RobotState, &Duration) -> JointVelocities,
        T: FnMut(&RobotState, &Duration) -> Torques,
        L: Into<Option<bool>>,
        CF: Into<Option<f64>>,
    >(
        &mut self,
        mut control_callback: T,
        mut motion_generator_callback: F,
        limit_rate: L,
        cutoff_frequency: CF,
    ) -> FrankaResult<()> {
        let motion_generator_cb = |state: &<Self as RobotData>::State, duration: &Duration| {
            motion_generator_callback(&(state.clone().into()), duration)
        };
        let mut control_cb = |state: &<Self as RobotData>::State, duration: &Duration| {
            control_callback(&(state.clone().into()), duration)
        };
        self.control_torques_intern(
            motion_generator_cb,
            &mut control_cb,
            limit_rate.into(),
            cutoff_frequency.into(),
        )
    }

    fn control_torques_and_cartesian_pose<
        F: FnMut(&RobotState, &Duration) -> CartesianPose,
        T: FnMut(&RobotState, &Duration) -> Torques,
        L: Into<Option<bool>>,
        CF: Into<Option<f64>>,
    >(
        &mut self,
        mut control_callback: T,
        mut motion_generator_callback: F,
        limit_rate: L,
        cutoff_frequency: CF,
    ) -> FrankaResult<()> {
        let motion_generator_cb = |state: &<Self as RobotData>::State, duration: &Duration| {
            motion_generator_callback(&(state.clone().into()), duration)
        };
        let mut control_cb = |state: &<Self as RobotData>::State, duration: &Duration| {
            control_callback(&(state.clone().into()), duration)
        };
        self.control_torques_intern(
            motion_generator_cb,
            &mut control_cb,
            limit_rate.into(),
            cutoff_frequency.into(),
        )
    }

    fn control_torques_and_cartesian_velocities<
        F: FnMut(&RobotState, &Duration) -> CartesianVelocities,
        T: FnMut(&RobotState, &Duration) -> Torques,
        L: Into<Option<bool>>,
        CF: Into<Option<f64>>,
    >(
        &mut self,
        mut control_callback: T,
        mut motion_generator_callback: F,
        limit_rate: L,
        cutoff_frequency: CF,
    ) -> FrankaResult<()> {
        let motion_generator_cb = |state: &<Self as RobotData>::State, duration: &Duration| {
            motion_generator_callback(&(state.clone().into()), duration)
        };
        let mut control_cb = |state: &<Self as RobotData>::State, duration: &Duration| {
            control_callback(&(state.clone().into()), duration)
        };
        self.control_torques_intern(
            motion_generator_cb,
            &mut control_cb,
            limit_rate.into(),
            cutoff_frequency.into(),
        )
    }

    fn load_model(&mut self, persistent: bool) -> FrankaResult<RobotModel> {
        <Self as PrivateRobot>::get_rob_mut(self)
            .load_model(persistent)
            .map(|model| model.into())
    }

    #[allow(clippy::too_many_arguments)]
    fn set_collision_behavior(
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
        let command = <Self as PrivateRobotData>::create_set_collision_behavior_request(
            &mut <Self as PrivateRobot>::get_net(self).command_id,
            SetCollisionBehaviorRequest::new(
                lower_torque_thresholds_acceleration,
                upper_torque_thresholds_acceleration,
                lower_torque_thresholds_nominal,
                upper_torque_thresholds_nominal,
                lower_force_thresholds_acceleration,
                upper_force_thresholds_acceleration,
                lower_force_thresholds_nominal,
                upper_force_thresholds_nominal,
            ),
        );
        let command_id: u32 = <Self as PrivateRobot>::get_net(self).tcp_send_request(command);
        let status = <Self as PrivateRobot>::get_net(self).tcp_blocking_receive_status(command_id);
        <Self as PrivateRobotData>::handle_getter_setter_status(status)
    }

    fn set_default_behavior(&mut self) -> FrankaResult<()> {
        self.set_collision_behavior(
            [20.; 7], [20.; 7], [10.; 7], [10.; 7], [20.; 6], [20.; 6], [10.; 6], [10.; 6],
        )?;
        self.set_joint_impedance([3000., 3000., 3000., 2500., 2500., 2000., 2000.])?;
        self.set_cartesian_impedance([3000., 3000., 3000., 300., 300., 300.])?;
        Ok(())
    }

    #[allow(non_snake_case)]
    fn set_joint_impedance(&mut self, K_theta: [f64; 7]) -> FrankaResult<()> {
        let command = <Self as PrivateRobotData>::create_set_joint_impedance_request(
            &mut <Self as PrivateRobot>::get_net(self).command_id,
            SetJointImpedanceRequest::new(K_theta),
        );
        let command_id: u32 = <Self as PrivateRobot>::get_net(self).tcp_send_request(command);
        let status = <Self as PrivateRobot>::get_net(self).tcp_blocking_receive_status(command_id);
        <Self as PrivateRobotData>::handle_getter_setter_status(status)
    }

    #[allow(non_snake_case)]
    fn set_cartesian_impedance(&mut self, K_x: [f64; 6]) -> FrankaResult<()> {
        let command = <Self as PrivateRobotData>::create_set_cartesian_impedance_request(
            &mut <Self as PrivateRobot>::get_net(self).command_id,
            SetCartesianImpedanceRequest::new(K_x),
        );
        let command_id: u32 = <Self as PrivateRobot>::get_net(self).tcp_send_request(command);
        let status = <Self as PrivateRobot>::get_net(self).tcp_blocking_receive_status(command_id);
        <Self as PrivateRobotData>::handle_getter_setter_status(status)
    }

    #[allow(non_snake_case)]
    fn set_load(
        &mut self,
        load_mass: f64,
        F_x_Cload: [f64; 3],
        load_inertia: [f64; 9],
    ) -> FrankaResult<()> {
        let command = <Self as PrivateRobotData>::create_set_load_request(
            &mut <Self as PrivateRobot>::get_net(self).command_id,
            SetLoadRequest::new(load_mass, F_x_Cload, load_inertia),
        );
        let command_id: u32 = <Self as PrivateRobot>::get_net(self).tcp_send_request(command);
        let status = <Self as PrivateRobot>::get_net(self).tcp_blocking_receive_status(command_id);
        <Self as PrivateRobotData>::handle_getter_setter_status(status)
    }

    fn set_guiding_mode(&mut self, guiding_mode: [bool; 6], elbow: bool) -> FrankaResult<()> {
        let command = <Self as PrivateRobotData>::create_set_guiding_mode_request(
            &mut <Self as PrivateRobot>::get_net(self).command_id,
            SetGuidingModeRequest::new(guiding_mode, elbow),
        );
        let command_id: u32 = <Self as PrivateRobot>::get_net(self).tcp_send_request(command);
        let status = <Self as PrivateRobot>::get_net(self).tcp_blocking_receive_status(command_id);
        <Self as PrivateRobotData>::handle_getter_setter_status(status)
    }

    #[allow(non_snake_case)]
    fn set_K(&mut self, EE_T_K: [f64; 16]) -> FrankaResult<()> {
        let command = <Self as PrivateRobotData>::create_set_ee_to_k_request(
            &mut <Self as PrivateRobot>::get_net(self).command_id,
            SetEeToKRequest::new(EE_T_K),
        );
        let command_id: u32 = <Self as PrivateRobot>::get_net(self).tcp_send_request(command);
        let status = <Self as PrivateRobot>::get_net(self).tcp_blocking_receive_status(command_id);
        <Self as PrivateRobotData>::handle_getter_setter_status(status)
    }

    #[allow(non_snake_case)]
    fn set_EE(&mut self, NE_T_EE: [f64; 16]) -> FrankaResult<()> {
        let command = <Self as PrivateRobotData>::create_set_ne_to_ee_request(
            &mut <Self as PrivateRobot>::get_net(self).command_id,
            SetNeToEeRequest::new(NE_T_EE),
        );
        let command_id: u32 = <Self as PrivateRobot>::get_net(self).tcp_send_request(command);
        let status = <Self as PrivateRobot>::get_net(self).tcp_blocking_receive_status(command_id);
        <Self as PrivateRobotData>::handle_getter_setter_status(status)
    }

    fn automatic_error_recovery(&mut self) -> FrankaResult<()> {
        let command = <Self as PrivateRobotData>::create_automatic_error_recovery_request(
            &mut <Self as PrivateRobot>::get_net(self).command_id,
        );
        let command_id: u32 = <Self as PrivateRobot>::get_net(self).tcp_send_request(command);
        let status = <Self as PrivateRobot>::get_net(self).tcp_blocking_receive_status(command_id);
        <Self as PrivateRobotData>::handle_automatic_error_recovery_status(status)
    }

    fn stop(&mut self) -> FrankaResult<()> {
        let command = <Self as PrivateRobotData>::create_stop_request(
            &mut <Self as PrivateRobot>::get_net(self).command_id,
        );
        let command_id: u32 = <Self as PrivateRobot>::get_net(self).tcp_send_request(command);
        let status: StopMoveStatusPanda =
            <Self as PrivateRobot>::get_net(self).tcp_blocking_receive_status(command_id);
        match status {
            StopMoveStatusPanda::Success => Ok(()),
            StopMoveStatusPanda::CommandNotPossibleRejected | StopMoveStatusPanda::Aborted => {
                Err(create_command_exception(
                    "libfranka-rs: command rejected: command not possible in current mode",
                ))
            }
            StopMoveStatusPanda::EmergencyAborted => Err(create_command_exception(
                "libfranka-rs: command aborted: User Stop pressed!",
            )),
            StopMoveStatusPanda::ReflexAborted => Err(create_command_exception(
                "libfranka-rs: command aborted: motion aborted by reflex!",
            )),
        }
    }

    fn server_version(&self) -> u16 {
        <Self as PrivateRobot>::get_rob(self).server_version()
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
    use crate::robot::service_types::{
        ConnectRequestWithPandaHeader, ConnectResponsePanda, ConnectStatus, Fr3CommandEnum,
        Fr3CommandHeader, GetterSetterStatusPanda, MoveControllerMode, MoveDeviation,
        MoveMotionGeneratorMode, MoveRequest, MoveRequestWithPandaHeader, MoveStatusPanda,
        PandaCommandEnum, PandaCommandHeader, SetCollisionBehaviorRequest,
        SetCollisionBehaviorRequestWithFr3Header, SetCollisionBehaviorRequestWithPandaHeader,
        SetterResponseFr3, COMMAND_PORT, FR3_VERSION,
    };
    use crate::robot::types::PandaStateIntern;
    use crate::{Fr3, Robot};
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
                    let mut buffer = vec![0_u8; 3000];
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
            let mut bytes = vec![0_u8; 100];
            (tcp_socket.receive_bytes)(&mut bytes);
            let response = reaction.process_received_bytes(&mut bytes);
            (tcp_socket.send_bytes)(&response);
        }
        fn receive_robot_connect_request<F: Fn(&Vec<u8>), G: Fn(&mut Vec<u8>)>(
            &self,
            tcp_socket: &mut Socket<F, G>,
        ) -> ConnectRequestWithPandaHeader {
            let mut bytes = vec![0_u8; 100];
            (tcp_socket.receive_bytes)(&mut bytes);
            let request: ConnectRequestWithPandaHeader = deserialize(bytes.as_slice()).unwrap();
            request
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
        let requests_server = requests;
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
                        deserialize(bytes).unwrap();
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
        let requests_server = requests;
        let thread = std::thread::spawn(|| {
            let mut robot_server = RobotMockServer::new(FR3_VERSION);
            let mut mock = MockServerReaction::default();
            let num_requests = requests_server.len();
            let mut counter = 0;
            mock.expect_process_received_bytes()
                .returning(move |bytes: &mut Vec<u8>| -> Vec<u8> {
                    let expected_request = requests_server.get(counter).unwrap();
                    let serialized_expected_request = serialize(expected_request).unwrap();
                    let req: MoveRequestWithPandaHeader = deserialize(bytes).unwrap();
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
            mock.expect_number_of_reactions().return_const(0_usize);
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
            mock.expect_number_of_reactions().return_const(0_usize);
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
            mock.expect_number_of_reactions().return_const(0_usize);
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
