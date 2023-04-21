use crate::exception::create_command_exception;
use crate::robot::robot::PrivateRobot;
use crate::robot::robot::Robot;
use crate::robot::robot_control::RobotControl;
use crate::robot::robot_data::RobotData;
use crate::robot::robot_impl::RobotImplementation;
use crate::robot::service_types::{
    SetCartesianImpedanceRequest, SetCollisionBehaviorRequest, SetEeToKRequest,
    SetGuidingModeRequest, SetJointImpedanceRequest, SetLoadRequest, SetNeToEeRequest,
    StopMoveStatusPanda,
};
use crate::{
    CartesianPose, CartesianVelocities, ControllerMode, FrankaResult, JointPositions,
    JointVelocities, MotionGenerator, RobotModel, RobotState, Torques, MAX_CUTOFF_FREQUENCY,
};
use std::time::Duration;

pub trait RobotWrapper {
    type Model: RobotModel;

    /// Starts a loop for reading the current robot state.
    ///
    /// Cannot be executed while a control or motion generator loop is running.
    ///
    /// This minimal example will print the robot state 100 times:
    /// ```no_run
    /// use franka::{Fr3, RobotState, RobotWrapper, FrankaResult};
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

    /// Executes a joint pose motion to a goal position. Adapted from:
    /// Wisama Khalil and Etienne Dombre. 2002. Modeling, Identification and Control of Robots
    /// (Kogan Page Science Paper edition).
    /// # Arguments
    /// * `speed_factor` - General speed factor in range [0, 1].
    /// * `q_goal` - Target joint positions.
    fn joint_motion(&mut self, speed_factor: f64, q_goal: &[f64; 7]) -> FrankaResult<()>;

    /// Waits for a robot state update and returns it.
    ///
    /// # Return
    /// Current robot state.
    /// # Error
    /// * [`NetworkException`](`crate::exception::FrankaException::NetworkException`) if the connection is lost, e.g. after a timeout.
    ///
    /// See [`Robot::read`](`Self::read`) for a way to repeatedly receive the robot state.
    fn read_once(&mut self) -> FrankaResult<RobotState>;

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
    fn load_model(&mut self, persistent: bool) -> FrankaResult<Self::Model>;

    /// Returns the software version reported by the connected server.
    ///
    /// # Return
    /// Software version of the connected server.
    fn server_version(&self) -> u16;
}

impl<R: PrivateRobot> RobotWrapper for R
where
    RobotState: From<<<R as Robot>::Data as RobotData>::State>,
    CartesianPose: crate::ConvertMotion<<<R as Robot>::Data as RobotData>::State>,
    JointVelocities: crate::ConvertMotion<<<R as Robot>::Data as RobotData>::State>,
    JointPositions: crate::ConvertMotion<<<R as Robot>::Data as RobotData>::State>,
    CartesianVelocities: crate::ConvertMotion<<<R as Robot>::Data as RobotData>::State>,
{
    type Model = <<R as Robot>::Data as RobotData>::Model;

    fn read<F: FnMut(&RobotState) -> bool>(&mut self, mut read_callback: F) -> FrankaResult<()> {
        loop {
            let state = <R as PrivateRobot>::get_rob_mut(self).update(None, None)?;
            if !read_callback(&state.into()) {
                break;
            }
        }
        Ok(())
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
        let cb = |state: &<<Self as Robot>::Data as RobotData>::State, duration: &Duration| {
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

    fn joint_motion(&mut self, speed_factor: f64, q_goal: &[f64; 7]) -> FrankaResult<()> {
        let mut motion_generator = MotionGenerator::new(speed_factor, q_goal);
        self.control_joint_positions(
            |state, time| motion_generator.generate_motion(state, time),
            Some(ControllerMode::JointImpedance),
            Some(true),
            Some(MAX_CUTOFF_FREQUENCY),
        )
    }

    fn read_once(&mut self) -> FrankaResult<RobotState> {
        match <Self as PrivateRobot>::get_rob_mut(self).read_once() {
            Ok(state) => Ok(state.into()),
            Err(e) => Err(e),
        }
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
        let command = <Self as Robot>::Data::create_set_collision_behavior_request(
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
        <Self as Robot>::Data::handle_getter_setter_status(status)
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
        let command = <Self as Robot>::Data::create_set_joint_impedance_request(
            &mut <Self as PrivateRobot>::get_net(self).command_id,
            SetJointImpedanceRequest::new(K_theta),
        );
        let command_id: u32 = <Self as PrivateRobot>::get_net(self).tcp_send_request(command);
        let status = <Self as PrivateRobot>::get_net(self).tcp_blocking_receive_status(command_id);
        <Self as Robot>::Data::handle_getter_setter_status(status)
    }

    #[allow(non_snake_case)]
    fn set_cartesian_impedance(&mut self, K_x: [f64; 6]) -> FrankaResult<()> {
        let command = <Self as Robot>::Data::create_set_cartesian_impedance_request(
            &mut <Self as PrivateRobot>::get_net(self).command_id,
            SetCartesianImpedanceRequest::new(K_x),
        );
        let command_id: u32 = <Self as PrivateRobot>::get_net(self).tcp_send_request(command);
        let status = <Self as PrivateRobot>::get_net(self).tcp_blocking_receive_status(command_id);
        <Self as Robot>::Data::handle_getter_setter_status(status)
    }

    #[allow(non_snake_case)]
    fn set_load(
        &mut self,
        load_mass: f64,
        F_x_Cload: [f64; 3],
        load_inertia: [f64; 9],
    ) -> FrankaResult<()> {
        let command = <Self as Robot>::Data::create_set_load_request(
            &mut <Self as PrivateRobot>::get_net(self).command_id,
            SetLoadRequest::new(load_mass, F_x_Cload, load_inertia),
        );
        let command_id: u32 = <Self as PrivateRobot>::get_net(self).tcp_send_request(command);
        let status = <Self as PrivateRobot>::get_net(self).tcp_blocking_receive_status(command_id);
        <Self as Robot>::Data::handle_getter_setter_status(status)
    }

    fn set_guiding_mode(&mut self, guiding_mode: [bool; 6], elbow: bool) -> FrankaResult<()> {
        let command = <Self as Robot>::Data::create_set_guiding_mode_request(
            &mut <Self as PrivateRobot>::get_net(self).command_id,
            SetGuidingModeRequest::new(guiding_mode, elbow),
        );
        let command_id: u32 = <Self as PrivateRobot>::get_net(self).tcp_send_request(command);
        let status = <Self as PrivateRobot>::get_net(self).tcp_blocking_receive_status(command_id);
        <Self as Robot>::Data::handle_getter_setter_status(status)
    }

    #[allow(non_snake_case)]
    fn set_K(&mut self, EE_T_K: [f64; 16]) -> FrankaResult<()> {
        let command = <Self as Robot>::Data::create_set_ee_to_k_request(
            &mut <Self as PrivateRobot>::get_net(self).command_id,
            SetEeToKRequest::new(EE_T_K),
        );
        let command_id: u32 = <Self as PrivateRobot>::get_net(self).tcp_send_request(command);
        let status = <Self as PrivateRobot>::get_net(self).tcp_blocking_receive_status(command_id);
        <Self as Robot>::Data::handle_getter_setter_status(status)
    }

    #[allow(non_snake_case)]
    fn set_EE(&mut self, NE_T_EE: [f64; 16]) -> FrankaResult<()> {
        let command = <Self as Robot>::Data::create_set_ne_to_ee_request(
            &mut <Self as PrivateRobot>::get_net(self).command_id,
            SetNeToEeRequest::new(NE_T_EE),
        );
        let command_id: u32 = <Self as PrivateRobot>::get_net(self).tcp_send_request(command);
        let status = <Self as PrivateRobot>::get_net(self).tcp_blocking_receive_status(command_id);
        <Self as Robot>::Data::handle_getter_setter_status(status)
    }

    fn automatic_error_recovery(&mut self) -> FrankaResult<()> {
        let command = <Self as Robot>::Data::create_automatic_error_recovery_request(
            &mut <Self as PrivateRobot>::get_net(self).command_id,
        );
        let command_id: u32 = <Self as PrivateRobot>::get_net(self).tcp_send_request(command);
        let status = <Self as PrivateRobot>::get_net(self).tcp_blocking_receive_status(command_id);
        <Self as Robot>::Data::handle_automatic_error_recovery_status(status)
    }

    fn stop(&mut self) -> FrankaResult<()> {
        let command = <Self as Robot>::Data::create_stop_request(
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
        let cb = |state: &<<Self as Robot>::Data as RobotData>::State, duration: &Duration| {
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
        let cb = |state: &<<Self as Robot>::Data as RobotData>::State, duration: &Duration| {
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
        let cb = |state: &<<Self as Robot>::Data as RobotData>::State, duration: &Duration| {
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
            |_state: &<<Self as Robot>::Data as RobotData>::State, _time_step: &Duration| {
                JointVelocities::new([0.; 7])
            };
        let mut cb = |state: &<<Self as Robot>::Data as RobotData>::State, duration: &Duration| {
            control_callback(&(state.clone().into()), duration)
        };
        self.control_torques_intern(
            &motion_generator_callback,
            &mut cb,
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
        let motion_generator_cb = |state: &<<Self as Robot>::Data as RobotData>::State,
                                   duration: &Duration| {
            motion_generator_callback(&(state.clone().into()), duration)
        };
        let mut control_cb = |state: &<<Self as Robot>::Data as RobotData>::State,
                              duration: &Duration| {
            control_callback(&(state.clone().into()), duration)
        };
        self.control_torques_intern(
            motion_generator_cb,
            &mut control_cb,
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
        let motion_generator_cb = |state: &<<Self as Robot>::Data as RobotData>::State,
                                   duration: &Duration| {
            motion_generator_callback(&(state.clone().into()), duration)
        };
        let mut control_cb = |state: &<<Self as Robot>::Data as RobotData>::State,
                              duration: &Duration| {
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
        let motion_generator_cb = |state: &<<Self as Robot>::Data as RobotData>::State,
                                   duration: &Duration| {
            motion_generator_callback(&(state.clone().into()), duration)
        };
        let mut control_cb = |state: &<<Self as Robot>::Data as RobotData>::State,
                              duration: &Duration| {
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
        let motion_generator_cb = |state: &<<Self as Robot>::Data as RobotData>::State,
                                   duration: &Duration| {
            motion_generator_callback(&(state.clone().into()), duration)
        };
        let mut control_cb = |state: &<<Self as Robot>::Data as RobotData>::State,
                              duration: &Duration| {
            control_callback(&(state.clone().into()), duration)
        };
        self.control_torques_intern(
            motion_generator_cb,
            &mut control_cb,
            limit_rate.into(),
            cutoff_frequency.into(),
        )
    }

    fn load_model(&mut self, persistent: bool) -> FrankaResult<Self::Model> {
        <Self as PrivateRobot>::get_rob_mut(self).load_model(persistent)
    }

    fn server_version(&self) -> u16 {
        <Self as PrivateRobot>::get_rob(self).server_version()
    }
}
