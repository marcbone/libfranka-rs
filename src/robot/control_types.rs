// Copyright (c) 2021 Marco Boneberger
// Licensed under the EUPL-1.2-or-later

//! Contains helper types for returning motion generation and joint-level torque commands.

use serde::Deserialize;
use serde::Serialize;

use crate::robot::control_tools::is_homogeneous_transformation;
use crate::robot::low_pass_filter::{
    cartesian_low_pass_filter, low_pass_filter, MAX_CUTOFF_FREQUENCY,
};
use crate::robot::motion_generator_traits::MotionGeneratorTrait;
use crate::robot::rate_limiting::{
    limit_rate_cartesian_pose, limit_rate_cartesian_velocity, limit_rate_joint_positions,
    limit_rate_joint_velocities, limit_rate_position, DELTA_T, MAX_ELBOW_ACCELERATION,
    MAX_ELBOW_JERK, MAX_ELBOW_VELOCITY, MAX_JOINT_ACCELERATION, MAX_JOINT_JERK, MAX_JOINT_VELOCITY,
    MAX_ROTATIONAL_ACCELERATION, MAX_ROTATIONAL_JERK, MAX_ROTATIONAL_VELOCITY,
    MAX_TRANSLATIONAL_ACCELERATION, MAX_TRANSLATIONAL_JERK, MAX_TRANSLATIONAL_VELOCITY,
};
use crate::robot::robot_state::{FR3State, PandaState, RobotState};
use crate::robot::service_types::MoveMotionGeneratorMode;
use crate::robot::types::MotionGeneratorCommand;
use crate::utils::Vector7;
use nalgebra::{Isometry3, Vector6};

/// Available controller modes for a [`Robot`](`crate::Robot`)
pub enum ControllerMode {
    JointImpedance,
    CartesianImpedance,
}

/// Used to decide whether to enforce realtime mode for a control loop thread.
/// see [`Robot`](`crate::Robot`)
#[derive(Copy, Clone, PartialEq)]
pub enum RealtimeConfig {
    Enforce,
    Ignore,
}

/// Helper type for control and motion generation loops.
///
/// Used to determine whether to terminate a loop after the control callback has returned.
pub trait ConvertMotion<State3: RobotState> {

    /// converts the motion type to a MotionGeneratorCommand and applies rate limiting and filtering
    fn convert_motion(
        &self,
        robot_state: &State3,
        command: &mut MotionGeneratorCommand,
        cutoff_frequency: f64,
        limit_rate: bool,
    );
}

pub trait Finishable {
    /// Determines whether to finish a currently running motion.
    fn is_finished(&self) -> bool;
    /// Sets the attribute which decide if the currently running motion should be finished
    fn set_motion_finished(&mut self, finished: bool);
    fn motion_finished(self) -> Self;
}

// /// A trait for a Finshable control type to finish the motion
// pub trait MotionFinished : Finishable<T> {
//     /// Helper method to indicate that a motion should stop after processing the given command.
//     fn motion_finished(self) -> Self;
// }
//
// // impl<T: Finishable<PandaState> + Copy> MotionFinished for T {
// //     type State4 = PandaState ;
// //
// //     fn motion_finished(mut self) -> Self {
// //         self.set_motion_finished(true);
// //         self
// //     }
// // }
//
// impl<T: Finishable<PandaState> + Copy> MotionFinished for T {
//
//     fn motion_finished(mut self) -> Self {
//         self.set_motion_finished(true);
//         self
//     }
// }
//
// impl<T: Finishable<FR3State> + Copy> MotionFinished for T {
//
//     fn motion_finished(mut self) -> Self {
//         self.set_motion_finished(true);
//         self
//     }
// }

// impl<T: Finishable<X> + Copy,X> MotionFinished for T {
//     type State4 = FR3State ;
//
//     fn motion_finished(mut self) -> Self {
//         self.set_motion_finished(true);
//         self
//     }
// }

/// Stores joint-level torque commands without gravity and friction.
#[derive(Serialize, Deserialize, Debug, Copy, Clone)]
#[allow(non_snake_case)]
pub struct Torques {
    motion_finished: bool,
    /// Desired torques in \[Nm\].
    pub tau_J: [f64; 7],
}

impl From<Vector7> for Torques {
    fn from(vector: Vector7) -> Self {
        Torques::new(vector.into())
    }
}

impl Torques {
    /// Creates a new Torques instance
    /// # Arguments
    /// * `torques` - Desired joint-level torques without gravity and friction in \[Nm\].
    pub fn new(torques: [f64; 7]) -> Self {
        Torques {
            tau_J: torques,
            motion_finished: false,
        }
    }
}

impl Finishable for Torques{
    fn is_finished(&self) -> bool {
        self.motion_finished
    }
    fn set_motion_finished(&mut self, finished: bool) {
        self.motion_finished = finished;
    }
    fn motion_finished(mut self) -> Self {
        self.set_motion_finished(true);
        self
    }
}

impl<Statee:RobotState> ConvertMotion<Statee> for Torques {


    #[allow(unused_variables)]
    //todo pull  convert motion out of the Finishable trait
    fn convert_motion(
        &self,
        robot_state: &Statee,
        command: &mut MotionGeneratorCommand,
        cutoff_frequency: f64,
        limit_rate: bool,
    ) {
        unimplemented!()
    }
}
// impl Finishable<FR3State> for Torques {
//
//     fn is_finished(&self) -> bool {
//         self.motion_finished
//     }
//     fn set_motion_finished(&mut self, finished: bool) {
//         self.motion_finished = finished;
//     }
//     #[allow(unused_variables)]
//     //todo pull  convert motion out of the Finishable trait
//     fn convert_motion(
//         &self,
//         robot_state: &Self::State3,
//         command: &mut MotionGeneratorCommand,
//         cutoff_frequency: f64,
//         limit_rate: bool,
//     ) {
//         unimplemented!()
//     }
// }

/// Stores values for joint position motion generation.
#[derive(Serialize, Deserialize, Debug, Copy, Clone)]
#[allow(non_snake_case)]
pub struct JointPositions {
    motion_finished: bool,
    /// Desired joint angles in \[rad\].
    pub q: [f64; 7],
}

impl From<Vector7> for JointPositions {
    fn from(vector: Vector7) -> Self {
        JointPositions::new(vector.into())
    }
}

impl JointPositions {
    /// Creates a new JointPositions instance.
    /// # Arguments
    /// * `joint_positions` - Desired joint angles in \[rad\].
    pub fn new(joint_positions: [f64; 7]) -> Self {
        JointPositions {
            q: joint_positions,
            motion_finished: false,
        }
    }
}
impl Finishable for JointPositions{
    fn is_finished(&self) -> bool {
        self.motion_finished
    }
    fn set_motion_finished(&mut self, finished: bool) {
        self.motion_finished = finished;
    }
    fn motion_finished(mut self) -> Self {
        self.set_motion_finished(true);
        self
    }}
impl ConvertMotion<PandaState> for JointPositions {

    fn convert_motion(
        &self,
        robot_state: &PandaState,
        command: &mut MotionGeneratorCommand,
        cutoff_frequency: f64,
        limit_rate: bool,
    ) {
        command.q_c = self.q;
        if cutoff_frequency < MAX_CUTOFF_FREQUENCY {
            for i in 0..7 {
                command.q_c[i] = low_pass_filter(
                    DELTA_T,
                    command.q_c[i],
                    robot_state.q_d[i],
                    cutoff_frequency,
                );
            }
        }
        if limit_rate {
            command.q_c = limit_rate_joint_positions(
                &MAX_JOINT_VELOCITY,
                &MAX_JOINT_ACCELERATION,
                &MAX_JOINT_JERK,
                &command.q_c,
                &robot_state.q_d,
                &robot_state.dq_d,
                &robot_state.ddq_d,
            );
        }
        command.q_c.iter().for_each(|x| assert!(x.is_finite()));
    }
}
impl ConvertMotion<FR3State> for JointPositions {
    fn convert_motion(
        &self,
        robot_state: &FR3State,
        command: &mut MotionGeneratorCommand,
        cutoff_frequency: f64,
        limit_rate: bool,
    ) {
        command.q_c = self.q;
        if cutoff_frequency < MAX_CUTOFF_FREQUENCY {
            for i in 0..7 {
                command.q_c[i] = low_pass_filter(
                    DELTA_T,
                    command.q_c[i],
                    robot_state.q_d[i],
                    cutoff_frequency,
                );
            }
        }
        if limit_rate {
            command.q_c = limit_rate_joint_positions(
                &MAX_JOINT_VELOCITY,
                &MAX_JOINT_ACCELERATION,
                &MAX_JOINT_JERK,
                &command.q_c,
                &robot_state.q_d,
                &robot_state.dq_d,
                &robot_state.ddq_d,
            );
        }
        command.q_c.iter().for_each(|x| assert!(x.is_finite()));
    }
}

impl MotionGeneratorTrait for JointPositions {
    fn get_motion_generator_mode() -> MoveMotionGeneratorMode {
        MoveMotionGeneratorMode::JointPosition
    }
}

/// Stores values for joint velocity motion generation.
#[derive(Serialize, Deserialize, Debug, Copy, Clone)]
#[allow(non_snake_case)]
pub struct JointVelocities {
    motion_finished: bool,
    /// Desired joint velocities in \[rad/s\].
    pub dq: [f64; 7],
}

impl From<Vector7> for JointVelocities {
    fn from(vector: Vector7) -> Self {
        JointVelocities::new(vector.into())
    }
}

impl JointVelocities {
    /// Creates a new JointVelocities instance.
    /// # Arguments
    /// * `joint_velocities` - Desired joint velocities in \[rad/s\].
    pub fn new(joint_velocities: [f64; 7]) -> Self {
        JointVelocities {
            dq: joint_velocities,
            motion_finished: false,
        }
    }
}
impl Finishable for JointVelocities{
    fn is_finished(&self) -> bool {
        self.motion_finished
    }
    fn set_motion_finished(&mut self, finished: bool) {
        self.motion_finished = finished;
    }
    fn motion_finished(mut self) -> Self {
        self.set_motion_finished(true);
        self
    }
}
impl ConvertMotion<PandaState> for JointVelocities {

    fn convert_motion(
        &self,
        robot_state: &PandaState,
        command: &mut MotionGeneratorCommand,
        cutoff_frequency: f64,
        limit_rate: bool,
    ) {
        command.dq_c = self.dq;
        if cutoff_frequency < MAX_CUTOFF_FREQUENCY {
            for i in 0..7 {
                command.dq_c[i] = low_pass_filter(
                    DELTA_T,
                    command.dq_c[i],
                    robot_state.dq_d[i],
                    cutoff_frequency,
                );
            }
        }
        if limit_rate {
            command.dq_c = limit_rate_joint_velocities(
                &MAX_JOINT_VELOCITY,
                &MAX_JOINT_ACCELERATION,
                &MAX_JOINT_JERK,
                &command.dq_c,
                &robot_state.dq_d,
                &robot_state.ddq_d,
            );
        }
        command.dq_c.iter().for_each(|x| assert!(x.is_finite()));
    }
}

impl ConvertMotion<FR3State> for JointVelocities {
    fn convert_motion(
        &self,
        robot_state: &FR3State,
        command: &mut MotionGeneratorCommand,
        cutoff_frequency: f64,
        limit_rate: bool,
    ) {
        command.dq_c = self.dq;
        if cutoff_frequency < MAX_CUTOFF_FREQUENCY {
            for i in 0..7 {
                command.dq_c[i] = low_pass_filter(
                    DELTA_T,
                    command.dq_c[i],
                    robot_state.dq_d[i],
                    cutoff_frequency,
                );
            }
        }
        if limit_rate {
            command.dq_c = limit_rate_joint_velocities(
                &MAX_JOINT_VELOCITY,
                &MAX_JOINT_ACCELERATION,
                &MAX_JOINT_JERK,
                &command.dq_c,
                &robot_state.dq_d,
                &robot_state.ddq_d,
            );
        }
        command.dq_c.iter().for_each(|x| assert!(x.is_finite()));
    }
}

impl MotionGeneratorTrait for JointVelocities {
    fn get_motion_generator_mode() -> MoveMotionGeneratorMode {
        MoveMotionGeneratorMode::JointVelocity
    }
}

/// Stores values for Cartesian pose motion generation.
#[derive(Serialize, Deserialize, Debug, Copy, Clone)]
#[allow(non_snake_case)]
pub struct CartesianPose {
    motion_finished: bool,
    /// Homogeneous transformation ![^O{\mathbf{T}_{EE}}_{d}](https://latex.codecogs.com/png.latex?^O{\mathbf{T}_{EE}}_{d}), column major, that transforms from
    /// the end effector frame `EE` to base frame `O`.
    /// Equivalently, it is the desired end effector pose in base frame.
    pub O_T_EE: [f64; 16],
    /// Elbow configuration.
    ///
    /// If "None" the elbow will be controlled by the robot
    ///
    /// The values of the array are:
    ///  - \[0\] Position of the 3rd joint in \[rad\].
    ///  - \[1\] Sign of the 4th joint. Can be +1 or -1.
    pub elbow: Option<[f64; 2]>,
}

impl From<Isometry3<f64>> for CartesianPose {
    fn from(isometry: Isometry3<f64>) -> Self {
        let mut out = [0.; 16];
        for (i, &x) in isometry.to_homogeneous().into_iter().enumerate() {
            out[i] = x;
        }
        CartesianPose::new(out, None)
    }
}

impl From<[f64; 16]> for CartesianPose {
    fn from(array: [f64; 16]) -> Self {
        CartesianPose::new(array, None)
    }
}

impl CartesianPose {
    /// Creates a new CartesianPose instance.
    /// # Arguments
    /// * `cartesian_pose` - Desired vectorized homogeneous transformation matrix
    /// ![^O{\mathbf{T}_{EE}}_{d}](https://latex.codecogs.com/png.latex?^O{\mathbf{T}_{EE}}_{d})
    /// , column major, that transforms from the end effector frame `EE` to
    /// base frame `O`. Equivalently, it is the desired end effector pose in base frame.
    ///
    /// * `elbow` - Elbow configuration. See [elbow](#structfield.elbow)
    pub fn new(cartesian_pose: [f64; 16], elbow: Option<[f64; 2]>) -> Self {
        CartesianPose {
            O_T_EE: cartesian_pose,
            motion_finished: false,
            elbow,
        }
    }
    /// Determines whether there is a stored elbow configuration.
    pub fn has_elbow(&self) -> bool {
        self.elbow.is_some()
    }
    /// Determines whether the elbow configuration is valid and has finite values
    pub fn check_elbow(elbow: &[f64; 2]) {
        elbow.iter().for_each(|x| assert!(x.is_finite()));
        assert!(CartesianPose::is_valid_elbow(elbow));
    }
    /// Determines whether the given elbow configuration is valid or not.
    #[allow(clippy::float_cmp)]
    pub fn is_valid_elbow(elbow: &[f64; 2]) -> bool {
        elbow[1].abs() == 1.
    }
}

impl Finishable for CartesianPose{
    fn is_finished(&self) -> bool {
        self.motion_finished
    }
    fn set_motion_finished(&mut self, finished: bool) {
        self.motion_finished = finished;
    }
    fn motion_finished(mut self) -> Self {
        self.set_motion_finished(true);
        self
    }
}

impl ConvertMotion<PandaState> for CartesianPose {

    fn convert_motion(
        &self,
        robot_state: &PandaState,
        command: &mut MotionGeneratorCommand,
        cutoff_frequency: f64,
        limit_rate: bool,
    ) {
        command.O_T_EE_c = self.O_T_EE;
        if cutoff_frequency < MAX_CUTOFF_FREQUENCY {
            command.O_T_EE_c = cartesian_low_pass_filter(
                DELTA_T,
                &command.O_T_EE_c,
                &robot_state.O_T_EE_c,
                cutoff_frequency,
            );
        }

        if limit_rate {
            command.O_T_EE_c = limit_rate_cartesian_pose(
                MAX_TRANSLATIONAL_VELOCITY,
                MAX_TRANSLATIONAL_ACCELERATION,
                MAX_TRANSLATIONAL_JERK,
                MAX_ROTATIONAL_VELOCITY,
                MAX_ROTATIONAL_ACCELERATION,
                MAX_ROTATIONAL_JERK,
                &command.O_T_EE_c,
                &robot_state.O_T_EE_c,
                &robot_state.O_dP_EE_c,
                &robot_state.O_ddP_EE_c,
            );
        }
        check_matrix(&command.O_T_EE_c);

        if self.has_elbow() {
            command.valid_elbow = true;
            command.elbow_c = self.elbow.unwrap();
            if cutoff_frequency < MAX_CUTOFF_FREQUENCY {
                command.elbow_c[0] = low_pass_filter(
                    DELTA_T,
                    command.elbow_c[0],
                    robot_state.elbow_c[0],
                    cutoff_frequency,
                );
            }
            if limit_rate {
                command.elbow_c[0] = limit_rate_position(
                    MAX_ELBOW_VELOCITY,
                    MAX_ELBOW_ACCELERATION,
                    MAX_ELBOW_JERK,
                    command.elbow_c[0],
                    robot_state.elbow_c[0],
                    robot_state.delbow_c[0],
                    robot_state.ddelbow_c[0],
                );
            }
            CartesianPose::check_elbow(&command.elbow_c);
        } else {
            command.valid_elbow = false;
            command.elbow_c = [0.; 2];
        }
    }
}

impl ConvertMotion<FR3State> for CartesianPose {

    fn convert_motion(
        &self,
        robot_state: &FR3State,
        command: &mut MotionGeneratorCommand,
        cutoff_frequency: f64,
        limit_rate: bool,
    ) {
        command.O_T_EE_c = self.O_T_EE;
        if cutoff_frequency < MAX_CUTOFF_FREQUENCY {
            command.O_T_EE_c = cartesian_low_pass_filter(
                DELTA_T,
                &command.O_T_EE_c,
                &robot_state.O_T_EE_c,
                cutoff_frequency,
            );
        }

        if limit_rate {
            command.O_T_EE_c = limit_rate_cartesian_pose(
                MAX_TRANSLATIONAL_VELOCITY,
                MAX_TRANSLATIONAL_ACCELERATION,
                MAX_TRANSLATIONAL_JERK,
                MAX_ROTATIONAL_VELOCITY,
                MAX_ROTATIONAL_ACCELERATION,
                MAX_ROTATIONAL_JERK,
                &command.O_T_EE_c,
                &robot_state.O_T_EE_c,
                &robot_state.O_dP_EE_c,
                &robot_state.O_ddP_EE_c,
            );
        }
        check_matrix(&command.O_T_EE_c);

        if self.has_elbow() {
            command.valid_elbow = true;
            command.elbow_c = self.elbow.unwrap();
            if cutoff_frequency < MAX_CUTOFF_FREQUENCY {
                command.elbow_c[0] = low_pass_filter(
                    DELTA_T,
                    command.elbow_c[0],
                    robot_state.elbow_c[0],
                    cutoff_frequency,
                );
            }
            if limit_rate {
                command.elbow_c[0] = limit_rate_position(
                    MAX_ELBOW_VELOCITY,
                    MAX_ELBOW_ACCELERATION,
                    MAX_ELBOW_JERK,
                    command.elbow_c[0],
                    robot_state.elbow_c[0],
                    robot_state.delbow_c[0],
                    robot_state.ddelbow_c[0],
                );
            }
            CartesianPose::check_elbow(&command.elbow_c);
        } else {
            command.valid_elbow = false;
            command.elbow_c = [0.; 2];
        }
    }
}

impl MotionGeneratorTrait for CartesianPose {
    fn get_motion_generator_mode() -> MoveMotionGeneratorMode {
        MoveMotionGeneratorMode::CartesianPosition
    }
}

///  Stores values for Cartesian velocity motion generation.
#[derive(Serialize, Deserialize, Debug, Copy, Clone)]
#[allow(non_snake_case)]
pub struct CartesianVelocities {
    motion_finished: bool,
    /// Desired Cartesian velocity w.r.t. O-frame {dx in \[m/s\], dy in \[m/s\], dz in \[m/s\], omegax in
    /// \[rad/s\], omegay in \[rad/s\], omegaz in \[rad/s\]}.
    pub O_dP_EE: [f64; 6],
    /// Elbow configuration.
    ///
    /// If "None" the elbow will be controlled by the robot
    ///
    /// The values of the array are:
    ///  - \[0\] Position of the 3rd joint in \[rad\].
    ///  - \[1\] Sign of the 4th joint. Can be +1 or -1.
    pub elbow: Option<[f64; 2]>,
}

impl From<Vector6<f64>> for CartesianVelocities {
    fn from(vector: Vector6<f64>) -> Self {
        CartesianVelocities::new(vector.into(), None)
    }
}

impl CartesianVelocities {
    /// Creates a new CartesianVelocities instance.
    /// # Arguments
    /// * `cartesian_velocities` - cartesian_velocities Desired Cartesian velocity w.r.t. O-frame {dx in \[m/s\], dy in
    ///    * \[m/s\], dz in \[m/s\], omegax in \[rad/s\], omegay in \[rad/s\], omegaz in \[rad/s\]}.
    ///
    /// * `elbow` - Elbow configuration. See [`elbow`](`Self::elbow`)
    pub fn new(cartesian_velocities: [f64; 6], elbow: Option<[f64; 2]>) -> Self {
        CartesianVelocities {
            O_dP_EE: cartesian_velocities,
            motion_finished: false,
            elbow,
        }
    }
    /// Determines whether there is a stored elbow configuration.
    pub fn has_elbow(&self) -> bool {
        self.elbow.is_some()
    }
}
impl Finishable for CartesianVelocities{
    fn is_finished(&self) -> bool {
        self.motion_finished
    }
    fn set_motion_finished(&mut self, finished: bool) {
        self.motion_finished = finished;
    }
    fn motion_finished(mut self) -> Self {
        self.set_motion_finished(true);
        self
    }
}
impl ConvertMotion<PandaState> for CartesianVelocities {

    fn convert_motion(
        &self,
        robot_state: &PandaState,
        command: &mut MotionGeneratorCommand,
        cutoff_frequency: f64,
        limit_rate: bool,
    ) {
        command.O_dP_EE_c = self.O_dP_EE;
        if cutoff_frequency < MAX_CUTOFF_FREQUENCY {
            for i in 0..6 {
                command.O_dP_EE_c[i] = low_pass_filter(
                    DELTA_T,
                    command.O_dP_EE_c[i],
                    robot_state.O_dP_EE_c[i],
                    cutoff_frequency,
                );
            }
        }
        if limit_rate {
            command.O_dP_EE_c = limit_rate_cartesian_velocity(
                MAX_TRANSLATIONAL_VELOCITY,
                MAX_TRANSLATIONAL_ACCELERATION,
                MAX_TRANSLATIONAL_JERK,
                MAX_ROTATIONAL_VELOCITY,
                MAX_ROTATIONAL_ACCELERATION,
                MAX_ROTATIONAL_JERK,
                &command.O_dP_EE_c,
                &robot_state.O_dP_EE_c,
                &robot_state.O_ddP_EE_c,
            );
        }
        command
            .O_dP_EE_c
            .iter()
            .for_each(|x| assert!(x.is_finite()));

        if self.has_elbow() {
            command.valid_elbow = true;
            command.elbow_c = self.elbow.unwrap();
            if cutoff_frequency < MAX_CUTOFF_FREQUENCY {
                command.elbow_c[0] = low_pass_filter(
                    DELTA_T,
                    command.elbow_c[0],
                    robot_state.elbow_c[0],
                    cutoff_frequency,
                );
            }
            if limit_rate {
                command.elbow_c[0] = limit_rate_position(
                    MAX_ELBOW_VELOCITY,
                    MAX_ELBOW_ACCELERATION,
                    MAX_ELBOW_JERK,
                    command.elbow_c[0],
                    robot_state.elbow_c[0],
                    robot_state.delbow_c[0],
                    robot_state.ddelbow_c[0],
                );
            }
            CartesianPose::check_elbow(&command.elbow_c);
        } else {
            command.valid_elbow = false;
            command.elbow_c = [0.; 2];
        }
    }
}

impl ConvertMotion<FR3State> for CartesianVelocities {

    fn convert_motion(
        &self,
        robot_state: &FR3State,
        command: &mut MotionGeneratorCommand,
        cutoff_frequency: f64,
        limit_rate: bool,
    ) {
        command.O_dP_EE_c = self.O_dP_EE;
        if cutoff_frequency < MAX_CUTOFF_FREQUENCY {
            for i in 0..6 {
                command.O_dP_EE_c[i] = low_pass_filter(
                    DELTA_T,
                    command.O_dP_EE_c[i],
                    robot_state.O_dP_EE_c[i],
                    cutoff_frequency,
                );
            }
        }
        if limit_rate {
            command.O_dP_EE_c = limit_rate_cartesian_velocity(
                MAX_TRANSLATIONAL_VELOCITY,
                MAX_TRANSLATIONAL_ACCELERATION,
                MAX_TRANSLATIONAL_JERK,
                MAX_ROTATIONAL_VELOCITY,
                MAX_ROTATIONAL_ACCELERATION,
                MAX_ROTATIONAL_JERK,
                &command.O_dP_EE_c,
                &robot_state.O_dP_EE_c,
                &robot_state.O_ddP_EE_c,
            );
        }
        command
            .O_dP_EE_c
            .iter()
            .for_each(|x| assert!(x.is_finite()));

        if self.has_elbow() {
            command.valid_elbow = true;
            command.elbow_c = self.elbow.unwrap();
            if cutoff_frequency < MAX_CUTOFF_FREQUENCY {
                command.elbow_c[0] = low_pass_filter(
                    DELTA_T,
                    command.elbow_c[0],
                    robot_state.elbow_c[0],
                    cutoff_frequency,
                );
            }
            if limit_rate {
                command.elbow_c[0] = limit_rate_position(
                    MAX_ELBOW_VELOCITY,
                    MAX_ELBOW_ACCELERATION,
                    MAX_ELBOW_JERK,
                    command.elbow_c[0],
                    robot_state.elbow_c[0],
                    robot_state.delbow_c[0],
                    robot_state.ddelbow_c[0],
                );
            }
            CartesianPose::check_elbow(&command.elbow_c);
        } else {
            command.valid_elbow = false;
            command.elbow_c = [0.; 2];
        }
    }
}

impl MotionGeneratorTrait for CartesianVelocities {
    fn get_motion_generator_mode() -> MoveMotionGeneratorMode {
        MoveMotionGeneratorMode::CartesianVelocity
    }
}

fn check_matrix(transform: &[f64; 16]) {
    transform.iter().for_each(|x| assert!(x.is_finite()));
    assert!(is_homogeneous_transformation(transform));
}
