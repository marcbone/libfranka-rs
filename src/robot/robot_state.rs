// Copyright (c) 2021 Marco Boneberger
// Licensed under the EUPL-1.2-or-later

//! Contains the franka::RobotState types.
use std::time::Duration;

use crate::robot::errors::{FrankaErrorKind, FrankaErrors};
use crate::robot::types::{RobotMode, RobotStateIntern};
use nalgebra::{Matrix3, Vector3};

/// Describes the robot state.
#[derive(Debug, Clone, Default)]
#[allow(non_snake_case)]
pub struct RobotState {
    /// ![^{O}T_{EE}](https://latex.codecogs.com/png.latex?^{O}T_{EE})
    ///
    /// Measured end effector pose in base frame.
    /// Pose is represented as a 4x4 matrix in column-major format.
    pub O_T_EE: [f64; 16],
    /// ![{^OT_{EE}}_{d}](http://latex.codecogs.com/png.latex?{^OT_{EE}}_{d})
    ///
    /// Last desired end effector pose of motion generation in base frame.
    /// Pose is represented as a 4x4 matrix in column-major format.
    pub O_T_EE_d: [f64; 16],
    /// ![^{F}T_{EE}](http://latex.codecogs.com/png.latex?^{F}T_{EE})
    ///
    /// End effector frame pose in flange frame.
    /// Pose is represented as a 4x4 matrix in column-major format.
    /// # See
    /// * [`F_T_NE`](`Self::F_T_NE`)
    /// * [`NE_T_EE`](`Self::NE_T_EE`)
    /// * [`Robot`](`crate::Robot`) for an explanation of the NE and EE frames.
    pub F_T_EE: [f64; 16],
    /// ![^{F}T_{NE}](https://latex.codecogs.com/png.latex?^{F}T_{NE})
    ///
    /// Nominal end effector frame pose in flange frame.
    /// Pose is represented as a 4x4 matrix in column-major format.
    /// # See
    /// * [`F_T_NE`](`Self::F_T_NE`)
    /// * [`NE_T_EE`](`Self::NE_T_EE`)
    /// * [`Robot`](`crate::Robot`) for an explanation of the NE and EE frames.
    pub F_T_NE: [f64; 16],
    /// ![^{NE}T_{EE}](https://latex.codecogs.com/png.latex?^{NE}T_{EE})
    ///
    /// End effector frame pose in nominal end effector frame.
    /// Pose is represented as a 4x4 matrix in column-major format.
    /// # See
    /// * [`F_T_NE`](`Self::F_T_NE`)
    /// * [`NE_T_EE`](`Self::NE_T_EE`)
    /// * [`Robot`](`crate::Robot`) for an explanation of the NE and EE frames.
    pub NE_T_EE: [f64; 16],
    /// ![^{EE}T_{K}](https://latex.codecogs.com/png.latex?^{EE}T_{K})
    ///
    /// Stiffness frame pose in end effector frame.
    /// Pose is represented as a 4x4 matrix in column-major format.
    ///
    /// See also [K-Frame](`crate::Robot#stiffness-frame-k`)
    pub EE_T_K: [f64; 16],
    /// ![m_{EE}](https://latex.codecogs.com/png.latex?m_{EE})
    ///
    /// Configured mass of the end effector.
    pub m_ee: f64,
    /// ![I_{EE}](https://latex.codecogs.com/png.latex?I_{EE})
    ///
    /// Configured rotational inertia matrix of the end effector load with respect to center of mass.
    pub I_ee: [f64; 9],
    /// ![^{F}x_{C_{EE}}](https://latex.codecogs.com/png.latex?^{F}x_{C_{EE}})
    ///
    /// Configured center of mass of the end effector load with respect to flange frame.
    pub F_x_Cee: [f64; 3],
    /// ![m_{load}](https://latex.codecogs.com/png.latex?m_{load})
    ///
    /// Configured mass of the external load.
    pub m_load: f64,
    /// ![I_{load}](https://latex.codecogs.com/png.latex?I_{load})
    ///
    /// Configured rotational inertia matrix of the external load with respect to center of mass.
    pub I_load: [f64; 9],
    /// ![^{F}x_{C_{load}}](https://latex.codecogs.com/png.latex?^{F}x_{C_{load}})
    ///
    /// Configured center of mass of the external load with respect to flange frame.
    pub F_x_Cload: [f64; 3],
    /// ![m_{total}](https://latex.codecogs.com/png.latex?m_{total})
    ///
    /// Sum of the mass of the end effector and the external load.
    pub m_total: f64,
    /// ![I_{total}](https://latex.codecogs.com/png.latex?I_{total})
    ///
    /// Combined rotational inertia matrix of the end effector load and the external load with respect
    /// to the center of mass.
    pub I_total: [f64; 9],
    /// ![^{F}x_{C_{total}}](https://latex.codecogs.com/png.latex?^{F}x_{C_{total}})
    ///
    /// Combined center of mass of the end effector load and the external load with respect to flange
    /// frame.
    pub F_x_Ctotal: [f64; 3],
    /// Elbow configuration.
    ///
    /// The values of the array are:
    ///  - \[0\] Position of the 3rd joint in \[rad\].
    ///  - \[1\] Sign of the 4th joint. Can be +1 or -1.
    pub elbow: [f64; 2],
    /// Desired elbow configuration.
    ///
    /// The values of the array are:
    ///  - \[0\] Position of the 3rd joint in \[rad\].
    ///  - \[1\] Sign of the 4th joint. Can be +1 or -1.
    pub elbow_d: [f64; 2],
    /// Commanded elbow configuration.
    ///
    /// The values of the array are:
    ///  - \[0\] Position of the 3rd joint in \[rad\].
    ///  - \[1\] Sign of the 4th joint. Can be +1 or -1.
    pub elbow_c: [f64; 2],
    /// Commanded elbow velocity.
    ///
    /// The values of the array are:
    ///  - \[0\] Velocity of the 3rd joint in \[rad/s\].
    ///  - \[1\] Sign of the 4th joint. Can be +1 or -1.
    pub delbow_c: [f64; 2],
    /// Commanded elbow acceleration.
    ///
    /// The values of the array are:
    ///  - \[0\] Acceleration of the 3rd joint in \[rad/s^2\].
    ///  - \[1\] Sign of the 4th joint. Can be +1 or -1.
    pub ddelbow_c: [f64; 2],
    /// ![\tau_{J}](https://latex.codecogs.com/png.latex?\tau_{J})
    ///
    /// Measured link-side joint torque sensor signals. Unit: \[Nm\]
    pub tau_J: [f64; 7],
    /// ![{\tau_J}_d](https://latex.codecogs.com/png.latex?{\tau_J}_d)
    ///
    /// Desired link-side joint torque sensor signals without gravity. Unit:  \[Nm\]
    pub tau_J_d: [f64; 7],
    /// ![\dot{\tau_{J}}](https://latex.codecogs.com/png.latex?\dot{\tau_{J}})
    ///
    /// Derivative of measured link-side joint torque sensor signals. Unit: [Nm/s]
    pub dtau_J: [f64; 7],
    /// ![q](https://latex.codecogs.com/png.latex?q)
    ///
    /// Measured joint position. Unit: \[rad\]
    pub q: [f64; 7],
    /// ![q_d](https://latex.codecogs.com/png.latex?q_d)
    ///
    /// Desired joint position. Unit: \[rad\]
    pub q_d: [f64; 7],
    /// ![\dot{q}](https://latex.codecogs.com/png.latex?\dot{q})
    ///
    /// Measured joint velocity. Unit: \[rad/s\]
    pub dq: [f64; 7],
    /// ![\dot{q}_d](https://latex.codecogs.com/png.latex?\dot{q}_d)
    ///
    /// Desired joint velocity. Unit: \[rad/s\]
    pub dq_d: [f64; 7],
    /// ![\ddot{q}_d](https://latex.codecogs.com/png.latex?\ddot{q}_d)
    ///
    /// Desired joint acceleration. Unit: \[rad/s^2\]
    pub ddq_d: [f64; 7],
    /// Indicates which contact level is activated in which joint. After contact disappears, value
    /// turns to zero.
    ///
    /// See [`Robot::set_collision_behavior`](`crate::Robot::set_collision_behavior`)
    /// for setting sensitivity values.
    pub joint_contact: [f64; 7],
    /// Indicates which contact level is activated in which Cartesian dimension (x,y,z,R,P,Y).
    /// After contact disappears, the value turns to zero.
    ///
    /// See [`Robot::set_collision_behavior`](`crate::Robot::set_collision_behavior`)
    /// for setting sensitivity values.
    pub cartesian_contact: [f64; 6],
    /// Indicates which contact level is activated in which joint. After contact disappears, the value
    /// stays the same until a reset command is sent.
    ///
    /// See [`Robot::set_collision_behavior`](`crate::Robot::set_collision_behavior`)
    /// for setting sensitivity values.
    ///
    /// See [`Robot::automatic_error_recovery`](`crate::Robot::automatic_error_recovery`)
    /// for performing a reset after a collision.
    pub joint_collision: [f64; 7],
    /// Indicates which contact level is activated in which Cartesian dimension (x,y,z,R,P,Y).
    /// After contact disappears, the value stays the same until a reset command is sent.
    ///
    /// See [`Robot::set_collision_behavior`](`crate::Robot::set_collision_behavior`)
    /// for setting sensitivity values.
    ///
    /// See [`Robot::automatic_error_recovery`](`crate::Robot::automatic_error_recovery`)
    /// for performing a reset after a collision.
    pub cartesian_collision: [f64; 6],
    /// ![\hat{\tau}_{\text{ext}}](https://latex.codecogs.com/png.latex?\hat{\tau}_{\text{ext}})
    ///
    /// External torque, filtered. Unit: \[Nm\]
    pub tau_ext_hat_filtered: [f64; 7],
    /// ![^OF_{K,\text{ext}}](https://latex.codecogs.com/png.latex?^OF_{K,\text{ext}})
    ///
    /// Estimated external wrench (force, torque) acting on stiffness frame, expressed
    /// relative to the base frame. See also @ref k-frame "K frame".
    /// Unit: \[N,N,N,Nm,Nm,Nm\].
    pub O_F_ext_hat_K: [f64; 6],
    /// ![^{K}F_{K,\text{ext}}](https://latex.codecogs.com/png.latex?^{K}F_{K,\text{ext}})
    ///
    /// Estimated external wrench (force, torque) acting on stiffness frame,
    /// expressed relative to the stiffness frame. See also @ref k-frame "K frame".
    /// Unit: \[N,N,N,Nm,Nm,Nm\].
    pub K_F_ext_hat_K: [f64; 6],
    /// ![{^OdP_{EE}}_{d}](https://latex.codecogs.com/png.latex?{^OdP_{EE}}_{d})
    ///
    /// Desired end effector twist in base frame.
    /// Unit: [m/s,m/s,m/s,rad/s,rad/s,rad/s]
    pub O_dP_EE_d: [f64; 6],
    /// ![{^OT_{EE}}_{c}](https://latex.codecogs.com/png.latex?{^OT_{EE}}_{c})
    ///
    /// Last commanded end effector pose of motion generation in base frame.
    /// Pose is represented as a 4x4 matrix in column-major format.
    pub O_T_EE_c: [f64; 16],
    /// ![{^OdP_{EE}}_{c}](https://latex.codecogs.com/png.latex?{^OdP_{EE}}_{c})
    ///
    /// Last commanded end effector twist in base frame.
    /// Unit: [m/s,m/s,m/s,rad/s,rad/s,rad/s]
    pub O_dP_EE_c: [f64; 6],
    ///![{^OddP_{EE}}_{c}](https://latex.codecogs.com/png.latex?{^OddP_{EE}}_{c})
    ///
    /// Last commanded end effector acceleration in base frame.
    /// Unit:  [m/s^2,m/s^2,m/s^2,rad/s^2,rad/s^2,rad/s^2]
    pub O_ddP_EE_c: [f64; 6],
    /// ![\theta](https://latex.codecogs.com/png.latex?\theta)
    ///
    /// Motor position. Unit: \[rad\]
    pub theta: [f64; 7],
    /// ![\dot{\theta}](https://latex.codecogs.com/png.latex?\dot{\theta})
    ///
    /// Motor velocity. Unit: \[rad/s\]
    pub dtheta: [f64; 7],
    /// Current error state.
    pub current_errors: FrankaErrors,
    /// Contains the errors that aborted the previous motion
    pub last_motion_errors: FrankaErrors,
    /// Percentage of the last 100 control commands that were successfully received by the robot.
    ///
    /// Shows a value of zero if no control or motion generator loop is currently running.
    ///
    /// Range \[0,1\]
    pub control_command_success_rate: f64,
    /// Current robot mode.
    pub robot_mode: RobotMode,
    /// Strictly monotonically increasing timestamp since robot start.
    ///
    /// Inside of control loops "time_step" parameter of Robot::control can be used
    /// instead
    pub time: Duration,
}
impl From<RobotStateIntern> for RobotState {
    #[allow(non_snake_case)]
    fn from(robot_state: RobotStateIntern) -> Self {
        let O_T_EE = robot_state.O_T_EE;
        let O_T_EE_d = robot_state.O_T_EE_d;
        let F_T_NE = robot_state.F_T_NE;
        let NE_T_EE = robot_state.NE_T_EE;
        let F_T_EE = robot_state.F_T_EE;
        let EE_T_K = robot_state.EE_T_K;
        let m_ee = robot_state.m_ee;
        let F_x_Cee = robot_state.F_x_Cee;
        let I_ee = robot_state.I_ee;
        let m_load = robot_state.m_load;
        let F_x_Cload = robot_state.F_x_Cload;
        let I_load = robot_state.I_load;
        let m_total = robot_state.m_ee + robot_state.m_load;
        let F_x_Ctotal = combine_center_of_mass(
            robot_state.m_ee,
            robot_state.F_x_Cee,
            robot_state.m_load,
            robot_state.F_x_Cload,
        );
        let I_total = combine_inertia_tensor(
            robot_state.m_ee,
            robot_state.F_x_Cee,
            robot_state.I_ee,
            robot_state.m_load,
            robot_state.F_x_Cload,
            robot_state.I_load,
            m_total,
            F_x_Ctotal,
        );
        let elbow = robot_state.elbow;
        let elbow_d = robot_state.elbow_d;
        let elbow_c = robot_state.elbow_c;
        let delbow_c = robot_state.delbow_c;
        let ddelbow_c = robot_state.ddelbow_c;
        let tau_J = robot_state.tau_J;
        let tau_J_d = robot_state.tau_J_d;
        let dtau_J = robot_state.dtau_J;
        let q = robot_state.q;
        let dq = robot_state.dq;
        let q_d = robot_state.q_d;
        let dq_d = robot_state.dq_d;
        let ddq_d = robot_state.ddq_d;
        let joint_contact = robot_state.joint_contact;
        let cartesian_contact = robot_state.cartesian_contact;
        let joint_collision = robot_state.joint_collision;
        let cartesian_collision = robot_state.cartesian_collision;
        let tau_ext_hat_filtered = robot_state.tau_ext_hat_filtered;
        let O_F_ext_hat_K = robot_state.O_F_ext_hat_K;
        let K_F_ext_hat_K = robot_state.K_F_ext_hat_K;
        let O_dP_EE_d = robot_state.O_dP_EE_d;
        let O_T_EE_c = robot_state.O_T_EE_c;
        let O_dP_EE_c = robot_state.O_dP_EE_c;
        let O_ddP_EE_c = robot_state.O_ddP_EE_c;
        let theta = robot_state.theta;
        let dtheta = robot_state.dtheta;
        let control_command_success_rate = robot_state.control_command_success_rate;
        let time = Duration::from_millis(robot_state.message_id);
        let robot_mode = robot_state.robot_mode;
        let current_errors = FrankaErrors::new(robot_state.errors, FrankaErrorKind::Error);
        let last_motion_errors =
            FrankaErrors::new(robot_state.errors, FrankaErrorKind::ReflexReason);
        RobotState {
            O_T_EE,
            O_T_EE_d,
            F_T_EE,
            F_T_NE,
            NE_T_EE,
            EE_T_K,
            m_ee,
            I_ee,
            F_x_Cee,
            m_load,
            I_load,
            F_x_Cload,
            m_total,
            I_total,
            F_x_Ctotal,
            elbow,
            elbow_d,
            elbow_c,
            delbow_c,
            ddelbow_c,
            tau_J,
            tau_J_d,
            dtau_J,
            q,
            q_d,
            dq,
            dq_d,
            ddq_d,
            joint_contact,
            cartesian_contact,
            joint_collision,
            cartesian_collision,
            tau_ext_hat_filtered,
            O_F_ext_hat_K,
            K_F_ext_hat_K,
            O_dP_EE_d,
            O_T_EE_c,
            O_dP_EE_c,
            O_ddP_EE_c,
            theta,
            dtheta,
            current_errors,
            last_motion_errors,
            control_command_success_rate,
            robot_mode,
            time,
        }
    }
}

#[allow(non_snake_case, clippy::too_many_arguments)]
fn combine_inertia_tensor(
    m_ee: f64,
    F_x_Cee: [f64; 3],
    I_ee: [f64; 9],
    m_load: f64,
    F_x_Cload: [f64; 3],
    I_load: [f64; 9],
    m_total: f64,
    F_x_Ctotal: [f64; 3],
) -> [f64; 9] {
    let center_of_mass_ee = Vector3::from_column_slice(&F_x_Cee);
    let center_of_mass_load = Vector3::from_column_slice(&F_x_Cload);
    let center_of_mass_total = Vector3::from_column_slice(&F_x_Ctotal);

    let mut inertia_ee = Matrix3::from_column_slice(&I_ee);
    let mut inertia_load = Matrix3::from_column_slice(&I_load);

    if m_ee == 0. {
        inertia_ee = Matrix3::zeros();
    }
    if m_load == 0. {
        inertia_load = Matrix3::zeros();
    }
    let inertia_ee_flange = inertia_ee
        - m_ee
            * (skew_symmetric_matrix_from_vector(&center_of_mass_ee)
                * skew_symmetric_matrix_from_vector(&center_of_mass_ee));
    let inertia_load_flange = inertia_load
        - m_load
            * (skew_symmetric_matrix_from_vector(&center_of_mass_load)
                * skew_symmetric_matrix_from_vector(&center_of_mass_load));
    let inertia_total_flange = inertia_ee_flange + inertia_load_flange;

    let inertia_total: Matrix3<f64> = inertia_total_flange
        + m_total
            * (skew_symmetric_matrix_from_vector(&center_of_mass_total)
                * skew_symmetric_matrix_from_vector(&center_of_mass_total));
    let mut I_total = [0.; 9];
    for (i, &x) in inertia_total.as_slice().iter().enumerate() {
        I_total[i] = x;
    }
    I_total
}

#[allow(non_snake_case)]
fn combine_center_of_mass(
    m_ee: f64,
    F_x_Cee: [f64; 3],
    m_load: f64,
    F_x_Cload: [f64; 3],
) -> [f64; 3] {
    let mut F_x_Ctotal = [0.; 3];
    if m_ee + m_load > 0. {
        for i in 0..F_x_Ctotal.len() {
            F_x_Ctotal[i] = (m_ee * F_x_Cee[i] + m_load * F_x_Cload[i]) / (m_ee + m_load);
        }
    }
    F_x_Ctotal
}

fn skew_symmetric_matrix_from_vector(vector: &Vector3<f64>) -> Matrix3<f64> {
    Matrix3::new(
        0., -vector.z, vector.y, vector.z, 0., -vector.x, -vector.y, vector.x, 0.,
    )
}

#[cfg(test)]
mod tests {
    use crate::robot::robot_state::{combine_center_of_mass, combine_inertia_tensor};

    #[test]
    fn combine_com_test() {
        let m_ee = 0.;
        let m_load = 0.;
        let f_x_ctotal = combine_center_of_mass(m_ee, [0.; 3], m_load, [0.; 3]);
        f_x_ctotal
            .iter()
            .for_each(|&x| assert!(x.abs() < f64::EPSILON));

        let m_ee = 0.73;
        let m_load = 0.5;
        let f_x_cee = [-0.01, 0., -0.03];
        let i_ee = [0.001, 0.0, 0.0, 0.0, 0.0025, 0.0, 0.0, 0.0, 0.0017];
        let i_load = [0.001, 0.0, 0.0, 0.0, 0.025, 0.0, 0.0, 0.0, 0.3];
        let f_x_cload = [0.01, -0.2, 0.03];
        let m_total = m_ee + m_load;
        let f_x_ctotal = combine_center_of_mass(m_ee, f_x_cee, m_load, f_x_cload);
        let i_total = combine_inertia_tensor(
            m_ee, f_x_cee, i_ee, m_load, f_x_cload, i_load, m_total, f_x_ctotal,
        );
        let expected = [
            1.49382113821138e-02,
            1.18699186991870e-03,
            -3.56097560975610e-04,
            1.18699186991870e-03,
            2.36869918699187e-02,
            3.56097560975610e-03,
            -3.56097560975610e-04,
            3.56097560975610e-03,
            3.13688617886179e-01,
        ];
        i_total
            .iter()
            .zip(expected.iter())
            .take(3)
            .for_each(|(&x, &y)| assert!((x - y).abs() < 1e-14));
    }

    #[test]
    fn inertia_zero_test() {
        let m_ee = 0.;
        let m_load = 0.0;
        let f_x_cee = [-0.01, 0., -0.03];
        let i_ee = [0.001, 0.0, 0.0, 0.0, 0.0025, 0.0, 0.0, 0.0, 0.0017];
        let i_load = [0.001, 0.0, 0.0, 0.0, 0.025, 0.0, 0.0, 0.0, 0.3];
        let f_x_cload = [0.01, -0.2, 0.03];
        let m_total = m_ee + m_load;
        let f_x_ctotal = combine_center_of_mass(m_ee, f_x_cee, m_load, f_x_cload);
        let i_total = combine_inertia_tensor(
            m_ee, f_x_cee, i_ee, m_load, f_x_cload, i_load, m_total, f_x_ctotal,
        );
        let expected = [0.; 9];
        i_total
            .iter()
            .zip(expected.iter())
            .take(3)
            .for_each(|(&x, &y)| assert!((x - y).abs() < 1e-17));

        let m_ee = 0.;
        let m_load = 0.5;
        let f_x_cee = [-0.01, 0., -0.03];
        let i_ee = [0.001, 0.0, 0.0, 0.0, 0.0025, 0.0, 0.0, 0.0, 0.0017];
        let i_load = [0.001, 0.0, 0.0, 0.0, 0.025, 0.0, 0.0, 0.0, 0.3];
        let f_x_cload = [0.01, -0.2, 0.03];
        let m_total = m_ee + m_load;
        let f_x_ctotal = combine_center_of_mass(m_ee, f_x_cee, m_load, f_x_cload);
        let i_total = combine_inertia_tensor(
            m_ee, f_x_cee, i_ee, m_load, f_x_cload, i_load, m_total, f_x_ctotal,
        );
        let expected = i_load;
        i_total
            .iter()
            .zip(expected.iter())
            .take(3)
            .for_each(|(&x, &y)| assert!((x - y).abs() < 1e-17));

        let m_ee = 0.73;
        let m_load = 0.0;
        let f_x_cee = [-0.01, 0., -0.03];
        let i_ee = [0.001, 0.0, 0.0, 0.0, 0.0025, 0.0, 0.0, 0.0, 0.0017];
        let i_load = [0.001, 0.0, 0.0, 0.0, 0.025, 0.0, 0.0, 0.0, 0.3];
        let f_x_cload = [0.01, -0.2, 0.03];
        let m_total = m_ee + m_load;
        let f_x_ctotal = combine_center_of_mass(m_ee, f_x_cee, m_load, f_x_cload);
        let i_total = combine_inertia_tensor(
            m_ee, f_x_cee, i_ee, m_load, f_x_cload, i_load, m_total, f_x_ctotal,
        );
        let expected = i_ee;
        i_total
            .iter()
            .zip(expected.iter())
            .take(3)
            .for_each(|(&x, &y)| assert!((x - y).abs() < 1e-17));
    }
}
