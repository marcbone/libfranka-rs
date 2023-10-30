// Copyright (c) 2021 Marco Boneberger
// Licensed under the EUPL-1.2-or-later

//! Contains functions for limiting the rate of torques, Cartesian pose, Cartesian velocity,
//! joint position and joint velocity.

use nalgebra::{
    Isometry3, Matrix3, Matrix3x1, Matrix6x1, Rotation3, Translation3, UnitQuaternion, Vector3,
};

use crate::robot::control_tools::is_homogeneous_transformation;
use crate::utils::array_to_isometry;

/// Sample time constant
pub const DELTA_T: f64 = 1e-3;
///Epsilon value for checking limits
pub const LIMIT_EPS: f64 = 1e-3;
/// Epsilon value for limiting Cartesian accelerations/jerks or not
pub const NORM_EPS: f64 = f64::EPSILON;
/// Factor for the definition of rotational limits using the Cartesian Pose interface
const FACTOR_CARTESIAN_ROTATION_POSE_INTERFACE: f64 = 0.99;

/// The rate limiter can limit the rate of torques, Cartesian pose, Cartesian velocity,
/// joint position and joint velocity. The rate limiter trait is implemented for [Fr3](crate::Fr3#impl-RateLimiter-for-Fr3) and [Panda](crate::Panda#impl-RateLimiter-for-Panda)
/// in different ways.
pub trait RateLimiter {
    /// Defines if the rate limiter is used when `None` is passed to the `limit_rate` argument in
    /// control methods like [`control_joint_positions`](crate::robot::Robot::control_joint_positions)
    const RATE_LIMITING_ON_PER_DEFAULT: bool;
    /// Number of packets lost considered for the definition of velocity limits.
    /// When a packet is lost, FCI assumes a constant acceleration model
    const TOL_NUMBER_PACKETS_LOST: f64;
    /// Maximum torque rate
    const MAX_TORQUE_RATE: [f64; 7] = [1000. - LIMIT_EPS; 7];
    /// Maximum joint jerk
    const MAX_JOINT_JERK: [f64; 7];
    /// Maximum joint acceleration
    const MAX_JOINT_ACCELERATION: [f64; 7];
    /// Maximum translational jerk
    const MAX_TRANSLATIONAL_JERK: f64;
    /// Maximum translational acceleration
    const MAX_TRANSLATIONAL_ACCELERATION: f64;
    /// Maximum translational velocity
    const MAX_TRANSLATIONAL_VELOCITY: f64;
    /// Maximum rotational jerk
    const MAX_ROTATIONAL_JERK: f64;
    /// Maximum rotational acceleration
    const MAX_ROTATIONAL_ACCELERATION: f64;
    /// Maximum rotational velocity
    const MAX_ROTATIONAL_VELOCITY: f64;
    /// Maximum elbow jerk
    const MAX_ELBOW_JERK: f64;
    /// Maximum elbow acceleration
    const MAX_ELBOW_ACCELERATION: f64;
    /// Maximum elbow velocity
    const MAX_ELBOW_VELOCITY: f64;
    /// Computes the maximum joint velocity based on joint position
    /// # Arguments
    /// * `q` - joint position
    /// # Return
    /// Upper limits of joint velocity at the given joint position.
    fn compute_upper_limits_joint_velocity(q: &[f64; 7]) -> [f64; 7];
    /// Computes the minimum joint velocity based on joint position
    /// # Arguments
    /// * `q` - joint position
    /// # Return
    /// Lower limits of joint velocity at the given joint position.
    fn compute_lower_limits_joint_velocity(q: &[f64; 7]) -> [f64; 7];
}

/// Limits the rate of a desired joint position considering the limits provided.
/// # Note
/// FCI filters must be deactivated to work properly.
/// # Arguments
/// * `max_velocity` - Per-joint maximum allowed velocity.
/// * `min_velocity` - Per-joint minimum allowed velocity.
/// * `max_acceleration` - Per-joint maximum allowed acceleration.
/// * `max_jerk` - Per-joint maximum allowed jerk.
/// * `commanded_positions` - Commanded joint positions of the current time step.
/// * `last_commanded_velocities` - Commanded joint positions of the previous time step.
/// * `last_commanded_velocities` - Commanded joint velocities of the previous time step.
/// * `last_commanded_accelerations` - Commanded joint accelerations of the previous time step.
/// # Panics
/// * if commanded_positions are infinite or NaN.
/// # Return
/// Rate-limited vector of desired joint positions.
#[allow(clippy::too_many_arguments)]
pub fn limit_rate_joint_positions(
    max_velocity: &[f64; 7],
    min_velocity: &[f64; 7],
    max_acceleration: &[f64; 7],
    max_jerk: &[f64; 7],
    commanded_positions: &[f64; 7],
    last_commanded_positions: &[f64; 7],
    last_commanded_velocities: &[f64; 7],
    last_commanded_accelerations: &[f64; 7],
) -> [f64; 7] {
    commanded_positions
        .iter()
        .for_each(|x| assert!(x.is_finite()));
    let mut limited_commanded_positions = [0.; 7];
    for i in 0..7 {
        limited_commanded_positions[i] = limit_rate_position(
            max_velocity[i],
            min_velocity[i],
            max_acceleration[i],
            max_jerk[i],
            commanded_positions[i],
            last_commanded_positions[i],
            last_commanded_velocities[i],
            last_commanded_accelerations[i],
        );
    }
    limited_commanded_positions
}

/// Limits the rate of a desired joint position considering the limits provided.
/// # Note
/// FCI filters must be deactivated to work properly.
/// # Arguments
/// * `max_velocity` - Maximum allowed velocity.
/// * `min_velocity` - Minimum allowed velocity.
/// * `max_acceleration` - Maximum allowed acceleration.
/// * `max_jerk` - Maximum allowed jerk.
/// * `commanded_position` - Commanded joint position of the current time step.
/// * `last_commanded_velocity` - Commanded joint velocity of the previous time step.
/// * `last_commanded_acceleration` - Commanded joint acceleration of the previous time step.
/// # Panics
/// * if commanded_values are infinite or NaN.
/// # Return
/// Rate-limited desired joint position.
#[allow(clippy::too_many_arguments)]
pub fn limit_rate_position(
    max_velocity: f64,
    min_velocity: f64,
    max_acceleration: f64,
    max_jerk: f64,
    commanded_position: f64,
    last_commanded_position: f64,
    last_commanded_velocity: f64,
    last_commanded_acceleration: f64,
) -> f64 {
    assert!(commanded_position.is_finite());
    last_commanded_position
        + limit_rate_velocity(
            max_velocity,
            min_velocity,
            max_acceleration,
            max_jerk,
            (commanded_position - last_commanded_position) / DELTA_T,
            last_commanded_velocity,
            last_commanded_acceleration,
        ) * DELTA_T
}

/// Limits the rate of a desired joint velocity considering the limits provided.
/// # Note
/// FCI filters must be deactivated to work properly.
/// # Arguments
/// * `max_velocity` - Maximum allowed velocity.
/// * `min_velocity` - Minimum allowed velocity.
/// * `max_acceleration` - Maximum allowed acceleration.
/// * `max_jerk` - Maximum allowed jerk.
/// * `commanded_velocity` - Commanded joint velocity of the current time step.
/// * `last_commanded_velocity` - Commanded joint velocity of the previous time step.
/// * `last_commanded_acceleration` - Commanded joint acceleration of the previous time step.
/// # Panics
/// * if commanded_values are infinite or NaN.
/// # Return
/// Rate-limited desired joint velocity.
fn limit_rate_velocity(
    max_velocity: f64,
    min_velocity: f64,
    max_acceleration: f64,
    max_jerk: f64,
    commanded_velocity: f64,
    last_commanded_velocity: f64,
    last_commanded_acceleration: f64,
) -> f64 {
    assert!(commanded_velocity.is_finite());
    let commanded_jerk = (((commanded_velocity - last_commanded_velocity) / DELTA_T)
        - last_commanded_acceleration)
        / DELTA_T;
    let commanded_acceleration = last_commanded_acceleration
        + f64::max(f64::min(commanded_jerk, max_jerk), -max_jerk) * DELTA_T;
    let safe_max_acceleration = f64::min(
        (max_jerk / max_acceleration) * (max_velocity - last_commanded_velocity),
        max_acceleration,
    );
    let safe_min_acceleration = f64::max(
        (max_jerk / max_acceleration) * (min_velocity - last_commanded_velocity),
        -max_acceleration,
    );
    last_commanded_velocity
        + f64::max(
            f64::min(commanded_acceleration, safe_max_acceleration),
            safe_min_acceleration,
        ) * DELTA_T
}

/// Limits the rate of an input vector of per-joint commands considering the maximum allowed
/// time derivatives.
/// # Note
/// FCI filters must be deactivated to work properly.
/// # Arguments
/// * `max_derivatives` - Per-joint maximum allowed time derivative.
/// * `commanded_values` - Commanded values of the current time step.
/// * `last_commanded_values` - Commanded values of the previous time step.
/// # Panics
/// * if commanded_values are infinite or NaN.
/// # Return
/// Rate-limited vector of desired values.
pub fn limit_rate_torques(
    max_derivatives: &[f64; 7],
    commanded_values: &[f64; 7],
    last_commanded_values: &[f64; 7],
) -> [f64; 7] {
    commanded_values.iter().for_each(|x| assert!(x.is_finite()));
    let mut limited_values = [0.; 7];
    for i in 0..7 {
        let commanded_derivative = (commanded_values[i] - last_commanded_values[i]) / DELTA_T;
        limited_values[i] = last_commanded_values[i]
            + f64::max(
                f64::min(commanded_derivative, max_derivatives[i]),
                -max_derivatives[i],
            ) * DELTA_T;
    }
    limited_values
}

/// Limits the rate of a desired joint velocity considering the limits provided.
/// # Note
/// FCI filters must be deactivated to work properly.
/// # Arguments
/// * `max_velocity` - Per-joint maximum allowed velocity.
/// * `min_velocity` - Per-joint minimum allowed velocity.
/// * `max_acceleration` - Per-joint maximum allowed acceleration.
/// * `max_jerk` - Per-joint maximum allowed jerk.
/// * `commanded_velocities` - Commanded joint velocity of the current time step.
/// * `last_commanded_velocities` - Commanded joint velocities of the previous time step.
/// * `last_commanded_accelerations` - Commanded joint accelerations of the previous time step.
/// # Panics
/// * if commanded_velocities are infinite or NaN.
/// # Return
/// Rate-limited vector of desired joint velocities.
pub fn limit_rate_joint_velocities(
    max_velocity: &[f64; 7],
    min_velocity: &[f64; 7],
    max_acceleration: &[f64; 7],
    max_jerk: &[f64; 7],
    commanded_velocities: &[f64; 7],
    last_commanded_velocities: &[f64; 7],
    last_commanded_accelerations: &[f64; 7],
) -> [f64; 7] {
    commanded_velocities
        .iter()
        .for_each(|x| assert!(x.is_finite()));
    let mut limited_commanded_velocities = [0.; 7];
    for i in 0..7 {
        limited_commanded_velocities[i] = limit_rate_velocity(
            max_velocity[i],
            min_velocity[i],
            max_acceleration[i],
            max_jerk[i],
            commanded_velocities[i],
            last_commanded_velocities[i],
            last_commanded_accelerations[i],
        );
    }
    limited_commanded_velocities
}

/// Limits the rate of a desired Cartesian pose considering the limits provided.
/// # Note
/// FCI filters must be deactivated to work properly.
/// # Arguments
/// * `max_translational_velocity` - Maximum translational velocity.
/// * `max_translational_acceleration` - Maximum translational acceleration.
/// * `max_translational_jerk` - Maximum translational jerk.
/// * `max_rotational_velocity` - Maximum rotational velocity.
/// * `max_rotational_acceleration` - Maximum rotational acceleration.
/// * `max_rotational_jerk` - Maximum rotational jerk.
/// * `O_T_EE_c` - Commanded pose of the current time step.
/// * `last_O_T_EE_c` - Commanded pose of the previous time step.
/// * `last_O_dP_EE_c` - Commanded end effector twist of the previous time step.
/// * `last_O_ddP_EE_c` - Commanded end effector acceleration of the previous time step.
/// # Panics
/// * if an element of O_T_EE_c is infinite or NaN.
/// # Return
/// Rate-limited desired pose.
#[allow(non_snake_case, clippy::too_many_arguments)]
pub fn limit_rate_cartesian_pose(
    max_translational_velocity: f64,
    max_translational_acceleration: f64,
    max_translational_jerk: f64,
    max_rotational_velocity: f64,
    max_rotational_acceleration: f64,
    max_rotational_jerk: f64,
    O_T_EE_c: &[f64; 16],
    last_O_T_EE_c: &[f64; 16],
    last_O_dP_EE_c: &[f64; 6],
    last_O_ddP_EE_c: &[f64; 6],
) -> [f64; 16] {
    O_T_EE_c.iter().for_each(|x| assert!(x.is_finite()));
    assert!(is_homogeneous_transformation(O_T_EE_c));

    let commanded_pose = array_to_isometry(O_T_EE_c);
    let mut limited_commanded_pose = Isometry3::<f64>::identity();
    let last_commanded_pose = array_to_isometry(last_O_T_EE_c);

    let dx_head =
        (commanded_pose.translation.vector - last_commanded_pose.translation.vector) / DELTA_T;

    let mut rot_diff: Rotation3<f64> = commanded_pose.rotation.to_rotation_matrix()
        * last_commanded_pose
            .rotation
            .to_rotation_matrix()
            .transpose();
    rot_diff.renormalize();
    let dx_tail = rot_diff.scaled_axis() / DELTA_T;

    let mut commanded_O_dP_EE_c = [0.; 6];
    for i in 0..3 {
        commanded_O_dP_EE_c[i] = dx_head[i];
    }
    for i in 0..3 {
        commanded_O_dP_EE_c[i + 3] = dx_tail[i];
    }
    commanded_O_dP_EE_c = limit_rate_cartesian_velocity(
        max_translational_velocity,
        max_translational_acceleration,
        max_translational_jerk,
        FACTOR_CARTESIAN_ROTATION_POSE_INTERFACE * max_rotational_velocity,
        FACTOR_CARTESIAN_ROTATION_POSE_INTERFACE * max_rotational_acceleration,
        FACTOR_CARTESIAN_ROTATION_POSE_INTERFACE * max_rotational_jerk,
        &commanded_O_dP_EE_c,
        last_O_dP_EE_c,
        last_O_ddP_EE_c,
    );
    let dx: Matrix6x1<f64> = Matrix6x1::<f64>::from_column_slice(&commanded_O_dP_EE_c);
    limited_commanded_pose.translation = Translation3::from(
        last_commanded_pose.translation.vector + Vector3::new(dx[0], dx[1], dx[2]) * DELTA_T,
    );
    limited_commanded_pose.rotation = last_commanded_pose.rotation;
    let dx_tail = dx.remove_row(0).remove_row(0).remove_row(0);
    if dx_tail.norm() > NORM_EPS {
        let w_norm = dx_tail.normalize();
        let theta = DELTA_T * dx_tail.norm();
        let omega_skew = Matrix3::new(
            0., -w_norm[2], w_norm[1], w_norm[2], 0., -w_norm[0], -w_norm[1], w_norm[0], 0.,
        );
        let R = Matrix3::identity()
            + f64::sin(theta) * omega_skew
            + (1. - f64::cos(theta)) * (omega_skew * omega_skew);
        limited_commanded_pose.rotation =
            UnitQuaternion::from_matrix(&(R * last_commanded_pose.rotation.to_rotation_matrix()));
    }
    let mut limited_values = [0.; 16];
    for (i, &x) in limited_commanded_pose.to_homogeneous().iter().enumerate() {
        limited_values[i] = x;
    }
    limited_values
}

/// Limits the rate of a desired Cartesian velocity considering the limits provided.
/// # Note
/// FCI filters must be deactivated to work properly.
/// # Arguments
/// * `max_translational_velocity` - Maximum translational velocity.
/// * `max_translational_acceleration` - Maximum translational acceleration.
/// * `max_translational_jerk` - Maximum translational jerk.
/// * `max_rotational_velocity` - Maximum rotational velocity.
/// * `max_rotational_acceleration` - Maximum rotational acceleration.
/// * `max_rotational_jerk` - Maximum rotational jerk.
/// * `O_dP_EE_c` - Commanded pose of the current time step.
/// * `last_O_dP_EE_c` - Commanded end effector twist of the previous time step.
/// * `last_O_ddP_EE_c` - Commanded end effector acceleration of the previous time step.
/// # Panics
/// * if an element of O_dP_EE_c is infinite or NaN.
/// # Return
/// Rate-limited desired end effector twist.
#[allow(non_snake_case, clippy::too_many_arguments)]
pub fn limit_rate_cartesian_velocity(
    max_translational_velocity: f64,
    max_translational_acceleration: f64,
    max_translational_jerk: f64,
    max_rotational_velocity: f64,
    max_rotational_acceleration: f64,
    max_rotational_jerk: f64,
    O_dP_EE_c: &[f64; 6],
    last_O_dP_EE_c: &[f64; 6],
    last_O_ddP_EE_c: &[f64; 6],
) -> [f64; 6] {
    O_dP_EE_c.iter().for_each(|x| assert!(x.is_finite()));
    let dx: Matrix6x1<f64> = Matrix6x1::from_column_slice(O_dP_EE_c);
    let last_dx = Matrix6x1::from_column_slice(last_O_dP_EE_c);
    let last_ddx = Matrix6x1::from_column_slice(last_O_ddP_EE_c);
    let dx_head = limit_rate_single_cartesian_velocity(
        max_translational_velocity,
        max_translational_acceleration,
        max_translational_jerk,
        &Vector3::from(dx.fixed_view::<3, 1>(0, 0)),
        &Vector3::from(last_dx.fixed_view::<3, 1>(0, 0)),
        &Vector3::from(last_ddx.fixed_view::<3, 1>(0, 0)),
    );
    let dx_tail = limit_rate_single_cartesian_velocity(
        max_rotational_velocity,
        max_rotational_acceleration,
        max_rotational_jerk,
        &Vector3::from(dx.fixed_view::<3, 1>(3, 0)),
        &Vector3::from(last_dx.fixed_view::<3, 1>(3, 0)),
        &Vector3::from(last_ddx.fixed_view::<3, 1>(3, 0)),
    );

    let mut limited_values = [0.; 6];
    for i in 0..3 {
        limited_values[i] = dx_head[i];
    }
    for i in 0..3 {
        limited_values[i + 3] = dx_tail[i];
    }
    limited_values
}

fn limit_rate_single_cartesian_velocity(
    max_velocity: f64,
    max_acceleration: f64,
    max_jerk: f64,
    commanded_velocity: &Vector3<f64>,
    last_commanded_velocity: &Vector3<f64>,
    last_commanded_acceleration: &Vector3<f64>,
) -> Vector3<f64> {
    let commanded_jerk: Vector3<f64> = (((commanded_velocity - last_commanded_velocity) / DELTA_T)
        - last_commanded_acceleration)
        / DELTA_T;
    let mut commanded_acceleration = last_commanded_acceleration.clone_owned();
    if commanded_jerk.norm() > NORM_EPS {
        commanded_acceleration += commanded_jerk.normalize()
            * f64::max(f64::min(commanded_jerk.norm(), max_jerk), -max_jerk)
            * DELTA_T;
    }
    let unit_commanded_acceleration: Matrix3x1<f64> = commanded_acceleration.normalize();
    let dot_product = unit_commanded_acceleration.dot(last_commanded_velocity);
    let distance_to_max_velocity = -dot_product
        + f64::sqrt(
            f64::powf(dot_product, 2.) - last_commanded_velocity.norm_squared()
                + f64::powf(max_velocity, 2.),
        );
    let safe_max_acceleration = f64::min(
        (max_jerk / max_acceleration) * distance_to_max_velocity,
        max_acceleration,
    );
    let mut limited_commanded_velocity = last_commanded_velocity.clone_owned();
    if commanded_acceleration.norm() > NORM_EPS {
        limited_commanded_velocity += unit_commanded_acceleration
            * f64::min(commanded_acceleration.norm(), safe_max_acceleration)
            * DELTA_T;
    }
    limited_commanded_velocity
}

#[cfg(test)]
mod tests {
    use crate::{Fr3, Panda};
    use nalgebra::{Translation3, Unit, UnitQuaternion, Vector3, Vector6};
    use num_traits::Float;

    use crate::robot::rate_limiting::{limit_rate_cartesian_pose, RateLimiter, DELTA_T, LIMIT_EPS};
    use crate::utils::array_to_isometry;

    fn integrate_one_sample(last_pose: &[f64; 16], dx: &[f64; 6], delta_t: f64) -> [f64; 16] {
        let mut pose = array_to_isometry(last_pose);
        let twist_vector: Vector3<f64> = Vector3::new(dx[3], dx[4], dx[5]);
        pose.rotation = UnitQuaternion::from_axis_angle(
            &Unit::new_normalize(twist_vector),
            twist_vector.norm() * delta_t,
        ) * pose.rotation;
        pose.translation = Translation3::from(
            pose.translation.vector + Translation3::new(dx[0], dx[1], dx[2]).vector * delta_t,
        );

        let mut pose_after_integration = [0.; 16];
        for (i, &x) in pose.to_homogeneous().iter().enumerate() {
            pose_after_integration[i] = x;
        }
        pose_after_integration
    }

    fn integrate_one_sample_six(
        last_value: &[f64; 6],
        derivative: &[f64; 6],
        delta_t: f64,
    ) -> [f64; 6] {
        let mut result = [0.; 6];
        for i in 0..6 {
            result[i] = last_value[i] + derivative[i] * delta_t
        }
        result
    }

    fn generate_values_into_limits(
        last_cmd_values: &[f64; 6],
        max_translational_derivative: f64,
        max_rotational_derivative: f64,
        eps: f64,
        delta_t: f64,
    ) -> [f64; 6] {
        let mut result = *last_cmd_values;
        result[0] += (max_translational_derivative
            - f64::min(
                f64::max(f64::abs(eps), 0.),
                2. * max_translational_derivative,
            ))
            * delta_t;
        result[3] += (max_rotational_derivative
            - f64::min(
                f64::max(f64::abs(eps), 0.),
                2. * max_translational_derivative,
            ))
            * delta_t;
        result
    }

    fn generate_values_outside_limits(
        last_cmd_values: &[f64; 6],
        max_translational_derivative: f64,
        max_rotational_derivative: f64,
        eps: f64,
        delta_t: f64,
    ) -> [f64; 6] {
        let mut result = *last_cmd_values;
        result[0] += (max_translational_derivative + f64::max(f64::abs(eps), LIMIT_EPS)) * delta_t;
        result[3] += (max_rotational_derivative + f64::max(f64::abs(eps), LIMIT_EPS)) * delta_t;
        result
    }

    fn differentiate_one_sample(
        value: &[f64; 16],
        last_value: &[f64; 16],
        delta_t: f64,
    ) -> [f64; 6] {
        let pose = array_to_isometry(value);
        let last_pose = array_to_isometry(last_value);
        let head = (pose.translation.vector - last_pose.translation.vector) / delta_t;
        let delta_rotation = pose.rotation * last_pose.rotation.inverse();
        let scaled_axis = delta_rotation.scaled_axis() / delta_t;
        let mut res = [0.; 6];
        for i in 0..3 {
            res[i] = head[i];
        }
        for i in 0..3 {
            res[i + 3] = scaled_axis[i];
        }
        res
    }

    #[allow(non_snake_case)]
    fn violates_rate_limits(
        max_translational_dx: f64,
        max_translational_ddx: f64,
        max_translational_dddx: f64,
        max_rotational_dx: f64,
        max_rotational_ddx: f64,
        max_rotational_dddx: f64,
        cmd_dx: &[f64; 6],
        O_dP_EE_c: &[f64; 6],
        O_dd_P_EE_c: &[f64; 6],
        delta_t: f64,
    ) -> bool {
        let dx: Vector6<f64> = Vector6::from_column_slice(cmd_dx);
        let last_dx: Vector6<f64> = Vector6::from_column_slice(O_dP_EE_c);
        let last_ddx: Vector6<f64> = Vector6::from_column_slice(O_dd_P_EE_c);
        let ddx = (dx - last_dx) / delta_t;
        let dddx = (ddx - last_ddx) / delta_t;
        let violates_limits = |desired_value: f64, max_value: f64| desired_value.abs() > max_value;

        violates_limits(dx.fixed_rows::<3>(0).norm(), max_translational_dx)
            || violates_limits(ddx.fixed_rows::<3>(0).norm(), max_translational_ddx)
            || violates_limits(dddx.fixed_rows::<3>(0).norm(), max_translational_dddx)
            || violates_limits(dx.fixed_rows::<3>(3).norm(), max_rotational_dx)
            || violates_limits(ddx.fixed_rows::<3>(3).norm(), max_rotational_ddx)
            || violates_limits(dddx.fixed_rows::<3>(3).norm(), max_rotational_dddx)
    }
    #[test]
    #[allow(non_snake_case)]
    fn limit_rate_cartesian_pose_nan_test() {
        let O_T_EE_c = [
            1.0,
            -0.000000011046552201160267,
            0.000000008312911920110592,
            0.0,
            -0.000000011046552077288223,
            -0.9999999999999999,
            -0.000000014901161362226844,
            0.0,
            0.000000008312912084717049,
            0.000000014901161270397825,
            -0.9999999999999999,
            0.0,
            0.30689056578595225,
            -0.000000003883240449999549,
            0.486882056335292,
            1.0,
        ];
        let last_O_T_EE_c = [
            0.9999903734042683,
            -0.000000011046444370332864,
            0.00000000831299559264306,
            0.0,
            -0.000000011046444370332864,
            -0.9999903734042683,
            -0.0000000021131709957245164,
            0.0,
            0.000000008313075597526054,
            0.0000000021131912467324254,
            -0.9999999999999999,
            0.0,
            0.30689056578595225,
            -0.000000003883240449999549,
            0.486882056335292,
            1.0,
        ];
        let O_dP_EE_c = [
            0.0,
            0.0,
            0.0,
            0.000018516789037693124,
            0.0000000000926701758470142,
            -0.0000000000487067383271437,
        ];
        let O_ddP_EE_c = [
            0.0,
            0.0,
            0.0,
            -0.001693525168601239,
            0.00000005362986228972482,
            -0.00000002304731382519481,
        ];

        let out = limit_rate_cartesian_pose(
            Panda::MAX_TRANSLATIONAL_VELOCITY,
            Panda::MAX_TRANSLATIONAL_ACCELERATION,
            Panda::MAX_TRANSLATIONAL_JERK,
            Panda::MAX_ROTATIONAL_VELOCITY,
            Panda::MAX_ROTATIONAL_ACCELERATION,
            Panda::MAX_ROTATIONAL_JERK,
            &O_T_EE_c,
            &last_O_T_EE_c,
            &O_dP_EE_c,
            &O_ddP_EE_c,
        );
        for i in out.iter() {
            assert!(i.is_finite());
        }
    }
    #[test]
    #[allow(non_snake_case)]
    fn limit_rate_cartesian_pose_constant_pose_test() {
        let O_T_EE_c = [
            1.0,
            -0.000000011046552201160267,
            0.000000008312911920110592,
            0.0,
            -0.000000011046552077288223,
            -0.9999999999999999,
            -0.000000014901161362226844,
            0.0,
            0.000000008312912084717049,
            0.000000014901161270397825,
            -0.9999999999999999,
            0.0,
            0.30689056578595225,
            -0.000000003883240449999549,
            0.486882056335292,
            1.0,
        ];
        let out = limit_rate_cartesian_pose(
            Panda::MAX_TRANSLATIONAL_VELOCITY,
            Panda::MAX_TRANSLATIONAL_ACCELERATION,
            Panda::MAX_TRANSLATIONAL_JERK,
            Panda::MAX_ROTATIONAL_VELOCITY,
            Panda::MAX_ROTATIONAL_ACCELERATION,
            Panda::MAX_ROTATIONAL_JERK,
            &O_T_EE_c,
            &O_T_EE_c,
            &[0.; 6],
            &[0.; 6],
        );
        for i in 0..O_T_EE_c.len() {
            assert!(f64::abs(O_T_EE_c[i] - out[i]) < 1e-15);
        }
    }
    #[test]
    fn limit_rate_cartesian_test() {
        let last_cmd_pose = [
            1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0,
        ];
        let last_cmd_velocity = [0.; 6];
        let last_cmd_acceleration = [0.; 6];
        let max_translational_acceleration = 10.;
        let max_translational_jerk = 100.;
        let max_rotational_acceleration = 5.;
        let max_rotational_jerk = 50.;
        let eps = 1e-2;
        let limits = generate_values_into_limits(
            &last_cmd_acceleration,
            max_translational_jerk,
            max_rotational_jerk,
            eps,
            DELTA_T,
        );
        let twist = integrate_one_sample_six(&last_cmd_velocity, &limits, DELTA_T);
        let cartesian_pose_into_limits_hardcoded = [
            1., 0., 0., 0., 0., 1., 5e-8, 0., 0., -5e-8, 1., 0., 1e-8, 0., 0., 1.,
        ];
        let cartesian_pose_into_limits = integrate_one_sample(&last_cmd_pose, &twist, DELTA_T);
        cartesian_pose_into_limits_hardcoded
            .iter()
            .zip(cartesian_pose_into_limits.iter())
            .for_each(|(&x, &y)| assert!(f64::abs(x - y) < 1e-6));

        let k_no_limit = f64::max_value();
        assert!(!violates_rate_limits(
            k_no_limit,
            k_no_limit,
            max_translational_jerk,
            k_no_limit,
            k_no_limit,
            max_rotational_jerk,
            &differentiate_one_sample(&cartesian_pose_into_limits, &last_cmd_pose, DELTA_T),
            &last_cmd_velocity,
            &last_cmd_acceleration,
            DELTA_T
        ));
        let cartesian_pose_limited = limit_rate_cartesian_pose(
            k_no_limit,
            k_no_limit,
            max_translational_jerk,
            k_no_limit,
            k_no_limit,
            max_rotational_jerk,
            &cartesian_pose_into_limits,
            &last_cmd_pose,
            &last_cmd_velocity,
            &last_cmd_acceleration,
        );
        cartesian_pose_limited
            .iter()
            .zip(cartesian_pose_into_limits.iter())
            .for_each(|(&x, &y)| assert!(f64::abs(x - y) < 1e-6));

        let cartesian_pose_into_limits = integrate_one_sample(
            &last_cmd_pose,
            &generate_values_into_limits(
                &last_cmd_velocity,
                max_translational_acceleration,
                max_rotational_acceleration,
                eps,
                DELTA_T,
            ),
            DELTA_T,
        );
        assert!(!violates_rate_limits(
            k_no_limit,
            max_translational_acceleration,
            k_no_limit,
            k_no_limit,
            max_rotational_acceleration,
            k_no_limit,
            &differentiate_one_sample(&cartesian_pose_into_limits, &last_cmd_pose, DELTA_T),
            &last_cmd_velocity,
            &last_cmd_acceleration,
            DELTA_T
        ));
        let cartesian_pose_limited = limit_rate_cartesian_pose(
            k_no_limit,
            max_translational_acceleration,
            k_no_limit,
            k_no_limit,
            max_rotational_acceleration,
            k_no_limit,
            &cartesian_pose_into_limits,
            &last_cmd_pose,
            &last_cmd_velocity,
            &last_cmd_acceleration,
        );
        cartesian_pose_limited
            .iter()
            .zip(cartesian_pose_into_limits.iter())
            .for_each(|(&x, &y)| assert!(f64::abs(x - y) < 1e-6));

        let cartesian_pose_outside_limits = integrate_one_sample(
            &last_cmd_pose,
            &integrate_one_sample_six(
                &last_cmd_velocity,
                &generate_values_outside_limits(
                    &last_cmd_acceleration,
                    max_translational_jerk,
                    max_rotational_jerk,
                    eps,
                    DELTA_T,
                ),
                DELTA_T,
            ),
            DELTA_T,
        );
        let limited_cartesian_pose = limit_rate_cartesian_pose(
            k_no_limit,
            k_no_limit,
            max_translational_jerk,
            k_no_limit,
            k_no_limit,
            max_rotational_jerk,
            &cartesian_pose_outside_limits,
            &last_cmd_pose,
            &last_cmd_velocity,
            &last_cmd_acceleration,
        );

        assert!(violates_rate_limits(
            k_no_limit,
            k_no_limit,
            max_translational_jerk,
            k_no_limit,
            k_no_limit,
            max_rotational_jerk,
            &differentiate_one_sample(&cartesian_pose_outside_limits, &last_cmd_pose, DELTA_T),
            &last_cmd_velocity,
            &last_cmd_acceleration,
            DELTA_T
        ));
        assert_ne!(cartesian_pose_outside_limits, limited_cartesian_pose);
        assert!(!violates_rate_limits(
            k_no_limit,
            k_no_limit,
            max_translational_jerk,
            k_no_limit,
            k_no_limit,
            max_rotational_jerk,
            &differentiate_one_sample(&limited_cartesian_pose, &last_cmd_pose, DELTA_T),
            &last_cmd_velocity,
            &last_cmd_acceleration,
            DELTA_T
        ));

        let cartesian_pose_outside_limits = integrate_one_sample(
            &last_cmd_pose,
            &generate_values_outside_limits(
                &last_cmd_velocity,
                max_translational_acceleration,
                max_rotational_acceleration,
                eps,
                DELTA_T,
            ),
            DELTA_T,
        );
        let limited_cartesian_pose = limit_rate_cartesian_pose(
            k_no_limit,
            max_translational_acceleration,
            k_no_limit,
            k_no_limit,
            max_rotational_acceleration,
            k_no_limit,
            &cartesian_pose_outside_limits,
            &last_cmd_pose,
            &last_cmd_velocity,
            &last_cmd_acceleration,
        );
        assert!(violates_rate_limits(
            k_no_limit,
            max_translational_acceleration,
            k_no_limit,
            k_no_limit,
            max_rotational_acceleration,
            k_no_limit,
            &differentiate_one_sample(&cartesian_pose_outside_limits, &last_cmd_pose, DELTA_T),
            &last_cmd_velocity,
            &last_cmd_acceleration,
            DELTA_T
        ));
        assert_ne!(cartesian_pose_outside_limits, limited_cartesian_pose);
        assert!(!violates_rate_limits(
            k_no_limit,
            max_translational_acceleration,
            k_no_limit,
            k_no_limit,
            max_rotational_acceleration,
            k_no_limit,
            &differentiate_one_sample(&limited_cartesian_pose, &last_cmd_pose, DELTA_T),
            &last_cmd_velocity,
            &last_cmd_acceleration,
            DELTA_T
        ));
    }
    #[test]
    fn cartesian_pose_integration_and_differentiation_test() {
        let last_cmd_pose = [
            0.0, 1.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0,
        ];
        let last_cmd_velocity = [1.0, 2.0, 3.0, 0.4, 0.5, 0.3];

        let cartesian_pose = integrate_one_sample(&last_cmd_pose, &last_cmd_velocity, DELTA_T);
        println!("{:?}", cartesian_pose);
        let twist = differentiate_one_sample(&cartesian_pose, &last_cmd_pose, DELTA_T);
        println!("{:?}", twist);
        twist
            .iter()
            .zip(last_cmd_velocity.iter())
            .for_each(|(&x, &y)| assert!(f64::abs(x - y) < 1e-6));
    }

    #[test]
    fn position_based_velocity_limit_boundary_check_negative_velocity() {
        let q_lower_limits = [-2.9007, -1.8361, -2.9107, -3.0770, -2.8763, 0.4398, -3.0508];
        let dq_upper_limits = [2.62, 2.62, 2.62, 2.62, 5.26, 4.18, 5.26];
        let dq_lower_limits = [0.; 7];

        let dq_max = Fr3::compute_upper_limits_joint_velocity(&q_lower_limits);
        assert!(dq_max
            .iter()
            .zip(dq_upper_limits.iter())
            .all(|(dq, dq_limit)| dq < dq_limit));

        let dq_min = Fr3::compute_lower_limits_joint_velocity(&q_lower_limits);
        assert!(dq_max
            .iter()
            .zip(dq_lower_limits.iter())
            .all(|(dq, dq_limit)| dq > dq_limit));

        assert!(dq_max.iter().zip(dq_min.iter()).all(|(max, min)| max > min));
    }

    #[test]
    fn position_based_velocity_limit_boundary_check_positive_velocity() {
        let q_upper_limits = [2.9007, 1.8361, 2.9107, -0.1169, 2.8763, 4.6216, 3.0508];
        let dq_upper_limits = [0.; 7];
        let dq_lower_limits = [-2.62, -2.62, -2.62, -2.62, -5.26, -4.18, -5.26];

        let dq_max = Fr3::compute_upper_limits_joint_velocity(&q_upper_limits);
        assert!(dq_max
            .iter()
            .zip(dq_upper_limits.iter())
            .all(|(dq, dq_limit)| dq < dq_limit));

        let dq_min = Fr3::compute_lower_limits_joint_velocity(&q_upper_limits);
        assert!(dq_max
            .iter()
            .zip(dq_lower_limits.iter())
            .all(|(dq, dq_limit)| dq > dq_limit));

        assert!(dq_max.iter().zip(dq_min.iter()).all(|(max, min)| max > min));
    }
}
