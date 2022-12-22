// Copyright (c) 2021 Marco Boneberger
// Licensed under the EUPL-1.2-or-later
use clap::Parser;
use core::f64::consts::PI;
use franka::exception::FrankaResult;
use franka::model::Frame;
use franka::robot::control_types::Torques;
use franka::robot::{Robot, FR3};
use franka::utils::{array_to_isometry, Matrix6x7, Vector7};
use franka::RobotState;
use franka::{Matrix7, RobotModel};
use nalgebra::{Matrix3, Matrix6, Matrix6x1, Quaternion, UnitQuaternion, Vector3, U1, U3};
use std::sync::mpsc::channel;
use std::time::Duration;

/// An example where one robot is guided by the user and the other robot acts as a mirror. In this
/// case mirror does not mean equal joint positions. Instead, it acts like a real physical mirror that
/// stands in front of the robot (mirrored cartesian poses). Hand guide the end-effector of the user robot
/// and see how the other robot mirrors this pose.
/// WARNING: Before executing this example, make sure there is enough space in between the robots.
#[derive(Parser, Debug)]
#[clap(author, version, name = "mirror_robot")]
struct CommandLineArguments {
    /// IP-Address or hostname of the robot which the user can hand guide
    #[clap(long)]
    pub franka_ip_user: String,
    /// IP-Address or hostname of the robot which mirrors the movement
    #[clap(long)]
    pub franka_ip_mirror: String,
}

const NULLSPACE_TORQUE_SCALING: f64 = 5.;
fn main() -> FrankaResult<()> {
    let args = CommandLineArguments::parse();
    let translational_stiffness = 400.;
    let rotational_stiffness = 50.;

    let mut stiffness: Matrix6<f64> = Matrix6::zeros();
    let mut damping: Matrix6<f64> = Matrix6::zeros();
    {
        let mut top_left_corner = stiffness.fixed_slice_mut::<U3, U3>(0, 0);
        top_left_corner.copy_from(&(Matrix3::identity() * translational_stiffness));
        let mut top_left_corner = damping.fixed_slice_mut::<U3, U3>(0, 0);
        top_left_corner.copy_from(&(2. * f64::sqrt(translational_stiffness) * Matrix3::identity()));
    }
    {
        let mut bottom_right_corner = stiffness.fixed_slice_mut::<U3, U3>(3, 3);
        bottom_right_corner.copy_from(&(Matrix3::identity() * rotational_stiffness));
        let mut bottom_right_corner = damping.fixed_slice_mut::<U3, U3>(3, 3);
        bottom_right_corner
            .copy_from(&(2. * f64::sqrt(rotational_stiffness) * Matrix3::identity()));
    }
    let mut robot_user = FR3::new(args.franka_ip_user.as_str(), None, None)?;
    let mut robot_mirror = FR3::new(args.franka_ip_mirror.as_str(), None, None)?;
    let model = robot_mirror.load_model(true)?;
    robot_mirror.set_collision_behavior(
        [100.; 7], [100.; 7], [100.; 7], [100.; 7], [100.; 6], [100.; 6], [100.; 6], [100.; 6],
    )?;
    robot_mirror.set_joint_impedance([3000., 3000., 3000., 2500., 2500., 2000., 2000.])?;
    robot_mirror.set_cartesian_impedance([3000., 3000., 3000., 300., 300., 300.])?;
    robot_user.set_collision_behavior(
        [100.; 7], [100.; 7], [100.; 7], [100.; 7], [100.; 6], [100.; 6], [100.; 6], [100.; 6],
    )?;
    robot_user.set_joint_impedance([3000., 3000., 3000., 2500., 2500., 2000., 2000.])?;
    robot_user.set_cartesian_impedance([3000., 3000., 3000., 300., 300., 300.])?;
    let initial_state = robot_user.read_once()?;
    let initial_transform = array_to_isometry(&initial_state.O_T_EE);
    let mut desired_state = initial_state.O_T_EE;
    let mut position_d = initial_transform.translation.vector;
    let mut orientation_d = initial_transform.rotation;
    let (sender, receiver) = channel();

    println!(
        "WARNING: Collision thresholds are set to high values. \
             Make sure you have both user stops at hand!"
    );
    println!("After starting try to guide the user robot by hand and see how the other reacts.");
    println!("Press Enter to continue...");
    std::io::stdin().read_line(&mut String::new()).unwrap();
    let q_goal = [0., -PI / 4., 0., -3. * PI / 4., 0., PI / 2., PI / 4.];
    let thread = std::thread::spawn(|| {
        let q_goal = [0., -PI / 4., 0., -3. * PI / 4., 0., PI / 2., PI / 4.];
        robot_user.joint_motion(0.1, &q_goal).unwrap();
        robot_user
    });

    robot_mirror.joint_motion(0.1, &q_goal)?;
    let mut robot_user = thread.join().unwrap();

    std::thread::spawn(move || {
        robot_user.control_torques(
            move |state, _step| -> Torques {
                sender.send(state.O_T_EE).unwrap();
                Torques::new([0.; 7])
            },
            None,
            None,
        )
    });

    robot_mirror.control_torques(
        |state: &RobotState, _step: &Duration| -> Torques {
            let home: Vector7 = q_goal.into();
            let coriolis: Vector7 = model.coriolis_from_state(&state).into();
            let jacobian_array = model.zero_jacobian_from_state(&Frame::EndEffector, &state);
            let jacobian = Matrix6x7::from_column_slice(&jacobian_array);
            let q = Vector7::from_column_slice(&state.q);
            let dq = Vector7::from_column_slice(&state.dq);
            let transform = array_to_isometry(&state.O_T_EE);
            let position = transform.translation.vector;
            let mut orientation = *transform.rotation.quaternion();

            desired_state = receiver
                .recv_timeout(Duration::from_micros(100))
                .unwrap_or(desired_state);
            let desired_transform = array_to_isometry(&desired_state);
            position_d = desired_transform.translation.vector;
            position_d.y = -position_d.y;
            orientation_d = desired_transform.rotation;
            let quaternion = Quaternion::<f64>::new(
                orientation_d.w,
                -orientation_d.i,
                orientation_d.j,
                -orientation_d.k,
            );
            orientation_d = UnitQuaternion::from_quaternion(quaternion);
            let mut error: Matrix6x1<f64> = Matrix6x1::<f64>::zeros();
            {
                let mut error_head = error.fixed_slice_mut::<U3, U1>(0, 0);
                error_head.set_column(0, &(position - position_d));
            }

            if orientation_d.coords.dot(&orientation.coords) < 0. {
                orientation.coords = -orientation.coords;
            }
            let orientation = UnitQuaternion::new_normalize(orientation);
            let error_quaternion: UnitQuaternion<f64> = orientation.inverse() * orientation_d;
            {
                let mut error_tail = error.fixed_slice_mut::<U3, U1>(3, 0);
                error_tail.copy_from(
                    &-(transform.rotation.to_rotation_matrix()
                        * Vector3::new(error_quaternion.i, error_quaternion.j, error_quaternion.k)),
                );
            }
            let nullspace_projection = Matrix7::identity()
                - jacobian.transpose() * jacobian.transpose().pseudo_inverse(0.001).unwrap();
            let tau_nullspace = nullspace_projection * NULLSPACE_TORQUE_SCALING * (home - q);
            let tau_task: Vector7 =
                jacobian.transpose() * (-stiffness * error - damping * (jacobian * dq));
            let tau_d: Vector7 = tau_task + tau_nullspace + coriolis;

            tau_d.into()
        },
        None,
        None,
    )
}
