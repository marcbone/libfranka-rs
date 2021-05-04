// Copyright (c) 2021 Marco Boneberger
// Licensed under the EUPL-1.2-or-later

use franka::Frame;
use franka::FrankaResult;
use franka::Robot;
use franka::RobotState;
use franka::Torques;
use franka::{array_to_isometry, Matrix6x7, Vector7};
use nalgebra::{Matrix3, Matrix6, Matrix6x1, UnitQuaternion, Vector3, U1, U3};
use std::time::Duration;
use structopt::StructOpt;

///An example showing a simple cartesian impedance controller without inertia shaping
/// that renders a spring damper system where the equilibrium is the initial configuration.
/// After starting the controller try to push the robot around and try different stiffness levels.
///
/// WARNING collision thresholds are set to high values. Make sure you have the user stop at hand!
#[derive(StructOpt, Debug)]
#[structopt(name = "cartesian impedance_control")]
struct CommandLineArguments {
    /// IP-Address or hostname of the robot
    #[structopt()]
    pub franka_ip: String,
}

fn main() -> FrankaResult<()> {
    let args = CommandLineArguments::from_args();
    let translational_stiffness = 150.;
    let rotational_stiffness = 10.;

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
    let mut robot = Robot::new(args.franka_ip.as_str(), None, None)?;
    let model = robot.load_model(true)?;

    // Set additional parameters always before the control loop, NEVER in the control loop!
    // Set collision behavior.
    robot.set_collision_behavior(
        [100.; 7], [100.; 7], [100.; 7], [100.; 7], [100.; 6], [100.; 6], [100.; 6], [100.; 6],
    )?;
    robot.set_joint_impedance([3000., 3000., 3000., 2500., 2500., 2000., 2000.])?;
    robot.set_cartesian_impedance([3000., 3000., 3000., 300., 300., 300.])?;
    let initial_state = robot.read_once()?;
    let initial_transform = array_to_isometry(&initial_state.O_T_EE);
    let position_d = initial_transform.translation.vector;
    let orientation_d = initial_transform.rotation;

    println!(
        "WARNING: Collision thresholds are set to high values. \
             Make sure you have the user stop at hand!"
    );
    println!("After starting try to push the robot and see how it reacts.");
    println!("Press Enter to continue...");
    std::io::stdin().read_line(&mut String::new()).unwrap();
    let result = robot.control_torques(
        |state: &RobotState, _step: &Duration| -> Torques {
            let coriolis: Vector7 = model.coriolis_from_state(&state).into();
            let jacobian_array = model.zero_jacobian_from_state(&Frame::kEndEffector, &state);
            let jacobian = Matrix6x7::from_column_slice(&jacobian_array);
            let _q = Vector7::from_column_slice(&state.q);
            let dq = Vector7::from_column_slice(&state.dq);
            let transform = array_to_isometry(&state.O_T_EE);
            let position = transform.translation.vector;
            let mut orientation = *transform.rotation.quaternion();

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
            let tau_task: Vector7 =
                jacobian.transpose() * (-stiffness * error - damping * (jacobian * dq));
            let tau_d: Vector7 = tau_task + coriolis;

            tau_d.into()
        },
        None,
        None,
    );

    match result {
        Ok(_) => Ok(()),
        Err(e) => {
            eprintln!("{}", e);
            Ok(())
        }
    }
}
