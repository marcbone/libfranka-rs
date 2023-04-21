// Copyright (c) 2021 Marco Boneberger
// Licensed under the EUPL-1.2-or-later

use std::f64::consts::PI;
use std::time::Duration;

use clap::Parser;

use franka::{
    CartesianVelocities, FrankaResult, MotionFinished, Panda, RobotState, RobotWrapper, FR3,
};

/// An example showing how to generate a Cartesian velocity motion.
///
/// WARNING: Before executing this example, make sure there is enough space in front of the robot.
#[derive(Parser, Debug)]
#[clap(author, version, name = "generate_cartesian_velocity_motion")]
struct CommandLineArguments {
    /// IP-Address or hostname of the robot
    pub franka_ip: String,
    /// Use this option to run the example on a Panda
    #[clap(short, long, action)]
    pub panda: bool,
}

fn main() -> FrankaResult<()> {
    let args = CommandLineArguments::parse();
    match args.panda {
        true => {
            let robot = Panda::new(args.franka_ip.as_str(), None, None)?;
            generate_motion(robot)
        }
        false => {
            let robot = FR3::new(args.franka_ip.as_str(), None, None)?;
            generate_motion(robot)
        }
    }
}

fn generate_motion<R: RobotWrapper>(mut robot: R) -> FrankaResult<()> {
    robot.set_default_behavior()?;
    println!("WARNING: This example will move the robot! Please make sure to have the user stop button at hand!");
    println!("Press Enter to continue...");
    std::io::stdin().read_line(&mut String::new()).unwrap();

    // Set additional parameters always before the control loop, NEVER in the control loop!
    // Set the joint impedance.
    robot.set_joint_impedance([3000., 3000., 3000., 2500., 2500., 2000., 2000.])?;
    let lower_torque_thresholds_nominal = [25.0, 25.0, 22.0, 20.0, 19.0, 17.0, 14.];
    let upper_torque_thresholds_nominal = [35.0, 35.0, 32.0, 30.0, 29.0, 27.0, 24.0];
    let lower_torque_thresholds_acceleration = lower_torque_thresholds_nominal;
    let upper_torque_thresholds_acceleration = upper_torque_thresholds_nominal;

    let lower_force_thresholds_nominal = [30.0, 30.0, 30.0, 25.0, 25.0, 25.0];
    let upper_force_thresholds_nominal = [40.0, 40.0, 40.0, 35.0, 35.0, 35.0];
    let lower_force_thresholds_acceleration = lower_force_thresholds_nominal;
    let upper_force_thresholds_acceleration = upper_force_thresholds_nominal;
    robot.set_collision_behavior(
        lower_torque_thresholds_acceleration,
        upper_torque_thresholds_acceleration,
        lower_torque_thresholds_nominal,
        upper_torque_thresholds_nominal,
        lower_force_thresholds_acceleration,
        upper_force_thresholds_acceleration,
        lower_force_thresholds_nominal,
        upper_force_thresholds_nominal,
    )?;

    let q_goal = [0., -PI / 4., 0., -3. * PI / 4., 0., PI / 2., PI / 4.];
    robot.joint_motion(0.5, &q_goal)?;
    println!("Finished moving to initial joint configuration.");
    let time_max = 4.;
    let v_max = 0.1;
    let angle = PI / 4.;
    let mut time = 0.;
    let callback = |_state: &RobotState, period: &Duration| -> CartesianVelocities {
        time += period.as_secs_f64();

        let cycle = f64::floor(f64::powf(
            -1.0,
            (time - float_extras::f64::fmod(time, time_max)) / time_max,
        ));

        let v = cycle * v_max / 2. * (1. - f64::cos(2. * PI / time_max * time));
        let v_x = f64::cos(angle) * v;
        let v_z = -f64::sin(angle) * v;
        let output = CartesianVelocities::new([v_x, 0., v_z, 0., 0., 0.], None);
        if time >= 2. * time_max {
            println!("Finished motion, shutting down example");
            return output.motion_finished();
        }
        output
    };
    robot.control_cartesian_velocities(callback, None, None, None)
}
