// Copyright (c) 2021 Marco Boneberger
// Licensed under the EUPL-1.2-or-later

use clap::Parser;
use franka::FrankaResult;
use franka::Robot;
use franka::PandaState;
use franka::{JointVelocities, MotionFinished};
use std::f64::consts::PI;
use std::time::Duration;

/// An example showing how to generate a joint velocity motion.
///
/// WARNING: Before executing this example, make sure there is enough space in front of the robot.
#[derive(Parser, Debug)]
#[clap(author, version, name = "generate_joint_velocity_motion")]
struct CommandLineArguments {
    /// IP-Address or hostname of the robot
    pub franka_ip: String,
}

fn main() -> FrankaResult<()> {
    let address = CommandLineArguments::parse();
    let mut robot = Robot::new(address.franka_ip.as_str(), None, None)?;
    robot.set_default_behavior()?;
    println!("WARNING: This example will move the robot! Please make sure to have the user stop button at hand!");
    println!("Press Enter to continue...");
    std::io::stdin().read_line(&mut String::new()).unwrap();

    // Set additional parameters always before the control loop, NEVER in the control loop!
    // Set collision behavior.
    robot.set_collision_behavior(
        [20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0],
        [20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0],
        [20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0],
        [20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0],
        [20.0, 20.0, 20.0, 25.0, 25.0, 25.0],
        [20.0, 20.0, 20.0, 25.0, 25.0, 25.0],
        [20.0, 20.0, 20.0, 25.0, 25.0, 25.0],
        [20.0, 20.0, 20.0, 25.0, 25.0, 25.0],
    )?;

    let q_goal = [0., -PI / 4., 0., -3. * PI / 4., 0., PI / 2., PI / 4.];
    robot.joint_motion(0.5, &q_goal)?;
    println!("Finished moving to initial joint configuration.");
    let mut time = 0.;
    let omega_max = 1.0;
    let time_max = 1.0;
    let callback = move |_state: &PandaState, time_step: &Duration| -> JointVelocities {
        time += time_step.as_secs_f64();

        let cycle = f64::floor(f64::powf(
            -1.0,
            (time - float_extras::f64::fmod(time, time_max)) / time_max,
        ));

        let omega = cycle * omega_max / 2. * (1. - f64::cos(2. * PI / time_max * time));
        let out = JointVelocities::new([0., 0., 0., omega, omega, omega, omega]);
        if time >= 2. * time_max {
            println!("Finished motion, shutting down example");
            return out.motion_finished();
        }
        out
    };
    robot.control_joint_velocities(callback, None, None, None)
}
