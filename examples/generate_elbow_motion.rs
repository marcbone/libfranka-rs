// Copyright (c) 2021 Marco Boneberger
// Licensed under the EUPL-1.2-or-later

use std::f64::consts::PI;
use std::time::Duration;

use clap::Parser;

use franka::{CartesianPose, Fr3, FrankaResult, MotionFinished, Panda, Robot, RobotState};

/// An example showing how to move the robot's elbow.
///
/// WARNING: Before executing this example, make sure that the elbow has enough space to move.
#[derive(Parser, Debug)]
#[clap(author, version, name = "generate_elbow_motion")]
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
            let robot = Fr3::new(args.franka_ip.as_str(), None, None)?;
            generate_motion(robot)
        }
    }
}

fn generate_motion<R: Robot>(mut robot: R) -> FrankaResult<()> {
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
    let mut initial_pose = [0.; 16];
    let mut initial_elbow = [0.; 2];
    let mut time = 0.;
    let callback = |state: &RobotState, time_step: &Duration| -> CartesianPose {
        time += time_step.as_secs_f64();
        if time == 0. {
            initial_pose = state.O_T_EE_c;
            initial_elbow = state.elbow_c;
        }
        let angle = PI / 10. * (1. - f64::cos(PI / 5. * time));
        let mut elbow = initial_elbow;
        elbow[0] += angle;
        let out = CartesianPose::new(initial_pose, Some(elbow));
        if time >= 10.0 {
            println!("Finished motion, shutting down example");
            return out.motion_finished();
        }
        out
    };
    robot.control_cartesian_pose(callback, None, None, None)
}
