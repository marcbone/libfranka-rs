// Copyright (c) 2021 Marco Boneberger
// Licensed under the EUPL-1.2-or-later

use std::f64::consts::PI;
use std::time::Duration;

use clap::Parser;

use franka::exception::FrankaException;
use franka::{Finishable, FrankaResult, JointVelocities, Panda, RobotState, RobotWrapper, FR3};

/// An example showing how to execute consecutive motions with error recovery.
///
/// WARNING: Before executing this example, make sure there is enough space in front and to the side
/// of the robot.
#[derive(Parser, Debug)]
#[clap(author, version, name = "generate_consecutive_motions")]
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
    // Set collision behavior.
    robot.set_collision_behavior(
        [10.0, 10.0, 9.0, 9.0, 6.0, 7.0, 6.0],
        [10.0, 10.0, 9.0, 9.0, 6.0, 7.0, 6.0],
        [10.0, 10.0, 9.0, 9.0, 6.0, 7.0, 6.0],
        [10.0, 10.0, 9.0, 9.0, 6.0, 7.0, 6.0],
        [10.0, 10.0, 10.0, 12.5, 12.5, 12.5],
        [10.0, 10.0, 10.0, 12.5, 12.5, 12.5],
        [10.0, 10.0, 10.0, 12.5, 12.5, 12.5],
        [10.0, 10.0, 10.0, 12.5, 12.5, 12.5],
    )?;

    let q_goal = [0., -PI / 4., 0., -3. * PI / 4., 0., PI / 2., PI / 4.];
    robot.joint_motion(0.5, &q_goal)?;
    println!("Finished moving to initial joint configuration.");
    let mut time = 0.;
    let omega_max = 0.2;
    let time_max = 4.0;
    let callback = move |_state: &RobotState, time_step: &Duration| -> JointVelocities {
        time += time_step.as_secs_f64();

        let cycle = f64::floor(f64::powf(
            -1.0,
            (time - float_extras::f64::fmod(time, time_max)) / time_max,
        ));

        let omega = cycle * omega_max / 2. * (1. - f64::cos(2. * PI / time_max * time));
        let out = JointVelocities::new([0., 0., 0., omega, omega, omega, omega]);
        if time >= 2. * time_max {
            println!("Finished motion.");
            return out.motion_finished();
        }
        out
    };
    for _ in 0..5 {
        let result = robot.control_joint_velocities(callback, None, None, None);
        match result {
            Ok(_) => {}
            Err(e) => match e {
                FrankaException::ControlException { log: _, error: msg } => {
                    println!("{}", msg);
                    println!("Running error recovery...");
                    robot.automatic_error_recovery()?;
                }
                _ => {
                    return Err(e);
                }
            },
        }
    }
    Ok(())
}
