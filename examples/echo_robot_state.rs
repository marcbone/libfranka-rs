// Copyright (c) 2021 Marco Boneberger
// Licensed under the EUPL-1.2-or-later

use clap::Parser;

use franka::{Fr3, FrankaResult, Panda, Robot, RobotState};

/// An example showing how to continuously read the robot state.
#[derive(Parser, Debug)]
#[clap(author, version, name = "echo_robot_state")]
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
            echo_robot_state(robot)
        }
        false => {
            let robot = Fr3::new(args.franka_ip.as_str(), None, None)?;
            echo_robot_state(robot)
        }
    }
}

fn echo_robot_state<R: Robot>(mut robot: R) -> FrankaResult<()> {
    robot.set_collision_behavior(
        [0.; 7], [0.; 7], [0.; 7], [0.; 7], [0.; 6], [0.; 6], [0.; 6], [0.; 6],
    )?;
    robot.set_joint_impedance([3000., 3000., 3000., 2500., 2500., 2000., 2000.])?;
    let mut count = 0;
    robot.read(|robot_state: &RobotState| {
        // Printing to standard output adds a delay. This is acceptable for a read loop such as this, but
        // should not be done in a control loop.
        println!("{:?}", robot_state);
        count += 1;
        count <= 100
    })?;
    Ok(())
}
