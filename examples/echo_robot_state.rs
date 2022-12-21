// Copyright (c) 2021 Marco Boneberger
// Licensed under the EUPL-1.2-or-later

use clap::Parser;
use franka::robot::{Robot, FR3};
use franka::{FrankaResult, PandaState};
// use franka::Robot;
// use franka::RobotState;

/// An example showing how to continuously read the robot state.
#[derive(Parser, Debug)]
#[clap(author, version, name = "echo_robot_state")]
struct CommandLineArguments {
    /// IP-Address or hostname of the robot
    pub franka_ip: String,
}

fn main() -> FrankaResult<()> {
    // let address = CommandLineArguments::parse();
    let mut robot = FR3::new("localhost", None, None)?;
    // robot.set_collision_behavior([0.;7],[0.;7],[0.;7],[0.;7],[0.;6],[0.;6],[0.;6],[0.;6]);
    robot.set_joint_impedance([3000., 3000., 3000., 2500., 2500., 2000., 2000.])?;
    let mut count = 0;
    robot.read(|robot_state: &PandaState| {
        // Printing to standard output adds a delay. This is acceptable for a read loop such as this, but
        // should not be done in a control loop.
        println!("{:?}", robot_state);
        count += 1;
        count <= 100
    })?;
    // println!("Done");
    Ok(())
}
