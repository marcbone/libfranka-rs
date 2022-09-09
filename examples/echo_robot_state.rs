// Copyright (c) 2021 Marco Boneberger
// Licensed under the EUPL-1.2-or-later

use clap::Parser;
use franka::{FrankaResult, Panda, PandaState};
use franka::robot::Robot;
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
    let address = CommandLineArguments::parse();
    let mut robot = Panda::new(address.franka_ip.as_str(), None, None)?;
    let mut count = 0;
    robot.read(|robot_state: &PandaState| {
        // Printing to standard output adds a delay. This is acceptable for a read loop such as this, but
        // should not be done in a control loop.
        println!("{:?}", robot_state);
        count += 1;
        count <= 100
    })?;
    println!("Done");
    Ok(())
}
