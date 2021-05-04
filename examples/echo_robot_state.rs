// Copyright (c) 2021 Marco Boneberger
// Licensed under the EUPL-1.2-or-later

use franka::FrankaResult;
use franka::Robot;
use franka::RobotState;
use structopt::StructOpt;

/// An example showing how to continuously read the robot state.
#[derive(StructOpt, Debug)]
#[structopt(name = "echo_robot_state")]
struct CommandLineArguments {
    /// IP-Address or hostname of the robot
    #[structopt()]
    pub franka_ip: String,
}

fn main() -> FrankaResult<()> {
    let address = CommandLineArguments::from_args();
    let mut robot = Robot::new(address.franka_ip.as_str(), None, None)?;
    let mut count = 0;
    robot.read(|robot_state: &RobotState| {
        // Printing to standard output adds a delay. This is acceptable for a read loop such as this, but
        // should not be done in a control loop.
        println!("{:?}", robot_state);
        count += 1;
        count <= 100
    })?;
    println!("Done");
    Ok(())
}
