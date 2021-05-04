// Copyright (c) 2021 Marco Boneberger
// Licensed under the EUPL-1.2-or-later
use franka::{Frame, FrankaResult, RealtimeConfig, Robot};
use nalgebra::Matrix4;
use structopt::StructOpt;

/// An example showing how to use the model library that prints the transformation
/// matrix of each joint with respect to the base frame.
#[derive(StructOpt, Debug)]
#[structopt(name = "print_joint_poses")]
struct CommandLineArguments {
    /// IP-Address or hostname of the robot
    #[structopt()]
    pub franka_ip: String,
}

fn main() -> FrankaResult<()> {
    let args = CommandLineArguments::from_args();
    let mut robot = Robot::new(args.franka_ip.as_str(), RealtimeConfig::kIgnore, None)?;
    let model = robot.load_model(false)?;
    let state = robot.read_once()?;
    let frames = vec![
        Frame::kJoint1,
        Frame::kJoint2,
        Frame::kJoint3,
        Frame::kJoint4,
        Frame::kJoint5,
        Frame::kJoint6,
        Frame::kJoint7,
        Frame::kFlange,
        Frame::kEndEffector,
    ];
    for frame in frames {
        let pose = Matrix4::from_column_slice(&model.pose_from_state(&frame, &state));
        println!("{} {}", frame, pose);
    }
    Ok(())
}
