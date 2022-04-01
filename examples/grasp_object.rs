// Copyright (c) 2021 Marco Boneberger
// Licensed under the EUPL-1.2-or-later
use clap::Parser;
use franka::{FrankaResult, Gripper};
use std::time::Duration;

/// An example showing how to control FRANKA's gripper.
#[derive(Parser, Debug)]
#[clap(author, version, name = "grasp_object")]
struct CommandLineArguments {
    /// IP-Address or hostname of the gripper
    pub gripper_hostname: String,
    /// Perform homing before grasping to calibrate the gripper
    #[clap(long)]
    pub homing: bool,
    /// Width of the object in meter
    pub object_width: f64,
}

fn main() -> FrankaResult<()> {
    let args: CommandLineArguments = CommandLineArguments::parse();
    let mut gripper = Gripper::new(args.gripper_hostname.as_str())?;
    if args.homing {
        gripper.homing()?;
    }
    let state = gripper.read_once()?;
    if state.max_width < args.object_width {
        eprintln!("Object is too large for the current fingers on the gripper.");
        std::process::exit(-1);
    }
    gripper.grasp(args.object_width, 0.1, 60., None, None)?;
    std::thread::sleep(Duration::from_secs(3));
    let state = gripper.read_once()?;
    if !state.is_grasped {
        eprintln!("Object lost");
        std::process::exit(-1);
    }
    println!("Grasped object, will release it now.");
    gripper.stop()?;
    Ok(())
}
