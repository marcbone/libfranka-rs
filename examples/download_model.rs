// Copyright (c) 2021 Marco Boneberger
// Licensed under the EUPL-1.2-or-later

use franka::exception::FrankaException::ModelException;
use franka::Robot;
use franka::{FrankaResult, RealtimeConfig};
use std::fs;
use std::path::PathBuf;
use structopt::StructOpt;

/// Downloads the model for offline usage
#[derive(StructOpt, Debug)]
#[structopt(name = "download_model")]
struct CommandLineArguments {
    /// IP-Address or hostname of the robot
    #[structopt()]
    pub franka_ip: String,

    /// Directory where the model should be downloaded
    #[structopt(parse(from_os_str))]
    download_path: PathBuf,
}

fn main() -> FrankaResult<()> {
    let args: CommandLineArguments = CommandLineArguments::from_args();
    let mut path = args.download_path;
    path.push("model.so");
    let mut robot = Robot::new(args.franka_ip.as_str(), RealtimeConfig::Ignore, None)?;
    robot.load_model(true)?;
    fs::copy("/tmp/model.so", &path).map_err(|_| ModelException {
        message: "Could copy model to download location".to_string(),
    })?;
    println!("Model successfully downloaded to {:?}", &path);
    Ok(())
}
