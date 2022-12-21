// Copyright (c) 2021 Marco Boneberger
// Licensed under the EUPL-1.2-or-later

use clap::Parser;
use franka::exception::FrankaException::ModelException;
use franka::robot::{Robot, FR3};
use franka::{FrankaResult, RealtimeConfig};
use std::fs;
use std::path::PathBuf;

/// Downloads the model for offline usage
#[derive(Parser, Debug)]
#[clap(author, version, name = "download_model")]
struct CommandLineArguments {
    /// IP-Address or hostname of the robot
    pub franka_ip: String,

    /// Directory where the model should be downloaded
    #[clap(parse(from_os_str))]
    download_path: PathBuf,
}

fn main() -> FrankaResult<()> {
    let args: CommandLineArguments = CommandLineArguments::parse();
    let mut path = args.download_path;
    path.push("model.so");
    let mut robot = FR3::new(args.franka_ip.as_str(), RealtimeConfig::Ignore, None)?;
    robot.load_model(true)?;
    fs::copy("/tmp/model.so", &path).map_err(|_| ModelException {
        message: "Could copy model to download location".to_string(),
    })?;
    println!("Model successfully downloaded to {:?}", &path);
    Ok(())
}
