// Copyright (c) 2021 Marco Boneberger
// Licensed under the EUPL-1.2-or-later

use std::fs;
use std::path::PathBuf;

use clap::Parser;

use franka::exception::FrankaException::ModelException;
use franka::{Fr3, FrankaResult, Panda, RealtimeConfig, Robot, RobotModel};

/// Downloads the model for offline usage
#[derive(Parser, Debug)]
#[clap(author, version, name = "download_model")]
struct CommandLineArguments {
    /// IP-Address or hostname of the robot
    pub franka_ip: String,

    /// Directory where the model should be downloaded
    download_path: PathBuf,
    /// Use this option to run the example on a Panda
    #[clap(short, long, action)]
    pub panda: bool,
}

fn main() -> FrankaResult<()> {
    let args: CommandLineArguments = CommandLineArguments::parse();
    let path = args.download_path;
    match args.panda {
        true => {
            let robot = Panda::new(args.franka_ip.as_str(), RealtimeConfig::Ignore, None)?;
            download_model(robot, path)
        }
        false => {
            let robot = Fr3::new(args.franka_ip.as_str(), RealtimeConfig::Ignore, None)?;
            download_model(robot, path)
        }
    }
}

fn download_model<R: Robot>(mut robot: R, mut path: PathBuf) -> FrankaResult<()> {
    let model = robot.load_model(true)?;
    let model_path = model.get_model_path().expect("Model file has been deleted");
    if path.is_dir() {
        path.push(model_path.file_name().unwrap());
    }
    fs::copy(model_path, &path).map_err(|_| ModelException {
        message: "Could not copy model to download location".to_string(),
    })?;
    println!("Model successfully downloaded to {:?}", &path);
    Ok(())
}
