// Copyright (c) 2021 Marco Boneberger
// Licensed under the EUPL-1.2-or-later
use crate::exception::FrankaException::ModelException;
use crate::network::Network;
use crate::robot::service_types::{
    LoadModelLibraryArchitecture, LoadModelLibraryRequest, LoadModelLibraryStatus,
    LoadModelLibrarySystem,
};
use crate::robot_data::RobotData;
use crate::FrankaResult;
use std::fmt;
use std::fmt::Display;
use std::fmt::Formatter;
use std::fs::File;
use std::io::Write;
use std::path::Path;

pub trait LibraryDownloader {
    type Data: RobotData;
    fn download(network: &mut Network<Self::Data>, download_path: &Path) -> FrankaResult<()>;
}
pub struct LibraryDownloaderGeneric<Data: RobotData> {
    data: std::marker::PhantomData<Data>,
}
impl<Data: RobotData> LibraryDownloader for LibraryDownloaderGeneric<Data> {
    type Data = Data;

    fn download(network: &mut Network<Data>, download_path: &Path) -> FrankaResult<()> {
        if cfg!(all(target_os = "linux", target_arch = "x86_64")) {
            let request = LoadModelLibraryRequest {
                architecture: LoadModelLibraryArchitecture::X64,
                system: LoadModelLibrarySystem::Linux,
            };
            let command = Data::create_model_library_request(&mut network.command_id, request);
            let command_id: u32 = network.tcp_send_request(command);
            let mut buffer = Vec::<u8>::new();
            let _status: LoadModelLibraryStatus =
                network.tcp_blocking_receive_load_library_response(command_id, &mut buffer)?;
            let mut file = File::create(download_path).map_err(|_| ModelException {
                message: "Error writing model to disk:".to_string(),
            })?;
            file.write(&buffer).map_err(|_| ModelException {
                message: "Error writing model to disk:".to_string(),
            })?;
            Ok(())
        } else {
            Err(ModelException {
                message:
                    "Your platform is not yet supported for Downloading models. Please use Linux on\
                        x86_64 for now"
                        .to_string(),
            })
        }
    }
}

#[derive(Debug)]
pub struct UnsupportedPlatform {}

impl std::error::Error for UnsupportedPlatform {}

impl Display for UnsupportedPlatform {
    fn fmt(&self, f: &mut Formatter) -> fmt::Result {
        writeln!(
            f,
            "Your platform is not yet supported for Downloading models. Please use Linux on\
        x86_64 for now"
        )
    }
}
