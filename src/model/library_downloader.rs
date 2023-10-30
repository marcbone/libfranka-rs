// Copyright (c) 2021 Marco Boneberger
// Licensed under the EUPL-1.2-or-later

use std::fs::File;
use std::io::Write;
use std::path::Path;

use crate::exception::FrankaException::ModelException;
use crate::network::Network;
use crate::robot::service_types::{
    LoadModelLibraryArchitecture, LoadModelLibraryRequest, LoadModelLibraryStatus,
    LoadModelLibrarySystem,
};
use crate::robot::PrivateRobotData;
use crate::FrankaResult;

pub(crate) trait LibraryDownloader {
    type Data: PrivateRobotData;
    fn download(network: &mut Network<Self::Data>, download_path: &Path) -> FrankaResult<()>;
}

pub(crate) struct LibraryDownloaderGeneric<Data: PrivateRobotData> {
    data: std::marker::PhantomData<Data>,
}

impl<Data: PrivateRobotData> LibraryDownloader for LibraryDownloaderGeneric<Data> {
    type Data = Data;

    fn download(network: &mut Network<Data>, download_path: &Path) -> FrankaResult<()> {
        let request = if cfg!(all(target_os = "linux", target_arch = "x86_64")) {
            LoadModelLibraryRequest {
                architecture: LoadModelLibraryArchitecture::X64,
                system: LoadModelLibrarySystem::Linux,
            }
        } else if cfg!(all(target_os = "linux", target_arch = "aarch64")) {
            LoadModelLibraryRequest {
                architecture: LoadModelLibraryArchitecture::Arm64,
                system: LoadModelLibrarySystem::Linux,
            }
        } else if cfg!(all(target_os = "linux", target_arch = "x86")) {
            LoadModelLibraryRequest {
                architecture: LoadModelLibraryArchitecture::X86,
                system: LoadModelLibrarySystem::Linux,
            }
        } else if cfg!(all(target_os = "linux", target_arch = "arm")) {
            LoadModelLibraryRequest {
                architecture: LoadModelLibraryArchitecture::Arm,
                system: LoadModelLibrarySystem::Linux,
            }
        } else {
            return Err(ModelException {
                message:
                    "Your platform is not yet supported for Downloading models. Please use Linux on\
                        x86_64, x86, arm, or arm64 for now"
                        .to_string(),
            });
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
    }
}
