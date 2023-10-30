// Copyright (c) 2021 Marco Boneberger
// Licensed under the EUPL-1.2-or-later

use std::fmt::Debug;

use serde::Deserialize;
use serde::Serialize;
use serde_repr::{Deserialize_repr, Serialize_repr};

use crate::network::MessageCommand;
use serde::de::DeserializeOwned;
use std::time::Duration;

pub static GRIPPER_VERSION: u16 = 3;
pub static COMMAND_PORT: u16 = 1338;

#[derive(Serialize_repr, Deserialize_repr, Debug, Copy, Clone)]
#[repr(u16)]
pub enum GripperCommandEnum {
    Connect,
    Homing,
    Grasp,
    Move,
    Stop,
}

#[derive(Serialize_repr, Deserialize_repr, Debug, Copy, Clone)]
#[repr(u16)]
pub enum Status {
    Success,
    Fail,
    Unsuccessful,
    Aborted,
}

#[derive(Serialize, Deserialize, Debug, Copy, Clone)]
#[repr(packed)]
pub struct GripperStateIntern {
    pub message_id: u32,
    pub width: f64,
    pub max_width: f64,
    pub is_grasped: bool,
    pub temperature: u16,
}

impl GripperStateIntern {
    pub fn get_time(&self) -> Duration {
        Duration::from_millis(self.message_id as u64)
    }
}
// TODO is static a problem?
pub trait CommandHeader: Serialize + MessageCommand + Debug + DeserializeOwned + 'static {
    fn get_command_id(&self) -> u32;
    fn get_size(&self) -> u32;
}

#[derive(Serialize, Deserialize, Debug, Copy, Clone)]
#[repr(packed)]
pub struct GripperCommandHeader {
    pub command: GripperCommandEnum,
    pub command_id: u32,
    pub size: u32,
}

impl CommandHeader for GripperCommandHeader {
    fn get_command_id(&self) -> u32 {
        self.command_id
    }

    fn get_size(&self) -> u32 {
        self.size
    }
}

impl GripperCommandHeader {
    pub fn new(command: GripperCommandEnum, command_id: u32, size: u32) -> GripperCommandHeader {
        GripperCommandHeader {
            command,
            command_id,
            size,
        }
    }
}

#[derive(Serialize, Deserialize, Debug, Copy, Clone)]
#[repr(packed)]
pub struct ConnectRequest {
    pub version: u16,
    pub udp_port: u16,
}
#[derive(Serialize, Deserialize, Debug, Copy, Clone)]
#[repr(packed)]
pub struct MoveRequest {
    width: f64,
    speed: f64,
}

#[derive(Serialize, Deserialize, Debug, Copy, Clone)]
#[repr(packed)]
pub struct GraspEpsilon {
    inner: f64,
    outer: f64,
}

#[derive(Serialize, Deserialize, Debug, Copy, Clone)]
#[repr(packed)]
pub struct GraspRequest {
    width: f64,
    epsilon: GraspEpsilon,
    speed: f64,
    force: f64,
}

impl ConnectRequest {
    pub fn new(udp_port: u16) -> Self {
        ConnectRequest {
            version: GRIPPER_VERSION,
            udp_port,
        }
    }
}

impl MoveRequest {
    pub fn new(width: f64, speed: f64) -> Self {
        MoveRequest { width, speed }
    }
}

impl GraspRequest {
    pub fn new(width: f64, speed: f64, force: f64, epsilon_inner: f64, epsilon_outer: f64) -> Self {
        GraspRequest {
            width,
            epsilon: GraspEpsilon {
                inner: epsilon_inner,
                outer: epsilon_outer,
            },
            speed,
            force,
        }
    }
}

#[derive(Serialize, Deserialize, Debug, Copy, Clone)]
#[repr(packed)]
pub struct ConnectRequestWithHeader {
    pub header: GripperCommandHeader,
    pub request: ConnectRequest,
}

#[derive(Serialize, Deserialize, Debug, Copy, Clone)]
#[repr(packed)]
pub struct MoveRequestWithHeader {
    pub header: GripperCommandHeader,
    pub request: MoveRequest,
}

#[derive(Serialize, Deserialize, Debug, Copy, Clone)]
#[repr(packed)]
pub struct GraspRequestWithHeader {
    pub header: GripperCommandHeader,
    pub request: GraspRequest,
}

impl MessageCommand for GripperCommandHeader {
    fn get_command_message_id(&self) -> u32 {
        self.command_id
    }
}

impl MessageCommand for ConnectRequestWithHeader {
    fn get_command_message_id(&self) -> u32 {
        self.header.get_command_message_id()
    }
}

impl MessageCommand for MoveRequestWithHeader {
    fn get_command_message_id(&self) -> u32 {
        self.header.get_command_message_id()
    }
}

impl MessageCommand for GraspRequestWithHeader {
    fn get_command_message_id(&self) -> u32 {
        self.header.get_command_message_id()
    }
}

#[derive(Serialize, Deserialize, Debug)]
pub struct ConnectResponse {
    pub header: GripperCommandHeader,
    pub status: Status,
    pub version: u16,
}
