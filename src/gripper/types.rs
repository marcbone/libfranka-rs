// Copyright (c) 2021 Marco Boneberger
// Licensed under the EUPL-1.2-or-later
#![allow(non_upper_case_globals)]

use std::fmt::Debug;

use serde::Deserialize;
use serde::Serialize;
use serde_repr::{Deserialize_repr, Serialize_repr};

use crate::network::MessageCommand;
use std::time::Duration;

pub static kVersion: u16 = 3;
pub static kCommandPort: u16 = 1338;

#[derive(Serialize_repr, Deserialize_repr, Debug, Copy, Clone)]
#[repr(u16)]
#[allow(non_camel_case_types)]
pub enum Command {
    kConnect,
    kHoming,
    kGrasp,
    kMove,
    kStop,
}

#[derive(Serialize_repr, Deserialize_repr, Debug)]
#[repr(u16)]
#[allow(non_camel_case_types)]
pub enum Status {
    kSuccess,
    kFail,
    kUnsuccessful,
    kAborted,
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

#[derive(Serialize, Deserialize, Debug, Copy, Clone)]
#[repr(packed)]
pub struct CommandHeader {
    pub command: Command,
    pub command_id: u32,
    pub size: u32,
}

impl CommandHeader {
    pub fn new(command: Command, command_id: u32, size: u32) -> CommandHeader {
        CommandHeader {
            command,
            command_id,
            size,
        }
    }
}

// struct RequestBase<T> {
//
// }
// struct ResponseBase<T>{
//     status : Status,
// }
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
            version: kVersion,
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
    pub header: CommandHeader,
    pub request: ConnectRequest,
}

#[derive(Serialize, Deserialize, Debug, Copy, Clone)]
#[repr(packed)]
pub struct MoveRequestWithHeader {
    pub header: CommandHeader,
    pub request: MoveRequest,
}

pub type HomingRequestWithHeader = CommandHeader;
pub type StopRequestWithHeader = CommandHeader;

#[derive(Serialize, Deserialize, Debug, Copy, Clone)]
#[repr(packed)]
pub struct GraspRequestWithHeader {
    pub header: CommandHeader,
    pub request: GraspRequest,
}

impl MessageCommand for CommandHeader {
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
    pub header: CommandHeader,
    pub status: Status,
    pub version: u16,
}

#[derive(Serialize, Deserialize, Debug)]
pub struct MoveResponse {
    pub header: CommandHeader,
    pub status: Status,
}

pub type GraspResponse = MoveResponse;
pub type HomingResponse = MoveResponse;
pub type StopResponse = HomingResponse;
