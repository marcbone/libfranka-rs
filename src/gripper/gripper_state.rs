// Copyright (c) 2021 Marco Boneberger
// Licensed under the EUPL-1.2-or-later

//! Contains the franka::GripperState type.

use crate::gripper::types::GripperStateIntern;
use serde::Deserialize;
use serde::Serialize;
use std::time::Duration;

/// Describes the gripper state.
#[derive(Serialize, Deserialize, Debug, Copy, Clone)]
pub struct GripperState {
    /// Current gripper opening width. Unit: \[m\].
    pub width: f64,

    /// Maximum gripper opening width.
    /// This parameter is estimated by homing the gripper.
    /// After changing the gripper fingers, a homing needs to be done. Unit: \[m\].
    ///
    /// See [`Gripper::homing()`](`crate::Gripper::homing`)
    pub max_width: f64,

    /// Indicates whether an object is currently grasped.
    pub is_grasped: bool,

    /// Current gripper temperature. Unit: [°C].
    pub temperature: u16,

    /// Strictly monotonically increasing timestamp since robot start.
    pub time: Duration,
}

impl From<GripperStateIntern> for GripperState {
    fn from(intern: GripperStateIntern) -> Self {
        GripperState {
            width: intern.width,
            max_width: intern.max_width,
            is_grasped: intern.is_grasped,
            temperature: intern.temperature,
            time: Duration::from_millis(intern.message_id as u64),
        }
    }
}
