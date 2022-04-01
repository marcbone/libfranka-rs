// Copyright (c) 2021 Marco Boneberger
// Licensed under the EUPL-1.2-or-later
use std::fmt::Debug;

use crate::robot::types::RobotMode::Other;
use serde::Deserialize;
use serde::Serialize;
use serde_repr::{Deserialize_repr, Serialize_repr};

#[derive(Serialize_repr, Deserialize_repr, Debug, Copy, Clone, PartialEq)]
#[repr(u8)]
pub enum MotionGeneratorMode {
    Idle,
    JointPosition,
    JointVelocity,
    CartesianPosition,
    CartesianVelocity,
}

#[derive(Serialize_repr, Deserialize_repr, Debug, Copy, Clone, PartialEq)]
#[repr(u8)]
pub enum ControllerMode {
    JointImpedance,
    CartesianImpedance,
    ExternalController,
    Other,
}
/// Describes the robot's current mode.
#[derive(Serialize_repr, Deserialize_repr, Debug, Copy, Clone, PartialEq)]
#[repr(u8)]
pub enum RobotMode {
    Other,
    Idle,
    Move,
    Guiding,
    Reflex,
    UserStopped,
    AutomaticErrorRecovery,
}
impl Default for RobotMode {
    fn default() -> Self {
        Other
    }
}

#[derive(Serialize, Deserialize, Debug, Copy, Clone, PartialEq)]
#[allow(non_snake_case)]
#[repr(packed)]
pub struct RobotStateIntern {
    pub message_id: u64,
    pub O_T_EE: [f64; 16],
    pub O_T_EE_d: [f64; 16],
    pub F_T_EE: [f64; 16],
    pub EE_T_K: [f64; 16],
    pub F_T_NE: [f64; 16],
    pub NE_T_EE: [f64; 16],
    pub m_ee: f64,
    pub I_ee: [f64; 9],
    pub F_x_Cee: [f64; 3],
    pub m_load: f64,
    pub I_load: [f64; 9],
    pub F_x_Cload: [f64; 3],
    pub elbow: [f64; 2],
    pub elbow_d: [f64; 2],
    pub tau_J: [f64; 7],
    pub tau_J_d: [f64; 7],
    pub dtau_J: [f64; 7],
    pub q: [f64; 7],
    pub q_d: [f64; 7],
    pub dq: [f64; 7],
    pub dq_d: [f64; 7],
    pub ddq_d: [f64; 7],
    pub joint_contact: [f64; 7],
    pub cartesian_contact: [f64; 6],
    pub joint_collision: [f64; 7],
    pub cartesian_collision: [f64; 6],
    pub tau_ext_hat_filtered: [f64; 7],
    pub O_F_ext_hat_K: [f64; 6],
    pub K_F_ext_hat_K: [f64; 6],
    pub O_dP_EE_d: [f64; 6],
    pub O_ddP_O: [f64; 3],
    pub elbow_c: [f64; 2],
    pub delbow_c: [f64; 2],
    pub ddelbow_c: [f64; 2],
    pub O_T_EE_c: [f64; 16],
    pub O_dP_EE_c: [f64; 6],
    pub O_ddP_EE_c: [f64; 6],
    pub theta: [f64; 7],
    pub dtheta: [f64; 7],
    pub motion_generator_mode: MotionGeneratorMode,
    pub controller_mode: ControllerMode,
    // pub errors: [bool; 37], reenable when const generics arrive
    // pub reflex_reason: [bool; 37], reenable when const generics arrive
    pub errors: RoboErrorHelperStruct,
    pub robot_mode: RobotMode,
    pub control_command_success_rate: f64,
}

impl RobotStateIntern {
    pub fn dummy() -> Self {
        RobotStateIntern {
            message_id: 0,
            O_T_EE: [0.; 16],
            O_T_EE_d: [0.; 16],
            F_T_EE: [0.; 16],
            EE_T_K: [0.; 16],
            F_T_NE: [0.; 16],
            NE_T_EE: [0.; 16],
            m_ee: 0.,
            I_ee: [0.; 9],
            F_x_Cee: [0.; 3],
            m_load: 0.,
            I_load: [0.; 9],
            F_x_Cload: [0.; 3],
            elbow: [0.; 2],
            elbow_d: [0.; 2],
            tau_J: [0.; 7],
            tau_J_d: [0.; 7],
            dtau_J: [0.; 7],
            q: [0.; 7],
            q_d: [0.; 7],
            dq: [0.; 7],
            dq_d: [0.; 7],
            ddq_d: [0.; 7],
            joint_contact: [0.; 7],
            cartesian_contact: [0.; 6],
            joint_collision: [0.; 7],
            cartesian_collision: [0.; 6],
            tau_ext_hat_filtered: [0.; 7],
            O_F_ext_hat_K: [0.; 6],
            K_F_ext_hat_K: [0.; 6],
            O_dP_EE_d: [0.; 6],
            O_ddP_O: [0.; 3],
            elbow_c: [0.; 2],
            delbow_c: [0.; 2],
            ddelbow_c: [0.; 2],
            O_T_EE_c: [0.; 16],
            O_dP_EE_c: [0.; 6],
            O_ddP_EE_c: [0.; 6],
            theta: [0.; 7],
            dtheta: [0.; 7],
            motion_generator_mode: MotionGeneratorMode::Idle,
            controller_mode: ControllerMode::JointImpedance,
            errors: RoboErrorHelperStruct {
                errors1: [false; 32],
                errors2: [false; 9],
                reflex_reason1: [false; 32],
                reflex_reason2: [false; 9],
            },
            robot_mode: RobotMode::Other,
            control_command_success_rate: 0.0,
        }
    }
}

/// this struct is used as serde cant serialize arrays bigger than 32.
/// we have to wait for const generics to arrive in serde.
/// https://crates.io/crates/serde-big-array would be an alternative
/// but I do not want to add a dependency with less than 1 million downloads.
#[derive(Serialize, Deserialize, Debug, Copy, Clone, PartialEq)]
#[repr(packed)]
pub struct RoboErrorHelperStruct {
    pub errors1: [bool; 32],
    pub errors2: [bool; 9],
    pub reflex_reason1: [bool; 32],
    pub reflex_reason2: [bool; 9],
}

impl RoboErrorHelperStruct {
    pub fn combine_errors(&self) -> [bool; 41] {
        let mut out = [false; 41];
        for (i, &val) in self.errors1.iter().enumerate() {
            out[i] = val;
        }
        for (i, &val) in self.errors2.iter().enumerate() {
            out[i + 32] = val;
        }
        out
    }
    pub fn combine_reflex_reasons(&self) -> [bool; 41] {
        let mut out = [false; 41];
        for (i, &val) in self.reflex_reason1.iter().enumerate() {
            out[i] = val;
        }
        for (i, &val) in self.reflex_reason2.iter().enumerate() {
            out[i + 32] = val;
        }
        out
    }
}

#[derive(Serialize, Deserialize, Debug, Copy, Clone)]
#[allow(non_snake_case)]
#[repr(packed)]
pub struct MotionGeneratorCommandPacked {
    pub q_c: [f64; 7],
    pub dq_c: [f64; 7],
    pub O_T_EE_c: [f64; 16],
    pub O_dP_EE_c: [f64; 6],
    pub elbow_c: [f64; 2],
    pub valid_elbow: bool,
    pub motion_generation_finished: bool,
}

impl MotionGeneratorCommandPacked {
    pub fn new(command: MotionGeneratorCommand) -> Self {
        MotionGeneratorCommandPacked {
            q_c: command.q_c,
            dq_c: command.dq_c,
            O_T_EE_c: command.O_T_EE_c,
            O_dP_EE_c: command.O_dP_EE_c,
            elbow_c: command.elbow_c,
            valid_elbow: command.valid_elbow,
            motion_generation_finished: command.motion_generation_finished,
        }
    }
}

#[derive(Serialize, Deserialize, Debug, Copy, Clone)]
#[allow(non_snake_case)]
pub struct MotionGeneratorCommand {
    pub q_c: [f64; 7],
    pub dq_c: [f64; 7],
    pub O_T_EE_c: [f64; 16],
    pub O_dP_EE_c: [f64; 6],
    pub elbow_c: [f64; 2],
    pub valid_elbow: bool,
    pub motion_generation_finished: bool,
}

#[allow(non_snake_case)]
impl MotionGeneratorCommand {
    pub fn new(
        q_c: [f64; 7],
        dq_c: [f64; 7],
        O_T_EE_c: [f64; 16],
        O_dP_EE_c: [f64; 6],
        elbow_c: [f64; 2],
    ) -> Self {
        MotionGeneratorCommand {
            q_c,
            dq_c,
            O_T_EE_c,
            O_dP_EE_c,
            elbow_c,
            valid_elbow: false,
            motion_generation_finished: false,
        }
    }
    pub fn pack(self) -> MotionGeneratorCommandPacked {
        MotionGeneratorCommandPacked::new(self)
    }
}

#[derive(Serialize, Deserialize, Debug, Copy, Clone)]
#[allow(non_snake_case)]
#[repr(packed)]
pub struct ControllerCommandPacked {
    pub tau_J_d: [f64; 7],
}

#[derive(Serialize, Deserialize, Debug, Copy, Clone)]
#[allow(non_snake_case)]
pub struct ControllerCommand {
    pub tau_J_d: [f64; 7],
}

impl ControllerCommand {
    pub fn pack(self) -> ControllerCommandPacked {
        ControllerCommandPacked {
            tau_J_d: self.tau_J_d,
        }
    }
}

#[derive(Serialize, Deserialize, Debug, Copy, Clone)]
#[repr(packed)]
pub struct RobotCommand {
    pub message_id: u64,
    pub motion: MotionGeneratorCommandPacked,
    pub control: ControllerCommandPacked,
}
