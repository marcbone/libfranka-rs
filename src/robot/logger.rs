// Copyright (c) 2021 Marco Boneberger
// Licensed under the EUPL-1.2-or-later

//! Contains the logging type definitions for [`ControlException`](`crate::exception::FrankaException::ControlException`)
use crate::robot::control_types::{
    CartesianPose, CartesianVelocities, JointPositions, JointVelocities, Torques,
};
use crate::robot::robot_state::RobotState;
use crate::robot::types::RobotCommand;
use std::collections::VecDeque;
use std::fmt::Debug;

/// Command sent to the robot. Structure used only for logging purposes.
#[derive(Debug, Copy, Clone)]
pub struct RobotCommandLog {
    /// sent joint positions.
    pub joint_positions: JointPositions,

    /// sent joint velocities.
    pub joint_velocities: JointVelocities,

    /// sent cartesian positions.
    pub cartesian_pose: CartesianPose,

    /// sent cartesian velocities.
    pub cartesian_velocities: CartesianVelocities,

    /// sent torques.
    pub torques: Torques,
}

/// One row of the log contains a robot command of timestamp n and a
/// corresponding robot state of timestamp n+1.
/// Provided by the [`ControlException`](`crate::exception::FrankaException::ControlException`).
#[derive(Debug, Clone)]
pub struct Record<State: Debug> {
    /// Robot state of timestamp n+1.
    pub state: State,
    /// Robot command of timestamp n, after rate limiting (if activated).
    pub command: RobotCommandLog,
}

impl<State: Debug> Record<State> {
    /// creates a string representation based on the debug formatter
    pub fn log(&self) -> String {
        format!("{:?}", self.clone())
    }
}

pub(crate) struct Logger<State: RobotState> {
    states: VecDeque<State>,
    commands: VecDeque<RobotCommand>,
    ring_front: usize,
    ring_size: usize,
    log_size: usize,
}

impl<State: RobotState> Logger<State> {
    pub fn new(log_size: usize) -> Self {
        Logger {
            states: VecDeque::with_capacity(log_size),
            commands: VecDeque::with_capacity(log_size),
            ring_front: 0,
            ring_size: 0,
            log_size,
        }
    }
    pub fn log(&mut self, state: &State, command: &RobotCommand) {
        self.states.push_back(state.clone());
        self.commands.push_back(*command);
        self.ring_front = (self.ring_front + 1) % self.log_size;
        if self.ring_size == self.log_size {
            self.states.pop_front();
            self.commands.pop_front();
        }
        self.ring_size = usize::min(self.log_size, self.ring_size + 1);
    }
    pub fn flush(&mut self) -> Vec<Record<State>> {
        let mut out: Vec<Record<State>> = Vec::new();
        for i in 0..self.ring_size {
            let index = (self.ring_front + i) % self.ring_size;
            let cmd = self.commands.get(index).unwrap();
            let command = RobotCommandLog {
                joint_positions: JointPositions::new(cmd.motion.q_c),
                joint_velocities: JointVelocities::new(cmd.motion.dq_c),
                cartesian_pose: CartesianPose::new(cmd.motion.O_T_EE_c, None),
                cartesian_velocities: CartesianVelocities::new(cmd.motion.O_dP_EE_c, None),
                torques: Torques::new(cmd.control.tau_J_d),
            };
            let record = Record {
                state: self.states.get(index).unwrap().clone(),
                command,
            };
            out.push(record);
        }
        self.ring_front = 0;
        self.ring_size = 0;
        out
    }
}
