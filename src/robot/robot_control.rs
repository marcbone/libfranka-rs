// Copyright (c) 2021 Marco Boneberger
// Licensed under the EUPL-1.2-or-later

use crate::exception::FrankaResult;
use crate::robot::control_types::RealtimeConfig;
use crate::robot::robot_state::AbstractRobotState;
use crate::robot::service_types::{MoveControllerMode, MoveDeviation, MoveMotionGeneratorMode};
use crate::robot::types::{ControllerCommand, MotionGeneratorCommand};

pub(crate) trait RobotControl {
    type State: AbstractRobotState;
    fn start_motion(
        &mut self,
        controller_mode: MoveControllerMode,
        motion_generator_mode: MoveMotionGeneratorMode,
        maximum_path_deviation: &MoveDeviation,
        maximum_goal_deviation: &MoveDeviation,
    ) -> FrankaResult<u32>;
    fn finish_motion(
        &mut self,
        motion_id: u32,
        motion_command: Option<&MotionGeneratorCommand>,
        control_command: Option<&ControllerCommand>,
    ) -> FrankaResult<()>;
    fn cancel_motion(&mut self, motion_id: u32);
    fn update(
        &mut self,
        motion_command: Option<&MotionGeneratorCommand>,
        control_command: Option<&ControllerCommand>,
    ) -> FrankaResult<Self::State>;
    fn realtime_config(&self) -> RealtimeConfig;
    fn throw_on_motion_error(
        &mut self,
        robot_state: &Self::State,
        motion_id: u32,
    ) -> FrankaResult<()>;
}
