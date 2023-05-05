// Copyright (c) 2021 Marco Boneberger
// Licensed under the EUPL-1.2-or-later
use std::fmt::Debug;
use std::time::Duration;

use crate::exception::{FrankaException, FrankaResult};
use crate::robot::control_tools::{
    has_realtime_kernel, set_current_thread_to_highest_scheduler_priority,
};
use crate::robot::control_types::{ControllerMode, ConvertMotion, RealtimeConfig, Torques};
use crate::robot::low_pass_filter::{low_pass_filter, MAX_CUTOFF_FREQUENCY};
use crate::robot::motion_generator_traits::MotionGeneratorTrait;
use crate::robot::rate_limiting::{limit_rate_torques, DELTA_T, MAX_TORQUE_RATE};
use crate::robot::robot_data::RobotData;
use crate::robot::robot_impl::RobotImplementation;
use crate::robot::robot_state::AbstractRobotState;
use crate::robot::service_types::{MoveControllerMode, MoveDeviation};
use crate::robot::types::{ControllerCommand, MotionGeneratorCommand};
use crate::Finishable;

type ControlCallback<'b, State> = &'b mut dyn FnMut(&State, &Duration) -> Torques;
pub struct ControlLoop<
    'a,
    'b,
    Data: RobotData,
    T: RobotImplementation<Data>,
    U: ConvertMotion<<Data as RobotData>::State> + Debug + MotionGeneratorTrait + Finishable,
    F: FnMut(&<Data as RobotData>::State, &Duration) -> U,
> {
    pub default_deviation: MoveDeviation,
    robot: &'a mut T,
    motion_callback: F,
    control_callback: Option<ControlCallback<'b, <Data as RobotData>::State>>,
    limit_rate: bool,
    cutoff_frequency: f64,
    pub motion_id: u32,
}

impl<
        'a,
        'b,
        Data: RobotData,
        T: RobotImplementation<Data>,
        U: ConvertMotion<<Data as RobotData>::State> + Debug + MotionGeneratorTrait + Finishable,
        F: FnMut(&<Data as RobotData>::State, &Duration) -> U,
    > ControlLoop<'a, 'b, Data, T, U, F>
{
    pub fn new(
        robot: &'a mut T,
        control_callback: ControlCallback<'b, <Data as RobotData>::State>,
        motion_callback: F,
        limit_rate: bool,
        cutoff_frequency: f64,
    ) -> FrankaResult<Self> {
        let mut control_loop = ControlLoop::new_intern(
            robot,
            motion_callback,
            Some(control_callback),
            limit_rate,
            cutoff_frequency,
        )?;
        control_loop.motion_id = control_loop.robot.start_motion(
            MoveControllerMode::ExternalController,
            U::get_motion_generator_mode(),
            &control_loop.default_deviation,
            &control_loop.default_deviation,
        )?;
        Ok(control_loop)
    }
    pub fn from_control_mode(
        robot: &'a mut T,
        control_mode: ControllerMode,
        motion_callback: F,
        limit_rate: bool,
        cutoff_frequency: f64,
    ) -> FrankaResult<Self> {
        let mode: MoveControllerMode = match control_mode {
            ControllerMode::JointImpedance => MoveControllerMode::JointImpedance,
            ControllerMode::CartesianImpedance => MoveControllerMode::CartesianImpedance,
        };
        let mut control_loop =
            ControlLoop::new_intern(robot, motion_callback, None, limit_rate, cutoff_frequency)?;
        control_loop.motion_id = control_loop.robot.start_motion(
            mode,
            U::get_motion_generator_mode(),
            &control_loop.default_deviation,
            &control_loop.default_deviation,
        )?;
        Ok(control_loop)
    }
    fn new_intern(
        robot: &'a mut T,
        motion_callback: F,
        control_callback: Option<ControlCallback<'b, <Data as RobotData>::State>>,
        limit_rate: bool,
        cutoff_frequency: f64,
    ) -> FrankaResult<Self> {
        let enforce_real_time = robot.realtime_config() == RealtimeConfig::Enforce;
        let control_loop = ControlLoop {
            default_deviation: MoveDeviation {
                translation: 10.,
                rotation: 3.12,
                elbow: 2. * std::f64::consts::PI,
            },
            robot,
            motion_callback,
            control_callback,
            limit_rate,
            cutoff_frequency,
            motion_id: 0,
        };
        if enforce_real_time {
            if has_realtime_kernel() {
                set_current_thread_to_highest_scheduler_priority()?;
            } else {
                return Err(FrankaException::RealTimeException {
                    message: "libfranka-rs: Running kernel does not have realtime capabilities."
                        .to_string(),
                });
            }
        }
        Ok(control_loop)
    }
    /// this is the function which is operator() in c++
    pub fn run(&mut self) -> FrankaResult<()> {
        match self.do_loop() {
            Ok(_) => Ok(()),
            Err(error) => {
                self.robot.cancel_motion(self.motion_id);
                Err(error)
            }
        }
    }

    fn do_loop(&mut self) -> FrankaResult<()> {
        let mut robot_state = self.robot.update(None, None)?;
        self.robot
            .throw_on_motion_error(&robot_state, self.motion_id)?;
        let mut previous_time = robot_state.get_time();
        let mut motion_command =
            MotionGeneratorCommand::new([0.; 7], [0.; 7], [0.; 16], [0.; 6], [0.; 2]);
        if self.control_callback.is_some() {
            let mut control_command = ControllerCommand { tau_J_d: [0.; 7] };
            while self.spin_motion(
                &robot_state,
                &(robot_state.get_time() - previous_time),
                &mut motion_command,
            ) && self.spin_control(
                &robot_state,
                &(robot_state.get_time() - previous_time),
                &mut control_command,
            ) {
                previous_time = robot_state.get_time();
                robot_state = self
                    .robot
                    .update(Some(&motion_command), Some(&control_command))?;
                self.robot
                    .throw_on_motion_error(&robot_state, self.motion_id)?;
            }
            self.robot.finish_motion(
                self.motion_id,
                Some(&motion_command),
                Some(&control_command),
            )?;
        } else {
            while self.spin_motion(
                &robot_state,
                &(robot_state.get_time() - previous_time),
                &mut motion_command,
            ) {
                previous_time = robot_state.get_time();
                robot_state = self.robot.update(Some(&motion_command), None)?;
                self.robot
                    .throw_on_motion_error(&robot_state, self.motion_id)?;
            }
            self.robot
                .finish_motion(self.motion_id, Some(&motion_command), None)?;
        }
        Ok(())
    }

    fn spin_control(
        &mut self,
        robot_state: &<Data as RobotData>::State,
        time_step: &Duration,
        command: &mut ControllerCommand,
    ) -> bool {
        let mut control_output: Torques =
            (self.control_callback.as_mut().unwrap())(robot_state, time_step);
        if self.cutoff_frequency < MAX_CUTOFF_FREQUENCY {
            for i in 0..7 {
                control_output.tau_J[i] = low_pass_filter(
                    DELTA_T,
                    control_output.tau_J[i],
                    robot_state.get_tau_J_d()[i],
                    self.cutoff_frequency,
                );
            }
        }
        if self.limit_rate {
            control_output.tau_J = limit_rate_torques(
                &MAX_TORQUE_RATE,
                &control_output.tau_J,
                &robot_state.get_tau_J_d(),
            );
        }
        command.tau_J_d = control_output.tau_J;
        command.tau_J_d.iter().for_each(|x| assert!(x.is_finite()));
        !control_output.is_finished()
    }
    fn spin_motion(
        &mut self,
        robot_state: &T::State,
        time_step: &Duration,
        command: &mut MotionGeneratorCommand,
    ) -> bool {
        let motion_output = (self.motion_callback)(robot_state, time_step);
        motion_output.convert_motion(robot_state, command, self.cutoff_frequency, self.limit_rate);
        !motion_output.is_finished()
    }
}
