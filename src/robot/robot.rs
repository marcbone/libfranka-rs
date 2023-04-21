use crate::network::Network;
use crate::robot::control_loop::ControlLoop;
use crate::robot::motion_generator_traits::MotionGeneratorTrait;
use crate::robot::robot_impl::RobotImplementation;
use crate::robot_data::RobotData;
use crate::{
    CartesianPose, CartesianVelocities, ControllerMode, ConvertMotion, Finishable, FrankaResult,
    JointPositions, JointVelocities, RobotState, Torques, DEFAULT_CUTOFF_FREQUENCY,
};
use std::fmt::Debug;
use std::time::Duration;

pub(crate) trait PrivateRobot: Robot
where
    CartesianPose: ConvertMotion<<<Self as Robot>::Data as RobotData>::State>,
    JointVelocities: ConvertMotion<<<Self as Robot>::Data as RobotData>::State>,
    JointPositions: ConvertMotion<<<Self as Robot>::Data as RobotData>::State>,
    CartesianVelocities: ConvertMotion<<<Self as Robot>::Data as RobotData>::State>,
    RobotState: From<<<Self as Robot>::Data as RobotData>::State>,
{
    fn get_rob_mut(&mut self) -> &mut Self::Rob;
    fn get_rob(&self) -> &Self::Rob;
    fn get_net(&mut self) -> &mut Network<Self::Data>;

    fn control_motion_intern<
        F: FnMut(&<<Self as Robot>::Data as RobotData>::State, &Duration) -> U,
        U: ConvertMotion<<<Self as Robot>::Data as RobotData>::State>
            + Debug
            + MotionGeneratorTrait
            + Finishable,
    >(
        &mut self,
        motion_generator_callback: F,
        controller_mode: Option<ControllerMode>,
        limit_rate: Option<bool>,
        cutoff_frequency: Option<f64>,
    ) -> FrankaResult<()> {
        let controller_mode = controller_mode.unwrap_or(ControllerMode::JointImpedance);
        let limit_rate = limit_rate.unwrap_or(true);
        let cutoff_frequency = cutoff_frequency.unwrap_or(DEFAULT_CUTOFF_FREQUENCY);
        let mut control_loop = ControlLoop::from_control_mode(
            self.get_rob_mut(),
            controller_mode,
            motion_generator_callback,
            limit_rate,
            cutoff_frequency,
        )?;
        control_loop.run()
    }

    fn control_torques_intern<
        F: FnMut(&<<Self as Robot>::Data as RobotData>::State, &Duration) -> U,
        U: ConvertMotion<<<Self as Robot>::Data as RobotData>::State>
            + Debug
            + MotionGeneratorTrait
            + Finishable,
    >(
        &mut self,
        motion_generator_callback: F,
        control_callback: &mut dyn FnMut(
            &<<Self as Robot>::Data as RobotData>::State,
            &Duration,
        ) -> Torques,
        limit_rate: Option<bool>,
        cutoff_frequency: Option<f64>,
    ) -> FrankaResult<()> {
        let limit_rate = limit_rate.unwrap_or(true);
        let cutoff_frequency = cutoff_frequency.unwrap_or(DEFAULT_CUTOFF_FREQUENCY);
        let mut control_loop = ControlLoop::new(
            self.get_rob_mut(),
            control_callback,
            motion_generator_callback,
            limit_rate,
            cutoff_frequency,
        )?;
        control_loop.run()
    }
}

pub trait Robot
where
    CartesianPose: ConvertMotion<<<Self as Robot>::Data as RobotData>::State>,
    JointVelocities: ConvertMotion<<<Self as Robot>::Data as RobotData>::State>,
    JointPositions: ConvertMotion<<<Self as Robot>::Data as RobotData>::State>,
    CartesianVelocities: ConvertMotion<<<Self as Robot>::Data as RobotData>::State>,
    RobotState: From<<<Self as Robot>::Data as RobotData>::State>,
{
    type Data: RobotData;
    type Rob: RobotImplementation<Data = Self::Data>;
}
