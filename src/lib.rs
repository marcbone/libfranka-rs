// Copyright (c) 2021 Marco Boneberger
// Licensed under the EUPL-1.2-or-later

//! # libfranka-rs
//! libfranka-rs is a library to control [Franka Emika](https://franka.de) research robots.
//! It is an unofficial port of [libfranka](https://github.com/frankaemika/libfranka).
//!
//! **ALWAYS HAVE THE USER STOP BUTTON AT
//! HAND WHILE CONTROLLING THE ROBOT!**
//!
//!
//! ## Design
//! The design of this library is very similar to the original library. All files are
//! named like their libfranka counterparts and contain the same functionality apart from some
//! exceptions.
//!
//! The library is divided into three main Modules:
//! * [gripper](`crate::gripper`) - contains everything which is only needed for controlling the gripper.
//! * [model](`crate::model`) -  contains everything needed for using the model.
//! * [robot](`crate::robot`) - contains everything which is only needed for controlling the robot.
//!
//! # Example:
//!```no_run
//! use std::time::Duration;
//! use std::f64::consts::PI;
//! use franka::{JointPositions, MotionFinished, RobotState, Panda, FrankaResult};
//! fn main() -> FrankaResult<()> {
//! let mut robot = Panda::new("robotik-bs.de", None, None)?;
//!     robot.set_default_behavior()?;
//!     robot.set_collision_behavior([20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0], [20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0],
//!                                  [20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0], [20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0],
//!                                  [20.0, 20.0, 20.0, 25.0, 25.0, 25.0], [20.0, 20.0, 20.0, 25.0, 25.0, 25.0],
//!                                  [20.0, 20.0, 20.0, 25.0, 25.0, 25.0], [20.0, 20.0, 20.0, 25.0, 25.0, 25.0])?;
//!     let q_goal = [0., -PI / 4., 0., -3. * PI / 4., 0., PI / 2., PI / 4.];
//!     robot.joint_motion(0.5, &q_goal)?;
//!     let mut initial_position = JointPositions::new([0.0; 7]);
//!     let mut time = 0.;
//!     let callback = |state: &RobotState, time_step: &Duration| -> JointPositions {
//!         time += time_step.as_secs_f64();
//!         if time == 0. {
//!             initial_position.q = state.q_d;
//!         }
//!         let mut out = JointPositions::new(initial_position.q);
//!         let delta_angle = PI / 8. * (1. - f64::cos(PI / 2.5 * time));
//!         out.q[3] += delta_angle;
//!         out.q[4] += delta_angle;
//!         out.q[6] += delta_angle;
//!         if time >= 5.0 {
//!             return out.motion_finished();
//!         }
//!         out
//!     };
//!     robot.control_joint_positions(callback, None, None, None)
//! }
//!   ```
//!
//! The main function returns a FrankaResult<()> which means that it returns either Ok(())
//! or an Error of type FrankaException which correspond to the C++ exceptions in libfranka.
//!
//!```no_run
//! # use std::time::Duration;
//! # use std::f64::consts::PI;
//! # use franka::{JointPositions, MotionFinished, RobotState, Panda, FrankaResult};
//! # fn main() -> FrankaResult<()> {
//! let mut robot = Robot::new("robotik-bs.de", None, None)?;
//! # Ok(())
//! # }
//! ```
//! connects with the robot. You can either provide an IP Address or a hostname. The other options
//! are for setting the RealtimeConfig and the logger size. But we are happy with the defaults so
//! we set them to none. With the "?" we forward eventual errors like "Connection refused" for example
//!
//!
//! ```ignore
//! robot.set_default_behavior()?;
//! robot.set_collision_behavior(...)?;
//! ```
//! this specifies the default collision behavior, joint impedance and Cartesian impedance
//!
//!```no_run
//! # use std::time::Duration;
//! # use std::f64::consts::PI;
//! # use franka::{JointPositions, MotionFinished, RobotState, Robot, FrankaResult};
//! # fn main() -> FrankaResult<()> {
//! # let mut robot = Robot::new("robotik-bs.de", None, None)?;
//! let q_goal = [0., -PI / 4., 0., -3. * PI / 4., 0., PI / 2., PI / 4.];
//! robot.joint_motion(0.5, &q_goal)?;
//! # Ok(())
//! # }
//! ```
//! These lines specify a joint position goal and moves the robot at 50% of its max speed towards
//! the goal. In this case we use it to bring the robot into his home position
//!
//! ```no_run
//! # use franka::JointPositions;
//! let mut initial_position = JointPositions::new([0.0; 7]);
//! ```
//! Here we define a variable for the initial joint position. You cannot set the right values
//! here because they will change until you are in the control loop. Therefore it is necessary
//! specify them at the first time the control loop is executed like here:
//! ```no_run
//! #  use franka::{JointPositions, RobotState};
//! #  let mut initial_position = JointPositions::new([0.0; 7]);
//! # let time = 0.;
//! # let state = RobotState::default();
//! if time == 0. {
//!     initial_position.q = state.q_d;
//! }
//! ```
//!
//! ```no_run
//! # use franka::JointPositions;
//! # use std::f64::consts::PI;
//! # let time = 0.;
//! # let delta_angle = 0.;
//! #  let mut initial_position = JointPositions::new([0.0; 7]);
//! let mut out = JointPositions::new(initial_position.q);
//! let delta_angle = PI / 8. * (1. - f64::cos(PI / 2.5 * time));
//! out.q[3] += delta_angle;
//! out.q[4] += delta_angle;
//! out.q[6] += delta_angle;
//! ```
//! Here we define our desired JointPositions. It is important that we provide a smooth signal
//! to the robot, therefore we use a cosine function. Without it we would get a CommandException
//! while running.
//!
//!```no_run
//! # use franka::{Finishable, JointPositions};
//! # fn joint_positions() -> JointPositions {
//! # let time = 0.;
//! # let mut out = JointPositions::new([0.;7]);
//!
//! if time >= 5.0 {
//!     return out.motion_finished();
//! }
//! out
//! # }
//! ```
//! Until 5 seconds have passed we just return our JointPositions. As we want to stop the control
//! loop after 5 seconds. We tell the robot that this is our last command and that we want to exit
//! the control loop.
//!
//!
//! All of this we put inside our closure (they are called lambda-functions in C++) callback
//! ```ignore
//! let callback = |state: &RobotState, time_step: &Duration| -> JointPositions {...}
//! ```
//! Our callback Takes a immutable Reference to a RobotState and a  immutable reference to the passed time
//! since the callback was executed the last time (the time is zero at the first call of the function)
//! and returns JointPositions.
//!
//! With this callback we can now control the joint positions of the robot:
//! ```no_run
//! # use std::time::Duration;
//! # use std::f64::consts::PI;
//! # use franka::{JointPositions, MotionFinished, RobotState, Robot, FrankaResult};
//! # fn main() -> FrankaResult<()> {
//! # let mut robot = Robot::new("robotik-bs.de", None, None)?;
//! # let callback = |state: &RobotState, time_step: &Duration| -> JointPositions {JointPositions::new([0.;7])};
//! robot.control_joint_positions(callback, None, None, None)
//! # }
//! ```
//! There are optional arguments for specifying the Controller mode, rate limiting and the cutoff frequency.
//! As we are happy with the defaults we set them to None. Note that this function returns a FrankaResult.
//! As it is the last statement of the main method we do not have to forward the error with a "?".
pub mod exception;
pub mod gripper;
mod network;
pub mod robot;

pub mod model;
pub mod utils;

pub use exception::FrankaResult;
pub use gripper::gripper_state::GripperState;
pub use gripper::Gripper;
pub use model::FR3Model;
pub use model::Frame;
pub use model::PandaModel;
pub use model::RobotModel;
pub use network::RobotData;
pub use robot::control_types::*;
pub use robot::low_pass_filter::DEFAULT_CUTOFF_FREQUENCY;
pub use robot::low_pass_filter::MAX_CUTOFF_FREQUENCY;
pub use robot::robot_state::RobotState;
pub use robot::Panda;
pub use utils::*;
