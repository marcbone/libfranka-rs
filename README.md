[![Crates.io](https://img.shields.io/crates/v/libfranka-rs?style=flat-square)](https://crates.io/crates/libfranka-rs)
[![GitHub Workflow Status](https://img.shields.io/github/actions/workflow/status/marcbone/libfranka-rs/rust.yml?style=flat-square)](https://github.com/marcbone/libfranka-rs/actions)
[![Crates.io](https://img.shields.io/crates/l/libfranka-rs?style=flat-square)](https://github.com/marcbone/libfranka-rs/blob/master/LICENSE)
[![Crates.io](https://img.shields.io/crates/d/libfranka-rs?style=flat-square)](https://crates.io/crates/libfranka-rs)
[![docs.rs](https://img.shields.io/docsrs/libfranka-rs?style=flat-square)](https://docs.rs/libfranka-rs)
[![Codecov](https://img.shields.io/codecov/c/github/marcbone/libfranka-rs?style=flat-square)](https://codecov.io/gh/marcbone/libfranka-rs)

# libfranka-rs
libfranka-rs is an **unofficial** port of [libfranka](https://github.com/frankaemika/libfranka) written in Rust.
This library can interact with research versions of Franka Emika Robots (Panda and FR3).
The library aims to provide researchers the possibility to experiment with Rust within a real-time robotics
application.
 
 **ALWAYS HAVE THE USER STOP BUTTON AT HAND WHILE CONTROLLING
THE ROBOT!**

## Features
 * Real-time control of the robot
 * A libfranka-like API
 * Generic over Panda and FR3
 * Usage with Preempt_RT or stock Linux kernel
 * Usage of the gripper
 * Usage of the robot model
 * Download the robot model for offline usage
 * Ports of the libfranka examples to help you to get started
 * The functionality of the [example_commons](https://github.com/frankaemika/libfranka/blob/master/examples/examples_common.cpp) is directly part of the library, so you do not have to copy these files to your project
 * Direct Conversions from [nalgebra](https://nalgebra.org/) (Eigen3 equivalent) types into libfranka control types (JointPositions, CartesianPose, ...)
 * Proper error handling with Result types
 
Not supported:
 * Windows (macOS could maybe work, but I have not tested it)
 * Vacuum Grippers (we do not have those, so I cannot test them)

## Example
A small example for controlling joint positions. You can find more in the examples folder.
```rust
 use std::time::Duration;
 use std::f64::consts::PI;
 use franka::{JointPositions, MotionFinished, Robot, RobotState, Fr3, FrankaResult};
 fn main() -> FrankaResult<()> {
     let mut robot = Fr3::new("robotik-bs.de", None, None)?;
     robot.set_default_behavior()?;
     robot.set_collision_behavior([20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0], [20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0],
                                  [20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0], [20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0],
                                  [20.0, 20.0, 20.0, 25.0, 25.0, 25.0], [20.0, 20.0, 20.0, 25.0, 25.0, 25.0],
                                  [20.0, 20.0, 20.0, 25.0, 25.0, 25.0], [20.0, 20.0, 20.0, 25.0, 25.0, 25.0])?;
     let q_goal = [0., -PI / 4., 0., -3. * PI / 4., 0., PI / 2., PI / 4.];
     robot.joint_motion(0.5, &q_goal)?;
     let mut initial_position = JointPositions::new([0.0; 7]);
     let mut time = 0.;
     let callback = |state: &RobotState, time_step: &Duration| -> JointPositions {
         time += time_step.as_secs_f64();
         if time == 0. {
             initial_position.q = state.q_d;
         }
         let mut out = JointPositions::new(initial_position.q);
         let delta_angle = PI / 8. * (1. - f64::cos(PI / 2.5 * time));
         out.q[3] += delta_angle;
         out.q[4] += delta_angle;
         out.q[6] += delta_angle;
         if time >= 5.0 {
             return out.motion_finished();
         }
         out
     };
     robot.control_joint_positions(callback, None, None, None) 
 }
```

It is also possible to write the example above in a generic way, so it can be used for FR3 or Panda:
```rust
use franka::{Fr3, FrankaResult, JointPositions, MotionFinished, Panda, Robot, RobotState};
use std::f64::consts::PI;
use std::time::Duration;

fn generate_motion<R: Robot>(robot: &mut R) -> FrankaResult<()> {
    robot.set_default_behavior()?;
    robot.set_collision_behavior(
        [20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0],
        [20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0],
        [20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0],
        [20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0],
        [20.0, 20.0, 20.0, 25.0, 25.0, 25.0],
        [20.0, 20.0, 20.0, 25.0, 25.0, 25.0],
        [20.0, 20.0, 20.0, 25.0, 25.0, 25.0],
        [20.0, 20.0, 20.0, 25.0, 25.0, 25.0],
    )?;
    let q_goal = [0., -PI / 4., 0., -3. * PI / 4., 0., PI / 2., PI / 4.];
    robot.joint_motion(0.5, &q_goal)?;
    let mut initial_position = JointPositions::new([0.0; 7]);
    let mut time = 0.;
    let callback = |state: &RobotState, time_step: &Duration| -> JointPositions {
        time += time_step.as_secs_f64();
        if time == 0. {
            initial_position.q = state.q_d;
        }
        let mut out = JointPositions::new(initial_position.q);
        let delta_angle = PI / 8. * (1. - f64::cos(PI / 2.5 * time));
        out.q[3] += delta_angle;
        out.q[4] += delta_angle;
        out.q[6] += delta_angle;
        if time >= 5.0 {
            return out.motion_finished();
        }
        out
    };
    robot.control_joint_positions(callback, None, None, None)
}
fn main() -> FrankaResult<()> {
    let mut fr3 = Fr3::new("robotik-bs.de", None, None)?;
    let mut panda = Panda::new("robotik-bs.de", None, None)?;
    generate_motion(&mut fr3)?;
    generate_motion(&mut panda)
}
```



## How to get started

Since libfranka-rs is a port of libfranka, you may find the
[Franka Control Interface Documentation](https://frankaemika.github.io/docs/index.html) helpful.

### With zero Rust knowledge
If this is your first time using Rust, I recommend reading the [Rust Book](https://doc.rust-lang.org/stable/book/).

If you have Rust installed and just want to play with the examples, you can also run:
```bash
cargo install libfranka-rs --examples --version 0.10.0
generate_joint_position_motion <ip_of_your_robot>
```

If you are already familiar with the original libfranka examples. I suggest you take a look at the examples folder.
The examples that are named like an original libfranka example are ports that stay as close to the original as possible,
hoping that it makes your introduction into the Rust world as smooth as possible.



### With zero libfranka knowledge
The [Franka Control Interface Documentation](https://frankaemika.github.io/docs/index.html) also includes a setup guide.
You can skip the installation of libfranka as you will be using libfranka-rs.
Take a look at the [Documentation](https://docs.rs/libfranka-rs) and the [examples](examples) folder. You should run the
[communication_test](examples/communication_test.rs) example to verify that your setup is correct.

### How to use libfranka-rs
If you want to use libfranka-rs in your project, you have to add
```text
libfranka-rs = "0.10.0"
```
to your Cargo.toml file.

## Licence
This library is copyrighted © 2021 Marco Boneberger


Licensed under the EUPL, Version 1.2 or – as soon they will be approved by the European Commission - subsequent versions of the EUPL (the "Licence");

You may not use this work except in compliance with the Licence.
You may obtain a copy of the Licence at:

[https://joinup.ec.europa.eu/software/page/eupl](https://joinup.ec.europa.eu/software/page/eupl)
 
Unless required by applicable law or agreed to in writing, software distributed under the Licence is distributed on an "AS IS" basis
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.

See the Licence for the specific language governing permissions and limitations under the Licence.
