// Copyright (c) 2021 Marco Boneberger
// Licensed under the EUPL-1.2-or-later
use std::f64::consts::PI;
use std::time::Duration;

use clap::Parser;

use franka::{FrankaResult, MotionFinished, Panda, RobotState, RobotWrapper, Torques, FR3};

/// An example indicating the network performance.
///
/// WARNING: Before executing this example, make sure there is enough space in front of the robot.
#[derive(Parser, Debug)]
#[clap(author, version, name = "communication_test")]
struct CommandLineArguments {
    /// IP-Address or hostname of the robot
    pub franka_ip: String,
    // Use this option to run the example on a Panda
    #[clap(short, long, action)]
    pub panda: bool,
}

fn main() -> FrankaResult<()> {
    let address = CommandLineArguments::parse();
    match address.panda {
        true => {
            let robot = Panda::new(address.franka_ip.as_str(), None, None)?;
            generate_motion(robot)
        }
        false => {
            let robot = FR3::new(address.franka_ip.as_str(), None, None)?;
            generate_motion(robot)
        }
    }
}

fn generate_motion<R: RobotWrapper>(mut robot: R) -> FrankaResult<()> {
    let q_goal = [0., -PI / 4., 0., -3. * PI / 4., 0., PI / 2., PI / 4.];

    println!("WARNING: This example will move the robot! Please make sure to have the user stop button at hand!");
    println!("Press Enter to continue...");
    std::io::stdin().read_line(&mut String::new()).unwrap();

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

    robot.joint_motion(0.5, &q_goal)?;
    let zero_torques = Torques::new([0.; 7]);
    let mut time: u64 = 0;
    let mut counter: u64 = 0;
    let mut avg_success_rate = 0.;
    let mut min_success_rate = 1.;
    let mut max_success_rate = 0.;

    let callback = |state: &RobotState, time_step: &Duration| -> Torques {
        time += time_step.as_millis() as u64;
        if time == 0 {
            return zero_torques;
        }
        counter += 1;
        if counter % 100 == 0 {
            println!(
                "#{} Current success rate: {}",
                counter, state.control_command_success_rate
            );
        }
        std::thread::sleep(Duration::from_micros(100));
        avg_success_rate += state.control_command_success_rate;
        if state.control_command_success_rate > max_success_rate {
            max_success_rate = state.control_command_success_rate
        }
        if state.control_command_success_rate < min_success_rate {
            min_success_rate = state.control_command_success_rate
        }
        if time >= 10000 {
            return zero_torques.motion_finished();
        }
        zero_torques
    };
    robot.control_torques(callback, false, 1000.)?;

    avg_success_rate /= counter as f64;
    let lost_robot_states = time - counter;
    println!("#######################################################");
    if lost_robot_states > 0 {
        println!(
            "The control loop did not get executed {} times in the",
            lost_robot_states
        );
        println!(
            "last {} milliseconds! (lost {} robot_states)",
            time, lost_robot_states
        );
        println!();
    }
    println!("Control command success rate of {} samples: ", counter);
    println!("Max: {}", max_success_rate);
    println!("Avg: {}", avg_success_rate);
    println!("Min {}", min_success_rate);

    if avg_success_rate < 0.9 {
        println!();
        println!("WARNING: THIS SETUP IS PROBABLY NOT SUFFICIENT FOR FCI!");
        println!("PLEASE TRY OUT A DIFFERENT PC / NIC");
    } else if avg_success_rate < 0.95 {
        println!();
        println!("WARNING: MANY PACKETS GOT LOST!");
        println!("PLEASE INSPECT YOUR SETUP AND FOLLOW ADVICE ON");
        println!("https://frankaemika.github.io/docs/troubleshooting.html")
    }
    println!("#######################################################");
    Ok(())
}
