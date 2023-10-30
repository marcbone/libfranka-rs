// Copyright (c) 2021 Marco Boneberger
// Licensed under the EUPL-1.2-or-later

//! contains useful type definitions and conversion functions.
use crate::robot::control_types::JointPositions;
use crate::robot::robot_state::AbstractRobotState;
use crate::Finishable;
use nalgebra::{Isometry3, Matrix4, Rotation3, SMatrix, SVector, Vector3};
use std::time::Duration;

/// converts a 4x4 column-major homogenous matrix to an Isometry
pub fn array_to_isometry(array: &[f64; 16]) -> Isometry3<f64> {
    let rot = Rotation3::from_matrix(
        &Matrix4::from_column_slice(array)
            .remove_column(3)
            .remove_row(3),
    );
    Isometry3::from_parts(
        Vector3::new(array[12], array[13], array[14]).into(),
        rot.into(),
    )
}
/// A Vector with 7 entries
pub type Vector7 = SVector<f64, 7>;
/// A Matrix with 6 rows and 7 columns
pub type Matrix6x7 = SMatrix<f64, 6, 7>;
/// A Matrix with 7 rows and 7 columns
pub type Matrix7 = SMatrix<f64, 7, 7>;
/// An example showing how to generate a joint pose motion to a goal position. Adapted from:
/// Wisama Khalil and Etienne Dombre. 2002. Modeling, Identification and Control of Robots
/// (Kogan Page Science Paper edition).
#[derive(Debug)]
pub struct MotionGenerator {
    q_goal: Vector7,
    q_start: Vector7,
    delta_q: Vector7,
    dq_max_sync: Vector7,
    t_1_sync: Vector7,
    t_2_sync: Vector7,
    t_f_sync: Vector7,
    q_1: Vector7,

    time: f64,
    dq_max: Vector7,
    ddq_max_start: Vector7,
    ddq_max_goal: Vector7,
}

impl MotionGenerator {
    const DELTA_Q_MOTION_FINISHED: f64 = 1e-6;
    /// Creates a new  MotionGenerator instance for a target q.
    ///
    /// # Arguments
    /// * `speed_factor` - General speed factor in range [0, 1].
    /// * `q_goal` - Target joint positions.
    pub fn new(speed_factor: f64, q_goal: &[f64; 7]) -> Self {
        let time = 0.;
        let q_goal = Vector7::from_row_slice(q_goal);
        let dq_max = Vector7::from_row_slice(&[2.0, 2.0, 2.0, 2.0, 2.5, 2.5, 2.5]) * speed_factor;
        let ddq_max_start = Vector7::from_row_slice(&[5.; 7]) * speed_factor;
        let ddq_max_goal = Vector7::from_row_slice(&[5.; 7]) * speed_factor;

        let dq_max_sync = Vector7::zeros();
        let q_start = Vector7::zeros();
        let delta_q = Vector7::zeros();
        let t_1_sync = Vector7::zeros();
        let t_2_sync = Vector7::zeros();
        let t_f_sync = Vector7::zeros();
        let q_1 = Vector7::zeros();
        MotionGenerator {
            q_goal,
            q_start,
            delta_q,
            dq_max_sync,
            t_1_sync,
            t_2_sync,
            t_f_sync,
            q_1,
            time,
            dq_max,
            ddq_max_start,
            ddq_max_goal,
        }
    }
    fn calculate_desired_values(&self, t: f64, delta_q_d: &mut Vector7) -> bool {
        let sign_delta_q = Vector7::from_iterator(self.delta_q.iter().map(|&x| x.signum()));
        let t_d = self.t_2_sync - self.t_1_sync;
        let delta_t_2_sync = self.t_f_sync - self.t_2_sync;
        let mut joint_motion_finished = [false; 7];
        for i in 0..7 {
            if self.delta_q[i].abs() < MotionGenerator::DELTA_Q_MOTION_FINISHED {
                delta_q_d[i] = 0.;
                joint_motion_finished[i] = true;
            } else if t < self.t_1_sync[i] {
                delta_q_d[i] = -1. / self.t_1_sync[i].powi(3)
                    * self.dq_max_sync[i]
                    * sign_delta_q[i]
                    * (0.5 * t - self.t_1_sync[i])
                    * t.powi(3);
            } else if t >= self.t_1_sync[i] && t < self.t_2_sync[i] {
                delta_q_d[i] =
                    self.q_1[i] + (t - self.t_1_sync[i]) * self.dq_max_sync[i] * sign_delta_q[i];
            } else if t >= self.t_2_sync[i] && t < self.t_f_sync[i] {
                delta_q_d[i] = self.delta_q[i]
                    + 0.5
                        * (1. / delta_t_2_sync[i].powi(3)
                            * (t - self.t_1_sync[i] - 2.0 * delta_t_2_sync[i] - t_d[i])
                            * (t - self.t_1_sync[i] - t_d[i]).powi(3)
                            + (2. * t - 2. * self.t_1_sync[i] - delta_t_2_sync[i] - 2. * t_d[i]))
                        * self.dq_max_sync[i]
                        * sign_delta_q[i];
            } else {
                delta_q_d[i] = self.delta_q[i];
                joint_motion_finished[i] = true;
            }
        }
        joint_motion_finished.iter().all(|i| *i)
    }
    fn calculate_synchronized_values(&mut self) {
        let mut dq_max_reach = self.dq_max;
        let mut t_f: Vector7 = Vector7::zeros();
        let mut delta_t_2 = Vector7::zeros();
        let mut t_1 = Vector7::zeros();

        let mut delta_t_2_sync = Vector7::zeros();
        let sign_delta_q = Vector7::from_iterator(self.delta_q.iter().map(|&x| x.signum()));
        for i in 0..7 {
            if self.delta_q[i].abs() > MotionGenerator::DELTA_Q_MOTION_FINISHED {
                if self.delta_q[i].abs()
                    < (3.0 / 4.0 * (self.dq_max[i].powi(2) / self.ddq_max_start[i])
                        + 3. / 4. * (self.dq_max[i].powi(2) / self.ddq_max_goal[i]))
                {
                    dq_max_reach[i] = f64::sqrt(
                        4. / 3.
                            * self.delta_q[i]
                            * sign_delta_q[i]
                            * (self.ddq_max_start[i] * self.ddq_max_goal[i])
                            / (self.ddq_max_start[i] + self.ddq_max_goal[i]),
                    )
                }
                t_1[i] = 1.5 * dq_max_reach[i] / self.ddq_max_start[i];
                delta_t_2[i] = 1.5 * dq_max_reach[i] / self.ddq_max_goal[i];
                t_f[i] =
                    t_1[i] / 2. + delta_t_2[i] / 2. + f64::abs(self.delta_q[i]) / dq_max_reach[i];
            }
        }
        let max_t_f = t_f.max();
        for i in 0..7 {
            if self.delta_q[i].abs() > MotionGenerator::DELTA_Q_MOTION_FINISHED {
                let a = 1.5 / 2. * (self.ddq_max_goal[i] + self.ddq_max_start[i]);
                let b = -1. * max_t_f * self.ddq_max_goal[i] * self.ddq_max_start[i];
                let c = f64::abs(self.delta_q[i]) * self.ddq_max_goal[i] * self.ddq_max_start[i];
                let delta = f64::max(b * b - 4. * a * c, 0.);
                self.dq_max_sync[i] = (-1. * b - f64::sqrt(delta)) / (2. * a);
                self.t_1_sync[i] = 1.5 * self.dq_max_sync[i] / self.ddq_max_start[i];
                delta_t_2_sync[i] = 1.5 * self.dq_max_sync[i] / self.ddq_max_goal[i];
                self.t_f_sync[i] = self.t_1_sync[i] / 2.
                    + delta_t_2_sync[i] / 2.
                    + f64::abs(self.delta_q[i] / self.dq_max_sync[i]);
                self.t_2_sync[i] = self.t_f_sync[i] - delta_t_2_sync[i];
                self.q_1[i] = self.dq_max_sync[i] * sign_delta_q[i] * (0.5 * self.t_1_sync[i])
            }
        }
    }
    /// Sends joint position calculations
    ///
    /// # Arguments
    /// `robot_state` -  Current state of the robot.
    /// `period` - Duration of execution.
    ///
    /// # Return
    /// Joint positions for use inside a control loop.
    pub fn generate_motion<State: AbstractRobotState>(
        &mut self,
        robot_state: &State,
        period: &Duration,
    ) -> JointPositions {
        self.time += period.as_secs_f64();

        if self.time == 0. {
            self.q_start = Vector7::from_column_slice(&robot_state.get_q_d());
            self.delta_q = self.q_goal - self.q_start;
            self.calculate_synchronized_values();
        }
        let mut delta_q_d = Vector7::zeros();
        let motion_finished = self.calculate_desired_values(self.time, &mut delta_q_d);

        let mut output = JointPositions::new((self.q_start + delta_q_d).into());
        output.set_motion_finished(motion_finished);
        output
    }
}

#[cfg(test)]
mod test {
    use crate::{array_to_isometry, Finishable, MotionGenerator, RobotState};
    use nalgebra::Rotation3;
    use std::time::Duration;

    fn slice_compare(a: &[f64], b: &[f64], thresh: f64) {
        for i in 0..a.len() {
            float_compare(a[i], b[i], thresh);
        }
    }

    fn float_compare(a: f64, b: f64, thresh: f64) {
        assert!((a - b).abs() < thresh);
    }

    #[test]
    fn motion_generator() {
        let q_des = [
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [
                0.005530250638946291,
                0.005530250638946291,
                0.005530250638946291,
                0.005530250638946291,
                0.005530250638946291,
                0.005530250638946291,
                0.005530250638946291,
            ],
            [
                0.039797560667125885,
                0.039797560667125885,
                0.039797560667125885,
                0.039797560667125885,
                0.039797560667125885,
                0.039797560667125885,
                0.039797560667125885,
            ],
            [
                0.11931676725154987,
                0.11931676725154987,
                0.11931676725154987,
                0.11931676725154987,
                0.11931676725154987,
                0.11931676725154987,
                0.11931676725154987,
            ],
            [
                0.24726937422589595,
                0.24726937422589595,
                0.24726937422589595,
                0.24726937422589595,
                0.24726937422589595,
                0.24726937422589595,
                0.24726937422589595,
            ],
            [
                0.4135035520905085,
                0.4135035520905085,
                0.4135035520905085,
                0.4135035520905085,
                0.4135035520905085,
                0.4135035520905085,
                0.4135035520905085,
            ],
            [
                0.5946171259214752,
                0.5946171259214752,
                0.5946171259214752,
                0.5946171259214752,
                0.5946171259214752,
                0.5946171259214752,
                0.5946171259214752,
            ],
            [
                0.7595171113649393,
                0.7595171113649393,
                0.7595171113649393,
                0.7595171113649393,
                0.7595171113649393,
                0.7595171113649393,
                0.7595171113649393,
            ],
            [
                0.8853832940789487,
                0.8853832940789487,
                0.8853832940789487,
                0.8853832940789487,
                0.8853832940789487,
                0.8853832940789487,
                0.8853832940789487,
            ],
            [
                0.9626711625624701,
                0.9626711625624701,
                0.9626711625624701,
                0.9626711625624701,
                0.9626711625624701,
                0.9626711625624701,
                0.9626711625624701,
            ],
            [
                0.9951695386478036,
                0.9951695386478036,
                0.9951695386478036,
                0.9951695386478036,
                0.9951695386478036,
                0.9951695386478036,
                0.9951695386478036,
            ],
            [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0],
        ];
        let mut state = RobotState::default();
        let mut motion_generator = MotionGenerator::new(1.0, &[1.; 7]);
        let mut joint_pos = motion_generator.generate_motion(&state, &Duration::from_secs_f64(0.0));
        slice_compare(&joint_pos.q, &q_des[0], 1e-10);
        let mut counter = 1;
        while !joint_pos.is_finished() {
            state.q_d = joint_pos.q;
            joint_pos = motion_generator.generate_motion(&state, &Duration::from_secs_f64(0.1));
            slice_compare(&joint_pos.q, &q_des[counter], 1e-10);
            counter += 1;
        }
        assert_eq!(counter, q_des.len())
    }

    #[test]
    fn array_to_isometry_nan_test_1() {
        let o_t_ee_c = [
            1.0,
            -0.000000011046552201160267,
            0.000000008312911920110592,
            0.0,
            -0.000000011046552077288223,
            -0.9999999999999999,
            -0.000000014901161362226844,
            0.0,
            0.000000008312912084717049,
            0.000000014901161270397825,
            -0.9999999999999999,
            0.0,
            0.30689056578595225,
            -0.000000003883240449999549,
            0.486882056335292,
            1.0,
        ];
        let last_o_t_ee_c = [
            0.9999903734042683,
            -0.000000011046444370332864,
            0.00000000831299559264306,
            0.0,
            -0.000000011046444370332864,
            -0.9999903734042683,
            -0.0000000021131709957245164,
            0.0,
            0.000000008313075597526054,
            0.0000000021131912467324254,
            -0.9999999999999999,
            0.0,
            0.30689056578595225,
            -0.000000003883240449999549,
            0.486882056335292,
            1.0,
        ];
        let commanded_pose = array_to_isometry(&o_t_ee_c);
        let last_commanded_pose = array_to_isometry(&last_o_t_ee_c);

        let mut rot_diff: Rotation3<f64> = commanded_pose.rotation.to_rotation_matrix()
            * last_commanded_pose
                .rotation
                .to_rotation_matrix()
                .transpose();
        rot_diff.renormalize();
        for i in rot_diff.matrix().iter() {
            assert!(i.is_finite());
        }
    }

    #[test]
    fn array_to_isometry_nan_test_2() {
        let y = [
            0.9999903734042686,
            0.0000000002540163079878255,
            -0.00000000012581154368346085,
            0.0,
            0.0000000002540163079878255,
            -0.9999903734042686,
            0.00000000004614105974113725,
            0.0,
            -0.00000000012581275483128821,
            -0.000000000046141503958700795,
            -1.0,
            0.0,
            0.30689056659844144,
            0.0000000000692086410879149,
            0.4868820527992277,
            1.0,
        ];
        assert!(array_to_isometry(&y).rotation.angle().is_finite())
    }
}
