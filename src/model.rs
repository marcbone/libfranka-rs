// Copyright (c) 2021 Marco Boneberger
// Licensed under the EUPL-1.2-or-later

//! Contains model library types.
use nalgebra::Matrix4;

use crate::model::model_library::ModelLibrary;
use crate::robot::robot_state::PandaState;
use crate::FrankaResult;
use std::fmt;
use std::path::Path;

pub(crate) mod library_downloader;
mod model_library;

/// Enumerates the seven joints, the flange, and the end effector of a robot.
pub enum Frame {
    Joint1,
    Joint2,
    Joint3,
    Joint4,
    Joint5,
    Joint6,
    Joint7,
    Flange,
    EndEffector,
    Stiffness,
}

impl fmt::Display for Frame {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        match self {
            Frame::Joint1 => {
                write!(f, "Joint 1")
            }
            Frame::Joint2 => {
                write!(f, "Joint 2")
            }
            Frame::Joint3 => {
                write!(f, "Joint 3")
            }
            Frame::Joint4 => {
                write!(f, "Joint 4")
            }
            Frame::Joint5 => {
                write!(f, "Joint 5")
            }
            Frame::Joint6 => {
                write!(f, "Joint 6")
            }
            Frame::Joint7 => {
                write!(f, "Joint 7")
            }
            Frame::Flange => {
                write!(f, "Flange")
            }
            Frame::EndEffector => {
                write!(f, "End-Effector")
            }
            Frame::Stiffness => {
                write!(f, "Stiffness")
            }
        }
    }
}

/// Calculates poses of joints and dynamic properties of the robot.
pub struct Model {
    library: ModelLibrary,
}

#[allow(non_snake_case)]
impl Model {
    /// Creates a new Model instance from the shared object of the Model for offline usage.
    ///
    /// If you just want to use the model to control the Robot you should use
    /// [`Robot::load_model`](`crate::Robot::load_model`).
    ///
    /// If you do not have the model you can use the download_model example to download the model
    /// # Arguments
    /// * `model_filename` - Path to the model.
    /// * `libm_filename` - Path to libm.so.6 . You probably do not need to specify the path as it
    /// it should be detected automatically. However if you get panics for unresolved symbols or libm could
    /// not be found you have to specify the path. On most Ubuntu systems it is in
    /// ```text
    /// /lib/x86_64-linux-gnu/libm.so.6
    /// ```
    /// It maybe has a different name on your machine but it is not the "libm.so". You can
    /// verify that it is the correct libm by checking:
    /// ```bash
    ///  nm -D /lib/x86_64-linux-gnu/libm.so.6 | grep sincos
    /// ```
    /// # Errors
    ///  * ModelException
    /// # libm FAQ
    /// What is libm? - Libm is the Math library of the C Standard Library. It is what you get when
    /// you include <math.h> in C
    ///
    /// Why do we need it? - Because the model is not self contained and relies on some functions from libm
    ///
    /// How does the libfranka embed libm? They are not including <math.h> - Apparently it gets somehow included when using \<array> ¯\_(ツ)_/¯
    pub fn new<S: AsRef<Path>>(
        model_filename: S,
        libm_filename: Option<&Path>,
    ) -> FrankaResult<Self> {
        Ok(Model {
            library: ModelLibrary::new(model_filename.as_ref(), libm_filename)?,
        })
    }
    /// Gets the 4x4 pose matrix for the given frame in base frame.
    ///
    /// The pose is represented as a 4x4 matrix in column-major format.
    /// # Arguments
    /// * `frame` - The desired frame.
    /// * `q` - Joint position.
    /// * `F_T_EE` - End effector in flange frame.
    /// * `EE_T_K` - Stiffness frame K in the end effector frame.
    /// # Return
    /// Vectorized 4x4 pose matrix, column-major.
    pub fn pose(
        &self,
        frame: &Frame,
        q: &[f64; 7],
        F_T_EE: &[f64; 16],
        EE_T_K: &[f64; 16],
    ) -> [f64; 16] {
        let mut output = [0.; 16];
        match frame {
            Frame::Joint1 => self.library.joint1(q, &mut output),
            Frame::Joint2 => self.library.joint2(q, &mut output),
            Frame::Joint3 => self.library.joint3(q, &mut output),
            Frame::Joint4 => self.library.joint4(q, &mut output),
            Frame::Joint5 => self.library.joint5(q, &mut output),
            Frame::Joint6 => self.library.joint6(q, &mut output),
            Frame::Joint7 => self.library.joint7(q, &mut output),
            Frame::Flange => self.library.flange(q, &mut output),
            Frame::EndEffector => self.library.ee(q, F_T_EE, &mut output),
            Frame::Stiffness => {
                let tmp: Matrix4<f64> =
                    Matrix4::from_column_slice(F_T_EE) * Matrix4::from_column_slice(EE_T_K);
                let mut stiffness_f_t_ee = [0.; 16];
                tmp.iter()
                    .enumerate()
                    .for_each(|(i, &x)| stiffness_f_t_ee[i] = x);
                self.library.ee(q, &stiffness_f_t_ee, &mut output)
            }
        }
        output
    }
    /// Gets the 4x4 pose matrix for the given frame in base frame.
    ///
    /// The pose is represented as a 4x4 matrix in column-major format.
    /// # Arguments
    /// * `frame` - The desired frame.
    /// * `robot_state` - State from which the pose should be calculated.
    /// # Return
    /// Vectorized 4x4 pose matrix, column-major.
    pub fn pose_from_state(&self, frame: &Frame, robot_state: &PandaState) -> [f64; 16] {
        self.pose(
            frame,
            &robot_state.q,
            &robot_state.F_T_EE,
            &robot_state.EE_T_K,
        )
    }
    /// Gets the 6x7 Jacobian for the given frame, relative to that frame.
    ///
    /// The Jacobian is represented as a 6x7 matrix in column-major format.
    /// # Arguments
    /// * `frame` - The desired frame.
    /// * `q` - Joint position.
    /// * `F_T_EE` - End effector in flange frame.
    /// * `EE_T_K` - Stiffness frame K in the end effector frame.
    /// # Return
    /// Vectorized 6x7 Jacobian, column-major.
    pub fn body_jacobian(
        &self,
        frame: &Frame,
        q: &[f64; 7],
        F_T_EE: &[f64; 16],
        EE_T_K: &[f64; 16],
    ) -> [f64; 42] {
        let mut output = [0.; 42];
        match frame {
            Frame::Joint1 => self.library.body_jacobian_joint1(&mut output),
            Frame::Joint2 => self.library.body_jacobian_joint2(q, &mut output),
            Frame::Joint3 => self.library.body_jacobian_joint3(q, &mut output),
            Frame::Joint4 => self.library.body_jacobian_joint4(q, &mut output),
            Frame::Joint5 => self.library.body_jacobian_joint5(q, &mut output),
            Frame::Joint6 => self.library.body_jacobian_joint6(q, &mut output),
            Frame::Joint7 => self.library.body_jacobian_joint7(q, &mut output),
            Frame::Flange => self.library.body_jacobian_flange(q, &mut output),
            Frame::EndEffector => self.library.body_jacobian_ee(q, F_T_EE, &mut output),
            Frame::Stiffness => {
                let tmp: Matrix4<f64> =
                    Matrix4::from_column_slice(F_T_EE) * Matrix4::from_column_slice(EE_T_K);
                let mut stiffness_f_t_ee = [0.; 16];
                tmp.iter()
                    .enumerate()
                    .for_each(|(i, &x)| stiffness_f_t_ee[i] = x);
                self.library
                    .body_jacobian_ee(q, &stiffness_f_t_ee, &mut output)
            }
        }
        output
    }

    /// Gets the 6x7 Jacobian for the given frame, relative to that frame.
    ///
    /// The Jacobian is represented as a 6x7 matrix in column-major format.
    /// # Arguments
    /// * `frame` - The desired frame.
    /// * `robot_state` - State from which the pose should be calculated.
    /// # Return
    /// Vectorized 6x7 Jacobian, column-major.
    pub fn body_jacobian_from_state(&self, frame: &Frame, robot_state: &PandaState) -> [f64; 42] {
        self.body_jacobian(
            frame,
            &robot_state.q,
            &robot_state.F_T_EE,
            &robot_state.EE_T_K,
        )
    }
    /// Gets the 6x7 Jacobian for the given joint relative to the base frame.
    ///
    /// The Jacobian is represented as a 6x7 matrix in column-major format.
    /// # Arguments
    /// * `frame` - The desired frame.
    /// * `q` - Joint position.
    /// * `F_T_EE` - End effector in flange frame.
    /// * `EE_T_K` - Stiffness frame K in the end effector frame.
    /// # Return
    /// Vectorized 6x7 Jacobian, column-major.
    pub fn zero_jacobian(
        &self,
        frame: &Frame,
        q: &[f64; 7],
        F_T_EE: &[f64; 16],
        EE_T_K: &[f64; 16],
    ) -> [f64; 42] {
        let mut output = [0.; 42];
        match frame {
            Frame::Joint1 => self.library.zero_jacobian_joint1(&mut output),
            Frame::Joint2 => self.library.zero_jacobian_joint2(q, &mut output),
            Frame::Joint3 => self.library.zero_jacobian_joint3(q, &mut output),
            Frame::Joint4 => self.library.zero_jacobian_joint4(q, &mut output),
            Frame::Joint5 => self.library.zero_jacobian_joint5(q, &mut output),
            Frame::Joint6 => self.library.zero_jacobian_joint6(q, &mut output),
            Frame::Joint7 => self.library.zero_jacobian_joint7(q, &mut output),
            Frame::Flange => self.library.zero_jacobian_flange(q, &mut output),
            Frame::EndEffector => self.library.zero_jacobian_ee(q, F_T_EE, &mut output),
            Frame::Stiffness => {
                let tmp: Matrix4<f64> =
                    Matrix4::from_column_slice(F_T_EE) * Matrix4::from_column_slice(EE_T_K);
                let mut stiffness_f_t_ee = [0.; 16];
                tmp.iter()
                    .enumerate()
                    .for_each(|(i, &x)| stiffness_f_t_ee[i] = x);
                self.library
                    .zero_jacobian_ee(q, &stiffness_f_t_ee, &mut output)
            }
        }
        output
    }
    /// Gets the 6x7 Jacobian for the given joint relative to the base frame.
    ///
    /// The Jacobian is represented as a 6x7 matrix in column-major format.
    /// # Arguments
    /// * `frame` - The desired frame.
    /// * `robot_state` - State from which the pose should be calculated.
    /// # Return
    /// Vectorized 6x7 Jacobian, column-major.
    pub fn zero_jacobian_from_state(&self, frame: &Frame, robot_state: &PandaState) -> [f64; 42] {
        self.zero_jacobian(
            frame,
            &robot_state.q,
            &robot_state.F_T_EE,
            &robot_state.EE_T_K,
        )
    }
    /// Calculates the 7x7 mass matrix. Unit: [kg \times m^2].
    /// # Arguments
    /// * `q` - Joint position.
    /// * `I_total` - Inertia of the attached total load including end effector, relative to
    /// center of mass, given as vectorized 3x3 column-major matrix. Unit: [kg * m^2].
    /// * `m_total` - Weight of the attached total load including end effector. Unit: \[kg\].
    /// * `F_x_Ctotal` - Translation from flange to center of mass of the attached total load. Unit: \[m\].
    /// # Return
    /// Vectorized 7x7 mass matrix, column-major.
    pub fn mass(
        &self,
        q: &[f64; 7],
        I_total: &[f64; 9],
        m_total: f64,
        F_x_Ctotal: &[f64; 3],
    ) -> [f64; 49] {
        let mut output = [0.; 49];
        self.library
            .mass(q, I_total, &m_total, F_x_Ctotal, &mut output);
        output
    }
    /// Calculates the 7x7 mass matrix. Unit: [kg \times m^2].
    /// # Arguments
    /// * `robot_state` - State from which the mass matrix should be calculated.
    /// # Return
    /// Vectorized 7x7 mass matrix, column-major.
    pub fn mass_from_state(&self, robot_state: &PandaState) -> [f64; 49] {
        self.mass(
            &robot_state.q,
            &robot_state.I_total,
            robot_state.m_total,
            &robot_state.F_x_Ctotal,
        )
    }
    /// Calculates the Coriolis force vector (state-space equation):
    /// ![c= C \times dq](https://latex.codecogs.com/png.latex?c=&space;C&space;\times&space;dq), in \[Nm\].
    /// # Arguments
    /// * `q` - Joint position.
    /// * `dq` - Joint velocity.
    /// * `I_total` - Inertia of the attached total load including end effector, relative to
    /// center of mass, given as vectorized 3x3 column-major matrix. Unit: [kg * m^2].
    /// * `m_total` - Weight of the attached total load including end effector. Unit: \[kg\].
    /// * `F_x_Ctotal` - Translation from flange to center of mass of the attached total load. Unit: \[m\].
    /// # Return
    /// Coriolis force vector.
    pub fn coriolis(
        &self,
        q: &[f64; 7],
        dq: &[f64; 7],
        I_total: &[f64; 9],
        m_total: f64,
        F_x_Ctotal: &[f64; 3],
    ) -> [f64; 7] {
        let mut output = [0.; 7];
        self.library
            .coriolis(q, dq, I_total, &m_total, F_x_Ctotal, &mut output);
        output
    }

    /// Calculates the Coriolis force vector (state-space equation):
    /// ![c= C \times dq](https://latex.codecogs.com/png.latex?c=&space;C&space;\times&space;dq), in \[Nm\].
    /// # Arguments
    /// * `robot_state` - State from which the Coriolis force vector should be calculated.
    /// # Return
    /// Coriolis force vector.
    pub fn coriolis_from_state(&self, robot_state: &PandaState) -> [f64; 7] {
        self.coriolis(
            &robot_state.q,
            &robot_state.dq,
            &robot_state.I_load,
            robot_state.m_total,
            &robot_state.F_x_Ctotal,
        )
    }
    ///  Calculates the gravity vector. Unit: \[Nm\].
    /// # Arguments
    /// * `q` - Joint position.
    /// * `m_total` - Weight of the attached total load including end effector. Unit: \[kg\].
    /// * `F_x_Ctotal` - Translation from flange to center of mass of the attached total load. Unit: \[m\].
    /// * `gravity_earth` - Earth's gravity vector. Unit: [m / s^2]
    /// Default to [0.0, 0.0, -9.81].
    /// # Return
    /// Gravity vector.
    pub fn gravity<'a, Grav: Into<Option<&'a [f64; 3]>>>(
        &self,
        q: &[f64; 7],
        m_total: f64,
        F_x_Ctotal: &[f64; 3],
        gravity_earth: Grav,
    ) -> [f64; 7] {
        let gravity_earth = gravity_earth.into().unwrap_or(&[0., 0., -9.81]);
        let mut output = [0.; 7];
        self.library
            .gravity(q, gravity_earth, &m_total, F_x_Ctotal, &mut output);
        output
    }
    ///  Calculates the gravity vector. Unit: \[Nm\].
    /// # Arguments
    /// * `robot_state` - State from which the gravity vector should be calculated.
    /// * `gravity_earth` - Earth's gravity vector. Unit: [m / s^2].
    /// By default `gravity_earth` will be calculated from [`RobotState::O_ddP_O`](`crate::RobotState::O_ddP_O`).
    /// # Return
    /// Gravity vector.
    pub fn gravity_from_state<'a, Grav: Into<Option<&'a [f64; 3]>>>(
        &self,
        robot_state: &PandaState,
        gravity_earth: Grav,
    ) -> [f64; 7] {
        self.gravity(
            &robot_state.q,
            robot_state.m_total,
            &robot_state.F_x_Ctotal,
            gravity_earth.into().unwrap_or(&robot_state.O_ddP_O),
        )
    }
}
