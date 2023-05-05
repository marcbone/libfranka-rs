// Copyright (c) 2021 Marco Boneberger
// Licensed under the EUPL-1.2-or-later
use crate::exception::FrankaException;
use crate::exception::FrankaException::ModelException;
use crate::FrankaResult;
use libc::c_double;
use libloading as lib;
use libloading::os::unix::Library;
use libloading::os::unix::Symbol;
use std::path::Path;

#[allow(non_snake_case, dead_code)]
pub(crate) struct ModelLibrary {
    libm: Library,
    lib_model: Library,
    M_NE: Symbol<
        unsafe extern "C" fn(
            *const c_double,
            *const c_double,
            c_double,
            *const c_double,
            *mut c_double,
        ),
    >,
    Ji_J_J1: Symbol<unsafe extern "C" fn(*mut c_double)>,
    Ji_J_J2: Symbol<unsafe extern "C" fn(*const c_double, *mut c_double)>,
    Ji_J_J3: Symbol<unsafe extern "C" fn(*const c_double, *mut c_double)>,
    Ji_J_J4: Symbol<unsafe extern "C" fn(*const c_double, *mut c_double)>,
    Ji_J_J5: Symbol<unsafe extern "C" fn(*const c_double, *mut c_double)>,
    Ji_J_J6: Symbol<unsafe extern "C" fn(*const c_double, *mut c_double)>,
    Ji_J_J7: Symbol<unsafe extern "C" fn(*const c_double, *mut c_double)>,
    Ji_J_J8: Symbol<unsafe extern "C" fn(*const c_double, *mut c_double)>,
    Ji_J_J9: Symbol<unsafe extern "C" fn(*const c_double, *const c_double, *mut c_double)>,

    O_J_J1: Symbol<unsafe extern "C" fn(*mut c_double)>,
    O_J_J2: Symbol<unsafe extern "C" fn(*const c_double, *mut c_double)>,
    O_J_J3: Symbol<unsafe extern "C" fn(*const c_double, *mut c_double)>,
    O_J_J4: Symbol<unsafe extern "C" fn(*const c_double, *mut c_double)>,
    O_J_J5: Symbol<unsafe extern "C" fn(*const c_double, *mut c_double)>,
    O_J_J6: Symbol<unsafe extern "C" fn(*const c_double, *mut c_double)>,
    O_J_J7: Symbol<unsafe extern "C" fn(*const c_double, *mut c_double)>,
    O_J_J8: Symbol<unsafe extern "C" fn(*const c_double, *mut c_double)>,
    O_J_J9: Symbol<unsafe extern "C" fn(*const c_double, *const c_double, *mut c_double)>,

    O_T_J1: Symbol<unsafe extern "C" fn(*const c_double, *mut c_double)>,
    O_T_J2: Symbol<unsafe extern "C" fn(*const c_double, *mut c_double)>,
    O_T_J3: Symbol<unsafe extern "C" fn(*const c_double, *mut c_double)>,
    O_T_J4: Symbol<unsafe extern "C" fn(*const c_double, *mut c_double)>,
    O_T_J5: Symbol<unsafe extern "C" fn(*const c_double, *mut c_double)>,
    O_T_J6: Symbol<unsafe extern "C" fn(*const c_double, *mut c_double)>,
    O_T_J7: Symbol<unsafe extern "C" fn(*const c_double, *mut c_double)>,
    O_T_J8: Symbol<unsafe extern "C" fn(*const c_double, *mut c_double)>,
    O_T_J9: Symbol<unsafe extern "C" fn(*const c_double, *const c_double, *mut c_double)>,

    c_NE: Symbol<
        unsafe extern "C" fn(
            *const c_double,
            *const c_double,
            *const c_double,
            c_double,
            *const c_double,
            *mut c_double,
        ),
    >,
    g_NE: Symbol<
        unsafe extern "C" fn(
            *const c_double,
            *const c_double,
            c_double,
            *const c_double,
            *mut c_double,
        ),
    >,
}

#[allow(non_snake_case, dead_code)]
impl ModelLibrary {
    pub fn new(model_filename: &Path, libm_filename: Option<&Path>) -> FrankaResult<Self> {
        let libm_filename = match libm_filename {
            Some(filename) => filename,
            None => Path::new("libm.so.6"),
        };
        unsafe {
            let libm = lib::os::unix::Library::open(
                Some(libm_filename),
                libc::RTLD_NOW | libc::RTLD_GLOBAL,
            )
            .map_err(|_| FrankaException::ModelException {
                message: "lifbranka-rs: Can not open libm library".to_string(),
            })?;
            let lib_model = lib::os::unix::Library::open(Some(model_filename), libc::RTLD_NOW)
                .map_err(|_| FrankaException::ModelException {
                    message: "lifbranka-rs: Can not open model library".to_string(),
                })?;

            let M_NE = ModelLibrary::get_symbol(&lib_model, "M_NE")?;
            let Ji_J_J1 = ModelLibrary::get_symbol(&lib_model, "Ji_J_J1")?;
            let Ji_J_J2 = ModelLibrary::get_symbol(&lib_model, "Ji_J_J2")?;
            let Ji_J_J3 = ModelLibrary::get_symbol(&lib_model, "Ji_J_J3")?;
            let Ji_J_J4 = ModelLibrary::get_symbol(&lib_model, "Ji_J_J4")?;
            let Ji_J_J5 = ModelLibrary::get_symbol(&lib_model, "Ji_J_J5")?;
            let Ji_J_J6 = ModelLibrary::get_symbol(&lib_model, "Ji_J_J6")?;
            let Ji_J_J7 = ModelLibrary::get_symbol(&lib_model, "Ji_J_J7")?;
            let Ji_J_J8 = ModelLibrary::get_symbol(&lib_model, "Ji_J_J8")?;
            let Ji_J_J9 = ModelLibrary::get_symbol(&lib_model, "Ji_J_J9")?;

            let O_J_J1 = ModelLibrary::get_symbol(&lib_model, "O_J_J1")?;
            let O_J_J2 = ModelLibrary::get_symbol(&lib_model, "O_J_J2")?;
            let O_J_J3 = ModelLibrary::get_symbol(&lib_model, "O_J_J3")?;
            let O_J_J4 = ModelLibrary::get_symbol(&lib_model, "O_J_J4")?;
            let O_J_J5 = ModelLibrary::get_symbol(&lib_model, "O_J_J5")?;
            let O_J_J6 = ModelLibrary::get_symbol(&lib_model, "O_J_J6")?;
            let O_J_J7 = ModelLibrary::get_symbol(&lib_model, "O_J_J7")?;
            let O_J_J8 = ModelLibrary::get_symbol(&lib_model, "O_J_J8")?;
            let O_J_J9 = ModelLibrary::get_symbol(&lib_model, "O_J_J9")?;

            let O_T_J1 = ModelLibrary::get_symbol(&lib_model, "O_T_J1")?;
            let O_T_J2 = ModelLibrary::get_symbol(&lib_model, "O_T_J2")?;
            let O_T_J3 = ModelLibrary::get_symbol(&lib_model, "O_T_J3")?;
            let O_T_J4 = ModelLibrary::get_symbol(&lib_model, "O_T_J4")?;
            let O_T_J5 = ModelLibrary::get_symbol(&lib_model, "O_T_J5")?;
            let O_T_J6 = ModelLibrary::get_symbol(&lib_model, "O_T_J6")?;
            let O_T_J7 = ModelLibrary::get_symbol(&lib_model, "O_T_J7")?;
            let O_T_J8 = ModelLibrary::get_symbol(&lib_model, "O_T_J8")?;
            let O_T_J9 = ModelLibrary::get_symbol(&lib_model, "O_T_J9")?;

            let c_NE = ModelLibrary::get_symbol(&lib_model, "c_NE")?;
            let g_NE = ModelLibrary::get_symbol(&lib_model, "g_NE")?;

            Ok(ModelLibrary {
                libm,
                lib_model,
                M_NE,
                Ji_J_J1,
                Ji_J_J2,
                Ji_J_J3,
                Ji_J_J4,
                Ji_J_J5,
                Ji_J_J6,
                Ji_J_J7,
                Ji_J_J8,
                Ji_J_J9,

                O_J_J1,
                O_J_J2,
                O_J_J3,
                O_J_J4,
                O_J_J5,
                O_J_J6,
                O_J_J7,
                O_J_J8,
                O_J_J9,

                O_T_J1,
                O_T_J2,
                O_T_J3,
                O_T_J4,
                O_T_J5,
                O_T_J6,
                O_T_J7,
                O_T_J8,
                O_T_J9,

                c_NE,
                g_NE,
            })
        }
    }
    pub fn mass(
        &self,
        q: &[f64; 7],
        I_load: &[f64; 9],
        m_load: &f64,
        F_x_Cload: &[f64; 3],
        m_ne: &mut [f64; 49],
    ) {
        unsafe {
            (self.M_NE)(
                q.as_ptr(),
                I_load.as_ptr(),
                *m_load,
                F_x_Cload.as_ptr(),
                m_ne.as_mut_ptr(),
            )
        }
    }
    pub fn body_jacobian_joint1(&self, b_Ji_J_J1: &mut [f64; 42]) {
        unsafe { (self.Ji_J_J1)(b_Ji_J_J1.as_mut_ptr()) }
    }
    pub fn body_jacobian_joint2(&self, q: &[f64; 7], b_Ji_J_J2: &mut [f64; 42]) {
        unsafe { (self.Ji_J_J2)(q.as_ptr(), b_Ji_J_J2.as_mut_ptr()) }
    }
    pub fn body_jacobian_joint3(&self, q: &[f64; 7], b_Ji_J_J3: &mut [f64; 42]) {
        unsafe { (self.Ji_J_J3)(q.as_ptr(), b_Ji_J_J3.as_mut_ptr()) }
    }
    pub fn body_jacobian_joint4(&self, q: &[f64; 7], b_Ji_J_J4: &mut [f64; 42]) {
        unsafe { (self.Ji_J_J4)(q.as_ptr(), b_Ji_J_J4.as_mut_ptr()) }
    }
    pub fn body_jacobian_joint5(&self, q: &[f64; 7], b_Ji_J_J5: &mut [f64; 42]) {
        unsafe { (self.Ji_J_J5)(q.as_ptr(), b_Ji_J_J5.as_mut_ptr()) }
    }
    pub fn body_jacobian_joint6(&self, q: &[f64; 7], b_Ji_J_J6: &mut [f64; 42]) {
        unsafe { (self.Ji_J_J6)(q.as_ptr(), b_Ji_J_J6.as_mut_ptr()) }
    }
    pub fn body_jacobian_joint7(&self, q: &[f64; 7], b_Ji_J_J7: &mut [f64; 42]) {
        unsafe { (self.Ji_J_J7)(q.as_ptr(), b_Ji_J_J7.as_mut_ptr()) }
    }
    pub fn body_jacobian_flange(&self, q: &[f64; 7], b_Ji_J_J8: &mut [f64; 42]) {
        unsafe { (self.Ji_J_J8)(q.as_ptr(), b_Ji_J_J8.as_mut_ptr()) }
    }
    pub fn body_jacobian_ee(&self, q: &[f64; 7], F_T_EE: &[f64; 16], b_Ji_J_J9: &mut [f64; 42]) {
        unsafe { (self.Ji_J_J9)(q.as_ptr(), F_T_EE.as_ptr(), b_Ji_J_J9.as_mut_ptr()) }
    }

    pub fn zero_jacobian_joint1(&self, b_O_J_J1: &mut [f64; 42]) {
        unsafe { (self.O_J_J1)(b_O_J_J1.as_mut_ptr()) }
    }
    pub fn zero_jacobian_joint2(&self, q: &[f64; 7], b_O_J_J2: &mut [f64; 42]) {
        unsafe { (self.O_J_J2)(q.as_ptr(), b_O_J_J2.as_mut_ptr()) }
    }
    pub fn zero_jacobian_joint3(&self, q: &[f64; 7], b_O_J_J3: &mut [f64; 42]) {
        unsafe { (self.O_J_J3)(q.as_ptr(), b_O_J_J3.as_mut_ptr()) }
    }
    pub fn zero_jacobian_joint4(&self, q: &[f64; 7], b_O_J_J4: &mut [f64; 42]) {
        unsafe { (self.O_J_J4)(q.as_ptr(), b_O_J_J4.as_mut_ptr()) }
    }
    pub fn zero_jacobian_joint5(&self, q: &[f64; 7], b_O_J_J5: &mut [f64; 42]) {
        unsafe { (self.O_J_J5)(q.as_ptr(), b_O_J_J5.as_mut_ptr()) }
    }
    pub fn zero_jacobian_joint6(&self, q: &[f64; 7], b_O_J_J6: &mut [f64; 42]) {
        unsafe { (self.O_J_J6)(q.as_ptr(), b_O_J_J6.as_mut_ptr()) }
    }
    pub fn zero_jacobian_joint7(&self, q: &[f64; 7], b_O_J_J7: &mut [f64; 42]) {
        unsafe { (self.O_J_J7)(q.as_ptr(), b_O_J_J7.as_mut_ptr()) }
    }
    pub fn zero_jacobian_flange(&self, q: &[f64; 7], b_O_J_J8: &mut [f64; 42]) {
        unsafe { (self.O_J_J8)(q.as_ptr(), b_O_J_J8.as_mut_ptr()) }
    }
    pub fn zero_jacobian_ee(&self, q: &[f64; 7], F_T_EE: &[f64; 16], b_O_J_J9: &mut [f64; 42]) {
        unsafe { (self.O_J_J9)(q.as_ptr(), F_T_EE.as_ptr(), b_O_J_J9.as_mut_ptr()) }
    }

    pub fn joint1(&self, q: &[f64; 7], b_O_T_J1: &mut [f64; 16]) {
        unsafe { (self.O_T_J1)(q.as_ptr(), b_O_T_J1.as_mut_ptr()) }
    }
    pub fn joint2(&self, q: &[f64; 7], b_O_T_J2: &mut [f64; 16]) {
        unsafe { (self.O_T_J2)(q.as_ptr(), b_O_T_J2.as_mut_ptr()) }
    }
    pub fn joint3(&self, q: &[f64; 7], b_O_T_J3: &mut [f64; 16]) {
        unsafe { (self.O_T_J3)(q.as_ptr(), b_O_T_J3.as_mut_ptr()) }
    }
    pub fn joint4(&self, q: &[f64; 7], b_O_T_J4: &mut [f64; 16]) {
        unsafe { (self.O_T_J4)(q.as_ptr(), b_O_T_J4.as_mut_ptr()) }
    }
    pub fn joint5(&self, q: &[f64; 7], b_O_T_J5: &mut [f64; 16]) {
        unsafe { (self.O_T_J5)(q.as_ptr(), b_O_T_J5.as_mut_ptr()) }
    }
    pub fn joint6(&self, q: &[f64; 7], b_O_T_J6: &mut [f64; 16]) {
        unsafe { (self.O_T_J6)(q.as_ptr(), b_O_T_J6.as_mut_ptr()) }
    }
    pub fn joint7(&self, q: &[f64; 7], b_O_T_J7: &mut [f64; 16]) {
        unsafe { (self.O_T_J7)(q.as_ptr(), b_O_T_J7.as_mut_ptr()) }
    }
    pub fn flange(&self, q: &[f64; 7], b_O_T_J8: &mut [f64; 16]) {
        unsafe { (self.O_T_J8)(q.as_ptr(), b_O_T_J8.as_mut_ptr()) }
    }
    pub fn ee(&self, q: &[f64; 7], F_T_EE: &[f64; 16], b_O_T_J9: &mut [f64; 16]) {
        unsafe { (self.O_T_J9)(q.as_ptr(), F_T_EE.as_ptr(), b_O_T_J9.as_mut_ptr()) }
    }

    pub fn coriolis(
        &self,
        q: &[f64; 7],
        qd: &[f64; 7],
        I_load: &[f64; 9],
        m_load: &f64,
        F_x_Cload: &[f64; 3],
        c_NE: &mut [f64; 7],
    ) {
        unsafe {
            (self.c_NE)(
                q.as_ptr(),
                qd.as_ptr(),
                I_load.as_ptr(),
                *m_load,
                F_x_Cload.as_ptr(),
                c_NE.as_mut_ptr(),
            )
        }
    }
    pub fn gravity(
        &self,
        q: &[f64; 7],
        g_earth: &[f64; 3],
        m_load: &f64,
        F_x_Cload: &[f64; 3],
        g_NE: &mut [f64; 7],
    ) {
        unsafe {
            (self.g_NE)(
                q.as_ptr(),
                g_earth.as_ptr(),
                *m_load,
                F_x_Cload.as_ptr(),
                g_NE.as_mut_ptr(),
            )
        }
    }
    fn get_symbol<T>(lib: &Library, symbol: &str) -> FrankaResult<Symbol<T>> {
        unsafe {
            lib.get(symbol.as_bytes()).map_err(|_| ModelException {
                message: format!("libfranka-rs: Symbol cannot be found: {}", symbol),
            })
        }
    }
}

#[cfg(test)]
mod test {
    use crate::exception::FrankaException;
    use crate::model::ModelLibrary;
    use std::path::Path;

    #[test]
    fn can_generate_model_exception() {
        match ModelLibrary::new(Path::new("/dev/null"), None) {
            Err(FrankaException::ModelException { message: _ }) => {}
            _ => panic!(),
        };
    }

    #[test]
    #[ignore]
    fn m_ne_test() {
        let model =
            ModelLibrary::new(Path::new("/home/marco/franka_model/model.so"), None).unwrap();
        let c = 5.;
        let a = [0., 0., 0., 0., 0., 0., 0.];
        let b = [0.; 9];
        let d = [0., 0., 0.];
        let mut e = [0.; 49];
        model.mass(&a, &b, &c, &d, &mut e);
        println!("{}", e[0]);
    }

    #[test]
    #[allow(non_snake_case)]
    #[ignore]
    fn coriolis_test() {
        let model =
            ModelLibrary::new(Path::new("/home/marco/franka_model/model.so"), None).unwrap();
        let mut c_NE = [0.; 7];
        let q = [0.; 7];
        let qd = [0.; 7];
        let I_load = [0.; 9];
        let m_load = 0.;
        let F_x_Cload = [0.; 3];
        model.coriolis(&q, &qd, &I_load, &m_load, &F_x_Cload, &mut c_NE);
    }
}
