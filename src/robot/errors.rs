// Copyright (c) 2021 Marco Boneberger
// Licensed under the EUPL-1.2-or-later

//! Contains the FrankaErrors type.
use std::fmt;

use crate::robot::error::FrankaError;
use crate::robot::types::RoboErrorHelperStruct;
use num_traits::FromPrimitive;

/// Represent the Errors which were received from the robot.
#[derive(Debug, Clone, Default)]
pub struct FrankaErrors {
    /// contains all errors
    pub franka_errors: Vec<FrankaError>,
}

pub enum FrankaErrorKind {
    Error,
    ReflexReason,
}

impl FrankaErrors {
    /// Creates a new set of errors.
    /// # Arguments
    /// * `errors` - a [RoboErrorHelperStruct](`crate::robot::types::RoboErrorHelperStruct`) containing
    /// all of the errors from [RobotStateIntern](`crate::robot::types::RobotStateIntern`)
    /// * `kind` - specifies if the errors are normal Errors or Reflex reasons
    pub(crate) fn new(errors: RoboErrorHelperStruct, kind: FrankaErrorKind) -> FrankaErrors {
        let error_array = match kind {
            FrankaErrorKind::Error => errors.combine_errors(),
            FrankaErrorKind::ReflexReason => errors.combine_reflex_reasons(),
        };

        let franka_errors = error_array
            .iter()
            .enumerate()
            .filter(|(_i, x)| **x)
            .map(|(i, _x)| FrankaError::from_i32(i as i32).unwrap())
            .collect::<Vec<FrankaError>>();
        FrankaErrors { franka_errors }
    }
    /// check if a specific error is received
    pub fn contains(&self, error: FrankaError) -> bool {
        self.franka_errors.contains(&error)
    }
}

impl fmt::Display for FrankaErrors {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "[")?;
        let mut iter = self.franka_errors.iter().peekable();
        while let Some(error) = iter.next() {
            match iter.peek() {
                Some(_) => {
                    write!(f, "\"{}\", ", error)?;
                }
                None => {
                    return write!(f, "\"{}\"]", error);
                }
            }
        }
        Ok(())
    }
}
