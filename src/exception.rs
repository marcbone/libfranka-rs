// Copyright (c) 2021 Marco Boneberger
// Licensed under the EUPL-1.2-or-later

//! Contains exception and Result definitions
use crate::robot::logger::Record;
use crate::RobotState;
use thiserror::Error;

/// Represents all kind of errors which correspond to the franka::Exception in the C++ version of
/// this library
#[derive(Error, Debug)]
pub enum FrankaException {
    /// ControlException is thrown if an error occurs during motion generation or torque control.
    /// The exception holds a vector with the last received robot states. The number of recorded
    /// states can be configured in the Robot constructor.
    #[error("{error}")]
    ControlException {
        /// Vector of states and commands logged just before the exception occurred.
        log: Option<Vec<Record<RobotState>>>,
        /// Explanatory string.
        error: String,
    },

    /// IncompatibleVersionException is thrown if the robot does not support this version of libfranka-rs.
    #[error("Incompatible library version: Robot has version {server_version:?} and libfranka-rs has {library_version:?}")]
    IncompatibleLibraryVersionError {
        /// Control's protocol version.
        server_version: u16,
        /// libfranka-rs protocol version.
        library_version: u16,
    },

    /// NoMotionGeneratorRunningError is thrown when trying to send a motion command without an motion generator running.
    #[error("Trying to send motion command, but no motion generator running!")]
    NoMotionGeneratorRunningError,

    /// NoControllerRunningError is thrown when trying to send a control command without an controller running.
    #[error("Trying to send control command, but no controller generator running!")]
    NoControllerRunningError,
    /// PartialCommandError when trying to send a partial command.
    #[error("Trying to send partial command!")]
    PartialCommandError,

    /// NetworkException is thrown if a connection to the robot cannot be established, or when a timeout occurs.
    #[error("{message:?}")]
    NetworkException { message: String },

    /// CommandException is thrown if an error occurs during command execution.
    #[error("{message:?}")]
    CommandException { message: String },

    /// ModelException is thrown if an error occurs when loading the model library
    #[error("{message:?}")]
    ModelException { message: String },

    /// RealTimeException is thrown if the real-time priority cannot be set
    #[error("{message:?}")]
    RealTimeException { message: String },
}

/// creates a CommandException from a static string slice
pub(crate) fn create_command_exception(message: &'static str) -> FrankaException {
    FrankaException::CommandException {
        message: message.to_string(),
    }
}

/// Result type which can have FrankaException as Error
pub type FrankaResult<T> = Result<T, FrankaException>;
// wait for https://github.com/rust-lang/rust/issues/43301 to be closed
// impl Termination for FrankaResult<()> {
//     fn report(self) -> i32 {
//        return match self {
//            Ok(_) => {ExitCode::SUCCESS.report();}
//            Err(e) => {
//                eprintln!("{}",e);
//                ExitCode::FAILURE.report();
//            }
//        }
//     }
// }
