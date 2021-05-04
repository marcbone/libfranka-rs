// Copyright (c) 2021 Marco Boneberger
// Licensed under the EUPL-1.2-or-later
#![allow(non_upper_case_globals)]

use crate::exception::FrankaException;
use crate::FrankaResult;
use std::path::Path;

/// Determines whether the current OS kernel is a realtime kernel.
///
/// On Linux, this checks for the existence of `/sys/kernel/realtime`.
pub fn has_realtime_kernel() -> bool {
    Path::new("/sys/kernel/realtime").exists()
}

/// Sets the current thread to the highest possible scheduler priority.
///
/// # Errors
/// * RealtimeException if realtime priority cannot be set for the current thread.
///
/// If the method returns an Error please check your /etc/security/limits.conf file
/// There should be a line like this:
/// ```text
///marco            -       rtprio          99
/// ```
pub fn set_current_thread_to_highest_scheduler_priority() -> FrankaResult<()> {
    unsafe {
        let max_priority = libc::sched_get_priority_max(libc::SCHED_FIFO);
        if max_priority == -1 {
            return Err(FrankaException::RealTimeException {
                message: "libfranka-rs: unable to get maximum possible thread priority".to_string(),
            });
        }
        let thread_param = libc::sched_param {
            // In the original libfranka the priority is set to the maximum priority (99 in this
            // case). However, we will set the priority 1 lower as
            // https://rt.wiki.kernel.org/index.php/HOWTO:_Build_an_RT-application recommends
            sched_priority: max_priority - 1,
        };
        if libc::pthread_setschedparam(libc::pthread_self(), libc::SCHED_FIFO, &thread_param) != 0 {
            return Err(FrankaException::RealTimeException {
                message: "libfranka-rs: unable to set realtime scheduling".to_string(),
            });
        }
        // The original libfranka does not use mlock. However, we use it to prevent our memory from
        // being swapped.
        if libc::mlockall(libc::MCL_CURRENT | libc::MCL_FUTURE) != 0 {
            return Err(FrankaException::RealTimeException {
                message: "libfranka-rs: unable to lock memory".to_string(),
            });
        }
    }
    Ok(())
}

/// Determines whether the given array represents a valid homogeneous transformation matrix.
/// transform is represented as a 4x4 matrix in column-major format
#[allow(clippy::float_cmp)]
pub fn is_homogeneous_transformation(transform: &[f64; 16]) -> bool {
    const kOrthonormalThreshold: f64 = 1e-5;
    if transform[3] != 0.0 || transform[7] != 0.0 || transform[11] != 0.0 || transform[15] != 1.0 {
        return false;
    }
    for j in 0..3 {
        if f64::abs(
            f64::sqrt(
                transform[j * 4].powf(2.)
                    + transform[j * 4 + 1].powf(2.)
                    + transform[j * 4 + 2].powf(2.),
            ) - 1.,
        ) > kOrthonormalThreshold
        {
            return false;
        }
    }
    for i in 0..3 {
        if f64::abs(
            f64::sqrt(
                transform[i].powf(2.) + transform[4 + i].powf(2.) + transform[2 * 4 + i].powf(2.),
            ) - 1.,
        ) > kOrthonormalThreshold
        {
            return false;
        }
    }

    true
}
