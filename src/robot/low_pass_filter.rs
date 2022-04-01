// Copyright (c) 2021 Marco Boneberger
// Licensed under the EUPL-1.2-or-later

//! Contains functions for filtering signals with a low-pass filter.

use crate::utils::array_to_isometry;
use std::f64::consts::PI;

/// Maximum cutoff frequency: 1000 Hz
pub static MAX_CUTOFF_FREQUENCY: f64 = 1000.0;
///  Default cutoff frequency: 100 Hz
pub static DEFAULT_CUTOFF_FREQUENCY: f64 = 100.0;

/// Applies a first-order low-pass filter
///
/// # Arguments
/// * `sample_time` - Sample time constant
/// * `y` - Current value of the signal to be filtered
/// * `y_last` - Value of the signal to be filtered in the previous time step
/// * `cutoff_frequency` - Cutoff frequency of the low-pass filter
/// # Panics
/// This function panics if:
/// * y is infinite or NaN.
/// * y_last is infinite or NaN.
/// * cutoff_frequency is zero, negative, infinite or NaN.
/// * sample_time is negative, infinite or NaN.
/// # Return
/// Filtered value.
pub fn low_pass_filter(sample_time: f64, y: f64, y_last: f64, cutoff_frequency: f64) -> f64 {
    assert!(sample_time.is_sign_positive() && sample_time.is_finite());
    assert!(cutoff_frequency.is_sign_positive() && cutoff_frequency.is_finite());
    assert!(y.is_finite() && y_last.is_finite());
    let gain = sample_time / (sample_time + (1.0 / (2.0 * PI * cutoff_frequency)));
    gain * y + (1. - gain) * y_last
}

/// Applies a first-order low-pass filter to the translation and spherical linear interpolation
/// to the rotation of a transformation matrix which represents a Cartesian Motion.
///
/// # Arguments
/// * `sample_time` - Sample time constant
/// * `y` - Current Cartesian transformation matrix to be filtered
/// * `y_last` - Cartesian transformation matrix from the previous time step
/// * `cutoff_frequency` - Cutoff frequency of the low-pass filter
/// # Panics
/// This function panics if:
/// * elements of y is infinite or NaN.
/// * elements of y_last is infinite or NaN.
/// * cutoff_frequency is zero, negative, infinite or NaN.
/// * sample_time is negative, infinite or NaN.
/// # Return
/// Filtered Cartesian transformation matrix.
pub fn cartesian_low_pass_filter(
    sample_time: f64,
    y: &[f64; 16],
    y_last: &[f64; 16],
    cutoff_frequency: f64,
) -> [f64; 16] {
    assert!(sample_time.is_sign_positive() && sample_time.is_finite());
    assert!(cutoff_frequency.is_sign_positive() && cutoff_frequency.is_finite());
    y.iter()
        .zip(y_last.iter())
        .for_each(|(i, j)| assert!(i.is_finite() && j.is_finite()));
    let mut transform = array_to_isometry(y);
    let transform_last = array_to_isometry(y_last);
    let gain = sample_time / (sample_time + (1.0 / (2.0 * PI * cutoff_frequency)));
    transform.translation.vector =
        gain * transform.translation.vector + (1. - gain) * transform_last.translation.vector;
    transform.rotation = transform_last.rotation.slerp(&transform.rotation, gain);

    let mut out = [0.; 16];
    for (i, &x) in transform.to_homogeneous().iter().enumerate() {
        out[i] = x;
    }
    out
}

#[cfg(test)]
mod tests {
    use crate::robot::low_pass_filter::{cartesian_low_pass_filter, low_pass_filter};

    #[test]
    fn low_pass_test() {
        assert!(f64::abs(low_pass_filter(0.001, 1.0, 1.0, 100.0) - 1.) < 0.000001);
        assert!(f64::abs(low_pass_filter(0.001, 1.0, 1.0, 500.0) - 1.) < 0.000001);
        assert!(f64::abs(low_pass_filter(0.001, 1.0, 1.0, 1000.0) - 1.) < 0.000001);
        assert!(f64::abs(low_pass_filter(0.001, 1.0, 0.0, 100.0) - 0.3859) < 0.0001);
        assert!(f64::abs(low_pass_filter(0.001, 1.0, 0.0, 500.0) - 0.7585) < 0.0001);
        assert!(f64::abs(low_pass_filter(0.001, 1.0, 0.0, 900.0) - 0.8497) < 0.0001);
    }

    #[test]
    fn low_pass_cartesian_test() {
        let sample_time = 0.001;
        let cutoff_frequency = 100.;
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
        let y_last = [
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

        println!(
            "{:?}",
            cartesian_low_pass_filter(sample_time, &y, &y_last, cutoff_frequency)
        );
    }
}
