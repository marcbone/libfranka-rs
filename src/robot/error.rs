// Copyright (c) 2021 Marco Boneberger
// Licensed under the EUPL-1.2-or-later
//! Defines the errors that can occur while controlling the robot.
use std::fmt::{Debug, Display, Formatter, Result};

use num_derive::{FromPrimitive, ToPrimitive};
use serde_repr::{Deserialize_repr, Serialize_repr};

/// Controlling errors of the robot.
#[derive(Serialize_repr, Deserialize_repr, Debug, PartialEq, Copy, Clone)]
#[repr(u64)]
#[allow(non_camel_case_types)]
#[derive(FromPrimitive, ToPrimitive)]
pub enum FrankaError {
    /// The robot moved past the joint limits.
    kJointPositionLimitsViolation = 0,
    /// The robot moved past any of the virtual walls.
    kCartesianPositionLimitsViolation = 1,
    /// The robot would have collided with itself
    kSelfcollisionAvoidanceViolation = 2,
    /// The robot exceeded joint velocity limits.
    kJointVelocityViolation = 3,
    /// The robot exceeded Cartesian velocity limits.
    kCartesianVelocityViolation = 4,
    /// The robot exceeded safety threshold during force control.
    kForceControlSafetyViolation = 5,
    /// A collision was detected, i.e.\ the robot exceeded a torque threshold in a joint motion.
    kJointReflex = 6,
    /// A collision was detected, i.e.\ the robot exceeded a torque threshold in a Cartesian motion.
    kCartesianReflex = 7,
    /// Internal motion generator did not reach the goal pose.
    kMaxGoalPoseDeviationViolation = 8,
    /// True if internal motion generator deviated from the path.
    kMaxPathPoseDeviationViolation = 9,
    /// Cartesian velocity profile for internal motions was exceeded.
    kCartesianVelocityProfileSafetyViolation = 10,
    /// An external joint position motion generator was started with a pose too far from the current pose.
    kJointPositionMotionGeneratorStartPoseInvalid = 11,
    /// An external joint motion generator would move into a joint limit.
    kJointMotionGeneratorPositionLimitsViolation = 12,
    /// An external joint motion generator exceeded velocity limits.
    kJointMotionGeneratorVelocityLimitsViolation = 13,
    /// Commanded velocity in joint motion generators is discontinuous (target values are too far apart).
    kJointMotionGeneratorVelocityDiscontinuity = 14,
    /// Commanded acceleration in joint motion generators is discontinuous (target values are too far apart).
    kJointMotionGeneratorAccelerationDiscontinuity = 15,
    /// An external Cartesian position motion generator was started with a pose too far from  the current pose.
    kCartesianPositionMotionGeneratorStartPoseInvalid = 16,
    /// An external Cartesian motion generator would move into an elbow limit.
    kCartesianMotionGeneratorElbowLimitViolation = 17,
    /// An external Cartesian motion generator would move with too high velocity.
    kCartesianMotionGeneratorVelocityLimitsViolation = 18,
    /// Commanded velocity in Cartesian motion generators is discontinuous (target values are too far apart).
    kCartesianMotionGeneratorVelocityDiscontinuity = 19,
    /// Commanded acceleration in Cartesian motion generators is discontinuous (target values are too far apart).
    kCartesianMotionGeneratorAccelerationDiscontinuity = 20,
    /// Commanded elbow values in Cartesian motion generators are inconsistent.
    kCartesianMotionGeneratorElbowSignInconsistent = 21,
    /// The first elbow value in Cartesian motion generators is too far from initial one.
    kCartesianMotionGeneratorStartElbowInvalid = 22,
    /// The torque set by the external controller is discontinuous.
    kForceControllerDesiredForceToleranceViolation = 23,
    /// The start elbow sign was inconsistent.
    /// Applies only to motions started from Desk.
    kStartElbowSignInconsistent = 24,
    /// Minimum network communication quality could not be held during a motion.
    kCommunicationConstraintsViolation = 25,
    /// Commanded values would result in exceeding the power limit.
    kPowerLimitViolation = 26,
    /// The joint position limits would be exceeded after IK calculation.
    kCartesianMotionGeneratorJointPositionLimitsViolation = 27,
    /// The joint velocity limits would be exceeded after IK calculation.
    kCartesianMotionGeneratorJointVelocityLimitsViolation = 28,
    /// The joint velocity in Cartesian motion generators is discontinuous after IK calculation
    kCartesianMotionGeneratorJointVelocityDiscontinuity = 29,
    /// The joint acceleration in Cartesian motion generators is discontinuous after IK calculation.
    kCartesianMotionGeneratorJointAccelerationDiscontinuity = 30,
    /// Cartesian pose is not a valid transformation matrix.
    kCartesianPositionMotionGeneratorInvalidFrame = 31,
    /// The torque set by the external controller is discontinuous.
    kControllerTorqueDiscontinuity = 32,
    /// The robot is overloaded for the required motion.
    ///
    /// Applies only to motions started from Desk.
    kJointP2PInsufficientTorqueForPlanning = 33,
    /// The measured torque signal is out of the safe range.
    kTauJRangeViolation = 34,
    /// An instability is detected.
    kInstabilityDetection = 35,
    /// The robot is in joint position limits violation error and the user guides the robot further towards the limit.
    kJointMoveInWrongDirection = 36,
}

impl Display for FrankaError {
    fn fmt(&self, f: &mut Formatter) -> Result {
        write!(f, "{:?}", self)
    }
}
