// Copyright (c) 2021 Marco Boneberger
// Licensed under the EUPL-1.2-or-later
//! Defines the errors that can occur while controlling the robot.
use std::fmt::{Debug, Display, Formatter, Result};

use num_derive::{FromPrimitive, ToPrimitive};
use serde_repr::{Deserialize_repr, Serialize_repr};

/// Controlling errors of the robot.
#[derive(Serialize_repr, Deserialize_repr, Debug, PartialEq, Copy, Clone)]
#[repr(u64)]
#[derive(FromPrimitive, ToPrimitive)]
pub enum FrankaError {
    /// The robot moved past the joint limits.
    JointPositionLimitsViolation = 0,
    /// The robot moved past any of the virtual walls.
    CartesianPositionLimitsViolation = 1,
    /// The robot would have collided with itself
    SelfCollisionAvoidanceViolation = 2,
    /// The robot exceeded joint velocity limits.
    JointVelocityViolation = 3,
    /// The robot exceeded Cartesian velocity limits.
    CartesianVelocityViolation = 4,
    /// The robot exceeded safety threshold during force control.
    ForceControlSafetyViolation = 5,
    /// A collision was detected, i.e.\ the robot exceeded a torque threshold in a joint motion.
    JointReflex = 6,
    /// A collision was detected, i.e.\ the robot exceeded a torque threshold in a Cartesian motion.
    CartesianReflex = 7,
    /// Internal motion generator did not reach the goal pose.
    MaxGoalPoseDeviationViolation = 8,
    /// True if internal motion generator deviated from the path.
    MaxPathPoseDeviationViolation = 9,
    /// Cartesian velocity profile for internal motions was exceeded.
    CartesianVelocityProfileSafetyViolation = 10,
    /// An external joint position motion generator was started with a pose too far from the current pose.
    JointPositionMotionGeneratorStartPoseInvalid = 11,
    /// An external joint motion generator would move into a joint limit.
    JointMotionGeneratorPositionLimitsViolation = 12,
    /// An external joint motion generator exceeded velocity limits.
    JointMotionGeneratorVelocityLimitsViolation = 13,
    /// Commanded velocity in joint motion generators is discontinuous (target values are too far apart).
    JointMotionGeneratorVelocityDiscontinuity = 14,
    /// Commanded acceleration in joint motion generators is discontinuous (target values are too far apart).
    JointMotionGeneratorAccelerationDiscontinuity = 15,
    /// An external Cartesian position motion generator was started with a pose too far from  the current pose.
    CartesianPositionMotionGeneratorStartPoseInvalid = 16,
    /// An external Cartesian motion generator would move into an elbow limit.
    CartesianMotionGeneratorElbowLimitViolation = 17,
    /// An external Cartesian motion generator would move with too high velocity.
    CartesianMotionGeneratorVelocityLimitsViolation = 18,
    /// Commanded velocity in Cartesian motion generators is discontinuous (target values are too far apart).
    CartesianMotionGeneratorVelocityDiscontinuity = 19,
    /// Commanded acceleration in Cartesian motion generators is discontinuous (target values are too far apart).
    CartesianMotionGeneratorAccelerationDiscontinuity = 20,
    /// Commanded elbow values in Cartesian motion generators are inconsistent.
    CartesianMotionGeneratorElbowSignInconsistent = 21,
    /// The first elbow value in Cartesian motion generators is too far from initial one.
    CartesianMotionGeneratorStartElbowInvalid = 22,
    /// The torque set by the external controller is discontinuous.
    ForceControllerDesiredForceToleranceViolation = 23,
    /// The start elbow sign was inconsistent.
    /// Applies only to motions started from Desk.
    StartElbowSignInconsistent = 24,
    /// Minimum network communication quality could not be held during a motion.
    CommunicationConstraintsViolation = 25,
    /// Commanded values would result in exceeding the power limit.
    PowerLimitViolation = 26,
    /// The joint position limits would be exceeded after IK calculation.
    CartesianMotionGeneratorJointPositionLimitsViolation = 27,
    /// The joint velocity limits would be exceeded after IK calculation.
    CartesianMotionGeneratorJointVelocityLimitsViolation = 28,
    /// The joint velocity in Cartesian motion generators is discontinuous after IK calculation
    CartesianMotionGeneratorJointVelocityDiscontinuity = 29,
    /// The joint acceleration in Cartesian motion generators is discontinuous after IK calculation.
    CartesianMotionGeneratorJointAccelerationDiscontinuity = 30,
    /// Cartesian pose is not a valid transformation matrix.
    CartesianPositionMotionGeneratorInvalidFrame = 31,
    /// The torque set by the external controller is discontinuous.
    ControllerTorqueDiscontinuity = 32,
    /// The robot is overloaded for the required motion.
    ///
    /// Applies only to motions started from Desk.
    JointP2PInsufficientTorqueForPlanning = 33,
    /// The measured torque signal is out of the safe range.
    TauJRangeViolation = 34,
    /// An instability is detected.
    InstabilityDetection = 35,
    /// The robot is in joint position limits violation error and the user guides the robot further towards the limit.
    JointMoveInWrongDirection = 36,
}

impl Display for FrankaError {
    fn fmt(&self, f: &mut Formatter) -> Result {
        write!(f, "{:?}", self)
    }
}
