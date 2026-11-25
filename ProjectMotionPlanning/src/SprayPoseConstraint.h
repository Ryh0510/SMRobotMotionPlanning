#pragma once
#include <ProjectMotionPlanning/SprayAxisContinuity.h>

namespace motion_planning { namespace detail {
// distance is the original positive first-hit range, or NaN for a transfer.
// The caller independently rechecks first-hit visibility and actual-model FK.
bool constrainedSprayPose(const Eigen::Isometry3d& reference, const Eigen::Vector3d& axis,
    double distance, const Eigen::Vector3d& angles, double distanceShift,
    const SprayAxisContinuityOptions& options, Eigen::Isometry3d& result);
const char* sprayPoseConstraintName(SprayPoseConstraint constraint);
} }
