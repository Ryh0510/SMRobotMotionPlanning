#pragma once

#include <MotionPlanningCore/MotionPlanning.h>
#include <Eigen/Geometry>
#include <functional>
#include <limits>

namespace motion_planning
{
    struct TrajectoryQuality
    {
        bool valid = false;
        bool timingValid = false;
        int nonPositiveIntervals = 0;
        int positionLimitViolations = 0;
        int velocityLimitViolations = 0;
        int accelerationLimitViolations = 0;
        int fallbackVelocityLimits = 0;
        int fallbackAccelerationLimits = 0;
        double duration = 0.0;
        double jointLength = 0.0;
        double bendingCost = 0.0;
        double maximumCornerRadians = 0.0;
        double peakVelocity = std::numeric_limits<double>::quiet_NaN();
        double peakAcceleration = std::numeric_limits<double>::quiet_NaN();
        std::vector<double> jointPeakVelocities, jointPeakAccelerations;
        std::vector<double> jointVelocityLimits, jointAccelerationLimits;
        bool tcpAvailable = false;
        double tcpLength = 0.0;
        double peakTcpSpeed = std::numeric_limits<double>::quiet_NaN();
        double maximumTcpDeviation = 0.0;
        double maximumOrientationDeviation = 0.0;
        std::vector<Eigen::Vector3d> tcpPositions;
    };

    using TrajectoryForwardKinematics = std::function<Eigen::Isometry3d(const std::vector<double>&)>;

    // Strictly increasing geometric parameter; no wrapping of legal turn values.
    std::vector<double> jointPathParameters(const std::vector<std::vector<double>>& path,
        double scale = 0.04);

    // Reports piecewise-linear segment velocities and finite-difference knot
    // accelerations (including start/end at rest), not impulse-free dynamics.
    TrajectoryQuality evaluateTrajectoryQuality(robottrajectory::JointTrajectory& trajectory,
        const std::vector<JointBound>& bounds, double fallbackVelocity, double fallbackAcceleration,
        const std::vector<double>& comparisonParameters = {},
        const TrajectoryForwardKinematics& fk = {},
        const robottrajectory::JointTrajectory* reference = nullptr);

    // Changes time/qd/qdd only. Preserves every q and its turn, fixes zero-duration
    // intervals, then globally stretches time to satisfy the sampled limits.
    bool retimeJointTrajectory(robottrajectory::JointTrajectory& trajectory,
        const std::vector<JointBound>& bounds, double fallbackVelocity, double fallbackAcceleration);
}
