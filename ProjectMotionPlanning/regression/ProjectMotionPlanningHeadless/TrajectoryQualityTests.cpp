#include <ProjectMotionPlanning/TrajectoryQuality.h>
#include "../../src/PathRefinement.h"
#include <cmath>
#include <iostream>
#include <limits>

using namespace motion_planning;
int main()
{
    constexpr double pi = 3.14159265358979323846;
    std::vector<JointBound> bounds{{-10, 10, 1.0, 2.0, false}, {-10, 10, 0.0, 0.0, false}};
    robottrajectory::JointTrajectory path;
    path.points = {{0.0, {2*pi, 0.0}, {}, {}}, {0.0, {2*pi + 0.2, 0.1}, {}, {}}, {0.02, {2*pi + 0.4, 0.15}, {}, {}}};
    const auto original = path;
    const auto fk = [](const std::vector<double>& q) {
        Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
        pose.translation() = Eigen::Vector3d(q[0], q[1], 0.0);
        pose.linear() = Eigen::AngleAxisd(q[1], Eigen::Vector3d::UnitZ()).toRotationMatrix();
        return pose;
    };
    const auto invalidTiming = evaluateTrajectoryQuality(path, bounds, 0.8, 1.5, {}, fk, &original);
    if(!invalidTiming.valid || invalidTiming.timingValid || invalidTiming.nonPositiveIntervals != 1 ||
        std::isfinite(invalidTiming.peakVelocity) || invalidTiming.maximumTcpDeviation != 0.0 ||
        invalidTiming.fallbackAccelerationLimits != 1) return 1;
    if(!retimeJointTrajectory(path, bounds, 0.8, 1.5)) return 2;
    const auto measured = evaluateTrajectoryQuality(path, bounds, 0.8, 1.5, {}, fk, &original);
    if(!measured.timingValid || measured.nonPositiveIntervals || measured.velocityLimitViolations ||
        measured.accelerationLimitViolations || measured.positionLimitViolations || !measured.tcpAvailable) return 3;
    for(std::size_t i = 0; i < path.points.size(); ++i)
        if(path.points[i].q != original.points[i].q || path.points[i].qd.size() != 2 || path.points[i].qdd.size() != 2) return 4;
    if(path.points[0].q[0] < 2*pi || measured.maximumTcpDeviation != 0.0) return 5;
    auto shifted = path;
    shifted.points[1].q[1] += 0.03;
    const auto shiftMetric = evaluateTrajectoryQuality(shifted, bounds, 0.8, 1.5, {}, fk, &path);
    if(std::abs(shiftMetric.maximumTcpDeviation - 0.03) > 1e-10 ||
        std::abs(shiftMetric.maximumOrientationDeviation - 0.03) > 1e-10) return 6;
    robottrajectory::JointTrajectory singleton;
    singleton.points = {{0.0, {0.1, 0.2}, {}, {}}};
    if(!retimeJointTrajectory(singleton, bounds, 0.8, 1.5) || singleton.points[0].qd[0] != 0.0) return 7;
    path.points[0].q[0] = std::numeric_limits<double>::quiet_NaN();
    if(retimeJointTrajectory(path, bounds, 0.8, 1.5) || evaluateTrajectoryQuality(path, bounds, 0.8, 1.5).valid) return 8;
    const std::vector<std::vector<double>> straight{{0, 0}, {0.02, 0.04}, {0.4, 0.8}, {1, 2}};
    if(detail::jointPathBending(straight, jointPathParameters(straight)) > 1e-20) return 9;
    // Smoothing must reduce a real zigzag, keep endpoints/corridor, and validate
    // accepted connections. This also exercises line-search recovery.
    detail::ApfPath noisy;
    for(int i = 0; i < 129; ++i) noisy.push_back({i * 0.02, 0.08 * std::sin(i * 0.6)});
    const auto before = noisy; const auto parameter = jointPathParameters(noisy);
    detail::ApfOracle oracle; int checks = 0;
    oracle.motionValid = [&](const auto&, const auto&) { ++checks; return true; };
    if(!detail::smoothValidatedPath(&noisy, parameter, oracle, 6, 0.5, 0.1) || checks == 0 ||
        noisy.front() != before.front() || noisy.back() != before.back() ||
        !(detail::jointPathBending(noisy, parameter) < 0.5 * detail::jointPathBending(before, parameter))) return 10;
    for(std::size_t i = 0; i < noisy.size(); ++i) for(std::size_t j = 0; j < 2; ++j)
        if(std::abs(noisy[i][j] - before[i][j]) > 0.1 + 1e-12) return 11;
    std::cout << "PASS trajectory quality: invalid timing, turns, time scaling, sampled dynamics, actual FK deviations, nonuniform parameter, constrained smoothing\n";
    return 0;
}
