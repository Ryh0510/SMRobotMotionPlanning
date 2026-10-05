#include <ProjectMotionPlanning/TrajectoryQuality.h>
#include "PathRefinement.h"

#include <algorithm>
#include <cmath>
#include <numeric>

namespace motion_planning
{
    namespace
    {
        double length(const std::vector<double>& a, const std::vector<double>& b)
        {
            double squared = 0.0;
            for(std::size_t j = 0; j < a.size(); ++j) squared += (b[j] - a[j]) * (b[j] - a[j]);
            return std::sqrt(squared);
        }
        double limit(double configured, double fallback)
        {
            return std::isfinite(configured) && configured > 0.0 ? configured : fallback;
        }
        bool validate(const robottrajectory::JointTrajectory& trajectory)
        {
            if(trajectory.points.empty() || trajectory.points.front().q.empty()) return false;
            const auto dimension = trajectory.points.front().q.size();
            for(const auto& point : trajectory.points) {
                if(point.q.size() != dimension || !std::isfinite(point.time)) return false;
                for(double q : point.q) if(!std::isfinite(q)) return false;
            }
            return true;
        }
    }

    std::vector<double> jointPathParameters(const std::vector<std::vector<double>>& path, double scale)
    {
        if(path.empty() || !std::isfinite(scale) || scale <= 0.0) return {};
        std::vector<double> parameters(path.size(), 0.0);
        for(std::size_t i = 1; i < path.size(); ++i) {
            if(path[i].size() != path.front().size()) return {};
            // A bounded floor avoids ill-conditioning at coincident/dense knots.
            parameters[i] = parameters[i - 1] + std::max(0.001, length(path[i - 1], path[i])) / scale;
        }
        return parameters;
    }

    TrajectoryQuality evaluateTrajectoryQuality(robottrajectory::JointTrajectory& trajectory,
        const std::vector<JointBound>& bounds, double fallbackVelocity, double fallbackAcceleration,
        const std::vector<double>& comparisonParameters, const TrajectoryForwardKinematics& fk,
        const robottrajectory::JointTrajectory* reference)
    {
        TrajectoryQuality result;
        if(!validate(trajectory) || (!bounds.empty() && bounds.size() != trajectory.points.front().q.size())) return result;
        const auto count = trajectory.points.size(), dimension = trajectory.points.front().q.size();
        result.valid = true;
        result.duration = trajectory.points.back().time - trajectory.points.front().time;
        result.timingValid = result.duration >= 0.0;
        std::vector<std::vector<double>> path;
        for(auto& point : trajectory.points) { path.push_back(point.q); point.qd.clear(); point.qdd.clear(); }
        result.jointLength = detail::jointPathLength(path);
        const auto parameters = comparisonParameters.size() == count ? comparisonParameters : jointPathParameters(path);
        if(parameters.size() == count) result.bendingCost = detail::jointPathBending(path, parameters);
        for(std::size_t i = 1; i + 1 < count; ++i) {
            const double a = length(path[i - 1], path[i]), b = length(path[i], path[i + 1]);
            if(a < 1e-9 || b < 1e-9) continue;
            double dot = 0.0;
            for(std::size_t j = 0; j < dimension; ++j)
                dot += (path[i][j] - path[i - 1][j]) * (path[i + 1][j] - path[i][j]);
            result.maximumCornerRadians = std::max(result.maximumCornerRadians,
                std::acos(std::clamp(dot / (a * b), -1.0, 1.0)));
        }
        std::vector<double> dt(count > 0 ? count - 1 : 0);
        for(std::size_t i = 0; i + 1 < count; ++i) {
            dt[i] = trajectory.points[i + 1].time - trajectory.points[i].time;
            if(dt[i] <= 0.0) { result.timingValid = false; ++result.nonPositiveIntervals; }
        }
        std::vector<double> vmax(dimension), amax(dimension);
        for(std::size_t j = 0; j < dimension; ++j) {
            const double v = bounds.empty() ? 0.0 : bounds[j].maxVelocity;
            const double a = bounds.empty() ? 0.0 : bounds[j].maxAcceleration;
            vmax[j] = limit(v, fallbackVelocity); amax[j] = limit(a, fallbackAcceleration);
            if(!(v > 0.0) || !std::isfinite(v)) ++result.fallbackVelocityLimits;
            if(!(a > 0.0) || !std::isfinite(a)) ++result.fallbackAccelerationLimits;
            if(!bounds.empty() && !bounds[j].continuous) {
                for(const auto& point : trajectory.points)
                    if(point.q[j] < bounds[j].lower - 1e-9 || point.q[j] > bounds[j].upper + 1e-9)
                        ++result.positionLimitViolations;
            }
        }
        result.jointVelocityLimits = vmax;
        result.jointAccelerationLimits = amax;
        if(result.timingValid) {
            result.peakVelocity = result.peakAcceleration = 0.0;
            result.jointPeakVelocities.assign(dimension, 0.0);
            result.jointPeakAccelerations.assign(dimension, 0.0);
            std::vector<std::vector<double>> velocities(dt.size(), std::vector<double>(dimension));
            for(std::size_t i = 0; i < dt.size(); ++i) for(std::size_t j = 0; j < dimension; ++j) {
                const double v = (path[i + 1][j] - path[i][j]) / dt[i];
                velocities[i][j] = v;
                result.jointPeakVelocities[j] = std::max(result.jointPeakVelocities[j], std::abs(v));
                if(vmax[j] > 0.0 && std::abs(v) > vmax[j] * (1.0 + 1e-8)) ++result.velocityLimitViolations;
            }
            for(std::size_t i = 0; i < count; ++i) {
                auto& point = trajectory.points[i];
                point.qd.assign(dimension, 0.0); point.qdd.assign(dimension, 0.0);
                if(count == 1) continue;
                for(std::size_t j = 0; j < dimension; ++j) {
                    double a = 0.0;
                    if(i == 0) a = 2.0 * velocities.front()[j] / dt.front();
                    else if(i + 1 == count) a = -2.0 * velocities.back()[j] / dt.back();
                    else {
                        point.qd[j] = (velocities[i - 1][j] * dt[i] + velocities[i][j] * dt[i - 1]) / (dt[i - 1] + dt[i]);
                        a = 2.0 * (velocities[i][j] - velocities[i - 1][j]) / (dt[i - 1] + dt[i]);
                    }
                    point.qdd[j] = a;
                    result.jointPeakAccelerations[j] = std::max(result.jointPeakAccelerations[j], std::abs(a));
                    if(amax[j] > 0.0 && std::abs(a) > amax[j] * (1.0 + 1e-8)) ++result.accelerationLimitViolations;
                }
            }
            for(std::size_t j = 0; j < dimension; ++j) {
                result.peakVelocity = std::max(result.peakVelocity, result.jointPeakVelocities[j]);
                result.peakAcceleration = std::max(result.peakAcceleration, result.jointPeakAccelerations[j]);
            }
        }
        if(fk) {
            result.tcpAvailable = true;
            for(std::size_t i = 0; i < count; ++i) {
                const Eigen::Isometry3d pose = fk(path[i]);
                if(!pose.matrix().allFinite()) { result.tcpAvailable = false; break; }
                result.tcpPositions.push_back(pose.translation());
                if(reference && reference->points.size() == count) {
                    const auto target = fk(reference->points[i].q);
                    if(!target.matrix().allFinite()) { result.tcpAvailable = false; break; }
                    result.maximumTcpDeviation = std::max(result.maximumTcpDeviation, (pose.translation() - target.translation()).norm());
                    const double cosine = std::clamp((target.linear().transpose() * pose.linear()).trace() * 0.5 - 0.5, -1.0, 1.0);
                    result.maximumOrientationDeviation = std::max(result.maximumOrientationDeviation, std::acos(cosine));
                }
            }
            if(result.tcpAvailable) {
                if(result.timingValid) result.peakTcpSpeed = 0.0;
                for(std::size_t i = 1; i < count; ++i) {
                    const double distance = (result.tcpPositions[i] - result.tcpPositions[i - 1]).norm();
                    result.tcpLength += distance;
                    if(result.timingValid) result.peakTcpSpeed = std::max(result.peakTcpSpeed, distance / dt[i - 1]);
                }
            } else result.tcpPositions.clear();
        }
        return result;
    }

    bool retimeJointTrajectory(robottrajectory::JointTrajectory& trajectory,
        const std::vector<JointBound>& bounds, double fallbackVelocity, double fallbackAcceleration)
    {
        if(!validate(trajectory) || bounds.size() != trajectory.points.front().q.size() ||
            !std::isfinite(fallbackVelocity) || fallbackVelocity <= 0.0 ||
            !std::isfinite(fallbackAcceleration) || fallbackAcceleration <= 0.0) return false;
        const auto count = trajectory.points.size(), dimension = bounds.size();
        std::vector<double> intervals;
        for(std::size_t i = 1; i < count; ++i) {
            const double dt = trajectory.points[i].time - trajectory.points[i - 1].time;
            if(dt > 0.0) intervals.push_back(dt);
        }
        std::sort(intervals.begin(), intervals.end());
        const double median = intervals.empty() ? 0.01 : intervals[intervals.size() / 2];
        std::vector<double> originalTimes;
        for(const auto& point : trajectory.points) originalTimes.push_back(point.time);
        for(std::size_t i = 1; i < count; ++i) {
            double dt = originalTimes[i] - originalTimes[i - 1];
            if(dt <= 0.0) dt = median;
            for(std::size_t j = 0; j < dimension; ++j) dt = std::max(dt,
                std::abs(trajectory.points[i].q[j] - trajectory.points[i - 1].q[j]) /
                limit(bounds[j].maxVelocity, fallbackVelocity));
            trajectory.points[i].time = trajectory.points[i - 1].time + std::max(0.001, dt);
        }
        auto measured = evaluateTrajectoryQuality(trajectory, bounds, fallbackVelocity, fallbackAcceleration);
        if(!measured.valid || !measured.timingValid) return false;
        double scale = 1.0;
        for(std::size_t j = 0; j < dimension; ++j) {
            scale = std::max(scale, measured.jointPeakVelocities[j] / limit(bounds[j].maxVelocity, fallbackVelocity));
            scale = std::max(scale, std::sqrt(measured.jointPeakAccelerations[j] / limit(bounds[j].maxAcceleration, fallbackAcceleration)));
        }
        const double begin = trajectory.points.front().time;
        for(auto& point : trajectory.points) point.time = begin + (point.time - begin) * scale * 1.001;
        measured = evaluateTrajectoryQuality(trajectory, bounds, fallbackVelocity, fallbackAcceleration);
        return measured.timingValid && measured.positionLimitViolations == 0 &&
            measured.velocityLimitViolations == 0 && measured.accelerationLimitViolations == 0;
    }
}
