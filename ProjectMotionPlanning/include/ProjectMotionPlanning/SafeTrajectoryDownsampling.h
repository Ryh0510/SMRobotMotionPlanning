#pragma once

#include <MotionPlanningCore/MotionPlanning.h>
#include <functional>
#include <string>
#include <vector>

namespace motion_planning
{
    struct SafeTrajectoryDownsamplingOptions
    {
        std::size_t targetPoints = 4000;
        // Includes endpoints and ALL required samples of the new joint-linear edge.
        // A rejected shortcut is split until safe, even when this exceeds targetPoints.
        std::function<bool(std::size_t, std::size_t)> edgeAdmissible;
        std::function<bool()> cancelled;
        std::function<void(std::size_t, std::size_t)> progress;
        std::vector<std::size_t> requiredIndices;
    };

    struct SafeTrajectoryDownsamplingResult
    {
        bool success = false;
        bool cancelled = false;
        robottrajectory::JointTrajectory trajectory;
        std::vector<std::size_t> sourceIndices;
        std::size_t checkedEdges = 0;
        std::string message;
    };

    // Geometry, joint turns, endpoints and source times are preserved at retained
    // nodes. Collision/process policy belongs to the caller's isolated scene.
    SafeTrajectoryDownsamplingResult downsampleTrajectorySafely(
        const robottrajectory::JointTrajectory& source,
        const SafeTrajectoryDownsamplingOptions& options);
}
