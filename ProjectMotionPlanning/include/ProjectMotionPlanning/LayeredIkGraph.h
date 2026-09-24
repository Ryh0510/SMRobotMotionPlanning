#pragma once

#include <ProjectMotionPlanning/TrajectoryInverseKinematics.h>

namespace motion_planning
{
    struct LayeredIkGraphOptions
    {
        std::size_t maxPaths = 30;
        // Empty means unit weights. Units: radians, cost = sum w[j] * dq[j]^2.
        std::vector<double> jointWeights;
        std::size_t maxStoredPrefixes = 8000000;
        std::function<bool()> cancelled;
        std::function<void(std::size_t, std::size_t)> progress;
    };

    struct LayeredIkPath
    {
        double cost = 0.0;
        // Zero-based candidate index at every original Cartesian control point.
        std::vector<std::size_t> selections;
    };

    struct LayeredIkGraphResult
    {
        bool success = false;
        bool cancelled = false;
        std::vector<LayeredIkPath> paths;
        std::string message;
    };

    class ProjectLayeredIkGraph
    {
    public:
        // Consumes the already FK/limit-validated IK result. No collision checks,
        // node penalties, velocity constraints or wrapToPi; q includes all turns.
        // Exact Top-M over the supplied candidate sets, not over undiscovered IK roots.
        static LayeredIkGraphResult filter(const CartesianMultiIkResult& ik,
            const LayeredIkGraphOptions& options = {});
    };
}
