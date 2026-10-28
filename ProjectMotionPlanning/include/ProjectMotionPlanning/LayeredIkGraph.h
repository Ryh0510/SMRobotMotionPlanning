#pragma once

#include <ProjectMotionPlanning/TrajectoryInverseKinematics.h>

namespace motion_planning
{
    struct LayeredIkGraphOptions
    {
        std::size_t maxPaths = 30;
        // Empty means unit weights. Units: radians, cost = sum w[j] * dq[j]^2.
        std::vector<double> jointWeights;
        // Empty disables velocity checks; otherwise positive limits per joint, units/s.
        std::vector<double> maxJointVelocities;
        // Zero disables the hard unwrapped per-axis jump limit.
        double maxJointDelta = 0.0;
        // Called synchronously, once per node / reachable edge per search.
        // Use these for collision, singularity and process constraints on an isolated scene.
        std::function<bool(std::size_t, const CartesianIkCandidate&)> nodeAdmissible;
        // Optional nonnegative soft node cost. Empty preserves legacy ranking.
        std::function<double(std::size_t, const CartesianIkCandidate&)> nodeCost;
        std::function<bool(std::size_t, const CartesianIkCandidate&, const CartesianIkCandidate&)> edgeAdmissible;
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
        // Consumes the already FK/limit-validated IK result. Optional hard constraints
        // remove nodes/edges before ranking. No implicit angle wrapping; q includes all turns.
        // Exact Top-M over the supplied candidate sets, not over undiscovered IK roots.
        static LayeredIkGraphResult filter(const CartesianMultiIkResult& ik,
            const LayeredIkGraphOptions& options = {});

        // Independently solve Top-K for EVERY candidate in the first layer.
        // options.maxPaths is K per start. Paths are ordered by start index, then
        // cost within that start; selections retain original candidate indices.
        // Failure/cancellation returns no partial groups. Same optional constraints as filter.
        static LayeredIkGraphResult filterByStart(const CartesianMultiIkResult& ik,
            const LayeredIkGraphOptions& options = {});
    };
}
