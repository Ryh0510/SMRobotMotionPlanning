#pragma once

#include <MotionPlanningCore/MotionPlanning.h>

#include <Eigen/Geometry>

#include <cstddef>
#include <functional>
#include <filesystem>
#include <string>
#include <vector>

namespace simulation_project
{
    struct ProjectDocument;
}

namespace motion_planning
{
    enum class CartesianIkToolMode
    {
        Flange,
        FixedTool
    };

    struct CartesianIkOptions
    {
        std::string robotId;
        std::vector<std::string> jointNames;
        std::vector<double> seedJoints;
        CartesianIkToolMode toolMode = CartesianIkToolMode::FixedTool;
        bool usePreviousSolutionAsSeed = true;
        int maxIterations = 1000;
        double tolerance = 1.0e-6;
        double stepSize = 0.1;
        double damping = 0.01;
        // Optional actual-model FK in world coordinates, consuming the stored IK joint
        // convention. Includes the chosen TCP/flange transform; toolMode is then ignored.
        // Without it, the legacy nominal IRB4600 DH model remains available.
        std::function<Eigen::Isometry3d(const std::vector<double>&)> worldForwardKinematics;
    };

    struct CartesianIkPointResult
    {
        std::size_t pointIndex = 0;
        bool success = false;
        std::vector<double> joints;
        double errorNorm = 0.0;
        std::string message;
    };

    struct CartesianIkResult
    {
        bool success = false;
        StoredMotionPlan plan;
        std::vector<CartesianIkPointResult> points;
        std::vector<MotionPlanningDiagnostic> diagnostics;

        std::size_t solvedPointCount() const;
    };

    // All angles are in the stored IK convention, in radians. Bounds are finite
    // search windows intersected with known model limits; turn values are retained.
    struct CartesianMultiIkOptions
    {
        CartesianIkOptions model;
        std::vector<double> lower;
        std::vector<double> upper;
        int seedCount = 64;
        int maxIterations = 160;
        std::size_t maxCandidatesPerPoint = 512;
        double positionTolerance = 1.0e-6;
        double orientationTolerance = 1.0e-6;
        double duplicateTolerance = 1.0e-4;
        std::function<bool()> cancelled;
        std::function<void(std::size_t, std::size_t)> progress;
    };

    struct CartesianIkCandidate
    {
        std::vector<double> joints;
        std::vector<int> turns;
        double positionError = 0.0;
        double orientationError = 0.0;
    };

    struct CartesianIkLayer
    {
        std::size_t pointIndex = 0;
        double time = 0.0;
        std::vector<CartesianIkCandidate> candidates;
        bool truncated = false;
        std::string message;
    };

    struct CartesianMultiIkResult
    {
        bool success = false;
        bool cancelled = false;
        StoredMotionPlan source;
        std::vector<CartesianIkLayer> layers;
        std::string message;
    };

    class ProjectTrajectoryInverseKinematics
    {
    public:
        // Numerical multi-start enumeration, not a proof of exhaustive analytic IK.
        static CartesianMultiIkResult solveAllCartesianControlPoints(
            const StoredMotionPlan& plan, const CartesianMultiIkOptions& options);

        // Read actual model limits without requiring collision configuration.
        // Continuous axes return infinite limits; callers must provide a finite window.
        static bool readRevoluteJointLimits(
            const simulation_project::ProjectDocument& document,
            const std::filesystem::path& basePath, const std::string& robotId,
            const std::vector<std::string>& names, std::vector<double>& lower,
            std::vector<double>& upper, std::string& error);

        static bool selectMultiIkTrajectory(const CartesianMultiIkResult& result,
            const std::vector<std::size_t>& selections, StoredMotionPlan& plan,
            std::string& error);

        static std::vector<std::string> defaultIrb4600JointNames();
        static std::vector<double> irb4600RobotSystemJointValues(
            const std::vector<double>& ikJointValues);

        static CartesianIkResult solveCartesianControlPoints(
            const simulation_project::ProjectDocument& document,
            const StoredMotionPlan& plan,
            const CartesianIkOptions& options);

        static bool applyJointValuesToRobotInitialJoints(
            simulation_project::ProjectDocument& document,
            const std::string& robotId,
            const std::vector<std::string>& jointNames,
            const std::vector<double>& jointValues,
            std::string* errorMessage = nullptr);
    };
}
