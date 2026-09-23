#pragma once

#include <MotionPlanningCore/MotionPlanning.h>

#include <Eigen/Geometry>

#include <cstddef>
#include <functional>
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

    class ProjectTrajectoryInverseKinematics
    {
    public:
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
