#pragma once

#include <RobotTrajectoryCore/RobotTrajectory.h>

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace simulation_project
{
    struct ProjectDocument;
}

namespace motion_planning
{
    struct MotionPlanningConstraint
    {
        double duration = 5.0;
        std::size_t sampleCount = 50;
    };

    struct MotionPlanningRequest
    {
        std::string robotId;
        std::vector<std::string> jointNames;
        std::vector<double> startJoints;
        std::vector<double> goalJoints;
        MotionPlanningConstraint constraint;
    };

    enum class MotionPlanningStatus
    {
        Success,
        InvalidRequest,
        NoSolution,
        Timeout,
        Failed,
        Cancelled
    };

    struct MotionPlanningDiagnostic
    {
        std::string code;
        std::string message;
    };

    struct MotionPlanningResult
    {
        MotionPlanningStatus status = MotionPlanningStatus::Failed;
        robottrajectory::JointTrajectory trajectory;
        std::vector<MotionPlanningDiagnostic> diagnostics;
        double planningTimeSeconds = 0.0;
        std::size_t sampledStateCount = 0;

        bool succeeded() const;
    };

    struct JointBound
    {
        double lower = 0.0;
        double upper = 0.0;
        double maxVelocity = 0.0;
        double maxAcceleration = 0.0;
        bool continuous = false;
    };

    struct PlannerOptions
    {
        std::string plannerId = "RRTConnect";
        double timeoutSeconds = 5.0;
        double range = 0.0;
        std::uint32_t randomSeed = 1;
        bool simplifyPath = true;
    };

    struct MotionValidationOptions
    {
        double maxJointStep = 0.05;
    };

    struct TrajectoryPostProcessOptions
    {
        double duration = 5.0;
        std::size_t minimumWaypointCount = 2;
    };

    struct JointPlanningProblem
    {
        std::string robotId;
        std::vector<std::string> jointNames;
        std::vector<JointBound> jointBounds;
        std::vector<double> start;
        std::vector<double> goal;
        PlannerOptions planner;
        MotionValidationOptions validation;
        TrajectoryPostProcessOptions postProcess;
    };

    struct StateValidationResult
    {
        bool valid = false;
        std::string diagnosticCode;
        std::string message;

        static StateValidationResult success();
        static StateValidationResult failure(std::string code, std::string message);
    };

    class CancellationToken
    {
    public:
        void cancel();
        bool isCancellationRequested() const;

    private:
        std::atomic_bool m_cancelled{ false };
    };

    class IPlanningScene
    {
    public:
        virtual ~IPlanningScene() = default;

        virtual StateValidationResult validateState(
            const std::vector<double>& jointValues) = 0;

        virtual StateValidationResult validateMotion(
            const std::vector<double>& from,
            const std::vector<double>& to,
            const MotionValidationOptions& options) = 0;
    };

    class IJointMotionPlanner
    {
    public:
        virtual ~IJointMotionPlanner() = default;

        virtual MotionPlanningResult plan(
            const JointPlanningProblem& problem,
            IPlanningScene& scene,
            CancellationToken* cancellation = nullptr) const = 0;
    };

    class IMotionPlanner
    {
    public:
        virtual ~IMotionPlanner() = default;
        virtual MotionPlanningResult plan(const MotionPlanningRequest& request) const = 0;
    };

    class LinearJointMotionPlanner final : public IMotionPlanner
    {
    public:
        MotionPlanningResult plan(const MotionPlanningRequest& request) const override;
    };

    struct StoredMotionPlan
    {
        std::string id;
        std::string name;
        std::string robotId;
        std::vector<std::string> jointNames;
        robottrajectory::JointTrajectory trajectory;
        robottrajectory::CartesianTrajectory cartesianControlPoints;
    };

    class MotionPlanningProjectStore
    {
    public:
        static const char* extensionKey();
        static std::vector<StoredMotionPlan> plans(
            const simulation_project::ProjectDocument& document);
        static bool upsertPlan(
            simulation_project::ProjectDocument& document,
            const StoredMotionPlan& plan,
            std::string* errorMessage = nullptr);
    };
}
