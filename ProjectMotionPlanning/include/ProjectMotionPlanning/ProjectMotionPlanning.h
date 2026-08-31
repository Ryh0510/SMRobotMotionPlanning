#pragma once

#include <MotionPlanningCore/MotionPlanning.h>
#include <SimulationRuntime/ProjectCollisionRuntime.h>
#include <SimulationRuntime/ProjectSimulationRuntime.h>

#include <Eigen/Geometry>

#include <filesystem>
#include <memory>
#include <string>
#include <vector>

namespace simulation_project
{
    struct ProjectDocument;
}

namespace motion_planning
{
    struct ProjectPlanningRequest
    {
        std::string robotId;
        std::vector<std::string> jointNames;
        std::vector<double> start;
        std::vector<double> goal;
        std::vector<std::string> collisionDetectorIds;
        PlannerOptions planner;
        MotionValidationOptions validation;
        TrajectoryPostProcessOptions postProcess;
    };

    class ProjectPlanningSceneSnapshot final : public IPlanningScene
    {
    public:
        ~ProjectPlanningSceneSnapshot() override;

        ProjectPlanningSceneSnapshot(const ProjectPlanningSceneSnapshot&) = delete;
        ProjectPlanningSceneSnapshot& operator=(const ProjectPlanningSceneSnapshot&) = delete;

        StateValidationResult validateState(
            const std::vector<double>& jointValues) override;

        StateValidationResult validateMotion(
            const std::vector<double>& from,
            const std::vector<double>& to,
            const MotionValidationOptions& options) override;

        const std::string& robotId() const;
        const std::vector<std::string>& jointNames() const;
        const std::vector<JointBound>& jointBounds() const;
        const std::vector<std::string>& collisionDetectorIds() const;

        bool setState(const std::vector<double>& jointValues, std::string* errorMessage = nullptr);
        Eigen::Isometry3d linkWorldTransform(const std::string& linkName) const;
        Eigen::Isometry3d attachmentWorldTransform(const std::string& attachmentId) const;

        simulation_runtime::ProjectSimulationRuntime& simulationRuntime();
        const simulation_runtime::ProjectSimulationRuntime& simulationRuntime() const;
        simulation_runtime::ProjectCollisionRuntime& collisionRuntime();
        const simulation_runtime::ProjectCollisionRuntime& collisionRuntime() const;

    private:
        class Impl;
        explicit ProjectPlanningSceneSnapshot(std::unique_ptr<Impl> impl);

        std::unique_ptr<Impl> m_impl;

        friend class ProjectPlanningSceneBuilder;
    };

    class ProjectPlanningSceneBuilder
    {
    public:
        static std::unique_ptr<ProjectPlanningSceneSnapshot> build(
            const simulation_project::ProjectDocument& document,
            const std::filesystem::path& projectBasePath,
            const ProjectPlanningRequest& request,
            std::string* errorMessage = nullptr);
    };

    class ProjectMotionPlanningService
    {
    public:
        MotionPlanningResult plan(
            const simulation_project::ProjectDocument& document,
            const std::filesystem::path& projectBasePath,
            const ProjectPlanningRequest& request,
            CancellationToken* cancellation = nullptr) const;

        MotionPlanningResult plan(
            ProjectPlanningSceneSnapshot& scene,
            const ProjectPlanningRequest& request,
            CancellationToken* cancellation = nullptr) const;
    };
}
