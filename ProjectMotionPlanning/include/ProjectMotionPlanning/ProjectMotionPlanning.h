#pragma once

#include <MotionPlanningCore/MotionPlanning.h>
#include <SimulationRuntime/ProjectCollisionRuntime.h>
#include <SimulationRuntime/ProjectSimulationRuntime.h>

#include <Eigen/Geometry>

#include <filesystem>
#include <memory>
#include <limits>
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

    struct PlanningSceneDistanceSample
    {
        bool valid = false;
        bool inCollision = false;
        double distanceMeters = std::numeric_limits<double>::quiet_NaN();
        std::string detectorId;
        std::string message;
    };

    struct PlanningSceneQueryCounts
    {
        std::size_t stateValidationRequests=0, collisionDetectorChecks=0, baffleChecks=0;
        std::size_t nearestDistanceRequests=0, distanceDetectorQueries=0;
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

        void useConservativeMotionValidation(const collision::MotionQueryRequest& request,
            const std::vector<std::string>& collisionFreeLinks);
        void useSampledMotionValidation();

        const std::string& robotId() const;
        const std::vector<std::string>& jointNames() const;
        const std::vector<JointBound>& jointBounds() const;
        const std::vector<std::string>& collisionDetectorIds() const;

        // Nonnegative geometric separation; collision is zero, not penetration depth.
        // Uses the snapshot's detector filters, without changing detector options.
        PlanningSceneDistanceSample nearestDistance(const std::vector<double>& jointValues);

        bool setState(const std::vector<double>& jointValues, std::string* errorMessage = nullptr);
        Eigen::Isometry3d linkWorldTransform(const std::string& linkName) const;
        Eigen::Isometry3d attachmentWorldTransform(const std::string& attachmentId) const;

        std::string virtualBaffleStatus() const;
        bool validateVirtualBaffles(double clearance, std::string& reason) const;
        double virtualBaffleSearchViolation(double clearance) const;
        std::vector<double> virtualBaffleSearchViolations(const std::vector<std::vector<double>>&,double clearance,bool gpu);
        std::string virtualBaffleSearchStatus() const;
        PlanningSceneQueryCounts queryCounts() const;
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
