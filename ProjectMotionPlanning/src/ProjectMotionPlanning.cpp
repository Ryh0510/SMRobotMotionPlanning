#include <ProjectMotionPlanning/ProjectMotionPlanning.h>

#include <MotionPlanningOmpl/OmplMotionPlanner.h>
#include <SimulationProject/ProjectDocument.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <set>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace motion_planning
{
    namespace
    {
        constexpr double kPi = 3.14159265358979323846;
        constexpr double kTwoPi = 2.0 * kPi;

        bool motionSnapshotCoversDetector(
            const simulation_runtime::ProjectCollisionRuntime& runtime,
            const std::string& detectorId)
        {
            const auto& detectors = runtime.detectors();
            const auto detector = std::find_if(detectors.begin(), detectors.end(),
                [&](const auto& value) { return value.id == detectorId; });
            if(detector == detectors.end() || !detector->enabled || !detector->valid)
                return false;

            // The 1.0.6 SDK can omit selected link pairs from its motion snapshot.
            // A Clear verdict only certifies the pairs present in that snapshot.
            // Require the same effective pairs as the existing detector first.
            using Pair = std::pair<collision::ObjectID, collision::ObjectID>;
            std::set<Pair> expected;
            for(const auto& pair : runtime.pairDiagnostics(detectorId)) {
                if(pair.effective)
                    expected.insert(std::minmax(pair.first.object, pair.second.object));
            }
            collision::CollisionMotionSnapshot snapshot;
            if(runtime.scene().motionSnapshot(detector->options, snapshot) != collision::CollisionStatus::Success)
                return false;
            std::set<Pair> actual;
            for(const auto& pair : snapshot.pairs) {
                if(pair.first >= snapshot.objects.size() || pair.second >= snapshot.objects.size())
                    return false;
                actual.insert(std::minmax(snapshot.objects[pair.first].key, snapshot.objects[pair.second].key));
            }
            return actual == expected;
        }

        void setError(std::string* errorMessage, const std::string& message)
        {
            if (errorMessage != nullptr)
                *errorMessage = message;
        }

        MotionPlanningResult invalidResult(const std::string& code, const std::string& message)
        {
            MotionPlanningResult result;
            result.status = MotionPlanningStatus::InvalidRequest;
            result.diagnostics.push_back({ code, message });
            return result;
        }

        bool hasFiniteOrderedPositionLimits(const robot::RobotJoint& joint)
        {
            return joint.hasPositionLimits &&
                std::isfinite(joint.lowerPositionLimit) &&
                std::isfinite(joint.upperPositionLimit) &&
                joint.lowerPositionLimit < joint.upperPositionLimit;
        }

        double wrapContinuousAngle(double value)
        {
            if (!std::isfinite(value))
                return value;

            double wrapped = std::remainder(value, kTwoPi);
            if (wrapped <= -kPi)
                wrapped += kTwoPi;
            else if (wrapped > kPi)
                wrapped -= kTwoPi;
            return wrapped;
        }

        double shortestContinuousDelta(double from, double to)
        {
            double delta = std::remainder(to - from, kTwoPi);
            if (delta <= -kPi)
                delta += kTwoPi;
            else if (delta > kPi)
                delta -= kTwoPi;
            return delta;
        }

        double normalizedJointValueForBounds(
            double value,
            const JointBound& bound)
        {
            if (!bound.continuous)
                return value;
            return wrapContinuousAngle(value);
        }

        std::vector<double> normalizedJointValuesForBounds(
            const std::vector<double>& values,
            const std::vector<JointBound>& bounds)
        {
            std::vector<double> normalized = values;
            const std::size_t count = std::min(normalized.size(), bounds.size());
            for (std::size_t index = 0; index < count; ++index)
                normalized[index] = normalizedJointValueForBounds(normalized[index], bounds[index]);
            return normalized;
        }

        JointBound planningBoundForJoint(const robot::RobotJoint& joint)
        {
            JointBound bound;
            if (joint.continuous)
            {
                bound.lower = -kPi;
                bound.upper = kPi;
            }
            else
            {
                bound.lower = joint.lowerPositionLimit;
                bound.upper = joint.upperPositionLimit;
            }
            bound.maxVelocity = joint.hasVelocityLimit ? joint.maxVelocity : 0.0;
            bound.maxAcceleration = joint.hasAccelerationLimit ? joint.maxAcceleration : 0.0;
            bound.continuous = joint.continuous;
            return bound;
        }

        const simulation_project::CollisionDetectorDesc* findDetector(
            const simulation_project::ProjectDocument& document,
            const std::string& detectorId)
        {
            const auto detectorIt = std::find_if(
                document.collision.detectors.begin(),
                document.collision.detectors.end(),
                [&](const simulation_project::CollisionDetectorDesc& detector) {
                    return detector.id == detectorId;
                });
            return detectorIt == document.collision.detectors.end() ? nullptr : &(*detectorIt);
        }

        const robot::RobotJoint* findJoint(
            const robot::RobotModel& model,
            const std::string& jointName)
        {
            const auto jointIt = model.jointNameToIndex.find(jointName);
            if (jointIt == model.jointNameToIndex.end())
                return nullptr;
            return &model.joints[static_cast<std::size_t>(jointIt->second)];
        }

        std::vector<std::string> movableJointNames(const robot::RobotModel& model)
        {
            std::vector<const robot::RobotJoint*> ordered(model.dof(), nullptr);
            for (const robot::RobotJoint& joint : model.joints)
            {
                if (joint.dofIndex >= 0 && static_cast<std::size_t>(joint.dofIndex) < ordered.size())
                    ordered[static_cast<std::size_t>(joint.dofIndex)] = &joint;
            }

            std::vector<std::string> result;
            result.reserve(ordered.size());
            for (const robot::RobotJoint* joint : ordered)
            {
                if (joint != nullptr)
                    result.push_back(joint->name);
            }
            return result;
        }
    }

    class ProjectPlanningSceneSnapshot::Impl
    {
    public:
        simulation_runtime::ProjectSimulationRuntime simulation;
        simulation_runtime::ProjectCollisionRuntime collision;
        std::string robotId;
        std::vector<std::string> jointNames;
        std::vector<JointBound> jointBounds;
        std::vector<std::string> detectorIds;
        bool conservativeMotion = false;
        collision::MotionQueryRequest motionRequest;
        std::vector<std::string> collisionFreeLinks;

        StateValidationResult applyState(const std::vector<double>& jointValues, bool checkCollision)
        {
            if (jointValues.size() != jointNames.size())
                return StateValidationResult::failure("state_size_mismatch", "Joint state size does not match the planning scene.");

            std::vector<double> runtimeJointValues = jointValues;
            for (std::size_t index = 0; index < jointValues.size(); ++index)
            {
                const double value = normalizedJointValueForBounds(jointValues[index], jointBounds[index]);
                const JointBound& bound = jointBounds[index];
                if (!std::isfinite(jointValues[index]) || !std::isfinite(value))
                    return StateValidationResult::failure("non_finite_state", "Joint state contains a non-finite value.");
                if (value < bound.lower || value > bound.upper)
                    return StateValidationResult::failure("state_out_of_bounds", "Joint state is outside the configured bounds.");
                runtimeJointValues[index] = value;
            }

            const simulation_runtime::Result setResult = simulation.setRobotJointValues(
                robotId,
                jointNames,
                runtimeJointValues);
            if (!setResult.success)
                return StateValidationResult::failure("runtime_state_update_failed", setResult.message);

            const simulation_runtime::Result updateResult = collision.update(simulation);
            if (!updateResult.success)
                return StateValidationResult::failure("collision_update_failed", updateResult.message);
            if (!checkCollision)
                return StateValidationResult::success();

            for (const std::string& detectorId : detectorIds)
            {
                const simulation_runtime::Result checkResult = collision.checkDetector(detectorId);
                if (!checkResult.success)
                    return StateValidationResult::failure("collision_query_failed", checkResult.message);

                const collision::CollisionResult* result = collision.resultOf(detectorId);
                if (result == nullptr)
                    return StateValidationResult::failure("collision_result_missing", "Collision detector produced no reliable result: " + detectorId);
                if (result->inCollision())
                    return StateValidationResult::failure("state_in_collision", "State is in collision according to detector: " + detectorId);
            }
            return StateValidationResult::success();
        }
    };

    ProjectPlanningSceneSnapshot::ProjectPlanningSceneSnapshot(std::unique_ptr<Impl> impl)
        : m_impl(std::move(impl))
    {
    }

    ProjectPlanningSceneSnapshot::~ProjectPlanningSceneSnapshot() = default;

    StateValidationResult ProjectPlanningSceneSnapshot::validateState(
        const std::vector<double>& jointValues)
    {
        return m_impl->applyState(jointValues, true);
    }

    StateValidationResult ProjectPlanningSceneSnapshot::validateMotion(
        const std::vector<double>& from,
        const std::vector<double>& to,
        const MotionValidationOptions& options)
    {
        if (from.size() != m_impl->jointNames.size() || to.size() != from.size())
            return StateValidationResult::failure("motion_size_mismatch", "Motion endpoint sizes do not match the planning scene.");
        if (m_impl->conservativeMotion)
        {
            for(const auto& detectorId : m_impl->detectorIds) {
                if(!motionSnapshotCoversDetector(m_impl->collision, detectorId))
                    return StateValidationResult::failure("motion_indeterminate",
                        "Motion snapshot does not cover the effective detector pairs: " + detectorId);
            }
            const auto* robot = m_impl->simulation.robot(m_impl->robotId);
            if (!robot || !robot->instance)
                return StateValidationResult::failure("motion_robot_missing", "Planning robot is unavailable.");
            auto fullFrom = robot->instance->getState().q;
            auto fullTo = fullFrom;
            for (std::size_t i=0;i<from.size();++i) {
                const auto& bound=m_impl->jointBounds[i];
                const double first = normalizedJointValueForBounds(from[i], bound);
                const double last = bound.continuous
                    ? first + shortestContinuousDelta(from[i], to[i]) : to[i];
                if (!std::isfinite(from[i]) || !std::isfinite(to[i]) ||
                    !std::isfinite(first) || !std::isfinite(last) ||
                    (!bound.continuous &&
                     (std::min(first, last) < bound.lower || std::max(first, last) > bound.upper)))
                    return StateValidationResult::failure("motion_out_of_bounds", "Invalid joint segment.");
                const auto* joint=findJoint(robot->model,m_impl->jointNames[i]);
                if (!joint || joint->dofIndex<0 || static_cast<std::size_t>(joint->dofIndex)>=fullFrom.size())
                    return StateValidationResult::failure("motion_joint_mapping", "Invalid joint mapping.");
                fullFrom[joint->dofIndex] = first;
                fullTo[joint->dofIndex] = last;
            }
            const auto results=m_impl->collision.queryRobotJointMotion(m_impl->simulation,m_impl->robotId,
                fullFrom,fullTo,m_impl->detectorIds,m_impl->collisionFreeLinks,m_impl->motionRequest);
            if (results.empty()) return StateValidationResult::failure("motion_no_detectors", "No motion validation result.");
            for (const auto& result:results) {
                if (result.status!=collision::CollisionStatus::Success || result.verdict!=collision::MotionVerdict::Clear)
                    return StateValidationResult::failure(
                        result.verdict==collision::MotionVerdict::Collision?"motion_in_collision":"motion_indeterminate",result.message);
            }
            return StateValidationResult::success();
        }
        if (!(options.maxJointStep > 0.0) || !std::isfinite(options.maxJointStep))
            return StateValidationResult::failure("invalid_motion_step", "Maximum joint validation step must be positive and finite.");

        double maximumDelta = 0.0;
        for (std::size_t index = 0; index < from.size(); ++index)
        {
            const JointBound& bound = m_impl->jointBounds[index];
            const double delta = bound.continuous
                ? shortestContinuousDelta(from[index], to[index])
                : to[index] - from[index];
            maximumDelta = std::max(maximumDelta, std::abs(delta));
        }
        const std::size_t stepCount = std::max<std::size_t>(
            1,
            static_cast<std::size_t>(std::ceil(maximumDelta / options.maxJointStep)));

        std::vector<double> sample(from.size(), 0.0);
        for (std::size_t step = 0; step <= stepCount; ++step)
        {
            const double ratio = static_cast<double>(step) / static_cast<double>(stepCount);
            for (std::size_t index = 0; index < sample.size(); ++index)
            {
                const JointBound& bound = m_impl->jointBounds[index];
                const double delta = bound.continuous
                    ? shortestContinuousDelta(from[index], to[index])
                    : to[index] - from[index];
                sample[index] = from[index] + delta * ratio;
                if (bound.continuous)
                    sample[index] = wrapContinuousAngle(sample[index]);
            }

            StateValidationResult result = m_impl->applyState(sample, true);
            if (!result.valid)
            {
                std::ostringstream message;
                message << "Motion is invalid at interpolation ratio " << ratio;
                if (!result.message.empty())
                    message << ": " << result.message;
                result.message = message.str();
                return result;
            }
        }
        return StateValidationResult::success();
    }

    void ProjectPlanningSceneSnapshot::useConservativeMotionValidation(
        const collision::MotionQueryRequest& request,const std::vector<std::string>& collisionFreeLinks)
    {
        m_impl->motionRequest=request;
        m_impl->motionRequest.mode=collision::MotionQueryMode::Conservative;
        m_impl->collisionFreeLinks=collisionFreeLinks;
        m_impl->conservativeMotion=true;
    }

    void ProjectPlanningSceneSnapshot::useSampledMotionValidation()
    {
        m_impl->conservativeMotion=false;
    }

    const std::string& ProjectPlanningSceneSnapshot::robotId() const
    {
        return m_impl->robotId;
    }

    const std::vector<std::string>& ProjectPlanningSceneSnapshot::jointNames() const
    {
        return m_impl->jointNames;
    }

    const std::vector<JointBound>& ProjectPlanningSceneSnapshot::jointBounds() const
    {
        return m_impl->jointBounds;
    }

    const std::vector<std::string>& ProjectPlanningSceneSnapshot::collisionDetectorIds() const
    {
        return m_impl->detectorIds;
    }

    bool ProjectPlanningSceneSnapshot::setState(
        const std::vector<double>& jointValues,
        std::string* errorMessage)
    {
        const StateValidationResult result = m_impl->applyState(jointValues, false);
        setError(errorMessage, result.valid ? std::string() : result.message);
        return result.valid;
    }

    Eigen::Isometry3d ProjectPlanningSceneSnapshot::linkWorldTransform(const std::string& linkName) const
    {
        const simulation_runtime::RuntimeRobot* runtime = m_impl->simulation.robot(m_impl->robotId);
        if (runtime == nullptr || !runtime->instance)
            return Eigen::Isometry3d::Identity();
        return runtime->instance->getLinkTransform(linkName);
    }

    Eigen::Isometry3d ProjectPlanningSceneSnapshot::attachmentWorldTransform(const std::string& attachmentId) const
    {
        return m_impl->simulation.attachmentWorldTransform(attachmentId);
    }

    simulation_runtime::ProjectSimulationRuntime& ProjectPlanningSceneSnapshot::simulationRuntime()
    {
        return m_impl->simulation;
    }

    const simulation_runtime::ProjectSimulationRuntime& ProjectPlanningSceneSnapshot::simulationRuntime() const
    {
        return m_impl->simulation;
    }

    simulation_runtime::ProjectCollisionRuntime& ProjectPlanningSceneSnapshot::collisionRuntime()
    {
        return m_impl->collision;
    }

    const simulation_runtime::ProjectCollisionRuntime& ProjectPlanningSceneSnapshot::collisionRuntime() const
    {
        return m_impl->collision;
    }

    std::unique_ptr<ProjectPlanningSceneSnapshot> ProjectPlanningSceneBuilder::build(
        const simulation_project::ProjectDocument& document,
        const std::filesystem::path& projectBasePath,
        const ProjectPlanningRequest& request,
        std::string* errorMessage)
    {
        if (request.robotId.empty())
        {
            setError(errorMessage, "Robot id is required.");
            return nullptr;
        }
        if (request.collisionDetectorIds.empty())
        {
            setError(errorMessage, "At least one collision detector id is required.");
            return nullptr;
        }
        if (!document.collision.query.enabled)
        {
            setError(errorMessage, "Project collision queries are disabled.");
            return nullptr;
        }

        auto impl = std::make_unique<ProjectPlanningSceneSnapshot::Impl>();
        simulation_runtime::Result runtimeResult = impl->simulation.loadProject(document, projectBasePath);
        if (!runtimeResult.success)
        {
            setError(errorMessage, runtimeResult.message);
            return nullptr;
        }
        runtimeResult = impl->simulation.update(0.0);
        if (!runtimeResult.success)
        {
            setError(errorMessage, runtimeResult.message);
            return nullptr;
        }

        const simulation_runtime::RuntimeRobot* runtimeRobot = impl->simulation.robot(request.robotId);
        if (runtimeRobot == nullptr || !runtimeRobot->instance)
        {
            setError(errorMessage, "Planning robot is not available in the project runtime: " + request.robotId);
            return nullptr;
        }
        if (!runtimeRobot->collisionEnabled)
        {
            setError(errorMessage, "Planning robot collision is disabled: " + request.robotId);
            return nullptr;
        }

        impl->robotId = request.robotId;
        impl->jointNames = request.jointNames.empty()
            ? movableJointNames(runtimeRobot->model)
            : request.jointNames;
        if (impl->jointNames.empty())
        {
            setError(errorMessage, "Planning robot has no movable joints.");
            return nullptr;
        }

        impl->jointBounds.reserve(impl->jointNames.size());
        for (const std::string& jointName : impl->jointNames)
        {
            const robot::RobotJoint* joint = findJoint(runtimeRobot->model, jointName);
            if (joint == nullptr || joint->dofIndex < 0)
            {
                setError(errorMessage, "Planning joint is missing or fixed: " + jointName);
                return nullptr;
            }
            const bool supportedType =
                joint->type == robot::JointType::Revolute ||
                joint->type == robot::JointType::Prismatic;
            if (joint->isLoop || !supportedType || (!joint->continuous && !hasFiniteOrderedPositionLimits(*joint)))
            {
                setError(errorMessage, "The first planner requires a bounded non-loop revolute or prismatic joint: " + jointName);
                return nullptr;
            }
            if (joint->continuous && joint->type != robot::JointType::Revolute)
            {
                setError(errorMessage, "Continuous planning joints must be revolute: " + jointName);
                return nullptr;
            }

            impl->jointBounds.push_back(planningBoundForJoint(*joint));
        }

        for (const std::string& detectorId : request.collisionDetectorIds)
        {
            const simulation_project::CollisionDetectorDesc* detector = findDetector(document, detectorId);
            if (detector == nullptr)
            {
                setError(errorMessage, "Collision detector not found: " + detectorId);
                return nullptr;
            }
            if (!detector->enabled)
            {
                setError(errorMessage, "Collision detector is disabled: " + detectorId);
                return nullptr;
            }
            impl->detectorIds.push_back(detectorId);
        }

        simulation_project::ProjectDocument selectedDetectorDocument = document;
        auto& selectedDetectors = selectedDetectorDocument.collision.detectors;
        selectedDetectors.erase(
            std::remove_if(
                selectedDetectors.begin(),
                selectedDetectors.end(),
                [&](const simulation_project::CollisionDetectorDesc& detector) {
                    return std::find(
                        request.collisionDetectorIds.begin(),
                        request.collisionDetectorIds.end(),
                        detector.id) == request.collisionDetectorIds.end();
                }),
            selectedDetectors.end());
        const simulation_runtime::CollisionDetectorBuildPlan buildPlan =
            simulation_runtime::ProjectCollisionDetectorBuilder::collectBuildPlan(selectedDetectorDocument);
        if (!buildPlan.requiresRobot(request.robotId))
        {
            setError(errorMessage, "Selected project collision detectors do not include the planning robot: " + request.robotId);
            return nullptr;
        }

        runtimeResult = impl->collision.build(impl->simulation);
        if (!runtimeResult.success)
        {
            setError(errorMessage, runtimeResult.message);
            return nullptr;
        }
        for (const std::string& detectorId : impl->detectorIds)
        {
            if (!impl->collision.setActiveDetector(detectorId))
            {
                setError(errorMessage, "Collision detector was not built: " + detectorId);
                return nullptr;
            }
        }

        setError(errorMessage, std::string());
        return std::unique_ptr<ProjectPlanningSceneSnapshot>(
            new ProjectPlanningSceneSnapshot(std::move(impl)));
    }

    MotionPlanningResult ProjectMotionPlanningService::plan(
        const simulation_project::ProjectDocument& document,
        const std::filesystem::path& projectBasePath,
        const ProjectPlanningRequest& request,
        CancellationToken* cancellation) const
    {
        std::string errorMessage;
        std::unique_ptr<ProjectPlanningSceneSnapshot> scene = ProjectPlanningSceneBuilder::build(
            document,
            projectBasePath,
            request,
            &errorMessage);
        if (!scene)
            return invalidResult("planning_scene_build_failed", errorMessage);
        return plan(*scene, request, cancellation);
    }

    MotionPlanningResult ProjectMotionPlanningService::plan(
        ProjectPlanningSceneSnapshot& scene,
        const ProjectPlanningRequest& request,
        CancellationToken* cancellation) const
    {
        if (request.robotId != scene.robotId())
            return invalidResult("robot_mismatch", "Request robot does not match the planning scene.");
        if (!request.jointNames.empty() && request.jointNames != scene.jointNames())
            return invalidResult("joint_order_mismatch", "Request joint order does not match the planning scene.");

        JointPlanningProblem problem;
        problem.robotId = scene.robotId();
        problem.jointNames = scene.jointNames();
        problem.jointBounds = scene.jointBounds();
        problem.start = normalizedJointValuesForBounds(request.start, scene.jointBounds());
        problem.goal = normalizedJointValuesForBounds(request.goal, scene.jointBounds());
        problem.planner = request.planner;
        problem.validation = request.validation;
        problem.postProcess = request.postProcess;

        const OmplMotionPlanner planner;
        return planner.plan(problem, scene, cancellation);
    }
}
