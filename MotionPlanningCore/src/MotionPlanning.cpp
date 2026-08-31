#include <MotionPlanningCore/MotionPlanning.h>

#include <SimulationProject/ProjectDocument.h>

#include <nlohmann/json.hpp>

#include <algorithm>
#include <exception>
#include <utility>

namespace motion_planning
{
    namespace
    {
        using Json = nlohmann::json;
        constexpr const char* kMotionPlanningExtensionKey = "motion_planning.trajectories";

        MotionPlanningResult invalidResult(const std::string& message)
        {
            MotionPlanningResult result;
            result.status = MotionPlanningStatus::InvalidRequest;
            result.diagnostics.push_back({ "invalid_request", message });
            return result;
        }

        Json writeTrajectory(const robottrajectory::JointTrajectory& trajectory)
        {
            Json points = Json::array();
            for(const robottrajectory::TimedJointPoint& point : trajectory.points) {
                points.push_back(Json{
                    { "time", point.time },
                    { "q", point.q },
                    { "qd", point.qd },
                    { "qdd", point.qdd }
                });
            }
            return Json{
                { "name", trajectory.name },
                { "interpolation", trajectory.interpolation == robottrajectory::TrajectoryInterpolation::Step
                    ? "step" : "linear" },
                { "points", points }
            };
        }

        Json writePose(const Eigen::Isometry3d& pose)
        {
            Json rows = Json::array();
            const Eigen::Matrix4d matrix = pose.matrix();
            for(int row = 0; row < 4; ++row) {
                Json values = Json::array();
                for(int column = 0; column < 4; ++column) {
                    values.push_back(matrix(row, column));
                }
                rows.push_back(std::move(values));
            }
            return rows;
        }

        Json writeCartesianTrajectory(const robottrajectory::CartesianTrajectory& trajectory)
        {
            Json points = Json::array();
            for(const robottrajectory::TimedCartesianPoint& point : trajectory.points) {
                points.push_back(Json{
                    { "time", point.time },
                    { "tcpPose", writePose(point.tcpPose) },
                    { "linearSpeed", point.linearSpeed },
                    { "angularSpeed", point.angularSpeed }
                });
            }
            return Json{
                { "name", trajectory.name },
                { "interpolation", trajectory.interpolation == robottrajectory::TrajectoryInterpolation::Step
                    ? "step" : "linear" },
                { "points", points }
            };
        }
         
        robottrajectory::JointTrajectory readTrajectory(const Json& json)
        {
            robottrajectory::JointTrajectory trajectory;
            trajectory.name = json.value("name", std::string());
            trajectory.interpolation = json.value("interpolation", std::string("linear")) == "step"
                ? robottrajectory::TrajectoryInterpolation::Step
                : robottrajectory::TrajectoryInterpolation::Linear;
            if(json.contains("points") && json.at("points").is_array()) {
                for(const Json& pointJson : json.at("points")) {
                    robottrajectory::TimedJointPoint point;
                    point.time = pointJson.value("time", 0.0);
                    point.q = pointJson.value("q", std::vector<double>());
                    point.qd = pointJson.value("qd", std::vector<double>());
                    point.qdd = pointJson.value("qdd", std::vector<double>());
                    trajectory.points.push_back(std::move(point));
                }
            }
            return trajectory;
        }

        Eigen::Isometry3d readPose(const Json& json)
        {
            Eigen::Matrix4d matrix = Eigen::Matrix4d::Identity();
            if(json.is_array() && json.size() >= 4 && json.front().is_array()) {
                for(std::size_t row = 0; row < 4 && row < json.size(); ++row) {
                    const Json& rowJson = json.at(row);
                    for(std::size_t column = 0; column < 4 && column < rowJson.size(); ++column) {
                        matrix(static_cast<int>(row), static_cast<int>(column)) =
                            rowJson.at(column).get<double>();
                    }
                }
            } else if(json.is_array() && json.size() >= 16) {
                for(std::size_t index = 0; index < 16; ++index) {
                    matrix(static_cast<int>(index / 4), static_cast<int>(index % 4)) =
                        json.at(index).get<double>();
                }
            }

            Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
            pose.matrix() = matrix;
            return pose;
        }

        robottrajectory::CartesianTrajectory readCartesianTrajectory(const Json& json)
        {
            robottrajectory::CartesianTrajectory trajectory;
            trajectory.name = json.value("name", std::string());
            trajectory.interpolation = json.value("interpolation", std::string("linear")) == "step"
                ? robottrajectory::TrajectoryInterpolation::Step
                : robottrajectory::TrajectoryInterpolation::Linear;
            if(json.contains("points") && json.at("points").is_array()) {
                for(const Json& pointJson : json.at("points")) {
                    robottrajectory::TimedCartesianPoint point;
                    point.time = pointJson.value("time", 0.0);
                    if(pointJson.contains("tcpPose")) {
                        point.tcpPose = readPose(pointJson.at("tcpPose"));
                    }
                    point.linearSpeed = pointJson.value("linearSpeed", 0.0);
                    point.angularSpeed = pointJson.value("angularSpeed", 0.0);
                    trajectory.points.push_back(std::move(point));
                }
            }
            return trajectory;
        }

        simulation_project::ProjectExtensionDesc* findExtension(
            simulation_project::ProjectDocument& document)
        {
            const auto it = std::find_if(
                document.extensions.begin(),
                document.extensions.end(),
                [](const simulation_project::ProjectExtensionDesc& extension) {
                    return extension.key == kMotionPlanningExtensionKey;
                });
            return it == document.extensions.end() ? nullptr : &(*it);
        }

        const simulation_project::ProjectExtensionDesc* findExtension(
            const simulation_project::ProjectDocument& document)
        {
            const auto it = std::find_if(
                document.extensions.begin(),
                document.extensions.end(),
                [](const simulation_project::ProjectExtensionDesc& extension) {
                    return extension.key == kMotionPlanningExtensionKey;
                });
            return it == document.extensions.end() ? nullptr : &(*it);
        }
    }

    bool MotionPlanningResult::succeeded() const
    {
        return status == MotionPlanningStatus::Success;
    }

    StateValidationResult StateValidationResult::success()
    {
        StateValidationResult result;
        result.valid = true;
        return result;
    }

    StateValidationResult StateValidationResult::failure(
        std::string code,
        std::string message)
    {
        StateValidationResult result;
        result.valid = false;
        result.diagnosticCode = std::move(code);
        result.message = std::move(message);
        return result;
    }

    void CancellationToken::cancel()
    {
        m_cancelled.store(true, std::memory_order_release);
    }

    bool CancellationToken::isCancellationRequested() const
    {
        return m_cancelled.load(std::memory_order_acquire);
    }

    MotionPlanningResult LinearJointMotionPlanner::plan(
        const MotionPlanningRequest& request) const
    {
        if(request.robotId.empty()) {
            return invalidResult("Robot id is required.");
        }
        if(request.startJoints.empty()) {
            return invalidResult("At least one start joint value is required.");
        }
        if(request.startJoints.size() != request.goalJoints.size()) {
            return invalidResult("Start and goal joint vectors must have equal size.");
        }
        if(!request.jointNames.empty() && request.jointNames.size() != request.startJoints.size()) {
            return invalidResult("Joint names must match the joint vector size.");
        }
        if(request.constraint.duration <= 0.0) {
            return invalidResult("Planning duration must be positive.");
        }
        if(request.constraint.sampleCount < 2) {
            return invalidResult("Planning sample count must be at least two.");
        }

        MotionPlanningResult result;
        result.status = MotionPlanningStatus::Success;
        result.trajectory.name = request.robotId + "_linear_plan";
        result.trajectory.interpolation = robottrajectory::TrajectoryInterpolation::Linear;
        result.trajectory.points.reserve(request.constraint.sampleCount);

        const std::size_t lastIndex = request.constraint.sampleCount - 1;
        for(std::size_t sampleIndex = 0; sampleIndex <= lastIndex; ++sampleIndex) {
            const double ratio = static_cast<double>(sampleIndex) /
                static_cast<double>(lastIndex);

            robottrajectory::TimedJointPoint point;
            point.time = request.constraint.duration * ratio;
            point.q.resize(request.startJoints.size());
            for(std::size_t jointIndex = 0; jointIndex < point.q.size(); ++jointIndex) {
                const double start = request.startJoints[jointIndex];
                const double goal = request.goalJoints[jointIndex];
                point.q[jointIndex] = start + (goal - start) * ratio;
            }
            result.trajectory.points.push_back(std::move(point));
        }

        return result;
    }

    const char* MotionPlanningProjectStore::extensionKey()
    {
        return kMotionPlanningExtensionKey;
    }

    std::vector<StoredMotionPlan> MotionPlanningProjectStore::plans(
        const simulation_project::ProjectDocument& document)
    {
        const simulation_project::ProjectExtensionDesc* extension = findExtension(document);
        if(extension == nullptr || extension->serializedPayload.empty()) {
            return {};
        }

        std::vector<StoredMotionPlan> result;
        try {
            const Json payload = Json::parse(extension->serializedPayload);
            if(!payload.contains("plans") || !payload.at("plans").is_array()) {
                return result;
            }
            for(const Json& planJson : payload.at("plans")) {
                StoredMotionPlan plan;
                plan.id = planJson.value("id", std::string());
                plan.name = planJson.value("name", std::string());
                plan.robotId = planJson.value("robotId", std::string());
                plan.jointNames = planJson.value("jointNames", std::vector<std::string>());
                if(planJson.contains("trajectory")) {
                    plan.trajectory = readTrajectory(planJson.at("trajectory"));
                }
                if(planJson.contains("cartesianControlPoints")) {
                    plan.cartesianControlPoints =
                        readCartesianTrajectory(planJson.at("cartesianControlPoints"));
                }
                if(!plan.id.empty()) {
                    result.push_back(std::move(plan));
                }
            }
        } catch(const std::exception&) {
            return {};
        }
        return result;
    }

    bool MotionPlanningProjectStore::upsertPlan(
        simulation_project::ProjectDocument& document,
        const StoredMotionPlan& plan,
        std::string* errorMessage)
    {
        if(plan.id.empty() || plan.robotId.empty() ||
            (plan.trajectory.empty() && plan.cartesianControlPoints.empty())) {
            if(errorMessage != nullptr) {
                *errorMessage = "Motion plan id, robot id, and at least one trajectory/control point sequence are required.";
            }
            return false;
        }

        std::vector<StoredMotionPlan> storedPlans = plans(document);
        const auto existing = std::find_if(
            storedPlans.begin(),
            storedPlans.end(),
            [&](const StoredMotionPlan& candidate) {
                return candidate.id == plan.id;
            });
        if(existing == storedPlans.end()) {
            storedPlans.push_back(plan);
        } else {
            *existing = plan;
        }

        Json planArray = Json::array();
        for(const StoredMotionPlan& storedPlan : storedPlans) {
            planArray.push_back(Json{
                { "id", storedPlan.id },
                { "name", storedPlan.name },
                { "robotId", storedPlan.robotId },
                { "jointNames", storedPlan.jointNames },
                { "trajectory", writeTrajectory(storedPlan.trajectory) },
                { "cartesianControlPoints", writeCartesianTrajectory(storedPlan.cartesianControlPoints) }
            });
        }
        const std::string payload = Json{
            { "schemaVersion", 1 },
            { "plans", planArray }
        }.dump();

        simulation_project::ProjectExtensionDesc* extension = findExtension(document);
        if(extension == nullptr) {
            simulation_project::ProjectExtensionDesc newExtension;
            newExtension.key = kMotionPlanningExtensionKey;
            newExtension.version = 1;
            newExtension.serializedPayload = payload;
            document.extensions.push_back(std::move(newExtension));
        } else {
            extension->version = 1;
            extension->serializedPayload = payload;
        }
        return true;
    }
}
