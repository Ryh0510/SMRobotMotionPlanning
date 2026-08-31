#include <MotionPlanningOmpl/OmplMotionPlanner.h>

#include <ompl/base/MotionValidator.h>
#include <ompl/base/PlannerData.h>
#include <ompl/base/PlannerTerminationCondition.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/util/RandomNumbers.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace motion_planning
{
    namespace
    {
        namespace ob = ompl::base;
        namespace og = ompl::geometric;

        MotionPlanningResult resultWithDiagnostic(
            MotionPlanningStatus status,
            std::string code,
            std::string message)
        {
            MotionPlanningResult result;
            result.status = status;
            result.diagnostics.push_back({ std::move(code), std::move(message) });
            return result;
        }

        bool isFiniteVector(const std::vector<double>& values)
        {
            return std::all_of(values.begin(), values.end(), [](double value) {
                return std::isfinite(value);
            });
        }

        MotionPlanningResult validateProblem(const JointPlanningProblem& problem)
        {
            if (problem.robotId.empty())
                return resultWithDiagnostic(MotionPlanningStatus::InvalidRequest, "robot_required", "Robot id is required.");
            if (problem.start.empty())
                return resultWithDiagnostic(MotionPlanningStatus::InvalidRequest, "start_required", "Start state is required.");
            if (problem.start.size() != problem.goal.size())
                return resultWithDiagnostic(MotionPlanningStatus::InvalidRequest, "state_size_mismatch", "Start and goal state sizes differ.");
            if (problem.jointNames.size() != problem.start.size())
                return resultWithDiagnostic(MotionPlanningStatus::InvalidRequest, "joint_size_mismatch", "Joint name and state sizes differ.");
            if (problem.jointBounds.size() != problem.start.size())
                return resultWithDiagnostic(MotionPlanningStatus::InvalidRequest, "bound_size_mismatch", "Joint bound and state sizes differ.");
            if (!isFiniteVector(problem.start) || !isFiniteVector(problem.goal))
                return resultWithDiagnostic(MotionPlanningStatus::InvalidRequest, "non_finite_state", "Start and goal states must be finite.");
            if (problem.planner.plannerId != "RRTConnect")
                return resultWithDiagnostic(MotionPlanningStatus::InvalidRequest, "unsupported_planner", "Only RRTConnect is supported in the first OMPL backend.");
            if (!(problem.planner.timeoutSeconds > 0.0))
                return resultWithDiagnostic(MotionPlanningStatus::InvalidRequest, "invalid_timeout", "Planning timeout must be positive.");
            if (!(problem.validation.maxJointStep > 0.0))
                return resultWithDiagnostic(MotionPlanningStatus::InvalidRequest, "invalid_motion_step", "Maximum joint validation step must be positive.");
            if (!(problem.postProcess.duration > 0.0))
                return resultWithDiagnostic(MotionPlanningStatus::InvalidRequest, "invalid_duration", "Trajectory duration must be positive.");

            for (std::size_t index = 0; index < problem.jointBounds.size(); ++index)
            {
                const JointBound& bound = problem.jointBounds[index];
                if (!std::isfinite(bound.lower) || !std::isfinite(bound.upper) ||
                    !(bound.lower < bound.upper))
                {
                    return resultWithDiagnostic(MotionPlanningStatus::InvalidRequest, "unsupported_joint_bound", "The first OMPL backend requires finite bounded joints.");
                }
                if (problem.start[index] < bound.lower || problem.start[index] > bound.upper ||
                    problem.goal[index] < bound.lower || problem.goal[index] > bound.upper)
                {
                    return resultWithDiagnostic(MotionPlanningStatus::InvalidRequest, "state_out_of_bounds", "Start or goal state is outside a joint bound.");
                }
            }

            MotionPlanningResult valid;
            valid.status = MotionPlanningStatus::Success;
            return valid;
        }

        std::vector<double> stateValues(const ob::State* state, std::size_t dimension)
        {
            const auto* vectorState = state->as<ob::RealVectorStateSpace::StateType>();
            return std::vector<double>(vectorState->values, vectorState->values + dimension);
        }

        class SceneMotionValidator final : public ob::MotionValidator
        {
        public:
            SceneMotionValidator(
                const ob::SpaceInformationPtr& spaceInformation,
                IPlanningScene& scene,
                MotionValidationOptions options,
                std::size_t dimension)
                : ob::MotionValidator(spaceInformation)
                , m_scene(scene)
                , m_options(options)
                , m_dimension(dimension)
            {
            }

            bool checkMotion(const ob::State* state1, const ob::State* state2) const override
            {
                return m_scene.validateMotion(
                    stateValues(state1, m_dimension),
                    stateValues(state2, m_dimension),
                    m_options).valid;
            }

            bool checkMotion(
                const ob::State* state1,
                const ob::State* state2,
                std::pair<ob::State*, double>& lastValid) const override
            {
                if (checkMotion(state1, state2))
                {
                    lastValid.second = 1.0;
                    if (lastValid.first != nullptr)
                        si_->copyState(lastValid.first, state2);
                    return true;
                }

                lastValid.second = 0.0;
                if (lastValid.first != nullptr)
                    si_->copyState(lastValid.first, state1);
                return false;
            }

        private:
            IPlanningScene& m_scene;
            MotionValidationOptions m_options;
            std::size_t m_dimension = 0;
        };

        class SeededRealVectorStateSampler final : public ob::RealVectorStateSampler
        {
        public:
            SeededRealVectorStateSampler(
                const ob::StateSpace* stateSpace,
                std::uint32_t seed)
                : ob::RealVectorStateSampler(stateSpace)
            {
                rng_.setLocalSeed(seed);
            }
        };

        class SeededRRTConnect final : public og::RRTConnect
        {
        public:
            SeededRRTConnect(
                const ob::SpaceInformationPtr& spaceInformation,
                std::uint32_t seed)
                : og::RRTConnect(spaceInformation)
            {
                rng_.setLocalSeed(seed);
            }
        };

        double stateDistance(const std::vector<double>& first, const std::vector<double>& second)
        {
            double squaredDistance = 0.0;
            for (std::size_t index = 0; index < first.size(); ++index)
            {
                const double delta = second[index] - first[index];
                squaredDistance += delta * delta;
            }
            return std::sqrt(squaredDistance);
        }

        // Keep short-path simplification under adapter control. OMPL 2.0.1
        // ropeShortcutPath can index past a short state array in MSVC Debug builds.
        void simplifyPathDeterministically(
            og::PathGeometric& path,
            IPlanningScene& scene,
            const MotionValidationOptions& options,
            std::size_t dimension)
        {
            const std::size_t stateCount = path.getStateCount();
            if (stateCount <= 2)
                return;

            std::vector<std::vector<double>> states;
            states.reserve(stateCount);
            for (std::size_t index = 0; index < stateCount; ++index)
                states.push_back(stateValues(path.getState(index), dimension));

            std::vector<std::size_t> keptIndices;
            keptIndices.reserve(stateCount);
            keptIndices.push_back(0);

            std::size_t anchor = 0;
            while (anchor + 1 < stateCount)
            {
                std::size_t next = anchor + 1;
                for (std::size_t candidate = stateCount - 1; candidate > anchor + 1; --candidate)
                {
                    if (scene.validateMotion(states[anchor], states[candidate], options).valid)
                    {
                        next = candidate;
                        break;
                    }
                }
                keptIndices.push_back(next);
                anchor = next;
            }

            og::PathGeometric simplified(path.getSpaceInformation());
            for (const std::size_t index : keptIndices)
                simplified.append(path.getState(index));
            path = simplified;
        }

        robottrajectory::JointTrajectory makeTrajectory(
            const JointPlanningProblem& problem,
            const og::PathGeometric& path)
        {
            std::vector<std::vector<double>> states;
            states.reserve(path.getStateCount());
            for (std::size_t index = 0; index < path.getStateCount(); ++index)
                states.push_back(stateValues(path.getState(index), problem.start.size()));

            std::vector<double> cumulative(states.size(), 0.0);
            for (std::size_t index = 1; index < states.size(); ++index)
                cumulative[index] = cumulative[index - 1] + stateDistance(states[index - 1], states[index]);

            const double totalDistance = cumulative.empty() ? 0.0 : cumulative.back();
            robottrajectory::JointTrajectory trajectory;
            trajectory.name = problem.robotId + "_ompl_plan";
            trajectory.interpolation = robottrajectory::TrajectoryInterpolation::Linear;
            trajectory.points.reserve(states.size());
            for (std::size_t index = 0; index < states.size(); ++index)
            {
                robottrajectory::TimedJointPoint point;
                const double ratio = totalDistance > 0.0
                    ? cumulative[index] / totalDistance
                    : (states.size() > 1 ? static_cast<double>(index) / static_cast<double>(states.size() - 1) : 0.0);
                point.time = problem.postProcess.duration * ratio;
                point.q = std::move(states[index]);
                trajectory.points.push_back(std::move(point));
            }
            return trajectory;
        }
    }

    MotionPlanningResult OmplMotionPlanner::plan(
        const JointPlanningProblem& problem,
        IPlanningScene& scene,
        CancellationToken* cancellation) const
    {
        const MotionPlanningResult validation = validateProblem(problem);
        if (!validation.succeeded())
            return validation;
        if (cancellation != nullptr && cancellation->isCancellationRequested())
            return resultWithDiagnostic(MotionPlanningStatus::Cancelled, "cancelled", "Planning was cancelled before it started.");

        const StateValidationResult startValidation = scene.validateState(problem.start);
        if (!startValidation.valid)
            return resultWithDiagnostic(MotionPlanningStatus::InvalidRequest, "invalid_start", startValidation.message);
        const StateValidationResult goalValidation = scene.validateState(problem.goal);
        if (!goalValidation.valid)
            return resultWithDiagnostic(MotionPlanningStatus::InvalidRequest, "invalid_goal", goalValidation.message);

        const auto startTime = std::chrono::steady_clock::now();
        auto stateSpace = std::make_shared<ob::RealVectorStateSpace>(
            static_cast<unsigned int>(problem.start.size()));
        ob::RealVectorBounds bounds(problem.start.size());
        for (std::size_t index = 0; index < problem.jointBounds.size(); ++index)
        {
            bounds.setLow(index, problem.jointBounds[index].lower);
            bounds.setHigh(index, problem.jointBounds[index].upper);
        }
        stateSpace->setBounds(bounds);
        stateSpace->setStateSamplerAllocator([seed = problem.planner.randomSeed](const ob::StateSpace* space) {
            return std::make_shared<SeededRealVectorStateSampler>(space, seed);
        });

        og::SimpleSetup setup(stateSpace);
        const ob::SpaceInformationPtr spaceInformation = setup.getSpaceInformation();
        setup.setStateValidityChecker([&scene, dimension = problem.start.size()](const ob::State* state) {
            return scene.validateState(stateValues(state, dimension)).valid;
        });
        spaceInformation->setMotionValidator(std::make_shared<SceneMotionValidator>(
            spaceInformation,
            scene,
            problem.validation,
            problem.start.size()));

        auto planner = std::make_shared<SeededRRTConnect>(
            spaceInformation,
            problem.planner.randomSeed);
        if (problem.planner.range > 0.0)
            planner->setRange(problem.planner.range);
        setup.setPlanner(planner);

        ob::ScopedState<ob::RealVectorStateSpace> start(stateSpace);
        ob::ScopedState<ob::RealVectorStateSpace> goal(stateSpace);
        for (std::size_t index = 0; index < problem.start.size(); ++index)
        {
            start[index] = problem.start[index];
            goal[index] = problem.goal[index];
        }
        setup.setStartAndGoalStates(start, goal);

        const ob::PlannerTerminationCondition timed = ob::timedPlannerTerminationCondition(problem.planner.timeoutSeconds);
        const ob::PlannerTerminationCondition cancelled([cancellation]() {
            return cancellation != nullptr && cancellation->isCancellationRequested();
        });
        const ob::PlannerStatus status = setup.solve(ob::plannerOrTerminationCondition(timed, cancelled));

        const double elapsed = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - startTime).count();
        if (!status)
        {
            MotionPlanningResult result;
            if (cancellation != nullptr && cancellation->isCancellationRequested())
                result = resultWithDiagnostic(MotionPlanningStatus::Cancelled, "cancelled", "Planning was cancelled.");
            else if (elapsed >= problem.planner.timeoutSeconds)
                result = resultWithDiagnostic(MotionPlanningStatus::Timeout, "timeout", "Planning timed out without a solution.");
            else
                result = resultWithDiagnostic(MotionPlanningStatus::NoSolution, "no_solution", "OMPL did not find a solution.");
            result.planningTimeSeconds = elapsed;
            return result;
        }

        og::PathGeometric& path = setup.getSolutionPath();
        if (problem.planner.simplifyPath && path.getStateCount() > 2)
            simplifyPathDeterministically(
                path,
                scene,
                problem.validation,
                problem.start.size());
        if (path.getStateCount() < problem.postProcess.minimumWaypointCount)
            path.interpolate(problem.postProcess.minimumWaypointCount);

        for (std::size_t index = 1; index < path.getStateCount(); ++index)
        {
            const StateValidationResult motionValidation = scene.validateMotion(
                stateValues(path.getState(index - 1), problem.start.size()),
                stateValues(path.getState(index), problem.start.size()),
                problem.validation);
            if (!motionValidation.valid)
            {
                MotionPlanningResult result = resultWithDiagnostic(
                    MotionPlanningStatus::Failed,
                    "solution_revalidation_failed",
                    motionValidation.message.empty() ? "The OMPL solution failed final motion validation." : motionValidation.message);
                result.planningTimeSeconds = elapsed;
                return result;
            }
        }

        MotionPlanningResult result;
        result.status = MotionPlanningStatus::Success;
        result.trajectory = makeTrajectory(problem, path);
        result.planningTimeSeconds = elapsed;
        ob::PlannerData plannerData(spaceInformation);
        setup.getPlannerData(plannerData);
        result.sampledStateCount = plannerData.numVertices();
        return result;
    }
}
