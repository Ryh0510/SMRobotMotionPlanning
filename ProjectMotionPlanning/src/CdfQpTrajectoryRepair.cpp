#include <ProjectMotionPlanning/CdfQpTrajectoryRepair.h>

#include <ProjectMotionPlanning/ProjectMotionPlanning.h>
#include <MotionPlanningOmpl/OmplMotionPlanner.h>
#include <SimulationProject/ProjectDocument.h>
#include <osqp.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <memory>
#include <random>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#ifdef max
#undef max
#endif
#ifdef min
#undef min
#endif

namespace motion_planning
{
    namespace
    {
        constexpr double kPi = 3.14159265358979323846;
        constexpr double kTwoPi = 2.0 * kPi;
        constexpr double kTiny = 1.0e-12;

        void addDiagnostic(
            std::vector<MotionPlanningDiagnostic>* diagnostics,
            const std::string& code,
            const std::string& message)
        {
            if(diagnostics != nullptr) {
                diagnostics->push_back({ code, message });
            }
        }

        bool containsCaseSensitive(
            const std::string& text,
            const std::string& token)
        {
            return text.find(token) != std::string::npos;
        }

        const simulation_project::RobotDesc* findRobot(
            const simulation_project::ProjectDocument& document,
            const std::string& robotId)
        {
            const auto it = std::find_if(
                document.robots.begin(),
                document.robots.end(),
                [&](const simulation_project::RobotDesc& robot) {
                    return robot.id == robotId;
                });
            return it == document.robots.end() ? nullptr : &(*it);
        }

        const simulation_project::RobotDesc* findBurnnerRobot(
            const simulation_project::ProjectDocument& document,
            const ProjectCdfQpRepairOptions& options,
            const std::string& planningRobotId)
        {
            const auto exactIt = std::find_if(
                document.robots.begin(),
                document.robots.end(),
                [&](const simulation_project::RobotDesc& robot) {
                    return robot.id == options.obstacleId && robot.id != planningRobotId;
                });
            if(exactIt != document.robots.end()) {
                return &(*exactIt);
            }

            const auto sourceIt = std::find_if(
                document.robots.begin(),
                document.robots.end(),
                [&](const simulation_project::RobotDesc& robot) {
                    return robot.id != planningRobotId &&
                        (containsCaseSensitive(robot.id, "burnner") ||
                         containsCaseSensitive(robot.name, "burnner") ||
                         containsCaseSensitive(robot.sourcePath, "burnner"));
                });
            return sourceIt == document.robots.end() ? nullptr : &(*sourceIt);
        }

        const simulation_project::SceneObjectDesc* findBurnnerObject(
            const simulation_project::ProjectDocument& document,
            const ProjectCdfQpRepairOptions& options)
        {
            const auto exactIt = std::find_if(
                document.objects.begin(),
                document.objects.end(),
                [&](const simulation_project::SceneObjectDesc& object) {
                    return object.id == options.obstacleId;
                });
            if(exactIt != document.objects.end()) {
                return &(*exactIt);
            }

            const auto sourceIt = std::find_if(
                document.objects.begin(),
                document.objects.end(),
                [](const simulation_project::SceneObjectDesc& object) {
                    return containsCaseSensitive(object.id, "burnner") ||
                        containsCaseSensitive(object.name, "burnner") ||
                        containsCaseSensitive(object.sourcePath, "burnner");
                });
            return sourceIt == document.objects.end() ? nullptr : &(*sourceIt);
        }

        bool hasDetector(
            const simulation_project::ProjectDocument& document,
            const std::string& detectorId)
        {
            return std::any_of(
                document.collision.detectors.begin(),
                document.collision.detectors.end(),
                [&](const simulation_project::CollisionDetectorDesc& detector) {
                    return detector.id == detectorId;
                });
        }

        simulation_project::CollisionDetectorDesc makeRobotRobotDetector(
            const std::string& detectorId,
            const std::string& planningRobotId,
            const std::string& obstacleRobotId,
            const ProjectCdfQpRepairOptions& options)
        {
            simulation_project::CollisionDetectorDesc detector;
            detector.id = detectorId;
            detector.name = "CDF ABB4600-burnner detector";
            detector.type = "RobotRobot";
            detector.enabled = true;
            detector.geometryRole = "Exact";
            detector.contacts = true;
            detector.nearestPoints = true;
            detector.distance = true;
            detector.maxContacts = 64;
            detector.distanceThreshold = options.distanceThreshold;

            simulation_project::CollisionDetectorTargetDesc robotTarget;
            robotTarget.robotId = planningRobotId;
            detector.targets.push_back(std::move(robotTarget));

            simulation_project::CollisionDetectorTargetDesc obstacleTarget;
            obstacleTarget.robotId = obstacleRobotId;
            detector.targets.push_back(std::move(obstacleTarget));

            simulation_project::CollisionPairGeneratorDesc generator;
            generator.type = "RobotRobot";
            generator.robotA = planningRobotId;
            generator.robotB = obstacleRobotId;
            detector.pairGenerators.push_back(std::move(generator));
            return detector;
        }

        simulation_project::CollisionDetectorDesc makeRobotObjectDetector(
            const std::string& detectorId,
            const std::string& planningRobotId,
            const std::string& obstacleObjectId,
            const ProjectCdfQpRepairOptions& options)
        {
            simulation_project::CollisionDetectorDesc detector;
            detector.id = detectorId;
            detector.name = "CDF ABB4600-burnner detector";
            detector.type = "RobotObject";
            detector.enabled = true;
            detector.geometryRole = "Exact";
            detector.contacts = true;
            detector.nearestPoints = true;
            detector.distance = true;
            detector.maxContacts = 64;
            detector.distanceThreshold = options.distanceThreshold;

            simulation_project::CollisionDetectorTargetDesc robotTarget;
            robotTarget.robotId = planningRobotId;
            detector.targets.push_back(std::move(robotTarget));

            simulation_project::CollisionDetectorTargetDesc obstacleTarget;
            obstacleTarget.objectId = obstacleObjectId;
            detector.targets.push_back(std::move(obstacleTarget));

            simulation_project::CollisionPairGeneratorDesc generator;
            generator.type = "RobotObject";
            generator.robotId = planningRobotId;
            generator.objectId = obstacleObjectId;
            detector.pairGenerators.push_back(std::move(generator));
            return detector;
        }

        double wrapContinuousAngle(double value)
        {
            if(!std::isfinite(value)) {
                return value;
            }

            double wrapped = std::remainder(value, kTwoPi);
            if(wrapped <= -kPi) {
                wrapped += kTwoPi;
            } else if(wrapped > kPi) {
                wrapped -= kTwoPi;
            }
            return wrapped;
        }

        std::vector<double> clampToBounds(
            const std::vector<double>& q,
            const std::vector<JointBound>& bounds,
            int* clampedCount)
        {
            std::vector<double> clamped = q;
            const std::size_t count = std::min(clamped.size(), bounds.size());
            for(std::size_t index = 0; index < count; ++index) {
                const double before = clamped[index];
                clamped[index] = bounds[index].continuous
                    ? wrapContinuousAngle(clamped[index])
                    : std::max(bounds[index].lower, std::min(bounds[index].upper, clamped[index]));
                if(before != clamped[index] && clampedCount != nullptr) {
                    ++(*clampedCount);
                }
            }
            return clamped;
        }

        double vectorNorm(const std::vector<double>& values)
        {
            double sum = 0.0;
            for(const double value : values) {
                sum += value * value;
            }
            return std::sqrt(sum);
        }

        double stateDistance(
            const std::vector<double>& from,
            const std::vector<double>& to,
            const std::vector<JointBound>& bounds)
        {
            double sum = 0.0;
            const std::size_t count = std::min(from.size(), to.size());
            for(std::size_t jointIndex = 0; jointIndex < count; ++jointIndex) {
                double delta = to[jointIndex] - from[jointIndex];
                const bool isContinuous = !bounds.empty() &&
                    (jointIndex < bounds.size() ? bounds[jointIndex] : bounds.back()).continuous;
                if(isContinuous) {
                    delta = std::remainder(delta, kTwoPi);
                }
                sum += delta * delta;
            }
            return std::sqrt(sum);
        }

        double nearestPointDistance(const collision::CollisionResult& collisionResult)
        {
            if(!collisionResult.hasNearestPoints) {
                return std::numeric_limits<double>::max();
            }
            return (collisionResult.nearestPointA - collisionResult.nearestPointB).norm();
        }

        double maxContactPenetrationDepth(const collision::CollisionResult& collisionResult)
        {
            double penetrationDepth = 0.0;
            for(const collision::Contact& contact : collisionResult.contacts) {
                if(std::isfinite(contact.penetrationDepth)) {
                    penetrationDepth = std::max(penetrationDepth, contact.penetrationDepth);
                }
            }
            return penetrationDepth;
        }

        bool extractNearestDirection(
            const collision::CollisionResult& collisionResult,
            collision::Vec3* direction)
        {
            if(direction == nullptr) {
                return false;
            }

            if(collisionResult.hasNearestPoints) {
                const collision::Vec3 delta =
                    collisionResult.nearestPointB - collisionResult.nearestPointA;
                const double norm = delta.norm();
                if(std::isfinite(norm) && norm > kTiny) {
                    *direction = delta / norm;
                    return true;
                }
            }

            for(const collision::Contact& contact : collisionResult.contacts) {
                const double norm = contact.normal.norm();
                if(std::isfinite(norm) && norm > kTiny) {
                    *direction = contact.normal / norm;
                    return true;
                }
            }

            direction->setZero();
            return false;
        }

        double dot(
            const std::vector<double>& lhs,
            const std::vector<double>& rhs)
        {
            double value = 0.0;
            const std::size_t count = std::min(lhs.size(), rhs.size());
            for(std::size_t index = 0; index < count; ++index) {
                value += lhs[index] * rhs[index];
            }
            return value;
        }

        std::vector<robottrajectory::TimedJointPoint> densifyTrajectory(
            const robottrajectory::JointTrajectory& trajectory,
            const std::vector<JointBound>& bounds,
            int minimumIntermediateSamples,
            double maxJointStep)
        {
            std::vector<robottrajectory::TimedJointPoint> densePoints;
            if(trajectory.points.empty()) {
                return densePoints;
            }

            const int minimumSamples = std::max(0, minimumIntermediateSamples);
            const double effectiveJointStep = std::isfinite(maxJointStep) && maxJointStep > 0.0
                ? maxJointStep
                : 0.0;
            densePoints.push_back(trajectory.points.front());

            if(trajectory.points.size() == 1) {
                return densePoints;
            }

            for(std::size_t index = 0; index + 1 < trajectory.points.size(); ++index) {
                const robottrajectory::TimedJointPoint& start = trajectory.points[index];
                const robottrajectory::TimedJointPoint& end = trajectory.points[index + 1];
                int samplesPerSegment = minimumSamples;
                if(effectiveJointStep > 0.0) {
                    double maxDelta = 0.0;
                    const std::size_t count = std::min(start.q.size(), end.q.size());
                    for(std::size_t jointIndex = 0; jointIndex < count; ++jointIndex) {
                        double delta = end.q[jointIndex] - start.q[jointIndex];
                        const bool isContinuous = !bounds.empty() &&
                            (jointIndex < bounds.size() ? bounds[jointIndex] : bounds.back()).continuous;
                        if(isContinuous) {
                            delta = wrapContinuousAngle(delta);
                        }
                        maxDelta = std::max(maxDelta, std::abs(delta));
                    }
                    const int adaptiveSamples = static_cast<int>(
                        std::ceil(std::max(0.0, maxDelta / effectiveJointStep) - 1.0));
                    samplesPerSegment = std::max(samplesPerSegment, adaptiveSamples);
                }

                for(int sampleIndex = 1; sampleIndex <= samplesPerSegment; ++sampleIndex) {
                    const double fraction = static_cast<double>(sampleIndex) /
                        static_cast<double>(samplesPerSegment + 1);
                    robottrajectory::TimedJointPoint sample = start;
                    sample.time = start.time + (end.time - start.time) * fraction;
                    sample.q.resize(start.q.size());
                    for(std::size_t jointIndex = 0; jointIndex < start.q.size(); ++jointIndex) {
                        const double startValue = jointIndex < start.q.size() ? start.q[jointIndex] : 0.0;
                        const double endValue = jointIndex < end.q.size() ? end.q[jointIndex] : startValue;
                        double delta = endValue - startValue;
                        const bool isContinuous = !bounds.empty() &&
                            (jointIndex < bounds.size() ? bounds[jointIndex] : bounds.back()).continuous;
                        if(isContinuous) {
                            delta = wrapContinuousAngle(delta);
                        }
                        const double interpolated = startValue + delta * fraction;
                        sample.q[jointIndex] = isContinuous
                            ? wrapContinuousAngle(interpolated)
                            : interpolated;
                    }
                    sample.qd.clear();
                    sample.qdd.clear();
                    densePoints.push_back(std::move(sample));
                }
                densePoints.push_back(end);
            }

            return densePoints;
        }

        struct InvalidSegmentRun
        {
            std::size_t beginSegment = 0;
            std::size_t endSegment = 0;
        };

        struct RepairWindow
        {
            std::size_t beginIndex = 0;
            std::size_t endIndex = 0;
        };

        template<typename ValidationRequest>
        std::vector<InvalidSegmentRun> collectInvalidSegmentRuns(
            ProjectPlanningSceneSnapshot& scene,
            const std::vector<std::vector<double>>& path,
            const ValidationRequest& validation)
        {
            std::vector<InvalidSegmentRun> runs;
            std::size_t index = 0;
            while(index + 1 < path.size()) {
                const StateValidationResult segmentValidation =
                    scene.validateMotion(path[index], path[index + 1], validation);
                if(segmentValidation.valid) {
                    ++index;
                    continue;
                }

                InvalidSegmentRun run;
                run.beginSegment = index;
                run.endSegment = index;
                while(run.endSegment + 1 < path.size() - 1) {
                    const std::size_t nextIndex = run.endSegment + 1;
                    const StateValidationResult nextValidation =
                        scene.validateMotion(path[nextIndex], path[nextIndex + 1], validation);
                    if(nextValidation.valid) {
                        break;
                    }
                    run.endSegment = nextIndex;
                }
                runs.push_back(run);
                index = run.endSegment + 1;
            }
            return runs;
        }

        std::vector<double> interpolateJointState(
            const std::vector<double>& from,
            const std::vector<double>& to,
            double fraction,
            const std::vector<JointBound>& bounds)
        {
            std::vector<double> interpolated = from;
            const std::size_t count = std::min(from.size(), to.size());
            for(std::size_t jointIndex = 0; jointIndex < count; ++jointIndex) {
                double delta = to[jointIndex] - from[jointIndex];
                const bool isContinuous = !bounds.empty() &&
                    (jointIndex < bounds.size() ? bounds[jointIndex] : bounds.back()).continuous;
                if(isContinuous) {
                    delta = std::remainder(delta, kTwoPi);
                }
                interpolated[jointIndex] = from[jointIndex] + delta * fraction;
                if(isContinuous) {
                    interpolated[jointIndex] = wrapContinuousAngle(interpolated[jointIndex]);
                }
            }
            return clampToBounds(interpolated, bounds, nullptr);
        }

        std::vector<std::vector<double>> resampleJointPath(
            const std::vector<std::vector<double>>& coarsePath,
            std::size_t sampleCount,
            const std::vector<JointBound>& bounds)
        {
            std::vector<std::vector<double>> samples;
            if(coarsePath.empty() || sampleCount == 0) {
                return samples;
            }
            if(coarsePath.size() == 1 || sampleCount == 1) {
                samples.assign(sampleCount, clampToBounds(coarsePath.front(), bounds, nullptr));
                return samples;
            }

            std::vector<double> cumulative(coarsePath.size(), 0.0);
            for(std::size_t index = 1; index < coarsePath.size(); ++index) {
                cumulative[index] = cumulative[index - 1] +
                    stateDistance(coarsePath[index - 1], coarsePath[index], bounds);
            }

            const double totalDistance = cumulative.back();
            samples.reserve(sampleCount);
            for(std::size_t sampleIndex = 0; sampleIndex < sampleCount; ++sampleIndex) {
                const double targetDistance = sampleCount > 1
                    ? totalDistance * static_cast<double>(sampleIndex) / static_cast<double>(sampleCount - 1)
                    : 0.0;

                std::size_t segmentIndex = 0;
                while(segmentIndex + 1 < cumulative.size() &&
                    cumulative[segmentIndex + 1] < targetDistance) {
                    ++segmentIndex;
                }

                if(segmentIndex + 1 >= cumulative.size()) {
                    samples.push_back(clampToBounds(coarsePath.back(), bounds, nullptr));
                    continue;
                }

                const double segmentDistance = cumulative[segmentIndex + 1] - cumulative[segmentIndex];
                const double fraction = segmentDistance > kTiny
                    ? (targetDistance - cumulative[segmentIndex]) / segmentDistance
                    : 0.0;
                samples.push_back(interpolateJointState(
                    coarsePath[segmentIndex],
                    coarsePath[segmentIndex + 1],
                    std::clamp(fraction, 0.0, 1.0),
                    bounds));
            }

            return samples;
        }

        bool locateSafeOmplAnchors(
            ProjectPlanningSceneSnapshot& scene,
            const std::vector<std::vector<double>>& path,
            const InvalidSegmentRun& run,
            std::size_t* beginIndex,
            std::size_t* endIndex)
        {
            if(beginIndex == nullptr || endIndex == nullptr || path.size() < 2) {
                return false;
            }

            std::size_t left = run.beginSegment;
            while(true) {
                if(scene.validateState(path[left]).valid) {
                    break;
                }
                if(left == 0) {
                    return false;
                }
                --left;
            }

            std::size_t right = run.endSegment + 1;
            while(right < path.size() && !scene.validateState(path[right]).valid) {
                ++right;
            }
            if(right >= path.size()) {
                return false;
            }

            if(left >= right) {
                return false;
            }

            *beginIndex = left;
            *endIndex = right;
            return true;
        }

        bool findNearbySafeState(
            ProjectPlanningSceneSnapshot& scene,
            const std::vector<double>& center,
            const std::vector<JointBound>& bounds,
            std::vector<double>* safeState)
        {
            if(safeState == nullptr || center.empty()) {
                return false;
            }

            std::vector<double> candidate = clampToBounds(center, bounds, nullptr);
            if(scene.validateState(candidate).valid) {
                *safeState = std::move(candidate);
                return true;
            }

            // Escape a configuration that starts inside the obstacle by probing
            // small joint-space shells first. This changes only an invalid end
            // point; all interior waypoints remain governed by OMPL validation.
            const std::array<double, 6> radii = { 0.05, 0.10, 0.20, 0.35, 0.55, 0.80 };
            for(const double radius : radii) {
                for(std::size_t joint = 0; joint < center.size(); ++joint) {
                    for(const double sign : { -1.0, 1.0 }) {
                        candidate = center;
                        candidate[joint] += sign * radius;
                        candidate = clampToBounds(candidate, bounds, nullptr);
                        if(scene.validateState(candidate).valid) {
                            *safeState = std::move(candidate);
                            return true;
                        }
                    }
                }
            }

            std::mt19937 generator(928371u);
            std::uniform_real_distribution<double> perturbation(-1.0, 1.0);
            for(std::size_t sample = 0; sample < 256; ++sample) {
                candidate = center;
                const double radius = 0.10 + 0.90 * static_cast<double>(sample) / 255.0;
                for(std::size_t joint = 0; joint < center.size(); ++joint) {
                    candidate[joint] += radius * perturbation(generator);
                }
                candidate = clampToBounds(candidate, bounds, nullptr);
                if(scene.validateState(candidate).valid) {
                    *safeState = std::move(candidate);
                    return true;
                }
            }
            return false;
        }

        std::vector<std::vector<double>> unwrapContinuousPath(
            const std::vector<std::vector<double>>& source,
            const std::vector<JointBound>& bounds)
        {
            std::vector<std::vector<double>> result = source;
            if(result.empty()) {
                return result;
            }

            for(std::size_t point = 1; point < result.size(); ++point) {
                const std::size_t count = std::min(result[point].size(), bounds.size());
                for(std::size_t joint = 0; joint < count; ++joint) {
                    if(!bounds[joint].continuous) {
                        continue;
                    }
                    result[point][joint] = result[point - 1][joint] +
                        std::remainder(result[point][joint] - result[point - 1][joint], kTwoPi);
                }
            }
            return result;
        }

        double localSmoothingCost(
            const std::vector<double>& previous,
            const std::vector<double>& current,
            const std::vector<double>& next,
            const std::vector<double>& reference,
            const std::vector<JointBound>& bounds,
            double seedWeight)
        {
            double cost = 0.0;
            const std::size_t count = std::min(current.size(), std::min(previous.size(), next.size()));
            for(std::size_t joint = 0; joint < count; ++joint) {
                const bool continuous = joint < bounds.size() && bounds[joint].continuous;
                const double midpoint = 0.5 * (previous[joint] + next[joint]);
                const double curvature = current[joint] - midpoint;
                double referenceValue = joint < reference.size() ? reference[joint] : current[joint];
                if(continuous) {
                    referenceValue = current[joint] +
                        std::remainder(referenceValue - current[joint], kTwoPi);
                }
                const double seedError = current[joint] - referenceValue;
                cost += curvature * curvature + seedWeight * seedError * seedError;
            }
            return cost;
        }

        bool smoothCollisionFreePath(
            ProjectPlanningSceneSnapshot& scene,
            const std::vector<JointBound>& bounds,
            const MotionValidationOptions& validation,
            const std::vector<std::vector<double>>& referencePath,
            int iterationCount,
            double smoothingStep,
            double seedWeight,
            bool keepEndpoints,
            std::vector<std::vector<double>>* path)
        {
            if(path == nullptr || path->size() < 3) {
                return false;
            }

            *path = unwrapContinuousPath(*path, bounds);
            const std::vector<std::vector<double>> reference =
                unwrapContinuousPath(referencePath, bounds);
            const int passes = std::max(0, std::min(iterationCount, 8));
            const double step = std::clamp(
                std::isfinite(smoothingStep) ? smoothingStep : 0.25,
                0.02,
                0.50);
            const double effectiveSeedWeight = std::clamp(
                std::isfinite(seedWeight) ? seedWeight : 0.10,
                0.0,
                1.0);
            bool changed = false;

            for(int pass = 0; pass < passes; ++pass) {
                for(std::size_t point = 1; point + 1 < path->size(); ++point) {
                    if(keepEndpoints && (point == 0 || point + 1 == path->size())) {
                        continue;
                    }

                    const std::vector<double>& previous = (*path)[point - 1];
                    const std::vector<double>& current = (*path)[point];
                    const std::vector<double>& next = (*path)[point + 1];
                    const std::vector<double>& referencePoint = point < reference.size()
                        ? reference[point]
                        : current;
                    std::vector<double> candidate = current;
                    const std::size_t count = std::min(candidate.size(), std::min(previous.size(), next.size()));
                    for(std::size_t joint = 0; joint < count; ++joint) {
                        const bool continuous = joint < bounds.size() && bounds[joint].continuous;
                        const double midpoint = 0.5 * (previous[joint] + next[joint]);
                        double referenceValue = joint < referencePoint.size()
                            ? referencePoint[joint]
                            : current[joint];
                        if(continuous) {
                            referenceValue = current[joint] +
                                std::remainder(referenceValue - current[joint], kTwoPi);
                        }
                        const double desired = (1.0 - effectiveSeedWeight) * midpoint +
                            effectiveSeedWeight * referenceValue;
                        candidate[joint] = current[joint] + step * (desired - current[joint]);
                    }
                    for(std::size_t joint = 0; joint < count && joint < bounds.size(); ++joint) {
                        if(!bounds[joint].continuous) {
                            candidate[joint] = std::max(
                                bounds[joint].lower,
                                std::min(bounds[joint].upper, candidate[joint]));
                        }
                    }

                    const double beforeCost = localSmoothingCost(
                        previous, current, next, referencePoint, bounds, effectiveSeedWeight);
                    const double afterCost = localSmoothingCost(
                        previous, candidate, next, referencePoint, bounds, effectiveSeedWeight);
                    if(!(afterCost + 1.0e-12 < beforeCost) ||
                        !scene.validateState(candidate).valid ||
                        !scene.validateMotion(previous, candidate, validation).valid ||
                        !scene.validateMotion(candidate, next, validation).valid) {
                        continue;
                    }

                    (*path)[point] = std::move(candidate);
                    changed = true;
                }
            }
            return changed;
        }

        std::vector<std::vector<double>> pathSegmentFromTrajectory(
            const robottrajectory::JointTrajectory& trajectory)
        {
            std::vector<std::vector<double>> path;
            path.reserve(trajectory.points.size());
            for(const robottrajectory::TimedJointPoint& point : trajectory.points) {
                path.push_back(point.q);
            }
            return path;
        }

        bool repairCollisionRunsWithOmpl(
            ProjectPlanningSceneSnapshot& scene,
            const std::vector<JointBound>& bounds,
            const MotionValidationOptions& validation,
            const ProjectCdfQpRepairOptions& options,
            ProjectCdfQpRepairStatistics* statistics,
            std::vector<MotionPlanningDiagnostic>* diagnostics,
            std::vector<std::vector<double>>* path)
        {
            (void)options;
            (void)statistics;
            if(path == nullptr || path->size() < 2) {
                return false;
            }

            // A long imported trajectory can contain many disconnected collision
            // runs. Give each run several chances with progressively wider local
            // bounds, but keep every attempt short so one bad window cannot stall
            // the complete repair.
            const int maxPasses = path->size() > 100 ? 96 : 12;
            bool changedAny = false;
            std::vector<std::pair<std::size_t, std::size_t>> failedIntervals;
            for(int pass = 0; pass < maxPasses; ++pass) {
                const std::vector<InvalidSegmentRun> runs =
                    collectInvalidSegmentRuns(scene, *path, validation);
                if(runs.empty()) {
                    return changedAny;
                }

                const InvalidSegmentRun* selectedRun = nullptr;
                for(const InvalidSegmentRun& candidate : runs) {
                    const auto candidateKey = std::make_pair(candidate.beginSegment, candidate.endSegment);
                    if(std::find(failedIntervals.begin(), failedIntervals.end(), candidateKey) != failedIntervals.end()) {
                        continue;
                    }
                    const std::size_t candidateLength =
                        candidate.endSegment >= candidate.beginSegment
                            ? (candidate.endSegment - candidate.beginSegment + 1)
                            : 0;
                    const std::size_t selectedLength = selectedRun == nullptr
                        ? 0
                        : (selectedRun->endSegment >= selectedRun->beginSegment
                            ? (selectedRun->endSegment - selectedRun->beginSegment + 1)
                            : 0);
                    if(selectedRun == nullptr || candidateLength > selectedLength) {
                        selectedRun = &candidate;
                    }
                }

                if(selectedRun == nullptr) {
                    return changedAny;
                }

                const InvalidSegmentRun& run = *selectedRun;
                const auto intervalKey = std::make_pair(run.beginSegment, run.endSegment);
                std::size_t beginAnchor = 0;
                std::size_t endAnchor = 0;
                const bool hasRegularAnchors = locateSafeOmplAnchors(
                    scene,
                    *path,
                    run,
                    &beginAnchor,
                    &endAnchor);
                std::vector<double> escapedStart;
                std::vector<double> escapedGoal;
                if(!hasRegularAnchors) {
                    const bool touchesStart = run.beginSegment == 0;
                    const bool touchesEnd = run.endSegment + 1 >= path->size() - 1;
                    if(touchesStart) {
                        endAnchor = run.endSegment + 1;
                        if(endAnchor >= path->size() ||
                            !scene.validateState((*path)[endAnchor]).valid ||
                            !findNearbySafeState(scene, (*path)[0], bounds, &escapedStart)) {
                            escapedStart.clear();
                        }
                        beginAnchor = 0;
                    }
                    if(touchesEnd && escapedStart.empty()) {
                        beginAnchor = run.beginSegment;
                        if(!scene.validateState((*path)[beginAnchor]).valid ||
                            !findNearbySafeState(scene, path->back(), bounds, &escapedGoal)) {
                            escapedGoal.clear();
                        }
                        endAnchor = path->size() - 1;
                    }
                    if((!touchesStart && !touchesEnd) ||
                        (escapedStart.empty() && escapedGoal.empty())) {
                        addDiagnostic(
                            diagnostics,
                            "ompl_anchor_missing",
                            "Failed to locate safe anchor states around collision interval "
                                + std::to_string(run.beginSegment + 1)
                                + " -> " + std::to_string(run.endSegment + 2) + ".");
                        failedIntervals.push_back(intervalKey);
                        continue;
                    }
                }

                if(endAnchor <= beginAnchor + 1) {
                    failedIntervals.push_back(intervalKey);
                    continue;
                }

                std::vector<std::vector<double>> rawSegment;
                rawSegment.reserve(endAnchor - beginAnchor + 1);
                for(std::size_t index = beginAnchor; index <= endAnchor; ++index) {
                    rawSegment.push_back((*path)[index]);
                }

                const std::size_t replacementCount = endAnchor - beginAnchor + 1;
                std::vector<double> rawStart = escapedStart.empty()
                    ? (*path)[beginAnchor]
                    : escapedStart;
                std::vector<double> rawGoal = escapedGoal.empty()
                    ? (*path)[endAnchor]
                    : escapedGoal;

                // Limit OMPL to a corridor around the original collision interval.
                // A global joint-space RRT can legally solve the problem by taking a
                // very large detour, which is unsafe for a surface-following path.
                std::vector<JointBound> localBounds = bounds;
                for(std::size_t joint = 0; joint < localBounds.size(); ++joint) {
                    const JointBound& globalBound = bounds[joint];
                    double reference = joint < rawStart.size() ? rawStart[joint] : 0.0;
                    double minimum = reference;
                    double maximum = reference;
                    for(const std::vector<double>& state : rawSegment) {
                        if(joint >= state.size()) {
                            continue;
                        }
                        double value = state[joint];
                        if(globalBound.continuous) {
                            value = reference + std::remainder(value - reference, kTwoPi);
                        }
                        minimum = std::min(minimum, value);
                        maximum = std::max(maximum, value);
                    }

                    // Try progressively wider local corridors. Continuous joints
                    // are represented in this local unwrapped chart; the planning
                    // scene normalizes them again before collision evaluation.
                    const double corridor = pass < 4 ? 0.25 : (pass < 8 ? 0.45 : 0.75);
                    const double lower = globalBound.continuous
                        ? minimum - corridor
                        : std::max(globalBound.lower, minimum - corridor);
                    const double upper = globalBound.continuous
                        ? maximum + corridor
                        : std::min(globalBound.upper, maximum + corridor);
                    if(std::isfinite(lower) && std::isfinite(upper) && lower < upper) {
                        localBounds[joint].lower = lower;
                        localBounds[joint].upper = upper;
                    }
                }

                JointPlanningProblem problem;
                problem.robotId = scene.robotId();
                problem.jointNames = scene.jointNames();
                problem.jointBounds = localBounds;
                problem.start = rawStart;
                problem.goal = rawGoal;
                for(std::size_t joint = 0; joint < problem.start.size() && joint < localBounds.size(); ++joint) {
                    if(localBounds[joint].continuous) {
                        const double reference = rawStart[joint];
                        problem.start[joint] = reference;
                        problem.goal[joint] = reference + std::remainder(rawGoal[joint] - reference, kTwoPi);
                    }
                }
                problem.planner.plannerId = "RRTConnect";
                problem.planner.timeoutSeconds = pass < 8 ? 1.0 : 1.5;
                problem.planner.range = 0.0;
                problem.planner.randomSeed = static_cast<std::uint32_t>(1337u + pass * 97u + run.beginSegment * 13u);
                problem.planner.simplifyPath = true;
                problem.validation = validation;
                problem.validation.maxJointStep = std::min(
                    std::max(validation.maxJointStep, 0.005),
                    0.02);
                problem.postProcess.duration = std::max(
                    1.0,
                    static_cast<double>(replacementCount > 1 ? replacementCount - 1 : 1));
                problem.postProcess.minimumWaypointCount = std::max<std::size_t>(
                    rawSegment.size(),
                    6);

                OmplMotionPlanner omplPlanner;
                MotionPlanningResult omplResult = omplPlanner.plan(problem, scene);
                if(!omplResult.succeeded() || omplResult.trajectory.points.size() < 2) {
                    addDiagnostic(
                        diagnostics,
                        "ompl_segment_failed",
                        omplResult.diagnostics.empty()
                            ? "OMPL failed to repair a collision interval."
                            : omplResult.diagnostics.front().message);
                    failedIntervals.push_back(intervalKey);
                    continue;
                }

                std::vector<std::vector<double>> coarseSegment =
                    pathSegmentFromTrajectory(omplResult.trajectory);
                if(coarseSegment.size() < 2) {
                    failedIntervals.push_back(intervalKey);
                    continue;
                }

                std::vector<std::vector<double>> densifiedSegment =
                    resampleJointPath(coarseSegment, replacementCount, bounds);
                if(densifiedSegment.size() != replacementCount) {
                    addDiagnostic(
                        diagnostics,
                        "ompl_segment_resample_failed",
                        "Failed to resample OMPL repaired segment to the original waypoint count.");
                    failedIntervals.push_back(intervalKey);
                    continue;
                }

                std::vector<std::vector<double>> candidatePath = *path;
                for(std::size_t index = 0; index < replacementCount; ++index) {
                    candidatePath[beginAnchor + index] = densifiedSegment[index];
                }

                bool replacementValid = true;
                for(std::size_t index = beginAnchor; index < endAnchor; ++index) {
                    if(!scene.validateMotion(candidatePath[index], candidatePath[index + 1], validation).valid) {
                        replacementValid = false;
                        break;
                    }
                }
                // Check the two seams as well. A replacement may be valid in its
                // interior yet invalidate an already repaired neighbouring edge.
                if(replacementValid && beginAnchor > 0) {
                    replacementValid = scene.validateMotion(
                        candidatePath[beginAnchor - 1],
                        candidatePath[beginAnchor],
                        validation).valid;
                }
                if(replacementValid && endAnchor + 1 < candidatePath.size()) {
                    replacementValid = scene.validateMotion(
                        candidatePath[endAnchor],
                        candidatePath[endAnchor + 1],
                        validation).valid;
                }
                if(!replacementValid) {
                    addDiagnostic(
                        diagnostics,
                        "ompl_segment_validation_failed",
                        "Rejected OMPL replacement because its resampled segment is still invalid.");
                    failedIntervals.push_back(intervalKey);
                    continue;
                }

                for(std::size_t index = 0; index < replacementCount; ++index) {
                    (*path)[beginAnchor + index] = std::move(candidatePath[beginAnchor + index]);
                }
                addDiagnostic(
                    diagnostics,
                    "ompl_segment_repaired",
                    "Repaired collision interval "
                        + std::to_string(run.beginSegment + 1) + " -> "
                        + std::to_string(run.endSegment + 2)
                        + " with OMPL/RRTConnect before CDF/QP smoothing.");
                changedAny = true;
            }

            return changedAny;
        }

        struct SignedDistanceSample
        {
            bool valid = false;
            bool inCollision = false;
            double phi = -std::numeric_limits<double>::max();
            double rawDistance = std::numeric_limits<double>::max();
            collision::Vec3 nearestDirection = collision::Vec3::Zero();
            bool hasNearestDirection = false;
            std::string message;
        };

        SignedDistanceSample evaluateSignedPhi(
            ProjectPlanningSceneSnapshot& scene,
            const std::vector<double>& q,
            double safetyMargin,
            double distanceThreshold,
            ProjectCdfQpRepairStatistics& statistics)
        {
            SignedDistanceSample sample;

            std::string stateError;
            if(!scene.setState(q, &stateError)) {
                sample.message = stateError.empty() ? "Failed to set planning scene state." : stateError;
                return sample;
            }

            bool sawFiniteDistance = false;
            bool checkedAnyDetector = false;
            double minimumPhi = std::numeric_limits<double>::max();
            double minimumDistance = std::numeric_limits<double>::max();
            collision::Vec3 bestDirection = collision::Vec3::Zero();
            bool bestDirectionValid = false;
            bool inCollision = false;

            for(const std::string& detectorId : scene.collisionDetectorIds()) {
                checkedAnyDetector = true;
                ++statistics.collisionQueries;
                const simulation_runtime::Result checkResult =
                    scene.collisionRuntime().checkDetector(detectorId);
                if(!checkResult.success) {
                    sample.message = checkResult.message;
                    return sample;
                }

                const collision::CollisionResult* collisionResult =
                    scene.collisionRuntime().resultOf(detectorId);
                if(collisionResult == nullptr) {
                    sample.message = "Collision detector produced no result: " + detectorId;
                    return sample;
                }

                if(collisionResult->inCollision()) {
                    inCollision = true;
                    const double penetrationDepth = maxContactPenetrationDepth(*collisionResult);
                    const double nearestDistance = nearestPointDistance(*collisionResult);
                    const double collisionMagnitude =
                        penetrationDepth > 0.0
                            ? penetrationDepth
                            : (std::isfinite(nearestDistance) && nearestDistance > 0.0
                                ? nearestDistance
                                : 1.0e-5);
                    const double signedDistance = -collisionMagnitude;
                    collision::Vec3 direction = collision::Vec3::Zero();
                    const bool directionValid = extractNearestDirection(*collisionResult, &direction);
                    const double candidatePhi = signedDistance - safetyMargin;
                    if(candidatePhi < minimumPhi) {
                        minimumPhi = candidatePhi;
                        minimumDistance = signedDistance;
                        bestDirection = direction;
                        bestDirectionValid = directionValid;
                    } else {
                        minimumDistance = std::min(minimumDistance, signedDistance);
                    }
                    sawFiniteDistance = true;
                } else if(std::isfinite(collisionResult->minDistance) &&
                    collisionResult->minDistance < std::numeric_limits<double>::max() * 0.25) {
                    const double nearestDistance = nearestPointDistance(*collisionResult);
                    const double signedDistance =
                        std::isfinite(nearestDistance) ? nearestDistance : collisionResult->minDistance;
                    collision::Vec3 direction = collision::Vec3::Zero();
                    const bool directionValid = extractNearestDirection(*collisionResult, &direction);
                    sawFiniteDistance = true;
                    if(signedDistance - safetyMargin < minimumPhi) {
                        minimumPhi = signedDistance - safetyMargin;
                        minimumDistance = signedDistance;
                        bestDirection = direction;
                        bestDirectionValid = directionValid;
                    } else {
                        minimumDistance = std::min(minimumDistance, signedDistance);
                    }
                }
            }

            if(!inCollision && !sawFiniteDistance && checkedAnyDetector) {
                const double saturatedDistance =
                    distanceThreshold > 0.0 && std::isfinite(distanceThreshold)
                        ? distanceThreshold
                        : safetyMargin;
                sample.valid = true;
                sample.inCollision = false;
                sample.rawDistance = saturatedDistance;
                sample.phi = saturatedDistance - safetyMargin;
                return sample;
            }

            if(!sawFiniteDistance) {
                sample.message = "Collision detector did not report a usable distance or collision result.";
                return sample;
            }

            sample.valid = true;
            sample.inCollision = inCollision;
            sample.rawDistance = minimumDistance;
            sample.phi = minimumPhi;
            sample.nearestDirection = bestDirection;
            sample.hasNearestDirection = bestDirectionValid;
            return sample;
        }

        struct CdfLinearization
        {
            SignedDistanceSample sample;
            std::vector<double> gradient;
        };

        CdfLinearization linearizeSignedPhi(
            ProjectPlanningSceneSnapshot& scene,
            const std::vector<double>& q,
            const std::vector<JointBound>& bounds,
            double safetyMargin,
            double distanceThreshold,
            double finiteDifferenceStep,
            ProjectCdfQpRepairStatistics& statistics)
        {
            CdfLinearization linearization;
            linearization.sample = evaluateSignedPhi(scene, q, safetyMargin, distanceThreshold, statistics);
            linearization.gradient.assign(q.size(), 0.0);
            if(!linearization.sample.valid) {
                return linearization;
            }

            const double step = finiteDifferenceStep > 0.0 && std::isfinite(finiteDifferenceStep)
                ? finiteDifferenceStep
                : 5.0e-4;

            for(std::size_t index = 0; index < q.size(); ++index) {
                std::vector<double> plus = q;
                std::vector<double> minus = q;
                plus[index] += step;
                minus[index] -= step;
                if(index < bounds.size() && !bounds[index].continuous) {
                    plus[index] = std::max(bounds[index].lower, std::min(bounds[index].upper, plus[index]));
                    minus[index] = std::max(bounds[index].lower, std::min(bounds[index].upper, minus[index]));
                }

                const double denominator = plus[index] - minus[index];
                if(std::abs(denominator) <= kTiny) {
                    continue;
                }

                const SignedDistanceSample plusSample =
                    evaluateSignedPhi(scene, plus, safetyMargin, distanceThreshold, statistics);
                const SignedDistanceSample minusSample =
                    evaluateSignedPhi(scene, minus, safetyMargin, distanceThreshold, statistics);

                if(plusSample.valid && minusSample.valid) {
                    linearization.gradient[index] =
                        (plusSample.phi - minusSample.phi) / denominator;
                } else if(plusSample.valid && std::abs(plus[index] - q[index]) > kTiny) {
                    linearization.gradient[index] =
                        (plusSample.phi - linearization.sample.phi) / (plus[index] - q[index]);
                } else if(minusSample.valid && std::abs(q[index] - minus[index]) > kTiny) {
                    linearization.gradient[index] =
                        (linearization.sample.phi - minusSample.phi) / (q[index] - minus[index]);
                }
            }

            return linearization;
        }

        struct SparseTripletEntry
        {
            c_int row = 0;
            c_int col = 0;
            c_float value = 0.0;
        };

        struct OsqpMatrixDeleter
        {
            void operator()(csc* matrix) const
            {
                if(matrix != nullptr) {
                    csc_spfree(matrix);
                }
            }
        };

        struct OsqpWorkspaceDeleter
        {
            void operator()(OSQPWorkspace* workspace) const
            {
                if(workspace != nullptr) {
                    osqp_cleanup(workspace);
                }
            }
        };

        using OsqpMatrixPtr = std::unique_ptr<csc, OsqpMatrixDeleter>;
        using OsqpWorkspacePtr = std::unique_ptr<OSQPWorkspace, OsqpWorkspaceDeleter>;

        OsqpMatrixPtr makeSparseMatrixFromTriplets(
            c_int rows,
            c_int cols,
            const std::vector<SparseTripletEntry>& entries)
        {
            const c_int nonZeroCount = static_cast<c_int>(entries.size());
            csc* triplet = csc_spalloc(rows, cols, std::max<c_int>(1, nonZeroCount), 1, 1);
            if(triplet == nullptr) {
                return OsqpMatrixPtr(nullptr);
            }

            for(c_int index = 0; index < nonZeroCount; ++index) {
                const SparseTripletEntry& entry = entries[static_cast<std::size_t>(index)];
                triplet->i[index] = entry.row;
                triplet->p[index] = entry.col;
                triplet->x[index] = entry.value;
            }
            triplet->nz = nonZeroCount;

            csc* matrix = triplet_to_csc(triplet, OSQP_NULL);
            csc_spfree(triplet);
            return OsqpMatrixPtr(matrix);
        }

        struct QpSolveResult
        {
            bool success = false;
            std::vector<std::vector<double>> path;
            int solverIterations = 0;
            double maximumSlack = 0.0;
            double maximumCorrection = 0.0;
            std::string message;
        };

        struct QpNumericsAttempt
        {
            double regularizer = 1.0e-5;
            double slackPenalty = 1.0e3;
            double smoothScale = 1.0;
            int maxIterations = 4000;
            double epsAbs = 1.0e-4;
            double epsRel = 1.0e-4;
            const char* label = "default";
        };

        std::string osqpSetupFailureMessage(c_int code)
        {
            std::ostringstream stream;
            stream << "OSQP setup failed with code " << code;
            if(code == OSQP_NONCVX_ERROR) {
                stream << " (KKT factorization failed; increasing Hessian regularization)";
            } else if(code >= OSQP_DATA_VALIDATION_ERROR && code <= OSQP_WORKSPACE_NOT_INIT_ERROR) {
                stream << " (" << OSQP_ERROR_MESSAGE[code - 1] << ")";
            }
            stream << ".";
            return stream.str();
        }

        QpSolveResult solveTrajectoryWithOsqp(
            const std::vector<std::vector<double>>& currentPath,
            const std::vector<std::vector<double>>& seedPath,
            const std::vector<CdfLinearization>& linearizations,
            const std::vector<JointBound>& bounds,
            const ProjectCdfQpRepairOptions& options)
        {
            QpSolveResult result;
            const std::size_t waypointCount = currentPath.size();
            if(waypointCount == 0 || currentPath.front().empty()) {
                result.message = "QP solve requested with an empty trajectory.";
                return result;
            }
            if(bounds.empty()) {
                result.message = "QP solve requested without joint bounds.";
                return result;
            }
            if(linearizations.size() != waypointCount) {
                result.message = "QP solve received inconsistent linearization data.";
                return result;
            }

            const std::size_t jointCount = currentPath.front().size();
            const std::size_t configVariableCount = waypointCount * jointCount;
            const std::size_t slackVariableCount = waypointCount;
            const std::size_t variableCount = configVariableCount + slackVariableCount;
            const std::size_t constraintCount = (2 * waypointCount) + variableCount;

            const double seedTrackingWeight = options.seedTrackingWeight > 0.0 && std::isfinite(options.seedTrackingWeight)
                ? options.seedTrackingWeight
                : 0.0;
            const double smoothWeight = options.smoothWeight > 0.0 && std::isfinite(options.smoothWeight)
                ? options.smoothWeight
                : 0.0;
            const double repairGain = std::clamp(options.repairGain, 0.0, 1.0);
            const double trustRegion = options.trustRegion > 0.0 && std::isfinite(options.trustRegion)
                ? options.trustRegion
                : 0.03;
            const double seedCorridor = options.seedCorridor > 0.0 && std::isfinite(options.seedCorridor)
                ? options.seedCorridor
                : 0.15;
            const double targetPhi = options.targetClearance;

            const auto variableIndex = [jointCount](std::size_t waypoint, std::size_t joint) -> std::size_t {
                return waypoint * jointCount + joint;
            };

            std::vector<c_float> qVector(variableCount, 0.0);
            for(std::size_t waypoint = 0; waypoint < waypointCount; ++waypoint) {
                for(std::size_t joint = 0; joint < jointCount; ++joint) {
                    const std::size_t index = variableIndex(waypoint, joint);
                    const double seedValue = waypoint < seedPath.size() && joint < seedPath[waypoint].size()
                        ? seedPath[waypoint][joint]
                        : 0.0;
                    qVector[index] = static_cast<c_float>(-seedTrackingWeight * seedValue);
                }
            }

            std::vector<SparseTripletEntry> aEntries;
            aEntries.reserve(waypointCount * (jointCount + 1) + variableCount + waypointCount);
            std::vector<c_float> lowerBounds(constraintCount, -OSQP_INFTY);
            std::vector<c_float> upperBounds(constraintCount, OSQP_INFTY);

            for(std::size_t waypoint = 0; waypoint < waypointCount; ++waypoint) {
                const CdfLinearization& linearization = linearizations[waypoint];
                double gradientNorm = vectorNorm(linearization.gradient);
                if(!std::isfinite(gradientNorm) || gradientNorm <= 1.0e-9) {
                    gradientNorm = 1.0;
                }
                const double rowScale = 1.0 / gradientNorm;
                const double rhs = dot(linearization.gradient, currentPath[waypoint]) +
                    repairGain * (targetPhi - linearization.sample.phi);
                if(!std::isfinite(rhs)) {
                    result.message = "QP linearized CDF constraint has a non-finite right-hand side.";
                    return result;
                }
                const double scaledRhs = rhs * rowScale;

                const c_int row = static_cast<c_int>(waypoint);
                for(std::size_t joint = 0; joint < jointCount; ++joint) {
                    if(joint >= linearization.gradient.size() ||
                        !std::isfinite(linearization.gradient[joint])) {
                        result.message = "QP linearized CDF constraint has a non-finite gradient.";
                        return result;
                    }
                    const std::size_t index = variableIndex(waypoint, joint);
                    aEntries.push_back({
                        row,
                        static_cast<c_int>(index),
                        static_cast<c_float>(linearization.gradient[joint] * rowScale)});
                }
                aEntries.push_back({
                    row,
                    static_cast<c_int>(configVariableCount + waypoint),
                    static_cast<c_float>(rowScale)});

                lowerBounds[waypoint] = static_cast<c_float>(scaledRhs);
                upperBounds[waypoint] = OSQP_INFTY;
            }

            const std::vector<JointBound>& effectiveBounds = bounds;
            for(std::size_t waypoint = 0; waypoint < waypointCount; ++waypoint) {
                for(std::size_t joint = 0; joint < jointCount; ++joint) {
                    const std::size_t index = variableIndex(waypoint, joint);
                    const c_int row = static_cast<c_int>(waypointCount + index);
                    const JointBound& jointBound = joint < effectiveBounds.size()
                        ? effectiveBounds[joint]
                        : effectiveBounds.back();
                    double value = currentPath[waypoint][joint];
                    if(jointBound.continuous) {
                        value = wrapContinuousAngle(value);
                    }
                    const double lowerLimit = jointBound.lower;
                    const double upperLimit = jointBound.upper;
                    double lower = std::max(lowerLimit, value - trustRegion);
                    double upper = std::min(upperLimit, value + trustRegion);
                    double seedValue = waypoint < seedPath.size() && joint < seedPath[waypoint].size()
                        ? seedPath[waypoint][joint]
                        : value;
                    if(jointBound.continuous) {
                        seedValue = wrapContinuousAngle(seedValue);
                    }
                    const double seedLower = std::max(lowerLimit, seedValue - seedCorridor);
                    const double seedUpper = std::min(upperLimit, seedValue + seedCorridor);
                    lower = std::max(lower, seedLower);
                    upper = std::min(upper, seedUpper);
                    if(options.keepEndpoints && (waypoint == 0 || waypoint + 1 == waypointCount)) {
                        lower = upper = value;
                    } else if(lower > upper) {
                        const double clamped = std::max(lowerLimit, std::min(upperLimit, value));
                        lower = upper = clamped;
                    }
                    if(!std::isfinite(lower) || !std::isfinite(upper) || lower > upper) {
                        result.message = "QP joint trust-region bound is invalid.";
                        return result;
                    }

                    aEntries.push_back({ row, static_cast<c_int>(index), static_cast<c_float>(1.0) });
                    lowerBounds[static_cast<std::size_t>(row)] = static_cast<c_float>(lower);
                    upperBounds[static_cast<std::size_t>(row)] = static_cast<c_float>(upper);
                }
            }

            for(std::size_t waypoint = 0; waypoint < waypointCount; ++waypoint) {
                const std::size_t index = configVariableCount + waypoint;
                const c_int row = static_cast<c_int>(waypointCount + configVariableCount + waypoint);
                aEntries.push_back({
                    row,
                    static_cast<c_int>(index),
                    static_cast<c_float>(1.0)});
                lowerBounds[static_cast<std::size_t>(row)] = static_cast<c_float>(0.0);
                upperBounds[static_cast<std::size_t>(row)] = OSQP_INFTY;
            }

            OsqpMatrixPtr A = makeSparseMatrixFromTriplets(
                static_cast<c_int>(constraintCount),
                static_cast<c_int>(variableCount),
                aEntries);
            if(A == nullptr) {
                result.message = "Failed to build the QP constraint matrix.";
                return result;
            }

            std::vector<c_float> warmStart(variableCount, 0.0);
            for(std::size_t waypoint = 0; waypoint < waypointCount; ++waypoint) {
                for(std::size_t joint = 0; joint < jointCount; ++joint) {
                    const std::size_t index = variableIndex(waypoint, joint);
                    double value = currentPath[waypoint][joint];
                    if(joint < effectiveBounds.size() && effectiveBounds[joint].continuous) {
                        value = wrapContinuousAngle(value);
                    }
                    warmStart[index] = static_cast<c_float>(value);
                }
            }

            const auto buildHessian = [&](const QpNumericsAttempt& attempt) -> OsqpMatrixPtr {
                const double effectiveRegularizer = std::max(1.0e-8, attempt.regularizer);
                const double effectiveSmoothWeight = std::max(0.0, smoothWeight * attempt.smoothScale);
                const double effectiveSlackPenalty = std::max(1.0, attempt.slackPenalty);

                std::vector<SparseTripletEntry> pEntries;
                pEntries.reserve(
                    configVariableCount +
                    (waypointCount > 0 ? (waypointCount - 1) * jointCount : 0) +
                    slackVariableCount);

                for(std::size_t waypoint = 0; waypoint < waypointCount; ++waypoint) {
                    const bool hasPrevious = waypoint > 0;
                    const bool hasNext = waypoint + 1 < waypointCount;
                    for(std::size_t joint = 0; joint < jointCount; ++joint) {
                        const std::size_t index = variableIndex(waypoint, joint);
                        double diagonal = effectiveRegularizer + seedTrackingWeight;
                        if(hasPrevious) {
                            diagonal += effectiveSmoothWeight;
                        }
                        if(hasNext) {
                            diagonal += effectiveSmoothWeight;
                        }
                        pEntries.push_back({
                            static_cast<c_int>(index),
                            static_cast<c_int>(index),
                            static_cast<c_float>(diagonal)});
                        if(hasPrevious && effectiveSmoothWeight > 0.0) {
                            const std::size_t previousIndex = variableIndex(waypoint - 1, joint);
                            pEntries.push_back({
                                static_cast<c_int>(previousIndex),
                                static_cast<c_int>(index),
                                static_cast<c_float>(-effectiveSmoothWeight)});
                        }
                    }
                }
                for(std::size_t waypoint = 0; waypoint < slackVariableCount; ++waypoint) {
                    const std::size_t index = configVariableCount + waypoint;
                    pEntries.push_back({
                        static_cast<c_int>(index),
                        static_cast<c_int>(index),
                        static_cast<c_float>(effectiveSlackPenalty)});
                }

                return makeSparseMatrixFromTriplets(
                    static_cast<c_int>(variableCount),
                    static_cast<c_int>(variableCount),
                    pEntries);
            };

            const std::vector<QpNumericsAttempt> attempts = {
                { 1.0e-4, 1.0e3, 0.35, 5000, 2.0e-4, 2.0e-4, "mild_smooth_regularized" },
                { 1.0e-3, 8.0e2, 0.15, 7000, 3.0e-4, 3.0e-4, "reduced_smooth_regularized" },
                { 1.0e-2, 5.0e2, 0.0, 12000, 5.0e-4, 5.0e-4, "strict_diagonal_fallback" },
                { 1.0e-1, 2.0e2, 0.0, 16000, 1.0e-3, 1.0e-3, "loose_diagonal_fallback" }
            };

            std::string lastFailure;
            for(const QpNumericsAttempt& attempt : attempts) {
                OsqpMatrixPtr P = buildHessian(attempt);
                if(P == nullptr) {
                    lastFailure = "Failed to build the QP Hessian matrix.";
                    continue;
                }

                OSQPData data;
                data.n = static_cast<c_int>(variableCount);
                data.m = static_cast<c_int>(constraintCount);
                data.P = P.get();
                data.A = A.get();
                data.q = qVector.data();
                data.l = lowerBounds.data();
                data.u = upperBounds.data();

                OSQPSettings settings;
                osqp_set_default_settings(&settings);
                settings.verbose = 0;
                settings.warm_start = 1;
                settings.polish = 0;
                settings.max_iter = attempt.maxIterations;
                settings.eps_abs = static_cast<c_float>(attempt.epsAbs);
                settings.eps_rel = static_cast<c_float>(attempt.epsRel);
                settings.sigma = std::max(settings.sigma, static_cast<c_float>(attempt.regularizer));
                settings.check_termination = 25;

                OSQPWorkspace* workspace = nullptr;
                const c_int setupStatus = osqp_setup(&workspace, &data, &settings);
                OsqpWorkspacePtr workspaceGuard(workspace);
                if(setupStatus != 0 || workspace == nullptr) {
                    std::ostringstream stream;
                    stream << osqpSetupFailureMessage(setupStatus)
                        << " attempt=" << attempt.label
                        << ", reg=" << attempt.regularizer
                        << ", slack=" << attempt.slackPenalty
                        << ", smooth_scale=" << attempt.smoothScale;
                    lastFailure = stream.str();
                    continue;
                }

                if(osqp_warm_start_x(workspace, warmStart.data()) != 0) {
                    lastFailure = "OSQP warm start failed.";
                    continue;
                }

                const c_int solveStatus = osqp_solve(workspace);
                if(solveStatus != 0 || workspace->info == nullptr || workspace->solution == nullptr) {
                    std::ostringstream stream;
                    stream << "OSQP solve failed with code " << solveStatus << ".";
                    if(workspace->info != nullptr) {
                        stream << " status=" << workspace->info->status;
                    }
                    lastFailure = stream.str();
                    continue;
                }

                result.solverIterations = workspace->info->iter;
                if(!(workspace->info->status_val == OSQP_SOLVED ||
                    workspace->info->status_val == OSQP_SOLVED_INACCURATE)) {
                    std::ostringstream stream;
                    stream << "OSQP returned status " << workspace->info->status
                        << " in attempt " << attempt.label << ".";
                    lastFailure = stream.str();
                    continue;
                }

                result.path.resize(waypointCount);
                result.maximumSlack = 0.0;
                result.maximumCorrection = 0.0;
                for(std::size_t waypoint = 0; waypoint < waypointCount; ++waypoint) {
                    result.path[waypoint].resize(jointCount);
                    for(std::size_t joint = 0; joint < jointCount; ++joint) {
                        const std::size_t index = variableIndex(waypoint, joint);
                        const double value = static_cast<double>(workspace->solution->x[index]);
                        result.path[waypoint][joint] = effectiveBounds[joint].continuous
                            ? wrapContinuousAngle(value)
                            : value;
                    }

                    const double slackValue = static_cast<double>(workspace->solution->x[configVariableCount + waypoint]);
                    result.maximumSlack = std::max(result.maximumSlack, std::max(0.0, slackValue));
                    result.path[waypoint] = clampToBounds(result.path[waypoint], effectiveBounds, nullptr);
                    for(std::size_t joint = 0; joint < jointCount; ++joint) {
                        result.maximumCorrection = std::max(
                            result.maximumCorrection,
                            std::abs(result.path[waypoint][joint] - currentPath[waypoint][joint]));
                    }
                }

                if(options.keepEndpoints && waypointCount >= 2) {
                    result.path.front() = seedPath.front();
                    result.path.back() = seedPath.back();
                }

                result.success = true;
                return result;
            }

            result.message = lastFailure.empty()
                ? "OSQP failed to solve the CDF/QP subproblem."
                : lastFailure;
            return result;
        }

        std::filesystem::path projectBaseOrParent(
            const std::filesystem::path& projectBasePath)
        {
            return projectBasePath;
        }

        std::string segmentFailureMessage(
            std::size_t index,
            const StateValidationResult& validation)
        {
            std::ostringstream stream;
            stream << "Repaired trajectory segment " << (index + 1)
                   << " -> " << (index + 2) << " is still invalid";
            if(!validation.message.empty()) {
                stream << ": " << validation.message;
            }
            return stream.str();
        }

        std::vector<RepairWindow> buildRepairWindows(
            const std::vector<InvalidSegmentRun>& runs,
            std::size_t pathSize,
            std::size_t basePadding,
            std::size_t maxWindowPoints,
            std::size_t overlap,
            std::size_t riskPaddingBoost)
        {
            std::vector<RepairWindow> windows;
            if(pathSize < 2 || runs.empty()) {
                return windows;
            }

            std::vector<RepairWindow> paddedWindows;
            paddedWindows.reserve(runs.size());
            for(const InvalidSegmentRun& run : runs) {
                const std::size_t runLength = run.endSegment >= run.beginSegment
                    ? (run.endSegment - run.beginSegment + 1)
                    : 0;
                const std::size_t dynamicPadding =
                    basePadding + std::min(riskPaddingBoost, runLength / 4);
                RepairWindow window;
                window.beginIndex =
                    run.beginSegment > dynamicPadding ? run.beginSegment - dynamicPadding : 0;
                window.endIndex = std::min(pathSize - 1, run.endSegment + 1 + dynamicPadding);
                if(window.beginIndex < window.endIndex) {
                    paddedWindows.push_back(window);
                }
            }

            if(paddedWindows.empty()) {
                return windows;
            }

            std::sort(
                paddedWindows.begin(),
                paddedWindows.end(),
                [](const RepairWindow& lhs, const RepairWindow& rhs) {
                    if(lhs.beginIndex != rhs.beginIndex) {
                        return lhs.beginIndex < rhs.beginIndex;
                    }
                    return lhs.endIndex < rhs.endIndex;
                });

            std::vector<RepairWindow> mergedWindows;
            mergedWindows.push_back(paddedWindows.front());
            for(std::size_t index = 1; index < paddedWindows.size(); ++index) {
                RepairWindow& current = mergedWindows.back();
                const RepairWindow& next = paddedWindows[index];
                if(next.beginIndex <= current.endIndex + 1) {
                    current.endIndex = std::max(current.endIndex, next.endIndex);
                } else {
                    mergedWindows.push_back(next);
                }
            }

            const std::size_t effectiveOverlap = std::min(overlap, maxWindowPoints > 1 ? maxWindowPoints - 1 : 0);
            for(const RepairWindow& merged : mergedWindows) {
                std::size_t start = merged.beginIndex;
                while(start < merged.endIndex) {
                    const std::size_t chunkEnd = std::min(
                        merged.endIndex,
                        start + maxWindowPoints - 1);
                    windows.push_back({ start, chunkEnd });
                    if(chunkEnd >= merged.endIndex) {
                        break;
                    }

                    std::size_t nextStart = chunkEnd + 1;
                    if(effectiveOverlap > 0) {
                        nextStart = chunkEnd + 1 > effectiveOverlap
                            ? chunkEnd + 1 - effectiveOverlap
                            : 0;
                    }
                    if(nextStart <= start) {
                        nextStart = start + 1;
                    }
                    start = nextStart;
                }
            }

            return windows;
        }

        template<typename ValidationRequest>
        bool runCdfQpRepairIterations(
            ProjectPlanningSceneSnapshot& scene,
            const std::vector<std::vector<double>>& startPath,
            const std::vector<std::vector<double>>& seedPath,
            const std::vector<JointBound>& bounds,
            const ValidationRequest& validation,
            const ProjectCdfQpRepairOptions& options,
            ProjectCdfQpRepairStatistics* statistics,
            std::vector<MotionPlanningDiagnostic>* diagnostics,
            std::vector<std::vector<double>>* repairedPath)
        {
            if(repairedPath == nullptr || statistics == nullptr) {
                return false;
            }
            if(startPath.size() < 2 || startPath.front().empty()) {
                return false;
            }

            std::vector<std::vector<double>> path = startPath;
            auto countInvalidSegments = [&](const std::vector<std::vector<double>>& candidate) {
                int invalidCount = 0;
                for(std::size_t index = 0; index + 1 < candidate.size(); ++index) {
                    const StateValidationResult segmentValidation =
                        scene.validateMotion(candidate[index], candidate[index + 1], validation);
                    if(!segmentValidation.valid) {
                        ++invalidCount;
                    }
                }
                return invalidCount;
            };

            auto evaluatePathMinimum = [&](const std::vector<std::vector<double>>& candidate) {
                double minimumPhi = std::numeric_limits<double>::max();
                for(const std::vector<double>& q : candidate) {
                    const SignedDistanceSample sample =
                        evaluateSignedPhi(
                            scene,
                            q,
                            options.safetyMargin,
                            options.distanceThreshold,
                            *statistics);
                    if(!sample.valid) {
                        addDiagnostic(diagnostics, "cdf_distance_failed", sample.message);
                        return -std::numeric_limits<double>::max();
                    }
                    minimumPhi = std::min(minimumPhi, sample.phi);
                }
                return minimumPhi;
            };

            auto blendPath = [&](const std::vector<std::vector<double>>& from,
                                 const std::vector<std::vector<double>>& to,
                                 double alpha) {
                std::vector<std::vector<double>> blended = from;
                const std::size_t waypointCount = std::min(from.size(), to.size());
                for(std::size_t waypoint = 0; waypoint < waypointCount; ++waypoint) {
                    const std::size_t jointCount = std::min(from[waypoint].size(), to[waypoint].size());
                    blended[waypoint].resize(jointCount);
                    for(std::size_t joint = 0; joint < jointCount; ++joint) {
                        double delta = to[waypoint][joint] - from[waypoint][joint];
                        if(joint < bounds.size() && bounds[joint].continuous) {
                            delta = std::remainder(delta, kTwoPi);
                        }
                        blended[waypoint][joint] = from[waypoint][joint] + delta * alpha;
                        if(joint < bounds.size() && bounds[joint].continuous) {
                            blended[waypoint][joint] = wrapContinuousAngle(blended[waypoint][joint]);
                        }
                    }
                    blended[waypoint] = clampToBounds(blended[waypoint], bounds, nullptr);
                }
                return blended;
            };

            const double targetPhi = options.targetClearance;
            const int maxIterations = std::max(1, options.maxIterations);

            for(int iteration = 0; iteration < maxIterations; ++iteration) {
                std::vector<CdfLinearization> linearizations;
                linearizations.reserve(path.size());

                double currentPhi = std::numeric_limits<double>::max();
                bool allSamplesValid = true;
                for(const std::vector<double>& q : path) {
                    CdfLinearization linearization = linearizeSignedPhi(
                        scene,
                        q,
                        bounds,
                        options.safetyMargin,
                        options.distanceThreshold,
                        options.finiteDifferenceStep,
                        *statistics);
                    if(!linearization.sample.valid) {
                        addDiagnostic(diagnostics, "cdf_linearization_failed", linearization.sample.message);
                        allSamplesValid = false;
                        break;
                    }
                    currentPhi = std::min(currentPhi, linearization.sample.phi);
                    linearizations.push_back(std::move(linearization));
                }
                if(!allSamplesValid) {
                    return false;
                }

                QpSolveResult qpResult = solveTrajectoryWithOsqp(
                    path,
                    seedPath,
                    linearizations,
                    bounds,
                    options);
                if(!qpResult.success) {
                    addDiagnostic(
                        diagnostics,
                        "cdf_qp_solver_failed",
                        qpResult.message.empty()
                            ? "OSQP failed to solve the CDF/QP subproblem."
                            : qpResult.message);
                    return false;
                }

                statistics->iterations += 1;
                statistics->qpIterations += qpResult.solverIterations;
                statistics->maximumCorrection =
                    std::max(statistics->maximumCorrection, qpResult.maximumCorrection);
                statistics->maximumSlack =
                    std::max(statistics->maximumSlack, qpResult.maximumSlack);

                const int currentInvalidSegments = countInvalidSegments(path);
                std::vector<std::vector<double>> candidatePath = qpResult.path;
                double candidatePhi = evaluatePathMinimum(candidatePath);
                int candidateInvalidSegments = countInvalidSegments(candidatePath);

                if(candidateInvalidSegments == 0 &&
                    (currentInvalidSegments > 0 || candidatePhi >= currentPhi - 1.0e-9))
                {
                    path = std::move(candidatePath);
                    currentPhi = candidatePhi;
                } else {
                    double alpha = 0.5;
                    for(int backtrack = 0; backtrack < 5; ++backtrack) {
                        std::vector<std::vector<double>> blended = blendPath(path, qpResult.path, alpha);
                        const double blendedPhi = evaluatePathMinimum(blended);
                        const int blendedInvalidSegments = countInvalidSegments(blended);
                        if(blendedInvalidSegments == 0 &&
                            (currentInvalidSegments > 0 || blendedPhi >= currentPhi - 1.0e-9))
                        {
                            path = std::move(blended);
                            currentPhi = blendedPhi;
                            break;
                        }
                        alpha *= 0.5;
                    }
                }

                if(currentPhi >= targetPhi &&
                    qpResult.maximumCorrection < 1.0e-5 &&
                    qpResult.maximumSlack < 1.0e-5)
                {
                    break;
                }
            }

            *repairedPath = std::move(path);
            return countInvalidSegments(*repairedPath) == 0;
        }
    }

    bool ProjectCdfQpTrajectoryRepairService::ensureCollisionSetup(
        simulation_project::ProjectDocument& document,
        const std::string& robotId,
        const ProjectCdfQpRepairOptions& options,
        std::string* detectorId,
        std::vector<MotionPlanningDiagnostic>* diagnostics)
    {
        if(robotId.empty()) {
            addDiagnostic(diagnostics, "cdf_missing_robot_id", "Planning robot id is empty.");
            return false;
        }
        if(findRobot(document, robotId) == nullptr) {
            addDiagnostic(diagnostics, "cdf_robot_missing", "Planning robot is not present in the project: " + robotId);
            return false;
        }

        document.collision.query.enabled = true;

        const simulation_project::RobotDesc* obstacleRobot =
            findBurnnerRobot(document, options, robotId);
        const simulation_project::SceneObjectDesc* obstacleObject =
            obstacleRobot == nullptr ? findBurnnerObject(document, options) : nullptr;

        if(obstacleRobot == nullptr && obstacleObject == nullptr) {
            simulation_project::RobotDesc burnner;
            burnner.id = options.obstacleId;
            burnner.name = options.obstacleName.empty() ? options.obstacleId : options.obstacleName;
            burnner.sourceType = "urdf";
            burnner.sourcePath = options.obstacleRobotSourcePath;
            burnner.collisionEnabled = true;
            burnner.visible = true;
            burnner.baseTransform.x = options.obstacleX;
            burnner.baseTransform.y = options.obstacleY;
            burnner.baseTransform.z = options.obstacleZ;
            burnner.baseTransform.roll = options.obstacleRoll;
            burnner.baseTransform.pitch = options.obstaclePitch;
            burnner.baseTransform.yaw = options.obstacleYaw;
            document.robots.push_back(std::move(burnner));
            obstacleRobot = &document.robots.back();
            addDiagnostic(diagnostics, "cdf_burnner_added", "Added burnner as a static URDF robot obstacle for CDF/QP planning.");
        }

        const std::string selectedDetectorId = options.detectorId.empty()
            ? std::string("cdf_abb4600_burnner_detector")
            : options.detectorId;

        simulation_project::CollisionDetectorDesc detector =
            obstacleRobot != nullptr
                ? makeRobotRobotDetector(selectedDetectorId, robotId, obstacleRobot->id, options)
                : makeRobotObjectDetector(selectedDetectorId, robotId, obstacleObject->id, options);

        const auto detectorIt = std::find_if(
            document.collision.detectors.begin(),
            document.collision.detectors.end(),
            [&](const simulation_project::CollisionDetectorDesc& current) {
                return current.id == selectedDetectorId;
            });
        if(detectorIt == document.collision.detectors.end()) {
            document.collision.detectors.push_back(std::move(detector));
        } else {
            *detectorIt = std::move(detector);
        }

        if(detectorId != nullptr) {
            *detectorId = selectedDetectorId;
        }
        if(hasDetector(document, selectedDetectorId)) {
            return true;
        }

        addDiagnostic(diagnostics, "cdf_detector_missing", "Failed to create the CDF collision detector.");
        return false;
    }

    ProjectCdfQpRepairResult ProjectCdfQpTrajectoryRepairService::repair(
        const simulation_project::ProjectDocument& document,
        const std::filesystem::path& projectBasePath,
        const std::string& robotId,
        const std::vector<std::string>& jointNames,
        const robottrajectory::JointTrajectory& seedTrajectory,
        const ProjectCdfQpRepairOptions& options) const
    {
        ProjectCdfQpRepairResult result;
        result.statistics.inputWaypointCount = static_cast<int>(seedTrajectory.points.size());

        if(seedTrajectory.points.size() < 2) {
            addDiagnostic(&result.diagnostics, "cdf_seed_too_short", "CDF/QP repair requires at least two trajectory points.");
            return result;
        }
        if(robotId.empty()) {
            addDiagnostic(&result.diagnostics, "cdf_missing_robot_id", "Select ABB4600_urdf before running CDF/QP repair.");
            return result;
        }
        if(jointNames.empty()) {
            addDiagnostic(&result.diagnostics, "cdf_missing_joints", "Joint names are required for CDF/QP repair.");
            return result;
        }

        for(std::size_t index = 0; index < seedTrajectory.points.size(); ++index) {
            if(seedTrajectory.points[index].q.size() != jointNames.size()) {
                std::ostringstream stream;
                stream << "Seed point " << (index + 1)
                       << " has " << seedTrajectory.points[index].q.size()
                       << " joints, expected " << jointNames.size() << ".";
                addDiagnostic(&result.diagnostics, "cdf_seed_joint_count_mismatch", stream.str());
                return result;
            }
        }

        const std::string detectorId = options.detectorId.empty()
            ? std::string("cdf_abb4600_burnner_detector")
            : options.detectorId;
        if(!hasDetector(document, detectorId)) {
            addDiagnostic(
                &result.diagnostics,
                "cdf_detector_missing",
                "CDF/QP collision detector is not configured in the project: " + detectorId);
            return result;
        }

        simulation_project::ProjectDocument planningDocument = document;
        planningDocument.collision.detectors.erase(
            std::remove_if(
                planningDocument.collision.detectors.begin(),
                planningDocument.collision.detectors.end(),
                [&](const simulation_project::CollisionDetectorDesc& detector) {
                    return detector.id != detectorId;
                }),
            planningDocument.collision.detectors.end());

        ProjectPlanningRequest request;
        request.robotId = robotId;
        request.jointNames = jointNames;
        request.start = seedTrajectory.points.front().q;
        request.goal = seedTrajectory.points.back().q;
        request.collisionDetectorIds = { detectorId };
        request.validation.maxJointStep = options.validationMaxJointStep;

        std::string sceneError;
        std::unique_ptr<ProjectPlanningSceneSnapshot> scene =
            ProjectPlanningSceneBuilder::build(
                planningDocument,
                projectBaseOrParent(projectBasePath),
                request,
                &sceneError);
        if(!scene) {
            addDiagnostic(&result.diagnostics, "cdf_scene_build_failed", sceneError);
            return result;
        }

        const std::vector<robottrajectory::TimedJointPoint> denseSeedTrajectory =
            densifyTrajectory(
                seedTrajectory,
                scene->jointBounds(),
                options.segmentIntermediateSamples,
                options.validationMaxJointStep);
        result.statistics.inputWaypointCount = static_cast<int>(denseSeedTrajectory.size());

        std::vector<std::vector<double>> path;
        path.reserve(denseSeedTrajectory.size());
        for(const robottrajectory::TimedJointPoint& point : denseSeedTrajectory) {
            path.push_back(clampToBounds(
                point.q,
                scene->jointBounds(),
                &result.statistics.clampedSeedValues));
        }
        const std::vector<std::vector<double>> originalReferencePath = path;

        auto evaluatePathMinimum = [&](const std::vector<std::vector<double>>& candidate) {
            double minimumPhi = std::numeric_limits<double>::max();
            for(const std::vector<double>& q : candidate) {
                const SignedDistanceSample sample =
                    evaluateSignedPhi(
                        *scene,
                        q,
                        options.safetyMargin,
                        options.distanceThreshold,
                        result.statistics);
                if(!sample.valid) {
                    addDiagnostic(&result.diagnostics, "cdf_distance_failed", sample.message);
                    return -std::numeric_limits<double>::max();
                }
                minimumPhi = std::min(minimumPhi, sample.phi);
            }
            return minimumPhi;
        };

        auto countInvalidSegments = [&](const std::vector<std::vector<double>>& candidate) {
            int invalidCount = 0;
            for(std::size_t index = 0; index + 1 < candidate.size(); ++index) {
                const StateValidationResult validation =
                    scene->validateMotion(candidate[index], candidate[index + 1], request.validation);
                if(!validation.valid) {
                    ++invalidCount;
                }
            }
            return invalidCount;
        };

        auto blendPath = [&](const std::vector<std::vector<double>>& from,
                             const std::vector<std::vector<double>>& to,
                             double alpha) {
            std::vector<std::vector<double>> blended = from;
            const std::size_t waypointCount = std::min(from.size(), to.size());
            for(std::size_t waypoint = 0; waypoint < waypointCount; ++waypoint) {
                const std::size_t jointCount = std::min(from[waypoint].size(), to[waypoint].size());
                blended[waypoint].resize(jointCount);
                for(std::size_t joint = 0; joint < jointCount; ++joint) {
                    double delta = to[waypoint][joint] - from[waypoint][joint];
                    if(joint < scene->jointBounds().size() && scene->jointBounds()[joint].continuous) {
                        delta = std::remainder(delta, kTwoPi);
                    }
                    blended[waypoint][joint] = from[waypoint][joint] + delta * alpha;
                    if(joint < scene->jointBounds().size() && scene->jointBounds()[joint].continuous) {
                        blended[waypoint][joint] = wrapContinuousAngle(blended[waypoint][joint]);
                    }
                }
                blended[waypoint] = clampToBounds(blended[waypoint], scene->jointBounds(), nullptr);
            }
            return blended;
        };

        result.statistics.initialMinimumPhi = evaluatePathMinimum(path);
        if(result.statistics.initialMinimumPhi <= -std::numeric_limits<double>::max() * 0.25) {
            return result;
        }

        const double targetPhi = options.targetClearance;
        const int maxIterations = std::max(1, options.maxIterations);
        std::vector<std::vector<double>> seedPath = path;

        if(countInvalidSegments(path) != 0) {
            const bool omplRepaired = repairCollisionRunsWithOmpl(
                *scene,
                scene->jointBounds(),
                request.validation,
                options,
                &result.statistics,
                &result.diagnostics,
                &path);
            if(omplRepaired) {
                seedPath = path;
                result.statistics.finalMinimumPhi = evaluatePathMinimum(path);
                if(result.statistics.finalMinimumPhi <= -std::numeric_limits<double>::max() * 0.25) {
                    return result;
                }
            }
        }

        // A CDF linearization is not meaningful for a configuration that is
        // already deeply inside geometry. OMPL is responsible for leaving such
        // regions first; only a collision-free path is eligible for global QP
        // smoothing. Remaining invalid windows are handled by the local repair
        // stage below, where every candidate is hard-validated before write-back.
        const bool pathWasInitiallyCollisionFree = countInvalidSegments(path) == 0;
        if(!pathWasInitiallyCollisionFree) {
            addDiagnostic(
                &result.diagnostics,
                "cdf_qp_deferred_until_collision_free",
                "Global CDF/QP smoothing was deferred because the seed still contains collision segments after OMPL pre-repair.");
        }

        for(int iteration = 0; pathWasInitiallyCollisionFree && iteration < maxIterations; ++iteration) {
            std::vector<CdfLinearization> linearizations;
            linearizations.reserve(path.size());

            double minimumPhi = std::numeric_limits<double>::max();
            bool allSamplesValid = true;
            for(const std::vector<double>& q : path) {
                CdfLinearization linearization = linearizeSignedPhi(
                    *scene,
                    q,
                    scene->jointBounds(),
                    options.safetyMargin,
                    options.distanceThreshold,
                    options.finiteDifferenceStep,
                    result.statistics);
                if(!linearization.sample.valid) {
                    addDiagnostic(&result.diagnostics, "cdf_linearization_failed", linearization.sample.message);
                    allSamplesValid = false;
                    break;
                }
                minimumPhi = std::min(minimumPhi, linearization.sample.phi);
                linearizations.push_back(std::move(linearization));
            }
            if(!allSamplesValid) {
                return result;
            }

            result.statistics.finalMinimumPhi = minimumPhi;
            QpSolveResult qpResult = solveTrajectoryWithOsqp(
                path,
                seedPath,
                linearizations,
                scene->jointBounds(),
                options);
            if(!qpResult.success) {
                addDiagnostic(
                    &result.diagnostics,
                    "cdf_qp_solver_failed",
                    qpResult.message.empty()
                        ? "OSQP failed to solve the CDF/QP subproblem."
                        : qpResult.message);
                return result;
            }

            result.statistics.iterations = iteration + 1;
            result.statistics.qpIterations += qpResult.solverIterations;
            result.statistics.maximumCorrection =
                std::max(result.statistics.maximumCorrection, qpResult.maximumCorrection);
            result.statistics.maximumSlack =
                std::max(result.statistics.maximumSlack, qpResult.maximumSlack);

            const int currentInvalidSegments = countInvalidSegments(path);
            const double currentPhi = minimumPhi;
            std::vector<std::vector<double>> candidatePath = qpResult.path;
            double candidatePhi = evaluatePathMinimum(candidatePath);
            int candidateInvalidSegments = countInvalidSegments(candidatePath);

            if(candidateInvalidSegments == 0 &&
                (currentInvalidSegments > 0 || candidatePhi >= currentPhi - 1.0e-9))
            {
                path = std::move(candidatePath);
                result.statistics.finalMinimumPhi = candidatePhi;
            } else {
                double alpha = 0.5;
                for(int backtrack = 0; backtrack < 5; ++backtrack) {
                    std::vector<std::vector<double>> candidate = blendPath(path, qpResult.path, alpha);
                    const double blendedPhi = evaluatePathMinimum(candidate);
                    const int blendedInvalidSegments = countInvalidSegments(candidate);
                    if(blendedInvalidSegments == 0 &&
                        (currentInvalidSegments > 0 || blendedPhi >= currentPhi - 1.0e-9))
                    {
                        path = std::move(candidate);
                        result.statistics.finalMinimumPhi = blendedPhi;
                        break;
                    }
                    alpha *= 0.5;
                }
            }

            if(qpResult.maximumCorrection < 1.0e-5 &&
                qpResult.maximumSlack < 1.0e-5 &&
                minimumPhi >= targetPhi) {
                break;
            }
        }

        auto slicePath = [&](const std::vector<std::vector<double>>& source,
                             std::size_t beginIndex,
                             std::size_t endIndex) {
            std::vector<std::vector<double>> slice;
            if(beginIndex >= source.size() || endIndex >= source.size() || beginIndex > endIndex) {
                return slice;
            }
            slice.reserve(endIndex - beginIndex + 1);
            for(std::size_t index = beginIndex; index <= endIndex; ++index) {
                slice.push_back(source[index]);
            }
            return slice;
        };

        auto repairInvalidWindows = [&]() -> bool {
            ProjectCdfQpRepairOptions localOptions = options;
            localOptions.trustRegion = std::max(0.0025, std::min(options.trustRegion, 0.006));
            localOptions.seedCorridor = std::max(0.012, std::min(options.seedCorridor, 0.028));
            localOptions.repairGain = std::max(0.06, std::min(options.repairGain, 0.10));
            localOptions.seedTrackingWeight = std::max(options.seedTrackingWeight, 0.75);
            localOptions.smoothWeight = std::max(options.smoothWeight, 0.18);
            localOptions.targetClearance = std::max(options.targetClearance, 0.001);
            localOptions.maxIterations = 2;
            localOptions.keepEndpoints = true;

            MotionValidationOptions localValidation = request.validation;
            if(localValidation.maxJointStep > 0.0 && std::isfinite(localValidation.maxJointStep)) {
                localValidation.maxJointStep = std::min(localValidation.maxJointStep, 0.008);
            }

            const std::size_t primaryWindowPoints = 18;
            const std::size_t secondaryWindowPoints = 12;
            const std::size_t tertiaryWindowPoints = 8;
            const std::size_t primaryOverlap = 2;
            const std::size_t secondaryOverlap = 2;
            const std::size_t tertiaryOverlap = 1;

            auto sortWindowsByRisk = [](std::vector<RepairWindow>& windows) {
                std::sort(
                    windows.begin(),
                    windows.end(),
                    [](const RepairWindow& lhs, const RepairWindow& rhs) {
                        const std::size_t lhsLength = lhs.endIndex >= lhs.beginIndex
                            ? (lhs.endIndex - lhs.beginIndex + 1)
                            : 0;
                        const std::size_t rhsLength = rhs.endIndex >= rhs.beginIndex
                            ? (rhs.endIndex - rhs.beginIndex + 1)
                            : 0;
                        if(lhsLength != rhsLength) {
                            return lhsLength > rhsLength;
                        }
                        return lhs.beginIndex < rhs.beginIndex;
                    });
            };

            auto describeRun = [](std::size_t ordinal, const InvalidSegmentRun& run) {
                std::ostringstream stream;
                stream << "window #" << ordinal
                       << " [segment " << (run.beginSegment + 1)
                       << " -> " << (run.endSegment + 2) << "]";
                return stream.str();
            };

            auto repairWindowSet = [&](const std::vector<RepairWindow>& windows,
                                       std::size_t extraPadding,
                                       const char* stageTag,
                                       std::vector<std::string>* failedWindows) -> bool {
                bool updatedAnyWindow = false;
                for(std::size_t ordinal = 0; ordinal < windows.size(); ++ordinal) {
                    const RepairWindow& window = windows[ordinal];
                    if(window.beginIndex >= window.endIndex || window.endIndex >= path.size()) {
                        continue;
                    }

                    struct WindowRepairResult
                    {
                        std::size_t beginIndex = 0;
                        std::vector<std::vector<double>> path;
                    };

                    auto attemptWindowRepair = [&](std::size_t attemptPadding) {
                        WindowRepairResult resultWindow;
                        const std::size_t beginIndex =
                            window.beginIndex > attemptPadding ? window.beginIndex - attemptPadding : 0;
                        const std::size_t endIndex = std::min(
                            path.size() - 1,
                            window.endIndex + attemptPadding);
                        if(beginIndex >= endIndex) {
                            return resultWindow;
                        }

                        std::vector<std::vector<double>> windowPath =
                            slicePath(path, beginIndex, endIndex);
                        std::vector<std::vector<double>> windowSeed =
                            slicePath(seedPath, beginIndex, endIndex);
                        if(windowPath.size() < 2) {
                            return resultWindow;
                        }
                        if(windowSeed.size() != windowPath.size()) {
                            windowSeed = windowPath;
                        }

                        // Never linearize CDF inside a penetrating configuration.
                        // First give this window its own OMPL chance. QP is only a
                        // local smoother after the complete window is collision-free.
                        const bool omplWindowChanged = repairCollisionRunsWithOmpl(
                            *scene,
                            scene->jointBounds(),
                            localValidation,
                            localOptions,
                            &result.statistics,
                            &result.diagnostics,
                            &windowPath);
                        if(omplWindowChanged) {
                            windowSeed = windowPath;
                        }
                        if(countInvalidSegments(windowPath) != 0) {
                            return resultWindow;
                        }

                        std::vector<std::vector<double>> repairedWindow;
                        const bool windowSolved = runCdfQpRepairIterations(
                            *scene,
                            windowPath,
                            windowSeed,
                            scene->jointBounds(),
                            localValidation,
                            localOptions,
                            &result.statistics,
                            &result.diagnostics,
                            &repairedWindow);
                        if(!windowSolved || repairedWindow.size() != windowPath.size()) {
                            return resultWindow;
                        }
                        if(countInvalidSegments(repairedWindow) != 0) {
                            return resultWindow;
                        }
                        resultWindow.beginIndex = beginIndex;
                        resultWindow.path = std::move(repairedWindow);
                        return resultWindow;
                    };

                    WindowRepairResult repairedWindow = attemptWindowRepair(0);
                    if(repairedWindow.path.empty()) {
                        repairedWindow = attemptWindowRepair(extraPadding);
                    }
                    if(repairedWindow.path.empty()) {
                        if(failedWindows != nullptr) {
                            std::ostringstream stream;
                            stream << stageTag << " " << describeRun(ordinal + 1, InvalidSegmentRun{
                                window.beginIndex,
                                window.endIndex
                            }) << " still invalid after local repair.";
                            failedWindows->push_back(stream.str());
                        }
                        continue;
                    }

                    if(repairedWindow.beginIndex + repairedWindow.path.size() > path.size()) {
                        if(failedWindows != nullptr) {
                            std::ostringstream stream;
                            stream << stageTag << " " << describeRun(ordinal + 1, InvalidSegmentRun{
                                window.beginIndex,
                                window.endIndex
                            }) << " could not be written back safely.";
                            failedWindows->push_back(stream.str());
                        }
                        continue;
                    }
                    for(std::size_t index = 0; index < repairedWindow.path.size(); ++index) {
                        path[repairedWindow.beginIndex + index] = std::move(repairedWindow.path[index]);
                    }
                    updatedAnyWindow = true;
                }
                return updatedAnyWindow;
            };

            const std::vector<InvalidSegmentRun> initialRuns =
                collectInvalidSegmentRuns(*scene, path, localValidation);
            if(initialRuns.empty()) {
                return true;
            }

            std::vector<RepairWindow> primaryWindows = buildRepairWindows(
                initialRuns,
                path.size(),
                2,
                primaryWindowPoints,
                primaryOverlap,
                8);
            sortWindowsByRisk(primaryWindows);
            std::vector<std::string> failedWindows;
            repairWindowSet(primaryWindows, 4, "primary", &failedWindows);

            std::vector<InvalidSegmentRun> remainingRuns =
                collectInvalidSegmentRuns(*scene, path, localValidation);
            if(remainingRuns.empty()) {
                return true;
            }

            std::vector<RepairWindow> secondaryWindows = buildRepairWindows(
                remainingRuns,
                path.size(),
                5,
                secondaryWindowPoints,
                secondaryOverlap,
                16);
            sortWindowsByRisk(secondaryWindows);
            repairWindowSet(secondaryWindows, 8, "secondary", &failedWindows);

            remainingRuns = collectInvalidSegmentRuns(*scene, path, localValidation);
            if(!remainingRuns.empty()) {
                ProjectCdfQpRepairOptions tertiaryOptions = localOptions;
                tertiaryOptions.trustRegion = std::max(0.0015, std::min(localOptions.trustRegion, 0.004));
                tertiaryOptions.seedCorridor = std::max(0.008, std::min(localOptions.seedCorridor, 0.018));
                tertiaryOptions.repairGain = std::max(0.04, std::min(localOptions.repairGain, 0.07));
                tertiaryOptions.seedTrackingWeight = std::max(localOptions.seedTrackingWeight, 0.90);
                tertiaryOptions.smoothWeight = std::max(localOptions.smoothWeight, 0.24);
                tertiaryOptions.maxIterations = 3;

                const ProjectCdfQpRepairOptions savedLocalOptions = localOptions;
                localOptions = tertiaryOptions;
                std::vector<RepairWindow> tertiaryWindows = buildRepairWindows(
                    remainingRuns,
                    path.size(),
                    8,
                    tertiaryWindowPoints,
                    tertiaryOverlap,
                    20);
                sortWindowsByRisk(tertiaryWindows);
                repairWindowSet(tertiaryWindows, 10, "tertiary", &failedWindows);
                localOptions = savedLocalOptions;
                remainingRuns = collectInvalidSegmentRuns(*scene, path, localValidation);
            }

            if(!remainingRuns.empty()) {
                for(std::size_t ordinal = 0; ordinal < remainingRuns.size(); ++ordinal) {
                    std::ostringstream stream;
                    stream << "Remaining collision interval #" << (ordinal + 1)
                           << " spans segment " << (remainingRuns[ordinal].beginSegment + 1)
                           << " -> " << (remainingRuns[ordinal].endSegment + 2)
                           << " after primary, secondary, and tertiary CDF/QP repair.";
                    addDiagnostic(&result.diagnostics, "cdf_repair_window_failed", stream.str());
                }
                for(const std::string& message : failedWindows) {
                    addDiagnostic(&result.diagnostics, "cdf_repair_window_failed", message);
                }
                return false;
            }

            for(const std::string& message : failedWindows) {
                addDiagnostic(&result.diagnostics, "cdf_repair_window_partial", message);
            }
            return true;
        };

        result.statistics.finalMinimumPhi = evaluatePathMinimum(path);
        if(result.statistics.finalMinimumPhi <= -std::numeric_limits<double>::max() * 0.25) {
            return result;
        }

        if(countInvalidSegments(path) != 0 && !repairInvalidWindows()) {
            addDiagnostic(
                &result.diagnostics,
                "cdf_repair_segment_window_invalid",
                "High-risk CDF/QP windows still contain collision segments after local repair.");
        }

        if(countInvalidSegments(path) == 0) {
            const bool smoothed = smoothCollisionFreePath(
                *scene,
                scene->jointBounds(),
                request.validation,
                originalReferencePath,
                options.postSmoothingIterations,
                options.postSmoothingStep,
                options.postSmoothingSeedWeight,
                options.keepEndpoints,
                &path);
            if(smoothed) {
                addDiagnostic(
                    &result.diagnostics,
                    "cdf_collision_constrained_smoothing",
                    "Applied conservative collision-validated smoothing while tracking the imported trajectory.");
            }
        }

        result.statistics.finalMinimumPhi = evaluatePathMinimum(path);
        if(result.statistics.finalMinimumPhi <= -std::numeric_limits<double>::max() * 0.25) {
            return result;
        }

        result.statistics.invalidSegmentCount = 0;
        for(std::size_t index = 0; index + 1 < path.size(); ++index) {
            const StateValidationResult validation =
                scene->validateMotion(path[index], path[index + 1], request.validation);
            if(!validation.valid) {
                ++result.statistics.invalidSegmentCount;
                addDiagnostic(&result.diagnostics, "cdf_repaired_segment_invalid", segmentFailureMessage(index, validation));
            }
        }

        result.plan.id = robotId + "_cdf_qp_repaired";
        result.plan.name = "CDF/QP repaired trajectory";
        result.plan.robotId = robotId;
        result.plan.jointNames = jointNames;
        result.plan.trajectory.name = result.plan.id;
        result.plan.trajectory.interpolation = robottrajectory::TrajectoryInterpolation::Linear;
        result.plan.trajectory.points.reserve(denseSeedTrajectory.size());
        for(std::size_t index = 0; index < denseSeedTrajectory.size(); ++index) {
            robottrajectory::TimedJointPoint point = denseSeedTrajectory[index];
            point.q = path[index];
            point.qd.clear();
            point.qdd.clear();
            result.plan.trajectory.points.push_back(std::move(point));
        }

        result.success =
            result.statistics.finalMinimumPhi >= targetPhi &&
            result.statistics.invalidSegmentCount == 0;
        if(!result.success && result.diagnostics.empty()) {
            addDiagnostic(
                &result.diagnostics,
                "cdf_repair_not_converged",
                "CDF/QP repair finished but did not reach the requested clearance.");
        }
        return result;
    }
}
