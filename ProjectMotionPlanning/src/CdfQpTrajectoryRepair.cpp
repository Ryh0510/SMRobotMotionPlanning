#include <ProjectMotionPlanning/CdfQpTrajectoryRepair.h>

#include <ProjectMotionPlanning/ProjectMotionPlanning.h>
#include <ProjectMotionPlanning/TrajectoryInverseKinematics.h>
#include <Eigen/SVD>
#include "ApfLocalPlanner.h"
#include "PathRefinement.h"
#include "CdfDistanceField.h"
#include "CdfQueryBatch.h"
#include <SimulationProject/ProjectDocument.h>
#include <osqp.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
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
            double maxJointStep,
            std::vector<std::pair<std::size_t, double>>* correspondence)
        {
            if(correspondence) correspondence->clear();
            std::vector<robottrajectory::TimedJointPoint> densePoints;
            if(trajectory.points.empty()) {
                return densePoints;
            }

            const int minimumSamples = std::max(0, minimumIntermediateSamples);
            const double effectiveJointStep = std::isfinite(maxJointStep) && maxJointStep > 0.0
                ? maxJointStep
                : 0.0;
            densePoints.push_back(trajectory.points.front());
            if(correspondence) correspondence->push_back({0, 0.0});

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
                    if(correspondence) correspondence->push_back({index, fraction});
                }
                densePoints.push_back(end);
                if(correspondence) correspondence->push_back({index, 1.0});
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

        std::vector<InvalidSegmentRun> collectInvalidSegmentRuns(
            detail::CdfQueryBatch& queries,
            const std::vector<std::vector<double>>& path,
            const MotionValidationOptions& validation,
            const detail::ApfGuidance* guidance = nullptr)
        {
            const auto states = queries.motions(path, validation);
            std::vector<InvalidSegmentRun> runs;
            for(std::size_t i = 0; i < states.size(); ++i) {
                const auto valid = [&](std::size_t k) {
                    return states[k].valid && (!guidance || detail::followsApfGuide(path[k], path[k+1],
                        guidance->positions[k], guidance->positions[k+1], *guidance));
                };
                if(valid(i)) continue;
                const std::size_t begin = i;
                while(i + 1 < states.size() && !valid(i + 1)) ++i;
                runs.push_back({begin, i});
            }
            return runs;
        }

        bool locateSafeApfAnchors(
            ProjectPlanningSceneSnapshot& scene,
            const std::vector<std::vector<double>>& path,
            const InvalidSegmentRun& run,
            std::size_t* beginIndex,
            std::size_t* endIndex,
            const detail::ApfGuidance* guidance)
        {
            if(beginIndex == nullptr || endIndex == nullptr || path.size() < 2) {
                return false;
            }

            const auto safe = [&](std::size_t i) {
                return scene.validateState(path[i]).valid && (!guidance || detail::followsApfGuide(path[i], path[i],
                    guidance->positions[i], guidance->positions[i], *guidance));
            };
            std::size_t left = run.beginSegment;
            while(true) {
                if(safe(left)) {
                    break;
                }
                if(left == 0) {
                    return false;
                }
                --left;
            }

            std::size_t right = run.endSegment + 1;
            while(right < path.size() && !safe(right)) {
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


    }

    namespace detail
    {
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

                collision::CollisionResult distanceResult;
                if(!collisionResult->inCollision()) {
                    const auto& detectors = scene.collisionRuntime().detectors();
                    const auto detector = std::find_if(detectors.begin(), detectors.end(),
                        [&](const auto& value) { return value.id == detectorId; });
                    if(detector == detectors.end() || !detector->valid || !detector->enabled) {
                        sample.message = "Distance detector is unavailable: " + detectorId;
                        return sample;
                    }
                    auto query = detector->options;
                    query.enableDistance = true;
                    query.enableNearestPoints = true;
                    query.distanceThreshold = distanceThreshold;
                    ++statistics.collisionQueries;
                    // The runtime owns a mutable scene but exposes only a const
                    // accessor. distance() updates query caches, not geometry.
                    auto& collisionScene = const_cast<collision::CollisionScene&>(
                        scene.collisionRuntime().scene());
                    collisionScene.distance(query, distanceResult);
                    if(distanceResult.status != collision::CollisionStatus::Success) {
                        sample.message = "Distance query failed: " + distanceResult.message;
                        return sample;
                    }
                    collisionResult = &distanceResult;
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
                    const double signedDistance = collisionResult->minDistance;
                    collision::Vec3 direction = collision::Vec3::Zero();
                    const bool directionValid = extractNearestDirection(*collisionResult, &direction);
                    sawFiniteDistance = true;
                    if(signedDistance - safetyMargin < minimumPhi) {
                        minimumPhi = signedDistance - safetyMargin;
                        minimumDistance = signedDistance;
                        bestDirection = direction;
                        bestDirectionValid = directionValid;
                        sample.hasNearestFeature = false;
                        collision::Transform3 transformA, transformB;
                        const auto& geometry = scene.collisionRuntime().scene();
                        if(directionValid && collisionResult->hasNearestPoints && signedDistance > 1.0e-8 &&
                            std::abs((collisionResult->nearestPointB - collisionResult->nearestPointA).norm()
                                - signedDistance) <= 1.0e-6 * std::max(1.0, signedDistance) &&
                            geometry.objectTransform(collisionResult->nearestObjectA, transformA) &&
                            geometry.objectTransform(collisionResult->nearestObjectB, transformB)) {
                            sample.objectA = collisionResult->nearestObjectA;
                            sample.objectB = collisionResult->nearestObjectB;
                            sample.localPointA = transformA.inverse() * collisionResult->nearestPointA;
                            sample.localPointB = transformB.inverse() * collisionResult->nearestPointB;
                            sample.hasNearestFeature = true;
                        }
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

            // The derivative of the minimum separation is the relative velocity
            // of its nearest features projected onto their separating normal.
            // Differentiate their transforms instead of repeating mesh searches
            // for every joint. The next QP step still uses exact distance and
            // collision validation; unsupported nearest-point results fall back
            // to the original central distance differences below.
            if(linearization.sample.hasNearestFeature && !linearization.sample.inCollision) {
                bool usable = true;
                auto featureSeparation = [&](const std::vector<double>& state, double* projected) {
                    if(!scene.setState(state)) return false;
                    collision::Transform3 a, b;
                    const auto& geometry = scene.collisionRuntime().scene();
                    if(!geometry.objectTransform(linearization.sample.objectA, a) ||
                        !geometry.objectTransform(linearization.sample.objectB, b)) return false;
                    *projected = linearization.sample.nearestDirection.dot(
                        b * linearization.sample.localPointB - a * linearization.sample.localPointA);
                    return std::isfinite(*projected);
                };
                for(std::size_t j = 0; j < q.size(); ++j) {
                    auto plus = q, minus = q;
                    plus[j] += step;
                    minus[j] -= step;
                    if(j < bounds.size() && !bounds[j].continuous) {
                        plus[j] = std::clamp(plus[j], bounds[j].lower, bounds[j].upper);
                        minus[j] = std::clamp(minus[j], bounds[j].lower, bounds[j].upper);
                    }
                    if(plus[j] - minus[j] <= kTiny) continue;
                    double plusValue = 0.0, minusValue = 0.0;
                    if(!featureSeparation(plus, &plusValue) || !featureSeparation(minus, &minusValue)) {
                        usable = false;
                        break;
                    }
                    linearization.gradient[j] = (plusValue - minusValue) / (plus[j] - minus[j]);
                }
                if(usable) return linearization;
                linearization.gradient.assign(q.size(), 0.0);
            }

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

    }

    namespace
    {
        using detail::SignedDistanceSample;
        using detail::CdfLinearization;
        using detail::evaluateSignedPhi;
        using detail::linearizeSignedPhi;

        bool repairCollisionRunsWithApf(
            ProjectPlanningSceneSnapshot& scene,
            detail::CdfQueryBatch& queries,
            const std::vector<JointBound>& bounds,
            const MotionValidationOptions& validation,
            const ProjectCdfQpRepairOptions& options,
            ProjectCdfQpRepairStatistics* statistics,
            std::vector<MotionPlanningDiagnostic>* diagnostics,
            std::vector<std::vector<double>>* path,
            const detail::ApfGuidance* guidance)
        {
            if(path == nullptr || path->size() < 2 || statistics == nullptr) return false;
            const auto runs = collectInvalidSegmentRuns(queries, *path, validation, guidance);
            if(options.progress) options.progress("APF collision/shape intervals: " + std::to_string(runs.size()));
            bool changedAny = false;
            for(const auto& run : runs) {
                if(options.progress) options.progress("APF interval " + std::to_string(run.beginSegment + 1)
                    + " -> " + std::to_string(run.endSegment + 2));
                bool stillInvalid = false;
                for(std::size_t i = run.beginSegment; i <= run.endSegment; ++i)
                    if(!scene.validateMotion((*path)[i], (*path)[i + 1], validation).valid ||
                        (guidance && !detail::followsApfGuide((*path)[i], (*path)[i+1],
                            guidance->positions[i], guidance->positions[i+1], *guidance))) {
                        stillInvalid = true;
                        break;
                    }
                if(!stillInvalid) continue;
                std::size_t left = 0, right = 0;
                if(!locateSafeApfAnchors(scene, *path, run, &left, &right, guidance)) {
                    addDiagnostic(diagnostics, "apf_anchor_missing",
                        "No safe anchors on both sides of interval " + std::to_string(run.beginSegment + 1)
                        + " -> " + std::to_string(run.endSegment + 2)
                        + "; an unsafe trajectory endpoint cannot be silently moved.");
                    return changedAny;
                }
                const auto safe = [&](std::size_t i) {
                    return scene.validateState((*path)[i]).valid && (!guidance || detail::followsApfGuide((*path)[i], (*path)[i],
                        guidance->positions[i], guidance->positions[i], *guidance));
                };
                bool repaired = false;
                // Expand to nearby safe anchors when the initial anchors are too
                // close to contact or there are too few samples for a detour.
                for(int expansion = 0; expansion < 3 && !repaired; ++expansion) {
                    if(options.progress) options.progress("APF anchor expansion " + std::to_string(expansion + 1) + " / 3");
                    const std::size_t padding = expansion == 0 ? 12 : (expansion == 1 ? 40 : 120);
                    std::size_t begin = left > padding ? left - padding : 0;
                    std::size_t end = std::min(path->size() - 1, right + padding);
                    if(guidance) {
                        const double anticipation = expansion == 0 ? 0.06 : (expansion == 1 ? 0.12 : 0.24);
                        const auto span = [&](std::size_t a, std::size_t b) {
                            double length=0.0;
                            for(std::size_t i=a+1;i<=b;++i) {
                                const auto& p=guidance->positions[i-1];const auto& q=guidance->positions[i];
                                double squared=0.0;for(std::size_t j=0;j<p.size();++j)squared+=(q[j]-p[j])*(q[j]-p[j]);
                                length+=std::sqrt(squared);
                            }
                            return length;
                        };
                        double before=span(begin,left),after=span(right,end);
                        while(begin>0 && before<anticipation){before+=span(begin-1,begin);--begin;}
                        while(end+1<path->size() && after<anticipation){after+=span(end,end+1);++end;}
                    }
                    // Find an earlier/later safe anchor, never shrink the
                    // anticipation window back toward the obstructed interval.
                    while(begin > 0 && !safe(begin)) --begin;
                    while(end + 1 < path->size() && !safe(end)) ++end;
                    if(!safe(begin) || !safe(end))continue;
                    detail::ApfPath reference(path->begin() + begin, path->begin() + end + 1);
                    reference = unwrapContinuousPath(reference, bounds);
                    detail::ApfState lower = reference.front(), upper = lower;
                    const double corridor = expansion == 0 ? 0.25 : (expansion == 1 ? 0.45 : 0.75);
                    for(std::size_t j = 0; j < bounds.size(); ++j) {
                        for(const auto& q : reference) {
                            lower[j] = std::min(lower[j], q[j]);
                            upper[j] = std::max(upper[j], q[j]);
                        }
                        lower[j] -= corridor;
                        upper[j] += corridor;
                        if(!bounds[j].continuous) {
                            lower[j] = expansion==2 ? bounds[j].lower : std::max(lower[j], bounds[j].lower);
                            upper[j] = expansion==2 ? bounds[j].upper : std::min(upper[j], bounds[j].upper);
                        }
                    }
                    detail::ApfOracle oracle;
                    oracle.progress = [&](int attempt, int iteration) {
                        if(options.progress) options.progress("APF field " + std::to_string(attempt)
                            + ", local iterations " + std::to_string(iteration));
                    };
                    oracle.failedStation = [&](int field, std::size_t station, const auto& q) {
                        if(!options.progress) return;
                        std::ostringstream trace;
                        trace << "APF stalled: field=" << field << ", reference node=" << begin+station+1 << ", q(rad)=";
                        for(double angle:q) trace << angle << ' ';
                        if(guidance) {
                            const auto p = guidance->position(q);
                            if(p.size()==3) trace << ", TCP reference error(mm)=" << std::sqrt(
                                std::pow(p[0]-guidance->positions[begin+station][0],2)+
                                std::pow(p[1]-guidance->positions[begin+station][1],2)+
                                std::pow(p[2]-guidance->positions[begin+station][2],2))*1000.0;
                        }
                        options.progress(trace.str());
                    };
                    oracle.distance = [&](const auto& q) {
                        const auto sample = queries.distances({q}, 0.0, options.distanceThreshold, *statistics).front();
                        return sample.valid ? sample.rawDistance : std::numeric_limits<double>::quiet_NaN();
                    };
                    oracle.distances = [&](const auto& points) {
                        const auto samples = queries.distances(points, 0.0, options.distanceThreshold, *statistics);
                        std::vector<double> values;
                        values.reserve(samples.size());
                        for(const auto& sample : samples) values.push_back(sample.valid ? sample.rawDistance :
                            std::numeric_limits<double>::quiet_NaN());
                        return values;
                    };
                    oracle.motionValid = [&](const auto& a, const auto& b) {
                        return queries.pathValid({a, b}, validation);
                    };
                    detail::ApfGuidance localGuide;
                    if(guidance) {
                        localGuide = *guidance;
                        localGuide.positions.assign(guidance->positions.begin()+begin, guidance->positions.begin()+end+1);
                    }
                    detail::ApfPath replacement;
                    if(!detail::planApfPath(reference, lower, upper, oracle,
                        std::max(0.005, options.safetyMargin + options.targetClearance), &replacement,
                        guidance ? &localGuide : nullptr)) continue;
                    if(replacement.size() != reference.size() ||
                        (guidance && !detail::followsApfPath(replacement, localGuide)) ||
                        !queries.pathValid(replacement, validation)) continue;
                    // Anchors are unchanged, so the two external seams are unchanged.
                    std::copy(replacement.begin(), replacement.end(), path->begin() + begin);
                    repaired = changedAny = true;
                    addDiagnostic(diagnostics, "apf_segment_repaired",
                        "APF repaired interval " + std::to_string(run.beginSegment + 1)
                        + " -> " + std::to_string(run.endSegment + 2)
                        + "; safe anchors " + std::to_string(begin + 1) + " -> " + std::to_string(end + 1)
                        + ", timestamps and waypoint count preserved.");
                }
                if(options.progress) options.progress(repaired ? "APF interval verified" : "APF interval failed");
                if(!repaired) { addDiagnostic(diagnostics, "apf_segment_failed",
                    "APF exhausted local fields/anchor expansions for interval "
                    + std::to_string(run.beginSegment + 1) + " -> " + std::to_string(run.endSegment + 2) + "; could not connect within the configured TCP corridor (" + std::to_string(options.apfMaxTcpDeviation * 1000.0)
                    + " mm). No larger detour was accepted. APF failure is not proof of infeasibility.");
                    return changedAny;
                }
            }
            return changedAny;
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
            const std::vector<std::vector<double>>& currentInput,
            const std::vector<std::vector<double>>& seedInput,
            const std::vector<CdfLinearization>& linearizations,
            const std::vector<JointBound>& bounds,
            const ProjectCdfQpRepairOptions& options)
        {
            QpSolveResult result;
            const auto currentPath = unwrapContinuousPath(currentInput, bounds);
            auto seedPath = unwrapContinuousPath(seedInput, bounds);
            // Use one chart for the entire QP; wrapping each waypoint at +/-pi
            // creates artificial turns in the smoothness objective.
            for(std::size_t i = 0; i < seedPath.size() && i < currentPath.size(); ++i)
                for(std::size_t j = 0; j < bounds.size() && j < seedPath[i].size(); ++j)
                    if(bounds[j].continuous) seedPath[i][j] = currentPath[i][j] +
                        std::remainder(seedPath[i][j] - currentPath[i][j], kTwoPi);
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

            const auto parameters = jointPathParameters(seedPath);
            std::vector<double> trackingQuadrature(waypointCount, 1.0);
            for(std::size_t i = 0; i < waypointCount && waypointCount > 1; ++i) {
                const double left = i > 0 ? parameters[i] - parameters[i - 1] : 0.0;
                const double right = i + 1 < waypointCount ? parameters[i + 1] - parameters[i] : 0.0;
                trackingQuadrature[i] = 0.5 * (left + right);
            }
            std::vector<c_float> qVector(variableCount, 0.0);
            for(std::size_t waypoint = 0; waypoint < waypointCount; ++waypoint) {
                for(std::size_t joint = 0; joint < jointCount; ++joint) {
                    const std::size_t index = variableIndex(waypoint, joint);
                    const double seedValue = waypoint < seedPath.size() && joint < seedPath[waypoint].size()
                        ? seedPath[waypoint][joint]
                        : 0.0;
                    qVector[index] = static_cast<c_float>(-seedTrackingWeight * trackingQuadrature[waypoint] * seedValue);
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
                    const double lowerLimit = jointBound.continuous ? -OSQP_INFTY : jointBound.lower;
                    const double upperLimit = jointBound.continuous ? OSQP_INFTY : jointBound.upper;
                    double lower = std::max(lowerLimit, value - trustRegion);
                    double upper = std::min(upperLimit, value + trustRegion);
                    double seedValue = waypoint < seedPath.size() && joint < seedPath[waypoint].size()
                        ? seedPath[waypoint][joint]
                        : value;
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
                    warmStart[index] = static_cast<c_float>(value);
                }
            }

            const auto buildHessian = [&](const QpNumericsAttempt& attempt) -> OsqpMatrixPtr {
                const double effectiveRegularizer = std::max(1.0e-8, attempt.regularizer);
                const double effectiveSmoothWeight = std::max(0.0, smoothWeight * attempt.smoothScale);
                const double effectiveSlackPenalty = std::max(1.0, attempt.slackPenalty);

                // Quadrature-weighted tracking plus nonuniform SECOND differences.
                // The old first-difference term penalized path length, not corners.
                // Accumulate each upper-triangular coefficient once (no duplicate CSC entries).
                std::vector<double> diagonal(configVariableCount), upperOne(configVariableCount, 0.0), upperTwo(configVariableCount, 0.0);
                for(std::size_t i = 0; i < waypointCount; ++i) for(std::size_t j = 0; j < jointCount; ++j)
                    diagonal[variableIndex(i, j)] = (effectiveRegularizer + seedTrackingWeight) * trackingQuadrature[i];
                for(std::size_t i = 1; i + 1 < waypointCount; ++i) {
                    const double left = parameters[i] - parameters[i - 1], right = parameters[i + 1] - parameters[i];
                    const double a = 1.0 / left, b = -1.0 / left - 1.0 / right, c = 1.0 / right;
                    const double weight = effectiveSmoothWeight * 2.0 / (left + right);
                    for(std::size_t j = 0; j < jointCount; ++j) {
                        const auto x = variableIndex(i - 1, j), y = variableIndex(i, j), z = variableIndex(i + 1, j);
                        diagonal[x] += weight * a * a; diagonal[y] += weight * b * b; diagonal[z] += weight * c * c;
                        upperOne[x] += weight * a * b; upperOne[y] += weight * b * c; upperTwo[x] += weight * a * c;
                    }
                }
                std::vector<SparseTripletEntry> pEntries;
                pEntries.reserve(3 * configVariableCount + slackVariableCount);
                for(std::size_t i = 0; i < waypointCount; ++i) for(std::size_t j = 0; j < jointCount; ++j) {
                    const auto index = variableIndex(i, j);
                    pEntries.push_back({static_cast<c_int>(index), static_cast<c_int>(index), static_cast<c_float>(diagonal[index])});
                    if(i + 1 < waypointCount && upperOne[index] != 0.0)
                        pEntries.push_back({static_cast<c_int>(index), static_cast<c_int>(variableIndex(i + 1, j)), static_cast<c_float>(upperOne[index])});
                    if(i + 2 < waypointCount && upperTwo[index] != 0.0)
                        pEntries.push_back({static_cast<c_int>(index), static_cast<c_int>(variableIndex(i + 2, j)), static_cast<c_float>(upperTwo[index])});
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
                { 1.0e-4, 1.0e3, 1.0, 5000, 2.0e-4, 2.0e-4, "curvature_regularized" },
                { 1.0e-3, 8.0e2, 0.75, 7000, 3.0e-4, 3.0e-4, "curvature_conditioned" },
                { 1.0e-2, 5.0e2, 0.5, 12000, 5.0e-4, 5.0e-4, "curvature_fallback" },
                { 1.0e-1, 2.0e2, 0.25, 16000, 1.0e-3, 1.0e-3, "curvature_loose_fallback" }
            };

            std::string lastFailure;
            for(const QpNumericsAttempt& attempt : attempts) {
                OsqpMatrixPtr P = buildHessian(attempt);
                if(P == nullptr) {
                    lastFailure = "Failed to build the QP Hessian matrix.";
                    continue;
                }

                for(std::size_t i = 0; i < waypointCount; ++i) for(std::size_t j = 0; j < jointCount; ++j)
                    qVector[variableIndex(i, j)] = static_cast<c_float>(-trackingQuadrature[i] *
                        (seedTrackingWeight * seedPath[i][j] + attempt.regularizer * currentPath[i][j]));
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
                        result.path[waypoint][joint] = value;
                    }

                    const double slackValue = static_cast<double>(workspace->solution->x[configVariableCount + waypoint]);
                    result.maximumSlack = std::max(result.maximumSlack, std::max(0.0, slackValue));
                    for(std::size_t joint = 0; joint < jointCount; ++joint)
                        if(!effectiveBounds[joint].continuous)
                            result.path[waypoint][joint] = std::clamp(result.path[waypoint][joint],
                                effectiveBounds[joint].lower, effectiveBounds[joint].upper);
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

                result.message = std::string(attempt.label) + "; smooth_scale=" + std::to_string(attempt.smoothScale);
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
            detail::CdfQueryBatch& queries,
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
                const auto states = queries.motions(candidate, validation);
                return static_cast<int>(std::count_if(states.begin(), states.end(),
                    [](const auto& state) { return !state.valid; }));
            };

            auto evaluatePathMinimum = [&](const std::vector<std::vector<double>>& candidate) {
                double minimumPhi = std::numeric_limits<double>::max();
                const auto samples = queries.distances(candidate, options.safetyMargin,
                    options.distanceThreshold, *statistics);
                for(const auto& sample : samples) {
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
                if(options.progress) options.progress("Local CDF/QP iteration " + std::to_string(iteration + 1));
                auto linearizations = queries.linearizations(path, options.safetyMargin,
                    options.distanceThreshold, options.finiteDifferenceStep, *statistics);
                double currentPhi = std::numeric_limits<double>::max();
                bool allSamplesValid = true;
                for(const auto& linearization : linearizations) {
                    if(!linearization.sample.valid) {
                        addDiagnostic(diagnostics, "cdf_linearization_failed", linearization.sample.message);
                        allSamplesValid = false;
                        break;
                    }
                    currentPhi = std::min(currentPhi, linearization.sample.phi);
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
                if(options.progress) options.progress("Validating QP candidate clearance and motion");
                double candidatePhi = evaluatePathMinimum(candidatePath);
                int candidateInvalidSegments = countInvalidSegments(candidatePath);

                if(candidateInvalidSegments == 0 &&
                    (currentInvalidSegments > 0 || candidatePhi >= std::min(currentPhi, targetPhi) - 1.0e-9))
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
                            (currentInvalidSegments > 0 || blendedPhi >= std::min(currentPhi, targetPhi) - 1.0e-9))
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
        const robottrajectory::JointTrajectory& inputTrajectory,
        const ProjectCdfQpRepairOptions& options) const
    {
        auto seedTrajectory = inputTrajectory;
        bool usedEquivalentSeed = false;
        ProjectCdfQpRepairResult result;
        result.statistics.inputWaypointCount = static_cast<int>(seedTrajectory.points.size());
        if(!std::isfinite(options.apfMaxTcpDeviation) || options.apfMaxTcpDeviation < 0.0 ||
            (options.apfMaxTcpDeviation > 0.0 && !options.worldForwardKinematics)) {
            addDiagnostic(&result.diagnostics, "apf_tcp_model_missing", "A finite TCP deviation limit requires actual calibrated world TCP forward kinematics; no unbounded fallback was run.");
            return result;
        }

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
        request.validation.maxJointStep = std::min(options.validationMaxJointStep, 0.001);

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

        if(!options.cartesianTargets.empty()) {
            if(options.cartesianTargets.size() != seedTrajectory.points.size() || !options.worldForwardKinematics) {
                addDiagnostic(&result.diagnostics, "apf_cartesian_mismatch", "Original Cartesian targets must match every seed knot and have actual TCP FK.");
                return result;
            }
            const auto matchesTarget = [&](const std::vector<double>& q, std::size_t i) {
                const auto& target = options.cartesianTargets[i];
                if(!target.matrix().allFinite()) return false;
                const Eigen::JacobiSVD<Eigen::Matrix3d> svd(target.linear(), Eigen::ComputeFullU | Eigen::ComputeFullV);
                const Eigen::Matrix3d rotation = svd.matrixU() * svd.matrixV().transpose();
                const auto actual = options.worldForwardKinematics(q);
                return actual.matrix().allFinite() && (actual.translation() - target.translation()).norm() <= 1e-5 &&
                    Eigen::AngleAxisd(actual.linear().transpose() * rotation).angle() <= 1e-5;
            };
            for(std::size_t i=0;i<seedTrajectory.points.size();++i) {
                if(!matchesTarget(seedTrajectory.points[i].q,i)) {
                    addDiagnostic(&result.diagnostics, "apf_cartesian_seed_stale", "Selected joint seed does not match the original Cartesian targets; regenerate IK.");
                    return result;
                }
            }
            if(options.allowEquivalentEndpointConfigurations && jointNames.size()==6) {
                if(options.progress) options.progress("Following original Cartesian poses with continuous actual-model IK");
                StoredMotionPlan targetPlan; targetPlan.robotId=robotId; targetPlan.jointNames=jointNames;
                for(std::size_t i=0;i<options.cartesianTargets.size();++i) {
                    robottrajectory::TimedCartesianPoint point; point.time=seedTrajectory.points[i].time;
                    point.tcpPose=options.cartesianTargets[i]; targetPlan.cartesianControlPoints.points.push_back(point);
                }
                CartesianIkOptions ikOptions;ikOptions.robotId=robotId;ikOptions.jointNames=jointNames;
                // This callback and seed BOTH use runtime coordinates. No second sign mapping.
                ikOptions.worldForwardKinematics=options.worldForwardKinematics;
                ikOptions.seedJoints=seedTrajectory.points.front().q;ikOptions.stepSize=1.0;
                ikOptions.damping=0.001;ikOptions.maxIterations=300;
                const auto solved=ProjectTrajectoryInverseKinematics::solveCartesianControlPoints(document,targetPlan,ikOptions);
                bool valid=solved.success && solved.plan.trajectory.points.size()==seedTrajectory.points.size();
                auto candidate=solved.plan.trajectory;
                if(valid) {
                    candidate.points.front().q=seedTrajectory.points.front().q;
                    for(std::size_t i=0;i<candidate.points.size() && valid;++i) {
                        auto& q=candidate.points[i].q;
                        for(std::size_t j=0;j<q.size();++j) {
                            const auto& bound=scene->jointBounds()[j];
                            if(i>0 && bound.continuous)q[j]=candidate.points[i-1].q[j]+std::remainder(q[j]-candidate.points[i-1].q[j],kTwoPi);
                            if(!bound.continuous && (q[j]<bound.lower || q[j]>bound.upper))valid=false;
                        }
                        valid &= matchesTarget(q,i);
                    }
                }
                if(valid && scene->validateState(candidate.points.front().q).valid && scene->validateState(candidate.points.back().q).valid) {
                    seedTrajectory=std::move(candidate);
                    usedEquivalentSeed=true;
                    if(options.progress) options.progress("Continuous IK seed accepted: every original full TCP pose verified; APF collision repair still required");
                    addDiagnostic(&result.diagnostics,"apf_continuous_ik_seed",
                        "Used actual-model continuous IK of the ORIGINAL Cartesian targets; all full poses revalidated. Equivalent endpoint joint configurations/turns are allowed. Collision avoidance follows; this seed is not a collision-free result.");
                } else {
                    addDiagnostic(&result.diagnostics,"apf_continuous_ik_unavailable","Continuous IK candidate failed pose/limit/endpoint collision checks; retained the selected input seed.");
                }
            }
        }

        detail::CdfQueryBatch queries(*scene, planningDocument, projectBaseOrParent(projectBasePath), request, options.queryWorkers);
        if(options.progress) options.progress("CDF query workers: " + std::to_string(queries.workerCount()));

        result.referenceSeedTrajectory = seedTrajectory;
        std::vector<std::pair<std::size_t, double>> correspondence;
        const std::vector<robottrajectory::TimedJointPoint> denseSeedTrajectory =
            densifyTrajectory(
                seedTrajectory,
                scene->jointBounds(),
                options.segmentIntermediateSamples,
                options.optimizationMaxJointStep, &correspondence);
        result.statistics.inputWaypointCount = static_cast<int>(denseSeedTrajectory.size());

        std::vector<std::vector<double>> path;
        path.reserve(denseSeedTrajectory.size());
        for(const robottrajectory::TimedJointPoint& point : denseSeedTrajectory) {
            path.push_back(clampToBounds(
                point.q,
                scene->jointBounds(),
                &result.statistics.clampedSeedValues));
        }
        path = unwrapContinuousPath(path, scene->jointBounds());
        detail::ApfGuidance tcpGuide;
        const bool constrainTcp = options.apfMaxTcpDeviation > 0.0;
        if(options.worldForwardKinematics) {
            tcpGuide.position = [&](const auto& q) -> detail::ApfState {
                const auto pose = options.worldForwardKinematics(q);
                if(!pose.matrix().allFinite()) return {};
                return {pose.translation().x(), pose.translation().y(), pose.translation().z()};
            };
            tcpGuide.maxDeviation = options.apfMaxTcpDeviation;
            detail::ApfPath controlPositions;
            for(const auto& point : seedTrajectory.points) {
                const auto p = options.cartesianTargets.empty() ? tcpGuide.position(point.q) : detail::ApfState{
                    options.cartesianTargets[controlPositions.size()].translation().x(),
                    options.cartesianTargets[controlPositions.size()].translation().y(),
                    options.cartesianTargets[controlPositions.size()].translation().z()};
                if(p.size() != 3) { addDiagnostic(&result.diagnostics, "apf_tcp_invalid", "Actual TCP FK returned an invalid original control point."); return result; }
                controlPositions.push_back(p);
            }
            for(const auto& station : correspondence) {
                auto p = controlPositions[station.first];
                if(station.first+1 < controlPositions.size())
                    for(int j = 0; j < 3; ++j) p[j] += station.second * (controlPositions[station.first+1][j] - p[j]);
                tcpGuide.positions.push_back(std::move(p));
            }
        }
        const auto followsReference = [&](const auto& candidate, std::size_t begin = 0) {
            return !constrainTcp || detail::followsApfPath(candidate, tcpGuide, begin);
        };
        const std::vector<std::vector<double>> originalReferencePath = path;
        const auto started = std::chrono::steady_clock::now();
        const auto comparisonParameters = jointPathParameters(originalReferencePath);
        robottrajectory::JointTrajectory referenceTrajectory;
        referenceTrajectory.points = denseSeedTrajectory;
        auto captureStage = [&](const std::string& name, const std::vector<std::vector<double>>& stagePath,
                                double minimumPhi, int invalidSegments,
                                const robottrajectory::JointTrajectory* timed = nullptr) {
            CdfTrajectoryStage stage;
            stage.name = name;
            stage.plan.robotId = robotId;
            stage.plan.jointNames = jointNames;
            stage.plan.id = robotId + "_cdf_stage_" + std::to_string(result.stages.size());
            stage.plan.name = name;
            stage.plan.trajectory = timed ? *timed : referenceTrajectory;
            for(std::size_t i = 0; i < stagePath.size(); ++i) stage.plan.trajectory.points[i].q = stagePath[i];
            stage.plan.trajectory.name = stage.plan.id;
            stage.plan.trajectory.interpolation = robottrajectory::TrajectoryInterpolation::Linear;
            stage.quality = evaluateTrajectoryQuality(stage.plan.trajectory, scene->jointBounds(),
                options.fallbackMaxVelocity, options.fallbackMaxAcceleration, comparisonParameters,
                options.worldForwardKinematics, &referenceTrajectory);
            if(stage.quality.tcpAvailable && tcpGuide.positions.size() == stagePath.size()) {
                stage.quality.maximumTcpDeviation = 0.0;
                for(std::size_t i = 0; i < stagePath.size(); ++i) {
                    const auto& p = tcpGuide.positions[i];
                    const Eigen::Vector3d target(p[0], p[1], p[2]);
                    stage.referenceTcpPositions.push_back(target);
                    stage.quality.maximumTcpDeviation = std::max(stage.quality.maximumTcpDeviation,
                        (stage.quality.tcpPositions[i] - target).norm());
                }
            }
            stage.minimumPhi = minimumPhi;
            stage.invalidSegments = invalidSegments;
            stage.tcpCorridorValid = followsReference(stagePath);
            stage.tcpDeviationLimit = options.apfMaxTcpDeviation;
            stage.elapsedSeconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count();
            result.stages.push_back(std::move(stage));
        };

        auto evaluatePathMinimum = [&](const std::vector<std::vector<double>>& candidate) {
            double minimumPhi = std::numeric_limits<double>::max();
            const auto samples = queries.distances(candidate, options.safetyMargin, options.distanceThreshold, result.statistics);
            for(const auto& sample : samples) {
                if(!sample.valid) {
                    addDiagnostic(&result.diagnostics, "cdf_distance_failed", sample.message);
                    return -std::numeric_limits<double>::max();
                }
                minimumPhi = std::min(minimumPhi, sample.phi);
            }
            return minimumPhi;
        };

        auto countInvalidSegments = [&](const std::vector<std::vector<double>>& candidate) {
            const auto results = queries.motions(candidate, request.validation);
            return static_cast<int>(std::count_if(results.begin(), results.end(), [](const auto& value) { return !value.valid; }));
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

        if(options.progress) options.progress("Evaluating initial clearance at " + std::to_string(path.size()) + " waypoints");
        result.statistics.initialMinimumPhi = evaluatePathMinimum(path);
        if(result.statistics.initialMinimumPhi <= -std::numeric_limits<double>::max() * 0.25) {
            return result;
        }

        const double targetPhi = options.targetClearance;
        const int maxIterations = std::max(1, options.maxIterations);
        std::vector<std::vector<double>> seedPath = path;

        if(options.progress) options.progress("Scanning initial path motion collisions");
        const int initialInvalidSegments = countInvalidSegments(path);
        if(constrainTcp) {
            int shapeInvalidSegments = 0;
            for(std::size_t i=1; i<path.size(); ++i)
                if(!detail::followsApfGuide(path[i-1], path[i], tcpGuide.positions[i-1], tcpGuide.positions[i], tcpGuide)) ++shapeInvalidSegments;
            addDiagnostic(&result.diagnostics, "apf_tcp_corridor",
                "TCP corridor " + std::to_string(options.apfMaxTcpDeviation * 1000.0)
                + " mm relative to the ordered original control-point polyline; input shape violations="
                + std::to_string(shapeInvalidSegments) + ", input motion collisions=" + std::to_string(initialInvalidSegments)
                + ". Position only; no orientation constraint.");
        }
        captureStage(usedEquivalentSeed ? "Continuous IK seed (dense reference)" : "Input (dense reference)", path, result.statistics.initialMinimumPhi, initialInvalidSegments);
        if(initialInvalidSegments != 0 || !followsReference(path)) {
            const bool apfRepaired = repairCollisionRunsWithApf(
                *scene,
                queries,
                scene->jointBounds(),
                request.validation,
                options,
                &result.statistics,
                &result.diagnostics,
                &path, constrainTcp ? &tcpGuide : nullptr);
            if(apfRepaired) {
                seedPath = path;
                result.statistics.finalMinimumPhi = evaluatePathMinimum(path);
                if(result.statistics.finalMinimumPhi <= -std::numeric_limits<double>::max() * 0.25) {
                    return result;
                }
            }
        }

        if(options.progress) options.progress("Verifying APF full path");
        result.statistics.invalidSegmentCount = countInvalidSegments(path);
        const double apfMinimumPhi = evaluatePathMinimum(path);

        if(result.statistics.invalidSegmentCount != 0 || !followsReference(path)) {
            captureStage("APF (incomplete)", path, apfMinimumPhi, result.statistics.invalidSegmentCount);
            result.statistics.finalMinimumPhi = evaluatePathMinimum(path);
            const std::string apfFailureDetails = result.diagnostics.empty() ? std::string() : " Details: " + result.diagnostics.back().message;
            addDiagnostic(&result.diagnostics, "apf_pre_repair_failed",
                "APF could not produce a complete collision-free path within the configured TCP deviation limit (" + std::to_string(options.apfMaxTcpDeviation * 1000.0) + " mm). QP/CDF was not started; no optimization result was published. Try another Top-K starting configuration; APF failure does not prove the corridor infeasible." + apfFailureDetails);
            return result;
        }
        addDiagnostic(&result.diagnostics, "apf_full_path_validated",
            "All trajectory segments passed collision and configured TCP corridor validation before QP/CDF.");

        // Optimize geometry independently of duplicate/dwell source timestamps.
        const auto knotTimes = constrainTcp ? jointPathParameters(tcpGuide.positions, 0.01) : jointPathParameters(path);
        const double smoothingStep = options.postSmoothingStep /
            (1.0 + std::max(0.0, options.postSmoothingSeedWeight));
        detail::ApfOracle smoothingOracle;
        smoothingOracle.motionValid = [&](const auto& a, const auto& b) {
            return scene->validateMotion(a, b, request.validation).valid;
        };
        double smoothingPhiFloor = std::min(apfMinimumPhi, targetPhi);
        detail::ApfState refinementLower,refinementUpper;
        for(const auto& bound:scene->jointBounds()) {
            refinementLower.push_back(bound.continuous ? -std::numeric_limits<double>::infinity() : bound.lower);
            refinementUpper.push_back(bound.continuous ? std::numeric_limits<double>::infinity() : bound.upper);
        }
        if(constrainTcp) smoothingOracle.project = [&](auto& q,std::size_t i) {
            return detail::projectApfPosition(q,tcpGuide.positions[i],refinementLower,refinementUpper,tcpGuide,
                tcpGuide.maxDeviation*0.99);
        };
        smoothingOracle.progress = [&](int pass, int total) {
            if(options.progress) options.progress("APF curve refinement " + std::to_string(pass+1) + " / " + std::to_string(total));
        };
        smoothingOracle.pathValid = [&](const auto& candidate, std::size_t begin) {
            if(!followsReference(candidate, begin)) return false;
            // Gate each window before committing it. One clearance-limited corner
            // must not discard valid smoothing improvements elsewhere.
            const double horizon=std::min(options.distanceThreshold,
                std::max(0.001,smoothingPhiFloor+options.safetyMargin+1e-5));
            const auto samples = queries.distances(candidate, options.safetyMargin, horizon, result.statistics);
            for(const auto& sample : samples)
                if(!sample.valid || sample.phi < smoothingPhiFloor - 1.0e-9) return false;
            return queries.pathValid(candidate, request.validation);
        };
        // Shape restoration uses the same detector/filters, at the final dense
        // resolution. A closest-reference objective is separate from joint bend.
        detail::ApfOracle shapeOracle=smoothingOracle;
        auto shapeValidation=request.validation;
        shapeValidation.maxJointStep=std::min(shapeValidation.maxJointStep,0.00025);
        shapeOracle.distances=[&](const auto& states) {
            const auto observations=queries.distances(states,0.0,options.distanceThreshold,result.statistics);
            std::vector<double> values;for(const auto& v:observations)
                values.push_back(v.valid?v.rawDistance:-std::numeric_limits<double>::infinity());
            return values;
        };
        shapeOracle.motionsValid=[&](const auto& states) {
            const auto observations=queries.motions(states,shapeValidation);
            std::vector<bool> values;for(const auto& v:observations)values.push_back(v.valid);return values;
        };
        shapeOracle.pathValid=[&](const auto& candidate,std::size_t begin) {
            return smoothingOracle.pathValid(candidate,begin) && queries.pathValid(candidate,shapeValidation);
        };
        shapeOracle.progress=[&](int pass,int total) {
            if(options.progress)options.progress("TCP reference shape refinement " + std::to_string(pass+1) + " / " + std::to_string(total));
        };
        const auto refineShape=[&]() {
            if(!constrainTcp || options.postSmoothingIterations<=0)return;
            const bool restored=detail::refineApfTcpShape(&path,knotTimes,refinementLower,refinementUpper,
                shapeOracle,tcpGuide,3,std::min(1.0,2.2*smoothingStep));
            addDiagnostic(&result.diagnostics,restored?"tcp_shape_restored":"tcp_shape_preserved",
                restored?"Restored ordered TCP scan shape with rounded reference corners and smooth collision-limited offsets; joint fairing stays within 0.201 mm of this TCP curve at ordered knots."
                    :"No complete validated TCP shape restoration was found; retained the preceding path without relaxing collision or deviation limits.");
        };
        path = unwrapContinuousPath(path, scene->jointBounds());
        if(options.postSmoothingIterations > 0) {
            if(options.progress) options.progress("Smoothing APF seed with collision-validated windows");
            const double beforeLength = detail::jointPathLength(path);
            const double beforeBending = detail::jointPathBending(path, knotTimes);
            detail::smoothValidatedPath(&path, knotTimes, smoothingOracle,
                options.postSmoothingIterations, constrainTcp ? std::min(1.0,2.2*smoothingStep) : smoothingStep, options.seedCorridor, constrainTcp);
            std::ostringstream message;
            message << "APF seed smoothing: joint length " << beforeLength << " -> "
                << detail::jointPathLength(path) << ", reference-parameter bending " << beforeBending
                << " -> " << detail::jointPathBending(path, knotTimes) << ".";
            addDiagnostic(&result.diagnostics, "apf_seed_smoothing", message.str());
        }
        refineShape();
        captureStage("APF (refined, before QP)", path, evaluatePathMinimum(path), 0);
        seedPath = path;

        // The complete path has passed the gate above. QP candidates and the
        // final output retain their existing collision acceptance checks.
        for(int iteration = 0; iteration < maxIterations; ++iteration) {
            if(options.progress) options.progress("CDF/QP iteration " + std::to_string(iteration + 1));
            if(options.progress) options.progress("CDF gradients 0 / " + std::to_string(path.size()));
            auto linearizations = queries.linearizations(path, options.safetyMargin, options.distanceThreshold,
                options.finiteDifferenceStep, result.statistics);
            double minimumPhi = std::numeric_limits<double>::max();
            bool allSamplesValid = true;
            for(const auto& linearization : linearizations) {
                if(!linearization.sample.valid) {
                    addDiagnostic(&result.diagnostics, "cdf_linearization_failed", linearization.sample.message);
                    allSamplesValid = false; break;
                }
                minimumPhi = std::min(minimumPhi, linearization.sample.phi);
            }
            if(!allSamplesValid) {
                return result;
            }

            result.statistics.finalMinimumPhi = minimumPhi;
            if(options.progress) options.progress("Solving the trajectory QP");
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

            addDiagnostic(&result.diagnostics, "cdf_qp_numerics", qpResult.message);
            const int currentInvalidSegments = countInvalidSegments(path);
            const double currentPhi = minimumPhi;
            std::vector<std::vector<double>> candidatePath = qpResult.path;
            if(options.progress) options.progress("Validating QP candidate clearance and motion");
            double candidatePhi = evaluatePathMinimum(candidatePath);
            // A failed clearance predicate already rejects this candidate. Keep
            // the same acceptance rule, but avoid an irrelevant full motion scan.
            if((currentInvalidSegments > 0 || candidatePhi >= std::min(currentPhi, targetPhi) - 1.0e-9) &&
                followsReference(candidatePath) && countInvalidSegments(candidatePath) == 0)
            {
                path = std::move(candidatePath);
                ++result.statistics.acceptedQpSteps;
                result.statistics.finalMinimumPhi = candidatePhi;
            } else {
                double alpha = 0.5;
                for(int backtrack = 0; backtrack < 5; ++backtrack) {
                    if(options.progress) options.progress("QP line search " + std::to_string(backtrack + 1) + " / 5");
                    std::vector<std::vector<double>> candidate = blendPath(path, qpResult.path, alpha);
                    const double blendedPhi = evaluatePathMinimum(candidate);
                    if((currentInvalidSegments > 0 || blendedPhi >= std::min(currentPhi, targetPhi) - 1.0e-9) &&
                        followsReference(candidate) && countInvalidSegments(candidate) == 0)
                    {
                        path = std::move(candidate);
                        ++result.statistics.acceptedQpSteps;
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
                        // First give this window its own APF chance. QP is only a
                        // local smoother after the complete window is collision-free.
                        auto windowGuide = tcpGuide;
                        if(constrainTcp) windowGuide.positions.assign(tcpGuide.positions.begin()+beginIndex, tcpGuide.positions.begin()+endIndex+1);
                        const bool apfWindowChanged = repairCollisionRunsWithApf(
                            *scene,
                            queries,
                            scene->jointBounds(),
                            localValidation,
                            localOptions,
                            &result.statistics,
                            &result.diagnostics,
                            &windowPath, constrainTcp ? &windowGuide : nullptr);
                        if(apfWindowChanged) {
                            windowSeed = windowPath;
                        }
                        if(countInvalidSegments(windowPath) != 0) {
                            return resultWindow;
                        }

                        std::vector<std::vector<double>> repairedWindow;
                        const bool windowSolved = runCdfQpRepairIterations(
                            queries,
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
                        if(countInvalidSegments(repairedWindow) != 0 || !followsReference(repairedWindow, beginIndex)) {
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
                collectInvalidSegmentRuns(queries, path, localValidation);
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
                collectInvalidSegmentRuns(queries, path, localValidation);
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

            remainingRuns = collectInvalidSegmentRuns(queries, path, localValidation);
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
                remainingRuns = collectInvalidSegmentRuns(queries, path, localValidation);
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

        if(countInvalidSegments(path) == 0 && options.postSmoothingIterations > 0) {
            path = unwrapContinuousPath(path, scene->jointBounds());
            const auto beforeSmoothing = path;
            const double beforePhi = evaluatePathMinimum(path);
            smoothingPhiFloor = std::min(beforePhi, targetPhi);
            if(options.progress) options.progress("Smoothing QP result with collision-validated windows");
            const bool smoothed = detail::smoothValidatedPath(&path, knotTimes, smoothingOracle,
                options.postSmoothingIterations, constrainTcp ? std::min(1.0,2.2*smoothingStep) : smoothingStep, options.seedCorridor, constrainTcp);
            if(smoothed) {
                const double smoothedPhi = evaluatePathMinimum(path);
                if(smoothedPhi + 1.0e-9 < std::min(beforePhi, targetPhi)) {
                    path = beforeSmoothing;
                    addDiagnostic(&result.diagnostics, "cdf_smoothing_clearance_preserved",
                        "Kept the pre-smoothing path because smoothing reduced the achieved clearance.");
                } else {
                    std::ostringstream message;
                    message << "Collision-validated smoothing: joint length "
                        << detail::jointPathLength(beforeSmoothing) << " -> " << detail::jointPathLength(path)
                        << ", reference-parameter bending " << detail::jointPathBending(beforeSmoothing, knotTimes)
                        << " -> " << detail::jointPathBending(path, knotTimes) << ".";
                    addDiagnostic(&result.diagnostics, "cdf_collision_constrained_smoothing", message.str());
                }
            }
        }

        refineShape();
        result.statistics.finalMinimumPhi = evaluatePathMinimum(path);
        if(result.statistics.finalMinimumPhi <= -std::numeric_limits<double>::max() * 0.25) {
            return result;
        }

        // The trajectory player linearly interpolates raw joint values. Output
        // the same continuous chart used by shortest-arc motion validation, even
        // when smoothing is disabled or the trajectory has only two points.
        path = unwrapContinuousPath(path, scene->jointBounds());
        if(options.progress) options.progress("Final full-path collision verification");
        result.statistics.invalidSegmentCount = 0;
        // Fresh final validation: bypass the cache and retain every original sample.
        auto finalValidationOptions = request.validation;
        finalValidationOptions.maxJointStep = std::min(finalValidationOptions.maxJointStep, 0.00025);
        const auto finalValidation = queries.motions(path, finalValidationOptions, false);
        for(std::size_t index = 0; index + 1 < path.size(); ++index) {
            const StateValidationResult& validation = finalValidation[index];
            if(!validation.valid) {
                ++result.statistics.invalidSegmentCount;
                addDiagnostic(&result.diagnostics, "cdf_repaired_segment_invalid", segmentFailureMessage(index, validation));
            }
        }

        addDiagnostic(&result.diagnostics, "cdf_query_performance", queries.summary());
        if(result.statistics.invalidSegmentCount != 0 || !followsReference(path)) {
            addDiagnostic(&result.diagnostics, "cdf_final_path_rejected",
                "Final collision/TCP corridor verification failed; no trajectory was published.");
            return result;
        }

        if(!options.cartesianTargets.empty()) {
            for(std::size_t i : {std::size_t(0),path.size()-1}) {
                const auto actual=options.worldForwardKinematics(path[i]);
                const auto& target=i==0?options.cartesianTargets.front():options.cartesianTargets.back();
                const Eigen::JacobiSVD<Eigen::Matrix3d> svd(target.linear(),Eigen::ComputeFullU|Eigen::ComputeFullV);
                const Eigen::Matrix3d rotation=svd.matrixU()*svd.matrixV().transpose();
                if(!actual.matrix().allFinite() || (actual.translation()-target.translation()).norm()>1e-5 ||
                    Eigen::AngleAxisd(actual.linear().transpose()*rotation).angle()>1e-5) {
                    addDiagnostic(&result.diagnostics,"cdf_endpoint_pose_rejected","Final original endpoint TCP pose validation failed; no trajectory was published.");
                    return result;
                }
            }
        }

        for(std::size_t i = 0; i < path.size(); ++i) {
            for(std::size_t j = 0; j < path[i].size(); ++j) {
                double correction = path[i][j] - originalReferencePath[i][j];
                if(scene->jointBounds()[j].continuous) correction = std::remainder(correction, kTwoPi);
                result.statistics.maximumCorrection = std::max(result.statistics.maximumCorrection, std::abs(correction));
            }
        }

        result.plan.id = robotId + "_cdf_qp_repaired";
        result.plan.name = "APF + CDF/QP repaired trajectory";
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

        if(options.retimeOutput) {
            if(options.progress) options.progress("Time scaling and sampled velocity/acceleration validation");
            if(!retimeJointTrajectory(result.plan.trajectory, scene->jointBounds(),
                options.fallbackMaxVelocity, options.fallbackMaxAcceleration)) {
                addDiagnostic(&result.diagnostics, "cdf_time_scaling_failed", "Cannot assign valid sampled timing/limits; output was rejected.");
                result.plan = {};
                return result;
            }
        }
        captureStage("CDF/QP final", path, result.statistics.finalMinimumPhi,
            result.statistics.invalidSegmentCount, &result.plan.trajectory);
        addDiagnostic(&result.diagnostics, "cdf_local_optimization", "Curvature smoothing and clearance repair are local optimization, not proof of global path/time optimality. Knot accelerations are finite differences; linear interpolation is not a continuous acceleration certification.");
        result.success =
            result.statistics.finalMinimumPhi >= targetPhi &&
            result.statistics.invalidSegmentCount == 0;
        if(!result.success) {
            addDiagnostic(
                &result.diagnostics,
                "cdf_repair_not_converged",
                "CDF/QP repair finished but did not reach the requested clearance.");
        }
        return result;
    }
}
