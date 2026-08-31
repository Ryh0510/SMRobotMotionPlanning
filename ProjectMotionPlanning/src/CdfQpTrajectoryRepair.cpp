#include <ProjectMotionPlanning/CdfQpTrajectoryRepair.h>

#include <ProjectMotionPlanning/ProjectMotionPlanning.h>
#include <SimulationProject/ProjectDocument.h>
#include <osqp.h>

#include <algorithm>
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
                        const JointBound& jointBound = jointIndex < bounds.size() ? bounds[jointIndex] : bounds.back();
                        double delta = end.q[jointIndex] - start.q[jointIndex];
                        if(jointBound.continuous) {
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
                        sample.q[jointIndex] = startValue + (endValue - startValue) * fraction;
                    }
                    sample.qd.clear();
                    sample.qdd.clear();
                    densePoints.push_back(std::move(sample));
                }
                densePoints.push_back(end);
            }

            return densePoints;
        }

        struct SignedDistanceSample
        {
            bool valid = false;
            bool inCollision = false;
            double phi = -std::numeric_limits<double>::max();
            double rawDistance = std::numeric_limits<double>::max();
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
                    double penetrationDepth = 0.0;
                    for(const collision::Contact& contact : collisionResult->contacts) {
                        if(std::isfinite(contact.penetrationDepth)) {
                            penetrationDepth = std::max(penetrationDepth, contact.penetrationDepth);
                        }
                    }
                    const double signedDistance = -(penetrationDepth > 0.0 ? penetrationDepth : 1.0e-5);
                    minimumPhi = std::min(minimumPhi, signedDistance - safetyMargin);
                    minimumDistance = std::min(minimumDistance, signedDistance);
                    sawFiniteDistance = true;
                } else if(std::isfinite(collisionResult->minDistance) &&
                    collisionResult->minDistance < std::numeric_limits<double>::max() * 0.25) {
                    sawFiniteDistance = true;
                    minimumDistance = std::min(minimumDistance, collisionResult->minDistance);
                    minimumPhi = std::min(minimumPhi, collisionResult->minDistance - safetyMargin);
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
                { 1.0e-4, 1.0e3, 0.0, "diagonal_regularized" },
                { 1.0e-3, 1.0e3, 0.0, "strong_diagonal_regularized" },
                { 1.0e-2, 1.0e2, 0.0, "very_strong_diagonal_regularized" }
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
                settings.max_iter = 4000;
                settings.eps_abs = 1.0e-4;
                settings.eps_rel = 1.0e-4;
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
        const std::vector<std::vector<double>> seedPath = path;

        for(int iteration = 0; iteration < maxIterations; ++iteration) {
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
            localOptions.trustRegion = std::max(0.003, std::min(options.trustRegion, 0.008));
            localOptions.seedCorridor = std::max(0.02, std::min(options.seedCorridor, 0.04));
            localOptions.repairGain = std::max(0.10, std::min(options.repairGain, 0.18));
            localOptions.seedTrackingWeight = std::max(options.seedTrackingWeight, 0.60);
            localOptions.smoothWeight = std::max(options.smoothWeight, 0.24);
            localOptions.targetClearance = std::max(options.targetClearance, 0.001);
            localOptions.maxIterations = 1;
            localOptions.keepEndpoints = true;

            MotionValidationOptions localValidation = request.validation;
            if(localValidation.maxJointStep > 0.0 && std::isfinite(localValidation.maxJointStep)) {
                localValidation.maxJointStep = std::min(localValidation.maxJointStep, 0.01);
            }

            const std::size_t primaryWindowPoints = 32;
            const std::size_t secondaryWindowPoints = 20;
            const std::size_t primaryOverlap = 2;
            const std::size_t secondaryOverlap = 1;

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
                6,
                secondaryWindowPoints,
                secondaryOverlap,
                12);
            sortWindowsByRisk(secondaryWindows);
            repairWindowSet(secondaryWindows, 8, "secondary", &failedWindows);

            remainingRuns = collectInvalidSegmentRuns(*scene, path, localValidation);
            if(!remainingRuns.empty()) {
                for(std::size_t ordinal = 0; ordinal < remainingRuns.size(); ++ordinal) {
                    std::ostringstream stream;
                    stream << "Remaining collision interval #" << (ordinal + 1)
                           << " spans segment " << (remainingRuns[ordinal].beginSegment + 1)
                           << " -> " << (remainingRuns[ordinal].endSegment + 2)
                           << " after primary and secondary CDF/QP repair.";
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
