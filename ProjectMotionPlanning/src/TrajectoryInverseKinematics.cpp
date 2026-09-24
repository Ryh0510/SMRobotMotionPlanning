#include <ProjectMotionPlanning/TrajectoryInverseKinematics.h>

#include <SimulationProject/ProjectDocument.h>
#include <SimulationRuntime/ProjectSimulationRuntime.h>

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <utility>

namespace motion_planning
{
    namespace
    {
        constexpr double kPi = 3.14159265358979323846;
        constexpr std::size_t kIrb4600JointCount = 6;

        struct DhParam
        {
            double a = 0.0;
            double alpha = 0.0;
            double d = 0.0;
            double thetaOffset = 0.0;
        };

        std::vector<DhParam> irb4600DhParamsMeters()
        {
            constexpr double mmToMeters = 0.001;
            return {
                { 0.0 * mmToMeters, 0.0, 495.0 * mmToMeters, 0.0 },
                { 175.0 * mmToMeters, -kPi / 2.0, 0.0 * mmToMeters, -kPi / 2.0 },
                { 900.0 * mmToMeters, 0.0, 0.0 * mmToMeters, 0.0 },
                { 175.0 * mmToMeters, -kPi / 2.0, 960.0 * mmToMeters, 0.0 },
                { 0.0 * mmToMeters, kPi / 2.0, 0.0 * mmToMeters, 0.0 },
                { 0.0 * mmToMeters, -kPi / 2.0, 135.0 * mmToMeters, kPi }
            };
        }

        Eigen::Matrix4d fixedToolTransformMeters()
        {
            Eigen::Matrix4d transform;
            transform <<
                -0.4899, 0.0, -0.8718, -0.096651,
                -0.8718, 0.0, 0.4899, 0.054238,
                -0.0000, 1.0, -0.0000, 1.219800,
                0.0000, 0.0, 0.0000, 1.000000;
            return transform;
        }

        double normalizeAngle(double angle)
        {
            if(!std::isfinite(angle)) {
                return 0.0;
            }
            double normalized = std::fmod(angle, 2.0 * kPi);
            if(normalized > kPi) {
                normalized -= 2.0 * kPi;
            } else if(normalized < -kPi) {
                normalized += 2.0 * kPi;
            }
            return normalized;
        }

        Eigen::Matrix4d makeTransform(const simulation_project::TransformDesc& desc)
        {
            Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
            transform.block<3, 1>(0, 3) = Eigen::Vector3d(desc.x, desc.y, desc.z);
            transform.block<3, 3>(0, 0) =
                (Eigen::AngleAxisd(desc.yaw, Eigen::Vector3d::UnitZ()) *
                 Eigen::AngleAxisd(desc.pitch, Eigen::Vector3d::UnitY()) *
                 Eigen::AngleAxisd(desc.roll, Eigen::Vector3d::UnitX()))
                    .toRotationMatrix();
            return transform;
        }

        Eigen::Matrix4d forwardKinematicsModifiedDh(
            const std::vector<DhParam>& dhParams,
            const std::vector<double>& joints)
        {
            Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
            const std::size_t jointCount = std::min(dhParams.size(), joints.size());

            for(std::size_t index = 0; index < jointCount; ++index) {
                const DhParam& dh = dhParams[index];
                const double theta = joints[index] + dh.thetaOffset;
                const double cosTheta = std::cos(theta);
                const double sinTheta = std::sin(theta);
                const double cosAlpha = std::cos(dh.alpha);
                const double sinAlpha = std::sin(dh.alpha);

                Eigen::Matrix4d jointTransform;
                jointTransform <<
                    cosTheta, -sinTheta, 0.0, dh.a,
                    sinTheta * cosAlpha, cosTheta * cosAlpha, -sinAlpha, -sinAlpha * dh.d,
                    sinTheta * sinAlpha, cosTheta * sinAlpha, cosAlpha, cosAlpha * dh.d,
                    0.0, 0.0, 0.0, 1.0;
                transform = transform * jointTransform;
            }

            return transform;
        }

        Eigen::VectorXd poseError(
            const Eigen::Matrix4d& current,
            const Eigen::Matrix4d& desired)
        {
            Eigen::VectorXd error(6);
            error.head<3>() = desired.block<3, 1>(0, 3) - current.block<3, 1>(0, 3);

            const Eigen::Matrix3d currentRotation = current.block<3, 3>(0, 0);
            const Eigen::Matrix3d desiredRotation = desired.block<3, 3>(0, 0);
            const Eigen::Matrix3d rotationError = desiredRotation * currentRotation.transpose();
            const double traceValue = rotationError.trace();
            const double angle = std::acos(std::clamp((traceValue - 1.0) / 2.0, -1.0, 1.0));

            if(angle < 1.0e-10) {
                error.tail<3>() = Eigen::Vector3d::Zero();
            } else {
                Eigen::Vector3d axis;
                axis << rotationError(2, 1) - rotationError(1, 2),
                    rotationError(0, 2) - rotationError(2, 0),
                    rotationError(1, 0) - rotationError(0, 1);
                axis /= 2.0 * std::sin(angle);
                error.tail<3>() = axis * angle;
            }

            return error;
        }

        Eigen::MatrixXd computeJacobianModifiedDh(
            const std::vector<DhParam>& dhParams,
            const std::vector<double>& joints)
        {
            Eigen::MatrixXd jacobian = Eigen::MatrixXd::Zero(6, static_cast<int>(joints.size()));
            std::vector<Eigen::Matrix4d> transforms(joints.size());
            Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();

            for(std::size_t index = 0; index < joints.size() && index < dhParams.size(); ++index) {
                const DhParam& dh = dhParams[index];
                const double theta = joints[index] + dh.thetaOffset;
                const double cosTheta = std::cos(theta);
                const double sinTheta = std::sin(theta);
                const double cosAlpha = std::cos(dh.alpha);
                const double sinAlpha = std::sin(dh.alpha);

                Eigen::Matrix4d jointTransform;
                jointTransform <<
                    cosTheta, -sinTheta, 0.0, dh.a,
                    sinTheta * cosAlpha, cosTheta * cosAlpha, -sinAlpha, -sinAlpha * dh.d,
                    sinTheta * sinAlpha, cosTheta * sinAlpha, cosAlpha, cosAlpha * dh.d,
                    0.0, 0.0, 0.0, 1.0;
                transform = transform * jointTransform;
                transforms[index] = transform;
            }

            if(transforms.empty()) {
                return jacobian;
            }

            const Eigen::Vector3d pEnd = transforms.back().block<3, 1>(0, 3);

            for(std::size_t column = 0; column < joints.size(); ++column) {
                Eigen::Vector3d zAxis;
                Eigen::Vector3d origin;
                if(column == 0) {
                    zAxis << 0.0, 0.0, 1.0;
                    origin << 0.0, 0.0, 0.0;
                } else {
                    zAxis = transforms[column].block<3, 1>(0, 2);
                    origin = transforms[column].block<3, 1>(0, 3);
                }

                jacobian.block<3, 1>(0, static_cast<int>(column)) = zAxis.cross(pEnd - origin);
                jacobian.block<3, 1>(3, static_cast<int>(column)) = zAxis;
            }
            return jacobian;
        }

        struct IkAttempt
        {
            bool success = false;
            std::vector<double> joints;
            double errorNorm = std::numeric_limits<double>::infinity();
        };

        Eigen::Matrix<double, 6, 1> rigidPoseError(
            const Eigen::Matrix4d& current, const Eigen::Matrix4d& desired)
        {
            Eigen::Matrix<double, 6, 1> error;
            error.head<3>() = desired.block<3, 1>(0, 3) - current.block<3, 1>(0, 3);
            const Eigen::AngleAxisd rotation(desired.block<3, 3>(0, 0) *
                current.block<3, 3>(0, 0).transpose());
            error.tail<3>() = rotation.axis() * rotation.angle();
            return error;
        }

        bool rigidTarget(const Eigen::Matrix4d& input, Eigen::Matrix4d& target)
        {
            if(!input.allFinite() ||
                (input.row(3) - Eigen::RowVector4d(0, 0, 0, 1)).norm() > 1.0e-8) { return false; }
            const Eigen::Matrix3d rotation = input.block<3, 3>(0, 0);
            if(rotation.determinant() <= 0.0 ||
                (rotation.transpose() * rotation - Eigen::Matrix3d::Identity()).norm() > 0.05) {
                return false;
            }
            // Imported decimal matrices may not lie exactly on SO(3).
            const Eigen::JacobiSVD<Eigen::Matrix3d> svd(rotation, Eigen::ComputeFullU | Eigen::ComputeFullV);
            target = input;
            target.block<3, 3>(0, 0) = svd.matrixU() * svd.matrixV().transpose();
            return true;
        }

        IkAttempt solveModelPose(const Eigen::Matrix4d& desired,
            const std::vector<double>& seed, const CartesianIkOptions& options,
            const CartesianMultiIkOptions* multi = nullptr)
        {
            IkAttempt attempt;
            attempt.joints = seed;
            constexpr double probe = 1.0e-6;
            for(int iteration = 0; iteration < options.maxIterations; ++iteration) {
                if(multi && multi->cancelled && multi->cancelled()) { return attempt; }
                const Eigen::Matrix4d current = options.worldForwardKinematics(attempt.joints).matrix();
                if(!current.allFinite()) { return attempt; }
                const auto error = rigidPoseError(current, desired);
                attempt.errorNorm = error.norm();
                if(multi ? (error.head<3>().norm() <= multi->positionTolerance &&
                            error.tail<3>().norm() <= multi->orientationTolerance)
                         : attempt.errorNorm <= options.tolerance) {
                    attempt.success = true; return attempt;
                }
                Eigen::Matrix<double, 6, 6> jacobian;
                for(std::size_t column = 0; column < kIrb4600JointCount; ++column) {
                    auto perturbed = attempt.joints;
                    perturbed[column] += probe;
                    const Eigen::Matrix4d pose = options.worldForwardKinematics(perturbed).matrix();
                    if(!pose.allFinite()) { return attempt; }
                    jacobian.col(static_cast<int>(column)) = rigidPoseError(current, pose) / probe;
                }
                const Eigen::Matrix<double, 6, 6> regularizer =
                    options.damping * options.damping * Eigen::Matrix<double, 6, 6>::Identity();
                Eigen::Matrix<double, 6, 1> delta = jacobian.transpose() *
                    (jacobian * jacobian.transpose() + regularizer).ldlt().solve(error);
                if(!delta.allFinite()) { return attempt; }
                // Limit joint steps and backtrack near singularities or distant initial seeds.
                delta *= std::min(options.stepSize, 0.35 / std::max(0.35, delta.cwiseAbs().maxCoeff()));
                bool improved = false;
                for(int backtrack = 0; backtrack < 12; ++backtrack) {
                    auto candidate = attempt.joints;
                    for(std::size_t j = 0; j < candidate.size(); ++j) {
                        candidate[j] += delta(static_cast<int>(j));
                        if(!multi) { candidate[j] = normalizeAngle(candidate[j]); }
                    }
                    const Eigen::Matrix4d pose = options.worldForwardKinematics(candidate).matrix();
                    const double norm = pose.allFinite() ? rigidPoseError(pose, desired).norm()
                        : std::numeric_limits<double>::infinity();
                    if(norm < attempt.errorNorm) {
                        attempt.joints = std::move(candidate);
                        attempt.errorNorm = norm;
                        improved = true;
                        break;
                    }
                    delta *= 0.5;
                }
                if(!improved) { break; }
            }
            attempt.success = attempt.errorNorm <= options.tolerance;
            return attempt;
        }

        IkAttempt solveSinglePose(
            const std::vector<DhParam>& dhParams,
            const Eigen::Matrix4d& desired,
            const std::vector<double>& seed,
            const CartesianIkOptions& options)
        {
            IkAttempt attempt;
            attempt.joints = seed;
            if(attempt.joints.size() != dhParams.size()) {
                attempt.joints.assign(dhParams.size(), 0.0);
            }

            const double dampingSquared = options.damping * options.damping;
            for(int iteration = 0; iteration < options.maxIterations; ++iteration) {
                const Eigen::Matrix4d current = forwardKinematicsModifiedDh(dhParams, attempt.joints);
                const Eigen::VectorXd error = poseError(current, desired);
                attempt.errorNorm = error.norm();
                if(attempt.errorNorm <= options.tolerance) {
                    attempt.success = true;
                    return attempt;
                }

                const Eigen::MatrixXd jacobian = computeJacobianModifiedDh(dhParams, attempt.joints);
                const Eigen::MatrixXd regularizer =
                    dampingSquared * Eigen::MatrixXd::Identity(6, 6);
                Eigen::VectorXd delta =
                    options.stepSize *
                    jacobian.transpose() *
                    (jacobian * jacobian.transpose() + regularizer).ldlt().solve(error);

                if(!delta.allFinite()) {
                    break;
                }

                for(int index = 0; index < delta.size(); ++index) {
                    attempt.joints[static_cast<std::size_t>(index)] =
                        normalizeAngle(attempt.joints[static_cast<std::size_t>(index)] + delta(index));
                }
            }

            const Eigen::Matrix4d finalPose = forwardKinematicsModifiedDh(dhParams, attempt.joints);
            attempt.errorNorm = poseError(finalPose, desired).norm();
            attempt.success = attempt.errorNorm <= options.tolerance;
            return attempt;
        }

        std::vector<std::vector<double>> makeSeedSet(
            const std::vector<double>& preferredSeed,
            const std::vector<double>& previousSolution)
        {
            std::vector<std::vector<double>> seeds;
            auto appendSeed = [&](std::vector<double> seed) {
                if(seed.size() != kIrb4600JointCount) {
                    return;
                }
                for(double& joint : seed) {
                    joint = normalizeAngle(joint);
                }
                if(std::find(seeds.begin(), seeds.end(), seed) == seeds.end()) {
                    seeds.push_back(std::move(seed));
                }
            };

            appendSeed(previousSolution);
            appendSeed(preferredSeed);
            appendSeed({ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 });
            appendSeed({ 0.0, -0.4, 0.6, 0.0, 0.4, 0.0 });
            appendSeed({ 0.0, 0.5, -0.6, 0.0, -0.4, 0.0 });
            appendSeed({ kPi / 2.0, -0.4, 0.6, 0.0, 0.4, 0.0 });
            appendSeed({ -kPi / 2.0, -0.4, 0.6, 0.0, 0.4, 0.0 });
            return seeds;
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

        std::vector<std::string> resolvedJointNames(
            const StoredMotionPlan& plan,
            const CartesianIkOptions& options)
        {
            if(options.jointNames.size() == kIrb4600JointCount) {
                return options.jointNames;
            }
            if(plan.jointNames.size() == kIrb4600JointCount) {
                return plan.jointNames;
            }
            return ProjectTrajectoryInverseKinematics::defaultIrb4600JointNames();
        }

        MotionPlanningDiagnostic diagnostic(std::string code, std::string message)
        {
            MotionPlanningDiagnostic item;
            item.code = std::move(code);
            item.message = std::move(message);
            return item;
        }
    }

    std::size_t CartesianIkResult::solvedPointCount() const
    {
        return static_cast<std::size_t>(std::count_if(
            points.begin(),
            points.end(),
            [](const CartesianIkPointResult& point) {
                return point.success;
            }));
    }

    std::vector<std::string> ProjectTrajectoryInverseKinematics::defaultIrb4600JointNames()
    {
        return { "Joint1", "Joint2", "Joint3", "Joint4", "Joint5", "Joint6" };
    }

    std::vector<double> ProjectTrajectoryInverseKinematics::irb4600RobotSystemJointValues(
        const std::vector<double>& ikJointValues)
    {
        std::vector<double> robotJointValues = ikJointValues;
        if(robotJointValues.size() > 0) {
            robotJointValues[0] = -robotJointValues[0];
        }
        if(robotJointValues.size() > 3) {
            robotJointValues[3] = -robotJointValues[3];
        }
        if(robotJointValues.size() > 4) {
            robotJointValues[4] = -robotJointValues[4];
        }
        if(robotJointValues.size() > 5) {
            robotJointValues[5] = -robotJointValues[5];
        }
        return robotJointValues;
    }

    CartesianIkResult ProjectTrajectoryInverseKinematics::solveCartesianControlPoints(
        const simulation_project::ProjectDocument& document,
        const StoredMotionPlan& plan,
        const CartesianIkOptions& options)
    {
        CartesianIkResult result;
        result.plan = plan;
        result.plan.trajectory.points.clear();
        result.plan.trajectory.name = plan.name.empty() ? plan.id + "_ik" : plan.name + "_ik";
        result.plan.trajectory.interpolation = plan.cartesianControlPoints.interpolation;

        const std::string robotId = !options.robotId.empty() ? options.robotId : plan.robotId;
        if(robotId.empty()) {
            result.diagnostics.push_back(diagnostic("robot_required", "Robot id is required for inverse kinematics."));
            return result;
        }
        if(plan.cartesianControlPoints.empty()) {
            result.diagnostics.push_back(diagnostic(
                "cartesian_points_required",
                "The selected trajectory has no cartesian control points to solve."));
            return result;
        }

        const simulation_project::RobotDesc* robot = findRobot(document, robotId);
        if(robot == nullptr) {
            result.diagnostics.push_back(diagnostic("robot_not_found", "Selected robot is not in the project."));
            return result;
        }

        if(options.maxIterations <= 0 || !std::isfinite(options.tolerance) || options.tolerance <= 0.0 ||
            !std::isfinite(options.stepSize) || options.stepSize <= 0.0 ||
            !std::isfinite(options.damping) || options.damping <= 0.0) {
            result.diagnostics.push_back(diagnostic("invalid_ik_options", "IK parameters must be finite and positive."));
            return result;
        }
        result.plan.robotId = robotId;
        result.plan.jointNames = resolvedJointNames(plan, options);

        const std::vector<DhParam> dhParams = irb4600DhParamsMeters();
        const Eigen::Matrix4d worldFromBase = makeTransform(robot->baseTransform);
        const Eigen::Matrix4d baseFromWorld = worldFromBase.inverse();
        const Eigen::Matrix4d toolTransform = fixedToolTransformMeters();
        std::vector<double> preferredSeed = options.seedJoints;
        if(preferredSeed.size() != kIrb4600JointCount) {
            preferredSeed.assign(kIrb4600JointCount, 0.0);
        }

        std::vector<double> previousSolution;
        result.points.reserve(plan.cartesianControlPoints.points.size());
        result.plan.trajectory.points.reserve(plan.cartesianControlPoints.points.size());

        for(std::size_t pointIndex = 0; pointIndex < plan.cartesianControlPoints.points.size(); ++pointIndex) {
            const robottrajectory::TimedCartesianPoint& cartesianPoint =
                plan.cartesianControlPoints.points[pointIndex];
            const Eigen::Matrix4d baseFromTarget = baseFromWorld * cartesianPoint.tcpPose.matrix();
            const Eigen::Matrix4d flangeTarget = options.toolMode == CartesianIkToolMode::FixedTool
                ? baseFromTarget * toolTransform.inverse()
                : baseFromTarget;

            CartesianIkPointResult pointResult;
            pointResult.pointIndex = pointIndex;

            Eigen::Matrix4d worldTarget;
            if(options.worldForwardKinematics && !rigidTarget(cartesianPoint.tcpPose.matrix(), worldTarget)) {
                pointResult.message = "Invalid cartesian target: expected a finite rigid pose.";
                result.diagnostics.push_back(diagnostic("invalid_ik_pose", pointResult.message));
                result.points.push_back(std::move(pointResult));
                continue;
            }
            IkAttempt bestAttempt;
            const std::vector<std::vector<double>> seeds = makeSeedSet(
                preferredSeed,
                options.usePreviousSolutionAsSeed ? previousSolution : std::vector<double>());
            for(const std::vector<double>& seed : seeds) {
                IkAttempt attempt = options.worldForwardKinematics
                    ? solveModelPose(worldTarget, seed, options)
                    : solveSinglePose(dhParams, flangeTarget, seed, options);
                if(attempt.errorNorm < bestAttempt.errorNorm) {
                    bestAttempt = std::move(attempt);
                }
                if(bestAttempt.success) {
                    break;
                }
            }

            pointResult.success = bestAttempt.success;
            pointResult.joints = bestAttempt.joints;
            pointResult.errorNorm = bestAttempt.errorNorm;
            if(!pointResult.success) {
                std::ostringstream stream;
                stream << "IK failed for control point " << (pointIndex + 1)
                    << " with final error " << pointResult.errorNorm << ".";
                pointResult.message = stream.str();
                result.diagnostics.push_back(diagnostic("ik_point_failed", pointResult.message));
                result.points.push_back(std::move(pointResult));
                continue;
            }

            robottrajectory::TimedJointPoint jointPoint;
            jointPoint.time = cartesianPoint.time;
            jointPoint.q = pointResult.joints;
            result.plan.trajectory.points.push_back(std::move(jointPoint));

            previousSolution = pointResult.joints;
            preferredSeed = pointResult.joints;
            result.points.push_back(std::move(pointResult));
        }

        result.success = result.plan.trajectory.points.size() == plan.cartesianControlPoints.points.size();
        if(!result.success && result.diagnostics.empty()) {
            result.diagnostics.push_back(diagnostic("ik_failed", "Inverse kinematics failed."));
        }
        return result;
    }

    bool ProjectTrajectoryInverseKinematics::applyJointValuesToRobotInitialJoints(
        simulation_project::ProjectDocument& document,
        const std::string& robotId,
        const std::vector<std::string>& jointNames,
        const std::vector<double>& jointValues,
        std::string* errorMessage)
    {
        if(robotId.empty()) {
            if(errorMessage != nullptr) {
                *errorMessage = "Robot id is required.";
            }
            return false;
        }
        if(jointNames.size() != jointValues.size()) {
            if(errorMessage != nullptr) {
                *errorMessage = "Joint names must match joint values.";
            }
            return false;
        }

        auto robotIt = std::find_if(
            document.robots.begin(),
            document.robots.end(),
            [&](const simulation_project::RobotDesc& robot) {
                return robot.id == robotId;
            });
        if(robotIt == document.robots.end()) {
            if(errorMessage != nullptr) {
                *errorMessage = "Selected robot is not in the project.";
            }
            return false;
        }

        robotIt->initialJoints.erase(
            std::remove_if(
                robotIt->initialJoints.begin(),
                robotIt->initialJoints.end(),
                [&](const simulation_project::JointValueDesc& joint) {
                    return std::find(jointNames.begin(), jointNames.end(), joint.jointName) != jointNames.end();
                }),
            robotIt->initialJoints.end());

        for(std::size_t index = 0; index < jointNames.size(); ++index) {
            simulation_project::JointValueDesc joint;
            joint.jointName = jointNames[index];
            joint.value = jointValues[index];
            robotIt->initialJoints.push_back(std::move(joint));
        }
        return true;
    }

    bool ProjectTrajectoryInverseKinematics::readRevoluteJointLimits(
        const simulation_project::ProjectDocument& document,
        const std::filesystem::path& basePath, const std::string& robotId,
        const std::vector<std::string>& names, std::vector<double>& lower,
        std::vector<double>& upper, std::string& error)
    {
        simulation_runtime::ProjectSimulationRuntime runtime;
        const auto loaded = runtime.loadProject(document, basePath);
        if(!loaded.success) { error = loaded.message; return false; }
        const auto* item = runtime.robot(robotId);
        if(!item) { error = "Robot model not available."; return false; }
        lower.clear(); upper.clear();
        for(const auto& name : names) {
            const auto it = std::find_if(item->model.joints.begin(), item->model.joints.end(),
                [&](const robot::RobotJoint& joint) { return joint.name == name; });
            if(it == item->model.joints.end() || it->type != robot::JointType::Revolute ||
                it->isLoop || it->dofIndex < 0) {
                error = "Multi IK requires six independent revolute joints: " + name;
                return false;
            }
            if(!it->continuous && (!it->hasPositionLimits ||
                !std::isfinite(it->lowerPositionLimit) || !std::isfinite(it->upperPositionLimit) ||
                it->lowerPositionLimit > it->upperPositionLimit)) {
                error = "Missing joint position limits: " + name; return false;
            }
            lower.push_back(it->continuous ? -std::numeric_limits<double>::infinity() : it->lowerPositionLimit);
            upper.push_back(it->continuous ? std::numeric_limits<double>::infinity() : it->upperPositionLimit);
        }
        return true;
    }

    CartesianMultiIkResult ProjectTrajectoryInverseKinematics::solveAllCartesianControlPoints(
        const StoredMotionPlan& plan, const CartesianMultiIkOptions& options)
    {
        CartesianMultiIkResult result;
        result.source = plan;
        result.source.jointNames = options.model.jointNames;
        result.source.trajectory.points.clear();
        const auto cancelled = [&]() { return options.cancelled && options.cancelled(); };
        const auto invalid = [&](const std::string& message) {
            result.message = message; return result;
        };
        if(!options.model.worldForwardKinematics || options.model.jointNames.size() != 6 ||
            options.lower.size() != 6 || options.upper.size() != 6 || plan.cartesianControlPoints.empty() ||
            options.seedCount < 1 || options.seedCount > 4096 || options.maxIterations < 1 ||
            options.maxCandidatesPerPoint < 1 || options.maxCandidatesPerPoint > 65536 ||
            !std::isfinite(options.positionTolerance) || options.positionTolerance <= 0 ||
            !std::isfinite(options.orientationTolerance) || options.orientationTolerance <= 0 ||
            !std::isfinite(options.duplicateTolerance) || options.duplicateTolerance <= 0) {
            return invalid("Multi IK requires actual FK, six joints, finite bounds and positive search settings.");
        }
        for(std::size_t j = 0; j < 6; ++j) {
            if(!std::isfinite(options.lower[j]) || !std::isfinite(options.upper[j]) ||
                options.lower[j] > options.upper[j] || std::abs(options.lower[j]) > 1.0e6 ||
                std::abs(options.upper[j]) > 1.0e6) { return invalid("Invalid finite joint search range."); }
        }
        auto solver = options.model;
        solver.maxIterations = options.maxIterations;
        solver.stepSize = 1.0;
        solver.damping = 0.001;
        solver.tolerance = std::min(options.positionTolerance, options.orientationTolerance);
        const auto radicalInverse = [](int n, int base) {
            double value = 0, fraction = 1.0 / base;
            while(n > 0) { value += (n % base) * fraction; n /= base; fraction /= base; }
            return value;
        };
        const int primes[] = {2, 3, 5, 7, 11, 13};
        std::vector<std::vector<double>> previousRoots;
        std::size_t solved = 0, total = 0, truncated = 0;
        for(std::size_t index = 0; index < plan.cartesianControlPoints.points.size(); ++index) {
            if(cancelled()) { result.cancelled = true; break; }
            const auto& point = plan.cartesianControlPoints.points[index];
            CartesianIkLayer layer;
            layer.pointIndex = index; layer.time = point.time;
            Eigen::Matrix4d desired;
            if(!std::isfinite(point.time) || !rigidTarget(point.tcpPose.matrix(), desired)) {
                layer.message = "Invalid target pose/time.";
            } else {
                std::vector<std::vector<double>> seeds = previousRoots;
                if(solver.seedJoints.size() == 6 && std::all_of(solver.seedJoints.begin(), solver.seedJoints.end(),
                    [](double value) { return std::isfinite(value); })) { seeds.push_back(solver.seedJoints); }
                for(int n = 1; n <= options.seedCount; ++n) {
                    std::vector<double> seed(6);
                    for(int j = 0; j < 6; ++j) { seed[j] = -kPi + 2 * kPi * radicalInverse(n, primes[j]); }
                    seeds.push_back(std::move(seed));
                }
                std::vector<std::vector<double>> roots;
                for(const auto& seed : seeds) {
                    if(cancelled()) { result.cancelled = true; break; }
                    const auto attempt = solveModelPose(desired, seed, solver, &options);
                    if(!attempt.success) { continue; }
                    auto root = attempt.joints;
                    for(auto& q : root) { q = normalizeAngle(q); }
                    const bool duplicate = std::any_of(roots.begin(), roots.end(), [&](const auto& other) {
                        for(std::size_t j = 0; j < 6; ++j) {
                            if(std::abs(std::remainder(root[j] - other[j], 2 * kPi)) > options.duplicateTolerance) { return false; }
                        }
                        return true;
                    });
                    if(duplicate) { continue; }
                    // A singular pose can have an infinite family: retain a bounded numerical sample.
                    if(roots.size() >= 64) { layer.truncated = true; break; }
                    roots.push_back(root);
                    std::vector<double> lifted(6);
                    std::vector<int> turns(6);
                    std::function<void(std::size_t)> expand = [&](std::size_t j) {
                        if(cancelled() || layer.truncated) { return; }
                        if(j < 6) {
                            const int first = static_cast<int>(std::ceil((options.lower[j] - root[j] - 1.0e-10) / (2 * kPi)));
                            const int last = static_cast<int>(std::floor((options.upper[j] - root[j] + 1.0e-10) / (2 * kPi)));
                            for(int turn = first; turn <= last && !layer.truncated && !cancelled(); ++turn) {
                                lifted[j] = std::clamp(root[j] + 2 * kPi * turn, options.lower[j], options.upper[j]);
                                turns[j] = static_cast<int>(std::floor((lifted[j] + kPi) / (2 * kPi)));
                                expand(j + 1);
                            }
                            return;
                        }
                        const auto actual = solver.worldForwardKinematics(lifted).matrix().eval();
                        if(!actual.allFinite()) { return; }
                        const auto error = rigidPoseError(actual, desired);
                        if(error.head<3>().norm() > options.positionTolerance ||
                            error.tail<3>().norm() > options.orientationTolerance) { return; }
                        if(layer.candidates.size() >= options.maxCandidatesPerPoint) { layer.truncated = true; return; }
                        layer.candidates.push_back({lifted, turns, error.head<3>().norm(), error.tail<3>().norm()});
                    };
                    expand(0);
                    if(layer.truncated) { break; }
                }
                previousRoots = std::move(roots);
                std::sort(layer.candidates.begin(), layer.candidates.end(), [](const auto& a, const auto& b) {
                    return a.joints < b.joints;
                });
                layer.message = layer.candidates.empty() ? "No valid root found within search budget/range." :
                    (layer.truncated ? "Candidate budget reached; partial enumeration." : "Numerical candidates; completeness not guaranteed.");
            }
            if(result.cancelled || cancelled()) { result.cancelled = true; break; }
            if(!layer.candidates.empty()) { ++solved; }
            if(layer.truncated) { ++truncated; }
            total += layer.candidates.size();
            result.layers.push_back(std::move(layer));
            if(options.progress) { options.progress(index + 1, plan.cartesianControlPoints.points.size()); }
        }
        result.success = !result.cancelled && solved == plan.cartesianControlPoints.points.size();
        std::ostringstream summary;
        summary << (result.cancelled ? "Cancelled. " : "Search finished. ") << solved << "/"
            << plan.cartesianControlPoints.points.size() << " points, " << total << " candidates, "
            << truncated << " truncated layers. Numerical search; completeness is not guaranteed.";
        result.message = summary.str();
        return result;
    }

    bool ProjectTrajectoryInverseKinematics::selectMultiIkTrajectory(const CartesianMultiIkResult& result,
        const std::vector<std::size_t>& selections, StoredMotionPlan& plan, std::string& error)
    {
        if(result.cancelled || result.layers.empty() || selections.size() != result.layers.size() ||
            result.layers.size() != result.source.cartesianControlPoints.points.size()) {
            error = "Incomplete multi IK result; every control point requires a selected solution."; return false;
        }
        StoredMotionPlan selected = result.source;
        selected.trajectory.points.clear();
        for(std::size_t i = 0; i < result.layers.size(); ++i) {
            const auto& layer = result.layers[i];
            if(layer.pointIndex != i || selections[i] >= layer.candidates.size()) {
                error = "Missing solution at control point " + std::to_string(i + 1); return false;
            }
            robottrajectory::TimedJointPoint point;
            point.time = layer.time;
            point.q = layer.candidates[selections[i]].joints;
            selected.trajectory.points.push_back(std::move(point));
        }
        selected.trajectory.interpolation = robottrajectory::TrajectoryInterpolation::Linear;
        plan = std::move(selected);
        return true;
    }

}
