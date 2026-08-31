#include <ProjectMotionPlanning/TrajectoryInverseKinematics.h>

#include <SimulationProject/ProjectDocument.h>

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

            IkAttempt bestAttempt;
            const std::vector<std::vector<double>> seeds = makeSeedSet(
                preferredSeed,
                options.usePreviousSolutionAsSeed ? previousSolution : std::vector<double>());
            for(const std::vector<double>& seed : seeds) {
                IkAttempt attempt = solveSinglePose(dhParams, flangeTarget, seed, options);
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
}
