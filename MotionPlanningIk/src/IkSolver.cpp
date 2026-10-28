#include <MotionPlanningIk/IkSolver.h>

#include <RobotInstance/RobotInstance.h>

#include <Eigen/Cholesky>
#include <Eigen/SVD>

#include <algorithm>
#include <cmath>
#include <limits>

namespace motion_planning::ik
{
    namespace
    {
        Eigen::Vector<double, 6> poseError(
            const Eigen::Isometry3d& actual,
            const Eigen::Isometry3d& target)
        {
            const Eigen::Vector3d position = target.translation() - actual.translation();
            const Eigen::Matrix3d rotationError = actual.linear().transpose() * target.linear();
            Eigen::AngleAxisd angle(rotationError);
            Eigen::Vector3d angular = Eigen::Vector3d::Zero();
            if(angle.angle() >= 1.0e-9)
                angular = actual.linear() * (angle.axis() * angle.angle());
            Eigen::Vector<double, 6> result;
            result.head<3>() = position;
            result.tail<3>() = angular;
            return result;
        }

        Eigen::Vector<double, 6> poseDelta(
            const Eigen::Isometry3d& from,
            const Eigen::Isometry3d& to)
        {
            Eigen::Vector<double, 6> result;
            result.head<3>() = to.translation() - from.translation();
            const Eigen::AngleAxisd angle(from.linear().transpose() * to.linear());
            result.tail<3>() = Eigen::Vector3d::Zero();
            if(angle.angle() >= 1.0e-9)
                result.tail<3>() = from.linear() * (angle.axis() * angle.angle());
            return result;
        }

        bool converged(
            const Eigen::Vector<double, 6>& error,
            const IkOptions& options)
        {
            return error.head<3>().norm() <= options.positionToleranceMeters &&
                error.tail<3>().norm() <= options.orientationToleranceRadians;
        }
    }

    IkResult IkSolver::solve(const IkRequest& request)
    {
        IkResult result;
        if(request.robotModel == nullptr || request.targets.empty() ||
            request.tcpLinkName.empty() || request.jointNames.empty() ||
            request.seed.size() != request.jointNames.size()) {
            result.diagnostics.push_back({ 0, "The IK request is incomplete.", 0.0, 0.0 });
            return result;
        }

        const robot::RobotModel& model = *request.robotModel;
        robotinstance::RobotInstance instance(model, "motion_planning_ik");
        instance.setBaseTransform(request.baseTransform);
        std::vector<double> q = request.seed;
        instance.setJoints(q);
        instance.update();

        std::vector<int> dofIndices;
        dofIndices.reserve(request.jointNames.size());
        for(const std::string& name : request.jointNames) {
            const auto jointIt = model.jointNameToIndex.find(name);
            if(jointIt == model.jointNameToIndex.end() ||
                model.joints[static_cast<std::size_t>(jointIt->second)].dofIndex < 0) {
                result.diagnostics.push_back({ 0, "Joint is not present in the robot model: " + name, 0.0, 0.0 });
                return result;
            }
            dofIndices.push_back(model.joints[static_cast<std::size_t>(jointIt->second)].dofIndex);
        }

        result.trajectory.name = "Cartesian IK trajectory";
        result.trajectory.interpolation = robottrajectory::TrajectoryInterpolation::Linear;
        for(std::size_t targetIndex = 0; targetIndex < request.targets.size(); ++targetIndex) {
            const IkTarget& target = request.targets[targetIndex];
            bool targetConverged = false;
            Eigen::Vector<double, 6> error = Eigen::Vector<double, 6>::Zero();
            for(std::size_t iteration = 0; iteration < request.options.maxIterations; ++iteration) {
                instance.setJoints(q);
                instance.update();
                const Eigen::Isometry3d actualTcp =
                    instance.getLinkTransform(request.tcpLinkName) * request.linkFromTcp;
                error = poseError(actualTcp, target.baseFromTcp);
                if(converged(error, request.options)) {
                    targetConverged = true;
                    break;
                }

                Eigen::Matrix<double, 6, Eigen::Dynamic> jacobian(6, q.size());
                jacobian.setZero();
                for(std::size_t jointIndex = 0; jointIndex < q.size(); ++jointIndex) {
                    std::vector<double> perturbed = q;
                    perturbed[jointIndex] += request.options.finiteDifferenceStep;
                    instance.setJoints(perturbed);
                    instance.update();
                    const Eigen::Isometry3d perturbedTcp =
                        instance.getLinkTransform(request.tcpLinkName) * request.linkFromTcp;
                    jacobian.col(static_cast<Eigen::Index>(jointIndex)) =
                        poseDelta(actualTcp, perturbedTcp) /
                        request.options.finiteDifferenceStep;
                }
                instance.setJoints(q);
                instance.update();
                Eigen::MatrixXd reduced = jacobian;
                const Eigen::Matrix<double, 6, 6> regularized =
                    reduced * reduced.transpose() +
                    request.options.damping * request.options.damping *
                    Eigen::Matrix<double, 6, 6>::Identity();
                const Eigen::VectorXd delta =
                    reduced.transpose() * regularized.ldlt().solve(error);
                for(std::size_t jointIndex = 0; jointIndex < q.size(); ++jointIndex) {
                    q[jointIndex] += delta[static_cast<Eigen::Index>(jointIndex)];
                }
                for(std::size_t jointIndex = 0; jointIndex < q.size(); ++jointIndex) {
                    const robot::RobotJoint& joint = model.joints[
                        static_cast<std::size_t>(model.jointNameToIndex.at(request.jointNames[jointIndex]))];
                    if(joint.hasPositionLimits && !joint.continuous) {
                        q[jointIndex] = std::clamp(q[jointIndex], joint.lowerPositionLimit, joint.upperPositionLimit);
                    }
                }
            }

            result.maximumPositionErrorMeters = std::max(
                result.maximumPositionErrorMeters, error.head<3>().norm());
            result.maximumOrientationErrorRadians = std::max(
                result.maximumOrientationErrorRadians, error.tail<3>().norm());
            if(!targetConverged) {
                result.diagnostics.push_back({
                    targetIndex,
                    "Inverse kinematics did not converge.",
                    error.head<3>().norm(),
                    error.tail<3>().norm() });
                return result;
            }
            robottrajectory::TimedJointPoint point;
            point.time = target.timeSeconds;
            point.q = q;
            result.trajectory.points.push_back(std::move(point));
            ++result.solvedTargetCount;
        }
        result.success = true;
        return result;
    }
}
