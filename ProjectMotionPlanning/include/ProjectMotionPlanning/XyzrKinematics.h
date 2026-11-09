#pragma once

#include <ProjectMotionPlanning/TrajectoryInverseKinematics.h>
#include <memory>

namespace motion_planning
{
    // Immutable actual-model PPP+R solver. Translation joints use metres; R uses radians.
    class XyzrKinematics
    {
    public:
        // Non-PPP+R models return null without an error so existing solvers remain selected.
        static std::shared_ptr<const XyzrKinematics> create(
            const simulation_project::ProjectDocument& document,
            const std::filesystem::path& basePath, const std::string& robotId,
            CartesianIkToolMode toolMode, std::string& error);

        const std::vector<std::string>& jointNames() const { return m_names; }
        const std::vector<double>& lower() const { return m_lower; }
        const std::vector<double>& upper() const { return m_upper; }
        CartesianIkToolMode toolMode() const { return m_toolMode; }
        Eigen::Isometry3d forward(const std::vector<double>& q) const;
        bool solvePose(const Eigen::Isometry3d& target, const std::vector<double>& seed,
            std::vector<double>& q, std::string& error, double tolerance = 1e-6) const;
        CartesianIkResult solve(const StoredMotionPlan& plan, const CartesianIkOptions& options) const;
        bool sample(const std::vector<double>& from, const std::vector<double>& to,
            double fraction, std::vector<double>& q, std::string& error) const;

    private:
        std::vector<std::string> m_names;
        std::vector<double> m_lower, m_upper;
        CartesianIkToolMode m_toolMode = CartesianIkToolMode::FixedTool;
        Eigen::Matrix3d m_translation = Eigen::Matrix3d::Identity();
        Eigen::Matrix3d m_inverseTranslation = Eigen::Matrix3d::Identity();
        Eigen::Matrix3d m_homeRotation = Eigen::Matrix3d::Identity();
        Eigen::Vector3d m_axis = Eigen::Vector3d::UnitZ();
        Eigen::Vector3d m_center = Eigen::Vector3d::Zero();
        Eigen::Vector3d m_cosine = Eigen::Vector3d::Zero();
        Eigen::Vector3d m_sine = Eigen::Vector3d::Zero();
    };
}
