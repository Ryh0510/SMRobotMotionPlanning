#pragma once
#include <ProjectMotionPlanning/ProjectMotionPlanning.h>
#include <memory>

namespace motion_planning {
// Immutable actual-model TCP interpolation, shared by planning, replay and display.
class CartesianMotionSampler {
public:
    static std::shared_ptr<const CartesianMotionSampler> create(ProjectPlanningSceneSnapshot& scene,
        const std::string& tcpLink, const Eigen::Isometry3d& linkFromTcp, std::string& error);
    static bool isMoveL(const StoredMotionPlan& plan);
    ~CartesianMotionSampler();
    Eigen::Isometry3d forward(const std::vector<double>& q) const;
    Eigen::Matrix<double,6,6> jacobian(const std::vector<double>& q) const;
    bool sample(const std::vector<double>& a, const std::vector<double>& b, double fraction,
        std::vector<double>& q, std::string& error) const;
    // Exactly k/intervals, including both original encoder endpoints. GPU
    // proposals are CPU-FK checked; any failure uses the ordinary CPU sampler.
    bool uniformSamples(const std::vector<double>& a, const std::vector<double>& b,
        std::size_t intervals, std::vector<std::vector<double>>& samples, std::string& error,
        const std::function<bool()>& cancelled = {}) const;
    bool connection(const std::vector<double>& a, const std::vector<double>& b,
        double maxJointStep, std::size_t maxSamples, std::vector<std::vector<double>>& samples,
        std::string& error, const std::function<bool()>& cancelled = {}) const;
    bool execution(const robottrajectory::JointTrajectory& input, robottrajectory::JointTrajectory& output,
        std::string& error, double maxJointStep = .005, const std::function<bool()>& cancelled = {}) const;
private:
    CartesianMotionSampler();
    struct Model;
    std::unique_ptr<Model> m_model;
};
}
