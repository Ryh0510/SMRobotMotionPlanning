#pragma once
#include "GpuBaffleSearch.h"
#include <ProjectMotionPlanning/TrajectoryInverseKinematics.h>
#include <RobotCore/RobotModel.h>
namespace motion_planning { namespace detail {
class GpuSerialIk {
public:
    bool configure(const robot::RobotModel&,const std::vector<std::string>& names,const std::string& tcpLink,const CartesianMultiIkOptions&);
    bool solve(const std::vector<Eigen::Isometry3d>& targets,const std::vector<std::vector<double>>& seeds,
        std::vector<std::vector<CartesianIkCandidate>>& candidates,bool preferNearSeed=false);
    nlohmann::json status() const;
    // CPU product-of-exponentials chain, validated against the original runtime.
    // Numerical proposals only: accepted roots still require original-model FK.
    Eigen::Isometry3d forward(const std::vector<double>& joints) const;
    Eigen::Matrix<double,6,6> jacobian(const std::vector<double>& joints) const;
private:
    GpuSerialIkModel model_{};
    CartesianMultiIkOptions options_;
    GpuBaffleSearch gpu_;
    bool configured_=false;
    std::string modelError_;
    double modelErrorMax_=0;
    std::size_t cpuFkRejected_=0,verifiedRoots_=0;
};
}}
