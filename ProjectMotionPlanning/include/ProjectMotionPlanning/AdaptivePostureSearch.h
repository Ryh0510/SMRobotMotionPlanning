#pragma once
#include <Eigen/Geometry>
#include <functional>
#include <limits>
#include <string>
#include <vector>

namespace motion_planning {
struct AdaptivePostureOptions {
    int pointEvaluations = 256;
    int localIterations = 24;
    int candidateLimit = 12;
    int graphRefinementRounds = 4;
    double continuityWeight = 2.;
    double postureWeight = .2;
};
struct PostureConstraintFeedback {
    bool feasible = false;
    // Actual constraint violation, when available. Unavailable is infinity.
    double violation = std::numeric_limits<double>::infinity();
    double cost = 0.;
};
struct AdaptivePostureSearchReport {
    int evaluations = 0, localEvaluations = 0, safePostures = 0;
    std::string stopReason;
};
// Three independent coordinates: a two-dimensional normal-tilt disk and a
// periodic axial phase in [-1,1). Acceptance is exclusively the caller's oracle.
class AdaptivePostureSearch {
public:
    using X = Eigen::Vector3d;
    using Oracle = std::function<PostureConstraintFeedback(const X&)>;
    static X repair(X x, bool axialSpin);
    static Eigen::Quaterniond rotation(const X&, const Eigen::Vector3d& axis, double maxTilt, bool axialSpin);
    static X coordinates(const Eigen::Quaterniond& relative, const Eigen::Vector3d& axis, double maxTilt, bool axialSpin);
    static AdaptivePostureSearchReport search(const AdaptivePostureOptions&, const std::vector<X>& seeds,
        bool axialSpin, const Oracle&, const std::function<bool()>& stopped);
};
}
