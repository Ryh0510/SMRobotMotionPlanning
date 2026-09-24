#include <ProjectMotionPlanning/LayeredIkGraph.h>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <set>

using namespace motion_planning;
namespace
{
    int failures = 0;
    void check(bool value, const char* message)
    {
        std::cout << (value ? "[OK] " : "[FAIL] ") << message << '\n';
        if(!value) { ++failures; }
    }
    CartesianMultiIkResult fixture(const std::vector<std::vector<std::vector<double>>>& q)
    {
        CartesianMultiIkResult ik;
        ik.success = true;
        for(std::size_t j = 0; j < q[0][0].size(); ++j) { ik.source.jointNames.push_back("J" + std::to_string(j)); }
        for(std::size_t i = 0; i < q.size(); ++i) {
            CartesianIkLayer layer; layer.pointIndex = i; layer.time = static_cast<double>(i);
            for(const auto& value : q[i]) {
                CartesianIkCandidate candidate; candidate.joints = value;
                for(double v : value) { candidate.turns.push_back(static_cast<int>(std::floor((v + 3.141592653589793) / 6.283185307179586))); }
                layer.candidates.push_back(candidate);
            }
            ik.layers.push_back(layer);
            robottrajectory::TimedCartesianPoint point; point.time = layer.time;
            ik.source.cartesianControlPoints.points.push_back(point);
        }
        return ik;
    }
    double cost(const CartesianMultiIkResult& ik, const std::vector<std::size_t>& path, const std::vector<double>& weights)
    {
        double sum = 0;
        for(std::size_t i = 1; i < path.size(); ++i) {
            const auto& a = ik.layers[i - 1].candidates[path[i - 1]].joints;
            const auto& b = ik.layers[i].candidates[path[i]].joints;
            for(std::size_t j = 0; j < a.size(); ++j) { sum += weights[j] * std::pow(b[j] - a[j], 2); }
        }
        return sum;
    }
    std::vector<LayeredIkPath> enumerate(const CartesianMultiIkResult& ik, const std::vector<double>& weights)
    {
        std::vector<LayeredIkPath> paths;
        std::vector<std::size_t> sequence(ik.layers.size());
        std::function<void(std::size_t)> walk = [&](std::size_t i) {
            if(i == sequence.size()) { paths.push_back({cost(ik, sequence, weights), sequence}); return; }
            for(std::size_t c = 0; c < ik.layers[i].candidates.size(); ++c) { sequence[i] = c; walk(i + 1); }
        };
        walk(0);
        std::sort(paths.begin(), paths.end(), [](const auto& a, const auto& b) { return a.cost < b.cost; });
        return paths;
    }
}
int main()
{
    const auto ik = fixture({{{0,0},{2,1}}, {{1,0},{-1,3},{4,-2}}, {{0.5,2},{-2,-1}}, {{0,0},{3,4}}});
    LayeredIkGraphOptions options; options.jointWeights = {2.0, 0.3};
    const auto brute = enumerate(ik, options.jointWeights);
    for(std::size_t m : {std::size_t(1), std::size_t(7), std::size_t(30)}) {
        options.maxPaths = m;
        const auto result = ProjectLayeredIkGraph::filter(ik, options);
        bool valid = result.success && result.paths.size() == std::min(m, brute.size());
        std::set<std::vector<std::size_t>> distinct;
        for(std::size_t i = 0; valid && i < result.paths.size(); ++i) {
            const auto& path = result.paths[i];
            valid &= path.selections.size() == ik.layers.size() && distinct.insert(path.selections).second;
            valid &= std::abs(path.cost - brute[i].cost) < 1.0e-10;
            valid &= std::abs(path.cost - cost(ik, path.selections, options.jointWeights)) < 1.0e-10;
        }
        check(valid, "Top-M matches exhaustive enumeration, sorted costs and distinct complete sequences");
    }
    options.jointWeights = {0,0}; options.maxPaths = 30;
    const auto tied = ProjectLayeredIkGraph::filter(ik, options);
    const auto repeat = ProjectLayeredIkGraph::filter(ik, options);
    bool deterministic = tied.paths.size() == brute.size();
    std::set<std::vector<std::size_t>> distinct;
    for(std::size_t i = 0; i < tied.paths.size(); ++i) {
        deterministic &= tied.paths[i].cost == 0 && distinct.insert(tied.paths[i].selections).second &&
            tied.paths[i].selections == repeat.paths[i].selections;
    }
    check(deterministic, "Zero-weight ties retain every combination with deterministic ordering");
    const auto turns = fixture({{{0.0}}, {{6.283185307179586}, {0.2}}});
    options.jointWeights = {1}; options.maxPaths = 2;
    const auto turnResult = ProjectLayeredIkGraph::filter(turns, options);
    check(turnResult.success && turnResult.paths[0].selections[1] == 1 &&
        std::abs(turnResult.paths[1].cost - 39.47841760435743) < 1.0e-10,
        "Full-turn displacement is not wrapped to zero or pruned");
    StoredMotionPlan selected; std::string error;
    check(ProjectTrajectoryInverseKinematics::selectMultiIkTrajectory(turns, turnResult.paths[1].selections, selected, error) &&
        selected.trajectory.points[1].q[0] == 6.283185307179586 && selected.trajectory.points[1].time == 1,
        "Backtracked seed preserves unwrapped values and original times");
    auto single = fixture({{{0},{1},{2}}});
    options.maxPaths = 20;
    const auto oneLayer = ProjectLayeredIkGraph::filter(single, options);
    check(oneLayer.success && oneLayer.paths.size() == 3 && oneLayer.paths.back().cost == 0,
        "One layer has one zero-cost path per candidate without node costs");
    auto invalid = turns; invalid.layers[1].candidates.clear();
    check(!ProjectLayeredIkGraph::filter(invalid, options).success, "Empty layers cannot be skipped");
    invalid = turns; invalid.cancelled = true;
    check(!ProjectLayeredIkGraph::filter(invalid, options).success, "Incomplete IK cannot be filtered");
    invalid = turns; invalid.layers[0].candidates[0].joints[0] = std::numeric_limits<double>::quiet_NaN();
    check(!ProjectLayeredIkGraph::filter(invalid, options).success, "Non-finite candidate is rejected");
    options.jointWeights = {-1};
    check(!ProjectLayeredIkGraph::filter(turns, options).success, "Negative weights are rejected");
    options.jointWeights = {1,2};
    check(!ProjectLayeredIkGraph::filter(turns, options).success, "Weight dimension mismatch is rejected");
    options.jointWeights = {1}; options.maxStoredPrefixes = 1;
    const auto limited = ProjectLayeredIkGraph::filter(turns, options);
    check(!limited.success && limited.paths.empty(), "Insufficient memory budget fails explicitly without graph pruning");
    options.maxStoredPrefixes = 8000000;
    bool stop = false; options.cancelled = [&]() { return stop; };
    options.progress = [&](std::size_t, std::size_t) { stop = true; };
    const auto cancelled = ProjectLayeredIkGraph::filter(turns, options);
    check(cancelled.cancelled && !cancelled.success && cancelled.paths.empty(), "In-flight cancellation never publishes partial Top-M");
    return failures ? 1 : 0;
}
