#include "../../src/ApfLocalPlanner.h"
#include "../../src/PathRefinement.h"

#include <cmath>
#include <iostream>
#include <limits>

using namespace motion_planning::detail;

int main()
{
    ApfOracle oracle;
    oracle.distance = [](const ApfState& q) { return std::hypot(q[0], q[1]) - 0.3; };
    oracle.motionValid = [&](const ApfState& a, const ApfState& b) {
        for(int i = 0; i <= 400; ++i) {
            ApfState q = a;
            for(std::size_t j = 0; j < q.size(); ++j) q[j] += (b[j] - a[j]) * i / 400.0;
            if(oracle.distance(q) < 0.0) return false;
        }
        return true;
    };
    const ApfPath reference{{-1.0, 0.0}, {0.0, 0.0}, {1.0, 0.0}};
    const ApfState lower{-1.5, -1.5}, upper{1.5, 1.5};
    ApfPath path, repeat;
    if(!planApfPath(reference, lower, upper, oracle, 0.02, &path)) {
        std::cerr << "Failed symmetric obstacle / tangential escape\n";
        return 1;
    }
    if(path.front() != reference.front() || path.back() != reference.back()) return 2;
    for(std::size_t i = 0; i + 1 < path.size(); ++i)
        if(!oracle.motionValid(path[i], path[i + 1])) return 3;
    if(!planApfPath(reference, lower, upper, oracle, 0.02, &repeat) || repeat != path) return 4;
    const double originalLength = jointPathLength(path);
    shortcutApfPath(&path, oracle);
    if(path.front() != reference.front() || path.back() != reference.back() ||
        jointPathLength(path) > originalLength + 1.0e-10) return 9;
    for(std::size_t i = 1; i < path.size(); ++i)
        if(!oracle.motionValid(path[i - 1], path[i])) return 10;
    // Smooth a noisy arc around the obstacle. Every accepted edge must still
    // clear the circle, even though its endpoint chord intersects the circle.
    ApfPath arc;
    std::vector<double> times;
    for(int i = 0; i <= 64; ++i) {
        const double angle = 3.14159265358979323846 * i / 64.0;
        const double radius = 0.38 + 0.025 * std::sin(13.0 * angle);
        arc.push_back({-radius * std::cos(angle), radius * std::sin(angle)});
        times.push_back(i * 0.1 + i * i * 0.0001);
    }
    const auto initialArc = arc;
    const double initialBending = jointPathBending(arc, times);
    if(!smoothValidatedPath(&arc, times, oracle, 3, 0.5, 0.15) ||
        jointPathBending(arc, times) >= initialBending ||
        jointPathLength(arc) > jointPathLength(initialArc) + 1.0e-10 ||
        arc.front() != initialArc.front() || arc.back() != initialArc.back()) return 11;
    for(std::size_t i = 1; i < arc.size(); ++i)
        if(!oracle.motionValid(arc[i - 1], arc[i])) return 12;
    const auto preserved = arc;
    ApfOracle blocked = oracle;
    blocked.motionValid = [](const auto&, const auto&) { return false; };
    if(smoothValidatedPath(&arc, times, blocked, 3, 0.5, 0.15) || arc != preserved) return 13;
    times[1] = times[0];
    if(smoothValidatedPath(&arc, times, oracle, 3, 0.5, 0.15) || arc != preserved) return 14;
    if(planApfPath({{0.0, 0.0}, {1.0, 0.0}}, lower, upper, oracle, 0.02, &path) || !path.empty()) return 5;
    oracle.distance = [](const ApfState& q) { return std::abs(q[0]) - 0.3; };
    if(planApfPath(reference, lower, upper, oracle, 0.02, &path) || !path.empty()) return 6;
    oracle.distance = [](const ApfState&) { return std::numeric_limits<double>::quiet_NaN(); };
    if(planApfPath(reference, lower, upper, oracle, 0.02, &path)) return 7;
    oracle.distance = [](const ApfState&) { return 1.0; };
    if(!planApfPath(reference, lower, upper, oracle, 0.02, &path) || path.size() != 2) return 8;
    std::cout << "PASS APF: free path, obstacle, fixed anchors, deterministic escape, blocked corridor, invalid oracle, validated shortcut, time-aware smoothing\n";
    return 0;
}
