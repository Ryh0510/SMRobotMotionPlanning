#include "ApfLocalPlanner.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace motion_planning::detail
{
    namespace
    {
        double norm(const ApfState& q)
        {
            double sum = 0.0;
            for(double x : q) sum += x * x;
            return std::sqrt(sum);
        }

        ApfState difference(const ApfState& a, const ApfState& b)
        {
            ApfState delta(a.size());
            for(std::size_t j = 0; j < a.size(); ++j) delta[j] = a[j] - b[j];
            return delta;
        }

        void normalize(ApfState& q)
        {
            const double length = norm(q);
            if(length > 1.0e-12) for(double& x : q) x /= length;
        }
    }

    bool planApfPath(const ApfPath& reference, const ApfState& lower,
        const ApfState& upper, const ApfOracle& oracle, double clearance,
        ApfPath* output)
    {
        if(!output) return false;
        output->clear();
        if(reference.size() < 2 || lower.empty() || upper.size() != lower.size()) return false;
        const std::size_t dimension = lower.size();
        if(!oracle.distance || !oracle.motionValid || !std::isfinite(clearance) || clearance < 0.0) return false;
        for(std::size_t j = 0; j < dimension; ++j)
            if(!std::isfinite(lower[j]) || !std::isfinite(upper[j]) || lower[j] > upper[j]) return false;
        for(const auto& q : reference) {
            if(q.size() != dimension) return false;
            for(std::size_t j = 0; j < dimension; ++j)
                if(!std::isfinite(q[j]) || q[j] < lower[j] || q[j] > upper[j]) return false;
        }
        const double startDistance = oracle.distance(reference.front());
        const double goalDistance = oracle.distance(reference.back());
        if(!std::isfinite(startDistance) || !std::isfinite(goalDistance) ||
            startDistance < 0.0 || goalDistance < 0.0) return false;

        const double influence = std::max(0.04, 3.0 * clearance);
        // A finite family of tangential fields breaks symmetric local minima.
        // These are local integrations, not sampled trees or global roadmaps.
        const int attempts = 2 + static_cast<int>(dimension) * 2;
        for(int attempt = 0; attempt < attempts; ++attempt) {
            const bool reverse = attempt % 2 != 0;
            const ApfState& start = reverse ? reference.back() : reference.front();
            const ApfState& goal = reverse ? reference.front() : reference.back();
            ApfPath path{start};
            ApfState q = start;
            double bestGoalDistance = norm(difference(goal, q));
            int stagnant = 0;
            for(int iteration = 0; iteration < 600; ++iteration) {
                ApfState attraction = difference(goal, q);
                const double goalDistanceNow = norm(attraction);
                if(iteration % 6 == 0 || goalDistanceNow < 0.06) {
                    if(oracle.motionValid(q, goal)) {
                        path.push_back(goal);
                        if(reverse) std::reverse(path.begin(), path.end());
                        *output = std::move(path);
                        return true;
                    }
                }
                const double distance = oracle.distance(q);
                if(!std::isfinite(distance) || distance < 0.0) break;
                normalize(attraction);
                ApfState gradient(dimension, 0.0);
                if(distance < influence) {
                    constexpr double epsilon = 0.001;
                    for(std::size_t j = 0; j < dimension; ++j) {
                        ApfState plus = q, minus = q;
                        plus[j] = std::min(upper[j], q[j] + epsilon);
                        minus[j] = std::max(lower[j], q[j] - epsilon);
                        const double dp = oracle.distance(plus), dm = oracle.distance(minus);
                        // Mesh penetration depths need not form a differentiable
                        // signed field. Use free-space differences at its boundary.
                        if(std::isfinite(dp) && std::isfinite(dm) && dp >= 0.0 && dm >= 0.0 && plus[j] > minus[j])
                            gradient[j] = (dp - dm) / (plus[j] - minus[j]);
                        else if(std::isfinite(dp) && dp >= 0.0 && plus[j] > q[j])
                            gradient[j] = (dp - distance) / (plus[j] - q[j]);
                        else if(std::isfinite(dm) && dm >= 0.0 && q[j] > minus[j])
                            gradient[j] = (distance - dm) / (q[j] - minus[j]);
                    }
                    normalize(gradient);
                }
                ApfState force = attraction;
                const double repulsion = 2.5 * std::max(0.0, 1.0 - distance / influence);
                for(std::size_t j = 0; j < dimension; ++j) force[j] += repulsion * gradient[j];

                // Weak attraction to the original segment discourages unnecessary
                // detours without attracting the integrator into colliding points.
                const ApfState* nearest = &reference.front();
                double nearestDistance = std::numeric_limits<double>::max();
                for(const auto& point : reference) {
                    const double d = norm(difference(point, q));
                    if(d < nearestDistance) { nearestDistance = d; nearest = &point; }
                }
                if(nearestDistance > 0.1)
                    for(std::size_t j = 0; j < dimension; ++j)
                        force[j] += 0.15 * ((*nearest)[j] - q[j]) / nearestDistance;

                if(attempt >= 2 && (distance < influence || stagnant > 8)) {
                    const std::size_t axis = static_cast<std::size_t>((attempt - 2) / 2);
                    ApfState tangent(dimension, 0.0);
                    tangent[axis] = reverse ? -1.0 : 1.0;
                    const double projection = tangent[axis] * gradient[axis];
                    for(std::size_t j = 0; j < dimension; ++j) tangent[j] -= projection * gradient[j];
                    normalize(tangent);
                    for(std::size_t j = 0; j < dimension; ++j) force[j] += 1.4 * tangent[j];
                }
                normalize(force);
                bool advanced = false;
                for(double step = 0.04; step >= 0.000625; step *= 0.5) {
                    ApfState candidate = q;
                    for(std::size_t j = 0; j < dimension; ++j)
                        candidate[j] = std::clamp(q[j] + step * force[j], lower[j], upper[j]);
                    if(norm(difference(candidate, q)) < 1.0e-6) continue;
                    if(!oracle.motionValid(q, candidate)) continue;
                    const double candidateDistance = oracle.distance(candidate);
                    if(!std::isfinite(candidateDistance) || candidateDistance < 0.0) continue;
                    // Do not step into a narrow boundary layer when already clear.
                    if(candidateDistance < std::min(clearance, distance) * 0.5) continue;
                    q = std::move(candidate);
                    path.push_back(q);
                    advanced = true;
                    break;
                }
                if(!advanced) break;
                const double currentGoalDistance = norm(difference(goal, q));
                if(currentGoalDistance < bestGoalDistance - 0.002) {
                    bestGoalDistance = currentGoalDistance;
                    stagnant = 0;
                } else if(++stagnant > 100) break;
                if(path.size() > 16 && norm(difference(q, path[path.size() - 16])) < 0.008) break;
            }
        }
        return false;
    }
}
