#include "PathRefinement.h"

#include <algorithm>
#include <cmath>

namespace motion_planning::detail
{
    namespace
    {
        double distance(const ApfState& a, const ApfState& b)
        {
            double squared = 0.0;
            for(std::size_t j = 0; j < a.size(); ++j) {
                const double delta = b[j] - a[j];
                squared += delta * delta;
            }
            return std::sqrt(squared);
        }

        double bendingAt(const ApfState& a, const ApfState& b, const ApfState& c,
            double dtLeft, double dtRight)
        {
            double cost = 0.0;
            for(std::size_t j = 0; j < b.size(); ++j) {
                const double dv = (c[j] - b[j]) / dtRight - (b[j] - a[j]) / dtLeft;
                cost += 2.0 * dv * dv / (dtLeft + dtRight);
            }
            return cost;
        }
    }

    double jointPathLength(const ApfPath& path)
    {
        double length = 0.0;
        for(std::size_t i = 1; i < path.size(); ++i) length += distance(path[i - 1], path[i]);
        return length;
    }

    double jointPathBending(const ApfPath& path, const std::vector<double>& times)
    {
        double cost = 0.0;
        for(std::size_t i = 1; i + 1 < path.size(); ++i)
            cost += bendingAt(path[i - 1], path[i], path[i + 1],
                times[i] - times[i - 1], times[i + 1] - times[i]);
        return cost;
    }

    void shortcutApfPath(ApfPath* path, const ApfOracle& oracle)
    {
        if(!path || path->size() < 3 || !oracle.motionValid) return;
        // Only called on one APF repair interval: never shortcut across spray strokes.
        // A bounded lookahead removes integration zigzags without quadratic queries.
        ApfPath shortened{path->front()};
        std::size_t begin = 0;
        while(begin + 1 < path->size()) {
            std::size_t next = begin + 1;
            for(std::size_t span = std::min<std::size_t>(64, path->size() - 1 - begin);
                span >= 2; span /= 2) {
                const std::size_t end = begin + span;
                double length = 0.0;
                for(std::size_t i = begin; i < end; ++i) length += distance((*path)[i], (*path)[i + 1]);
                if(distance((*path)[begin], (*path)[end]) + 1.0e-8 < length &&
                    oracle.motionValid((*path)[begin], (*path)[end])) {
                    next = end;
                    break;
                }
            }
            shortened.push_back((*path)[next]);
            begin = next;
        }
        *path = std::move(shortened);
    }

    bool smoothValidatedPath(ApfPath* path, const std::vector<double>& times,
        const ApfOracle& oracle, int passes, double step, double maxDeviation)
    {
        if(!path || path->size() < 3 || times.size() != path->size() ||
            !oracle.motionValid || !std::isfinite(step) || !std::isfinite(maxDeviation) || maxDeviation <= 0.0)
            return false;
        const std::size_t dimension = path->front().size();
        for(std::size_t i = 0; i < path->size(); ++i) {
            if((*path)[i].size() != dimension || !std::isfinite(times[i]) ||
                (i > 0 && times[i] <= times[i - 1])) return false;
            for(double q : (*path)[i]) if(!std::isfinite(q)) return false;
        }
        const ApfPath reference = *path;
        step = std::clamp(step, 0.0, 0.85);
        bool changed = false;
        for(int pass = 0; pass < std::clamp(passes, 0, 8); ++pass) {
            const std::size_t span = std::max<std::size_t>(4, 32 >> std::min(pass, 3));
            const std::size_t stride = span / 2;
            for(std::size_t begin = 0; begin + 2 < path->size(); begin += stride) {
                const std::size_t end = std::min(begin + span, path->size() - 1);
                ApfPath candidate(path->begin() + begin, path->begin() + end + 1);
                bool withinCorridor = true;
                for(std::size_t i = begin + 1; i < end; ++i) {
                    const double fraction = (times[i] - times[begin]) / (times[end] - times[begin]);
                    const double envelope = std::pow(std::sin(3.14159265358979323846 * fraction), 2);
                    for(std::size_t j = 0; j < dimension; ++j) {
                        const double chord = (*path)[begin][j] + fraction * ((*path)[end][j] - (*path)[begin][j]);
                        candidate[i - begin][j] += step * envelope * (chord - (*path)[i][j]);
                        if(std::abs(candidate[i - begin][j] - reference[i][j]) > maxDeviation)
                            withinCorridor = false;
                    }
                }
                if(!withinCorridor) continue;
                const auto proposed = [&](std::size_t i) -> const ApfState& {
                    return i >= begin && i <= end ? candidate[i - begin] : (*path)[i];
                };
                double before = 0.0, after = 0.0, oldLength = 0.0;
                // Include both seams: reducing curvature inside a window must not
                // introduce a sharper corner just outside it.
                for(std::size_t i = std::max<std::size_t>(1, begin); i <= end && i + 1 < path->size(); ++i) {
                    const double left = times[i] - times[i - 1], right = times[i + 1] - times[i];
                    before += bendingAt((*path)[i - 1], (*path)[i], (*path)[i + 1], left, right);
                    after += bendingAt(proposed(i - 1), proposed(i), proposed(i + 1), left, right);
                }
                for(std::size_t i = begin; i < end; ++i) oldLength += distance((*path)[i], (*path)[i + 1]);
                if(!(after + 1.0e-12 < before) || jointPathLength(candidate) > oldLength + 1.0e-10) continue;
                bool valid = true;
                for(std::size_t i = 1; i < candidate.size(); ++i)
                    if(!oracle.motionValid(candidate[i - 1], candidate[i])) { valid = false; break; }
                if(!valid) continue;
                std::copy(candidate.begin(), candidate.end(), path->begin() + begin);
                changed = true;
            }
        }
        return changed;
    }
}
