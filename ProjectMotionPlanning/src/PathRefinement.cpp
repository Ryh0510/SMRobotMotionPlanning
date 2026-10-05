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

        double cornerAt(const ApfState& a, const ApfState& b, const ApfState& c)
        {
            const double left=distance(a,b),right=distance(b,c);
            if(left<1e-9 || right<1e-9)return 0.0;
            double dot=0.0;
            for(std::size_t j=0;j<b.size();++j)dot+=(b[j]-a[j])*(c[j]-b[j]);
            return std::acos(std::clamp(dot/(left*right),-1.0,1.0));
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

    bool smoothValidatedPath(ApfPath* path, const std::vector<double>& times,
        const ApfOracle& oracle, int passes, double step, double maxDeviation, bool orderedCorridor)
    {
        if(!path || path->size() < 3 || times.size() != path->size() ||
            (!oracle.motionValid || (orderedCorridor && !oracle.pathValid)) || !std::isfinite(step) || !std::isfinite(maxDeviation) || maxDeviation <= 0.0)
            return false;
        const std::size_t dimension = path->front().size();
        for(std::size_t i = 0; i < path->size(); ++i) {
            if((*path)[i].size() != dimension || !std::isfinite(times[i]) ||
                (i > 0 && times[i] <= times[i - 1])) return false;
            for(double q : (*path)[i]) if(!std::isfinite(q)) return false;
        }
        const ApfPath reference = *path;
        step = std::clamp(step, 0.0, orderedCorridor ? 1.0 : 0.85);
        bool changed = false;
        // The ordered TCP corridor replaces the arbitrary box around a noisy APF
        // seed. Include fine scales: stopping at 16 nodes leaves sharp reversals.
        const int sweeps = orderedCorridor ? (passes > 0 ? 7 * std::min(3, (passes + 2) / 3) : 0) : std::clamp(passes, 0, 12);
        for(int pass = 0; pass < sweeps; ++pass) {
            const std::size_t span = orderedCorridor ? (128 >> (pass % 7)) : std::max<std::size_t>(4, 128 >> (pass % 4));
            if(oracle.progress) oracle.progress(pass, sweeps);
            const std::size_t stride = span / 2;
            for(std::size_t begin = 0; begin + 2 < path->size(); begin += stride) {
                const std::size_t end = std::min(begin + span, path->size() - 1);
                for(double alpha = step; alpha >= step / 8.0 && alpha > 0.0; alpha *= 0.5) {
                    ApfPath candidate(path->begin() + begin, path->begin() + end + 1);
                    bool withinCorridor = true;
                    for(std::size_t i = begin + 1; i < end; ++i) {
                        const double fraction = (times[i] - times[begin]) / (times[end] - times[begin]);
                        const double envelope = std::pow(std::sin(3.14159265358979323846 * fraction), 2);
                        for(std::size_t j = 0; j < dimension; ++j) {
                            const double chord = (*path)[begin][j] + fraction * ((*path)[end][j] - (*path)[begin][j]);
                            candidate[i - begin][j] += alpha * envelope * (chord - (*path)[i][j]);
                            if(!orderedCorridor && std::abs(candidate[i - begin][j] - reference[i][j]) > maxDeviation) withinCorridor = false;
                        }
                    }
                    if(!withinCorridor) continue;
                    if(orderedCorridor && oracle.project) {
                        for(std::size_t i=begin+1;i<end;++i)
                            if(!oracle.project(candidate[i-begin],i)) { withinCorridor=false; break; }
                    }
                    if(!withinCorridor) continue;
                    const auto proposed = [&](std::size_t i) -> const ApfState& {
                        return i >= begin && i <= end ? candidate[i - begin] : (*path)[i];
                    };
                    double before = 0.0, after = 0.0, oldLength = 0.0;
                    double oldCorner=0.0,newCorner=0.0;
                    // Include both seams, not just the interior of the window.
                    for(std::size_t i = std::max<std::size_t>(1, begin); i <= end && i + 1 < path->size(); ++i) {
                        const double left = times[i] - times[i - 1], right = times[i + 1] - times[i];
                        before += bendingAt((*path)[i - 1], (*path)[i], (*path)[i + 1], left, right);
                        after += bendingAt(proposed(i - 1), proposed(i), proposed(i + 1), left, right);
                        oldCorner=std::max(oldCorner,cornerAt((*path)[i-1],(*path)[i],(*path)[i+1]));
                        newCorner=std::max(newCorner,cornerAt(proposed(i-1),proposed(i),proposed(i+1)));
                    }
                    for(std::size_t i = begin; i < end; ++i) oldLength += distance((*path)[i], (*path)[i + 1]);
                    if(!(after + 1.0e-12 < before) || jointPathLength(candidate) > oldLength + 1.0e-10) continue;
                    // Do not hide a new sharp reversal behind a lower integral cost.
                    if(orderedCorridor && newCorner>std::max(oldCorner,0.7853981633974483)+1e-9)continue;
                    bool valid = true;
                    if(oracle.pathValid) valid = oracle.pathValid(candidate, begin);
                    else for(std::size_t i = 1; i < candidate.size(); ++i)
                        if(!oracle.motionValid(candidate[i - 1], candidate[i])) { valid = false; break; }
                    if(!valid) continue;
                    std::copy(candidate.begin(), candidate.end(), path->begin() + begin);
                    changed = true;
                    break;
                }
            }
        }
        return changed;
    }
}
