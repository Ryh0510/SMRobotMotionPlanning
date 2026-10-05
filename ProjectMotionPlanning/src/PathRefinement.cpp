#include "PathRefinement.h"

#include <algorithm>
#include <cmath>
#include <limits>

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

    bool restoreApfTcpShape(ApfPath* path, const ApfState& lower, const ApfState& upper,
        const ApfOracle& oracle, const ApfGuidance& guidance)
    {
        if(!path || path->size()<3 || guidance.positions.size()!=path->size() ||
            !guidance.position || !oracle.distances || !oracle.motionValid || !oracle.pathValid ||
            !std::isfinite(guidance.maxDeviation) || guidance.maxDeviation<=0 ||
            lower.size()!=path->front().size() || upper.size()!=lower.size()) return false;
        const ApfPath original=*path;
        const std::size_t count=path->size();
        ApfPath positions,targets;
        std::vector<double> arc(count,0.0),cap(count,1.0);
        for(std::size_t i=0;i<count;++i) {
            if(guidance.positions[i].size()!=3 || original[i].size()!=lower.size())return false;
            for(double v:guidance.positions[i])if(!std::isfinite(v))return false;
            for(std::size_t j=0;j<lower.size();++j)
                if(!std::isfinite(original[i][j]) || std::isnan(lower[j]) || std::isnan(upper[j]) ||
                    lower[j]>upper[j] || original[i][j]<lower[j] || original[i][j]>upper[j])return false;
            auto p=guidance.position(original[i]);
            if(p.size()!=3)return false;
            for(double v:p)if(!std::isfinite(v))return false;
            positions.push_back(p);
            if(i)arc[i]=arc[i-1]+distance(guidance.positions[i-1],guidance.positions[i]);
        }
        if(arc.back()<1e-9)return false;
        // Convolve the ordered reference in physical arc length. This rounds
        // genuine input corners without connecting different returning strokes.
        const auto atArc=[&](double s) {
            if(s<=0)return guidance.positions.front();
            if(s>=arc.back())return guidance.positions.back();
            const auto it=std::upper_bound(arc.begin(),arc.end(),s);
            const std::size_t b=static_cast<std::size_t>(it-arc.begin()),a=b-1;
            auto p=guidance.positions[a];
            const double t=(s-arc[a])/(arc[b]-arc[a]);
            for(int j=0;j<3;++j)p[j]+=t*(guidance.positions[b][j]-p[j]);
            return p;
        };
        for(std::size_t i=0;i<count;++i) {
            auto target=guidance.positions[i];
            double weight=0;ApfState sum(3,0);
            const double radius=std::min({0.040,0.40*guidance.maxDeviation,arc[i],arc.back()-arc[i]});
            if(radius>1e-8) {
                for(int k=-12;k<=12;++k) {
                    const double w=1.0+std::cos(3.14159265358979323846*k/12.0);
                    const auto p=atArc(arc[i]+radius*k/12.0);
                    for(int j=0;j<3;++j)sum[j]+=w*p[j];weight+=w;
                }
                for(int j=0;j<3;++j)target[j]=sum[j]/weight;
            }
            targets.push_back(target);
        }
        const auto before=oracle.distances(original);
        if(before.size()!=count)return false;
        double floor=std::numeric_limits<double>::infinity();
        for(double d:before) {if(!std::isfinite(d) || d<0)return false;floor=std::min(floor,d);}
        cap.front()=cap.back()=0.0;
        // A blocked knot reduces nearby displacement gradually over 150 mm,
        // rather than cutting in/out at the first colliding point.
        const double transition=std::max(0.03,1.5*guidance.maxDeviation);
        for(int iteration=0;iteration<16;++iteration) {
            if(oracle.progress)oracle.progress(iteration,16);
            std::vector<double> amount=cap;
            for(std::size_t i=1;i+1<count;++i) {
                const auto first=std::lower_bound(arc.begin(),arc.end(),arc[i]-transition);
                const auto last=std::upper_bound(arc.begin(),arc.end(),arc[i]+transition);
                for(auto it=first;it!=last;++it) {
                    const std::size_t j=static_cast<std::size_t>(it-arc.begin());
                    // Endpoint TCP already agrees with its target: it does not
                    // need an artificial 150 mm zero-displacement neighborhood.
                    if(j==0 || j+1==count || cap[j]>=1.0)continue;
                    const double t=std::abs(arc[i]-arc[j])/transition;
                    const double blend=t*t*t*(10.0+t*(-15.0+6.0*t));
                    amount[i]=std::min(amount[i],cap[j]+(1.0-cap[j])*blend);
                }
            }
            ApfPath candidate=original;
            std::vector<bool> projected(count,true);
            for(std::size_t i=1;i+1<count;++i) {
                if(amount[i]<=1e-8)continue;
                auto target=positions[i];
                for(int j=0;j<3;++j)target[j]+=amount[i]*(targets[i][j]-target[j]);
                if(!projectApfPosition(candidate[i],target,lower,upper,guidance,0.00002,30))return false;
                const auto actual=guidance.position(candidate[i]);
                projected[i]=actual.size()==3;
                if(projected[i])for(double v:actual)if(!std::isfinite(v))projected[i]=false;
                if(projected[i])projected[i]=distance(actual,target)<=0.00005;
            }
            const auto observations=oracle.distances(candidate);
            if(observations.size()!=count)return false;
            bool invalid=false;
            const auto reduce=[&](std::size_t i) {
                if(i==0 || i+1==count)return;
                cap[i]=amount[i]<0.04?0.0:0.5*amount[i];invalid=true;
            };
            for(std::size_t i=1;i+1<count;++i) {
                if(!projected[i] || !std::isfinite(observations[i]) || observations[i]<floor-1e-9 ||
                    distance(guidance.position(candidate[i]),guidance.positions[i])>guidance.maxDeviation)
                    reduce(i);
            }
            if(invalid)continue;
            std::vector<bool> motions;
            if(oracle.motionsValid)motions=oracle.motionsValid(candidate);
            else for(std::size_t i=1;i<count;++i)motions.push_back(oracle.motionValid(candidate[i-1],candidate[i]));
            if(motions.size()!=count-1)return false;
            for(std::size_t i=0;i<motions.size();++i)if(!motions[i]){reduce(i);reduce(i+1);}
            if(invalid)continue;
            if(!oracle.pathValid(candidate,0))return false;
            *path=std::move(candidate);
            return true;
        }
        return false;
    }

    bool refineApfTcpShape(ApfPath* path, const std::vector<double>& times,
        const ApfState& lower, const ApfState& upper, const ApfOracle& oracle,
        const ApfGuidance& guidance, int passes, double step)
    {
        if(!restoreApfTcpShape(path,lower,upper,oracle,guidance))return false;
        ApfPath targets;
        for(const auto& q:*path)targets.push_back(guidance.position(q));
        ApfOracle projected=oracle;
        projected.project=[&](auto& q,std::size_t i) {
            if(!projectApfPosition(q,targets[i],lower,upper,guidance,0.0002,20))return false;
            const auto p=guidance.position(q);
            return p.size()==3 && distance(p,targets[i])<=0.000201;
        };
        smoothValidatedPath(path,times,projected,passes,step,guidance.maxDeviation,true);
        return true;
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
