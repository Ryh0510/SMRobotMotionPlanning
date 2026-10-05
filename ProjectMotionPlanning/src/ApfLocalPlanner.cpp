#include "ApfLocalPlanner.h"

#include <Eigen/Cholesky>
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
        double dot(const ApfState& a, const ApfState& b)
        {
            double value = 0.0;
            for(std::size_t j = 0; j < a.size(); ++j) value += a[j] * b[j];
            return value;
        }
        void normalize(ApfState& q)
        {
            const double length = norm(q);
            if(length > 1e-12) for(double& x : q) x /= length;
        }
        bool finite(const ApfState& q, std::size_t count)
        {
            return q.size() == count && std::all_of(q.begin(), q.end(), [](double x) { return std::isfinite(x); });
        }
        double segmentDistance(const ApfState& p, const ApfState& a, const ApfState& b)
        {
            auto d = difference(b, a);
            const double squared = dot(d, d);
            const double t = squared > 1e-20 ? std::clamp(dot(difference(p, a), d) / squared, 0.0, 1.0) : 0.0;
            auto residual = difference(p, a);
            for(std::size_t j = 0; j < residual.size(); ++j) residual[j] -= t * d[j];
            return norm(residual);
        }
        bool projectPosition(ApfState& q, const ApfState& target, const ApfState& lower,
            const ApfState& upper, const ApfGuidance& guidance, double radius, int iterations)
        {
            for(int iteration=0;iteration<iterations;++iteration) {
                const auto p=guidance.position(q);
                if(!finite(p,target.size()))return false;
                const auto residual=difference(target,p);
                const double error=norm(residual);
                if(error<=radius+1e-6)return true;
                Eigen::MatrixXd jacobian(p.size(),q.size());
                for(std::size_t j=0;j<q.size();++j) {
                    auto plus=q,minus=q;
                    plus[j]=std::min(upper[j],q[j]+0.001);minus[j]=std::max(lower[j],q[j]-0.001);
                    const auto pp=guidance.position(plus),pm=guidance.position(minus);
                    if(!finite(pp,p.size())||!finite(pm,p.size()))return false;
                    const double h=plus[j]-minus[j];
                    for(std::size_t r=0;r<p.size();++r)jacobian(r,j)=h>0?(pp[r]-pm[r])/h:0.0;
                }
                Eigen::MatrixXd normal=jacobian*jacobian.transpose();normal.diagonal().array()+=1e-5;
                Eigen::VectorXd delta(p.size());
                for(std::size_t r=0;r<p.size();++r)delta[r]=(1.0-radius/error)*residual[r];
                const Eigen::VectorXd dq=jacobian.transpose()*normal.ldlt().solve(delta);
                const double scale=std::min(1.0,0.10/std::max(1e-12,dq.norm()));
                bool advanced=false;
                for(double rate=scale;rate>=scale/16.0;rate*=0.5) {
                    auto candidate=q;
                    for(std::size_t j=0;j<q.size();++j)candidate[j]=std::clamp(q[j]+rate*dq[j],lower[j],upper[j]);
                    const auto cp=guidance.position(candidate);
                    if(!finite(cp,p.size()))return false;
                    if(norm(difference(target,cp))<error){q=std::move(candidate);advanced=true;break;}
                }
                if(!advanced)break;
            }
            return true; // Finite proposal; convergence is checked by the caller.
        }

    }

    bool projectApfPosition(ApfState& q, const ApfState& target,
        const ApfState& lower, const ApfState& upper, const ApfGuidance& guidance,
        double radius, int iterations)
    {
        if(!guidance.position || q.empty() || lower.size()!=q.size() || upper.size()!=q.size() ||
            !finite(q,q.size()) || target.empty() || !finite(target,target.size()) ||
            !std::isfinite(radius) || radius<0) return false;
        for(std::size_t j=0;j<q.size();++j)
            if(std::isnan(lower[j]) || std::isnan(upper[j]) || lower[j]>upper[j])return false;
        return projectPosition(q,target,lower,upper,guidance,radius,iterations);
    }

    bool followsApfGuide(const ApfState& from, const ApfState& to,
        const ApfState& guideFrom, const ApfState& guideTo,
        const ApfGuidance& guidance, double jointStep)
    {
        if(!guidance.position || from.empty() || from.size() != to.size() ||
            guideFrom.empty() || !finite(guideFrom, guideFrom.size()) || !finite(guideTo, guideFrom.size()) ||
            !finite(from, from.size()) || !finite(to, from.size()) ||
            !std::isfinite(jointStep) || jointStep <= 0.0 ||
            !std::isfinite(guidance.maxDeviation) || guidance.maxDeviation <= 0.0) return false;
        double maximumDelta = 0.0;
        for(std::size_t j = 0; j < from.size(); ++j) maximumDelta = std::max(maximumDelta, std::abs(to[j] - from[j]));
        const auto steps = std::max<std::size_t>(1, static_cast<std::size_t>(std::ceil(maximumDelta / jointStep)));
        for(std::size_t i = 0; i <= steps; ++i) {
            const double t = static_cast<double>(i) / steps;
            auto q = from, target = guideFrom;
            for(std::size_t j = 0; j < q.size(); ++j) q[j] += t * (to[j] - from[j]);
            for(std::size_t j = 0; j < target.size(); ++j) target[j] += t * (guideTo[j] - guideFrom[j]);
            const auto p = guidance.position(q);
            if(!finite(p, target.size()) || norm(difference(p, target)) > guidance.maxDeviation + 1e-9) return false;
        }
        return true;
    }

    bool followsApfPath(const ApfPath& path, const ApfGuidance& guidance, std::size_t begin)
    {
        if(path.empty() || begin >= guidance.positions.size() || path.size() > guidance.positions.size() - begin) return false;
        if(path.size() == 1) return followsApfGuide(path[0], path[0], guidance.positions[begin], guidance.positions[begin], guidance);
        for(std::size_t i = 1; i < path.size(); ++i)
            if(!followsApfGuide(path[i-1], path[i], guidance.positions[begin+i-1], guidance.positions[begin+i], guidance)) return false;
        return true;
    }

    bool planApfPath(const ApfPath& inputReference, const ApfState& lower,
        const ApfState& upper, const ApfOracle& oracle, double clearance,
        ApfPath* output, const ApfGuidance* guidance)
    {
        if(!output) return false;
        output->clear();
        ApfPath reference = inputReference;
        if(reference.size() < 2 || lower.empty() || upper.size() != lower.size() ||
            !oracle.distance || !oracle.motionValid || !std::isfinite(clearance) || clearance < 0.0) return false;
        const auto dimension = lower.size();
        for(std::size_t j = 0; j < dimension; ++j)
            if(!std::isfinite(lower[j]) || !std::isfinite(upper[j]) || lower[j] > upper[j]) return false;
        for(const auto& q : reference) {
            if(!finite(q, dimension)) return false;
            for(std::size_t j = 0; j < dimension; ++j) if(q[j] < lower[j] || q[j] > upper[j]) return false;
        }
        if(guidance && (!guidance->position || guidance->positions.size() != reference.size() ||
            !std::isfinite(guidance->maxDeviation) || guidance->maxDeviation <= 0.0)) return false;
        if(guidance) for(const auto& p : guidance->positions)
            if(guidance->positions.front().empty() || !finite(p, guidance->positions.front().size())) return false;
        // A joint-linear IK branch transition need not trace the Cartesian
        // control-point segment. Project the attraction seed with the ACTUAL
        // TCP model before collision planning; keep both exact boundary anchors.
        if(guidance) for(std::size_t i=1;i+1<reference.size();++i)
            if(!projectPosition(reference[i],guidance->positions[i],lower,upper,*guidance,0.001,60))return false;
        const auto metric = [&](const ApfState& q) { return guidance ? guidance->position(q) : q; };
        const auto metricReference = guidance ? guidance->positions : reference;
        const double tube = guidance ? guidance->maxDeviation : 0.45;
        const double influence = std::max(0.08, 5.0 * clearance);
        const double startDistance = oracle.distance(reference.front()), goalDistance = oracle.distance(reference.back());
        if(!std::isfinite(startDistance) || !std::isfinite(goalDistance) || startDistance < 0.0 || goalDistance < 0.0) return false;
        if(!finite(metric(reference.front()), metricReference.front().size()) ||
            !finite(metric(reference.back()), metricReference.front().size())) return false;
        const auto edgeValid = [&](const ApfState& a, const ApfState& b, const ApfState& ra, const ApfState& rb) {
            // Reject shape violations before launching expensive collision queries.
            return (!guidance || followsApfGuide(a, b, ra, rb, *guidance)) && oracle.motionValid(a, b);
        };

        // Visit ALL reference stations in order. A collision-free chord to the
        // final anchor must never skip the intervening zigzag or returning stroke.
        const int attempts = 2 + static_cast<int>(dimension) * 2;
        for(int attempt = 0; attempt < attempts; ++attempt) {
            const bool reverse = attempt % 2 != 0;
            const auto index = [&](std::size_t i) { return reverse ? reference.size() - 1 - i : i; };
            // Smooth deterministic early detours explore both sides of each
            // joint direction. Both boundary configurations remain exactly fixed.
            ApfPath fieldReference=reference;
            if(attempt>=2)for(std::size_t i=1;i+1<reference.size();++i) {
                const auto axis=static_cast<std::size_t>((attempt-2)/2);
                const double t=static_cast<double>(i)/(reference.size()-1);
                // Begin the wrist change while there is still room, instead
                // of inheriting a late, abrupt IK branch switch from the seed.
                if(guidance && dimension>=6) {
                    const double phase=std::clamp((t-(reverse?0.30:0.0))/0.70,0.0,1.0);
                    const double wristPhase=phase*phase*(3.0-2.0*phase);
                    for(std::size_t j=dimension-3;j<dimension;++j)
                        fieldReference[i][j]=reference.front()[j]+wristPhase*(reference.back()[j]-reference.front()[j]);
                }
                fieldReference[i][axis]=std::clamp(fieldReference[i][axis]+
                    (reverse?-0.55:0.55)*std::sin(3.14159265358979323846*t),lower[axis],upper[axis]);
                if(guidance && !projectPosition(fieldReference[i],metricReference[i],lower,upper,*guidance,
                    std::min(0.03,0.3*tube),60))return false;
            }
            ApfPath path{reference[index(0)]};
            bool failed = false;
            int totalIterations = 0, reportedIteration = -1;
            const int budget = std::min(1200, std::max(300, static_cast<int>(reference.size()) * 4));
            for(std::size_t station = 1; station < reference.size() && !failed; ++station) {
                const auto& target = fieldReference[index(station)];
                const auto& a = metricReference[index(station - 1)];
                const auto& b = metricReference[index(station)];
                const auto publishedStart = path.back();
                // Anticipatory obstacle potential: transport the current joint
                // offset to future reference stations, instead of waiting for
                // the next segment to collide. Do not forecast past a reversal.
                std::vector<std::size_t> preview{station};
                double ahead=0.0;
                const auto previewHeading=difference(b,a);
                for(std::size_t next=station+1;next<reference.size() && preview.size()<7;++next) {
                    const auto d=difference(metricReference[index(next)],metricReference[index(next-1)]);
                    if(dot(d,previewHeading)<0.5*norm(d)*norm(previewHeading))break;
                    ahead+=norm(d);
                    if(ahead>0.12)break;
                    if((next-station)%6==0)preview.push_back(next);
                }
                const auto forecast = [&](const ApfState& state) {
                    ApfPath states;
                    for(auto next:preview) {
                        auto trial=state;
                        for(std::size_t j=0;j<dimension;++j)
                            trial[j]=std::clamp(state[j]+fieldReference[index(next)][j]-target[j],lower[j],upper[j]);
                        states.push_back(std::move(trial));
                    }
                    std::vector<double> distances;
                    if(oracle.distances)distances=oracle.distances(states);
                    else for(const auto& state:states)distances.push_back(oracle.distance(state));
                    if(distances.size()!=states.size())return std::numeric_limits<double>::quiet_NaN();
                    double minimum=std::numeric_limits<double>::infinity();
                    for(double value:distances){if(!std::isfinite(value))return value;minimum=std::min(minimum,value);}
                    return minimum;
                };

                // Decay an existing local offset gradually instead of snapping
                // back to every collision-free target and creating saw teeth.
                if(station + 1 < reference.size() && path.size() > 1) {
                    auto carried = target;
                    for(std::size_t j = 0; j < dimension; ++j)
                        carried[j] = std::clamp(target[j] + 0.85 * (publishedStart[j] - fieldReference[index(station-1)][j]), lower[j], upper[j]);
                    if(norm(difference(carried, target)) > 1e-5 && forecast(carried) >= clearance && edgeValid(publishedStart, carried, a, b)) {
                        path.push_back(std::move(carried)); continue;
                    }
                }
                if(forecast(target) >= clearance && edgeValid(publishedStart, target, a, b)) { path.push_back(target); continue; }
                ApfState q = publishedStart, previousForce(dimension, 0.0);
                ApfPath integrated{publishedStart}, backfilled;
                const auto connectStation = [&](const ApfState& end) {
                    if(edgeValid(publishedStart,end,a,b))return true;
                    // A curved integration cannot be saved as one colliding
                    // chord. Retreat into preceding stations and distribute the
                    // verified detour over them; keep point count and anchors.
                    for(std::size_t support:{4u,12u,32u,64u,128u}) {
                        const std::size_t begin=path.size()>support?path.size()-support:0;
                        ApfPath curve(path.begin()+begin,path.end());
                        curve.insert(curve.end(),integrated.begin()+1,integrated.end());
                        if(curve.back()!=end)curve.push_back(end);
                        std::vector<double> lengths{0.0};
                        for(std::size_t i=1;i<curve.size();++i)
                            lengths.push_back(lengths.back()+norm(difference(curve[i],curve[i-1])));
                        if(lengths.back()<1e-12)continue;
                        const std::size_t count=path.size()+1-begin;
                        ApfPath replacement{curve.front()};
                        std::size_t edge=1;
                        for(std::size_t i=1;i+1<count;++i) {
                            const double at=lengths.back()*i/(count-1);
                            while(edge+1<lengths.size() && lengths[edge]<at)++edge;
                            const double span=lengths[edge]-lengths[edge-1];
                            const double t=span>1e-12?(at-lengths[edge-1])/span:0.0;
                            auto state=curve[edge-1];
                            for(std::size_t j=0;j<dimension;++j)state[j]+=t*(curve[edge][j]-state[j]);
                            replacement.push_back(std::move(state));
                        }
                        replacement.push_back(end);
                        bool valid=true;
                        if(guidance)for(std::size_t i=1;i<replacement.size() && valid;++i)
                            valid=followsApfGuide(replacement[i-1],replacement[i],
                                metricReference[index(begin+i-1)],metricReference[index(begin+i)],*guidance);
                        for(std::size_t i=1;i<replacement.size() && valid;++i)
                            valid=oracle.motionValid(replacement[i-1],replacement[i]);
                        if(!valid)continue;
                        backfilled.assign(path.begin(),path.begin()+begin);
                        backfilled.insert(backfilled.end(),replacement.begin(),replacement.end());
                        return true;
                    }
                    return false;
                };

                const auto heading = difference(b, a);
                const double headingSquared = dot(heading, heading);
                auto attractingPosition = b;
                if(guidance) {
                    // Local lookahead avoids a potential equilibrium immediately
                    // before a blocked station. Stop at a returning stroke/corner.
                    double lookedAhead = 0.0;
                    for(std::size_t next = station+1; next < reference.size(); ++next) {
                        const auto d = difference(metricReference[index(next)], metricReference[index(next-1)]);
                        if(dot(d, heading) < 0.8 * norm(d) * norm(heading)) break;
                        lookedAhead += norm(d);
                        attractingPosition = metricReference[index(next)];
                        if(lookedAhead >= std::min(0.03, 0.6*tube)) break;
                    }
                }
                bool reached = false;
                double bestDistance = std::numeric_limits<double>::max();
                int stagnant = 0;
                for(int iteration = 0; iteration < (station+1==reference.size()?500:120) && totalIterations < budget; ++iteration, ++totalIterations) {
                    if(oracle.progress && totalIterations % 100 == 0 && reportedIteration != totalIterations) {
                        reportedIteration = totalIterations;
                        oracle.progress(attempt + 1, totalIterations);
                    }
                    // The final anchor is fixed. A local trial may shift the preceding
                    // station, but must validate BOTH adjacent reference intervals.
                    if(station + 1 == reference.size() && path.size() >= 2 &&
                        edgeValid(path[path.size()-2], q, metricReference[index(station-2)], a) &&
                        edgeValid(q, target, a, b)) {
                        path.back() = q; q = target; reached = true; break;
                    }
                    if(station+1==reference.size() && edgeValid(q,target,b,b) && connectStation(target)) {
                        q=target;reached=true;break;
                    }
                    const auto position = metric(q);
                    if(!finite(position, b.size())) return false;
                    const double error = norm(difference(position, b));
                    const bool passed = headingSquared > 1e-16 && dot(difference(position, a), heading) >= headingSquared - 1e-8;
                    const bool last = station + 1 == reference.size();
                    if(!last && iteration > 0 && (passed || error < std::min(0.002, tube * 0.1) || (guidance && iteration>=5 && error<0.98*tube)) && error <= tube &&
                        connectStation(q)) { reached = true; break; }
                    const double distance = oracle.distance(q);
                    if(!std::isfinite(distance) || distance < 0.0) break;
                    ApfPath probes;
                    constexpr double epsilon = 0.001;
                    for(std::size_t j = 0; j < dimension; ++j) {
                        auto plus = q, minus = q;
                        plus[j] = std::min(upper[j], q[j] + epsilon);
                        minus[j] = std::max(lower[j], q[j] - epsilon);
                        probes.push_back(std::move(plus)); probes.push_back(std::move(minus));
                    }
                    const double forecastDistance=forecast(q);
                    if(!std::isfinite(forecastDistance))return false;
                    const bool imminent=distance<0.02;
                    const double potentialDistance=imminent?distance:std::min(distance,forecastDistance);
                    ApfState gradient(dimension, 0.0);
                    if(potentialDistance < influence) {
                        std::vector<double> distances;
                        ApfPath previewProbes;
                        const auto gradientPreview=imminent?std::vector<std::size_t>{station}:preview;
                        for(const auto& probe:probes) for(auto next:gradientPreview) {
                            auto trial=probe;
                            for(std::size_t j=0;j<dimension;++j)
                                trial[j]=std::clamp(probe[j]+fieldReference[index(next)][j]-target[j],lower[j],upper[j]);
                            previewProbes.push_back(std::move(trial));
                        }
                        std::vector<double> observed;
                        if(oracle.distances)observed=oracle.distances(previewProbes);
                        else for(const auto& probe:previewProbes)observed.push_back(oracle.distance(probe));
                        if(observed.size()!=previewProbes.size())return false;
                        for(std::size_t k=0;k<probes.size();++k) {
                            double value=std::numeric_limits<double>::infinity();
                            for(std::size_t f=0;f<gradientPreview.size();++f){const double d=observed[k*gradientPreview.size()+f];if(!std::isfinite(d))return false;value=std::min(value,d);}
                            distances.push_back(value);
                        }
                        if(distances.size() != probes.size()) return false;
                        for(std::size_t j = 0; j < dimension; ++j) {
                            const double dp = distances[2*j], dm = distances[2*j+1];
                            const double hp = probes[2*j][j] - q[j], hm = q[j] - probes[2*j+1][j];
                            if(std::isfinite(dp) && std::isfinite(dm) && hp + hm > 0)
                                gradient[j] = (dp - dm) / (hp + hm);
                            else if(std::isfinite(dp) && dp >= 0 && hp > 0) gradient[j] = (dp - distance) / hp;
                            else if(std::isfinite(dm) && dm >= 0 && hm > 0) gradient[j] = (distance - dm) / hm;
                        }
                        normalize(gradient);
                    }
                    auto attraction = difference(target, q); normalize(attraction);
                    ApfState force = attraction;
                    Eigen::MatrixXd jacobian;
                    Eigen::LDLT<Eigen::MatrixXd> inverse;
                    if(guidance) {
                        jacobian.resize(static_cast<int>(b.size()), static_cast<int>(dimension));
                        for(std::size_t j = 0; j < dimension; ++j) {
                            const auto plus = metric(probes[2*j]), minus = metric(probes[2*j+1]);
                            if(!finite(plus, b.size()) || !finite(minus, b.size())) return false;
                            const double h = probes[2*j][j] - probes[2*j+1][j];
                            for(std::size_t r = 0; r < b.size(); ++r) jacobian(r,j) = h > 0 ? (plus[r] - minus[r]) / h : 0.0;
                        }
                        Eigen::MatrixXd normal = jacobian * jacobian.transpose(); normal.diagonal().array() += 1e-4;
                        inverse.compute(normal);
                        Eigen::VectorXd residual(b.size());
                        for(std::size_t r = 0; r < b.size(); ++r) residual[r] = attractingPosition[r] - position[r];
                        const Eigen::VectorXd correction = jacobian.transpose() * inverse.solve(residual);
                        ApfState tracking(dimension);
                        for(std::size_t j = 0; j < dimension; ++j) tracking[j] = correction[j];
                        normalize(tracking);
                        Eigen::VectorXd preference(dimension);
                        for(std::size_t j=0;j<dimension;++j)preference[j]=attraction[j];
                        const Eigen::VectorXd taskPreference=jacobian.transpose()*inverse.solve(jacobian*preference);
                        // Restore the endpoint branch through position-preserving
                        // motion, rather than letting TCP tracking dominate it.
                        const double trackingWeight=1.5*std::min(1.0,residual.norm()/0.02);
                        for(std::size_t j=0;j<dimension;++j)
                            force[j]=trackingWeight*tracking[j]+0.35*attraction[j]+1.0*(preference[j]-taskPreference[j]);
                    }
                    auto escape = gradient;
                    if(attempt >= 2 && (potentialDistance < influence || stagnant > 6)) {
                        ApfState tangent(dimension, 0.0);
                        const auto axis = static_cast<std::size_t>((attempt - 2) / 2);
                        tangent[axis] = reverse ? -1.0 : 1.0;
                        const double projection = dot(tangent, gradient);
                        for(std::size_t j = 0; j < dimension; ++j) tangent[j] -= projection * gradient[j];
                        normalize(tangent);
                        for(std::size_t j = 0; j < dimension; ++j) escape[j] += 0.65 * tangent[j];
                    }
                    if(guidance) {
                        Eigen::VectorXd repulsion(dimension);
                        for(std::size_t j = 0; j < dimension; ++j) repulsion[j] = escape[j];
                        const Eigen::VectorXd taskComponent = jacobian.transpose() * inverse.solve(jacobian * repulsion);
                        // Prefer null-space motion near the corridor boundary, but do
                        // not suppress feasible positional detours in its interior.
                        const double taskSuppression = 0.8 * std::pow(std::clamp(segmentDistance(position, a, b) / tube, 0.0, 1.0), 4);
                        for(std::size_t j = 0; j < dimension; ++j) escape[j] -= taskSuppression * taskComponent[j];
                    }
                    const double repulsionWeight = 2.5 * std::clamp( 1.0 - potentialDistance / influence, 0.0, 2.0);
                    for(std::size_t j = 0; j < dimension; ++j) force[j] += repulsionWeight * escape[j];
                    normalize(force);
                    if(dot(force, previousForce) > 0.0) {
                        for(std::size_t j = 0; j < dimension; ++j) force[j] = 0.7 * force[j] + 0.3 * previousForce[j];
                        normalize(force);
                    }
                    previousForce = force;
                    bool advanced = false;
                    for(double step = 0.02; step >= 0.000625; step *= 0.5) {
                        auto candidate = q;
                        for(std::size_t j = 0; j < dimension; ++j) candidate[j] = std::clamp(q[j] + step * force[j], lower[j], upper[j]);
                        if(guidance && norm(difference(metric(candidate),b))>tube &&
                            !projectPosition(candidate,b,lower,upper,*guidance,0.9*tube,20))return false;
                        const auto p = metric(candidate);
                        if(!finite(p, b.size()) || segmentDistance(p, a, b) > tube || norm(difference(candidate,q)) < 1e-6) continue;
                        if(!oracle.motionValid(q, candidate)) continue;
                        const double candidateDistance = oracle.distance(candidate);
                        if(!std::isfinite(candidateDistance) || candidateDistance < 0.0 ||
                            candidateDistance < std::min(clearance, distance) * 0.5) continue;
                        // Stop at this reference station's forward plane. A
                        // 20 mrad integration step must not jump across several
                        // dense stations and turn the saved chord into a collision.
                        if(station+1 < reference.size() && headingSquared > 1e-16 &&
                            dot(difference(position,a),heading) < headingSquared &&
                            dot(difference(p,a),heading) >= headingSquared) {
                            double lo = 0.0, hi = 1.0;
                            for(int bisect=0; bisect<12; ++bisect) {
                                const double t = 0.5*(lo+hi);
                                auto trial = q;
                                for(std::size_t j=0;j<dimension;++j) trial[j] += t*(candidate[j]-q[j]);
                                const auto pt = metric(trial);
                                if(!finite(pt,b.size())) return false;
                                if(dot(difference(pt,a),heading) >= headingSquared) hi=t; else lo=t;
                            }
                            auto stationPoint = q;
                            for(std::size_t j=0;j<dimension;++j) stationPoint[j] += hi*(candidate[j]-q[j]);
                            if(norm(difference(metric(stationPoint),b)) <= tube && connectStation(stationPoint)) {
                                q = std::move(stationPoint); reached=true; advanced=true; break;
                            }
                        }
                        q = std::move(candidate); advanced = true; break;
                    }
                    // If the combined attraction/tangential force is blocked,
                    // first recover clearance along the measured distance
                    // gradient. Recheck the actual swept motion after projection.
                    if(!advanced && norm(gradient)>1e-8)for(double step=0.01;step>=0.00015625;step*=0.5) {
                        auto candidate=q;
                        for(std::size_t j=0;j<dimension;++j)candidate[j]=std::clamp(q[j]+step*gradient[j],lower[j],upper[j]);
                        if(guidance && !projectPosition(candidate,b,lower,upper,*guidance,0.95*tube,20))return false;
                        const auto p=metric(candidate);
                        if(!finite(p,b.size()) || segmentDistance(p,a,b)>tube || norm(difference(candidate,q))<1e-6)continue;
                        const double recovered=oracle.distance(candidate);
                        if(!std::isfinite(recovered) || recovered<=distance+1e-7 || !oracle.motionValid(q,candidate))continue;
                        q=std::move(candidate);advanced=true;break;
                    }
                    if(advanced && integrated.back()!=q)integrated.push_back(q);
                    if(reached || !advanced) break;
                    const double score = last ? norm(difference(q,target)) + 0.1*norm(difference(metric(q),b)) :
                        norm(difference(metric(q), b)) + 0.01 * norm(difference(q, target));
                    if(score < bestDistance - 1e-5) { bestDistance = score; stagnant = 0; }
                    else if(++stagnant > 35) break;
                }
                if(!reached) {
                    if(oracle.failedStation) oracle.failedStation(attempt+1, index(station), q);
                    failed = true; break;
                }
                if(!backfilled.empty())path=std::move(backfilled);
                else path.push_back(std::move(q));
            }
            if(!failed && path.size() == reference.size()) {
                if(reverse) std::reverse(path.begin(), path.end());
                // No chord shortcut or global arc-length redistribution: each
                // output station corresponds to the same original input station.
                *output = std::move(path); return true;
            }
        }
        return false;
    }
}
