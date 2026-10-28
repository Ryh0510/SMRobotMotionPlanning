#include <ProjectMotionPlanning/AdaptivePostureSearch.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <set>
#include <stdexcept>

namespace motion_planning {
namespace {
double radical(unsigned index, unsigned base) {
    double out=0., scale=1.;
    while(index){scale/=base;out+=(index%base)*scale;index/=base;}return out;
}
bool better(const PostureConstraintFeedback& a,const PostureConstraintFeedback& b) {
    if(a.feasible!=b.feasible)return a.feasible;
    if(!a.feasible && a.violation!=b.violation)return a.violation<b.violation;
    return a.cost<b.cost;
}
}
AdaptivePostureSearch::X AdaptivePostureSearch::repair(X x,bool spin) {
    if(x.head<2>().norm()>1.)x.head<2>().normalize();
    x[2]=spin?std::remainder(x[2],2.):0.;return x;
}
Eigen::Quaterniond AdaptivePostureSearch::rotation(const X& value,const Eigen::Vector3d& axis,double angle,bool spin) {
    const X x=repair(value,spin);const auto u=axis.unitOrthogonal().eval(),v=axis.cross(u).eval();
    const Eigen::Vector3d tilt=x[0]*u+x[1]*v;const double n=tilt.norm();
    Eigen::Quaterniond q=Eigen::Quaterniond::Identity();
    if(n>1e-12)q=Eigen::Quaterniond(Eigen::AngleAxisd(angle*n,tilt/n));
    if(spin)q=q*Eigen::Quaterniond(Eigen::AngleAxisd(x[2]*std::acos(-1.),axis));return q;
}
AdaptivePostureSearch::X AdaptivePostureSearch::coordinates(const Eigen::Quaterniond& q,const Eigen::Vector3d& axis,double angle,bool spin) {
    Eigen::Quaterniond twist(q.w(),axis.x()*q.vec().dot(axis),axis.y()*q.vec().dot(axis),axis.z()*q.vec().dot(axis));
    if(twist.norm()<1e-12)twist=Eigen::Quaterniond::Identity();else twist.normalize();
    Eigen::Quaterniond swing=spin?q*twist.conjugate():q;
    if(swing.w()<0)swing.coeffs()=-swing.coeffs();const Eigen::AngleAxisd aa(swing);
    const auto u=axis.unitOrthogonal().eval(),v=axis.cross(u).eval();
    const Eigen::Vector3d tilt=aa.axis()*aa.angle()/std::max(1e-12,angle);
    return repair(X(tilt.dot(u),tilt.dot(v),spin?2*std::atan2(twist.vec().dot(axis),twist.w())/std::acos(-1.):0.),spin);
}
AdaptivePostureSearchReport AdaptivePostureSearch::search(const AdaptivePostureOptions& o,const std::vector<X>& seeds,
    bool spin,const Oracle& oracle,const std::function<bool()>& stopped) {
    if(o.pointEvaluations<24 || o.pointEvaluations>2048 || o.localIterations<0 || o.localIterations>128)
        throw std::invalid_argument("Invalid adaptive posture budget");
    AdaptivePostureSearchReport report;
    struct Sample {X x;PostureConstraintFeedback feedback;};std::vector<Sample> samples;
    std::set<std::array<long long,3>> visited;
    const auto evaluate=[&](X x,bool local) {
        x=repair(x,spin);auto phase=x[2];if(phase>=1.-1e-10)phase-=2.;
        const std::array<long long,3> key{std::llround(x[0]*1e7),std::llround(x[1]*1e7),std::llround(phase*1e7)};
        if(!visited.insert(key).second)return false;
        const auto f=oracle(x);samples.push_back({x,f});++report.evaluations;
        if(local)++report.localEvaluations;if(f.feasible)++report.safePostures;return true;
    };
    const auto done=[&](){return report.evaluations>=o.pointEvaluations || (stopped && stopped());};
    // Warm starts and nested global coverage are interleaved: no seed list can
    // consume the entire budget before the axial/normal domain is visited.
    const double pi=std::acos(-1.);unsigned global=1;std::size_t seed=0;
    int localUsed=0,proposal=0;double mesh=.16;std::size_t localBase=0;int direction=0;
    while(!done() && proposal++<o.pointEvaluations*12) {
        bool local=false;X x=X::Zero();
        if(seed<seeds.size() && (report.evaluations%3==0 || global<3))x=seeds[seed++];
        else if(report.evaluations>=24 && localUsed<o.localIterations && report.evaluations%3==2 && !samples.empty()) {
            local=true;
            if(direction==0){localBase=0;for(std::size_t k=1;k<samples.size();++k)if(better(samples[k].feedback,samples[localBase].feedback))localBase=k;}
            x=samples[localBase].x;const int dimensions=spin?3:2;
            x[direction/2]+=(direction%2?1.:-1.)*mesh;
            if(++direction==2*dimensions){direction=0;mesh*=.55;if(mesh<.008)mesh=.12;}
            ++localUsed;
        } else {
            // Halton coverage of the normal disk and periodic phase. Every
            // fourth proposal additionally samples the limiting cone ring.
            const double radius=global%4==0?1.:std::sqrt(radical(global,2));
            const double azimuth=2*pi*radical(global,3);
            x=X(radius*std::cos(azimuth),radius*std::sin(azimuth),spin?2*radical(global,5)-1.:0.);++global;
        }
        evaluate(x,local);
        if(report.safePostures>=8 && report.evaluations>=32){report.stopReason="safe_candidate_pool";break;}
    }
    if(report.stopReason.empty())report.stopReason=report.evaluations>=o.pointEvaluations?"evaluation_budget":
        stopped && stopped()?"time_or_cancel_budget":"proposal_budget";
    return report;
}
}
