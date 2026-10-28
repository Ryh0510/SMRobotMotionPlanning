#pragma once
#include <Eigen/Geometry>
#include <vector>
#include <cmath>
#include <set>
#include <array>
#include <stdexcept>
#include <random>
#include <cstdint>

namespace motion_planning {
inline Eigen::Quaterniond surfacePostureDelta(const Eigen::Vector4d& gene,const Eigen::Vector3d& axis,bool axialSpin,double angle) {
    Eigen::Vector3d direction=gene.head<3>();
    if(axialSpin)direction-=direction.dot(axis)*axis;
    const double n=direction.norm();Eigen::Quaterniond delta=Eigen::Quaterniond::Identity();
    if(n>1e-12)delta=Eigen::Quaterniond(Eigen::AngleAxisd(angle*std::min(1.,n),direction/n));
    if(axialSpin)delta=delta*Eigen::Quaterniond(Eigen::AngleAxisd(gene[3]*std::acos(-1.),axis));
    return delta.normalized();
}
inline std::array<long long,4> surfacePostureKey(const Eigen::Quaterniond& rotation) {
    auto c=rotation.normalized().coeffs().eval();Eigen::Index largest;c.cwiseAbs().maxCoeff(&largest);
    if(c[largest]<0)c=-c;std::array<long long,4> key{};
    for(int j=0;j<4;++j)key[j]=std::llround(c[j]*1e9);
    return key;
}
// Genes describe a swing within the allowed normal cone and a periodic axial spin.
// The caller maps them at the CURRENT fixed surface station and rechecks real IK.
inline std::vector<Eigen::Vector4d> surfacePostureSeedGenes(const Eigen::Vector3d& axis,bool axialSpin,int requested=0,double maximumAngle=0) {
    const double pi=std::acos(-1.);
    const auto u=axis.unitOrthogonal().eval(),v=axis.cross(u).eval();
    std::vector<Eigen::Vector4d> result;
    const auto append=[&](double radius,double azimuth,double spin) {
        Eigen::Vector4d x=Eigen::Vector4d::Zero();
        x.head<3>()=radius*(std::cos(azimuth)*u+std::sin(azimuth)*v);x[3]=spin;result.push_back(x);
    };
    // Keep the established nominal-spin rings as a baseline portfolio. New
    // coverage augments them instead of displacing previously useful seeds.
    for(double radius:{.5,.8,1.})for(int k=0;k<16;++k)append(radius,k*pi/8,0.);
    for(double spin:axialSpin?std::vector<double>{0.,1.,.5,-.5}:std::vector<double>{0.})append(0,0,spin);
    // Near-boundary rings and small spin offsets find narrow openings that a
    // Cartesian axis-only seed set misses. Bit-reversed azimuths cover the whole
    // cone early even when the caller has a small evaluation/time budget.
    for(int k=0;k<16;++k) {
        const int a=((k&1)<<3)|((k&2)<<1)|((k&4)>>1)|((k&8)>>3);
        for(double spin:axialSpin?std::vector<double>{0.,1.,.055,-.055,.945,-.945}:std::vector<double>{0.})
            append(.95,(a+.5)*pi/8,spin);
    }
    for(double radius:{1.,.8,.45})for(int k=0;k<8;++k)
        for(double spin:axialSpin?std::vector<double>{0.,1.,.5,-.5}:std::vector<double>{0.})
            append(radius,k*pi/4,spin);
    // Half-cell offsets cover gaps in both azimuth and spin without changing limits.
    for(double radius:{1.,.85,.6})for(int k=0;k<8;++k)
        for(double spin:axialSpin?std::vector<double>{.25,-.25,.75,-.75}:std::vector<double>{0.})
            append(radius,(k+.5)*pi/4,spin);
    if(requested>0){
        std::vector<Eigen::Vector4d> unique;std::set<std::array<long long,4>> seen;
        const auto add=[&](Eigen::Vector4d x){x[3]=std::remainder(x[3],2.);if(x[3]>=1-1e-12)x[3]=-1.;std::array<long long,4> k{};for(int j=0;j<4;++j)k[j]=std::llround(x[j]*1e9);if(seen.insert(k).second)unique.push_back(x);};
        for(const auto& x:result){add(x);if(int(unique.size())>=requested)break;}
        const auto radical=[](int n,int base){double out=0,f=1./base;for(;n;n/=base,f/=base)out+=(n%base)*f;return out;};
        for(int n=1;int(unique.size())<requested;++n){const double s=radical(n,2);const double radius=maximumAngle>1e-8?std::acos(1-s*(1-std::cos(maximumAngle)))/maximumAngle:std::sqrt(s),azimuth=2*pi*radical(n,3);Eigen::Vector4d x=Eigen::Vector4d::Zero();x.head<3>()=radius*(std::cos(azimuth)*u+std::sin(azimuth)*v);x[3]=axialSpin?2*radical(n,5)-1.:0.;add(x);}
        return unique;
    }
    return result;
}
// Generate the WHOLE pool up front. cos(tilt), azimuth and axial spin are
// independent uniforms: equal spherical area, not uniform tilt or angle rings.
// No neighbor or deterministic ring is inserted into this initial population.
inline std::vector<Eigen::Vector4d> surfacePostureInitialPool(const Eigen::Vector3d& axis,bool axialSpin,int requested,double angle,std::uint64_t seed=947) {
    if(requested<1 || angle<=1e-8)throw std::runtime_error("A nonzero posture cone is required for a distinct initial pool");
    const double effectiveAngle=std::max(0.,angle-1e-5),pi=std::acos(-1.);
    const auto u=axis.unitOrthogonal().eval(),v=axis.cross(u).eval();
    std::mt19937_64 rng(seed);std::uniform_real_distribution<double> uniform(0.,1.);
    std::vector<Eigen::Vector4d> result;result.reserve(requested);std::set<std::array<long long,4>> seen;
    for(int attempts=0;int(result.size())<requested;++attempts){
        if(attempts>requested*8)throw std::runtime_error("Could not construct the requested distinct posture pool");
        const double cosine=1.-uniform(rng)*(1.-std::cos(effectiveAngle));
        const double radius=std::acos(std::clamp(cosine,-1.,1.))/effectiveAngle,azimuth=2*pi*uniform(rng);
        Eigen::Vector4d x=Eigen::Vector4d::Zero();x.head<3>()=radius*(std::cos(azimuth)*u+std::sin(azimuth)*v);
        x[3]=axialSpin?2*uniform(rng)-1.:0.;
        if(seen.insert(surfacePostureKey(surfacePostureDelta(x,axis,axialSpin,effectiveAngle))).second)result.push_back(x);
    }
    return result;
}
}
