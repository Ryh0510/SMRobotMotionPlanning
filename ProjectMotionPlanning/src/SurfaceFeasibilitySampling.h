#pragma once
#include <Eigen/Geometry>
#include <cstdint>
#include <cmath>
#include <algorithm>
namespace motion_planning {
// Source-anchored coverage: equal-area normal cap, independent axial phase and
// uniform authorized radius. A proposal is never a safety certificate.
inline Eigen::Matrix<double,5,1> surfaceFeasibilitySample(std::uint64_t n,
    const Eigen::Vector3d& axis,bool spin,double angle,double radiusFraction) {
    const auto radical=[](std::uint64_t k,unsigned base){double x=0,f=1;while(k){f/=base;x+=f*(k%base);k/=base;}return x;};
    const double pi=std::acos(-1.),limit=std::max(1e-8,angle-1e-5);
    const double tilt=std::acos(1-radical(n,2)*(1-std::cos(limit)))/limit;
    const double phi=2*pi*radical(n,3);const auto u=axis.unitOrthogonal().eval(),v=axis.cross(u).eval();
    Eigen::Matrix<double,5,1> x=Eigen::Matrix<double,5,1>::Zero();
    x.head<3>()=tilt*(std::cos(phi)*u+std::sin(phi)*v);
    x[3]=spin?2*radical(n,5)-1:0.;
    x[4]=(2*radical(n,7)-1)*radiusFraction/.8;
    return x;
}
}
