#include "SprayPoseConstraint.h"
#include <cmath>

namespace motion_planning { namespace detail {
const char* sprayPoseConstraintName(SprayPoseConstraint constraint) {
    switch(constraint) {
    case SprayPoseConstraint::FixedTcp: return "fixed_tcp";
    case SprayPoseConstraint::SurfacePivotFixedDistance: return "surface_pivot_fixed_distance";
    case SprayPoseConstraint::SurfacePivotAdjustableDistance: return "surface_pivot_adjustable_distance";
    }
    return "invalid";
}
bool constrainedSprayPose(const Eigen::Isometry3d& reference, const Eigen::Vector3d& axis,
    double distance, const Eigen::Vector3d& angles, double shift,
    const SprayAxisContinuityOptions& o, Eigen::Isometry3d& result) {
    if(!reference.matrix().allFinite() || !axis.allFinite() || std::abs(axis.norm()-1)>1e-10 ||
        !angles.allFinite() || !std::isfinite(shift) ||
        angles.head<2>().norm()>o.maximumTiltRadians+1e-12 || std::abs(angles.z())>o.maximumRollRadians+1e-12)
        return false;
    const bool hit=std::isfinite(distance);
    const bool fixed=o.poseConstraint==SprayPoseConstraint::FixedTcp;
    const bool flexible=o.poseConstraint==SprayPoseConstraint::SurfacePivotAdjustableDistance;
    if(!fixed && !flexible && o.poseConstraint!=SprayPoseConstraint::SurfacePivotFixedDistance)return false;
    if(((fixed || (hit && !flexible)) && std::abs(shift)>1e-14) || (fixed && hit && angles.head<2>().norm()>1e-14) ||
        (!hit && !o.allowNonSprayingTransitions) || (hit && distance<=1e-8))return false;
    result=reference;
    const Eigen::Vector3d u=axis.unitOrthogonal(),v=axis.cross(u),tilt=angles.x()*u+angles.y()*v;
    const double theta=tilt.norm();
    if(theta>1e-14)result.linear()*=Eigen::AngleAxisd(theta,tilt/theta).toRotationMatrix();
    result.linear()*=Eigen::AngleAxisd(angles.z(),axis).toRotationMatrix();
    if(hit && !fixed) {
        if(distance+shift<=1e-8)return false;
        const Eigen::Vector3d anchor=reference.translation()+distance*reference.linear()*axis;
        result.translation()=anchor-(distance+shift)*result.linear()*axis;
    } else if(!hit && !fixed)result.translation()-=shift*reference.linear()*axis;
    if(fixed)return (result.translation()-reference.translation()).norm()<1e-14;
    return (result.translation()-reference.translation()).norm()<=o.maximumTcpAdjustment+1e-9;
}
} }
