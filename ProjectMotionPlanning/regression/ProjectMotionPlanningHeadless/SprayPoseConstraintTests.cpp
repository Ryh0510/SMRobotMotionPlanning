#include "../../src/SprayPoseConstraint.h"
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
using namespace motion_planning;
void check(bool condition,const char* message) {if(!condition)throw std::runtime_error(message);}
int main() {
    try {
        SprayAxisContinuityOptions o;o.maximumTiltRadians=3.141592653589793;o.maximumRollRadians=3.141592653589793;o.maximumTcpAdjustment=.5;
        const double nan=std::numeric_limits<double>::quiet_NaN();int checks=0;
        for(const Eigen::Vector3d axis:std::vector<Eigen::Vector3d>{Eigen::Vector3d::UnitX(),-Eigen::Vector3d::UnitY(),Eigen::Vector3d::UnitZ(),Eigen::Vector3d(1,2,-3).normalized()})
        for(double d:{.001,.1,.4})for(double a:{-.3,0.,.3}) {
            Eigen::Isometry3d ref=Eigen::Isometry3d::Identity(),out;
            ref.linear()=(Eigen::AngleAxisd(.7,Eigen::Vector3d::UnitY())*Eigen::AngleAxisd(-.4,Eigen::Vector3d::UnitX())).toRotationMatrix();
            ref.translation()=Eigen::Vector3d(.2,-.1,.8);
            const Eigen::Vector3d hit=ref.translation()+d*ref.linear()*axis;
            o.poseConstraint=SprayPoseConstraint::FixedTcp;
            check(detail::constrainedSprayPose(ref,axis,d,Eigen::Vector3d(0,0,a),0,o,out),"Fixed TCP axial roll accepted");
            check((out.translation()-ref.translation()).norm()==0,"TCP remains exactly unchanged");
            check((out.translation()+d*out.linear()*axis-hit).norm()<1e-12,"Roll retains hit and range");
            check(!detail::constrainedSprayPose(ref,axis,d,Eigen::Vector3d(.1,0,0),0,o,out),"Fixed TCP plus fixed hit forbids tilt");
            check(!detail::constrainedSprayPose(ref,axis,d,Eigen::Vector3d::Zero(),.01,o,out),"Fixed TCP rejects range shift");
            check(detail::constrainedSprayPose(ref,axis,nan,Eigen::Vector3d(.1,0,a),0,o,out),"Fixed transfer can reorient in place");
            check((out.translation()-ref.translation()).norm()==0,"Transfer TCP fixed");
            o.poseConstraint=SprayPoseConstraint::SurfacePivotFixedDistance;
            check(detail::constrainedSprayPose(ref,axis,d,Eigen::Vector3d(.2,.1,a),0,o,out),"Surface pivot accepted");
            check((out.translation()+d*out.linear()*axis-hit).norm()<1e-12,"Pivot retains original hit and original range");
            check((out.translation()-ref.translation()).norm()>1e-5,"Tilting at surface moves TCP");
            // Independent ray intersection with the original normal plane.
            const Eigen::Vector3d normal=ref.linear()*axis,ray=out.linear()*axis;
            const double range=normal.dot(hit-out.translation())/normal.dot(ray);
            check(std::abs(range-d)<1e-12 && (out.translation()+range*ray-hit).norm()<1e-12,"Independent plane first hit unchanged");
            check(!detail::constrainedSprayPose(ref,axis,d,Eigen::Vector3d::Zero(),.01,o,out),"Equal-distance pivot forbids standoff optimization");
            const double budget=o.maximumTcpAdjustment;o.maximumTcpAdjustment=0.;
            check(!detail::constrainedSprayPose(ref,axis,d,Eigen::Vector3d(.1,0,0),0,o,out),"Zero position budget rejects tilt");o.maximumTcpAdjustment=budget;
            o.poseConstraint=SprayPoseConstraint::SurfacePivotAdjustableDistance;
            check(detail::constrainedSprayPose(ref,axis,d,Eigen::Vector3d(.1,0,a),.01,o,out),"Explicit legacy range shift accepted");
            check((out.translation()+(d+.01)*out.linear()*axis-hit).norm()<1e-12,"Legacy anchor retained");
            ++checks;
        }
        Eigen::Isometry3d ref=Eigen::Isometry3d::Identity(),out;
        check(!detail::constrainedSprayPose(ref,Eigen::Vector3d(0,0,2),.1,Eigen::Vector3d::Zero(),0,o,out),"Nonunit axis rejected");
        check(!detail::constrainedSprayPose(ref,Eigen::Vector3d::UnitZ(),.1,Eigen::Vector3d(nan,0,0),0,o,out),"Nonfinite angles rejected");
        o.allowNonSprayingTransitions=false;
        check(!detail::constrainedSprayPose(ref,Eigen::Vector3d::UnitZ(),nan,Eigen::Vector3d::Zero(),0,o,out),"Forbidden transfer rejected");
        std::cout<<checks<<" arbitrary-axis/ref-frame/range geometry cases passed\n";return 0;
    } catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}
