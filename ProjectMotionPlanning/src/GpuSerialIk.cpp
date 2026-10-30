#include "GpuSerialIk.h"
#include <algorithm>
#include <set>
#include <stdexcept>
#include <cmath>
#include <Eigen/Cholesky>
namespace motion_planning { namespace detail {
bool GpuSerialIk::definitelyUnreachable(const Eigen::Isometry3d& target) const {
    if(!reachBoundReady_ || !target.matrix().allFinite())return false;
    const Eigen::Vector3d d=target*wristInTcp_-reachAxisPoint_;
    const double axial=d.dot(reachAxis_),z=axial-shoulderAxial_;
    const double radius=(d-axial*reachAxis_).norm();
    const double near=std::pow(radius-shoulderRadius_,2)+z*z;
    const double far=std::pow(radius+shoulderRadius_,2)+z*z;
    return near>std::pow(reachMaximum_+reachPadding_,2) || far<std::pow(std::max(0.,reachMinimum_-reachPadding_),2);
}
Eigen::Isometry3d GpuSerialIk::forward(const std::vector<double>& q) const {
    if(!configured_ || q.size()!=6)throw std::runtime_error("Unverified serial FK model");
    Eigen::Isometry3d out=Eigen::Isometry3d::Identity(),home=Eigen::Isometry3d::Identity();
    for(int a=0;a<3;++a)for(int b=0;b<4;++b)home.matrix()(a,b)=model_[36+4*a+b];
    for(int i=0;i<6;++i){
        const Eigen::Vector3d w(model_[6*i],model_[6*i+1],model_[6*i+2]),p(model_[6*i+3],model_[6*i+4],model_[6*i+5]);
        Eigen::Isometry3d step=Eigen::Isometry3d::Identity();step.linear()=Eigen::AngleAxisd(q[i],w).toRotationMatrix();
        step.translation()=p-step.linear()*p;out=out*step;
    }
    return Eigen::Isometry3d(out*home);
}
Eigen::Matrix<double,6,6> GpuSerialIk::jacobian(const std::vector<double>& q) const {
    if(!configured_ || q.size()!=6)throw std::runtime_error("Unverified serial Jacobian model");
    Eigen::Isometry3d prefix=Eigen::Isometry3d::Identity();
    Eigen::Vector3d axes[6],centers[6];
    for(int i=0;i<6;++i) {
        const Eigen::Vector3d w(model_[6*i],model_[6*i+1],model_[6*i+2]),p(model_[6*i+3],model_[6*i+4],model_[6*i+5]);
        axes[i]=prefix.linear()*w;centers[i]=prefix*p;
        Eigen::Isometry3d step=Eigen::Isometry3d::Identity();step.linear()=Eigen::AngleAxisd(q[i],w).toRotationMatrix();
        step.translation()=p-step.linear()*p;prefix=prefix*step;
    }
    const Eigen::Vector3d home(model_[39],model_[43],model_[47]);
    const Eigen::Vector3d tcp=prefix*home;Eigen::Matrix<double,6,6> out;
    for(int i=0;i<6;++i){out.block<3,1>(0,i)=axes[i].cross(tcp-centers[i]);out.block<3,1>(3,i)=axes[i];}
    return out;
}
bool GpuSerialIk::configure(const robot::RobotModel& robot,const std::vector<std::string>& names,const std::string& tcpLink,const CartesianMultiIkOptions& options){
    reachBoundReady_=false;
    try {
        if(names.size()!=6 || !options.model.worldForwardKinematics || options.lower.size()!=6 || options.upper.size()!=6)throw std::runtime_error("GPU IK needs actual-model FK and six joint bounds");
        std::vector<std::string> chain;std::set<std::string> visited;auto link=tcpLink;
        while(visited.insert(link).second){const robot::RobotJoint* parent=nullptr;
            for(const auto& j:robot.joints)if(j.child==link){if(parent || j.isLoop)throw std::runtime_error("GPU IK does not support loop/ambiguous chains");parent=&j;}
            if(!parent)break;
            if(parent->type!=robot::JointType::Fixed){if(parent->type!=robot::JointType::Revolute || parent->dofIndex<0)throw std::runtime_error("GPU IK requires six independent revolute joints");chain.push_back(parent->name);}
            link=parent->parent;
        }
        std::reverse(chain.begin(),chain.end());if(chain!=names)throw std::runtime_error("GPU IK joint order must follow the actual TCP ancestor chain");
        options_=options;std::vector<double> zero(6,0);const auto home=options.model.worldForwardKinematics(zero);
        for(int a=0;a<3;++a)for(int b=0;b<4;++b)model_[36+4*a+b]=home.matrix()(a,b);
        // Read each actual world screw through FK, retaining all URDF origin,
        // base, fixed-link and selected TCP transforms. No nominal DH model.
        for(int i=0;i<6;++i){auto q=zero;q[i]=.25;const Eigen::Isometry3d delta=options.model.worldForwardKinematics(q)*home.inverse();Eigen::AngleAxisd angle(delta.linear());
            if(std::abs(angle.angle()-.25)>1e-8)throw std::runtime_error("Unsupported coupled/scaled GPU IK joint");
            const auto w=angle.axis().eval(),t=delta.translation().eval();
            if(std::abs(w.dot(t))>1e-9)throw std::runtime_error("Unsupported helical GPU IK joint");
            const Eigen::Vector3d p=.5*(t+w.cross(t)/std::tan(.125));for(int j=0;j<3;++j){model_[6*i+j]=w[j];model_[6*i+3+j]=p[j];}
        }
        for(int sample=0;sample<32;++sample){std::vector<double> q(6);Eigen::Isometry3d predicted=Eigen::Isometry3d::Identity();
            for(int i=0;i<6;++i){q[i]=1.1*std::sin((sample+1)*(i+1)*.371);const Eigen::Vector3d w(model_[6*i],model_[6*i+1],model_[6*i+2]),p(model_[6*i+3],model_[6*i+4],model_[6*i+5]);Eigen::Isometry3d step=Eigen::Isometry3d::Identity();step.linear()=Eigen::AngleAxisd(q[i],w).toRotationMatrix();step.translation()=p-step.linear()*p;predicted=predicted*step;}
            predicted=predicted*home;const double error=(predicted.matrix()-options.model.worldForwardKinematics(q).matrix()).norm();modelErrorMax_=std::max(modelErrorMax_,error);
            if(!std::isfinite(error) || error>1e-8)throw std::runtime_error("GPU IK exported chain disagrees with actual-model FK");
        }
        configured_=true;
        // Independently verify the derivative including base, fixed links and mounted TCP.
        for(int sample=0;sample<8;++sample){std::vector<double> q(6);for(int j=0;j<6;++j)q[j]=.7*std::sin((sample+1)*(j+1)*.371);
            const auto analytic=jacobian(q);Eigen::Matrix<double,6,6> numeric;
            for(int j=0;j<6;++j){auto lo=q,hi=q;lo[j]-=1e-6;hi[j]+=1e-6;
                const auto a=options.model.worldForwardKinematics(lo),b=options.model.worldForwardKinematics(hi);
                numeric.block<3,1>(0,j)=(b.translation()-a.translation())/2e-6;
                const Eigen::AngleAxisd aa(b.linear()*a.linear().transpose());numeric.block<3,1>(3,j)=aa.axis()*aa.angle()/2e-6;}
            if(!numeric.allFinite() || (analytic-numeric).norm()>1e-7)throw std::runtime_error("Serial Jacobian disagrees with actual-model derivative");
        }
        // A common point of the last three screw axes is unaffected by wrist
        // rotations. Its desired world position is therefore target*wristInTcp.
        // Joint 2's pivot moves on a circle under joint 1. The remaining two arm
        // links have fixed lengths, so their triangle inequality bounds the
        // distance from that circle to the wrist. This is a necessary condition
        // over ALL joint angles, independent of IK seeds or mechanical limits.
        Eigen::Matrix3d normal=Eigen::Matrix3d::Zero();Eigen::Vector3d rhs=Eigen::Vector3d::Zero();
        for(int i=3;i<6;++i) {
            const Eigen::Vector3d w(model_[6*i],model_[6*i+1],model_[6*i+2]),p(model_[6*i+3],model_[6*i+4],model_[6*i+5]);
            const Eigen::Matrix3d projection=Eigen::Matrix3d::Identity()-w*w.transpose();normal+=projection;rhs+=projection*p;
        }
        if(std::abs(normal.determinant())>1e-10) {
            const Eigen::Vector3d wrist=normal.ldlt().solve(rhs);
            double residual=0;
            for(int i=3;i<6;++i) {
                const Eigen::Vector3d w(model_[6*i],model_[6*i+1],model_[6*i+2]),p(model_[6*i+3],model_[6*i+4],model_[6*i+5]);
                residual=std::max(residual,(wrist-p).cross(w).norm());
            }
            if(wrist.allFinite() && residual<1e-9) {
                reachAxis_=Eigen::Vector3d(model_[0],model_[1],model_[2]);
                reachAxisPoint_=Eigen::Vector3d(model_[3],model_[4],model_[5]);
                const Eigen::Vector3d shoulder(model_[9],model_[10],model_[11]);
                const Eigen::Vector3d elbowAxis(model_[12],model_[13],model_[14]),elbowOrigin(model_[15],model_[16],model_[17]);
                const Eigen::Vector3d elbow=elbowOrigin+elbowAxis.dot(shoulder-elbowOrigin)*elbowAxis;
                const double upper=(elbow-shoulder).norm(),forearm=(wrist-elbow).norm();
                const Eigen::Vector3d offset=shoulder-reachAxisPoint_;
                shoulderAxial_=offset.dot(reachAxis_);shoulderRadius_=(offset-shoulderAxial_*reachAxis_).norm();
                wristInTcp_=home.inverse()*wrist;
                reachMinimum_=std::abs(upper-forearm);reachMaximum_=upper+forearm;
                // Include accepted TCP pose error and a larger numerical buffer.
                reachPadding_=std::max(1e-5,4*(options.positionTolerance+options.orientationTolerance*wristInTcp_.norm()));
                reachBoundReady_=std::isfinite(reachMaximum_) && reachMaximum_>1e-8;
                for(int n=0;reachBoundReady_ && n<64;++n) {
                    std::vector<double> q(6);for(int j=0;j<6;++j)q[j]=6*std::sin((n+1)*(j+1)*.371);
                    if(definitelyUnreachable(options.model.worldForwardKinematics(q)))reachBoundReady_=false;
                }
            }
        }
        return true;
    }catch(const std::exception& e){modelError_=e.what();configured_=false;return false;}
}
bool GpuSerialIk::solve(const std::vector<Eigen::Isometry3d>& targets,const std::vector<std::vector<double>>& seeds,std::vector<std::vector<CartesianIkCandidate>>& candidates,bool preferNearSeed){
    candidates.clear();if(!configured_ || seeds.empty() || targets.size()*seeds.size()>4096)return false;
    candidates.resize(targets.size());std::vector<GpuIkTask> tasks;std::vector<std::size_t> owners;
    for(std::size_t owner=0;owner<targets.size();++owner) {
        const auto& target=targets[owner];if(definitelyUnreachable(target)){++reachRejectedTargets_;continue;}
        for(const auto& seed:seeds){if(seed.size()!=6)return false;GpuIkTask t;for(int a=0;a<3;++a)for(int b=0;b<4;++b)t.target[4*a+b]=target.matrix()(a,b);std::copy(seed.begin(),seed.end(),t.seed.begin());tasks.push_back(t);owners.push_back(owner);}
    }
    if(tasks.empty())return true;
    std::vector<GpuIkRoot> roots;if(!gpu_.solveIk(model_,tasks,roots,options_.maxIterations,options_.positionTolerance*.5,options_.orientationTolerance*.5,options_.model.damping,1.))return false;
    const double pi=std::acos(-1.);
    for(std::size_t i=0;i<roots.size();++i){if(options_.cancelled && options_.cancelled())return false;if(!roots[i].converged)continue;const std::size_t owner=owners[i];auto& out=candidates[owner];auto root=roots[i].joints;for(double& x:root)x=std::remainder(x,2*pi);
        // Keep turns in the configured actual search window and verify every
        // returned root with the independent CPU actual-model FK callback.
        std::vector<double> q(6);std::function<void(int)> expand=[&](int j){
            if(out.size()>=options_.maxCandidatesPerPoint || (options_.cancelled && options_.cancelled()))return;
            if(j<6){int first=int(std::ceil((options_.lower[j]-root[j]-1e-10)/(2*pi))),last=int(std::floor((options_.upper[j]-root[j]+1e-10)/(2*pi)));
                if(preferNearSeed && first<=last){const auto near=int(std::llround((seeds[i%seeds.size()][j]-root[j])/(2*pi)));first=last=std::clamp(near,first,last);}
                for(int turn=first;turn<=last && out.size()<options_.maxCandidatesPerPoint;++turn){q[j]=std::clamp(root[j]+turn*2*pi,options_.lower[j],options_.upper[j]);expand(j+1);}return;}
            if(std::any_of(out.begin(),out.end(),[&](const auto& c){double d=0;for(int k=0;k<6;++k)d=std::max(d,std::abs(c.joints[k]-q[k]));return d<options_.duplicateTolerance;}))return;
            // The exported serial model is verified against the runtime in configure.
            // Use its pure CPU FK for parallel-search residuals, without mutating the
            // complete scene per root. Near a tolerance boundary use the runtime FK
            // so numerical differences cannot change the original admission decision.
            auto actual=forward(q);
            double position=(actual.translation()-targets[owner].translation()).norm();
            double orientation=Eigen::Quaterniond(actual.linear()).angularDistance(Eigen::Quaterniond(targets[owner].linear()));
            if(std::abs(position-options_.positionTolerance)<1e-8 || std::abs(orientation-options_.orientationTolerance)<1e-8) {
                actual=options_.model.worldForwardKinematics(q);
                position=(actual.translation()-targets[owner].translation()).norm();
                orientation=Eigen::Quaterniond(actual.linear()).angularDistance(Eigen::Quaterniond(targets[owner].linear()));
            }
            if(!actual.matrix().allFinite() || !std::isfinite(position) || !std::isfinite(orientation) || position>options_.positionTolerance || orientation>options_.orientationTolerance){++cpuFkRejected_;return;}
            CartesianIkCandidate c;c.joints=q;c.positionError=position;c.orientationError=orientation;for(double x:q)c.turns.push_back(int(std::floor((x+pi)/(2*pi))));if(options_.classifyConfiguration)c.configuration=options_.classifyConfiguration(q);out.push_back(std::move(c));++verifiedRoots_;
        };expand(0);
    }
    return true;
}
nlohmann::json GpuSerialIk::status()const{auto out=gpu_.status();out["model_verified"]=configured_;out["model_error"]=modelError_;out["model_fk_error_max"]=modelErrorMax_;out["cpu_fk_rejected"]=cpuFkRejected_;out["cpu_fk_verified_roots"]=verifiedRoots_;out["spherical_wrist_reach_bound"]=reachBoundReady_;out["geometrically_unreachable_targets"]=reachRejectedTargets_;return out;}
}}
