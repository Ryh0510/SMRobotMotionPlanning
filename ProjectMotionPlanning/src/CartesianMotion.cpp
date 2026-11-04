#include <ProjectMotionPlanning/CartesianMotion.h>
#include <RobotInstance/RobotInstance.h>
#include "GpuSerialIk.h"
#include <Eigen/Cholesky>
#include <algorithm>
#include <cmath>
#include <limits>

namespace motion_planning {
struct CartesianMotionSampler::Model {
    detail::GpuSerialIk chain;
    std::vector<double> lower, upper;
};
CartesianMotionSampler::CartesianMotionSampler() : m_model(std::make_unique<Model>()) {}
CartesianMotionSampler::~CartesianMotionSampler() = default;
std::shared_ptr<const CartesianMotionSampler> CartesianMotionSampler::create(ProjectPlanningSceneSnapshot& scene,
    const std::string& tcpLink,const Eigen::Isometry3d& linkFromTcp,std::string& error)
{
    auto out=std::shared_ptr<CartesianMotionSampler>(new CartesianMotionSampler);
    auto* runtime=scene.simulationRuntime().robot(scene.robotId());
    if(!runtime || scene.jointNames().size()!=6) {error="MoveL requires an actual six-axis robot.";return {};}
    struct RestoreState {
        robotinstance::RobotInstance& instance;
        std::vector<double> q;
        ~RestoreState() {instance.setJoints(q);instance.update();}
    } restore{*runtime->instance,runtime->instance->getState().q};
    CartesianMultiIkOptions options;std::vector<std::size_t> dofs;
    for(const auto& name:scene.jointNames()) {
        const auto& joint=runtime->model.joints.at(runtime->model.jointNameToIndex.at(name));
        if(joint.type!=robot::JointType::Revolute || joint.dofIndex<0) {error="Unsupported MoveL joint.";return {};}
        dofs.push_back(std::size_t(joint.dofIndex));
    }
    // Build a verified POE chain once; worker samples never touch mutable scenes.
    options.model.worldForwardKinematics=[runtime,&scene,tcpLink,linkFromTcp,dofs](const auto& q) {
        for(std::size_t i=0;i<dofs.size();++i)runtime->instance->setJoint(dofs[i],q[i]);
        runtime->instance->update();
        return Eigen::Isometry3d(scene.linkWorldTransform(tcpLink)*linkFromTcp);
    };
    for(const auto& bound:scene.jointBounds()) {
        options.lower.push_back(bound.continuous ? -std::numeric_limits<double>::infinity() : bound.lower);
        options.upper.push_back(bound.continuous ? std::numeric_limits<double>::infinity() : bound.upper);
    }
    if(!out->m_model->chain.configure(runtime->model,scene.jointNames(),tcpLink,options)) {
        error="MoveL actual-model verification failed: "+out->m_model->chain.status().dump();return {};
    }
    out->m_model->lower=options.lower;out->m_model->upper=options.upper;
    return out;
}
bool CartesianMotionSampler::isMoveL(const StoredMotionPlan& plan) {
    const auto value=nlohmann::json::parse(plan.optimizationMetadata,nullptr,false);
    return value.is_object() && value.contains("motion") && value["motion"].is_string() && value["motion"]=="cartesian_linear";
}
Eigen::Isometry3d CartesianMotionSampler::forward(const std::vector<double>& q) const {return m_model->chain.forward(q);}
bool CartesianMotionSampler::sample(const std::vector<double>& a,const std::vector<double>& b,double f,
    std::vector<double>& q,std::string& error) const
{
    q.clear();
    if(a.size()!=6 || b.size()!=6 || !std::isfinite(f) || f<0 || f>1) {error="Invalid MoveL sample.";return false;}
    for(int j=0;j<6;++j)if(!std::isfinite(a[j]) || !std::isfinite(b[j]) ||
        a[j]<m_model->lower[j]-1e-9 || a[j]>m_model->upper[j]+1e-9 ||
        b[j]<m_model->lower[j]-1e-9 || b[j]>m_model->upper[j]+1e-9) {error="MoveL encoder limit exceeded.";return false;}
    if(f==0) {q=a;return true;}if(f==1) {q=b;return true;}
    const auto from=forward(a),to=forward(b);
    const Eigen::Vector3d position=(1-f)*from.translation()+f*to.translation();
    const Eigen::Matrix3d rotation=Eigen::Quaterniond(from.linear()).slerp(f,Eigen::Quaterniond(to.linear())).toRotationMatrix();
    std::vector<double> candidate(6);for(int j=0;j<6;++j)candidate[j]=(1-f)*a[j]+f*b[j];
    for(int iteration=0;iteration<40;++iteration) {
        const auto actual=forward(candidate);Eigen::Matrix<double,6,1> residual;
        residual.head<3>()=position-actual.translation();
        const Eigen::AngleAxisd angle(rotation*actual.linear().transpose());residual.tail<3>()=angle.axis()*angle.angle();
        if(residual.head<3>().norm()<1e-10 && residual.tail<3>().norm()<1e-10) {q=std::move(candidate);return true;}
        const auto jacobian=m_model->chain.jacobian(candidate);
        const Eigen::Matrix<double,6,6> normal=jacobian*jacobian.transpose()+1e-12*Eigen::Matrix<double,6,6>::Identity();
        Eigen::Matrix<double,6,1> step=jacobian.transpose()*normal.ldlt().solve(residual);
        if(!step.allFinite())break;
        const double largest=step.cwiseAbs().maxCoeff();if(largest>.15)step*=.15/largest;
        bool improved=false;
        for(double scale=1.;scale>=1./128;scale*=.5) {
            auto next=candidate;for(int j=0;j<6;++j)next[j]=std::clamp(next[j]+scale*step[j],m_model->lower[j],m_model->upper[j]);
            const auto pose=forward(next);const Eigen::AngleAxisd change(rotation*pose.linear().transpose());
            const double merit=(position-pose.translation()).squaredNorm()+change.angle()*change.angle();
            if(merit<residual.squaredNorm()) {candidate=std::move(next);improved=true;break;}
        }
        if(!improved)break;
    }
    error="No continuous MoveL IK on the selected encoder branch.";return false;
}
bool CartesianMotionSampler::connection(const std::vector<double>& a,const std::vector<double>& b,
    double step,std::size_t budget,std::vector<std::vector<double>>& samples,std::string& error,
    const std::function<bool()>& cancelled) const
{
    samples.clear();
    if(a.size()!=6 || b.size()!=6 || !std::isfinite(step) || step<=0 || budget<2) {error="Invalid MoveL sampling budget.";return false;}
    double delta=0;for(int j=0;j<6;++j)delta=std::max(delta,std::abs(b[j]-a[j]));
    const double count=std::max(2.,std::ceil(delta/step));
    if(!std::isfinite(count) || count+1>double(budget)) {error="MoveL sample budget exceeded.";return false;}
    std::size_t intervals=std::size_t(count);
    for(;;) {
        if(cancelled && cancelled()) {error="MoveL cancelled.";return false;}
        std::vector<std::vector<double>> trial(intervals+1);double maximum=0;
        for(std::size_t k=0;k<=intervals;++k) {
            if(cancelled && cancelled()) {error="MoveL cancelled.";return false;}
            if(!sample(a,b,double(k)/intervals,trial[k],error))return false;
            if(k)for(int j=0;j<6;++j)maximum=std::max(maximum,std::abs(trial[k][j]-trial[k-1][j]));
        }
        if(maximum<=step*(1+1e-9)) {samples=std::move(trial);return true;}
        const double next=std::max(double(intervals)+1,std::ceil(intervals*maximum/step*1.01));
        if(next+1>double(budget)) {error="MoveL joint step/sample budget exceeded.";return false;}
        intervals=std::size_t(next);
    }
}
bool CartesianMotionSampler::execution(const robottrajectory::JointTrajectory& input,robottrajectory::JointTrajectory& output,
    std::string& error,double step,const std::function<bool()>& cancelled) const
{
    output=input;output.points.clear();if(input.points.empty())return true;
    output.points.push_back(input.points.front());
    for(std::size_t i=1;i<input.points.size();++i) {
        std::vector<std::vector<double>> internal;
        if(!connection(input.points[i-1].q,input.points[i].q,step,100000,internal,error,cancelled)) {output.points.clear();return false;}
        for(std::size_t k=1;k<internal.size();++k) {
            auto p=input.points[i];p.q=std::move(internal[k]);
            p.time=input.points[i-1].time+double(k)/(internal.size()-1)*(input.points[i].time-input.points[i-1].time);
            output.points.push_back(std::move(p));
        }
    }
    return true;
}
}
