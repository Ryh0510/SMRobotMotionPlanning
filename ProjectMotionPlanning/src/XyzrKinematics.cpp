#include <ProjectMotionPlanning/XyzrKinematics.h>
#include <ProjectMotionPlanning/GaApfCdfPipeline.h>
#include <SimulationRuntime/ProjectSimulationRuntime.h>
#include <RobotInstance/RobotInstance.h>
#include <Eigen/LU>
#include <nlohmann/json.hpp>
#include <algorithm>
#include <cmath>
#include <limits>
#include <set>

namespace motion_planning
{
    namespace
    {
        constexpr double pi = 3.14159265358979323846;
        bool rigid(const Eigen::Isometry3d& pose)
        {
            return pose.matrix().allFinite() &&
                (pose.linear().transpose()*pose.linear()-Eigen::Matrix3d::Identity()).norm()<1e-7 &&
                std::abs(pose.linear().determinant()-1.)<1e-7 &&
                (pose.matrix().row(3)-Eigen::RowVector4d(0,0,0,1)).norm()<1e-9;
        }
    }

    std::shared_ptr<const XyzrKinematics> XyzrKinematics::create(
        const simulation_project::ProjectDocument& document, const std::filesystem::path& basePath,
        const std::string& robotId, CartesianIkToolMode toolMode, std::string& error)
    {
        error.clear();
        simulation_runtime::ProjectSimulationRuntime runtime;
        const auto loaded=runtime.loadProject(document,basePath);
        if(!loaded.success) {error=loaded.message;return {};}
        auto* item=runtime.robot(robotId);
        if(!item || !item->instance) {error="Selected robot model is unavailable.";return {};}
        const auto& model=item->model;
        std::vector<const robot::RobotJoint*> moving;
        for(const auto& j:model.joints) if(j.dofIndex>=0 && !j.isLoop) moving.push_back(&j);
        if(moving.size()!=4) return {};
        // Follow the model tree instead of relying on initialJoints or joint vector order.
        const robot::RobotJoint* rotary=nullptr;
        for(const auto* j:moving) if(j->type==robot::JointType::Revolute) {
            if(rotary)return {};rotary=j;
        }
        if(!rotary)return {};
        std::vector<const robot::RobotJoint*> chain;
        std::string cursor=rotary->child;std::set<std::string> visited;
        while(visited.insert(cursor).second) {
            const auto j=std::find_if(model.joints.begin(),model.joints.end(),
                [&](const auto& v){return !v.isLoop && v.child==cursor;});
            if(j==model.joints.end())break;
            if(j->dofIndex>=0)chain.push_back(&*j);
            cursor=j->parent;
        }
        std::reverse(chain.begin(),chain.end());
        if(chain.size()!=4 || chain[3]!=rotary)return {};
        for(int i=0;i<3;++i)if(chain[i]->type!=robot::JointType::Prismatic)return {};
        auto out=std::shared_ptr<XyzrKinematics>(new XyzrKinematics);
        out->m_toolMode=toolMode;
        for(const auto* j:chain) {
            if(!j->continuous && (!j->hasPositionLimits || !std::isfinite(j->lowerPositionLimit) ||
                !std::isfinite(j->upperPositionLimit) || j->lowerPositionLimit>j->upperPositionLimit)) {
                error="Missing or invalid XYZR joint limits: "+j->name;return {};
            }
            out->m_names.push_back(j->name);
            out->m_lower.push_back(j->continuous ? -std::numeric_limits<double>::infinity() : j->lowerPositionLimit);
            out->m_upper.push_back(j->continuous ? std::numeric_limits<double>::infinity() : j->upperPositionLimit);
        }
        std::string link=rotary->child;
        Eigen::Isometry3d offset=Eigen::Isometry3d::Identity();
        if(toolMode==CartesianIkToolMode::FixedTool) {
            bool mounted=false;
            for(const auto& a:document.mountedAttachments) if(a.enabled) {
                const auto m=std::find_if(document.robotMounts.begin(),document.robotMounts.end(),
                    [&](const auto& v){return v.id==a.mountFrameId && v.robotId==robotId;});
                const auto asset=std::find_if(document.attachmentAssets.begin(),document.attachmentAssets.end(),
                    [&](const auto& v){return v.id==a.assetId && (v.assetKind.empty()?v.assetType:v.assetKind)=="tool";});
                mounted|=m!=document.robotMounts.end() && asset!=document.attachmentAssets.end();
            }
            if(mounted) {
                if(!resolvePlanningTcp(document,robotId,link,offset,error))return {};
            } else {
                // Calibration for this built-in spray gun, not an arbitrary four-axis tool.
                if(!model.links.count("tool0") || !model.links.count("spray_gun") ||
                    rotary->name!="yaw_joint" || rotary->child!="yaw_wrist") {
                    error="PPP+R robot requires an explicitly mounted tool with a saved TCP.";return {};
                }
                link="tool0";
                offset.linear()<<0,1,0, 0,0,1, 1,0,0;
            }
        }
        cursor=link;visited.clear();bool connected=false;
        while(visited.insert(cursor).second) {
            if(cursor==rotary->child) {connected=true;break;}
            auto j=std::find_if(model.joints.begin(),model.joints.end(),[&](const auto& v){return v.child==cursor && !v.isLoop;});
            if(j==model.joints.end() || j->type!=robot::JointType::Fixed)break;
            cursor=j->parent;
        }
        if(!connected) {error="XYZR TCP must be fixed to the rotary wrist.";return {};}
        item->instance->setBaseTransform(Eigen::Isometry3d::Identity());
        const auto fk=[&](const std::vector<double>& q)->Eigen::Isometry3d {
            for(int j=0;j<4;++j)item->instance->setJoint(std::size_t(chain[j]->dofIndex),q[j]);
            item->instance->update();
            return Eigen::Isometry3d(item->baseTransform*item->instance->getLinkTransform(link)*offset);
        };
        const std::vector<double> zero(4,0.);const auto home=fk(zero);
        if(!rigid(home)) {error="Invalid XYZR model forward kinematics.";return {};}
        out->m_homeRotation=home.linear();
        for(int j=0;j<3;++j) {
            auto probe=zero;probe[j]=.01;const auto pose=fk(probe);
            if((pose.linear()-home.linear()).norm()>1e-8) {error="XYZR translation joint changes orientation.";return {};}
            out->m_translation.col(j)=(pose.translation()-home.translation())/.01;
        }
        if(std::abs(out->m_translation.determinant())<1e-6) {error="XYZR linear axes are not independent.";return {};}
        out->m_inverseTranslation=out->m_translation.inverse();
        constexpr double angle=.1;
        auto probe=zero;probe[3]=angle;const auto plus=fk(probe);probe[3]=-angle;const auto minus=fk(probe);
        const Eigen::AngleAxisd turn(plus.linear()*home.linear().transpose());
        if(std::abs(turn.angle()-angle)>1e-8) {error="XYZR rotary model calibration failed.";return {};}
        out->m_axis=turn.axis();
        out->m_center=(plus.translation()+minus.translation()-2*std::cos(angle)*home.translation())/(2*(1-std::cos(angle)));
        out->m_cosine=home.translation()-out->m_center;
        out->m_sine=(plus.translation()-minus.translation())/(2*std::sin(angle));
        for(const auto& q:std::vector<std::vector<double>>{{.013,-.017,.021,.37},{-.027,.019,-.031,-.53},{.03,.02,.01,1.3}}) {
            if((out->forward(q).matrix()-fk(q).matrix()).norm()>1e-8) {
                error="XYZR analytic model does not match actual URDF forward kinematics.";return {};
            }
        }
        return out;
    }

    Eigen::Isometry3d XyzrKinematics::forward(const std::vector<double>& q) const
    {
        Eigen::Isometry3d pose=Eigen::Isometry3d::Identity();
        if(q.size()!=4 || !std::all_of(q.begin(),q.end(),[](double v){return std::isfinite(v);})) {
            pose.matrix().setConstant(std::numeric_limits<double>::quiet_NaN());return pose;
        }
        pose.linear()=Eigen::AngleAxisd(q[3],m_axis).toRotationMatrix()*m_homeRotation;
        pose.translation()=m_center+m_cosine*std::cos(q[3])+m_sine*std::sin(q[3])+
            m_translation*Eigen::Vector3d(q[0],q[1],q[2]);
        return pose;
    }

    bool XyzrKinematics::solvePose(const Eigen::Isometry3d& target,const std::vector<double>& seed,
        std::vector<double>& q,std::string& error,double tolerance) const
    {
        q.clear();error.clear();
        if(!rigid(target) || !std::isfinite(tolerance) || tolerance<=0) {error="Invalid XYZR target pose or tolerance.";return false;}
        if(!seed.empty() && (seed.size()!=4 || !std::all_of(seed.begin(),seed.end(),[](double v){return std::isfinite(v);}))) {
            error="XYZR seed requires four finite values (m, m, m, rad).";return false;
        }
        const Eigen::Matrix3d delta=target.linear()*m_homeRotation.transpose();
        const Eigen::Vector3d skew(delta(2,1)-delta(1,2),delta(0,2)-delta(2,0),delta(1,0)-delta(0,1));
        const double principal=std::atan2(.5*m_axis.dot(skew),.5*(delta.trace()-1.));
        double turns=std::round(((seed.empty()?0.:seed[3])-principal)/(2*pi));
        const double first=std::isfinite(m_lower[3])?std::ceil((m_lower[3]-principal)/(2*pi)-1e-10):-std::numeric_limits<double>::infinity();
        const double last=std::isfinite(m_upper[3])?std::floor((m_upper[3]-principal)/(2*pi)+1e-10):std::numeric_limits<double>::infinity();
        if(first>last) {error="XYZR yaw_joint exceeds the loaded URDF position limits.";return false;}
        turns=std::clamp(turns,first,last);const double angle=principal+2*pi*turns;
        const Eigen::Vector3d xyz=m_inverseTranslation*(target.translation()-m_center-m_cosine*std::cos(angle)-m_sine*std::sin(angle));
        std::vector<double> candidate={xyz.x(),xyz.y(),xyz.z(),angle};
        const auto actual=forward(candidate);
        const double rotationError=Eigen::AngleAxisd(target.linear()*actual.linear().transpose()).angle();
        if(!actual.matrix().allFinite() || (actual.translation()-target.translation()).norm()>tolerance || std::abs(rotationError)>tolerance) {
            error="Target orientation is incompatible with this four-axis XYZR robot.";return false;
        }
        for(int j=0;j<4;++j) {
            if(candidate[j]<m_lower[j]-1e-9 || candidate[j]>m_upper[j]+1e-9) {
                error="XYZR joint exceeds the loaded URDF position limits: "+m_names[j];return false;
            }
            candidate[j]=std::clamp(candidate[j],m_lower[j],m_upper[j]);
        }
        q=std::move(candidate);return true;
    }

    CartesianIkResult XyzrKinematics::solve(const StoredMotionPlan& plan,const CartesianIkOptions& options) const
    {
        CartesianIkResult result;result.plan=plan;result.plan.jointNames=m_names;
        result.plan.trajectory.points.clear();result.plan.trajectory.name=plan.name+"_xyzr_ik";
        result.plan.trajectory.interpolation=plan.cartesianControlPoints.interpolation;
        if(!options.jointNames.empty() && options.jointNames!=m_names) {
            result.diagnostics.push_back({"xyzr_joint_group","XYZR joint names must use the actual URDF chain order."});return result;
        }
        auto bounded=*this;
        if(!options.lower.empty() || !options.upper.empty()) {
            if(options.lower.size()!=4 || options.upper.size()!=4) {
                result.diagnostics.push_back({"invalid_ik_bounds","XYZR bounds require four SI joint intervals."});return result;
            }
            for(int j=0;j<4;++j) {
                if(std::isnan(options.lower[j]) || std::isnan(options.upper[j])) {
                    result.diagnostics.push_back({"invalid_ik_bounds","XYZR bounds cannot be NaN."});return result;
                }
                bounded.m_lower[j]=std::max(m_lower[j],options.lower[j]);bounded.m_upper[j]=std::min(m_upper[j],options.upper[j]);
                if(bounded.m_lower[j]>bounded.m_upper[j]) {
                    result.diagnostics.push_back({"invalid_ik_bounds","XYZR bounds have no intersection with the loaded model."});return result;
                }
            }
        }
        auto seed=options.seedJoints;
        for(std::size_t i=0;i<plan.cartesianControlPoints.points.size();++i) {
            const auto& point=plan.cartesianControlPoints.points[i];CartesianIkPointResult row;row.pointIndex=i;
            if(!std::isfinite(point.time) || point.time<0 || (i && point.time<plan.cartesianControlPoints.points[i-1].time))
                row.message="XYZR trajectory timestamps must be finite, nonnegative and ordered.";
            else row.success=bounded.solvePose(point.tcpPose,seed,row.joints,row.message,options.tolerance);
            if(row.success) {
                const auto actual=forward(row.joints);row.errorNorm=(actual.matrix()-point.tcpPose.matrix()).norm();
                robottrajectory::TimedJointPoint joint;joint.time=point.time;joint.q=row.joints;result.plan.trajectory.points.push_back(joint);
                if(options.usePreviousSolutionAsSeed)seed=row.joints;
            } else {
                row.errorNorm=std::numeric_limits<double>::infinity();
                result.diagnostics.push_back({"xyzr_ik_point_failed","Control point "+std::to_string(i+1)+": "+row.message});
            }
            result.points.push_back(std::move(row));
        }
        result.success=!result.points.empty() && result.plan.trajectory.points.size()==result.points.size();
        if(!result.success)result.plan.trajectory.points.clear();
        auto metadata=nlohmann::json::parse(plan.optimizationMetadata,nullptr,false);
        if(!metadata.is_object())metadata=nlohmann::json::object();
        metadata["ik_solver"]="xyzr_analytic_v1";metadata["joint_convention"]="runtime";
        metadata["xyzr_tool_mode"]=m_toolMode==CartesianIkToolMode::FixedTool?"process_tcp":"flange";
        metadata["motion"]="cartesian_linear";result.plan.optimizationMetadata=metadata.dump();
        return result;
    }

    bool XyzrKinematics::sample(const std::vector<double>& from,const std::vector<double>& to,double fraction,
        std::vector<double>& q,std::string& error) const
    {
        q.clear();error.clear();
        if(from.size()!=4 || to.size()!=4 || !std::isfinite(fraction) || fraction<0 || fraction>1) {error="Invalid XYZR playback sample.";return false;}
        for(int j=0;j<4;++j)if(!std::isfinite(from[j]) || !std::isfinite(to[j]) || from[j]<m_lower[j]-1e-9 ||
            from[j]>m_upper[j]+1e-9 || to[j]<m_lower[j]-1e-9 || to[j]>m_upper[j]+1e-9) {
            error="XYZR playback endpoint exceeds the loaded model limits.";return false;
        }
        if(fraction==0) {q=from;return true;}if(fraction==1) {q=to;return true;}
        const auto a=forward(from),b=forward(to);Eigen::Isometry3d target=Eigen::Isometry3d::Identity();
        target.translation()=(1-fraction)*a.translation()+fraction*b.translation();
        const double angle=(1-fraction)*from[3]+fraction*to[3];
        target.linear()=Eigen::AngleAxisd(angle,m_axis).toRotationMatrix()*m_homeRotation;
        std::vector<double> seed(4,0);seed[3]=angle;
        return solvePose(target,seed,q,error,1e-9);
    }
}
