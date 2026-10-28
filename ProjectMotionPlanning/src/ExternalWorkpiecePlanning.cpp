#include <ProjectMotionPlanning/ExternalWorkpiecePlanning.h>
#include <algorithm>
#include <chrono>
#include <cmath>

namespace motion_planning
{
    namespace
    {
        Eigen::Matrix3d rotation(const simulation_project::TransformDesc& p) {
            return (Eigen::AngleAxisd(p.yaw,Eigen::Vector3d::UnitZ())*
                Eigen::AngleAxisd(p.pitch,Eigen::Vector3d::UnitY())*
                Eigen::AngleAxisd(p.roll,Eigen::Vector3d::UnitX())).toRotationMatrix();
        }
        double halton(std::size_t n,int base) {
            double value=0,f=1.0/base;
            while(n) {value+=f*(n%base); n/=base; f/=base;} return value;
        }
    }
    bool ProjectExternalWorkpiecePlanner::poseAllowed(const simulation_project::TransformDesc& p,
        const WorkpiecePoseSearchOptions& o)
    {
        const Eigen::Vector3d position(p.x,p.y,p.z),rpy(p.roll,p.pitch,p.yaw);
        if(!position.allFinite() || !rpy.allFinite() || !o.lowerPosition.allFinite() || !o.upperPosition.allFinite() ||
            !o.lowerRpy.allFinite() || !o.upperRpy.allFinite() ||
            (position.array()<o.lowerPosition.array()-1e-12).any() || (position.array()>o.upperPosition.array()+1e-12).any() ||
            (rpy.array()<o.lowerRpy.array()-1e-12).any() || (rpy.array()>o.upperRpy.array()+1e-12).any()) return false;
        if(o.requireYVertical) {
            if(!o.groundNormalWorld.allFinite() || o.groundNormalWorld.norm()<1e-12 ||
                !std::isfinite(o.axisToleranceRad) || o.axisToleranceRad<0) return false;
            const Eigen::Vector3d y=rotation(p)*o.workpieceFromConstraintFrame.linear()*Eigen::Vector3d::UnitY();
            const double dot=y.normalized().dot(o.groundNormalWorld.normalized());
            return std::isfinite(dot) && std::acos(std::clamp(dot,-1.0,1.0))<=o.axisToleranceRad+1e-12;
        }
        return true;
    }
    ExternalWorkpiecePlanningResult ProjectExternalWorkpiecePlanner::plan(
        const simulation_project::ProjectDocument& document,const std::filesystem::path& basePath,
        const ExternalTrajectoryEvaluationRequest& evaluation,const WorkpiecePoseSearchOptions& search)
    {
        ExternalWorkpiecePlanningResult result;
        if(!search.enabled) {
            result.evaluation=ProjectExternalTrajectoryEvaluator::evaluate(document,basePath,evaluation);
            if(result.evaluation.success && evaluation.workpiece) result.selectedWorkpiecePose=evaluation.workpiece->transform;
            return result;
        }
        const auto invalid=[&](const std::string& message) {
            result.evaluation.diagnostics.push_back({"invalid_pose_search",message}); return result;
        };
        if(evaluation.frame!=ExternalTrajectoryFrame::Workpiece || evaluation.workpieceId.empty())
            return invalid("Pose search requires trajectories attached to a named workpiece frame.");
        auto request=evaluation;
        if(!request.workpiece) {
            const auto found=std::find_if(document.objects.begin(),document.objects.end(),[&](const auto& p){return p.id==request.workpieceId;});
            if(found==document.objects.end()) return invalid("Workpiece not found in scene snapshot.");
            request.workpiece=*found;
        }
        if(request.workpiece->id!=request.workpieceId) return invalid("Search workpiece and trajectory frame must match.");
        const auto& calibration=search.workpieceFromConstraintFrame;
        if(search.maxCandidates==0 || search.maxCandidates>1000 || !std::isfinite(search.maxDurationSeconds) || search.maxDurationSeconds<=0 ||
            !search.lowerPosition.allFinite() || !search.upperPosition.allFinite() || !search.lowerRpy.allFinite() || !search.upperRpy.allFinite() ||
            (search.lowerPosition.array()>search.upperPosition.array()).any() || (search.lowerRpy.array()>search.upperRpy.array()).any() ||
            !calibration.matrix().allFinite() || !(calibration.linear().transpose()*calibration.linear()).isApprox(Eigen::Matrix3d::Identity(),1e-8) ||
            std::abs(calibration.linear().determinant()-1)>1e-8 ||
            !std::isfinite(search.axisToleranceRad) || search.axisToleranceRad<0 || search.axisToleranceRad>3.141592653589793 ||
            !search.groundNormalWorld.allFinite() || search.groundNormalWorld.norm()<1e-12)
            return invalid("Invalid pose bounds, calibration, ground normal, tolerance or search budget.");
        const auto started=std::chrono::steady_clock::now();
        const auto timedOut=[&] {return std::chrono::duration<double>(std::chrono::steady_clock::now()-started).count()>=search.maxDurationSeconds;};
        const auto cancelled=evaluation.cancelled;
        request.cancelled=[&] {return timedOut() || (cancelled && cancelled());};
        const auto original=request.workpiece->transform;
        for(std::size_t i=0;i<search.maxCandidates;++i) {
            if(request.cancelled()) break;
            auto pose=original;
            if(i>0) {
                const int primes[]={2,3,5,7,11,13};
                Eigen::Vector3d position,rpy;
                for(int j=0;j<3;++j) {
                    position[j]=search.lowerPosition[j]+halton(i,primes[j])*(search.upperPosition[j]-search.lowerPosition[j]);
                    rpy[j]=search.lowerRpy[j]+halton(i,primes[j+3])*(search.upperRpy[j]-search.lowerRpy[j]);
                }
                if(search.requireYVertical) {
                    const Eigen::Vector3d normal=search.groundNormalWorld.normalized();
                    const Eigen::Quaterniond align=Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitY(),normal);
                    const Eigen::Matrix3d r=Eigen::AngleAxisd(rpy.z(),normal).toRotationMatrix()*align.toRotationMatrix()*calibration.linear().transpose();
                    rpy=Eigen::Vector3d(std::atan2(r(2,1),r(2,2)),std::asin(std::clamp(-r(2,0),-1.0,1.0)),std::atan2(r(1,0),r(0,0)));
                }
                pose.x=position.x(); pose.y=position.y(); pose.z=position.z();
                pose.roll=rpy.x(); pose.pitch=rpy.y(); pose.yaw=rpy.z();
            }
            if(!poseAllowed(pose,search)) {++result.rejectedByConstraints; continue;}
            request.workpiece->transform=pose;
            if(request.progress) request.progress("Workpiece pose candidate "+std::to_string(i+1)+" / "+std::to_string(search.maxCandidates));
            auto candidate=ProjectExternalTrajectoryEvaluator::evaluate(document,basePath,request);
            result.attempts.push_back({pose,candidate.success,candidate.diagnostics.empty()?"":candidate.diagnostics.back().message});
            result.evaluation=std::move(candidate);
            if(result.evaluation.success) {result.selectedWorkpiecePose=pose; return result;}
            if(!result.evaluation.retryWorkpiecePose) break; // Invalid input is not a reason to move a workpiece.
        }
        result.timedOut=timedOut();
        const bool userCancelled=cancelled && cancelled();
        if(result.timedOut || userCancelled) {
            result.evaluation={}; result.evaluation.cancelled=userCancelled;
            result.evaluation.diagnostics.push_back({result.timedOut?"pose_search_timeout":"cancelled","Search stopped without publishing a trajectory."});
        } else if(result.attempts.empty()) {
            result.evaluation.diagnostics.push_back({"no_admissible_pose","No sampled candidate satisfied the hard pose constraints."});
            result.evaluation.retryWorkpiecePose=true;
        } else if(!result.evaluation.success && result.evaluation.retryWorkpiecePose) {
            result.evaluation.diagnostics.push_back({"pose_search_budget_exhausted","No validated path found in this finite search; infeasibility is not proven."});
        }
        return result;
    }
}
