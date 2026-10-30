#include <ProjectMotionPlanning/GaApfCdfPipeline.h>
#include <nlohmann/json.hpp>
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace motion_planning
{
    bool prepareGaApfDownsamplingRequest(ExternalTrajectoryEvaluationRequest& request, std::string& error)
    {
        try {
            const auto data=nlohmann::json::parse(request.plan.optimizationMetadata);
            if(data.at("type")!="ga_apf_surface_reference" || data.at("version")!=1 ||
                data.at("robot_id")!=request.plan.robotId || data.at("joint_names").get<std::vector<std::string>>()!=request.plan.jointNames ||
                data.at("tcp_link")!=request.tcpLinkName)
                throw std::runtime_error("GA/APF reference does not match the selected robot, joints or mounted TCP.");
            const auto pose=[](const nlohmann::json& values) {
                if(!values.is_array() || values.size()!=16)throw std::runtime_error("Invalid saved TCP matrix.");
                Eigen::Isometry3d value=Eigen::Isometry3d::Identity();
                for(int r=0;r<4;++r)for(int c=0;c<4;++c)value.matrix()(r,c)=values.at(4*r+c).get<double>();
                if(!value.matrix().allFinite() || !(value.linear().transpose()*value.linear()).isApprox(Eigen::Matrix3d::Identity(),1e-8) ||
                    std::abs(value.linear().determinant()-1.)>1e-8 || !value.matrix().row(3).isApprox(Eigen::RowVector4d(0,0,0,1),1e-8))
                    throw std::runtime_error("Non-rigid saved TCP matrix.");
                return value;
            };
            if(!pose(data.at("link_from_tcp")).matrix().isApprox(request.linkFromTcp.matrix(),1e-10))
                throw std::runtime_error("Mounted TCP changed since GA/APF optimization; regenerate the trajectory.");
            const auto times=data.at("output_times_s").get<std::vector<double>>();
            const auto source=data.at("source_times_s").get<std::vector<double>>();
            const auto selected=data.at("selected_distances_m").get<std::vector<double>>();
            const auto& input=request.plan.trajectory.points;
            const auto& controls=data.at("source_controls");
            const double nominal=data.at("nominal_distance_m"),clearance=data.at("clearance_m");
            if(times.size()!=input.size() || times.size()<2 || source.size()!=times.size() || controls.size()<2 ||
                selected.size()!=controls.size() || !std::isfinite(nominal) || nominal<=0 || !std::isfinite(clearance) || clearance<0)
                throw std::runtime_error("Incomplete saved GA/APF surface/time reference.");
            for(std::size_t i=0;i<times.size();++i) {
                if(!std::isfinite(times[i]) || !std::isfinite(source[i]) || std::abs(times[i]-input[i].time)>1e-8 ||
                    (i && (times[i]<=times[i-1] || source[i]<=source[i-1])))
                    throw std::runtime_error("Trajectory timing changed or its saved surface correspondence is invalid.");
            }
            auto prepared=request;prepared.plan.cartesianControlPoints.points.clear();
            prepared.sprayTargets.clear();prepared.sprayDistances=selected;
            prepared.gaNominalSprayDistances.assign(controls.size(),nominal);
            double last=-1.;
            for(std::size_t i=0;i<controls.size();++i) {
                const double t=controls.at(i).at("time_s");
                if(!std::isfinite(t) || t<=last || t<source.front()-1e-8 || t>source.back()+1e-8 ||
                    !std::isfinite(selected[i]) || selected[i]<nominal*.2-1e-8 || selected[i]>nominal*1.8+1e-8)
                    throw std::runtime_error("Invalid original station or selected GA spray distance.");
                const auto original=pose(controls.at(i).at("matrix"));last=t;
                auto upper=std::upper_bound(source.begin(),source.end(),t);
                double mapped;
                if(upper==source.begin())mapped=times.front();
                else if(upper==source.end())mapped=times.back();
                else {const auto b=std::size_t(upper-source.begin()),a=b-1;const double f=(t-source[a])/(source[b]-source[a]);mapped=(1-f)*times[a]+f*times[b];}
                robottrajectory::TimedCartesianPoint point;point.time=mapped;point.tcpPose=original;
                prepared.plan.cartesianControlPoints.points.push_back(point);
                prepared.sprayTargets.push_back(original.translation()+nominal*original.linear()*prepared.sprayAxis);
            }
            if(std::abs(controls.front().at("time_s").get<double>()-source.front())>1e-8 ||
                std::abs(last-source.back())>1e-8)throw std::runtime_error("Saved source boundaries do not cover the complete trajectory.");
            prepared.pipelineStage="downsample";prepared.verificationTrajectory=request.plan.trajectory;
            prepared.optimizationSeed={};prepared.optimizeAvoidance=false;prepared.controlPointsOnly=false;
            prepared.allowSprayAxisRotation=true;prepared.pointSearchMaxAngleDeg=80.;
            prepared.maximumTcpPositionDeviation=.001;prepared.maximumTcpOrientationDeviation=1e-6;
            prepared.validationJointStep=.00025;prepared.minimumClearance=std::max(prepared.minimumClearance,clearance);
            prepared.pointSearchReferenceDelta.clear();prepared.pointSearchMinDistances.clear();prepared.pointSearchMaxDistances.clear();
            request=std::move(prepared);error.clear();return true;
        } catch(const std::exception& e) {error=e.what();return false;}
    }
}
