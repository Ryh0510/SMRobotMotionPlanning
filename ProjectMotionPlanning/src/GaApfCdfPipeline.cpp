#include <ProjectMotionPlanning/GaApfCdfPipeline.h>
#include <nlohmann/json.hpp>
#include <algorithm>
#include <stdexcept>
#include <chrono>

namespace motion_planning
{
    namespace
    {
        Eigen::Isometry3d transform(const simulation_project::TransformDesc& p)
        {
            Eigen::Isometry3d t=Eigen::Isometry3d::Identity();
            t.translation()=Eigen::Vector3d(p.x,p.y,p.z);
            t.linear()=(Eigen::AngleAxisd(p.yaw,Eigen::Vector3d::UnitZ())*
                Eigen::AngleAxisd(p.pitch,Eigen::Vector3d::UnitY())*
                Eigen::AngleAxisd(p.roll,Eigen::Vector3d::UnitX())).toRotationMatrix();
            return t;
        }
    }

    bool resolvePlanningTcp(const simulation_project::ProjectDocument& document,
        const std::string& robotId, std::string& linkName,
        Eigen::Isometry3d& linkFromTcp, std::string& error)
    {
        int tools=0;
        for(const auto& a:document.mountedAttachments) {
            if(!a.enabled) continue;
            const auto mount=std::find_if(document.robotMounts.begin(),document.robotMounts.end(),
                [&](const auto& m){return m.id==a.mountFrameId && m.robotId==robotId;});
            const auto asset=std::find_if(document.attachmentAssets.begin(),document.attachmentAssets.end(),
                [&](const auto& v){return v.id==a.assetId;});
            if(mount==document.robotMounts.end() || asset==document.attachmentAssets.end() ||
                (asset->assetKind.empty()?asset->assetType:asset->assetKind)!="tool") continue;
            ++tools;
            const simulation_project::AttachmentFunctionalFrameDesc* tcp=nullptr;
            int count=0,primary=0;
            for(const auto& f:asset->functionalFrames) if(f.frameType=="tcp") {
                ++count; if(!tcp || f.primary) tcp=&f; if(f.primary) ++primary;
            }
            if(!tcp || primary>1 || (count>1 && primary!=1)) {
                error="Tool requires one unambiguous saved TCP."; return false;
            }
            linkName=mount->linkName;
            linkFromTcp=transform(mount->linkToMount)*transform(a.mountToAssetMount)*transform(tcp->assetMountToFrame);
        }
        if(tools!=1) { error="Select exactly one enabled tool with a saved TCP for the planning robot."; return false; }
        return true;
    }

    ProjectCdfQpRepairResult runGaApfCdfPipeline(
        const simulation_project::ProjectDocument& document,
        const std::filesystem::path& basePath,
        ExternalTrajectoryEvaluationRequest request, bool useCdf,
        const GaSurfaceSearchOptions& surface)
    {
        ProjectCdfQpRepairResult result;
        const auto fail=[&](const std::string& code,const std::string& message) {
            result.success=false; result.plan={};
            result.cancelled=request.cancelled && request.cancelled();
            result.diagnostics.push_back({code,message}); return result;
        };
        try {
            if(request.cancelled && request.cancelled())return fail("pipeline_cancelled","Cancelled before scene construction.");
            if(!std::isfinite(surface.sprayDistance) || surface.sprayDistance<=0 || surface.sprayDistance>10)
                return fail("pipeline_spray_distance","Spray distance must be positive, finite and at most 10 metres.");
            std::string error;
            if(request.tcpLinkName.empty() && !resolvePlanningTcp(document,request.plan.robotId,
                request.tcpLinkName,request.linkFromTcp,error)) return fail("pipeline_tcp_missing",error);
            if(request.collisionDetectorIds.empty()) return fail("pipeline_detector_missing","Select a robot/workpiece collision detector.");
            ProjectPlanningRequest sceneRequest;
            sceneRequest.robotId=request.plan.robotId;sceneRequest.jointNames=request.plan.jointNames;
            sceneRequest.collisionDetectorIds=request.collisionDetectorIds;
            auto scene=ProjectPlanningSceneBuilder::build(document,basePath,sceneRequest,&error);
            if(!scene)return fail("pipeline_scene_failed",error);
            const auto fk=[&](const std::vector<double>& q) -> Eigen::Isometry3d {
                if(!scene->setState(q,&error))throw std::runtime_error(error);
                return scene->linkWorldTransform(request.tcpLinkName)*request.linkFromTcp;
            };
            // Joint imports acquire Cartesian controls from the actual mounted TCP.
            if(request.plan.cartesianControlPoints.points.empty()) {
                for(const auto& joint:request.plan.trajectory.points) {
                    robottrajectory::TimedCartesianPoint p;p.time=joint.time;p.tcpPose=fk(joint.q);
                    request.plan.cartesianControlPoints.points.push_back(p);
                }
            }
            if(request.plan.cartesianControlPoints.points.size()<2)
                return fail("pipeline_input_too_short","At least two Cartesian controls or joint knots are required.");
            const auto originalControls=request.plan.cartesianControlPoints;
            std::vector<Eigen::Vector3d> anchors;
            for(const auto& point:originalControls.points)
                anchors.push_back(point.tcpPose.translation()+surface.sprayDistance*point.tcpPose.linear()*request.sprayAxis);
            std::vector<double> radiusFractions(originalControls.points.size(),0.);
            std::vector<double> selectedDistances(originalControls.points.size(),surface.sprayDistance);
            bool distancesLocked=false;
            std::vector<std::pair<double,double>> sourceTimes;
            for(const auto& p:request.plan.cartesianControlPoints.points)sourceTimes.emplace_back(p.time,p.time);
            const auto sourceTimeAt=[](const auto& mapping,double time) {
                const auto upper=std::upper_bound(mapping.begin(),mapping.end(),time,
                    [](double t,const auto& p){return t<p.first;});
                if(upper==mapping.begin())return mapping.front().second;
                if(upper==mapping.end())return mapping.back().second;
                const auto& a=*(upper-1);const auto& b=*upper;
                const double f=(time-a.first)/(b.first-a.first);
                return (1-f)*a.second+f*b.second;
            };
            for(const auto& p:request.plan.trajectory.points)request.ik.preferredJoints.push_back(p.q);
            request.ik.lower.clear();request.ik.upper.clear();
            for(const auto& bound:scene->jointBounds()) {
                request.ik.lower.push_back(bound.continuous?-6.283185307179586:bound.lower);
                request.ik.upper.push_back(bound.continuous?6.283185307179586:bound.upper);
            }
            request.minimumClearance=std::max(0.,request.cdfOptions.targetClearance);
            // Surface aiming tolerance is separate from the APF spatial corridor.
            request.maximumTcpPositionDeviation=0.001;
            request.maximumTcpOrientationDeviation=1e-6;
            request.pointSearchMaxAngleDeg=80.;
            request.gaRadiusEvolutionEnabled=true;
            request.gaEvaluateAllPoints=false;
            request.allowSprayAxisRotation=true;
            request.continuousPathCount=std::max<std::size_t>(3,request.continuousPathCount);
            request.anchorSearchSeconds=0.;
            request.validationJointStep=std::min(.001,request.cdfOptions.validationMaxJointStep);
            request.densifyOptimizationKnots=false;
            request.preferredOutputWaypoints=100;

            request.progress=request.progress?request.progress:request.cdfOptions.progress;
            const auto report=[&](const std::string& text){if(request.progress)request.progress(text);};
            const auto evaluateStage=[&]() {
                const auto began=std::chrono::steady_clock::now();
                auto evaluated=ProjectExternalTrajectoryEvaluator::evaluate(document,basePath,request);
                report(nlohmann::json{{"type","pipeline_stage_timing"},{"stage",request.pipelineStage},
                    {"seconds",std::chrono::duration<double>(std::chrono::steady_clock::now()-began).count()},
                    {"points",evaluated.plan.trajectory.points.size()},{"success",evaluated.success}}.dump());
                return evaluated;
            };
            const auto append=[&](const ExternalTrajectoryEvaluationResult& evaluated) {
                result.diagnostics.insert(result.diagnostics.end(),evaluated.diagnostics.begin(),evaluated.diagnostics.end());
            };
            const auto stage=[&](const std::string& name,const StoredMotionPlan& plan,bool accepted) {
                CdfTrajectoryStage s;s.name=name;s.plan=plan;
                s.invalidSegments=accepted?0:-1;s.tcpCorridorValid=accepted;
                s.quality=evaluateTrajectoryQuality(s.plan.trajectory,scene->jointBounds(),
                    request.cdfOptions.fallbackMaxVelocity,request.cdfOptions.fallbackMaxAcceleration,{},fk);
                s.referenceTcpPositions=s.quality.tcpPositions;
                result.stages.push_back(std::move(s));
            };
            const auto retime=[&](StoredMotionPlan& plan,std::vector<std::pair<double,double>>& mapping) {
                if(!request.cdfOptions.retimeOutput)return;
                const auto original=plan.trajectory;
                if(!retimeJointTrajectory(plan.trajectory,scene->jointBounds(),
                    request.cdfOptions.fallbackMaxVelocity,request.cdfOptions.fallbackMaxAcceleration))
                    throw std::runtime_error("Cannot retime the exact joint path.");
                std::vector<std::pair<double,double>> updated;
                for(std::size_t i=0;i<original.points.size();++i)
                    updated.emplace_back(plan.trajectory.points[i].time,sourceTimeAt(mapping,original.points[i].time));
                mapping=std::move(updated);
                // Optimizers may add joint knots without adding Cartesian controls.
                // Project every reference time through the same monotone time map.
                std::vector<std::pair<double,double>> retimedTimes;
                for(std::size_t i=0;i<original.points.size();++i)
                    retimedTimes.emplace_back(original.points[i].time,plan.trajectory.points[i].time);
                for(auto& point:plan.cartesianControlPoints.points)
                    point.time=sourceTimeAt(retimedTimes,point.time);
            };
            // Every stage reuses immutable original surface points and normal references.
            // Only their clocks change. Never recompute an anchor from a GA-adjusted TCP.
            const auto prepareSurface=[&](const std::vector<std::pair<double,double>>& mapping) {
                request.plan.cartesianControlPoints=originalControls;
                std::vector<std::pair<double,double>> inverse;
                for(const auto& pair:mapping)inverse.emplace_back(pair.second,pair.first);
                for(auto& point:request.plan.cartesianControlPoints.points)
                    point.time=sourceTimeAt(inverse,point.time);
                request.sprayTargets=anchors;
                request.sprayDistances=distancesLocked?selectedDistances:std::vector<double>(anchors.size(),surface.sprayDistance);
                request.gaNominalSprayDistances.assign(anchors.size(),surface.sprayDistance);
                request.pointSearchReferenceDelta.clear();
                request.pointSearchMinDistances.clear();request.pointSearchMaxDistances.clear();
                for(std::size_t i=0;i<radiusFractions.size();++i) {
                    request.pointSearchMinDistances.push_back(distancesLocked?selectedDistances[i]:surface.sprayDistance*(1-radiusFractions[i]));
                    request.pointSearchMaxDistances.push_back(distancesLocked?selectedDistances[i]:surface.sprayDistance*(1+radiusFractions[i]));
                }
            };
            const auto lockSelectedDistances=[&](const ExternalTrajectoryEvaluationResult& ga) {
                if(ga.plan.trajectory.points.size()!=anchors.size())
                    throw std::runtime_error("GA selected controls do not match the immutable source anchors.");
                for(std::size_t i=0;i<anchors.size();++i) {
                    const auto actual=fk(ga.plan.trajectory.points[i].q);
                    const Eigen::Vector3d normal=actual.linear()*request.sprayAxis;
                    const double radius=std::clamp((anchors[i]-actual.translation()).dot(normal),
                        surface.sprayDistance*(1-radiusFractions[i]),surface.sprayDistance*(1+radiusFractions[i]));
                    if(!std::isfinite(radius) || radius<=0 ||
                        (actual.translation()+radius*normal-anchors[i]).norm()>request.maximumTcpPositionDeviation)
                        throw std::runtime_error("GA selected distance violates the original surface anchor.");
                    selectedDistances[i]=radius;
                }
                distancesLocked=true;request.gaRadiusEvolutionEnabled=false;
                result.gaSelectedSprayDistances=selectedDistances;
                report(nlohmann::json{{"type","ga_selected_spray_distances"},{"distances_m",selectedDistances},
                    {"scope","fixed per original control for APF/CDF and all independent replays"}}.dump());
            };
            if(request.plan.trajectory.points.size()==request.plan.cartesianControlPoints.points.size())retime(request.plan,sourceTimes);
            prepareSurface(sourceTimes);
            request.pipelineStage="ga";request.controlPointsOnly=true;request.independentPointsOnly=false;
            request.optimizeAvoidance=false;request.optimizationSeed={};
            report("GA: surface-anchor search, half-angle=80 deg, initial="+std::to_string(request.gaInitialPostures)+
                ", generations="+std::to_string(request.gaGenerations)+", population=24, axial spin enabled, preserve safe controls, nominal radius="+
                std::to_string(surface.sprayDistance*1000)+" mm, radius rounds=3:+/-30%,4:+/-50%,5:+/-80%; no single-point time limit");
            auto ga=evaluateStage();append(ga);
            if(ga.gaRadiusFractions.size()==radiusFractions.size())radiusFractions=ga.gaRadiusFractions;
            if(!ga.plan.trajectory.empty() && (ga.success || std::any_of(ga.diagnostics.begin(),ga.diagnostics.end(),
                [](const auto& d){return d.code=="ga_velocity_retime_required";})))lockSelectedDistances(ga);
            if(!ga.success && request.cdfOptions.retimeOutput && !ga.plan.trajectory.empty() &&
                std::any_of(ga.diagnostics.begin(),ga.diagnostics.end(),[](const auto& d){return d.code=="ga_velocity_retime_required";})) {
                for(std::size_t i=0;i<ga.plan.trajectory.points.size();++i)
                    ga.plan.cartesianControlPoints.points[i].tcpPose=fk(ga.plan.trajectory.points[i].q);
                retime(ga.plan,sourceTimes);request.plan=ga.plan;request.optimizationSeed=ga.plan.trajectory;
                prepareSurface(sourceTimes);
                request.gaEvaluateAllPoints=false; // Retiming replays the exact completed GA field.
                report("GA: exact selected controls retimed under actual model limits; fresh control-point validation");
                ga=evaluateStage();append(ga);
            }
            if(!ga.success)return fail("pipeline_ga_failed","GA did not accept all control points; APF was not started.");
            stage("GA safe control points (segment interiors unchecked)",ga.plan,false);
            retime(ga.plan,sourceTimes);
            request.plan=ga.plan;request.optimizationSeed=ga.plan.trajectory;
            request.pipelineStage="apf";request.controlPointsOnly=false;request.optimizeAvoidance=true;
            prepareSurface(sourceTimes);
            report("APF: connect GA controls and validate sampled segment interiors");
            auto apf=evaluateStage();append(apf);
            if(!apf.success)return fail("pipeline_apf_failed","APF did not produce an accepted connected path.");
            retime(apf.plan,sourceTimes);
            // Retiming never changes geometry; replay the exact output under all original gates.
            request.pipelineStage="validate";request.optimizeAvoidance=false;
            request.plan=apf.plan;request.optimizationSeed={};request.verificationTrajectory=apf.plan.trajectory;
            prepareSurface(sourceTimes);
            auto verified=evaluateStage();append(verified);
            if(!verified.success)return fail("pipeline_apf_replay_failed","Independent APF replay failed.");
            result.referenceSeedTrajectory=ga.plan.trajectory;
            result.plan=verified.plan;stage("APF accepted (sampled collision validation)",result.plan,true);
            if(useCdf) {
                report("CDF: optional optimization of accepted APF path");
                request.pipelineStage="cdf";request.optimizeAvoidance=true;
                request.plan=result.plan;request.optimizationSeed=result.plan.trajectory;request.verificationTrajectory={};
                prepareSurface(sourceTimes);
                auto cdf=evaluateStage();append(cdf);
                if(cdf.success) {
                    auto cdfTimes=sourceTimes;
                    retime(cdf.plan,cdfTimes);request.pipelineStage="validate";request.optimizeAvoidance=false;
                    request.plan=cdf.plan;request.optimizationSeed={};request.verificationTrajectory=cdf.plan.trajectory;
                    prepareSurface(cdfTimes);
                    auto checked=evaluateStage();append(checked);
                    if(checked.success) {result.plan=checked.plan;sourceTimes=std::move(cdfTimes);stage("CDF accepted",result.plan,true);}
                    else result.diagnostics.push_back({"cdf_fallback_apf","CDF replay rejected; accepted APF retained."});
                } else result.diagnostics.push_back({"cdf_fallback_apf","Optional CDF rejected; accepted APF retained."});
            }
            if(request.cancelled && request.cancelled())return fail("pipeline_cancelled","Cancelled; no trajectory published.");
            result.plan.cartesianControlPoints.points.clear();
            for(const auto& joint:result.plan.trajectory.points) {
                robottrajectory::TimedCartesianPoint point;point.time=joint.time;point.tcpPose=fk(joint.q);
                result.plan.cartesianControlPoints.points.push_back(point);
            }
            result.plan.id=request.plan.robotId+"_cdf_qp_ga_apf";
            result.plan.name=useCdf?"GA / APF / optional CDF":"GA / APF";
            result.plan.trajectory.name=result.plan.id;
            for(const auto& p:result.plan.trajectory.points)result.sourceReferenceTimes.push_back(sourceTimeAt(sourceTimes,p.time));
            result.statistics.inputWaypointCount=static_cast<int>(ga.plan.trajectory.points.size());
            result.success=true;
            result.diagnostics.push_back({"pipeline_sampled_valid","GA -> APF -> optional CDF completed; sampled collision checks passed, not a conservative whole-motion certificate."});
            return result;
        } catch(const std::exception& e) {return fail("pipeline_exception",e.what());}
    }
}
