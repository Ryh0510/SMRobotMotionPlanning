#include <ProjectMotionPlanning/GaApfCdfPipeline.h>
#include <ProjectMotionPlanning/LayeredIkGraph.h>
#include <RobotInstance/RobotInstance.h>
#include <nlohmann/json.hpp>
#include <algorithm>
#include <stdexcept>
#include <chrono>
#include <map>
#include <set>

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
        const simulation_project::ProjectDocument& inputDocument,
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
            auto document=inputDocument;
            if(request.workpiece) {
                auto found=std::find_if(document.objects.begin(),document.objects.end(),[&](const auto& o){return o.id==request.workpiece->id;});
                if(found==document.objects.end())document.objects.push_back(*request.workpiece);else *found=*request.workpiece;
            }
            if(request.cancelled && request.cancelled())return fail("pipeline_cancelled","Cancelled before scene construction.");
            if(surface.maxConnectionFeedbackRounds<0 || surface.maxConnectionFeedbackRounds>64)
                return fail("invalid_feedback_rounds","Connection feedback rounds must be 0..64.");
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
            request.progress=request.progress?request.progress:request.cdfOptions.progress;
            if(request.plan.trajectory.empty() && request.startJoints.empty()) {
                const auto began=std::chrono::steady_clock::now();
                if(request.progress)request.progress("Initialization: enumerate original physical IK, then layered Top-1");
                auto* runtime=scene->simulationRuntime().robot(request.plan.robotId);
                if(!runtime || !runtime->instance || request.plan.jointNames.size()!=6)
                    return fail("pipeline_initial_ik_model","Initialization requires the actual six-axis model.");
                CartesianMultiIkOptions ik;
                ik.model.robotId=request.plan.robotId;ik.model.jointNames=request.plan.jointNames;
                std::vector<std::size_t> dofs;
                for(const auto& name:request.plan.jointNames) {
                    const auto& joint=runtime->model.joints.at(runtime->model.jointNameToIndex.at(name));
                    if(joint.type!=robot::JointType::Revolute)return fail("pipeline_initial_ik_model","Six revolute joints required.");
                    dofs.push_back(static_cast<std::size_t>(joint.dofIndex));
                }
                for(const auto& robot:document.robots)if(robot.id==request.plan.robotId)
                    for(const auto& name:request.plan.jointNames) {
                        const auto value=std::find_if(robot.initialJoints.begin(),robot.initialJoints.end(),[&](const auto& j){return j.jointName==name;});
                        ik.model.seedJoints.push_back(value==robot.initialJoints.end()?0.:value->value);
                    }
                std::vector<double> lower,upper;
                if(!ProjectTrajectoryInverseKinematics::readRevoluteJointLimits(document,basePath,request.plan.robotId,
                    request.plan.jointNames,lower,upper,error))return fail("pipeline_initial_ik_limits",error);
                for(std::size_t j=0;j<lower.size();++j) {
                    ik.lower.push_back(std::max(-6.283185307179586,lower[j]));
                    ik.upper.push_back(std::min(6.283185307179586,upper[j]));
                }
                ik.model.worldForwardKinematics=[&](const std::vector<double>& q) -> Eigen::Isometry3d {
                    for(std::size_t j=0;j<dofs.size();++j)runtime->instance->setJoint(dofs[j],q.at(j));
                    runtime->instance->update();
                    return scene->linkWorldTransform(request.tcpLinkName)*request.linkFromTcp;
                };
                ik.cancelled=request.cancelled;
                ik.progress=[&](std::size_t done,std::size_t total) {
                    if(request.progress)request.progress(nlohmann::json{{"type","pipeline_initial_ik"},{"completed",done},{"total",total}}.dump());
                };
                const auto candidates=ProjectTrajectoryInverseKinematics::solveAllCartesianControlPoints(request.plan,ik);
                if(candidates.cancelled)return fail("pipeline_cancelled","Cancelled during original IK enumeration.");
                auto leading=candidates;
                std::size_t prefix=0;while(prefix<leading.layers.size() && !leading.layers[prefix].candidates.empty())++prefix;
                if(prefix==0)return fail("pipeline_initial_ik_missing","The first original pose has no physical IK for Top-1 initialization.");
                leading.layers.resize(prefix);leading.source.cartesianControlPoints.points.resize(prefix);leading.success=true;
                LayeredIkGraphOptions graphOptions;graphOptions.maxPaths=1;graphOptions.jointWeights.assign(6,1.);
                graphOptions.cancelled=request.cancelled;
                const auto graph=ProjectLayeredIkGraph::filter(leading,graphOptions);
                if(!graph.success || graph.paths.empty())return fail("pipeline_initial_graph_failed",graph.message);
                StoredMotionPlan selected;
                if(!ProjectTrajectoryInverseKinematics::selectMultiIkTrajectory(leading,graph.paths.front().selections,selected,error))
                    return fail("pipeline_initial_graph_failed",error);
                const auto initial=selected.trajectory.points.front().q;
                if(!ProjectTrajectoryInverseKinematics::applyJointValuesToRobotInitialJoints(document,request.plan.robotId,
                    request.plan.jointNames,initial,&error))return fail("pipeline_initial_graph_failed",error);
                if(prefix==candidates.layers.size())request.plan=std::move(selected);
                else {
                    request.ik.model.seedJoints=initial;request.ik.cachedCandidates.clear();
                    request.ik.cachedCandidates.resize(candidates.layers.size());
                    for(std::size_t i=0;i<candidates.layers.size();++i)
                        for(const auto& c:candidates.layers[i].candidates)request.ik.cachedCandidates[i].push_back(c.joints);
                }
                scene=ProjectPlanningSceneBuilder::build(document,basePath,sceneRequest,&error);
                if(!scene)return fail("pipeline_scene_failed",error);
                const auto event=nlohmann::json{{"type","pipeline_initial_top1"},{"source_points",candidates.layers.size()},
                    {"prefix_points",prefix},{"rank",1},{"cost_rad2",graph.paths.front().cost},{"initial_joints_rad",initial},
                    {"scope","kinematic initialization only; GA repairs missing/unsafe poses; initial configuration not hard-locked"},
                    {"seconds",std::chrono::duration<double>(std::chrono::steady_clock::now()-began).count()}}.dump();
                result.diagnostics.push_back({"pipeline_initial_top1",event});if(request.progress)request.progress(event);
            }
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
            const auto retime=[&](StoredMotionPlan& plan,std::vector<std::pair<double,double>>& mapping,bool preserveSurfaceClock=false) {
                if(!request.cdfOptions.retimeOutput)return;
                const auto original=plan.trajectory;
                if(!retimeJointTrajectory(plan.trajectory,scene->jointBounds(),
                    request.cdfOptions.fallbackMaxVelocity,request.cdfOptions.fallbackMaxAcceleration))
                    throw std::runtime_error("Cannot retime the exact joint path.");
                if(preserveSurfaceClock) {
                    // A local change of interval duration also changes the surface
                    // station at added APF knots. Preserve the original affine clock
                    // with one dilation; geometry and every station stay identical.
                    auto measured=original;
                    const auto quality=evaluateTrajectoryQuality(measured,scene->jointBounds(),
                        request.cdfOptions.fallbackMaxVelocity,request.cdfOptions.fallbackMaxAcceleration);
                    if(!quality.valid || !quality.timingValid)throw std::runtime_error("Invalid APF clock.");
                    double scale=1.;const auto& bounds=scene->jointBounds();
                    for(std::size_t j=0;j<bounds.size();++j) {
                        const double velocity=std::isfinite(bounds[j].maxVelocity) && bounds[j].maxVelocity>0?
                            bounds[j].maxVelocity:request.cdfOptions.fallbackMaxVelocity;
                        const double acceleration=std::isfinite(bounds[j].maxAcceleration) && bounds[j].maxAcceleration>0?
                            bounds[j].maxAcceleration:request.cdfOptions.fallbackMaxAcceleration;
                        scale=std::max(scale,quality.jointPeakVelocities[j]/velocity);
                        scale=std::max(scale,std::sqrt(quality.jointPeakAccelerations[j]/acceleration));
                    }
                    const double begin=original.points.front().time;
                    for(std::size_t i=0;i<original.points.size();++i)
                        plan.trajectory.points[i].time=begin+(original.points[i].time-begin)*scale*1.001;
                }
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
            ExternalTrajectoryEvaluationResult apf;
            bool pendingRetime=false;
            std::vector<GaRejectedConnection> rejectedConnections;
            std::map<std::size_t,int> feedbackVisits;
            int feedbackSearchSequence=0;
            for(int feedbackRound=0;;++feedbackRound) {
                apf=evaluateStage();append(apf);
                pendingRetime=!apf.success && request.cdfOptions.retimeOutput && !apf.retimingCandidate.empty() &&
                    std::any_of(apf.diagnostics.begin(),apf.diagnostics.end(),[](const auto& d){return d.code=="apf_velocity_retime_required";});
                if(apf.success || pendingRetime)break;
                if(apf.cancelled || (request.cancelled && request.cancelled()))
                    return fail("pipeline_cancelled","Cancelled during APF/GA connection feedback.");
                if(feedbackRound>=surface.maxConnectionFeedbackRounds || apf.apfConnectionFailures.empty()) {
                    result.apfConnectionFailures=apf.apfConnectionFailures;
                    return fail("pipeline_apf_failed","APF did not produce an accepted connected path after available connection feedback.");
                }
                std::set<std::size_t> failedEdges,controls;
                const auto& knots=ga.plan.trajectory.points;
                for(const auto& interval:apf.apfConnectionFailures)for(std::size_t i=0;i+1<knots.size();++i)
                    if(knots[i].time<interval.endTime-1e-10 && knots[i+1].time>interval.beginTime+1e-10)
                        failedEdges.insert(i);
                if(failedEdges.empty())return fail("pipeline_feedback_mapping_failed","APF failure does not map to an original control interval.");
                int visit=0;
                for(const auto edge:failedEdges) {
                    visit=std::max(visit,++feedbackVisits[edge]);
                    rejectedConnections.push_back({edge,knots[edge].q,knots[edge+1].q});
                }
                const std::size_t padding=std::min<std::size_t>(32,2*std::size_t(visit));
                for(const auto edge:failedEdges) {
                    const auto first=edge>padding?edge-padding:0;
                    const auto last=std::min(knots.size()-1,edge+1+padding);
                    for(std::size_t i=first;i<=last;++i)
                        if(i!=0 || request.startJoints.empty())controls.insert(i);
                }
                const auto feedbackEvent=nlohmann::json{{"round",feedbackRound+1},{"source_edges_zero_based",failedEdges},
                    {"resample_controls_zero_based",controls},{"seed","current point-safe GA controls"},
                    {"anchors","immutable original surface points"},{"rejected_connections",rejectedConnections.size()}}.dump();
                result.diagnostics.push_back({"pipeline_apf_ga_feedback",feedbackEvent});
                report("APF -> GA: "+feedbackEvent);
                // Re-enter GA with the accepted point field, not the incomplete APF path.
                // Existing radius authorizations survive; only a newly accepted GA may
                // select new distances, which are locked again before the next APF.
                distancesLocked=false;request.gaRadiusEvolutionEnabled=true;
                request.plan=ga.plan;request.optimizationSeed=ga.plan.trajectory;
                request.verificationTrajectory={};request.pipelineStage="ga";
                request.controlPointsOnly=true;request.optimizeAvoidance=false;
                request.gaResampleControls.assign(controls.begin(),controls.end());
                request.gaRejectedConnections=rejectedConnections;
                request.pointSearchSequence=++feedbackSearchSequence;
                prepareSurface(sourceTimes);
                auto replacement=evaluateStage();append(replacement);
                for(int localRetry=1;!replacement.success && !replacement.cancelled && localRetry<3;++localRetry) {
                    const bool disconnected=std::any_of(replacement.diagnostics.begin(),replacement.diagnostics.end(),[](const auto& d){
                        return d.code=="no_valid_candidate_path" || d.code=="ga_point_unresolved" || d.code=="ga_connection_unresolved";});
                    const bool timing=std::any_of(replacement.diagnostics.begin(),replacement.diagnostics.end(),[](const auto& d){return d.code=="ga_velocity_retime_required";});
                    if(!disconnected || timing || (request.cancelled && request.cancelled()))break;
                    if(replacement.gaRadiusFractions.size()==radiusFractions.size())for(std::size_t i=0;i<radiusFractions.size();++i)
                        radiusFractions[i]=std::max(radiusFractions[i],replacement.gaRadiusFractions[i]);
                    const std::size_t expandedPadding=std::min<std::size_t>(32,padding+2*localRetry);
                    for(const auto edge:failedEdges) {
                        const auto first=edge>expandedPadding?edge-expandedPadding:0;
                        const auto last=std::min(knots.size()-1,edge+1+expandedPadding);
                        for(std::size_t i=first;i<=last;++i)if(i!=0 || request.startJoints.empty())controls.insert(i);
                    }
                    request.gaResampleControls.assign(controls.begin(),controls.end());
                    request.pointSearchSequence=++feedbackSearchSequence;prepareSurface(sourceTimes);
                    const auto retryEvent=nlohmann::json{{"round",feedbackRound+1},{"local_retry",localRetry},
                        {"resample_controls_zero_based",controls},{"search_sequence",request.pointSearchSequence}}.dump();
                    result.diagnostics.push_back({"pipeline_feedback_ga_window_expanded",retryEvent});
                    report("GA feedback: expand disconnected local window: "+retryEvent);
                    replacement=evaluateStage();append(replacement);
                }
                if(replacement.cancelled || (request.cancelled && request.cancelled()))
                    return fail("pipeline_cancelled","Cancelled during GA feedback search.");
                const bool needsTiming=!replacement.success && !replacement.plan.trajectory.empty() &&
                    std::any_of(replacement.diagnostics.begin(),replacement.diagnostics.end(),[](const auto& d){return d.code=="ga_velocity_retime_required";});
                request.gaResampleControls.clear();request.gaRejectedConnections.clear();
                if(!replacement.success && !needsTiming)
                    return fail("pipeline_feedback_ga_failed","Local GA feedback did not return a new connected point-safe field.");
                if(replacement.gaRadiusFractions.size()==radiusFractions.size())radiusFractions=replacement.gaRadiusFractions;
                lockSelectedDistances(replacement);
                if(needsTiming) {
                    if(!request.cdfOptions.retimeOutput)return fail("pipeline_feedback_timing_required","Feedback controls require exact retiming.");
                    auto retimedTimes=sourceTimes;
                    retime(replacement.plan,retimedTimes);request.plan=replacement.plan;
                    request.optimizationSeed=replacement.plan.trajectory;prepareSurface(retimedTimes);
                    replacement=evaluateStage();append(replacement);
                    if(!replacement.success)return fail("pipeline_feedback_ga_replay_failed","Retimed feedback controls did not pass fresh validation.");
                    sourceTimes=std::move(retimedTimes);
                }
                ga=std::move(replacement);stage("GA feedback safe controls (segment interiors unchecked)",ga.plan,false);
                retime(ga.plan,sourceTimes);
                request.plan=ga.plan;request.optimizationSeed=ga.plan.trajectory;
                request.pipelineStage="apf";request.controlPointsOnly=false;request.optimizeAvoidance=true;
                prepareSurface(sourceTimes);
                report("APF: retry connection from new GA controls with locked selected spray distances");
            }
            if(pendingRetime) {
                apf.plan=request.plan;apf.plan.trajectory=std::move(apf.retimingCandidate);
                report("APF: geometry/process checked; retime the exact pending path, then independently replay all gates");
            }
            retime(apf.plan,sourceTimes,pendingRetime);
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
            nlohmann::json reference={{"type","ga_apf_surface_reference"},{"version",1},{"robot_id",result.plan.robotId},
                {"joint_names",result.plan.jointNames},{"nominal_distance_m",surface.sprayDistance},
                {"clearance_m",request.minimumClearance},{"tcp_link",request.tcpLinkName},
                {"source_controls",nlohmann::json::array()},{"output_times_s",nlohmann::json::array()},
                {"source_times_s",result.sourceReferenceTimes},{"selected_distances_m",selectedDistances}};
            const auto matrix=[](const Eigen::Isometry3d& pose) {
                std::vector<double> values;for(int row=0;row<4;++row)for(int column=0;column<4;++column)values.push_back(pose.matrix()(row,column));return values;
            };
            reference["link_from_tcp"]=matrix(request.linkFromTcp);
            for(const auto& point:originalControls.points)reference["source_controls"].push_back({{"time_s",point.time},{"matrix",matrix(point.tcpPose)}});
            for(const auto& point:result.plan.trajectory.points)reference["output_times_s"].push_back(point.time);
            result.plan.optimizationMetadata=reference.dump();
            result.success=true;
            result.diagnostics.push_back({"pipeline_sampled_valid","GA -> APF -> optional CDF completed; sampled collision checks passed, not a conservative whole-motion certificate."});
            return result;
        } catch(const std::exception& e) {return fail("pipeline_exception",e.what());}
    }
}
