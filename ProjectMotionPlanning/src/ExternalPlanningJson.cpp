#include <ProjectMotionPlanning/ExternalPlanningJson.h>
#include <ProjectMotionPlanning/ExternalWorkpiecePlanning.h>
#include <nlohmann/json.hpp>
#include <algorithm>
#include <stdexcept>
#include <cmath>
#include <limits>
using Json=nlohmann::json;
using namespace motion_planning;
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
    Eigen::Isometry3d userFrame(const simulation_project::ProjectDocument& doc)
    {
        Eigen::Isometry3d t=Eigen::Isometry3d::Identity();
        for(const auto& ext:doc.extensions) if(ext.key=="rws.user_frame") {
            if(ext.version!=1) throw std::runtime_error("Unsupported user frame version");
            const auto p=Json::parse(ext.serializedPayload);
            t.translation()=Eigen::Vector3d(p.at("x").get<double>(),p.at("y").get<double>(),p.at("z").get<double>());
            t.linear()=(Eigen::AngleAxisd(p.at("yaw").get<double>(),Eigen::Vector3d::UnitZ())*
                Eigen::AngleAxisd(p.at("pitch").get<double>(),Eigen::Vector3d::UnitY())*
                Eigen::AngleAxisd(p.at("roll").get<double>(),Eigen::Vector3d::UnitX())).toRotationMatrix();
            if(!t.matrix().allFinite()) throw std::runtime_error("Invalid user frame");
        }
        return t;
    }
    Eigen::Isometry3d pose(const Json& value)
    {
        const auto p=value.at("position_m").get<std::vector<double>>();
        const auto q=value.at("quaternion_xyzw").get<std::vector<double>>();
        if(p.size()!=3 || q.size()!=4) throw std::runtime_error("Pose requires position_m[3], quaternion_xyzw[4].");
        Eigen::Quaterniond rotation(q[3],q[0],q[1],q[2]);
        if(!rotation.coeffs().allFinite() || std::abs(rotation.norm()-1)>1e-6) throw std::runtime_error("Quaternion must be normalized.");
        Eigen::Isometry3d result=Eigen::Isometry3d::Identity();
        result.translation()=Eigen::Vector3d(p[0],p[1],p[2]); result.linear()=rotation.normalized().toRotationMatrix();
        if(!result.matrix().allFinite()) throw std::runtime_error("Non-finite pose.");
        return result;
    }
}

std::string motion_planning::runExternalPlanningJson(
    const simulation_project::ProjectDocument& sceneDocument,const std::filesystem::path& projectBasePath,
    const std::string& requestJson,const std::filesystem::path& requestBasePath,
    std::function<bool()> cancelled,std::function<void(const std::string&)> progress)
{
    Json report;
    try {
        if(cancelled && cancelled()) return Json{{"success",false},{"cancelled",true}}.dump();
        const Json json=Json::parse(requestJson);
        simulation_runtime::ProjectSimulationRuntime runtime;
        const auto loaded=runtime.loadProject(sceneDocument,projectBasePath);
        if(!loaded.success) throw std::runtime_error(loaded.message);
        if(json.at("version")!=1) throw std::runtime_error("Unsupported request version.");
        ExternalTrajectoryEvaluationRequest request;
        request.plan.robotId=json.at("robot_id").get<std::string>();
        request.plan.jointNames=json.at("joint_names").get<std::vector<std::string>>();
        request.tcpLinkName=json.value("tcp_link",std::string());
        std::string selectedTool=json.value("tool_attachment_id",std::string());
        if(!selectedTool.empty() && (json.contains("tcp_link") || json.contains("link_from_tcp")))
            throw std::runtime_error("Select saved tool TCP OR explicit tcp_link/link_from_tcp, not both.");
        if(request.tcpLinkName.empty()) {
            std::vector<const simulation_project::MountedAttachmentDesc*> tools;
            for(const auto& a:sceneDocument.mountedAttachments) {
                if(!a.enabled || (!selectedTool.empty() && a.id!=selectedTool)) continue;
                const auto mount=std::find_if(sceneDocument.robotMounts.begin(),sceneDocument.robotMounts.end(),
                    [&](const auto& m){return m.id==a.mountFrameId && m.robotId==request.plan.robotId;});
                const auto asset=std::find_if(sceneDocument.attachmentAssets.begin(),sceneDocument.attachmentAssets.end(),
                    [&](const auto& v){return v.id==a.assetId;});
                if(mount!=sceneDocument.robotMounts.end() && asset!=sceneDocument.attachmentAssets.end() &&
                    (asset->assetKind.empty()?asset->assetType:asset->assetKind)=="tool") tools.push_back(&a);
            }
            if(tools.size()!=1) throw std::runtime_error("Select exactly one enabled tool_attachment_id for this robot.");
            const auto& a=*tools.front(); selectedTool=a.id;
            const auto& mount=*std::find_if(sceneDocument.robotMounts.begin(),sceneDocument.robotMounts.end(),[&](const auto& m){return m.id==a.mountFrameId;});
            const auto& asset=*std::find_if(sceneDocument.attachmentAssets.begin(),sceneDocument.attachmentAssets.end(),[&](const auto& v){return v.id==a.assetId;});
            const simulation_project::AttachmentFunctionalFrameDesc* tcp=nullptr;
            int count=0,primary=0;
            for(const auto& f:asset.functionalFrames) if(f.frameType=="tcp") {
                ++count; if(!tcp || f.primary) tcp=&f; if(f.primary) ++primary;
            }
            if(!tcp || primary>1 || (count>1 && primary!=1)) throw std::runtime_error("Tool requires one unambiguous saved TCP; flange fallback is disabled.");
            request.tcpLinkName=mount.linkName;
            request.linkFromTcp=transform(mount.linkToMount)*transform(a.mountToAssetMount)*transform(tcp->assetMountToFrame);
        }
        request.collisionDetectorIds=json.at("detector_ids").get<std::vector<std::string>>();
        if(json.contains("seed_joints_rad") && !json.value("use_snapshot_seed",false)) {
            request.ik.model.seedJoints=json.at("seed_joints_rad").get<std::vector<double>>();
        } else {
            const auto* robot=runtime.robot(request.plan.robotId);
            if(!robot || !robot->instance) throw std::runtime_error("Unknown planning robot.");
            const auto& q=robot->instance->getState().q;
            for(const auto& name:request.plan.jointNames) {
                const auto found=robot->model.jointNameToIndex.find(name);
                if(found==robot->model.jointNameToIndex.end()) throw std::runtime_error("Unknown joint: "+name);
                const int index=robot->model.joints[found->second].dofIndex;
                if(index<0 || static_cast<std::size_t>(index)>=q.size()) throw std::runtime_error("Joint is not movable: "+name);
                request.ik.model.seedJoints.push_back(q[index]);
            }
        }
        if(json.contains("link_from_tcp")) request.linkFromTcp=pose(json.at("link_from_tcp"));
        const auto frame=json.value("frame",std::string("user"));
        if(frame!="world" && frame!="workpiece" && frame!="user") throw std::runtime_error("frame must be world, user or workpiece.");
        const Eigen::Isometry3d worldFromUser=userFrame(sceneDocument);
        request.frame=frame=="workpiece"?ExternalTrajectoryFrame::Workpiece:ExternalTrajectoryFrame::World;
        request.workpieceId=json.value("workpiece_id",std::string());
        if(json.contains("workpiece")) {
            const auto& part=json.at("workpiece");
            simulation_project::SceneObjectDesc object;
            object.id=part.at("id").get<std::string>(); object.name=object.id;
            auto path=std::filesystem::u8path(part.at("stl_path").get<std::string>());
            if(path.is_relative()) path=requestBasePath/path;
            object.sourcePath=path.generic_u8string();
            object.visualScale=object.collisionScale=part.at("mesh_scale_to_m").get<double>();
            if(part.contains("world_pose") && part.contains("user_pose")) throw std::runtime_error("Supply world_pose OR user_pose");
            const Eigen::Isometry3d world=part.contains("world_pose")?pose(part.at("world_pose")):
                (part.contains("user_pose")?worldFromUser*pose(part.at("user_pose")):worldFromUser);
            object.transform.x=world.translation().x(); object.transform.y=world.translation().y(); object.transform.z=world.translation().z();
            const auto angles=world.linear().eulerAngles(2,1,0);
            object.transform.roll=angles[2]; object.transform.pitch=angles[1]; object.transform.yaw=angles[0];
            request.workpiece=object;
        }
        for(const auto& point:json.at("points")) {
            robottrajectory::TimedCartesianPoint p; p.time=point.at("time_s").get<double>(); p.tcpPose=pose(point);
            if(frame=="user") p.tcpPose=worldFromUser*p.tcpPose;
            const auto typeValue=point.value("point_type",Json(0));
            if(!typeValue.is_number_integer())throw std::runtime_error("Point type must be an integer");
            request.pointTypes.push_back(typeValue.get<int>());
            Eigen::Vector3d center=p.tcpPose.translation();
            if(point.contains("transition_center_m")) {
                const auto c=point.at("transition_center_m").get<std::vector<double>>();
                if(c.size()!=3)throw std::runtime_error("Invalid transition center");
                center=Eigen::Vector3d(c[0],c[1],c[2]);
                if(frame=="user")center=worldFromUser*center;
            }
            request.transitionCenters.push_back(center);
            if(point.contains("spray")) {
                const auto& spec=point.at("spray");
                const auto target=spec.at("target_mm").get<std::vector<double>>();
                const auto axis=spec.at("axis_tool").get<std::vector<double>>();
                if(target.size()!=3 || axis.size()!=3)throw std::runtime_error("Invalid spray target/axis");
                Eigen::Vector3d t(target[0]/1000,target[1]/1000,target[2]/1000);
                if(frame=="user")t=worldFromUser*t;
                Eigen::Vector3d a(axis[0],axis[1],axis[2]);
                if(!request.sprayTargets.empty() && !a.isApprox(request.sprayAxis,1e-10))throw std::runtime_error("Mixed spray axes");
                request.sprayAxis=a;request.sprayTargets.push_back(t);
                request.sprayDistances.push_back(spec.at("distance_mm").get<double>()/1000);
            }
            request.plan.cartesianControlPoints.points.push_back(p);
        }
        if(json.contains("search_lower_rad")) request.ik.lower=json.at("search_lower_rad").get<std::vector<double>>();
        if(json.contains("search_upper_rad")) request.ik.upper=json.at("search_upper_rad").get<std::vector<double>>();
        request.ik.maxCandidatesPerPoint=json.value("ik_max_candidates_per_point",std::size_t(64));
        request.ik.seedCount=json.value("ik_seed_count",request.ik.seedCount);
        request.ik.maxIterations=json.value("ik_max_iterations",request.ik.maxIterations);
        request.maxJointDelta=json.value("max_joint_delta_rad",request.maxJointDelta);
        request.validationJointStep=json.value("validation_joint_step_rad",request.validationJointStep);
        request.minimumClearance=json.value("minimum_clearance_m",request.minimumClearance);
        request.minimumSingularValue=json.value("minimum_singular_value",request.minimumSingularValue);
        request.jacobianLengthScale=json.value("jacobian_length_scale_m",request.jacobianLengthScale);
        request.maximumTcpPositionDeviation=json.value("max_tcp_position_deviation_m",request.maximumTcpPositionDeviation);
        request.maximumTcpOrientationDeviation=json.value("max_tcp_orientation_deviation_rad",request.maximumTcpOrientationDeviation);
        if(json.contains("max_joint_velocities_rad_s")) request.maxJointVelocities=json.at("max_joint_velocities_rad_s").get<std::vector<double>>();
        // Optional fresh detector for externally supplied workpiece, explicit robot links.
        auto document=sceneDocument;
        if(json.contains("workpiece_collision_links")) {
            if(!request.workpiece) throw std::runtime_error("Fresh workpiece detector requires workpiece input.");
            const auto links=json.at("workpiece_collision_links").get<std::vector<std::string>>();
            if(links.empty()) throw std::runtime_error("workpiece_collision_links cannot be empty.");
            const auto id=json.at("workpiece_detector_id").get<std::string>();
            if(id.empty()) throw std::runtime_error("workpiece_detector_id cannot be empty.");
            for(const auto& d:document.collision.detectors) if(d.id==id) throw std::runtime_error("Workpiece detector id already exists.");
            simulation_project::CollisionSelectionSetDesc a,b; a.id=id+"_A"; b.id=id+"_B";
            for(const auto& set:document.collision.selectionSets) if(set.id==a.id || set.id==b.id) throw std::runtime_error("Selection set id already exists.");
            const auto* robot=runtime.robot(request.plan.robotId);
            if(!robot) throw std::runtime_error("Unknown robot.");
            for(const auto& link:links) {
                if(robot->model.linkNameToIndex.count(link)==0) throw std::runtime_error("Unknown collision link: "+link);
                simulation_project::CollisionSelectionSetMemberDesc m; m.robotId=request.plan.robotId; m.linkName=link; a.members.push_back(m);
            }
            simulation_project::CollisionSelectionSetMemberDesc m; m.objectId=request.workpiece->id; b.members.push_back(m);
            document.collision.selectionSets.push_back(a); document.collision.selectionSets.push_back(b);
            simulation_project::CollisionDetectorDesc d; d.id=id; d.type="SelectedObjects"; d.queryPolicy="BetweenSets";
            d.selectionSetAId=a.id; d.selectionSetBId=b.id; document.collision.detectors.push_back(d);
            if(std::find(request.collisionDetectorIds.begin(),request.collisionDetectorIds.end(),id)==request.collisionDetectorIds.end()) request.collisionDetectorIds.push_back(id);
        }

        request.cancelled=cancelled; request.progress=progress;
        if(json.contains("joint_weights")) request.jointWeights=json.at("joint_weights").get<std::vector<double>>();
        if(json.contains("start_joints_rad")) request.startJoints=json.at("start_joints_rad").get<std::vector<double>>();
        request.controlPointsOnly=json.value("control_points_only",false);
        request.independentPointsOnly=json.value("independent_points_only",false);
        if(request.independentPointsOnly && (!request.controlPointsOnly || json.contains("validate_joint_points")))
            throw std::runtime_error("Independent point batches cannot interpolate or replay a complete path.");
        request.returnCandidates=json.value("return_candidates",true);
        if(json.contains("cached_ik_candidates"))
            request.ik.cachedCandidates=json.at("cached_ik_candidates").get<std::vector<std::vector<std::vector<double>>>>();
        if(json.contains("preferred_ik_joints"))
            request.ik.preferredJoints=json.at("preferred_ik_joints").get<std::vector<std::vector<double>>>();
        request.transitionSearchRadius=json.value("transition_search_radius_mm",300.0)/1000;
        request.transitionSearchMaxAngleDeg=json.value("transition_search_max_angle_deg",180.0);
        request.pointSearchAttempts=json.value("point_search_attempts",0);
        if(json.contains("point_search_attempt_limits"))request.pointSearchAttemptLimits=json.at("point_search_attempt_limits").get<std::vector<int>>();
        request.pointSearchSequence=json.value("point_search_sequence",0);
        request.pointSearchPopulationStrategy=json.value("point_search_population_strategy",1);
        request.pointSearchStartStrategy=json.value("point_search_start_strategy",1);
        request.pointSearchSeconds=json.value("point_search_seconds",3.0);
        request.pointSearchMaxAngleDeg=json.value("point_search_max_angle_deg",30.0);
        request.allowSprayAxisRotation=json.value("allow_spray_axis_rotation",false);
        if(json.contains("point_search_min_distances_m"))request.pointSearchMinDistances=json.at("point_search_min_distances_m").get<std::vector<double>>();
        if(json.contains("point_search_max_distances_m"))request.pointSearchMaxDistances=json.at("point_search_max_distances_m").get<std::vector<double>>();
        if(json.contains("point_search_reference_delta"))for(const auto& q:json.at("point_search_reference_delta")) {
            if(q.size()!=4)throw std::runtime_error("Invalid point search reference quaternion");
            request.pointSearchReferenceDelta.emplace_back(q[3].get<double>(),q[0].get<double>(),q[1].get<double>(),q[2].get<double>());
        }
        request.continuousPathCount=json.value("continuous_path_count",std::size_t(3));
        request.optimizeAvoidance=json.value("optimize_avoidance",false);
        request.pipelineStage=json.value("pipeline_stage",std::string());
        request.includeKinematicAudit=json.value("include_kinematic_audit",false);
        request.stageTimeBudgetSeconds=json.value("stage_time_budget_s",0.0);
        if(json.contains("anchor_search_attempts") && !json.at("anchor_search_attempts").is_number_integer())
            throw std::runtime_error("anchor_search_attempts must be integer");
        request.anchorSearchAttempts=json.value("anchor_search_attempts",104);
        request.anchorSearchSeconds=json.value("anchor_search_seconds",0.0);
        request.gaRadiusEvolutionEnabled=json.value("ga_radius_evolution_enabled",false);
        if(json.contains("ga_nominal_distances_m"))request.gaNominalSprayDistances=json.at("ga_nominal_distances_m").get<std::vector<double>>();
        request.gaGpuEnabled=json.value("ga_gpu_enabled",true);
        request.gaGpuIkEnabled=json.value("ga_gpu_ik_enabled",true);
        if(!request.pipelineStage.empty()) {
            if(request.pipelineStage!="ik" && request.pipelineStage!="ga" && request.pipelineStage!="posture" && request.pipelineStage!="apf" && request.pipelineStage!="cdf" && request.pipelineStage!="validate")
                throw std::runtime_error("Unknown pipeline_stage");
            if(json.value("cdf_point_optimization",false) || json.value("point_search_attempts",0)!=0)
                throw std::runtime_error("Staged workflow cannot use point CDF/population optimization");
            request.optimizeAvoidance=request.pipelineStage=="apf" || request.pipelineStage=="cdf";
            request.controlPointsOnly=request.pipelineStage=="ik" || request.pipelineStage=="ga" || request.pipelineStage=="posture";
            request.independentPointsOnly=request.pipelineStage=="ik";
            request.returnCandidates=true;
            request.densifyOptimizationKnots=json.value("densify_optimization_knots",false);
        }
        if(json.contains("ga_parameters")) {
            const auto& p=json.at("ga_parameters");
            if(!p.is_object())throw std::runtime_error("ga_parameters must be an object");
            for(auto it=p.begin();it!=p.end();++it)if(it.key()!="feasibility_batch_size" && it.key()!="generations" && it.key()!="initial_postures" && it.key()!="point_evaluations" && it.key()!="field_evaluations" && it.key()!="continuity_weight" && it.key()!="posture_weight")throw std::runtime_error("Unknown GA parameter: "+it.key());
            for(const auto* k:{"feasibility_batch_size","generations","initial_postures","point_evaluations","field_evaluations"})if(p.contains(k) && !p.at(k).is_number_integer())throw std::runtime_error("GA evaluation budget must be integer");
            request.gaFeasibilityBatchSize=p.value("feasibility_batch_size",256);
            request.gaInitialPostures=p.value("initial_postures",5000);
            request.gaGenerations=p.value("generations",8);
            request.gaPointEvaluations=p.value("point_evaluations",131072);request.gaFieldEvaluations=p.value("field_evaluations",48);
            request.gaContinuityWeight=p.value("continuity_weight",2.);request.gaPostureWeight=p.value("posture_weight",.2);
        }
        if(json.contains("posture_parameters")) {
            const auto& p=json.at("posture_parameters");
            if(!p.is_object())throw std::runtime_error("posture_parameters must be an object");
            for(auto it=p.begin();it!=p.end();++it)if(it.key()!="point_evaluations" && it.key()!="local_iterations" && it.key()!="candidate_limit" && it.key()!="graph_refinement_rounds" && it.key()!="continuity_weight" && it.key()!="posture_weight")throw std::runtime_error("Unknown posture parameter: "+it.key());
            for(const auto* k:{"point_evaluations","local_iterations","candidate_limit","graph_refinement_rounds"})if(p.contains(k) && !p.at(k).is_number_integer())throw std::runtime_error("Posture budgets must be integer");
            auto& o=request.postureOptions;o.pointEvaluations=p.value("point_evaluations",384);o.localIterations=p.value("local_iterations",24);
            o.candidateLimit=p.value("candidate_limit",12);o.graphRefinementRounds=p.value("graph_refinement_rounds",4);
            o.continuityWeight=p.value("continuity_weight",2.);o.postureWeight=p.value("posture_weight",.2);
        }
        if(json.contains("seed_joint_points")) {
            if(request.pipelineStage!="apf" && request.pipelineStage!="cdf")throw std::runtime_error("Seed joints require APF or CDF stage");
            if(json.contains("validate_joint_points"))throw std::runtime_error("Seed optimization and exact validation are exclusive");
            for(const auto& p:json.at("seed_joint_points")) {
                decltype(request.optimizationSeed.points)::value_type point;
                point.time=p.at("time_s").get<double>();point.q=p.at("q_rad").get<std::vector<double>>();request.optimizationSeed.points.push_back(point);
            }
            if(request.optimizationSeed.points.empty())throw std::runtime_error("Optimization seed cannot be empty");
        }
        if(request.pipelineStage=="cdf" && request.optimizationSeed.points.empty())throw std::runtime_error("CDF requires exact APF output seed_joint_points");
        if(request.pipelineStage=="validate" && !json.contains("validate_joint_points"))throw std::runtime_error("Validation requires exact joint points");
        if(json.contains("validate_joint_points")) {
            if(request.optimizeAvoidance) throw std::runtime_error("Validation cannot optimize or replace the supplied path.");
            for(const auto& p:json.at("validate_joint_points")) {
                decltype(request.verificationTrajectory.points)::value_type point;
                point.time=p.at("time_s").get<double>(); point.q=p.at("q_rad").get<std::vector<double>>();
                request.verificationTrajectory.points.push_back(point);
            }
            if(request.verificationTrajectory.points.empty()) throw std::runtime_error("Validation path cannot be empty.");
        }
        request.optimizationIterations=json.value("optimization_iterations",3);
        request.cdfPointOptimization=json.value("cdf_point_optimization",false);
        if(json.contains("cdf_parameters")) {
            const auto& p=json.at("cdf_parameters");
            if(!p.is_object()) throw std::runtime_error("cdf_parameters must be an object");
            const auto read=[&](const char* key,double& value,double low,double high) {
                if(p.contains(key)) {const double x=p.at(key).get<double>();if(!std::isfinite(x)||x<low||x>high)throw std::runtime_error(std::string("Invalid CDF parameter: ")+key);value=x;}
            };
            for(auto it=p.begin();it!=p.end();++it) if(it.key()!="trust_region_rad" && it.key()!="seed_corridor_rad" && it.key()!="seed_tracking_weight" && it.key()!="smooth_weight" && it.key()!="repair_gain" && it.key()!="finite_difference_step_rad" && it.key()!="max_iterations" && it.key()!="smoothing_passes") throw std::runtime_error("Unknown CDF parameter: "+it.key());
            auto& c=request.cdfOptions;
            read("trust_region_rad",c.trustRegion,.0001,.2);read("seed_corridor_rad",c.seedCorridor,.001,1.0);
            read("seed_tracking_weight",c.seedTrackingWeight,.001,100);read("smooth_weight",c.smoothWeight,0,100);
            read("repair_gain",c.repairGain,.01,1);read("finite_difference_step_rad",c.finiteDifferenceStep,1e-6,.01);
            for(const auto* name : {"max_iterations","smoothing_passes"}) if(p.contains(name) && !p.at(name).is_number_integer()) throw std::runtime_error(std::string(name)+" must be an integer");
            c.maxIterations=p.value("max_iterations",3);c.postSmoothingIterations=p.value("smoothing_passes",0);
            if(c.maxIterations<1||c.maxIterations>100||c.postSmoothingIterations<0||c.postSmoothingIterations>20)throw std::runtime_error("Invalid CDF iteration count");
            request.optimizationIterations=c.maxIterations;
        }
        if(json.contains("apf_parameters")) {
            const auto& p=json.at("apf_parameters");
            if(!p.is_object())throw std::runtime_error("apf_parameters must be an object");
            for(auto it=p.begin();it!=p.end();++it)if(it.key()!="step_size_rad" && it.key()!="influence_distance_m" && it.key()!="repulsion_gain" && it.key()!="iteration_budget")throw std::runtime_error("Unknown APF parameter: "+it.key());
            const auto read=[&](const char* k,double& v,double lo,double hi){if(p.contains(k)){v=p.at(k).get<double>();if(!std::isfinite(v)||v<lo||v>hi)throw std::runtime_error(std::string("Invalid APF parameter: ")+k);}};
            read("step_size_rad",request.cdfOptions.apfStepSize,.001,.1);
            read("influence_distance_m",request.cdfOptions.apfInfluenceDistance,.005,1.);
            read("repulsion_gain",request.cdfOptions.apfRepulsionGain,.1,20.);
            if(p.contains("iteration_budget")) {
                if(!p.at("iteration_budget").is_number_integer())throw std::runtime_error("APF budget must be integer");
                request.cdfOptions.apfIterationBudget=p.at("iteration_budget").get<int>();
                if(request.cdfOptions.apfIterationBudget<100||request.cdfOptions.apfIterationBudget>10000)throw std::runtime_error("Invalid APF iteration budget");
            }
        }
        request.optimizationSeedPaths=json.value("optimization_seed_paths",std::size_t(3));
        WorkpiecePoseSearchOptions search;
        const auto vector3=[](const Json& value) -> Eigen::Vector3d {
            const auto values=value.get<std::vector<double>>();
            if(values.size()!=3) throw std::runtime_error("Expected vector with three values.");
            return Eigen::Vector3d(values[0],values[1],values[2]);
        };
        if(json.contains("pose_search")) {
            const auto& p=json.at("pose_search");
            search.enabled=p.value("enabled",true);
            if(search.enabled) {
                search.lowerPosition=vector3(p.at("lower_position_m"));
                search.upperPosition=vector3(p.at("upper_position_m"));
                search.lowerRpy=vector3(p.at("lower_rpy_rad"));
                search.upperRpy=vector3(p.at("upper_rpy_rad"));
                search.maxCandidates=p.value("max_candidates",search.maxCandidates);
                search.maxDurationSeconds=p.value("max_duration_s",search.maxDurationSeconds);
                search.requireYVertical=p.value("require_y_vertical",true);
                search.axisToleranceRad=p.value("axis_tolerance_rad",search.axisToleranceRad);
                if(p.contains("ground_normal_world")) search.groundNormalWorld=vector3(p.at("ground_normal_world"));
                if(p.contains("workpiece_from_constraint_frame")) search.workpieceFromConstraintFrame=pose(p.at("workpiece_from_constraint_frame"));
            }
        }
        const auto planned=ProjectExternalWorkpiecePlanner::plan(document,projectBasePath,request,search);
        const auto& result=planned.evaluation;
        report={{"success",result.success},{"cancelled",result.cancelled},{"retry_workpiece_pose",result.retryWorkpiecePose},
            {"candidate_counts",result.candidateCounts},{"rejected_states",result.rejectedStates},{"rejected_edges",result.rejectedEdges},
            {"joint_names",request.plan.jointNames},{"diagnostics",Json::array()},{"joint_points",Json::array()},
            {"validation_scope",request.controlPointsOnly?"original_control_points_only; interiors_unknown; not_controller_execution_certificate":"sampled_geometry_and_kinematics; not_controller_execution_certificate"}};
        for(const auto& d:result.diagnostics) report["diagnostics"].push_back({{"code",d.code},{"message",d.message}});
        for(const auto& p:result.plan.trajectory.points) report["joint_points"].push_back({{"time_s",p.time},{"q_rad",p.q}});

        report["pipeline_stage"]=request.pipelineStage;
        if(!result.kinematicAudit.empty())report["kinematic_audit"]=Json::parse(result.kinematicAudit);
        report["cartesian_points_world"]=result.cartesianPointsWorld.empty()?Json::array():Json::parse(result.cartesianPointsWorld);
        report["cartesian_points_request_frame"]=Json::array();
        Eigen::Isometry3d requestFromWorld=Eigen::Isometry3d::Identity();
        if(frame=="user")requestFromWorld=worldFromUser.inverse();
        else if(frame=="workpiece") {
            if(request.workpiece && request.workpiece->id==request.workpieceId)requestFromWorld=transform(request.workpiece->transform).inverse();
            else if(const auto* object=runtime.object(request.workpieceId))requestFromWorld=object->transform.inverse();
        }
        const auto convertWorldPose=[&](const Json& point) {
            auto converted=point;const auto t=requestFromWorld*pose(point);const Eigen::Quaterniond q(t.linear());
            converted["position_m"]={t.translation().x(),t.translation().y(),t.translation().z()};converted["quaternion_xyzw"]={q.x(),q.y(),q.z(),q.w()};
            return converted;
        };
        for(const auto& point:report["cartesian_points_world"]) {
            report["cartesian_points_request_frame"].push_back(convertWorldPose(point));
        }
        // User-frame inputs were transformed to World before evaluation. Apply
        // the same inverse conversion to unaccepted winding hints as to results.
        if(frame=="user")for(auto& diagnostic:report["diagnostics"])if(diagnostic["code"]=="ga_prefix_winding_required") {
            auto hint=Json::parse(diagnostic["message"].get<std::string>());
            for(auto& point:hint["cartesian_points_request_frame"])point=convertWorldPose(point);
            diagnostic["message"]=hint.dump();
        }
        report["scene_query_counts"]={{"state_validation_requests",result.sceneQueryCounts.stateValidationRequests},
            {"collision_detector_checks",result.sceneQueryCounts.collisionDetectorChecks},{"baffle_checks",result.sceneQueryCounts.baffleChecks},
            {"nearest_distance_requests",result.sceneQueryCounts.nearestDistanceRequests},{"distance_detector_queries",result.sceneQueryCounts.distanceDetectorQueries},
            {"scope","this evaluation snapshot; detector checks are not individual geometry-pair tests"}};
        report["surface_mapping"]=result.surfaceMapping.empty()?Json::object():Json::parse(result.surfaceMapping);
        report["velocity_telemetry"]=result.velocityTelemetry.empty()?Json::object():Json::parse(result.velocityTelemetry);
        report["optimizer"]=request.pipelineStage=="posture"?"adaptive_posture_graph":request.pipelineStage=="ga"?"continuity_aware_genetic":request.cdfPointOptimization || request.optimizeAvoidance?"apf_qp_cdf":"none";
        report["point_search_reports"]=Json::array();
        for(const auto& search:result.pointSearchReports)report["point_search_reports"].push_back(Json::parse(search));
        report["independent_points_only"]=request.independentPointsOnly;
        if(request.independentPointsOnly)report["validation_scope"]="independent_points_only; connectivity_and_interiors_unknown";
        report["control_points_only"]=request.controlPointsOnly;
        report["interiors_checked"]=!request.controlPointsOnly && result.success;
        report["tool_attachment_id"]=selectedTool;
        const Eigen::Quaterniond tcpRotation(request.linkFromTcp.linear());
        report["tcp"]={{"link",request.tcpLinkName},{"position_m",{request.linkFromTcp.translation().x(),request.linkFromTcp.translation().y(),request.linkFromTcp.translation().z()}},
            {"quaternion_xyzw",{tcpRotation.x(),tcpRotation.y(),tcpRotation.z(),tcpRotation.w()}}};
        report["candidate_layers"]=Json::array();
        for(std::size_t i=0;i<result.candidateLayers.size();++i) {
            const auto& layer=result.candidateLayers[i];
            Json item={{"point_index",layer.pointIndex},{"time_s",layer.time},{"truncated",layer.truncated},{"message",layer.message},{"candidates",Json::array()}};
            for(std::size_t j=0;j<layer.candidates.size();++j) {
                const auto& c=layer.candidates[j];
                const bool checked=i<result.candidateRejections.size() && j<result.candidateRejections[i].size();
                const auto reason=checked?result.candidateRejections[i][j]:std::string("not_checked");
                const double distance=i<result.candidateDistances.size() && j<result.candidateDistances[i].size()?result.candidateDistances[i][j]:std::numeric_limits<double>::quiet_NaN();
                item["candidates"].push_back({{"q_rad",c.joints},{"turns",c.turns},{"position_error_m",c.positionError},
                    {"orientation_error_rad",c.orientationError},{"min_distance_m",std::isfinite(distance)?Json(distance):Json(nullptr)},{"valid",checked && reason.empty()},{"rejection",reason},
                    {"configuration",{{"available",c.configuration.available},{"shoulder",c.configuration.shoulder},{"elbow",c.configuration.elbow},{"wrist",c.configuration.wrist}}}});
            }
            report["candidate_layers"].push_back(item);
        }
        report["continuous_paths"]=Json::array();
        for(const auto& p:result.continuousPaths) report["continuous_paths"].push_back({{"cost",p.cost},{"selections",p.selections}});
        const auto poseJson=[](const simulation_project::TransformDesc& p) {
            return Json{{"position_m",{p.x,p.y,p.z}},{"rpy_rad",{p.roll,p.pitch,p.yaw}}};
        };
        report["timed_out"]=planned.timedOut || std::any_of(result.diagnostics.begin(),result.diagnostics.end(),[](const auto& d){return d.code=="stage_budget_exhausted";});
        report["pose_attempts"]=Json::array();
        report["poses_rejected_by_constraints"]=planned.rejectedByConstraints;
        for(const auto& a:planned.attempts) report["pose_attempts"].push_back({{"pose",poseJson(a.pose)},{"success",a.success},{"message",a.message}});
        if(planned.selectedWorkpiecePose) report["selected_workpiece_pose"]=poseJson(*planned.selectedWorkpiecePose);

    } catch(const std::exception& e) {
        report={{"success",false},{"cancelled",cancelled && cancelled()},{"retry_workpiece_pose",false},{"error",e.what()}};
    }
    return report.dump();
}
