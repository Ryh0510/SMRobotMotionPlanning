#include <ProjectMotionPlanning/ExternalTrajectoryEvaluation.h>
#include <ProjectMotionPlanning/CdfQpTrajectoryRepair.h>
#include "CdfDistanceField.h"
#include <Eigen/SVD>
#include "PointSearchPopulation.h"
#include "SurfacePostureSearch.h"
#include "GaRadiusEvolution.h"
#include "SurfaceFeasibilitySampling.h"
#include "GpuSerialIk.h"
#include "CdfQueryBatch.h"
#include <nlohmann/json.hpp>
#include <algorithm>
#include <cmath>
#include <chrono>
#include <set>
#include <map>
#include <stdexcept>
#include <optional>

namespace motion_planning
{
    namespace
    {
        bool rigid(const Eigen::Isometry3d& t)
        {
            return t.matrix().allFinite() &&
                (t.linear().transpose()*t.linear()).isApprox(Eigen::Matrix3d::Identity(), 1e-8) &&
                std::abs(t.linear().determinant()-1.0) < 1e-8 &&
                t.matrix().row(3).isApprox(Eigen::RowVector4d(0,0,0,1), 1e-8);
        }
        Eigen::Isometry3d transform(const simulation_project::TransformDesc& p)
        {
            Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
            t.translation() = Eigen::Vector3d(p.x,p.y,p.z);
            t.linear() = (Eigen::AngleAxisd(p.yaw,Eigen::Vector3d::UnitZ()) *
                Eigen::AngleAxisd(p.pitch,Eigen::Vector3d::UnitY()) *
                Eigen::AngleAxisd(p.roll,Eigen::Vector3d::UnitX())).toRotationMatrix();
            return t;
        }
    }

    ExternalTrajectoryEvaluationResult ProjectExternalTrajectoryEvaluator::evaluate(
        const simulation_project::ProjectDocument& document, const std::filesystem::path& basePath,
        const ExternalTrajectoryEvaluationRequest& request)
    {
        ExternalTrajectoryEvaluationResult result;
        std::unique_ptr<ProjectPlanningSceneSnapshot> scene;
        std::unique_ptr<detail::CdfQueryBatch> queryBatch;
        const auto finish=[&]() {if(scene)result.sceneQueryCounts=scene->queryCounts();
            if(queryBatch){const auto counts=queryBatch->additionalQueryCounts();auto& out=result.sceneQueryCounts;
                out.stateValidationRequests+=counts.stateValidationRequests;out.collisionDetectorChecks+=counts.collisionDetectorChecks;out.baffleChecks+=counts.baffleChecks;
                out.nearestDistanceRequests+=counts.nearestDistanceRequests;out.distanceDetectorQueries+=counts.distanceDetectorQueries;}
            return result;};
        const auto evaluationStarted=std::chrono::steady_clock::now();
        const auto budgetExpired=[&]() { return request.stageTimeBudgetSeconds>0 &&
            std::chrono::duration<double>(std::chrono::steady_clock::now()-evaluationStarted).count()>=request.stageTimeBudgetSeconds; };
        const auto stop = [&]() { return (request.cancelled && request.cancelled()) || budgetExpired(); };
        const auto fail = [&](const std::string& code, const std::string& message, bool retry = false) {
            result.success=false;result.diagnostics.push_back({code,message}); result.retryWorkpiecePose = retry;
            result.cancelled = request.cancelled && request.cancelled();
            if(budgetExpired())result.diagnostics.push_back({"stage_budget_exhausted","Native stage time budget exhausted; no safe result published. This is not an infeasibility proof."});
            return finish();
        };
        try {
            if(!std::isfinite(request.stageTimeBudgetSeconds) || request.stageTimeBudgetSeconds<0 || request.stageTimeBudgetSeconds>14400 ||
                request.anchorSearchAttempts<1 || request.anchorSearchAttempts>512 ||
                !std::isfinite(request.anchorSearchSeconds) || request.anchorSearchSeconds<0 || request.anchorSearchSeconds>900)
                return fail("invalid_search_budget","Invalid stage/anchor search budget.");
            if(request.gaFeasibilityBatchSize<16 || request.gaFeasibilityBatchSize>256)return fail("invalid_ga_parameters","Feasibility batch must be 16..256");
            if(request.gaGenerations<1 || request.gaGenerations>256 || request.gaInitialPostures<24 || request.gaInitialPostures>65536 || request.gaPointEvaluations<24 || request.gaPointEvaluations>262144 || request.gaFieldEvaluations<4 || request.gaFieldEvaluations>256 ||
                !std::isfinite(request.gaContinuityWeight) || request.gaContinuityWeight<.1 || request.gaContinuityWeight>100 ||
                !std::isfinite(request.gaPostureWeight) || request.gaPostureWeight<0 || request.gaPostureWeight>100)
                return fail("invalid_ga_parameters","Invalid genetic search parameters");
            if(stop()) { return fail("cancelled", "Evaluation cancelled."); }
            if(!request.pipelineStage.empty() &&
                (request.cdfPointOptimization || request.pointSearchAttempts!=0 ||
                 (request.pipelineStage!="ik" && request.pipelineStage!="ga" && request.pipelineStage!="posture" && request.pipelineStage!="apf" && request.pipelineStage!="cdf" && request.pipelineStage!="validate")))
                return fail("invalid_pipeline_stage","Explicit workflow cannot run single-point CDF/population search.");
            if(request.pipelineStage=="cdf" && request.optimizationSeed.points.empty())
                return fail("cdf_seed_required","CDF requires the exact preceding APF trajectory.");
            if(request.pipelineStage=="ga" && (!request.controlPointsOnly || request.independentPointsOnly || request.optimizeAvoidance))
                return fail("invalid_ga_stage","GA requires control-points-only sequence selection and cannot run APF/CDF implicitly.");
            if(request.pipelineStage=="posture" && (!request.controlPointsOnly || request.independentPointsOnly || request.optimizeAvoidance))
                return fail("invalid_posture_stage","Posture stage requires original controls and cannot run APF/CDF implicitly.");
            const auto& po=request.postureOptions;
            if(po.pointEvaluations<24 || po.pointEvaluations>2048 || po.localIterations<0 || po.localIterations>128 ||
                po.candidateLimit<2 || po.candidateLimit>64 || po.graphRefinementRounds<0 || po.graphRefinementRounds>8 ||
                !std::isfinite(po.continuityWeight) || po.continuityWeight<.1 || po.continuityWeight>100 ||
                !std::isfinite(po.postureWeight) || po.postureWeight<0 || po.postureWeight>100)
                return fail("invalid_posture_parameters","Invalid adaptive posture parameters");

            if(request.pointSearchPopulationStrategy<1 || request.pointSearchPopulationStrategy>3)return fail("invalid_request","Invalid point-search population strategy.");
            if(request.pointSearchStartStrategy<0 || request.pointSearchStartStrategy>3)return fail("invalid_request","Invalid point-search start strategy.");
            if(request.pointSearchSequence<0 || request.pointSearchSequence>100000 || request.pointSearchAttempts<0 || request.pointSearchAttempts>2048 ||
                !std::isfinite(request.pointSearchSeconds) || request.pointSearchSeconds<=0 || request.pointSearchSeconds>300 ||
                (request.pointSearchAttempts && !request.independentPointsOnly))
                return fail("invalid_request","Invalid independent point search budget.");
            if(!request.pointSearchAttemptLimits.empty() && (request.pointSearchAttemptLimits.size()!=request.plan.cartesianControlPoints.points.size() ||
                std::any_of(request.pointSearchAttemptLimits.begin(),request.pointSearchAttemptLimits.end(),[](int n){return n<0 || n>2048;})))return fail("invalid_request","Invalid per-round per-point search budget.");
            if(!request.pointSearchReferenceDelta.empty() && request.pointSearchReferenceDelta.size()!=request.plan.cartesianControlPoints.points.size())
                return fail("invalid_request","Point search reference count mismatch.");
            for(const auto& q:request.pointSearchReferenceDelta)if(!q.coeffs().allFinite() || std::abs(q.norm()-1)>1e-6)
                return fail("invalid_request","Invalid point search reference.");
            const bool spray=!request.sprayTargets.empty();
            const auto count=request.plan.cartesianControlPoints.points.size();
            if((!request.pointTypes.empty() && (request.pointTypes.size()!=count || std::any_of(request.pointTypes.begin(),request.pointTypes.end(),[](int t){return t<0 || t>2;}))) ||
                (!request.transitionCenters.empty() && (request.transitionCenters.size()!=count || std::any_of(request.transitionCenters.begin(),request.transitionCenters.end(),[](const auto& p){return !p.allFinite();}))) ||
                !std::isfinite(request.transitionSearchRadius) || request.transitionSearchRadius<0 || request.transitionSearchRadius>10 ||
                !std::isfinite(request.transitionSearchMaxAngleDeg) || request.transitionSearchMaxAngleDeg<=0 || request.transitionSearchMaxAngleDeg>180)
                return fail("invalid_request","Invalid transition semantics or search bounds.");
            const auto isTransition=[&](std::size_t i){return i<request.pointTypes.size() && request.pointTypes[i]==2;};
            if(spray && (request.sprayTargets.size()!=count || request.sprayDistances.size()!=count ||
                !request.sprayAxis.allFinite() || std::abs(request.sprayAxis.norm()-1)>1e-8))
                return fail("invalid_spray","Spray targets, distances and unit tool axis required for every point.");
            if(spray)for(std::size_t i=0;i<count;++i)if(!request.sprayTargets[i].allFinite() ||
                !std::isfinite(request.sprayDistances[i]) || request.sprayDistances[i]<0)
                return fail("invalid_spray","Spray targets and nonnegative distances must be finite.");
            if(!request.gaNominalSprayDistances.empty()) {
                if(!spray || request.gaNominalSprayDistances.size()!=count)return fail("invalid_spray","Nominal radius count mismatch");
                for(std::size_t i=0;i<count;++i)if(!std::isfinite(request.gaNominalSprayDistances[i]) || request.gaNominalSprayDistances[i]<0 ||
                    (request.gaRadiusEvolutionEnabled && (request.sprayDistances[i]<.2*request.gaNominalSprayDistances[i]-1e-9 || request.sprayDistances[i]>1.8*request.gaNominalSprayDistances[i]+1e-9)))
                    return fail("invalid_spray","Spray radius outside immutable source +/-80% envelope");
            }
            if(!std::isfinite(request.pointSearchMaxAngleDeg) || request.pointSearchMaxAngleDeg<=0 || request.pointSearchMaxAngleDeg>80 ||
                (!spray && request.pointSearchMaxAngleDeg>30))return fail("invalid_request","Invalid point-search angle bound.");
            const bool distanceSearch=!request.pointSearchMinDistances.empty() || !request.pointSearchMaxDistances.empty();
            if(distanceSearch) {
                if(!spray || request.pointSearchMinDistances.size()!=count || request.pointSearchMaxDistances.size()!=count)
                    return fail("invalid_spray","Distance search bounds require a fixed target and one interval per point.");
                for(std::size_t i=0;i<count;++i)if(!std::isfinite(request.pointSearchMinDistances[i]) || !std::isfinite(request.pointSearchMaxDistances[i]) ||
                    request.pointSearchMinDistances[i]<0 || request.pointSearchMinDistances[i]>request.sprayDistances[i]+1e-12 ||
                    request.pointSearchMaxDistances[i]<request.sprayDistances[i]-1e-12)
                    return fail("invalid_spray","Invalid spray distance interval.");
            }
            if(request.independentPointsOnly && (!request.controlPointsOnly || !request.returnCandidates || !request.verificationTrajectory.points.empty() || request.optimizeAvoidance))
                return fail("invalid_request","Independent point batches require candidate reporting and cannot interpolate, optimize or replay paths.");
            if(request.progress) request.progress("Building isolated planning scene");
            const auto positive = [](double v) { return std::isfinite(v) && v > 0; };
            if(request.plan.robotId.empty() || request.plan.jointNames.size()!=6 || request.tcpLinkName.empty() ||
                request.plan.cartesianControlPoints.points.empty() || !rigid(request.linkFromTcp) ||
                !positive(request.maxJointDelta) || !positive(request.validationJointStep) ||
                !positive(request.jacobianLengthScale) || !positive(request.minimumSingularValue) ||
                !positive(request.maximumTcpPositionDeviation) || !positive(request.maximumTcpOrientationDeviation) ||
                !std::isfinite(request.minimumClearance) || request.minimumClearance < 0 || request.maxSamplesPerEdge < 2 ||
                request.optimizationIterations<1 || request.optimizationIterations>100 || request.optimizationSeedPaths<1 || request.optimizationSeedPaths>30 ||
                request.ik.seedCount<=0 || request.ik.seedCount>4096 || request.ik.maxIterations<=0 ||
                request.ik.maxCandidatesPerPoint==0 || request.ik.maxCandidatesPerPoint>65536 ||
                !positive(request.ik.positionTolerance) || !positive(request.ik.orientationTolerance) ||
                !positive(request.ik.duplicateTolerance) ||
                (!request.maxJointVelocities.empty() && (request.maxJointVelocities.size()!=6 ||
                    !std::all_of(request.maxJointVelocities.begin(),request.maxJointVelocities.end(),positive))) ||
                (!request.ik.model.seedJoints.empty() && (request.ik.model.seedJoints.size()!=6 ||
                    !std::all_of(request.ik.model.seedJoints.begin(),request.ik.model.seedJoints.end(),[](double q){return std::isfinite(q);})))) {
                return fail("invalid_request", "Robot, ordered joints, TCP and finite positive validation settings are required.");
            }
            if(std::set<std::string>(request.plan.jointNames.begin(),request.plan.jointNames.end()).size() != request.plan.jointNames.size()) {
                return fail("invalid_joints", "Duplicate joint names.");
            }
            auto privateDocument = document;
            if(request.workpiece) {
                const auto& object = *request.workpiece;
                auto mesh = std::filesystem::u8path(object.sourcePath);
                if(mesh.is_relative()) { mesh = basePath / mesh; }
                if(object.id.empty() || !object.collisionEnabled || !positive(object.visualScale) ||
                    !positive(object.collisionScale) || !rigid(transform(object.transform)) ||
                    !std::filesystem::is_regular_file(mesh)) {
                    return fail("invalid_workpiece", "Workpiece requires id, valid mesh, pose, positive scales and collision enabled.");
                }
                auto found = std::find_if(privateDocument.objects.begin(),privateDocument.objects.end(),
                    [&](const auto& o) { return o.id == object.id; });
                if(found == privateDocument.objects.end()) { privateDocument.objects.push_back(object); }
                else { *found = object; }
            }
            StoredMotionPlan source = request.plan;
            Eigen::Isometry3d worldFromFrame = Eigen::Isometry3d::Identity();
            if(request.frame == ExternalTrajectoryFrame::Workpiece) {
                const auto found = std::find_if(privateDocument.objects.begin(),privateDocument.objects.end(),
                    [&](const auto& o) { return o.id == request.workpieceId; });
                if(found == privateDocument.objects.end()) { return fail("unknown_workpiece", "Trajectory workpiece frame not found."); }
                worldFromFrame = transform(found->transform);
                if(!rigid(worldFromFrame)) { return fail("invalid_frame", "Non-finite workpiece frame."); }
            }
            double previousTime = -1;
            for(auto& point : source.cartesianControlPoints.points) {
                if(!rigid(point.tcpPose) || !std::isfinite(point.time) || point.time < 0 || point.time <= previousTime) {
                    return fail("invalid_trajectory", "Poses must be rigid; times must be nonnegative and strictly increasing.");
                }
                previousTime = point.time; point.tcpPose = worldFromFrame * point.tcpPose;
            }
            auto fixed=request.optimizationSeed.points.empty()?request.verificationTrajectory.points:request.optimizationSeed.points;
            if(!fixed.empty()) {
                if(request.optimizeAvoidance && request.optimizationSeed.points.empty()) return fail("invalid_validation","Exact replay cannot optimize.");
                const auto originals=source.cartesianControlPoints.points;
                if(std::abs(fixed.front().time-originals.front().time)>1e-8 ||
                   std::abs(fixed.back().time-originals.back().time)>1e-8)
                    return fail("invalid_validation","Exact replay must cover all source times.");
                double last=-1;
                for(const auto& p:fixed) {
                    if(!std::isfinite(p.time) || p.time<=last || p.q.size()!=6 ||
                       !std::all_of(p.q.begin(),p.q.end(),[](double v){return std::isfinite(v);}))
                        return fail("invalid_validation","Invalid replay time or joints.");
                    last=p.time;
                }
                std::set<double> times;
                for(const auto& p:fixed) times.insert(p.time);
                for(const auto& p:originals) times.insert(p.time);
                auto expanded=fixed; expanded.clear(); source.cartesianControlPoints.points.clear();
                for(double time:times) {
                    auto target=originals.front(); target.time=time;
                    auto upper=std::upper_bound(originals.begin(),originals.end(),time,[](double t,const auto& p){return t<p.time;});
                    if(upper==originals.end()) target.tcpPose=originals.back().tcpPose;
                    else if(upper!=originals.begin()) {
                        const auto& a=*(upper-1); const auto& b=*upper;
                        const double t=(time-a.time)/(b.time-a.time);
                        target.tcpPose.translation()=(1-t)*a.tcpPose.translation()+t*b.tcpPose.translation();
                        target.tcpPose.linear()=Eigen::Quaterniond(a.tcpPose.linear()).slerp(t,Eigen::Quaterniond(b.tcpPose.linear())).toRotationMatrix();
                    }
                    source.cartesianControlPoints.points.push_back(target);
                    auto q=fixed.front(); q.time=time;
                    auto next=std::upper_bound(fixed.begin(),fixed.end(),time,[](double t,const auto& p){return t<p.time;});
                    if(next==fixed.end()) q.q=fixed.back().q;
                    else if(next!=fixed.begin()) {
                        const auto& a=*(next-1); const auto& b=*next;
                        const double t=(time-a.time)/(b.time-a.time);
                        for(std::size_t j=0;j<6;++j) q.q[j]=a.q[j]+t*(b.q[j]-a.q[j]);
                    }
                    expanded.push_back(q);
                }
                fixed=std::move(expanded);
            }
            auto sprayTargets=request.sprayTargets;
            for(auto& target:sprayTargets)target=worldFromFrame*target;
            auto transitionCenters=request.transitionCenters;
            if(transitionCenters.empty())for(const auto& p:request.plan.cartesianControlPoints.points)transitionCenters.push_back(p.tcpPose.translation());
            for(auto& center:transitionCenters)center=worldFromFrame*center;
            for(std::size_t i=0;i<count;++i)if(isTransition(i) && ((worldFromFrame*request.plan.cartesianControlPoints.points[i].tcpPose).translation()-transitionCenters[i]).norm()>request.transitionSearchRadius+1e-9)
                return fail("transition_bounds","Transition TCP exceeds source-anchored workspace.");
            if(spray) {
                for(std::size_t i=0;i<sprayTargets.size();++i) {
                    const Eigen::Isometry3d tcp=worldFromFrame*request.plan.cartesianControlPoints.points[i].tcpPose;
                    // Source anchors use the immutable nominal radius. A later stage
                    // may lock a different GA-selected radius for actual joint states.
                    const double sourceDistance=request.gaNominalSprayDistances.empty()?
                        request.sprayDistances[i]:request.gaNominalSprayDistances[i];
                    if(!isTransition(i) && (tcp.translation()+sourceDistance*tcp.linear()*request.sprayAxis-sprayTargets[i]).norm()>request.maximumTcpPositionDeviation)
                        return fail("spray_target_error","TCP does not point at its fixed spray target.");
                }
                if(!request.controlPointsOnly) result.diagnostics.push_back({"spray_scope",request.pipelineStage.empty()?"Fixed spray targets verified at source points; intermediate TCP interpolation is checked geometrically, surface spray distance between source points is not certified.":"Ordered source surface anchors and adjusted normals are checked at trajectory knots and sampled motion interiors; this is not STL surface projection or a continuous certificate."});
            }
            ProjectPlanningRequest sceneRequest;
            sceneRequest.robotId = source.robotId; sceneRequest.jointNames = source.jointNames;
            sceneRequest.collisionDetectorIds = request.collisionDetectorIds;
            std::string error;
            if(request.workpiece) {
                auto selected=privateDocument;
                auto& detectors=selected.collision.detectors;
                detectors.erase(std::remove_if(detectors.begin(),detectors.end(),[&](const auto& d) {
                    return std::find(request.collisionDetectorIds.begin(),request.collisionDetectorIds.end(),d.id)==request.collisionDetectorIds.end();
                }),detectors.end());
                const auto buildPlan=simulation_runtime::ProjectCollisionDetectorBuilder::collectBuildPlan(selected);
                if(!buildPlan.requiresSceneObject(request.workpiece->id,request.workpiece->name,request.workpiece->objectType)) {
                    return fail("workpiece_not_in_detectors","Imported workpiece is absent from selected collision detectors.");
                }
            }
            scene = ProjectPlanningSceneBuilder::build(privateDocument,basePath,sceneRequest,&error);
            if(!scene) { return fail("invalid_scene",error); }
            const auto baffleStatus=nlohmann::json::parse(scene->virtualBaffleStatus());
            for(const auto& b:baffleStatus["objects"])if(!b.value("enabled",false))
                result.diagnostics.push_back({"virtual_baffle_disabled",b.value("object_id",std::string())+": "+b.value("diagnostic",std::string())});
            // Fail closed when selected filters did not materialize any collision pairs.
            for(const auto& id : request.collisionDetectorIds) {
                bool effective = false;
                for(const auto& diagnostic : scene->collisionRuntime().pairDiagnostics(id)) {
                    effective = effective || diagnostic.effective;
                }
                if(!effective) { return fail("empty_collision_pairs", "No effective collision pairs for detector: " + id); }
            }
            auto* runtime=scene->simulationRuntime().robot(source.robotId);
            if(!runtime || runtime->model.linkNameToIndex.count(request.tcpLinkName)==0) {
                return fail("unknown_tcp_link","TCP link is absent from the actual robot model.");
            }
            std::vector<std::size_t> dofs;
            for(const auto& name : source.jointNames) {
                const auto& joint=runtime->model.joints.at(runtime->model.jointNameToIndex.at(name));
                if(joint.type!=robot::JointType::Revolute) { return fail("unsupported_joint","Multi-IK currently requires six revolute joints."); }
                dofs.push_back(static_cast<std::size_t>(joint.dofIndex));
            }
            const auto fk = [&](const std::vector<double>& q) -> Eigen::Isometry3d {
                // Numerical IK probes are kinematic only and can temporarily exceed limits.
                // Final candidates and every validation sample are checked separately.
                for(std::size_t j=0;j<dofs.size();++j) { runtime->instance->setJoint(dofs[j],q.at(j)); }
                runtime->instance->update();
                Eigen::Isometry3d pose = scene->linkWorldTransform(request.tcpLinkName) * request.linkFromTcp;
                if(!rigid(pose)) { throw std::runtime_error("Invalid TCP transform."); }
                return pose;
            };
            auto ikOptions = request.ik;
            ikOptions.model.robotId = source.robotId; ikOptions.model.jointNames = source.jointNames;
            ikOptions.model.worldForwardKinematics = fk;
            ikOptions.cancelled = stop;
            // Actual model bounds, never nominal ABB limits. Continuous axes require an explicit window.
            const auto dimension = source.jointNames.size();
            if(ikOptions.lower.empty() && ikOptions.upper.empty()) {
                for(const auto& bound : scene->jointBounds()) {
                    if(bound.continuous) { return fail("search_window_required", "Continuous joints require explicit finite IK search bounds."); }
                    ikOptions.lower.push_back(bound.lower); ikOptions.upper.push_back(bound.upper);
                }
            }
            if(ikOptions.lower.size()!=dimension || ikOptions.upper.size()!=dimension) {
                return fail("invalid_bounds","IK search bounds must match joint count.");
            }
            for(std::size_t j=0;j<dimension;++j) {
                const auto& bound=scene->jointBounds()[j];
                if(!bound.continuous) {
                    ikOptions.lower[j]=std::max(ikOptions.lower[j],bound.lower);
                    ikOptions.upper[j]=std::min(ikOptions.upper[j],bound.upper);
                }
                if(!std::isfinite(ikOptions.lower[j]) || !std::isfinite(ikOptions.upper[j]) || ikOptions.lower[j]>=ikOptions.upper[j] ||
                    std::abs(ikOptions.lower[j])>1e6 || std::abs(ikOptions.upper[j])>1e6) {
                    return fail("invalid_bounds","Invalid or empty IK search window.");
                }
            }
            if(request.progress) request.progress("Enumerating actual-model IK candidates");
            ikOptions.progress=[&](std::size_t done,std::size_t total) {
                if(request.progress) request.progress("IK point "+std::to_string(done)+"/"+std::to_string(total));
            };
            if(request.returnCandidates && request.progress) ikOptions.layerReady=[&](const CartesianIkLayer& layer) {
                nlohmann::json event={{"type","ik_candidates"},{"point_index",layer.pointIndex},{"time_s",layer.time},
                    {"reused",layer.message.find("Reused")==0},{"continued",layer.message.find("Continued")==0},
                    {"truncated",layer.truncated},{"validation","kinematic_only; collision_and_singularity_pending"},{"candidates",nlohmann::json::array()}};
                for(const auto& c:layer.candidates) event["candidates"].push_back({{"q_rad",c.joints},{"turns",c.turns},
                    {"position_error_m",c.positionError},{"orientation_error_rad",c.orientationError}});
                request.progress(event.dump());
            };
            if(!request.startJoints.empty()) {
                if(request.startJoints.size()!=dimension || !std::all_of(request.startJoints.begin(),request.startJoints.end(),[](double v){return std::isfinite(v);})) return fail("invalid_start_joints","Invalid segment boundary joints");
                if(ikOptions.cachedCandidates.empty()) ikOptions.cachedCandidates.resize(source.cartesianControlPoints.points.size());
                ikOptions.cachedCandidates.front()={request.startJoints};
                ikOptions.model.seedJoints=request.startJoints;
            }
            detail::GpuSerialIk gpuIk;
            auto gpuIkOptions=ikOptions;gpuIkOptions.maxIterations=80;gpuIkOptions.model.damping=.001;gpuIkOptions.maxCandidatesPerPoint=128;
            const bool serialIkReady=!request.pipelineStage.empty() &&
                gpuIk.configure(runtime->model,source.jointNames,request.tcpLinkName,gpuIkOptions);
            const bool gpuIkReady=request.gaGpuEnabled && request.gaGpuIkEnabled && serialIkReady;
            const auto batch=[&]() -> detail::CdfQueryBatch& {
                if(!queryBatch)queryBatch=std::make_unique<detail::CdfQueryBatch>(*scene,privateDocument,basePath,sceneRequest,request.pipelineStage=="ga"?8:4,true,stop);
                return *queryBatch;
            };
            std::map<std::vector<double>,detail::CdfQueryBatch::StateCheck> collisionCertificates;
            const auto warmCollision=[&](const robottrajectory::JointTrajectory& trajectory){
                if(request.returnCandidates || request.pipelineStage.empty() || request.pipelineStage=="ga")return;
                std::vector<std::vector<double>> values;
                const auto add=[&](const std::vector<double>& q){if(!collisionCertificates.count(q))values.push_back(q);};
                const auto flush=[&](){const auto checked=batch().states(values,request.minimumClearance);for(std::size_t i=0;i<values.size();++i)collisionCertificates[values[i]]=checked[i];values.clear();};
                for(std::size_t i=0;i<trajectory.points.size() && !stop();++i){add(trajectory.points[i].q);if(i==0)continue;
                    const auto& a=trajectory.points[i-1];const auto& b=trajectory.points[i];double delta=0;for(std::size_t j=0;j<dimension;++j)delta=std::max(delta,std::abs(b.q[j]-a.q[j]));
                    const double steps=std::max(2.,std::ceil(delta/request.validationJointStep));if(!std::isfinite(steps) || steps>double(request.maxSamplesPerEdge))continue;
                    for(std::size_t sample=1;sample<static_cast<std::size_t>(steps);++sample){const double f=double(sample)/steps;auto q=a.q;
                        for(std::size_t j=0;j<dimension;++j)q[j]=a.q[j]+f*(b.q[j]-a.q[j]);add(q);}
                    if(values.size()>=4096)flush();
                }
                if(!values.empty())flush();
            };
            if(request.pipelineStage=="validate" && !fixed.empty()){auto replay=source.trajectory;replay.points=fixed;warmCollision(replay);}

            // Fresh original-point enumeration can use a real GPU batch. Existing
            // preferred continuation/cached roots retain their established order.
            if(gpuIkReady && request.pipelineStage=="ik" && ikOptions.cachedCandidates.empty() && ikOptions.preferredJoints.empty()) {
                std::vector<std::vector<double>> seeds;if(ikOptions.model.seedJoints.size()==6)seeds.push_back(ikOptions.model.seedJoints);
                const int primes[]={2,3,5,7,11,13};
                for(int n=1;n<=std::min(16,ikOptions.seedCount);++n){std::vector<double> q(6);for(int j=0;j<6;++j){double f=1./primes[j],h=0;for(int value=n;value;value/=primes[j],f/=primes[j])h+=(value%primes[j])*f;q[j]=-std::acos(-1.)+2*std::acos(-1.)*h;}seeds.push_back(q);}
                ikOptions.cachedCandidates.resize(source.cartesianControlPoints.points.size());
                for(std::size_t first=0;first<source.cartesianControlPoints.points.size() && !stop();first+=32){std::vector<Eigen::Isometry3d> targets;for(std::size_t i=first;i<std::min(first+32,source.cartesianControlPoints.points.size());++i)targets.push_back(source.cartesianControlPoints.points[i].tcpPose);
                    std::vector<std::vector<CartesianIkCandidate>> roots;if(!gpuIk.solve(targets,seeds,roots))break;for(std::size_t i=0;i<roots.size();++i)for(const auto& c:roots[i])ikOptions.cachedCandidates[first+i].push_back(c.joints);
                }
                result.diagnostics.push_back({"gpu_original_ik",gpuIk.status().dump()});
            }
            CartesianMultiIkResult ik;
            if(fixed.empty()) ik=ProjectTrajectoryInverseKinematics::solveAllCartesianControlPoints(source,ikOptions);
            else {
                ik.source=source; ik.success=true;
                for(std::size_t i=0;i<fixed.size();++i) {
                    const auto actual=fk(fixed[i].q); const auto& target=source.cartesianControlPoints.points[i].tcpPose;
                    if(request.pipelineStage.empty() && ((actual.translation()-target.translation()).norm()>request.maximumTcpPositionDeviation ||
                       Eigen::Quaterniond(actual.linear()).angularDistance(Eigen::Quaterniond(target.linear()))>request.maximumTcpOrientationDeviation))
                        return fail("replay_tcp_deviation","Supplied joint path does not match the source TCP targets.");
                    CartesianIkLayer layer; layer.pointIndex=i; layer.time=fixed[i].time;
                    CartesianIkCandidate c; c.joints=fixed[i].q; c.turns.resize(6); layer.candidates.push_back(c);
                    ik.layers.push_back(layer);
                }
            }
            if(!request.startJoints.empty() && !ik.layers.empty()) {
                auto& candidates=ik.layers.front().candidates;
                // A preceding APF segment may legally change the boundary surface posture.
                // Keep its EXACT joints; processValid below checks the immutable surface target.
                if(request.pipelineStage=="apf" || request.pipelineStage=="ga" || request.pipelineStage=="posture") {
                    CartesianIkCandidate boundary;boundary.joints=request.startJoints;boundary.turns.resize(dimension);
                    candidates={boundary};
                    ik.success=std::all_of(ik.layers.begin(),ik.layers.end(),[](const auto& l){return !l.candidates.empty();});
                }
                candidates.erase(std::remove_if(candidates.begin(),candidates.end(),[&](const auto& c) {
                    if(c.joints.size()!=request.startJoints.size())return true;
                    for(std::size_t j=0;j<c.joints.size();++j)if(std::abs(c.joints[j]-request.startJoints[j])>1e-8)return true;
                    return false;
                }),candidates.end());
                if(candidates.empty())return fail("segment_boundary_mismatch","No exact IK candidate at the previous segment endpoint");
            }
            for(const auto& layer : ik.layers) { result.candidateCounts.push_back(layer.candidates.size()); }
            if(request.returnCandidates) result.candidateLayers=ik.layers;
            if(!ik.success && (!request.independentPointsOnly || ik.cancelled)) { return fail(ik.cancelled?"cancelled":"ik_search_failed",ik.message,!ik.cancelled); }
            if(request.includeKinematicAudit) {
                const auto matrix=[](const Eigen::Isometry3d& pose){nlohmann::json rows=nlohmann::json::array();for(int i=0;i<4;++i)rows.push_back({pose.matrix()(i,0),pose.matrix()(i,1),pose.matrix()(i,2),pose.matrix()(i,3)});return rows;};
                nlohmann::json audit=nlohmann::json::array();
                for(std::size_t i=0;i<ik.layers.size();++i) {
                    if(ik.layers[i].candidates.empty())continue;
                    const auto q=ik.layers[i].candidates.front().joints;
                    scene->setState(q);const auto tcp=fk(q);
                    const auto validation=scene->validateState(q);
                    scene->nearestDistance(q);
                    nlohmann::json contacts=nlohmann::json::array();
                    nlohmann::json distances=nlohmann::json::array();
                    for(const auto& id:request.collisionDetectorIds) {
                        const auto* measured=scene->collisionRuntime().resultOf(id);
                        for(const auto& detector:scene->collisionRuntime().detectors())if(detector.id==id) {
                            auto options=detector.options;options.enableDistance=true;options.enableNearestPoints=true;options.distanceThreshold=100;
                            collision::CollisionResult nearest;
                            const_cast<collision::CollisionScene&>(scene->collisionRuntime().scene()).distance(options,nearest);
                            distances.push_back({{"distance_m",nearest.minDistance},{"collision",nearest.inCollision()},{"message",nearest.message},
                                {"check_message",measured?measured->message:""}});
                        }
                        if(measured)for(const auto& c:measured->contacts)
                            contacts.push_back({{"a",c.infoA.linkName},{"b",c.infoB.linkName},{"source_a",c.infoA.source},{"source_b",c.infoB.source},
                                {"object_a",c.objectA},{"object_b",c.objectB},{"triangle_a",c.triangleA},{"triangle_b",c.triangleB},
                                {"depth_m",c.penetrationDepth},{"position_m",{c.position.x(),c.position.y(),c.position.z()}},
                                {"normal",{c.normal.x(),c.normal.y(),c.normal.z()}}});
                    }
                    nlohmann::json tools=nlohmann::json::array();
                    nlohmann::json geometry=nlohmann::json::array();
                    for(const auto& g:scene->collisionRuntime().buildDebugDraw(true).geometry)
                        geometry.push_back({{"object",g.object},{"link",g.linkName},{"type",int(g.shape.type)},{"vertices",g.shape.vertices.size()},
                            {"triangles",g.shape.indices.size()/3},{"source",g.source},{"transform",matrix(g.worldTransform)}});
                    for(const auto& a:scene->simulationRuntime().mountedAttachments())if(a.owner.robotId==source.robotId)
                        tools.push_back({{"id",a.documentId},{"world_visual",matrix(a.transform.worldVisual)},
                            {"world_tcp",matrix(a.transform.worldTcp)},{"attachment_tcp",matrix(a.visual.attachmentMountToTcp)}});
                    audit.push_back({{"point_index",i},{"q_rad",q},{"target",matrix(source.cartesianControlPoints.points[i].tcpPose)},
                        {"fk_tcp",matrix(tcp)},{"tools",tools},{"collision_code",validation.diagnosticCode},{"collision_message",validation.message},
                        {"contacts",contacts},{"distances",distances},{"geometry",geometry},{"baffles",i==0?nlohmann::json::parse(scene->virtualBaffleStatus()):nlohmann::json(nullptr)}});
                }
                result.kinematicAudit=audit.dump();
            }
            const bool evolveRadius=spray && request.pipelineStage=="ga" && request.gaRadiusEvolutionEnabled;
            const auto& nominalDistances=request.gaNominalSprayDistances.empty()?request.sprayDistances:request.gaNominalSprayDistances;
            auto radiusLower=distanceSearch?request.pointSearchMinDistances:request.sprayDistances;
            auto radiusUpper=distanceSearch?request.pointSearchMaxDistances:request.sprayDistances;
            const bool variableRadius=evolveRadius || distanceSearch;
            result.gaRadiusFractions.assign(count,0.);
            if(distanceSearch)for(std::size_t i=0;i<count;++i)
                if(nominalDistances[i]>0)result.gaRadiusFractions[i]=std::max(
                    1-radiusLower[i]/nominalDistances[i],radiusUpper[i]/nominalDistances[i]-1);
            struct SurfaceReference {
                std::size_t edge=0; double fraction=0, distance=0, angle=0; bool constrained=false;
                Eigen::Vector3d anchor=Eigen::Vector3d::Zero();
                Eigen::Quaterniond orientation=Eigen::Quaterniond::Identity();
            };
            const auto postureAngle=[&](const Eigen::Quaterniond& reference,const Eigen::Quaterniond& actual) {
                if(!request.allowSprayAxisRotation)return reference.angularDistance(actual);
                return std::acos(std::clamp((reference*request.sprayAxis).dot(actual*request.sprayAxis),-1.,1.));
            };
            const auto clampSurfacePosture=[&](const Eigen::Quaterniond& reference,const Eigen::Quaterniond& actual,double limit) -> Eigen::Quaterniond {
                // Leave room for numerical IK residual at the authorized cone edge.
                limit=std::max(0.,limit-1e-5);
                const double angle=postureAngle(reference,actual);
                if(angle<=limit || angle<1e-12)return actual;
                if(!request.allowSprayAxisRotation)return reference.slerp(limit/angle,actual);
                // Shortest swing transports the normal; retain the residual axial twist.
                const auto relative=reference.conjugate()*actual;
                const Eigen::Quaterniond swing=Eigen::Quaterniond::FromTwoVectors(request.sprayAxis,relative*request.sprayAxis);
                const Eigen::Quaterniond twist=swing.conjugate()*relative;
                return (reference*Eigen::Quaterniond::Identity().slerp(limit/angle,swing)*twist).normalized();
            };
            const auto surfaceAt=[&](double time) {
                SurfaceReference r;
                if(!spray)return r;
                const auto& originals=request.plan.cartesianControlPoints.points;
                auto upper=std::upper_bound(originals.begin(),originals.end(),time,[](double t,const auto& p){return t<p.time;});
                r.edge=upper==originals.begin()?0:std::size_t(upper-originals.begin()-1);
                const auto j=std::min(r.edge+1,originals.size()-1);
                r.fraction=j==r.edge?0:std::clamp((time-originals[r.edge].time)/(originals[j].time-originals[r.edge].time),0.,1.);
                r.constrained=!isTransition(r.edge) && (r.fraction<1e-10 || !isTransition(j));
                r.anchor=(1-r.fraction)*sprayTargets[r.edge]+r.fraction*sprayTargets[j];
                r.distance=(1-r.fraction)*request.sprayDistances[r.edge]+r.fraction*request.sprayDistances[j];
                const auto originalRotation=[&](std::size_t i) -> Eigen::Quaterniond {
                    Eigen::Quaterniond q(worldFromFrame.linear()*originals[i].tcpPose.linear());
                    if(!request.pointSearchReferenceDelta.empty())q=q*request.pointSearchReferenceDelta[i];
                    return q;
                };
                r.orientation=originalRotation(r.edge).slerp(r.fraction,originalRotation(j));
                r.angle=request.pointSearchMaxAngleDeg*std::acos(-1.)/180;
                return r;
            };
            const auto mappedDistance=[&](const SurfaceReference& r,const Eigen::Isometry3d& actual) {
                if(!variableRadius || !r.constrained)return r.distance;
                const auto j=std::min(r.edge+1,count-1);
                return std::clamp((r.anchor-actual.translation()).dot(actual.linear()*request.sprayAxis),
                    (1-r.fraction)*radiusLower[r.edge]+r.fraction*radiusLower[j],
                    (1-r.fraction)*radiusUpper[r.edge]+r.fraction*radiusUpper[j]);
            };
            const auto radiusMappingValid=[&](const SurfaceReference& r,const Eigen::Isometry3d& actual) {
                const Eigen::Vector3d normal=actual.linear()*request.sprayAxis;
                if(!variableRadius)return (actual.translation()+r.distance*normal-r.anchor).norm()<=request.maximumTcpPositionDeviation;
                const auto j=std::min(r.edge+1,count-1);
                const double low=(1-r.fraction)*radiusLower[r.edge]+r.fraction*radiusLower[j];
                const double high=(1-r.fraction)*radiusUpper[r.edge]+r.fraction*radiusUpper[j];
                // Exact fixed-radius semantics until a generation actually unlocks it.
                const double distance=std::clamp(mappedDistance(r,actual),low,high);
                return (actual.translation()+distance*normal-r.anchor).norm()<=request.maximumTcpPositionDeviation;
            };
            const auto surfaceAudit=[&](const std::vector<double>& q,double time) {
                const auto r=surfaceAt(time);const auto actual=fk(q);
                const Eigen::Vector3d normal=actual.linear()*request.sprayAxis;
                const double distance=mappedDistance(r,actual);
                const Eigen::Vector3d mapped=r.anchor-distance*normal;
                return nlohmann::json{{"time_s",time},{"source_edge",r.edge},{"source_fraction",r.fraction},
                    {"constrained",r.constrained},{"surface_point_m",{r.anchor.x(),r.anchor.y(),r.anchor.z()}},
                    {"adjusted_normal_world",{normal.x(),normal.y(),normal.z()}},{"spray_distance_m",distance},
                    {"mapped_tcp_m",{mapped.x(),mapped.y(),mapped.z()}},
                    {"actual_tcp_m",{actual.translation().x(),actual.translation().y(),actual.translation().z()}},
                    {"normal_tilt_deg",std::acos(std::clamp((r.orientation*request.sprayAxis).dot(normal),-1.,1.))*180/std::acos(-1.)},
                    {"full_orientation_change_deg",r.orientation.angularDistance(Eigen::Quaterniond(actual.linear()))*180/std::acos(-1.)},
                    {"allow_spray_axis_rotation",request.allowSprayAxisRotation},
                    {"mapping_error_m",(actual.translation()-mapped).norm()}};
            };
            const auto recordVelocities=[&]() {
                using Json=nlohmann::json;
                Json data={{"frame","world"},{"linear_unit","m/s"},{"angular_unit","rad/s"},
                    {"joint_unit","rad/s"},{"sampling","joint-linear interval midpoint; calibrated TCP; no angle wrapping"},
                    {"interiors_checked",!request.controlPointsOnly},{"samples",Json::array()}};
                const auto& points=result.plan.trajectory.points;
                for(std::size_t i=1;i<points.size();++i) {
                    if(stop()) throw std::runtime_error("Velocity telemetry cancelled");
                    const auto& a=points[i-1];const auto& b=points[i];const double dt=b.time-a.time;
                    if(dt<=0 || a.q.size()!=dimension || b.q.size()!=dimension) throw std::runtime_error("Invalid telemetry trajectory timing");
                    std::vector<double> q(dimension),qd(dimension);
                    for(std::size_t j=0;j<dimension;++j){q[j]=(a.q[j]+b.q[j])/2;qd[j]=(b.q[j]-a.q[j])/dt;}
                    Eigen::MatrixXd jacobian(6,dimension);
                    for(std::size_t j=0;j<dimension;++j) {
                        // FK derivative probes are kinematic only. The IK search window is
                        // not a physical joint bound: a valid CDF repair can leave that window.
                        auto lo=q,hi=q;lo[j]-=1e-6;hi[j]+=1e-6;
                        const double h=hi[j]-lo[j];if(h<=0)throw std::runtime_error("No physical Jacobian difference interval");
                        const auto x=fk(lo),y=fk(hi);jacobian.block<3,1>(0,j)=(y.translation()-x.translation())/h;
                        const Eigen::AngleAxisd aa(y.linear()*x.linear().transpose());jacobian.block<3,1>(3,j)=aa.axis()*aa.angle()/h;
                    }
                    const Eigen::VectorXd twist=jacobian*Eigen::Map<const Eigen::VectorXd>(qd.data(),qd.size());
                    double h=std::min(1e-5,dt/4);for(double v:qd)h=std::min(h,1e-5/std::max(1.,std::abs(v)));
                    auto lo=q,hi=q;for(std::size_t j=0;j<dimension;++j){lo[j]-=h*qd[j];hi[j]+=h*qd[j];}
                    const auto x=fk(lo),y=fk(hi);Eigen::Matrix<double,6,1> fd;
                    fd.head<3>()=(y.translation()-x.translation())/(2*h);const Eigen::AngleAxisd aa(y.linear()*x.linear().transpose());fd.tail<3>()=aa.axis()*aa.angle()/(2*h);
                    Json matrix=Json::array();for(int row=0;row<6;++row){Json values=Json::array();for(std::size_t j=0;j<dimension;++j)values.push_back(jacobian(row,j));matrix.push_back(values);}
                    std::vector<double> velocity(twist.data(),twist.data()+6),difference(fd.data(),fd.data()+6);
                    data["samples"].push_back({{"from_time_s",a.time},{"to_time_s",b.time},{"time_s",(a.time+b.time)/2},
                        {"qdot_rad_s",qd},{"jacobian_world_tcp",matrix},{"tcp_twist",velocity},{"tcp_speed_m_s",twist.head<3>().norm()},
                        {"fk_difference_twist",difference},{"jacobian_fk_residual",(twist-fd).norm()}});
                }
                result.velocityTelemetry=data.dump();
                Json cartesian=Json::array();
                for(const auto& point:points) {
                    const auto pose=fk(point.q);const Eigen::Quaterniond rotation(pose.linear());
                    const auto& originals=request.plan.cartesianControlPoints.points;
                    const auto it=std::lower_bound(originals.begin(),originals.end(),point.time,[](const auto& p,double t){return p.time<t;});
                    const auto index=std::min(std::size_t(it-originals.begin()),originals.size()-1);
                    cartesian.push_back({{"time_s",point.time},{"position_m",{pose.translation().x(),pose.translation().y(),pose.translation().z()}},
                        {"quaternion_xyzw",{rotation.x(),rotation.y(),rotation.z(),rotation.w()}},{"point_type",std::abs(originals[index].time-point.time)<1e-8 && !request.pointTypes.empty()?request.pointTypes[index]:(isTransition(index)?2:0)},
                        {"spray_distance_mm",spray?mappedDistance(surfaceAt(point.time),pose)*1000:0}});
                }
                result.cartesianPointsWorld=cartesian.dump();
                if(spray) {
                    Json mapping={{"mode","fixed_surface_adjusted_normal"},{"formula","tcp = surface - distance * adjusted_normal"},
                        {"frame","world"},{"intermediate_surface","ordered interpolation of source anchors; not a projection onto STL"},{"samples",Json::array()}};
                    for(const auto& p:points)mapping["samples"].push_back(surfaceAudit(p.q,p.time));
                    result.surfaceMapping=mapping.dump();
                }
            };
            std::string stateFailure,stateFailureDetail;
            std::set<std::string> reportedFailures;
            // Central differences may cross a physical limit; use a one-sided derivative there.
            const auto stateValid = [&](const std::vector<double>& q, bool collisionChecks = true) {
                if(stop()) { return false; }
                const auto cached=collisionChecks?collisionCertificates.find(q):collisionCertificates.end();
                if(collisionChecks && cached!=collisionCertificates.end() && !cached->second.validation.valid){stateFailure=cached->second.validation.diagnosticCode;stateFailureDetail=cached->second.validation.message;++result.rejectedStates;return false;}
                const bool freshCollision=collisionChecks && cached==collisionCertificates.end();
                if(freshCollision) {
                    const auto validation=scene->validateState(q);
                    if(!validation.valid) { stateFailure=validation.diagnosticCode;stateFailureDetail=validation.message; ++result.rejectedStates; return false; }
                }
                if(freshCollision && request.minimumClearance > 0 && !scene->validateVirtualBaffles(request.minimumClearance,stateFailure)) {++result.rejectedStates;return false;}
                if(freshCollision && request.minimumClearance > 0) {
                    const auto distance=scene->nearestDistance(q);
                    if(!distance.valid || !std::isfinite(distance.distanceMeters) || distance.distanceMeters < request.minimumClearance) {
                        stateFailure="clearance_unavailable_or_too_small"; ++result.rejectedStates; return false;
                    }
                }
                const auto actualOrigin=fk(q);
                const auto origin=serialIkReady?gpuIk.forward(q):actualOrigin;
                if(serialIkReady && (actualOrigin.matrix()-origin.matrix()).norm()>1e-8){stateFailure="kinematic_model_mismatch";return false;}
                Eigen::MatrixXd jacobian(6,static_cast<Eigen::Index>(dimension));
                for(std::size_t j=0;j<dimension;++j) {
                    auto perturbed=q;
                    const double h=q[j]+1e-5 > ikOptions.upper[j] ? -1e-5 : 1e-5;
                    perturbed[j]+=h;
                    const auto pose=serialIkReady?gpuIk.forward(perturbed):fk(perturbed);
                    jacobian.block<3,1>(0,j)=(pose.translation()-origin.translation())/(h*request.jacobianLengthScale);
                    const Eigen::AngleAxisd rotation(pose.linear()*origin.linear().transpose());
                    jacobian.block<3,1>(3,j)=rotation.axis()*rotation.angle()/h;
                }
                const Eigen::JacobiSVD<Eigen::MatrixXd> svd(jacobian);
                const bool valid=dimension>=6 && svd.singularValues().allFinite() &&
                    svd.singularValues().minCoeff()>=request.minimumSingularValue;
                if(!valid) { stateFailure="near_singular_state"; ++result.rejectedStates; }
                return valid;
            };
            const auto sprayAdmissible=[&](const std::vector<double>& q,double time) {
                if(!spray) return true;
                const auto& originals=request.plan.cartesianControlPoints.points;
                const auto point=std::lower_bound(originals.begin(),originals.end(),time,[](const auto& p,double t){return p.time<t;});
                if(point==originals.end() || std::abs(point->time-time)>1e-8)return true;
                const auto i=std::size_t(point-originals.begin());if(isTransition(i))return true;
                const auto actual=fk(q);
                return radiusMappingValid(surfaceAt(time),actual);
            };
            const auto processValid=[&](const std::vector<double>& q,double time) {
                const auto actual=fk(q);
                const auto& originals=request.plan.cartesianControlPoints.points;
                auto upper=std::upper_bound(originals.begin(),originals.end(),time,[](double t,const auto& p){return t<p.time;});
                const auto i=upper==originals.begin()?0:std::size_t(upper-originals.begin()-1);
                const auto j=std::min(i+1,originals.size()-1);
                const double t=i==j?0:std::clamp((time-originals[i].time)/(originals[j].time-originals[i].time),0.,1.);
                const auto r=surfaceAt(time);
                if(spray && r.constrained)
                    return radiusMappingValid(r,actual) &&
                        postureAngle(r.orientation,Eigen::Quaterniond(actual.linear()))<=r.angle+
                            (request.allowSprayAxisRotation?std::min(1e-6,request.maximumTcpOrientationDeviation):request.maximumTcpOrientationDeviation);
                const Eigen::Vector3d position=(1-t)*(worldFromFrame*originals[i].tcpPose).translation()+t*(worldFromFrame*originals[j].tcpPose).translation();
                const auto originalRotation=[&](std::size_t k) {
                    Eigen::Quaterniond q(worldFromFrame.linear()*originals[k].tcpPose.linear());
                    if(!request.pointSearchReferenceDelta.empty())q=q*request.pointSearchReferenceDelta[k];
                    return q;
                };
                const auto reference=originalRotation(i).slerp(t,originalRotation(j));
                const bool transition=isTransition(i) && (t<1e-10 || isTransition(j));
                const Eigen::Vector3d center=(1-t)*transitionCenters[i]+t*transitionCenters[j];
                return (actual.translation()-(transition?center:position)).norm()<=(transition?request.transitionSearchRadius:request.maximumTcpPositionDeviation) &&
                    reference.angularDistance(Eigen::Quaterniond(actual.linear()))<=(transition?request.transitionSearchMaxAngleDeg:request.pointSearchMaxAngleDeg)*std::acos(-1.)/180+request.maximumTcpOrientationDeviation;
            };
            LayeredIkGraphOptions graph;
            if(request.continuousPathCount<1 || request.continuousPathCount>30)
                return fail("invalid_path_count","continuous_path_count must be 1..30");
            graph.jointWeights=request.jointWeights;
            graph.maxPaths=request.continuousPathCount; graph.maxJointDelta=request.maxJointDelta;
            graph.maxJointVelocities=request.maxJointVelocities; graph.cancelled=stop;
            if(graph.maxJointVelocities.empty()) {
                for(const auto& bound:scene->jointBounds()) {
                    if(!positive(bound.maxVelocity)) { return fail("velocity_limits_required","Supply velocity limits when the model has none."); }
                    graph.maxJointVelocities.push_back(bound.maxVelocity);
                }
            } else if(graph.maxJointVelocities.size()==dimension) {
                for(std::size_t j=0;j<dimension;++j) {
                    const auto limit=scene->jointBounds()[j].maxVelocity;
                    if(positive(limit)) graph.maxJointVelocities[j]=std::min(graph.maxJointVelocities[j],limit);
                }
            }
            const auto reportFailure=[&](std::size_t index,const std::string& reason) {
                if(!reason.empty() && reportedFailures.insert(reason).second) {
                    result.diagnostics.push_back({reason,"First rejected candidate/sample at zero-based point/segment " + std::to_string(index)});
                }
            };
            graph.nodeAdmissible=[&](std::size_t index,const CartesianIkCandidate& c) {
                if(request.returnCandidates && index<result.candidateRejections.size()) {
                    const auto& candidates=ik.layers.at(index).candidates;
                    const auto found=std::find_if(candidates.begin(),candidates.end(),[&](const auto& v){return &v==&c;});
                    if(found!=candidates.end()) {
                        const auto& reason=result.candidateRejections[index].at(static_cast<std::size_t>(found-candidates.begin()));
                        if(!reason.empty()) reportFailure(index,reason);
                        return reason.empty();
                    }
                }
                const bool valid=stateValid(c.joints); if(!valid) reportFailure(index,stateFailure); return valid;
            };
            const auto measuredPenetration=[&]() {
                double depth=0;
                for(const auto& id:request.collisionDetectorIds) {
                    const auto* measured=scene->collisionRuntime().resultOf(id);
                    if(measured && measured->inCollision())for(const auto& contact:measured->contacts)
                        if(std::isfinite(contact.penetrationDepth) && contact.penetrationDepth>depth)depth=contact.penetrationDepth;
                }
                return depth>0?depth:std::numeric_limits<double>::quiet_NaN();
            };
            // GA maintains per-candidate safety/distance arrays even when the
            // caller does not request their serialization. addSafe updates these
            // arrays during search; leaving them empty corrupts memory.
            if(request.returnCandidates || request.pipelineStage=="ga") {
                std::size_t checkedPoints=0,passedPoints=0;
                for(auto& layer:ik.layers) {
                    if(stop())return fail("cancelled","Candidate inspection interrupted.");
                    std::vector<std::string> reasons;
                    std::vector<double> distances,penetrations;
                    for(const auto& candidate:layer.candidates) {
                        if(stop())return fail("cancelled","Candidate inspection interrupted.");
                        stateFailure.clear();
                        bool valid=stateValid(candidate.joints);
                        if(valid && !(request.pipelineStage.empty()?sprayAdmissible(candidate.joints,layer.time):processValid(candidate.joints,layer.time))){valid=false;stateFailure="spray_target_error";}
                        reasons.push_back(valid?std::string():stateFailure);
                        const auto nearest=scene->nearestDistance(candidate.joints);
                        distances.push_back(!isVirtualBaffleFailure(stateFailure) && nearest.valid?nearest.distanceMeters:std::numeric_limits<double>::quiet_NaN());
                        penetrations.push_back(isVirtualBaffleFailure(stateFailure)?std::numeric_limits<double>::quiet_NaN():measuredPenetration());
                    }
                    if(request.cdfPointOptimization && fixed.empty() && !stop() &&
                        !std::any_of(reasons.begin(),reasons.end(),[](const auto& r){return r.empty();}) && !layer.candidates.empty()) {
                        const auto started=std::chrono::steady_clock::now();
                        const auto initial=source.cartesianControlPoints.points[layer.pointIndex].tcpPose;
                        const Eigen::Quaterniond initialRotation(initial.linear());
                        auto options=request.cdfOptions;options.safetyMargin=request.minimumClearance;options.targetClearance=0;
                        options.distanceThreshold=5;options.keepEndpoints=false;
                        bool solved=false;int attempts=0;std::vector<double> selected;
                        Eigen::Isometry3d selectedPose=initial;
                        for(const auto& seed:layer.candidates) {
                            auto current=seed.joints;
                            for(int iteration=0;iteration<options.maxIterations && !stop();++iteration) {
                                ++attempts;
                                auto proposed=detail::proposeCdfQpPointStep(*scene,current,seed.joints,options);
                                if(proposed.empty()) break;
                                auto target=fk(proposed);
                                const bool transition=isTransition(layer.pointIndex), fixedSpray=spray && !transition;
                                const double angle=(transition?request.transitionSearchMaxAngleDeg:request.pointSearchMaxAngleDeg)*std::acos(-1.)/180;
                                const auto reference=initialRotation*(request.pointSearchReferenceDelta.empty()?Eigen::Quaterniond::Identity():request.pointSearchReferenceDelta[layer.pointIndex]);
                                if(reference.angularDistance(Eigen::Quaterniond(target.linear()))>angle) break;
                                if(fixedSpray) target.translation()=sprayTargets[layer.pointIndex]-request.sprayDistances[layer.pointIndex]*target.linear()*request.sprayAxis;
                                else if(!transition) target.translation()=initial.translation();
                                else if((target.translation()-transitionCenters[layer.pointIndex]).norm()>request.transitionSearchRadius) break;
                                auto local=source;auto point=source.cartesianControlPoints.points[layer.pointIndex];point.tcpPose=target;local.cartesianControlPoints.points={point};
                                auto localOptions=ikOptions;localOptions.cachedCandidates.clear();localOptions.preferredJoints={proposed};localOptions.model.seedJoints=proposed;
                                localOptions.seedCount=1;localOptions.maxIterations=80;localOptions.progress={};localOptions.layerReady={};
                                const auto projected=ProjectTrajectoryInverseKinematics::solveAllCartesianControlPoints(local,localOptions);
                                if(projected.layers.empty() || projected.layers.front().candidates.empty()) break;
                                current=projected.layers.front().candidates.front().joints;
                                const auto actual=fk(current);
                                const bool process=(actual.translation()-target.translation()).norm()<=request.maximumTcpPositionDeviation &&
                                    Eigen::Quaterniond(actual.linear()).angularDistance(Eigen::Quaterniond(target.linear()))<=request.maximumTcpOrientationDeviation;
                                if(process && stateValid(current) && sprayAdmissible(current,layer.time)) {solved=true;selected=current;selectedPose=target;break;}
                                if(request.progress) request.progress(nlohmann::json{{"type","preview_state"},{"q_rad",current},{"validation","cdf_point_candidate"}}.dump());
                            }
                            if(solved) break;
                        }
                        if(solved) {
                            CartesianIkCandidate c;c.joints=selected;c.turns.resize(dimension);layer.candidates={c};reasons={std::string()};
                            const auto distance=scene->nearestDistance(selected);distances={distance.valid?distance.distanceMeters:std::numeric_limits<double>::quiet_NaN()};
                            source.cartesianControlPoints.points[layer.pointIndex].tcpPose=selectedPose;
                            const Eigen::Quaterniond delta=initialRotation.conjugate()*Eigen::Quaterniond(selectedPose.linear());
                            const Eigen::Vector3d dp=initial.linear().transpose()*(selectedPose.translation()-initial.translation());
                            result.pointSearchReports.push_back(nlohmann::json{{"point_index",layer.pointIndex},{"attempts",attempts},{"solved",true},
                                {"strategies",{{"apf_qp_cdf",{{"attempts",attempts},{"improvements",1}}}}},
                                {"orientation_delta_xyzw",{delta.x(),delta.y(),delta.z(),delta.w()}},{"position_delta_tcp_m",{dp.x(),dp.y(),dp.z()}},
                                {"position_m",{selectedPose.translation().x(),selectedPose.translation().y(),selectedPose.translation().z()}},
                                {"point_type",isTransition(layer.pointIndex)?2:0},{"fixed_spray_target",spray && !isTransition(layer.pointIndex)},
                                {"spray_distance_mm",spray?request.sprayDistances[layer.pointIndex]*1000:0},
                                {"surface_mapping",spray?surfaceAudit(selected,layer.time):nlohmann::json(nullptr)},
                                {"elapsed_ms",std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-started).count()}}.dump());
                            result.candidateCounts[layer.pointIndex]=1;
                        } else {
                            result.diagnostics.push_back({"cdf_point_unresolved","CDF/QP could not produce an admissible original point; no interior was queried."});
                            result.pointSearchReports.push_back(nlohmann::json{{"point_index",layer.pointIndex},{"attempts",attempts},{"solved",false},
                                {"strategies",{{"apf_qp_cdf",{{"attempts",attempts},{"improvements",0}}}}},
                                {"orientation_delta_xyzw",{0,0,0,1}},{"position_delta_tcp_m",{0,0,0}},
                                {"point_type",isTransition(layer.pointIndex)?2:0},{"spray_distance_mm",spray?request.sprayDistances[layer.pointIndex]*1000:0},
                                {"elapsed_ms",std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-started).count()}}.dump());
                        }
                    }
                    const int pointAttempts=request.pointSearchAttemptLimits.empty()?request.pointSearchAttempts:std::min(request.pointSearchAttempts,request.pointSearchAttemptLimits[layer.pointIndex]);
                    if(!request.cdfPointOptimization && pointAttempts>0 && !stop() &&
                        !std::any_of(reasons.begin(),reasons.end(),[](const auto& r){return r.empty();})) {
                        const auto started=std::chrono::steady_clock::now();
                        const auto expired=[&](){return std::chrono::duration<double>(std::chrono::steady_clock::now()-started).count()>=request.pointSearchSeconds;};
                        const bool transition=isTransition(layer.pointIndex);
                        const bool fixedSpray=spray && !transition;
                        const double maxAngle=transition?request.transitionSearchMaxAngleDeg:request.pointSearchMaxAngleDeg;
                        const auto center=transitionCenters[layer.pointIndex];
                        const auto initial=source.cartesianControlPoints.points[layer.pointIndex].tcpPose;
                        Eigen::Vector3d bestPosition=initial.translation();
                        const Eigen::Quaterniond initialRotation(initial.linear());
                        const auto reference=initialRotation*(request.pointSearchReferenceDelta.empty()?Eigen::Quaterniond::Identity():request.pointSearchReferenceDelta[layer.pointIndex]);
                        Eigen::Quaterniond bestRotation=initialRotation;
                        double bestSprayDistance=spray?request.sprayDistances[layer.pointIndex]:0;
                        const double lowerDistance=distanceSearch?request.pointSearchMinDistances[layer.pointIndex]:bestSprayDistance;
                        const double upperDistance=distanceSearch?request.pointSearchMaxDistances[layer.pointIndex]:bestSprayDistance;
                        std::vector<double> warm=layer.candidates.empty()?ikOptions.model.seedJoints:layer.candidates.front().joints;
                        double bestScore=-2, bestDistance=std::numeric_limits<double>::quiet_NaN(),bestDepth=std::numeric_limits<double>::quiet_NaN();
                        for(std::size_t c=0;c<reasons.size();++c) {
                            const double score=pointSearchRank(reasons[c].empty(),reasons[c],distances[c]);
                            if(score>bestScore || (score==bestScore && std::isfinite(penetrations[c]) && (!std::isfinite(bestDepth) || penetrations[c]<bestDepth))){bestScore=score;bestDepth=penetrations[c];bestDistance=distances[c];warm=layer.candidates[c].joints;}
                        }
                        int attempts=0,ikTrials=0,collisionQueries=0;bool solved=false;
                        std::map<std::string,TransitionSearchPopulation::Fitness> visited;
                        nlohmann::json history=nlohmann::json::array();
                        const double pi=std::acos(-1.0);
                        const double angleLimit=maxAngle*pi/180;
                        const Eigen::AngleAxisd initialDelta(reference.conjugate()*initialRotation);
                        TransitionSearchPopulation::X seed=TransitionSearchPopulation::X::Zero();
                        seed.head<3>()=initialDelta.axis()*initialDelta.angle()/angleLimit;
                        seed[3]=upperDistance>lowerDistance?2*(bestSprayDistance-lowerDistance)/(upperDistance-lowerDistance)-1:0;
                        if(transition && request.transitionSearchRadius>0)seed.tail<3>()=(bestPosition-center)/request.transitionSearchRadius;
                        const unsigned randomSeed=unsigned(104729*request.pointSearchSequence+7919*layer.pointIndex+17);
                        TransitionSearchPopulation ga(randomSeed,seed),de(randomSeed+1,seed),cma(randomSeed+2,seed);
                        const char* strategyNames[]={"local","genetic","differential_evolution","cma_es"};
                        nlohmann::json strategies=nlohmann::json::object();
                        for(const auto* name:strategyNames)strategies[name]={{"attempts",0},{"improvements",0}};
                        PointSearchSchedule schedule(request.pointSearchStartStrategy,request.pointSearchPopulationStrategy);
                        int ikMisses=0;
                        Eigen::Vector3d effectiveDirection=Eigen::Vector3d::Zero();
                        const auto poseKey=[](const Eigen::Quaterniond& rotation,double distance,const Eigen::Vector3d& position) {
                            std::string key;auto q=rotation.coeffs().eval();if(rotation.w()<0)q=-q;
                            for(int j=0;j<4;++j)key+=std::to_string(std::llround(q[j]*1e8))+":";
                            for(int j=0;j<3;++j)key+=std::to_string(std::llround(position[j]*1e9))+":";
                            return key+":"+std::to_string(std::llround(distance*1e9));
                        };
                        visited[poseKey(initialRotation,bestSprayDistance,bestPosition)]={bestScore,bestDepth,std::isfinite(bestDepth)};
                        for(int trial=0;trial<pointAttempts && !stop() && !expired();++trial) {
                            // A short local probe is followed by repeated population blocks.
                            // Round offset avoids always exhausting a small budget on the same algorithm.
                            const int active=schedule.begin();
                            TransitionSearchPopulation* population=active==1?&ga:active==2?&de:active==3?&cma:nullptr;
                            strategies[strategyNames[active]]["attempts"]=strategies[strategyNames[active]].value("attempts",0)+1;
                            const int axis=trial%6;
                            const double radii[]={1,3,6,12,maxAngle*0.67,maxAngle};
                            Eigen::Vector3d direction=Eigen::Vector3d::Zero();
                            Eigen::Quaterniond proposed;
                            if(trial%12<6) {
                                direction[axis/2]=(axis%2)?-1:1;
                                const double angle=radii[std::min(5,trial/24)]*pi/180;
                                proposed=bestRotation*Eigen::Quaterniond(Eigen::AngleAxisd(angle,direction));
                            } else {
                                const double sample=trial+128.0*request.pointSearchSequence;
                                const double z=1-2*((sample*.6180339887498949)-std::floor(sample*.6180339887498949));
                                const double phi=sample*2.399963229728653;
                                direction=Eigen::Vector3d(std::sqrt(std::max(0.0,1-z*z))*std::cos(phi),std::sqrt(std::max(0.0,1-z*z))*std::sin(phi),z);
                                proposed=reference*Eigen::Quaterniond(Eigen::AngleAxisd(radii[std::min(5,trial/18)]*pi/180,direction));
                            }
                            double trialSprayDistance=bestSprayDistance;
                            if(fixedSpray && distanceSearch && upperDistance>lowerDistance) {
                                const double sample=trial+128.0*request.pointSearchSequence;
                                const double fraction=trial%8==0?0:trial%8==1?1:std::fmod((sample+1)*0.6180339887498949,1.0);
                                trialSprayDistance=lowerDistance+fraction*(upperDistance-lowerDistance);
                                if(trial%8<2)proposed=bestRotation;
                            }
                            if(!population && effectiveDirection.norm()>1e-12 && trial%2==0)
                                proposed=bestRotation*Eigen::Quaterniond(Eigen::AngleAxisd(std::min(5.,radii[std::min(5,trial/24)])*pi/180,effectiveDirection));
                            Eigen::Vector3d trialPosition=bestPosition;
                            if(transition && !population && request.transitionSearchRadius>0) {
                                trialPosition+=direction*request.transitionSearchRadius*(trial%3==0?1.:.1);
                                const Eigen::Vector3d offset=trialPosition-center;
                                if(offset.norm()>request.transitionSearchRadius)trialPosition=center+offset.normalized()*request.transitionSearchRadius;
                            }
                            if(population) {
                                const auto x=population->ask(active);
                                const Eigen::Vector3d rv=x.head<3>()*angleLimit;
                                proposed=reference*Eigen::Quaterniond(Eigen::AngleAxisd(rv.norm(),rv.norm()>1e-12?rv.normalized():Eigen::Vector3d::UnitX()));
                                trialSprayDistance=lowerDistance+(x[3]+1)*.5*(upperDistance-lowerDistance);
                                if(transition)trialPosition=center+request.transitionSearchRadius*x.tail<3>();
                            }
                            proposed.normalize();
                            if(reference.angularDistance(proposed)>maxAngle*pi/180+1e-10)continue;
                            if(fixedSpray)trialPosition=sprayTargets[layer.pointIndex]-trialSprayDistance*proposed.toRotationMatrix()*request.sprayAxis;
                            const auto key=poseKey(proposed,trialSprayDistance,trialPosition);
                            const auto cached=visited.find(key);
                            if(cached!=visited.end()){if(population)population->tell(cached->second);schedule.feedback(false);continue;}
                            visited[key]={};
                            ++attempts;
                            auto point=source.cartesianControlPoints.points[layer.pointIndex];point.tcpPose.linear()=proposed.toRotationMatrix();
                            if(transition)point.tcpPose.translation()=trialPosition;
                            if(fixedSpray)point.tcpPose.translation()=sprayTargets[layer.pointIndex]-trialSprayDistance*point.tcpPose.linear()*request.sprayAxis;
                            auto localPlan=source;localPlan.cartesianControlPoints.points={point};
                            auto localOptions=ikOptions;localOptions.cachedCandidates.clear();localOptions.preferredJoints={warm};
                            localOptions.seedCount=(ikMisses>=4 || trial%16==15)?8:1;localOptions.maxIterations=80;
                            localOptions.model.seedJoints=warm;localOptions.progress={};localOptions.layerReady={};
                            localOptions.cancelled=[&](){return stop()||expired();};
                            ++ikTrials;
                            auto trialIk=ProjectTrajectoryInverseKinematics::solveAllCartesianControlPoints(localPlan,localOptions);
                            if(trialIk.layers.empty() || trialIk.layers.front().candidates.empty()) {
                                if(population)population->tell({});schedule.feedback(false);++ikMisses;
                                history.push_back({{"attempt",attempts},{"strategy",strategyNames[active]},{"position_m",{point.tcpPose.translation().x(),point.tcpPose.translation().y(),point.tcpPose.translation().z()}},{"reason","ik_search_failed"}});continue;
                            }
                            ikMisses=0;
                            auto candidates=trialIk.layers.front().candidates;
                            std::vector<std::string> trialReasons;std::vector<double> trialDistances;
                            std::string bestReason="ik_search_failed";
                            double score=-3,clearance=std::numeric_limits<double>::quiet_NaN(),depth=std::numeric_limits<double>::quiet_NaN();std::vector<double> trialWarm;
                            for(const auto& candidate:candidates) {
                                if(stop()||expired())break;
                                stateFailure.clear();++collisionQueries;
                                const auto actual=fk(candidate.joints);
                                const bool aimed=!fixedSpray || (actual.translation()+trialSprayDistance*actual.linear()*request.sprayAxis-sprayTargets[layer.pointIndex]).norm()<=request.maximumTcpPositionDeviation;
                                const bool valid=aimed && stateValid(candidate.joints);
                                if(!aimed)stateFailure="spray_target_error";
                                std::string reason=valid?std::string():stateFailure;
                                if(!valid && reason.empty())reason="not_checked";
                                const auto nearest=scene->nearestDistance(candidate.joints);
                                const double distance=!isVirtualBaffleFailure(reason) && nearest.valid?nearest.distanceMeters:std::numeric_limits<double>::quiet_NaN();
                                trialReasons.push_back(reason);trialDistances.push_back(distance);
                                const double rank=pointSearchRank(valid,reason,distance);
                                const double measuredDepth=isVirtualBaffleFailure(reason)?std::numeric_limits<double>::quiet_NaN():measuredPenetration();
                                if(rank>score || (rank==score && std::isfinite(measuredDepth) && (!std::isfinite(depth) || measuredDepth<depth))){score=rank;depth=measuredDepth;clearance=distance;trialWarm=candidate.joints;bestReason=reason;}
                            }
                            if(trialReasons.size()!=candidates.size())break;
                            const bool improved=score>bestScore+1e-12 || (score==-1 && bestScore==-1 && std::isfinite(depth) && std::isfinite(bestDepth) && depth<bestDepth-1e-9);
                            visited[key]={score,depth,std::isfinite(depth)};
                            if(population)population->tell(visited[key]);
                            schedule.feedback(improved);
                            if(improved)strategies[strategyNames[active]]["improvements"]=strategies[strategyNames[active]].value("improvements",0)+1;
                            history.push_back({{"attempt",attempts},{"strategy",strategyNames[active]},{"position_m",{point.tcpPose.translation().x(),point.tcpPose.translation().y(),point.tcpPose.translation().z()}},{"min_distance_m",clearance},{"rejection",bestReason},{"distance_scope","selected_detectors_only; unavailable_for_virtual_baffle_failure"},{"improved",improved},{"valid",score>=2},
                                {"measured_penetration_m",std::isfinite(depth)?nlohmann::json(depth):nlohmann::json(nullptr)},{"reference_angle_deg",reference.angularDistance(proposed)*180/pi},{"spray_distance_mm",trialSprayDistance*1000}});
                            if(improved) {
                                const Eigen::AngleAxisd sourceDelta(reference.conjugate()*proposed);
                                TransitionSearchPopulation::X shared=TransitionSearchPopulation::X::Zero();
                                shared.head<3>()=sourceDelta.axis()*sourceDelta.angle()/angleLimit;
                                shared[3]=upperDistance>lowerDistance?2*(trialSprayDistance-lowerDistance)/(upperDistance-lowerDistance)-1:0;
                                if(transition && request.transitionSearchRadius>0)shared.tail<3>()=(trialPosition-center)/request.transitionSearchRadius;
                                ga.guide(shared,visited[key]);de.guide(shared,visited[key]);cma.guide(shared,visited[key]);
                                const Eigen::AngleAxisd effectiveStep(bestRotation.conjugate()*proposed);
                                effectiveDirection=effectiveStep.angle()>1e-12?effectiveStep.axis():Eigen::Vector3d::Zero();
                                bestScore=score;bestDepth=depth;bestDistance=clearance;bestRotation=proposed;bestPosition=point.tcpPose.translation();bestSprayDistance=trialSprayDistance;warm=trialWarm;
                                layer.candidates=std::move(candidates);layer.truncated=trialIk.layers.front().truncated;
                                layer.message="Point-local search selected measured improvement; see point_search_reports.";
                                reasons=std::move(trialReasons);distances=std::move(trialDistances);
                                if(request.progress)request.progress(nlohmann::json{{"type","preview_state"},{"q_rad",warm}}.dump());
                            }
                            if(request.progress && (improved || attempts%8==0))
                                request.progress(nlohmann::json{{"type","point_search"},{"point_index",layer.pointIndex},{"attempts",attempts},
                                    {"strategy",strategyNames[active]},{"best_distance_m",bestDistance},{"solved",bestScore>=2},{"distance_plateau",bestScore==-1}}.dump());
                            if(bestScore>=2){solved=true;break;}

                        }
                        const Eigen::Quaterniond delta=initialRotation.conjugate()*bestRotation;
                        const Eigen::Vector3d positionDelta=initial.linear().transpose()*(bestPosition-initial.translation());
                        result.pointSearchReports.push_back(nlohmann::json{{"point_index",layer.pointIndex},{"attempts",attempts},
                            {"strategies",strategies},{"ik_trials",ikTrials},{"collision_queries",collisionQueries},{"solved",solved},{"best_distance_m",bestDistance},
                            {"measured_penetration_m",std::isfinite(bestDepth)?nlohmann::json(bestDepth):nlohmann::json(nullptr)},
                            {"stop_reason",stop()?"cancelled":solved?"solved":expired()?"time_budget":"attempt_budget"},
                            {"distance_plateau",bestScore==-1},{"orientation_delta_xyzw",{delta.x(),delta.y(),delta.z(),delta.w()}},
                            {"point_type",transition?2:(!request.pointTypes.empty()?request.pointTypes[layer.pointIndex]:0)},{"position_delta_tcp_m",{positionDelta.x(),positionDelta.y(),positionDelta.z()}},{"position_m",{bestPosition.x(),bestPosition.y(),bestPosition.z()}},{"fixed_spray_target",fixedSpray},{"spray_distance_mm",bestSprayDistance*1000},{"max_angle_deg",maxAngle},
                            {"spray_target_error_mm",fixedSpray && !warm.empty()?nlohmann::json(1000*(fk(warm).translation()+bestSprayDistance*fk(warm).linear()*request.sprayAxis-sprayTargets[layer.pointIndex]).norm()):nlohmann::json(nullptr)},
                            {"elapsed_ms",std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-started).count()},
                            {"trials",history}}.dump());
                        result.candidateCounts[layer.pointIndex]=layer.candidates.size();
                    }
                    result.candidateRejections.push_back(std::move(reasons));
                    result.candidateDistances.push_back(std::move(distances));
                    ++checkedPoints;
                    if(std::any_of(result.candidateRejections.back().begin(),result.candidateRejections.back().end(),
                        [](const auto& reason){return reason.empty();})) ++passedPoints;
                    if(request.progress && (checkedPoints%10==0 || checkedPoints==ik.layers.size()))
                        request.progress(nlohmann::json{{"type","point_validation"},{"checked",checkedPoints},
                            {"passed",passedPoints},{"total",ik.layers.size()},
                            {"scope","point_candidates_only; continuous_path_and_independent_replay_pending"}}.dump());
                }
            }
            if(request.independentPointsOnly) {
                if(request.returnCandidates)result.candidateLayers=ik.layers;
                if(stop())return fail("cancelled","Point batch cancelled.");
                result.success=std::all_of(result.candidateRejections.begin(),result.candidateRejections.end(),
                    [](const auto& reasons){return std::any_of(reasons.begin(),reasons.end(),[](const auto& reason){return reason.empty();});});
                result.diagnostics.push_back({"point_batch_only","Only supplied independent points checked; no connectivity or interior certificate."});
                return finish();
            }
#include "AdaptivePostureStage.inc"
            if(request.pipelineStage=="ga" && fixed.empty()) {
                // Chromosome acts on the fixed SURFACE posture plus an opt-in radius.
                // Anchors, physical collision policy and joint limits are never genes.
                // Safe candidates survive in a layered pool. A second, low-dimensional
                // trajectory chromosome anticipates obstacles with smooth posture fields.
                using Population=BasicPointSearchPopulation<5>;
                using X=Population::X;
                const double pi=std::acos(-1.);
                const auto referenceFor=[&](std::size_t i) {
                    auto r=surfaceAt(ik.layers[i].time);
                    if(!spray) {
                        r.orientation=Eigen::Quaterniond(source.cartesianControlPoints.points[i].tcpPose.linear());
                        if(!request.pointSearchReferenceDelta.empty())r.orientation=r.orientation*request.pointSearchReferenceDelta[i];
                        r.angle=(isTransition(i)?request.transitionSearchMaxAngleDeg:request.pointSearchMaxAngleDeg)*pi/180;
                    }
                    return r;
                };
                std::vector<double> referenceJoints;
                for(const auto& c:ik.layers.front().candidates)if(stateValid(c.joints) && processValid(c.joints,ik.layers.front().time))
                    if(referenceJoints.empty() || Eigen::Map<const Eigen::VectorXd>(c.joints.data(),dimension).squaredNorm()<Eigen::Map<const Eigen::VectorXd>(referenceJoints.data(),dimension).squaredNorm())referenceJoints=c.joints;
                const auto jointCost=[&](const std::vector<double>& q,const std::vector<double>& other) {
                    double value=0;if(q.size()!=other.size())return value;
                    for(std::size_t j=0;j<q.size();++j)value+=(q[j]-other[j])*(q[j]-other[j]);return value;
                };
                // Request-local exact certificates, never rounded or reused across scenes.
                // The independent point/path gates still bypass this GA-only cache.
                std::map<std::pair<std::size_t,std::vector<double>>,std::pair<bool,std::string>> pointCertificates;
                std::map<std::pair<std::size_t,std::vector<double>>,double> pointPenetrations;
                std::map<std::pair<std::size_t,std::vector<double>>,double> pointBaffleViolations;
                bool enhancedFeedback=false,batchedRescue=false;
                std::size_t rescueMeritFilteredRoots=0,rescueGpuNoRoot=0;
                double gaIkMs=0,gaSafetyMs=0,gaMeritMs=0;
                std::size_t gaExactChecks=0;
                const auto rotationKey=[](const Eigen::Quaterniond& rotation,double distance){auto c=rotation.coeffs().eval();if(c[3]<0)c=-c;return std::array<double,5>{c[0],c[1],c[2],c[3],distance};};
                std::map<std::pair<std::size_t,std::array<double,5>>,std::vector<CartesianIkCandidate>> gpuPostures;
                std::size_t gpuIkFallbacks=0;
                std::map<std::vector<double>,PlanningSceneDistanceSample> gaDistances;
                const auto gaDistance=[&](const std::vector<double>& q){const auto found=gaDistances.find(q);if(found!=gaDistances.end())return found->second;
                    auto distance=scene->nearestDistance(q);gaDistances.emplace(q,distance);return distance;};

                auto* gaBatch=serialIkReady?&batch():nullptr;
                const auto safeAt=[&](const auto& c,std::size_t i) {
                    if(stop())return false;
                    const auto key=std::make_pair(i,c.joints);
                    const auto found=pointCertificates.find(key);
                    if(found!=pointCertificates.end()){stateFailure=found->second.second;return found->second.first;}
                    const auto safetyBegan=std::chrono::steady_clock::now();++gaExactChecks;
                    bool valid=stateValid(c.joints);
                    if(!valid && stateFailure=="state_in_collision")pointPenetrations[key]=measuredPenetration();
                    if(valid && !processValid(c.joints,ik.layers[i].time)){valid=false;stateFailure="process_constraint_violation";}
                    if(valid)stateFailure.clear();
                    gaSafetyMs+=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-safetyBegan).count();
                    pointCertificates.emplace(key,std::make_pair(valid,stateFailure));return valid;
                };
                const auto cacheSafety=[&](const std::vector<std::vector<double>>& joints,std::size_t i){
                    if(!gaBatch || joints.empty())return;
                    std::set<std::vector<double>> distinct;
                    std::vector<std::vector<double>> pending;
                    for(const auto& q:joints)if(!pointCertificates.count(std::make_pair(i,q)) && distinct.insert(q).second)pending.push_back(q);
                    const auto began=std::chrono::steady_clock::now();const auto checks=gaBatch->states(pending,request.minimumClearance);
                    gaExactChecks+=checks.size();
                    for(std::size_t k=0;k<checks.size();++k){const auto key=std::make_pair(i,pending[k]);const auto& checked=checks[k].validation;
                        if(checked.diagnosticCode=="state_in_collision")pointPenetrations[key]=checks[k].penetration;
                        stateFailure=checked.diagnosticCode;
                        const bool valid=checked.valid && stateValid(pending[k],false) && processValid(pending[k],ik.layers[i].time);
                        if(checked.valid && !valid && stateFailure.empty())stateFailure="process_constraint_violation";
                        pointCertificates[key]={valid,valid?std::string():stateFailure};
                    }
                    gaSafetyMs+=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-began).count();
                };
                std::vector<std::map<std::vector<double>,double>> generatedScores(ik.layers.size());
                const auto addSafe=[&](std::size_t i,const CartesianIkCandidate& c) {
                    auto& layer=ik.layers[i];
                    for(const auto& old:layer.candidates)if(jointCost(old.joints,c.joints)<1e-10)return;
                    auto& scores=generatedScores[i];
                    double neighborCost=0;int neighborSides=0;
                    for(int direction:{-1,1}) {
                        const auto n=static_cast<long long>(i)+direction;
                        if(n<0 || n>=static_cast<long long>(ik.layers.size()))continue;
                        double best=1e100;
                        for(const auto& other:ik.layers[std::size_t(n)].candidates)if(safeAt(other,std::size_t(n))) {
                            double cost=0;for(std::size_t j=0;j<dimension;++j)cost+=std::pow(std::remainder(c.joints[j]-other.joints[j],2*pi),2);
                            best=std::min(best,cost);
                        }
                        if(best<1e99){neighborCost+=best;++neighborSides;}
                    }
                    // Rank retained poses by LOCAL branch compatibility. Solely
                    // ranking against the very first robot posture discarded useful
                    // candidates halfway along a scan, especially across wrist turns.
                    const double score=-(neighborSides?neighborCost:0.)-.001*(referenceJoints.empty()?0.:jointCost(c.joints,referenceJoints));
                    if(scores.size()>=128) {
                        auto worst=std::min_element(scores.begin(),scores.end(),[](const auto& a,const auto& b){return a.second<b.second;});
                        if(score<=worst->second)return;
                        const auto old=std::find_if(layer.candidates.begin(),layer.candidates.end(),[&](const auto& x){return x.joints==worst->first;});
                        if(old!=layer.candidates.end()){const auto index=std::size_t(old-layer.candidates.begin());*old=c;
                            const auto d=gaDistance(c.joints);result.candidateDistances[i][index]=d.valid?d.distanceMeters:std::numeric_limits<double>::quiet_NaN();
                            scores.erase(worst);scores[c.joints]=score;return;}
                    }
                    scores[c.joints]=score;
                    layer.candidates.push_back(c);
                    result.candidateRejections[i].push_back(std::string());
                    const auto d=gaDistance(c.joints);
                    result.candidateDistances[i].push_back(d.valid?d.distanceMeters:std::numeric_limits<double>::quiet_NaN());
                    result.candidateCounts[i]=layer.candidates.size();
                };
                std::vector<bool> expandedBranches(ik.layers.size(),false);
                std::vector<std::vector<std::vector<double>>> branchHints(ik.layers.size());
                Population::Fitness lastPostureFitness;
                const auto solvePosture=[&](std::size_t i,const Eigen::Quaterniond& rotation,const std::vector<double>& preferred,bool explore,double distance=-1.) {
                    lastPostureFitness=Population::Fitness{};
                    const auto r=referenceFor(i);
                    auto point=source.cartesianControlPoints.points[i];point.tcpPose.linear()=rotation.toRotationMatrix();
                    if(spray && r.constrained) {
                        if(distance<0)distance=std::clamp(r.distance,radiusLower[i],radiusUpper[i]);
                        point.tcpPose.translation()=r.anchor-distance*point.tcpPose.linear()*request.sprayAxis;
                    }
                    if(batchedRescue)stateFailure="batch_ik_no_root";
                    std::vector<CartesianIkCandidate> candidates;
                    std::vector<std::vector<double>> scoredBranches;
                    struct RejectedRoot {std::vector<double> joints;Population::Fitness fitness;bool virtualFailure;};
                    std::vector<RejectedRoot> pendingMerits;
                    const auto inspect=[&](const auto& c) {
                        // Search ranking may skip a proposal, but can never accept it.
                        const auto merit=pointBaffleViolations.find(std::make_pair(i,c.joints));
                        if(enhancedFeedback && merit!=pointBaffleViolations.end() && merit->second>1e-10) {
                            Population::Fitness f{-1.,merit->second,true};if(f.better(lastPostureFitness))lastPostureFitness=f;
                            ++rescueMeritFilteredRoots;stateFailure="search_merit_filtered";return;
                        }
                        const auto actual=fk(c.joints);
                        if((actual.translation()-point.tcpPose.translation()).norm()>ikOptions.positionTolerance ||
                            Eigen::Quaterniond(actual.linear()).angularDistance(rotation)>ikOptions.orientationTolerance) {
                            stateFailure="ik_actual_fk_mismatch";return;
                        }
                        if(safeAt(c,i)){candidates.push_back(c);return;}
                        if(enhancedFeedback) {
                            bool duplicate=false;
                            for(const auto& q:scoredBranches){double d=0;for(std::size_t j=0;j<dimension;++j)d+=std::pow(std::remainder(q[j]-c.joints[j],2*pi),2);if(d<1e-8){duplicate=true;break;}}
                            // Only the expensive search merit is deduplicated;
                            // safety above is still checked for every exact root.
                            if(duplicate)return;scoredBranches.push_back(c.joints);
                        }
                        Population::Fitness rejection;
                        rejection.rank=(stateFailure=="state_in_collision" || isVirtualBaffleFailure(stateFailure))?-1.:-2.;
                        const auto depth=pointPenetrations.find(std::make_pair(i,c.joints));
                        if(depth!=pointPenetrations.end() && std::isfinite(depth->second)) {rejection.depth=depth->second;rejection.hasDepth=true;}
                        if(enhancedFeedback && (stateFailure=="state_in_collision" || isVirtualBaffleFailure(stateFailure))) {
                            pendingMerits.push_back({c.joints,rejection,isVirtualBaffleFailure(stateFailure)});
                            return;
                        }
                        if(rejection.better(lastPostureFitness))lastPostureFitness=rejection;
                    };
                    auto local=source;local.cartesianControlPoints.points={point};
                    auto options=ikOptions;options.cachedCandidates.clear();options.progress={};options.layerReady={};
                    options.maxIterations=80;options.seedCount=1;
                    if(serialIkReady){options.model.worldForwardKinematics=[&](const auto& q){return gpuIk.forward(q);};
                        options.model.worldJacobian=[&](const auto& q){return gpuIk.jacobian(q);};}
                    std::vector<std::vector<double>> seeds;
                    if(!preferred.empty())seeds.push_back(preferred);
                    if(explore)for(const auto& c:ik.layers[i].candidates) {
                        bool duplicate=false;for(const auto& q:seeds){double d=0;for(std::size_t j=0;j<dimension;++j)d+=std::pow(std::remainder(q[j]-c.joints[j],2*pi),2);if(d<1e-8){duplicate=true;break;}}
                        if(!duplicate)seeds.push_back(c.joints);
                        if(seeds.size()>=8)break;
                    }
                    if(seeds.empty())seeds.push_back(ikOptions.model.seedJoints);
                    // Preferred continuation deliberately enumerates only one root.
                    // Once that root collides, recover other numerical branches once
                    // per station, then reuse them as hints for all posture trials.
                    if(explore && seeds.size()<4 && !expandedBranches[i]) {
                        expandedBranches[i]=true;
                        auto broad=options;broad.preferredJoints.clear();broad.seedCount=16;broad.maxCandidatesPerPoint=64;
                        const auto ikBegan=std::chrono::steady_clock::now();
                        const auto trial=ProjectTrajectoryInverseKinematics::solveAllCartesianControlPoints(local,broad);
                        gaIkMs+=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-ikBegan).count();
                        if(!trial.layers.empty())for(const auto& c:trial.layers.front().candidates) {
                            inspect(c);
                            bool duplicate=false;
                            for(const auto& q:branchHints[i]){double d=0;for(std::size_t j=0;j<dimension;++j)d+=std::pow(std::remainder(q[j]-c.joints[j],2*pi),2);if(d<1e-8){duplicate=true;break;}}
                            if(!duplicate)branchHints[i].push_back(c.joints);
                        }
                    }
                    if(explore)for(const auto& q:branchHints[i]) {
                        bool duplicate=false;
                        for(const auto& old:seeds){double d=0;for(std::size_t j=0;j<dimension;++j)d+=std::pow(std::remainder(q[j]-old[j],2*pi),2);if(d<1e-8){duplicate=true;break;}}
                        if(!duplicate)seeds.push_back(q);
                        if(seeds.size()>=8)break;
                    }
                    // A hot-started layer may contain just one wrist branch. A
                    // numerical alternate seed avoids inheriting that restriction
                    // for every posture. It is NOT assumed to be an analytic root:
                    // actual FK, bounds and all safety/process checks still apply.
                    if(explore && seeds.size()<4 && seeds.front().size()==6) {
                        auto alternate=seeds.front();alternate[3]+=pi;alternate[4]=-alternate[4];alternate[5]+=pi;
                        for(std::size_t j=0;j<dimension;++j) {
                            while(alternate[j]>ikOptions.upper[j] && alternate[j]-2*pi>=ikOptions.lower[j])alternate[j]-=2*pi;
                            while(alternate[j]<ikOptions.lower[j] && alternate[j]+2*pi<=ikOptions.upper[j])alternate[j]+=2*pi;
                            alternate[j]=std::clamp(alternate[j],ikOptions.lower[j],ikOptions.upper[j]);
                        }
                        if(std::none_of(seeds.begin(),seeds.end(),[&](const auto& q){double d=0;for(std::size_t j=0;j<dimension;++j)d+=std::pow(std::remainder(q[j]-alternate[j],2*pi),2);return d<1e-8;}))seeds.push_back(alternate);
                    }
                    bool gpuSolved=false;
                    if(gpuIkReady && enhancedFeedback) {
                        const auto key=std::make_pair(i,rotationKey(rotation,distance));const auto cached=gpuPostures.find(key);
                        if(cached!=gpuPostures.end()){
                            if(batchedRescue && cached->second.empty())++rescueGpuNoRoot;
                            if(batchedRescue || !cached->second.empty()){for(const auto& c:cached->second)inspect(c);gpuSolved=true;}
                            gpuPostures.erase(cached);
                        }
                        else {std::vector<std::vector<CartesianIkCandidate>> roots;if(gpuIk.solve({point.tcpPose},seeds,roots,true) && !roots.empty() && !roots.front().empty()){for(const auto& c:roots.front())inspect(c);gpuSolved=true;}}
                        if(!gpuSolved)++gpuIkFallbacks;
                    }
                    if(!gpuSolved) {
                        const auto ikBegan=std::chrono::steady_clock::now();
                        std::vector<CartesianMultiIkResult> trials(seeds.size());
                        const auto solveSeed=[&](std::size_t n){auto independent=options;independent.preferredJoints={seeds[n]};independent.model.seedJoints=seeds[n];independent.classifyConfiguration={};
                            trials[n]=ProjectTrajectoryInverseKinematics::solveAllCartesianControlPoints(local,independent);};
                        if(gaBatch)gaBatch->kinematics(seeds.size(),solveSeed);
                        else for(std::size_t n=0;n<seeds.size() && !stop();++n)solveSeed(n);
                        gaIkMs+=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-ikBegan).count();
                        std::vector<std::vector<double>> pending;
                        for(const auto& trial:trials)if(!trial.layers.empty())for(const auto& c:trial.layers.front().candidates)pending.push_back(c.joints);
                        cacheSafety(pending,i);
                        // Merge in original seed order, preserving deterministic evolution.
                        for(auto& trial:trials)if(!trial.layers.empty())for(auto& c:trial.layers.front().candidates){
                            if(options.classifyConfiguration)c.configuration=options.classifyConfiguration(c.joints);
                            inspect(c);
                        }
                    }
                    // Batch the actual IK roots from this posture. Only search
                    // ranking runs on CUDA; every root above still passes the
                    // unchanged CPU physical/process safety gate independently.
                    std::vector<std::vector<double>> uncached;
                    for(const auto& root:pendingMerits)if(!pointBaffleViolations.count(std::make_pair(i,root.joints)))uncached.push_back(root.joints);
                    if(!uncached.empty()) {
                        const auto meritBegan=std::chrono::steady_clock::now();
                        const auto merits=scene->virtualBaffleSearchViolations(uncached,request.minimumClearance,request.gaGpuEnabled);
                        gaMeritMs+=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-meritBegan).count();
                        for(std::size_t j=0;j<uncached.size();++j)pointBaffleViolations[std::make_pair(i,uncached[j])]=merits.at(j);
                    }
                    for(auto& root:pendingMerits) {
                        const auto measured=pointBaffleViolations.find(std::make_pair(i,root.joints));
                        if(measured!=pointBaffleViolations.end() && std::isfinite(measured->second) &&
                            (root.fitness.hasDepth || root.virtualFailure || measured->second>0)) {
                            root.fitness.depth=(root.fitness.hasDepth?root.fitness.depth:0.)+measured->second;
                            root.fitness.hasDepth=true;
                        }
                        if(root.fitness.better(lastPostureFitness))lastPostureFitness=root.fitness;
                    }
                    return candidates;
                };
                const auto decode=[&](const Eigen::Vector4d& gene,const SurfaceReference& r) {
                    return surfacePostureDelta(gene,request.sprayAxis,request.allowSprayAxisRotation,std::max(0.,r.angle-1e-5));
                };
                // Transport a SAFE relative surface posture along the scan in both
                // directions. Every station keeps its own fixed surface anchor and
                // solves real IK warm-started from the preceding joint solution.
                // This creates coherent branches rather than isolated safe poses.
                int transportedCandidates=0;
                const auto transportSafeField=[&](std::size_t station) {
                    std::vector<CartesianIkCandidate> starts;
                    std::vector<Eigen::Quaterniond> rotations;
                    for(const auto& c:ik.layers[station].candidates)if(safeAt(c,station)) {
                        const Eigen::Quaterniond rotation(fk(c.joints).linear());
                        if(std::any_of(rotations.begin(),rotations.end(),[&](const auto& r){return r.angularDistance(rotation)<1e-4;}))continue;
                        starts.push_back(c);rotations.push_back(rotation);
                        if(starts.size()>=4)break;
                    }
                    for(const auto& start:starts) {
                        const auto startPose=fk(start.joints);
                        const Eigen::Quaterniond relative=referenceFor(station).orientation.conjugate()*Eigen::Quaterniond(startPose.linear());
                        const double startRadius=mappedDistance(referenceFor(station),startPose);
                        for(int direction:{-1,1}) {
                            auto previous=start.joints;
                            for(long long index=static_cast<long long>(station)+direction;
                                index>=0 && index<static_cast<long long>(ik.layers.size()) && !stop();index+=direction) {
                                const auto i=static_cast<std::size_t>(index);
                                if(i==0 && !request.startJoints.empty())break;
                                const auto r=referenceFor(i);
                                const auto rotation=clampSurfacePosture(r.orientation,r.orientation*relative,r.angle);
                                const auto candidates=solvePosture(i,rotation,previous,false,
                                    spray && r.constrained?std::clamp(startRadius,radiusLower[i],radiusUpper[i]):-1.);
                                if(candidates.empty())break;
                                const auto best=std::min_element(candidates.begin(),candidates.end(),[&](const auto& a,const auto& b){return jointCost(a.joints,previous)<jointCost(b.joints,previous);});
                                const auto before=ik.layers[i].candidates.size();addSafe(i,*best);
                                transportedCandidates+=int(ik.layers[i].candidates.size()-before);
                                previous=best->joints;
                            }
                        }
                    }
                };
                if(!ProjectLayeredIkGraph::filter(ik,graph).success) {
                    transportSafeField(0);
                    if(ik.layers.size()>1)transportSafeField(ik.layers.size()-1);
                }
                const auto wrappedJump=[&](const auto& a,const auto& b) {
                    double jump=0;for(std::size_t j=0;j<dimension;++j)jump=std::max(jump,std::abs(std::remainder(a[j]-b[j],2*pi)));return jump;
                };
                const auto searchPoint=[&](std::size_t i,const std::vector<std::vector<double>>& linkFrom)->std::optional<ExternalTrajectoryEvaluationResult> {
                    auto& layer=ik.layers[i];
                    const bool connectionRepair=!linkFrom.empty();
                    const auto connectable=[&](const auto& q){return !connectionRepair || std::any_of(linkFrom.begin(),linkFrom.end(),[&](const auto& old){return wrappedJump(q,old)<=request.maxJointDelta;});};
                    if(!request.gaEvaluateAllPoints && !connectionRepair && std::any_of(layer.candidates.begin(),layer.candidates.end(),[&](const auto& c){return safeAt(c,i);}))return std::nullopt;
                    if(i==0 && !request.startJoints.empty())return fail("ga_locked_boundary","Previous segment boundary is unsafe; it cannot be changed independently");
                    const auto began=std::chrono::steady_clock::now();const auto r=referenceFor(i);
                    const auto pointMeritStart=rescueMeritFilteredRoots,pointNoRootStart=rescueGpuNoRoot;
                    const double nominal=evolveRadius && r.constrained?nominalDistances[i]:r.distance;
                    GaRadiusEvolution radius(evolveRadius && r.constrained);
                    if(evolveRadius && r.constrained){radiusLower[i]=radiusUpper[i]=nominal;pointCertificates.clear();}
                    nlohmann::json radiusRounds=nlohmann::json::array();
                    Population population(947u+unsigned(i),X::Zero());
                    if(request.allowSprayAxisRotation)population.setPeriodicAxis(3);
                    // Preserve the original four-dimensional GA's random sequence
                    // and all its nominal-radius offspring. Radius exploration has
                    // its own population and never consumes those search slots.
                    using NominalPopulation=PointSearchPopulation;
                    NominalPopulation nominalPopulation(947u+unsigned(i),Eigen::Vector4d::Zero());
                    if(request.allowSprayAxisRotation)nominalPopulation.setPeriodicAxis(3);
                    nominalPopulation.useGuidedGeneticEvolution(request.sprayAxis,request.allowSprayAxisRotation);
                    std::vector<X> neighborGenes;
                    auto pointPreferred=referenceJoints;
                    for(std::size_t distance=1;distance<ik.layers.size() && neighborGenes.size()<16;++distance) {
                        for(int sign:{-1,1}) {
                            const auto index=static_cast<long long>(i)+sign*static_cast<long long>(distance);
                            if(index<0 || index>=static_cast<long long>(ik.layers.size()))continue;
                            const auto n=static_cast<std::size_t>(index);
                            for(const auto& candidate:ik.layers[n].candidates)if(safeAt(candidate,n)) {
                                const auto relative=referenceFor(n).orientation.conjugate()*Eigen::Quaterniond(fk(candidate.joints).linear());
                                const auto swing=Eigen::Quaterniond::FromTwoVectors(request.sprayAxis,relative*request.sprayAxis);
                                const auto twist=swing.conjugate()*relative;const Eigen::AngleAxisd aa(swing);
                                X x=X::Zero();x.head<3>()=aa.axis()*std::min(1.,aa.angle()/std::max(1e-8,r.angle));
                                x[3]=request.allowSprayAxisRotation?std::remainder(2*std::atan2(twist.vec().dot(request.sprayAxis),twist.w()),2*pi)/pi:0.;
                                bool duplicate=false;for(const auto& old:neighborGenes)if((old-x).norm()<1e-4)duplicate=true;
                                if(duplicate)continue;
                                neighborGenes.push_back(x);if(neighborGenes.size()==1)pointPreferred=candidate.joints;
                                const auto u=request.sprayAxis.unitOrthogonal(),v=request.sprayAxis.cross(u).eval();
                                for(const Eigen::Vector3d& axis:std::vector<Eigen::Vector3d>{u,v})for(double amount:{-.04,.04}) {
                                    X local=x;local.head<3>()+=amount*axis;
                                    if(local.head<3>().norm()>1)local.head<3>().normalize();neighborGenes.push_back(local);
                                }
                                if(request.allowSprayAxisRotation)for(double amount:{-.08,.08,1.,.5,-.5}){X local=x;local[3]=std::remainder(local[3]+amount,2.);neighborGenes.push_back(local);}
                                break;
                            }
                        }
                    }
                    std::map<std::array<long long,5>,Population::Fitness> visited;
                    std::map<std::string,int> rejections;
                    int evaluations=0,safeCount=0,safePostures=0,measuredTrials=0,physicalTrials=0;
                    std::vector<X> feasibleGenes;
                    std::mt19937 refinementRng(65537u+unsigned(i));std::normal_distribution<double> refinementNoise(0,1);
                    const auto evaluate=[&](const X& gene) {
                        const auto delta=decode(gene.head<4>().eval(),r);const auto rotation=clampSurfacePosture(r.orientation,r.orientation*delta,r.angle);
                        const double distance=GaRadiusEvolution::distance(nominal,gene[4]);
                        const auto orientationKey=surfacePostureKey(r.orientation.conjugate()*rotation);
                        const std::array<long long,5> key{orientationKey[0],orientationKey[1],orientationKey[2],orientationKey[3],std::llround(distance*1e12)};
                        if(visited.count(key))return visited.at(key);
                        ++evaluations;
                        const auto candidates=solvePosture(i,rotation,pointPreferred,true,distance);
                        if(std::any_of(candidates.begin(),candidates.end(),[&](const auto& c){return connectable(c.joints);}))++safePostures;
                        Population::Fitness fitness=lastPostureFitness;
                        if(fitness.hasDepth)++measuredTrials;
                        if(fitness.hasDepth && !enhancedFeedback)++physicalTrials;
                        for(const auto& c:candidates) {
                            const auto nearest=gaDistance(c.joints);
                            const double clearance=nearest.valid?std::clamp(nearest.distanceMeters,0.,.02):0.;
                            double connection=jointCost(c.joints,referenceJoints);
                            if(enhancedFeedback) {
                                double cost=0.;int sides=0;
                                for(int direction:{-1,1})for(long long n=static_cast<long long>(i)+direction;
                                    n>=0 && n<static_cast<long long>(ik.layers.size());n+=direction) {
                                    double best=1e100;
                                    for(const auto& other:ik.layers[std::size_t(n)].candidates)if(safeAt(other,std::size_t(n)))best=std::min(best,jointCost(c.joints,other.joints));
                                    if(best<1e99){cost+=best;++sides;break;}
                                }
                                if(sides)connection=cost;
                            }
                            const double score=(connectionRepair?(connectable(c.joints)?10000.:1000.):1000.)+10*clearance-request.gaContinuityWeight*connection
                                -request.gaPostureWeight*std::pow(postureAngle(r.orientation,rotation)/std::max(1e-8,r.angle),2);
                            fitness.rank=std::max(fitness.rank,score);addSafe(i,c);++safeCount;
                        }
                        if(candidates.empty())++rejections[stateFailure.empty()?"no_safe_ik_or_process":stateFailure];
                        else if(feasibleGenes.size()<64 && std::any_of(candidates.begin(),candidates.end(),[&](const auto& c){return connectable(c.joints);}))feasibleGenes.push_back(gene);
                        visited[key]=fitness;return fitness;
                    };
                    // Evaluate the complete distinct initial pool before any offspring.
                    // A safe pose cannot short-circuit this requested full schedule.
                    const auto initial=surfacePostureInitialPool(request.sprayAxis,request.allowSprayAxisRotation,
                        request.gaInitialPostures,r.angle,947u+unsigned(i));
                    population.useGuidedGeneticEvolution(request.sprayAxis,request.allowSprayAxisRotation);
                    enhancedFeedback=true;
                    int initialCompleted=0,offspringTrials=0,radiusOffspring=0;bool radiusInterrupted=false;
                    const auto canSearch=[&](){return evaluations<request.gaPointEvaluations && !stop() &&
                        (request.anchorSearchSeconds==0 || std::chrono::duration<double>(std::chrono::steady_clock::now()-began).count()<request.anchorSearchSeconds);};
                    const auto prefetch=[&](int first) {
                        if(!gpuIkReady)return;
                        std::vector<std::vector<double>> seeds;
                        const auto addSeed=[&](const auto& q){if(q.size()==6 && seeds.size()<16 && std::none_of(seeds.begin(),seeds.end(),[&](const auto& old){return jointCost(q,old)<1e-8;}))seeds.push_back(q);};
                        addSeed(pointPreferred);
                        for(const auto& c:layer.candidates)addSeed(c.joints);for(const auto& q:branchHints[i])addSeed(q);
                        // Narrow actual joint windows may contain fewer distinct
                        // seeds than requested; bounded generation must terminate.
                        const int primes[]={2,3,5,7,11,13};
                        for(int n=1;seeds.size()<16 && n<=256;++n){std::vector<double> q(6);for(int j=0;j<6;++j){double f=1./primes[j],h=0;for(int k=n;k;k/=primes[j],f/=primes[j])h+=(k%primes[j])*f;q[j]=ikOptions.lower[j]+h*(ikOptions.upper[j]-ikOptions.lower[j]);}addSeed(q);}
                        std::vector<Eigen::Isometry3d> targets;std::vector<std::array<double,5>> keys;
                        for(int n=first;n<std::min(first+128,int(initial.size()));++n){
                            const auto rotation=clampSurfacePosture(r.orientation,r.orientation*decode(initial[n],r),r.angle);
                            auto pose=source.cartesianControlPoints.points[i].tcpPose;pose.linear()=rotation.toRotationMatrix();
                            if(spray && r.constrained)pose.translation()=r.anchor-nominal*pose.linear()*request.sprayAxis;
                            targets.push_back(pose);keys.push_back(rotationKey(rotation,nominal));
                        }
                        std::vector<std::vector<CartesianIkCandidate>> roots;
                        const auto batchBegan=std::chrono::steady_clock::now();
                        const bool ready=gpuIk.solve(targets,seeds,roots,true);
                        gaIkMs+=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-batchBegan).count();
                        if(ready){
                            std::vector<std::vector<double>> joints;for(const auto& group:roots)for(const auto& c:group)joints.push_back(c.joints);
                            const auto meritBegan=std::chrono::steady_clock::now();
                            const auto merits=scene->virtualBaffleSearchViolations(joints,request.minimumClearance,request.gaGpuEnabled);
                            gaMeritMs+=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-meritBegan).count();
                            for(std::size_t k=0;k<joints.size();++k)pointBaffleViolations[std::make_pair(i,joints[k])]=merits[k];
                            std::vector<std::vector<double>> pending;
                            for(std::size_t k=0;k<joints.size();++k)if(merits[k]<=1e-10)pending.push_back(joints[k]);
                            cacheSafety(pending,i);
                            for(std::size_t n=0;n<roots.size();++n)gpuPostures[std::make_pair(i,keys[n])]=std::move(roots[n]);
                        }
                    };
                    while(initialCompleted<int(initial.size()) && canSearch()) {
                        if(initialCompleted%128==0)prefetch(initialCompleted);
                        X x=X::Zero();x.head<4>()=initial[initialCompleted];const auto f=evaluate(x);
                        population.guide(x,f);nominalPopulation.guide(x.head<4>().eval(),{f.rank,f.depth,f.hasDepth});++initialCompleted;
                        if(initialCompleted%256==0 || initialCompleted==int(initial.size()))if(request.progress)
                            request.progress(nlohmann::json{{"type","genetic_initial_pool"},{"point_index",i},
                                {"completed",initialCompleted},{"total",initial.size()},{"safe_postures",safePostures},
                                {"elapsed_ms",std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-began).count()}}.dump());
                    }
                    const int initialEvaluations=evaluations;
                    const int offspringBudget=request.gaGenerations*Population::populationSize();
                    while(initialCompleted==int(initial.size()) && offspringTrials<offspringBudget && canSearch()) {
                        const int generation=offspringTrials/Population::populationSize()+1;
                        const int birth=offspringTrials%Population::populationSize();
                        if(birth==0) {
                            const double old=radius.fraction();radius.beginGeneration(generation,safePostures>0);
                            if(radius.fraction()!=old){radiusLower[i]=nominal*(1-radius.fraction());radiusUpper[i]=nominal*(1+radius.fraction());pointCertificates.clear();}
                            result.gaRadiusFractions[i]=radius.fraction();
                            radiusRounds.push_back({{"generation",generation},{"fraction",radius.fraction()},
                                {"minimum_mm",nominal*(1-radius.fraction())*1000},{"maximum_mm",nominal*(1+radius.fraction())*1000},
                                {"safe_postures_before",safePostures},{"expanded",radius.fraction()!=old}});
                        }
                        const auto base=nominalPopulation.ask(1);X nominalGene=X::Zero();nominalGene.head<4>()=base;
                        const auto nominalFitness=evaluate(nominalGene);
                        nominalPopulation.tell(base,{nominalFitness.rank,nominalFitness.depth,nominalFitness.hasDepth});
                        ++offspringTrials;
                        if(radius.fraction()>0) {
                            if(!canSearch()){radiusInterrupted=true;break;}
                            X x=population.ask(1);x[4]=radius.clampGene(x[4]);
                            // Additional radius offspring: half stratify the whole
                            // permitted interval, half inherit/mutate radius.
                            if(birth%2==0)x[4]=(2*((birth/2+.5)/(Population::populationSize()/2.))-1)*radius.fraction()/.8;
                            population.tell(x,evaluate(x));++radiusOffspring;
                        }
                        if(offspringTrials%Population::populationSize()==0 && request.progress)request.progress(nlohmann::json{
                            {"radius_fraction",radius.fraction()},{"radius_minimum_mm",nominal*(1-radius.fraction())*1000},{"radius_maximum_mm",nominal*(1+radius.fraction())*1000},
                            {"type","genetic_generation"},{"point_index",i},{"generation",offspringTrials/Population::populationSize()},
                            {"max_generations",request.gaGenerations},{"population_size",Population::populationSize()},
                            {"radius_offspring",radiusOffspring},{"evaluations",evaluations},{"safe_postures",safePostures},
                            {"elapsed_ms",std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-began).count()}}.dump());
                    }
                    const bool scheduleComplete=initialCompleted==int(initial.size()) && initialEvaluations==int(initial.size()) && offspringTrials==offspringBudget && !radiusInterrupted;
                    int rescueBatches=0,rescueProposals=0,firstSafeBatch=-1;
                    nlohmann::json rescueHistory=nlohmann::json::array();
                    // Retain the established full schedule, then search the complete
                    // authorized shell in large real-model IK batches. Failed merit
                    // can guide exploration; refinement parents must be CPU-safe.
                    while(scheduleComplete && (feasibleGenes.empty() || (rescueBatches<firstSafeBatch+2 && feasibleGenes.size()<8)) && canSearch()) {
                        const auto checksBefore=gaExactChecks,filteredBefore=rescueMeritFilteredRoots;
                        const int size=std::min(request.gaFeasibilityBatchSize,request.gaPointEvaluations-evaluations);
                        if(size<=0)break;
                        std::vector<X> proposals;std::vector<Eigen::Isometry3d> targets;
                        for(int slot=0;slot<size;++slot) {
                            X x;
                            if(rescueBatches==0 && slot<int(neighborGenes.size()))x=neighborGenes[slot];
                            else if(slot%2==0)x=surfaceFeasibilitySample(1+7919ULL*i+std::uint64_t(rescueBatches)*size+slot,
                                request.sprayAxis,request.allowSprayAxisRotation,r.angle,radius.fraction());
                            else if(!feasibleGenes.empty()) {
                                const auto& a=feasibleGenes[(slot/2)%feasibleGenes.size()];const auto& b=feasibleGenes[(slot/2+1)%feasibleGenes.size()];
                                x=(a+b)/2;x[3]=a[3]+.5*std::remainder(b[3]-a[3],2.);
                                for(int j=0;j<5;++j)x[j]+=(rescueBatches%3==0?.08:.025)*refinementNoise(refinementRng);
                            } else x=population.ask(1);
                            x=Population::repair(x);x.head<3>()-=request.sprayAxis.dot(x.head<3>())*request.sprayAxis;
                            x[3]=request.allowSprayAxisRotation?std::remainder(x[3],2.):0.;x[4]=radius.clampGene(x[4]);
                            const auto rotation=clampSurfacePosture(r.orientation,r.orientation*decode(x.head<4>().eval(),r),r.angle);
                            auto target=source.cartesianControlPoints.points[i].tcpPose;target.linear()=rotation.toRotationMatrix();
                            if(spray && r.constrained)target.translation()=r.anchor-GaRadiusEvolution::distance(nominal,x[4])*target.linear()*request.sprayAxis;
                            proposals.push_back(x);targets.push_back(target);
                        }
                        std::vector<std::vector<double>> seeds;
                        if(pointPreferred.size()==6)seeds.push_back(pointPreferred);
                        for(const auto& c:layer.candidates)if(seeds.size()<8)seeds.push_back(c.joints);
                        const int primes[]={2,3,5,7,11,13};
                        for(int n=1+16*rescueBatches;seeds.size()<16;++n){std::vector<double> q(6);for(int j=0;j<6;++j){double f=1./primes[j],h=0;for(int k=n;k;k/=primes[j],f/=primes[j])h+=(k%primes[j])*f;q[j]=ikOptions.lower[j]+h*(ikOptions.upper[j]-ikOptions.lower[j]);}seeds.push_back(q);}
                        std::vector<std::vector<CartesianIkCandidate>> roots;
                        const auto batchIkBegan=std::chrono::steady_clock::now();
                        batchedRescue=gpuIkReady && gpuIk.solve(targets,seeds,roots,true);
                        gaIkMs+=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-batchIkBegan).count();
                        if(batchedRescue) {
                            std::vector<std::vector<double>> joints;
                            for(const auto& group:roots)for(const auto& c:group)joints.push_back(c.joints);
                            const auto meritBegan=std::chrono::steady_clock::now();
                            const auto merits=scene->virtualBaffleSearchViolations(joints,request.minimumClearance,request.gaGpuEnabled);
                            gaMeritMs+=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-meritBegan).count();
                            for(std::size_t k=0;k<joints.size();++k)pointBaffleViolations[std::make_pair(i,joints[k])]=merits[k];
                            for(std::size_t k=0;k<targets.size();++k)gpuPostures[std::make_pair(i,rotationKey(Eigen::Quaterniond(targets[k].linear()),GaRadiusEvolution::distance(nominal,proposals[k][4])))]=std::move(roots[k]);
                        }
                        for(const auto& x:proposals) {
                            if(!canSearch())break;
                            const auto f=evaluate(x);population.guide(x,f);++rescueProposals;
                        }
                        batchedRescue=false;++rescueBatches;
                        if(!feasibleGenes.empty() && firstSafeBatch<0)firstSafeBatch=rescueBatches;
                        const auto event=nlohmann::json{{"type","genetic_feasibility_batch"},{"point_index",i},{"batch",rescueBatches},
                            {"batch_size",size},{"proposals",rescueProposals},{"evaluations",evaluations},{"safe_postures",safePostures},
                            {"cpu_exact_checks",gaExactChecks-checksBefore},{"merit_ranked_roots",rescueMeritFilteredRoots-filteredBefore},
                            {"radius_fraction",radius.fraction()},{"elapsed_ms",std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-began).count()}};
                        rescueHistory.push_back(event);if(request.progress)request.progress(event.dump());
                    }
                    enhancedFeedback=false;
                    const bool solved=std::any_of(layer.candidates.begin(),layer.candidates.end(),[&](const auto& c){return connectable(c.joints) && safeAt(c,i);});
                    const auto elapsed=std::chrono::duration<double>(std::chrono::steady_clock::now()-began).count();
                    const auto audit=nlohmann::json{{"point_index",i},{"strategy","full_pool_genetic"},{"connection_repair",connectionRepair},{"attempts",evaluations},{"solved",solved},
                        {"safe_candidates_found",safeCount},{"safe_postures",safePostures},
                        {"safe_postures_scope",connectionRepair?"point_safe_and_search_connected":"point_safe"},
                        {"guided_proposals",initialCompleted},{"genetic_offspring",offspringTrials},
                        {"radius_offspring",radiusOffspring},{"nominal_generation_budget_preserved",true},
                        {"feasibility_batches",rescueBatches},{"feasibility_proposals",rescueProposals},{"feasibility_history",rescueHistory},
                        {"feasible_refinement_parents",feasibleGenes.size()},{"gpu_merit_filtered_roots",rescueMeritFilteredRoots-pointMeritStart},{"batch_gpu_no_root",rescueGpuNoRoot-pointNoRootStart},
                        {"initial_postures_evaluated",initialEvaluations},{"initial_pool_completed",initialCompleted},
                        {"schedule_complete",scheduleComplete},{"early_exit_on_safe",false},
                        {"generations_requested",request.gaGenerations},{"generations_completed",offspringTrials/Population::populationSize()},{"population_size",Population::populationSize()},
                        {"measured_penetration_trials",physicalTrials},{"measured_violation_trials",measuredTrials},
                        {"stop_reason",scheduleComplete?(solved?"full_schedule_safe":"full_schedule_no_safe_posture"):
                            stop()?"stage_interrupted":request.anchorSearchSeconds>0 && elapsed>=request.anchorSearchSeconds?"point_time_budget":"point_evaluation_budget"},
                        {"rejections",rejections},{"elapsed_ms",elapsed*1000},
                        {"gpu_enabled",request.gaGpuEnabled},{"gpu_search",nlohmann::json::parse(scene->virtualBaffleSearchStatus())},
                        {"initial_postures",initial.size()},{"gpu_ik",gpuIk.status()},{"gpu_ik_cpu_fallbacks",gpuIkFallbacks},
                        {"normal_tilt_limit_deg",r.angle*180/pi},{"retained_candidate_limit",128},
                        {"surface_anchor_m",{r.anchor.x(),r.anchor.y(),r.anchor.z()}},{"single_point_time_limit_seconds",request.anchorSearchSeconds},
                        {"initial_sampling","uniform_random_spherical_area_and_axial_spin"},{"initial_random_seed",947u+unsigned(i)},
                        {"radius_evolution_enabled",evolveRadius && r.constrained},{"nominal_radius_mm",nominal*1000},
                        {"radius_fraction",radius.fraction()},{"radius_generations",radiusRounds},
                        {"cumulative_profile",{{"ik_ms",gaIkMs},{"cpu_safety_ms",gaSafetyMs},{"merit_ms",gaMeritMs},{"exact_checks",gaExactChecks}}}};
                    result.pointSearchReports.push_back(audit.dump());
                    if(request.progress)request.progress(nlohmann::json{{"type","genetic_point"},{"report",audit}}.dump());
                    if(!scheduleComplete) {if(request.returnCandidates)result.candidateLayers=ik.layers;return fail("ga_schedule_incomplete","Initial pool or requested generations were interrupted; full schedule was not completed");}
                    if(!solved) {if(request.returnCandidates)result.candidateLayers=ik.layers;return fail(connectionRepair?"ga_connection_unresolved":"ga_point_unresolved","Genetic search did not produce an eligible safe candidate at source point "+std::to_string(i)+"; APF/CDF must not start");}
                    transportSafeField(i);
                    return std::nullopt;
                };
                for(std::size_t reverse=ik.layers.size();reverse>0 && !stop();--reverse)
                    if(const auto failure=searchPoint(reverse-1,{}))return *failure;
                if(transportedCandidates)result.diagnostics.push_back({"ga_safe_field_transport","Added "+std::to_string(transportedCandidates)+" independently safe surface-mapped candidates by bidirectional joint continuation; interior checks remain for APF."});
                // Trajectory-level genes encode a smooth relative surface-posture field.
                // Each individual still re-solves and checks every ORIGINAL point. Fitness
                // ranks safe points first, then time-aware joint connectivity and smoothness.
                using Fields=TransitionSearchPopulation;
                const auto& last=ik.layers.back();const auto endReference=referenceFor(last.pointIndex);
                std::vector<Fields::X> guides;
                std::vector<CartesianIkCandidate> endpoints;
                for(const auto& c:last.candidates)if(safeAt(c,last.pointIndex))endpoints.push_back(c);
                const auto connectionCost=[&](const auto& c) {
                    if(ik.layers.size()<2)return jointCost(c.joints,referenceJoints);
                    double value=1e100;
                    for(const auto& p:ik.layers[ik.layers.size()-2].candidates)if(safeAt(p,ik.layers.size()-2))value=std::min(value,jointCost(c.joints,p.joints));
                    return value;
                };
                std::stable_sort(endpoints.begin(),endpoints.end(),[&](const auto& a,const auto& b){return connectionCost(a)<connectionCost(b);});
                std::vector<Eigen::Quaterniond> guideRotations;
                for(const auto& c:endpoints) {
                    const auto delta=endReference.orientation.conjugate()*Eigen::Quaterniond(fk(c.joints).linear());
                    if(std::any_of(guideRotations.begin(),guideRotations.end(),[&](const auto& q){return q.angularDistance(delta)<1e-4;}))continue;
                    guideRotations.push_back(delta);
                    const auto swing=Eigen::Quaterniond::FromTwoVectors(request.sprayAxis,delta*request.sprayAxis);
                    const auto twist=swing.conjugate()*delta;const Eigen::AngleAxisd aa(swing);
                    for(double ramp:{0.,.33,.66,1.}) {
                        Fields::X x=Fields::X::Zero();x.head<3>()=aa.axis()*aa.angle()/std::max(1e-8,endReference.angle);
                        x[3]=request.allowSprayAxisRotation?std::remainder(2*std::atan2(twist.vec().dot(request.sprayAxis),twist.w()),2*pi)/pi:0;
                        x[4]=2*ramp-1;x[5]=-1;guides.push_back(x);
                    }
                    // Equal FK rotations from other IK branches add no new field.
                    if(guides.size()>=16)break;
                }
                Fields fields(8101u,guides.empty()?Fields::X::Zero():guides.front());
                if(request.allowSprayAxisRotation)fields.setPeriodicAxis(3);
                const double duration=last.time-ik.layers.front().time;
                Eigen::Quaterniond startDelta=Eigen::Quaterniond::Identity();
                if(!request.startJoints.empty())startDelta=referenceFor(0).orientation.conjugate()*Eigen::Quaterniond(fk(request.startJoints).linear());
                // Adjacent scan lines often inherit an already coherent posture field.
                // Do not run 48 redundant population evaluations when the real checked
                // input already has a velocity-feasible graph. APF and independent
                // replay still own ALL interior collision/process validation.
                auto geometricGraph=graph;geometricGraph.maxJointVelocities.assign(dimension,1e6);
                const bool coherentInput=ProjectLayeredIkGraph::filter(ik,graph).success;
                const bool retimeOnly=!coherentInput && ProjectLayeredIkGraph::filter(ik,geometricGraph).success;
                if(coherentInput)result.diagnostics.push_back({"ga_coherent_seed_reused","All new input points independently checked and velocity-connected; GA retains their coherent field without redundant field mutations. Interior collision remains unknown until APF."});
                if(retimeOnly)result.diagnostics.push_back({"ga_connected_seed_needs_timing","Safe candidate graph meets geometric jump bounds but source timing needs fresh retiming and resubmission; redundant random posture fields skipped. This is not acceptance."});
                for(int trial=0;trial<((coherentInput || retimeOnly)?0:std::min(request.gaFieldEvaluations,request.gaGenerations*Population::populationSize())) && !stop();++trial) {
                    const auto gene=trial<int(guides.size())?guides[trial]:fields.ask(1);
                    const auto posture=gene.head<4>().eval();const auto delta=decode(posture,endReference);
                    const double begin=.15*(gene[5]+1),end=.55+.225*(gene[4]+1),power=1+.5*gene[6];
                    std::vector<double> previous=referenceJoints;int passed=0,links=0;double cost=0;std::vector<double> oldVelocity;
                    for(std::size_t i=0;i<ik.layers.size() && !stop();++i) {
                        const auto r=referenceFor(i);
                        if(i==0 && !request.startJoints.empty()){previous=request.startJoints;++passed;continue;}
                        const double t=duration>0?(ik.layers[i].time-ik.layers.front().time)/duration:1.;
                        const double fraction=std::pow(std::clamp((t-begin)/std::max(.01,end-begin),0.,1.),power);
                        // A locked boundary can already have a nonzero normal swing
                        // and axial spin. Starting every field at identity broke the
                        // first connecting edges on alternating scan directions.
                        const auto rotation=clampSurfacePosture(r.orientation,r.orientation*startDelta.slerp(fraction,delta),r.angle);
                        auto candidates=solvePosture(i,rotation,previous,false);
                        if(candidates.empty()){previous.clear();oldVelocity.clear();continue;}
                        const auto best=std::min_element(candidates.begin(),candidates.end(),[&](const auto& a,const auto& b){return jointCost(a.joints,previous)<jointCost(b.joints,previous);});
                        ++passed;bool connected=true;std::vector<double> velocity;
                        connected=previous.size()==dimension;
                        if(i>0 && connected)for(std::size_t j=0;j<dimension;++j) {
                            const double dt=ik.layers[i].time-ik.layers[i-1].time,dq=best->joints[j]-previous[j];
                            velocity.push_back(dq/dt);connected &= std::abs(dq)<=request.maxJointDelta && std::abs(dq)<=graph.maxJointVelocities[j]*dt;
                        }
                        if(i>0 && connected)++links;
                        cost+=jointCost(best->joints,previous);
                        if(!oldVelocity.empty())cost+=.01*jointCost(velocity,oldVelocity);
                        oldVelocity=velocity;previous=best->joints;
                        for(const auto& c:candidates)addSafe(i,c);
                    }
                    Fields::Fitness f;f.rank=1000.*passed+20.*links-request.gaContinuityWeight*cost;
                    if(trial<int(guides.size()))fields.guide(gene,f);else fields.tell(f);
                    if(request.progress && trial%8==0)request.progress(nlohmann::json{{"type","genetic_field"},{"evaluation",trial+1},{"safe_points",passed},{"velocity_links",links},{"total_points",ik.layers.size()}}.dump());
                    if(passed==int(ik.layers.size()) && links==int(ik.layers.size())-1)break;
                }
                // Preserve each already authorized radius interval while connecting
                // safe candidates. Never silently revert a radius-found posture to
                // the nominal 100 mm, or expand a point's unlocked interval here.
                const auto bridgeProposals=[&](std::size_t i,std::size_t from,const std::vector<double>& q) {
                    const auto r=referenceFor(i),s=referenceFor(from);
                    const auto pose=fk(q);
                    const auto relative=s.orientation.conjugate()*Eigen::Quaterniond(pose.linear());
                    const auto transported=clampSurfacePosture(r.orientation,r.orientation*relative,r.angle);
                    std::vector<double> radii;
                    const auto addRadius=[&](double d){
                        d=spray && r.constrained?std::clamp(d,radiusLower[i],radiusUpper[i]):-1.;
                        if(std::none_of(radii.begin(),radii.end(),[&](double old){return std::abs(old-d)<1e-6;}))radii.push_back(d);
                    };
                    addRadius(mappedDistance(s,pose));addRadius(r.distance);
                    std::vector<CartesianIkCandidate> targets;
                    for(const auto& c:ik.layers[i].candidates)if(safeAt(c,i))targets.push_back(c);
                    std::stable_sort(targets.begin(),targets.end(),[&](const auto& a,const auto& b){return jointCost(a.joints,q)<jointCost(b.joints,q);});
                    std::vector<Eigen::Quaterniond> rotations{transported};
                    for(const auto& c:targets){
                        const auto target=fk(c.joints);const Eigen::Quaterniond rotation(target.linear());
                        if(std::any_of(rotations.begin(),rotations.end(),[&](const auto& old){return old.angularDistance(rotation)<1e-3;}))continue;
                        for(double fraction:{.25,.5,.75,1.})rotations.push_back(clampSurfacePosture(r.orientation,transported.slerp(fraction,rotation),r.angle));
                        addRadius(mappedDistance(r,target));
                        if(rotations.size()>=17)break;
                    }
                    if(spray && r.constrained && radiusUpper[i]-radiusLower[i]>1e-6)
                        for(int bin=0;bin<=8;++bin)addRadius(radiusLower[i]+(radiusUpper[i]-radiusLower[i])*bin/8.);
                    std::vector<std::pair<Eigen::Quaterniond,double>> out;
                    for(double radius:radii)for(const auto& rotation:rotations)out.emplace_back(rotation,radius);
                    return out;
                };
                // Search backwards as well: a narrow safe endpoint can require the
                // preceding point to move first. A forward-only repair cannot bridge
                // this case even though both endpoint poses are individually safe.
                if(!ProjectLayeredIkGraph::filter(ik,geometricGraph).success) {
                    std::vector<std::vector<double>> backwards;
                    for(const auto& c:ik.layers.back().candidates)if(safeAt(c,ik.layers.size()-1))backwards.push_back(c.joints);
                    for(std::size_t reverse=ik.layers.size()-1;reverse>0 && !backwards.empty() && !stop();--reverse) {
                        const std::size_t i=reverse-1;
                        const auto linkedBack=[&](const auto& q) {
                            for(const auto& b:backwards) {
                                double jump=0;for(std::size_t j=0;j<dimension;++j)jump=std::max(jump,std::abs(q[j]-b[j]));
                                if(jump<=request.maxJointDelta)return true;
                            }
                            return false;
                        };
                        std::vector<std::vector<double>> next;
                        for(const auto& c:ik.layers[i].candidates)if(safeAt(c,i) && linkedBack(c.joints))next.push_back(c.joints);
                        if(next.empty() && (i!=0 || request.startJoints.empty())) {
                            for(std::size_t k=0;k<std::min<std::size_t>(backwards.size(),4) && next.empty() && !stop();++k) {
                                const auto& q=backwards[k];
                                for(const auto& proposal:bridgeProposals(i,i+1,q)) {
                                    if(stop())break;
                                    for(const auto& c:solvePosture(i,proposal.first,q,false,proposal.second))if(linkedBack(c.joints)) {
                                        addSafe(i,c);next.push_back(c.joints);
                                    }
                                    if(!next.empty())break;
                                }
                            }
                            if(!next.empty())result.diagnostics.push_back({"ga_backward_posture_bridge","Added independently safe predecessor at original point "+std::to_string(i)+" to connect a narrow safe endpoint; interval collision remains unknown."});
                        }
                        backwards=std::move(next);
                    }
                }
                // Repair a SAFE endpoint that is disconnected from the reachable
                // branch too. The old GA only mutated colliding points and could
                // retain an isolated safe endpoint with a >0.5 rad jump. Transport
                // the reachable predecessor's relative surface posture, then solve
                // at the CURRENT fixed target; this is a seed, never a motion proof.
                std::vector<std::vector<double>> reachable;
                for(const auto& c:ik.layers.front().candidates)if(safeAt(c,0))reachable.push_back(c.joints);
                for(std::size_t i=1;i<ik.layers.size() && !stop() && !reachable.empty();++i) {
                    const auto linked=[&](const auto& q) {
                        for(const auto& a:reachable) {
                            double jump=0;for(std::size_t j=0;j<dimension;++j)jump=std::max(jump,std::abs(q[j]-a[j]));
                            if(jump<=request.maxJointDelta)return true;
                        }
                        return false;
                    };
                    std::vector<std::vector<double>> next;
                    for(const auto& c:ik.layers[i].candidates)if(safeAt(c,i) && linked(c.joints))next.push_back(c.joints);
                    if(next.empty()) {
                        for(std::size_t k=0;k<std::min<std::size_t>(reachable.size(),4) && next.empty() && !stop();++k) {
                            const auto& q=reachable[k];const auto r=referenceFor(i);
                            const auto relative=referenceFor(i-1).orientation.conjugate()*Eigen::Quaterniond(fk(q).linear());
                            const auto transported=clampSurfacePosture(r.orientation,r.orientation*relative,r.angle);
                            for(const auto& proposal:bridgeProposals(i,i-1,q)) {
                                if(stop())break;
                                for(const auto& c:solvePosture(i,proposal.first,q,false,proposal.second))if(linked(c.joints)) {
                                    addSafe(i,c);next.push_back(c.joints);
                                }
                                if(!next.empty())break;
                            }
                            if(next.empty()) {
                                // Safe pools can occupy two disconnected posture
                                // regions. Search locally around the reachable
                                // predecessor, rather than keeping a separately
                                // safe but unusable endpoint or weakening jump limits.
                                Population localPopulation(13007u+unsigned(i),X::Zero());
                                if(request.allowSprayAxisRotation)localPopulation.setPeriodicAxis(3);
                                const auto u=request.sprayAxis.unitOrthogonal(),v=request.sprayAxis.cross(u).eval();
                                for(int trial=0;trial<std::min(request.gaPointEvaluations,48+(request.gaGenerations+1)*Population::populationSize()) && next.empty() && !stop();++trial) {
                                    X gene=X::Zero();
                                    if(trial<48) {
                                        const double radius=trial<16?.08:trial<32?.18:.35;
                                        gene.head<3>()=radius*(std::cos((trial%16)*pi/8)*u+std::sin((trial%16)*pi/8)*v);
                                    } else gene=localPopulation.ask(1);
                                    const auto rotation=clampSurfacePosture(r.orientation,transported*decode(gene.head<4>().eval(),r),r.angle);
                                    Population::Fitness fitness;fitness.rank=-2;
                                    const double nominal=evolveRadius && r.constrained?nominalDistances[i]:r.distance;
                                    const double distance=spray && r.constrained?std::clamp(
                                        trial<48?mappedDistance(referenceFor(i-1),fk(q)):GaRadiusEvolution::distance(nominal,gene[4]),radiusLower[i],radiusUpper[i]):-1.;
                                    for(const auto& c:solvePosture(i,rotation,q,false,distance)) {
                                        double jump=std::numeric_limits<double>::infinity();
                                        for(const auto& a:reachable) {
                                            double d=0;for(std::size_t j=0;j<dimension;++j)d=std::max(d,std::abs(c.joints[j]-a[j]));
                                            jump=std::min(jump,d);
                                        }
                                        fitness.rank=std::max(fitness.rank,100.-100.*jump);
                                        addSafe(i,c);
                                        if(linked(c.joints))next.push_back(c.joints);
                                    }
                                    if(trial<48)localPopulation.guide(gene,fitness);else localPopulation.tell(fitness);
                                }
                            }
                        }
                        if(!next.empty())result.diagnostics.push_back({"ga_reachable_posture_bridge","Added safe surface-posture candidates at disconnected original point "+std::to_string(i)+"; full connecting motion remains for APF."});
                    }
                    reachable=std::move(next);
                }
                // An individually safe but isolated station also needs the FULL
                // population search. Previously it was skipped forever, leaving only
                // a handful of unscored local bridge trials. Wrapped distance is a
                // SEARCH objective only; real bounds and exact encoder connectivity
                // are enforced by the graph below and any prefix replay.
                std::vector<std::vector<double>> wrappedReachable;
                for(const auto& c:ik.layers.front().candidates)if(safeAt(c,0))wrappedReachable.push_back(c.joints);
                for(std::size_t i=1;i<ik.layers.size() && !wrappedReachable.empty() && !stop();++i) {
                    const auto collect=[&](){std::vector<std::vector<double>> out;
                        for(const auto& c:ik.layers[i].candidates)if(safeAt(c,i) && std::any_of(wrappedReachable.begin(),wrappedReachable.end(),[&](const auto& q){return wrappedJump(q,c.joints)<=request.maxJointDelta;}))out.push_back(c.joints);
                        return out;
                    };
                    auto next=collect();
                    if(next.empty()) {
                        if(const auto failure=searchPoint(i,wrappedReachable))return *failure;
                        next=collect();
                        result.diagnostics.push_back({"ga_connected_population_repair","Configured full initial-pool/generation schedule also searched disconnected safe station "+std::to_string(i)+"; only connected-safe postures freeze radius evolution. Exact winding, velocity and APF checks still required."});
                    }
                    wrappedReachable=std::move(next);
                }
                // Drop rejected roots now; every remaining pool member was independently
                // checked above. GA graph edges enforce jumps and velocity, not collision
                // between knots. APF and its independent full-path gate own that certificate.
                for(std::size_t i=0;i<ik.layers.size();++i) {
                    auto& layer=ik.layers[i];std::vector<CartesianIkCandidate> safe;
                    for(const auto& c:layer.candidates)if(safeAt(c,i))safe.push_back(c);
                    layer.candidates=std::move(safe);result.candidateCounts[i]=layer.candidates.size();
                    result.candidateRejections[i].assign(layer.candidates.size(),std::string());result.candidateDistances[i].clear();
                    for(const auto& c:layer.candidates){const auto d=gaDistance(c.joints);result.candidateDistances[i].push_back(d.valid?d.distanceMeters:std::numeric_limits<double>::quiet_NaN());}
                }
                if(!request.startJoints.empty() && !ProjectLayeredIkGraph::filter(ik,geometricGraph).success && !stop()) {
                    // A safe posture pool may be disconnected because a preceding
                    // scan chose an unfortunate multi-turn encoder branch. Enumerate
                    // equivalent roots inside the REAL bounds, but never silently move
                    // the locked start. The caller must rebase and independently replay
                    // its ENTIRE accepted prefix before resubmitting this seed.
                    auto lifted=ik;
                    for(std::size_t i=0;i<lifted.layers.size() && !stop();++i) {
                        auto& output=lifted.layers[i].candidates;output.clear();
                        for(const auto& root:ik.layers[i].candidates) {
                            const auto originalPose=fk(root.joints);auto candidate=root;
                            std::function<void(std::size_t)> enumerate;
                            enumerate=[&](std::size_t j) {
                                if(stop() || output.size()>=4096)return;
                                if(j<dimension) {
                                    const int first=std::max(-1,int(std::ceil((ikOptions.lower[j]-root.joints[j]-1e-10)/(2*pi))));
                                    const int last=std::min(1,int(std::floor((ikOptions.upper[j]-root.joints[j]+1e-10)/(2*pi))));
                                    for(int turn=first;turn<=last;++turn) {
                                        candidate.joints[j]=std::clamp(root.joints[j]+2*pi*turn,ikOptions.lower[j],ikOptions.upper[j]);
                                        candidate.turns[j]=int(std::floor((candidate.joints[j]+pi)/(2*pi)));enumerate(j+1);
                                    }
                                    return;
                                }
                                if(std::any_of(output.begin(),output.end(),[&](const auto& c){double d=0;for(std::size_t k=0;k<dimension;++k)d=std::max(d,std::abs(c.joints[k]-candidate.joints[k]));return d<1e-8;}))return;
                                const auto actual=fk(candidate.joints);
                                if((actual.translation()-originalPose.translation()).norm()>1e-8 ||
                                    Eigen::Quaterniond(actual.linear()).angularDistance(Eigen::Quaterniond(originalPose.linear()))>1e-8)return;
                                if(safeAt(candidate,i))output.push_back(candidate);
                            };
                            enumerate(0);
                        }
                    }
                    auto windingGraph=graph;
                    windingGraph.nodeAdmissible=[&](std::size_t i,const auto& c){return safeAt(c,i);};
                    windingGraph.nodeCost=[&](std::size_t i,const auto& c){
                        if(i)return 0.;double d=0;for(std::size_t j=0;j<dimension;++j)d+=std::pow(c.joints[j]-request.startJoints[j],2);return d;
                    };
                    auto windingPath=ProjectLayeredIkGraph::filter(lifted,windingGraph);
                    if(!windingPath.success) {
                        windingGraph.maxJointVelocities.assign(dimension,1e6);
                        windingPath=ProjectLayeredIkGraph::filter(lifted,windingGraph);
                    }
                    if(windingPath.success && !stop()) {
                        nlohmann::json seed=nlohmann::json::array(),poses=nlohmann::json::array();
                        std::vector<double> offset(dimension);
                        const auto& first=lifted.layers.front().candidates[windingPath.paths.front().selections.front()];
                        for(std::size_t j=0;j<dimension;++j)offset[j]=first.joints[j]-request.startJoints[j];
                        for(std::size_t i=0;i<lifted.layers.size();++i) {
                            const auto& layer=lifted.layers[i];const auto& c=layer.candidates[windingPath.paths.front().selections[i]];
                            const auto world=fk(c.joints);const auto local=worldFromFrame.inverse()*world;const Eigen::Quaterniond rotation(local.linear());
                            seed.push_back({{"time_s",layer.time},{"q_rad",c.joints}});
                            poses.push_back({{"time_s",layer.time},{"position_m",{local.translation().x(),local.translation().y(),local.translation().z()}},
                                {"quaternion_xyzw",{rotation.x(),rotation.y(),rotation.z(),rotation.w()}},
                                {"spray_distance_mm",spray?mappedDistance(referenceFor(i),world)*1000:0.}});
                        }
                        if(std::all_of(offset.begin(),offset.end(),[](double v){return std::abs(v)<1e-8;})) {
                            ik=std::move(lifted);
                            for(std::size_t i=0;i<ik.layers.size();++i) {
                                result.candidateCounts[i]=ik.layers[i].candidates.size();
                                result.candidateRejections[i].assign(result.candidateCounts[i],std::string());
                                result.candidateDistances[i].assign(result.candidateCounts[i],std::numeric_limits<double>::quiet_NaN());
                            }
                            result.diagnostics.push_back({"ga_winding_branches_connected","Added independently checked equivalent encoder branches inside actual bounds; locked start unchanged. Normal graph/timing checks and subsequent APF remain required."});
                        } else {
                            result.diagnostics.push_back({"ga_prefix_winding_required",nlohmann::json{
                                {"start_offset_rad",offset},{"joint_points",seed},{"cartesian_points_request_frame",poses},
                                {"acceptance","NOT ACCEPTED. Replay the entire shifted prefix under unchanged constraints, then resubmit these poses and joints to GA and its independent point gate. Interior motion still requires APF."}}.dump()});
                        }
                    }
                }
                result.diagnostics.push_back({"genetic_point_gate","GA candidate points passed actual-model IK, process, collision, baffle and singularity checks; interval collision remains UNKNOWN until APF."});
            }
            if(request.returnCandidates)result.candidateLayers=ik.layers;
            graph.edgeAdmissible=[&](std::size_t index,const CartesianIkCandidate& a,const CartesianIkCandidate& b) {
                if(request.controlPointsOnly) return true;
                double delta=0;
                for(std::size_t j=0;j<dimension;++j) { delta=std::max(delta,std::abs(b.joints[j]-a.joints[j])); }
                const double steps=std::max(2.0,std::ceil(delta/request.validationJointStep));
                if(steps>double(request.maxSamplesPerEdge)) { ++result.rejectedEdges; return false; }
                const auto& from=source.cartesianControlPoints.points[index].tcpPose;
                const auto& to=source.cartesianControlPoints.points[index+1].tcpPose;
                for(std::size_t s=1;s<static_cast<std::size_t>(steps);++s) {
                    const double t=double(s)/steps;
                    std::vector<double> q(dimension);
                    for(std::size_t j=0;j<dimension;++j) { q[j]=a.joints[j]+t*(b.joints[j]-a.joints[j]); }
                    if(!stateValid(q)) { reportFailure(index,stateFailure); ++result.rejectedEdges; return false; }
                    if(!request.pipelineStage.empty()) {
                        const double time=(1-t)*ik.layers[index].time+t*ik.layers[index+1].time;
                        if(!processValid(q,time)){reportFailure(index,"process_constraint_violation");++result.rejectedEdges;return false;}
                        continue;
                    }
                    const auto actual=fk(q);
                    const Eigen::Quaterniond target=Eigen::Quaterniond(from.linear()).slerp(t,Eigen::Quaterniond(to.linear()));
                    if((actual.translation()-((1-t)*from.translation()+t*to.translation())).norm()>request.maximumTcpPositionDeviation ||
                        Eigen::Quaterniond(actual.linear()).angularDistance(target)>request.maximumTcpOrientationDeviation) {
                        reportFailure(index,"tcp_interpolation_deviation"); ++result.rejectedEdges; return false;
                    }
                }
                return true;
            };
            auto paths=ProjectLayeredIkGraph::filter(ik,graph);
            result.continuousPaths=paths.paths;
            if((request.pipelineStage=="ga" || request.pipelineStage=="posture") && !paths.success && !stop()) {
                // Diagnose insufficient source durations without accepting them or
                // relaxing the production validator. The caller may retime and
                // submit the exact selected poses again under the original limits.
                auto timingGraph=graph;timingGraph.maxJointVelocities.assign(dimension,1e6);
                const auto slower=ProjectLayeredIkGraph::filter(ik,timingGraph);
                if(slower.success && ProjectTrajectoryInverseKinematics::selectMultiIkTrajectory(
                    ik,slower.paths.front().selections,result.plan,error)) {
                    nlohmann::json intervals=nlohmann::json::array();
                    const auto& points=result.plan.trajectory.points;
                    for(std::size_t i=1;i<points.size();++i) {
                        double duration=points[i].time-points[i-1].time;
                        for(std::size_t j=0;j<dimension;++j)duration=std::max(duration,
                            1.05*std::abs(points[i].q[j]-points[i-1].q[j])/graph.maxJointVelocities[j]);
                        intervals.push_back(duration);
                    }
                    result.diagnostics.push_back({request.pipelineStage=="posture"?"posture_velocity_retime_required":"ga_velocity_retime_required",nlohmann::json{{"interval_seconds",intervals},
                        {"acceptance","FAILED at source timing; exact poses require resubmission and fresh validation"}}.dump()});
                    recordVelocities();
                }
            }
            if(request.pipelineStage=="apf" && paths.success && !stop()) {
                // Standard APF termination: an already admissible connected input needs
                // zero repulsive iterations. Graph validation checked all knots, timed
                // interpolated motions, process constraints, singularity and velocities.
                // Avoid repeated expensive distance-field construction for a clear path;
                // the external whole-path gate still replays these exact joints afresh.
                if(!ProjectTrajectoryInverseKinematics::selectMultiIkTrajectory(ik,paths.paths.front().selections,result.plan,error))return fail("selection_failed",error);
                result.success=true;recordVelocities();
                result.diagnostics.push_back({"apf_input_path_validated","Complete connected input already passes sampled collision, baffle, process and velocity checks; APF terminates with zero repulsive steps. Independent full replay remains required before CDF."});
                return finish();
            }
            if(request.optimizeAvoidance && !request.controlPointsOnly && !stop()) {
                // Keep every requested detector (self, workpiece, tools, fixtures, environment).
                // If no collision-free graph path exists, use continuous IK paths as repair seeds.
                auto seeds=paths;
                if(!seeds.success) {
                    auto relaxed=graph; relaxed.maxPaths=request.optimizationSeedPaths;
                    relaxed.nodeAdmissible=[&](std::size_t,const CartesianIkCandidate& c){return stateValid(c.joints,false);};
                    relaxed.edgeAdmissible={};
                    seeds=ProjectLayeredIkGraph::filter(ik,relaxed);
                    if(!seeds.success && request.pipelineStage=="apf") {
                        // Cartesian densification can expose a faster nonlinear IK
                        // variation than its original endpoint chord. Diagnose time
                        // only; the caller must extend original-edge durations and
                        // resubmit under the unchanged limits before planning.
                        auto timing=relaxed;timing.maxJointVelocities.assign(dimension,1e6);
                        const auto slower=ProjectLayeredIkGraph::filter(ik,timing);
                        if(slower.success && ProjectTrajectoryInverseKinematics::selectMultiIkTrajectory(
                            ik,slower.paths.front().selections,result.plan,error)) {
                            nlohmann::json intervals=nlohmann::json::array();
                            const auto& points=result.plan.trajectory.points;
                            for(std::size_t i=1;i<points.size();++i) {
                                double duration=points[i].time-points[i-1].time;
                                for(std::size_t j=0;j<dimension;++j)duration=std::max(duration,
                                    1.05*std::abs(points[i].q[j]-points[i-1].q[j])/graph.maxJointVelocities[j]);
                                intervals.push_back(duration);
                            }
                            result.diagnostics.push_back({"apf_velocity_retime_required",nlohmann::json{{"interval_seconds",intervals},
                                {"acceptance","FAILED at supplied timing; no interior collision certificate; resubmit after retiming"}}.dump()});
                            recordVelocities();
                            return fail("apf_timing_required","Densified joint seed needs longer durations; collision and velocity checks remain mandatory.");
                        }
                    }
                    if(!seeds.success)result.diagnostics.push_back({"apf_seed_graph_unconnected",seeds.message});
                }
                const auto poseAt=[&](double time) -> Eigen::Isometry3d {
                    const auto& points=source.cartesianControlPoints.points;
                    auto upper=std::upper_bound(points.begin(),points.end(),time,[](double t,const auto& p){return t<p.time;});
                    if(upper==points.begin()) return points.front().tcpPose;
                    if(upper==points.end()) return points.back().tcpPose;
                    const auto& a=*(upper-1); const auto& b=*upper;
                    const double t=(time-a.time)/(b.time-a.time);
                    Eigen::Isometry3d pose=Eigen::Isometry3d::Identity();
                    pose.translation()=(1-t)*a.tcpPose.translation()+t*b.tcpPose.translation();
                    pose.linear()=Eigen::Quaterniond(a.tcpPose.linear()).slerp(t,Eigen::Quaterniond(b.tcpPose.linear())).toRotationMatrix();
                    return pose;
                };
                const auto sampleValid=[&](const std::vector<double>& q,double time) {
                    if(!request.pipelineStage.empty()) {
                        stateFailure.clear();stateFailureDetail.clear();
                        if(!stateValid(q))return false;
                        if(!processValid(q,time)) {
                            stateFailure="process_constraint_violation";
                            stateFailureDetail=spray?surfaceAudit(q,time).dump():"TCP corridor or posture limit";return false;
                        }
                        return true;
                    }
                    if(!stateValid(q) || !sprayAdmissible(q,time)) return false;
                    const auto actual=fk(q); const auto target=poseAt(time);
                    if(spray) {
                        const auto r=surfaceAt(time);
                        if(r.constrained && ((actual.translation()+r.distance*actual.linear()*request.sprayAxis-r.anchor).norm()>request.maximumTcpPositionDeviation ||
                            r.orientation.angularDistance(Eigen::Quaterniond(actual.linear()))>r.angle+request.maximumTcpOrientationDeviation))return false;
                    }
                    return (actual.translation()-target.translation()).norm()<=request.maximumTcpPositionDeviation &&
                        Eigen::Quaterniond(actual.linear()).angularDistance(Eigen::Quaterniond(target.linear()))<=request.maximumTcpOrientationDeviation;
                };
                const auto repairedValid=[&](const robottrajectory::JointTrajectory& trajectory) {
                    warmCollision(trajectory);
                    const auto& points=trajectory.points;
                    const auto reject=[&](std::size_t index,const std::string& reason) {
                        result.diagnostics.push_back({"optimizer_replay_rejected",nlohmann::json{{"point_index",index},{"reason",reason},{"detail",stateFailureDetail}}.dump()});
                        return false;
                    };
                    bool timingRequired=false;
                    if(points.size()<2 || std::abs(points.front().time-source.cartesianControlPoints.points.front().time)>1e-8 ||
                        std::abs(points.back().time-source.cartesianControlPoints.points.back().time)>1e-8) return reject(0,"source time boundary changed");
                    for(std::size_t i=0;i<points.size();++i) {
                        if(stop() || points[i].q.size()!=dimension || !std::isfinite(points[i].time))return reject(i,"cancelled or invalid joint/time sample");
                        if(!sampleValid(points[i].q,points[i].time))return reject(i,"invalid knot: "+stateFailure+" or process constraint");
                        if(i==0) continue;
                        const auto& a=points[i-1]; const auto& b=points[i];
                        const double dt=b.time-a.time; if(dt<=0) return reject(i,"nonpositive duration");
                        double delta=0;
                        for(std::size_t j=0;j<dimension;++j) {
                            const double dq=std::abs(b.q[j]-a.q[j]);
                            if(dq>request.maxJointDelta)return reject(i,"joint jump exceeds configured limit");
                            if(dq>graph.maxJointVelocities[j]*dt) {
                                if(request.pipelineStage!="apf")return reject(i,"joint velocity exceeds configured limit");
                                timingRequired=true;
                            }
                            delta=std::max(delta,dq);
                        }
                        const double steps=std::max(2.0,std::ceil(delta/request.validationJointStep));
                        if(!std::isfinite(steps) || steps>double(request.maxSamplesPerEdge)) return reject(i,"motion sample budget exceeded");
                        for(std::size_t s=1;s<static_cast<std::size_t>(steps);++s) {
                            const double t=double(s)/steps; std::vector<double> q(dimension);
                            for(std::size_t j=0;j<dimension;++j) q[j]=a.q[j]+t*(b.q[j]-a.q[j]);
                            if(!sampleValid(q,a.time+t*dt)) return reject(i,"invalid interior: "+stateFailure+" or process constraint");
                        }
                    }
                    if(timingRequired) {
                        // Finish geometry/process verification FIRST. A partial
                        // speed recommendation for an invalid path caused repeated
                        // expensive APF runs without retaining a useful trajectory.
                        const auto& input=source.cartesianControlPoints.points;
                        std::vector<double> scales(input.size()-1,1.);
                        for(std::size_t n=1;n<points.size();++n) {
                            const double step=points[n].time-points[n-1].time;
                            double scale=1.;for(std::size_t joint=0;joint<dimension;++joint)
                                scale=std::max(scale,1.05*std::abs(points[n].q[joint]-points[n-1].q[joint])/(step*graph.maxJointVelocities[joint]));
                            for(std::size_t edge=0;edge<scales.size();++edge)
                                if(input[edge].time<points[n].time-1e-9 && input[edge+1].time>points[n-1].time+1e-9)scales[edge]=std::max(scales[edge],scale);
                        }
                        nlohmann::json intervals=nlohmann::json::array(),seed=nlohmann::json::array();
                        for(std::size_t edge=0;edge<scales.size();++edge)intervals.push_back((input[edge+1].time-input[edge].time)*scales[edge]);
                        for(const auto& point:points)seed.push_back({{"time_s",point.time},{"q_rad",point.q}});
                        result.diagnostics.push_back({"apf_velocity_retime_required",nlohmann::json{{"interval_seconds",intervals},
                            {"geometrically_checked_joint_points",seed},
                            {"acceptance","NOT ACCEPTED at source timing. All knots and chords passed geometry/process checks; retime and independently replay exact returned joints under original limits."}}.dump()});
                        return reject(0,"geometry/process checked; exact path requires velocity retiming");
                    }
                    return true;
                };
                for(const auto& path:seeds.paths) {
                    if(stop()) break;
                    StoredMotionPlan seed;
                    if(!ProjectTrajectoryInverseKinematics::selectMultiIkTrajectory(ik,path.selections,seed,error)) continue;
                    if(seed.trajectory.points.size()<2) break;
                    ProjectCdfQpRepairOptions options=request.cdfOptions;
                    options.detectorIds=request.collisionDetectorIds;
                    options.detectorId.clear(); options.obstacleId.clear(); options.obstacleRobotSourcePath.clear();
                    options.worldForwardKinematics=fk; options.cancelled=stop;
                    if(!request.pipelineStage.empty()) {
                        options.stage=request.pipelineStage=="apf"?ProjectCdfQpRepairOptions::Stage::ApfOnly:ProjectCdfQpRepairOptions::Stage::CdfOnly;
                        options.densifyKnots=request.densifyOptimizationKnots;
                        if(request.pipelineStage=="apf" && spray && !paths.success && !options.densifyKnots) {
                            // Disabling optional Cartesian pre-densification must not
                            // prevent the planner from adding necessary connection nodes.
                            // Long raw joint chords may violate a fixed surface station
                            // even when their endpoints and mesh collision tests pass.
                            // Native nodes are projected through their original surface
                            // correspondence and every original knot remains present.
                            options.densifyKnots=true;
                            options.segmentIntermediateSamples=0;
                            result.diagnostics.push_back({"apf_connection_support_nodes",
                                "No admissible raw joint chord path. APF adds joint-step-based connection nodes, maps each through its fixed surface station, and retains every original knot; optional Cartesian pre-densification remains disabled."});
                        }
                    }
                    if(spray) options.surfaceProject=[&](std::vector<double>& q,std::size_t edge,double fraction) {
                        if(stop() || edge>=seed.trajectory.points.size())return false;
                        const auto j=std::min(edge+1,seed.trajectory.points.size()-1);
                        const double time=(1-fraction)*seed.trajectory.points[edge].time+fraction*seed.trajectory.points[j].time;
                        const auto r=surfaceAt(time);if(!r.constrained)return true;
                        // Only the proposed ORIENTATION is retained. Position is rebuilt at its
                        // immutable surface station before IK, collision scoring or acceptance.
                        Eigen::Quaterniond adjusted(fk(q).linear());
                        adjusted=clampSurfacePosture(r.orientation,adjusted,std::max(0.,r.angle-1e-5));
                        Eigen::Isometry3d target=Eigen::Isometry3d::Identity();target.linear()=adjusted.toRotationMatrix();
                        target.translation()=r.anchor-r.distance*target.linear()*request.sprayAxis;
                        auto local=source;auto point=source.cartesianControlPoints.points.front();point.time=time;point.tcpPose=target;
                        local.cartesianControlPoints.points={point};
                        auto projection=ikOptions;projection.cachedCandidates.clear();projection.preferredJoints={q};projection.model.seedJoints=q;
                        projection.seedCount=1;projection.maxIterations=80;projection.progress={};projection.layerReady={};
                        if(serialIkReady){projection.model.worldForwardKinematics=[&](const auto& v){return gpuIk.forward(v);};
                            projection.model.worldJacobian=[&](const auto& v){return gpuIk.jacobian(v);};}
                        const auto solved=ProjectTrajectoryInverseKinematics::solveAllCartesianControlPoints(local,projection);
                        if(solved.layers.empty() || solved.layers.front().candidates.empty())return false;
                        q=solved.layers.front().candidates.front().joints;
                        const auto actual=fk(q);
                        return stateValid(q,false) && processValid(q,time) &&
                            (actual.translation()-target.translation()).norm()<=request.maximumTcpPositionDeviation &&
                            Eigen::Quaterniond(actual.linear()).angularDistance(adjusted)<=request.maximumTcpOrientationDeviation &&
                            (actual.translation()+r.distance*actual.linear()*request.sprayAxis-r.anchor).norm()<=request.maximumTcpPositionDeviation;
                    };
                    const auto refineProcess=[&](robottrajectory::JointTrajectory& supported) {
                        if(supported.points.size()<2)return false;
                        // Joint step alone does not bound nonlinear TCP or
                        // normal-cone error. Refine only offending process
                        // chords through their immutable surface stations.
                        const auto coarse=supported.points;supported.points.clear();supported.points.push_back(coarse.front());bool refined=true;
                        using JointPoint=robottrajectory::TimedJointPoint;
                        std::function<bool(const JointPoint&,const JointPoint&,int)> refine;
                        refine=[&](const JointPoint& a,const JointPoint& b,int depth) {
                            if(stop() || supported.points.size()>request.maxSamplesPerEdge*seed.trajectory.points.size())return false;
                            double delta=0;for(std::size_t j=0;j<dimension;++j)delta=std::max(delta,std::abs(b.q[j]-a.q[j]));
                            const auto steps=static_cast<std::size_t>(std::max(2.,std::ceil(delta/request.validationJointStep)));
                            bool valid=true;
                            for(std::size_t s=1;s<steps && valid;++s) {
                                const double f=double(s)/steps;auto q=a.q;for(std::size_t j=0;j<dimension;++j)q[j]+=f*(b.q[j]-a.q[j]);
                                valid=processValid(q,a.time+f*(b.time-a.time));
                            }
                            if(valid){supported.points.push_back(b);return true;}
                            if(depth>=8) {
                                result.diagnostics.push_back({"apf_surface_refinement_failed",nlohmann::json{{"reason","subdivision limit"},{"from_time_s",a.time},{"to_time_s",b.time},{"depth",depth}}.dump()});
                                return false;
                            }
                            auto middle=a;middle.time=(a.time+b.time)/2;
                            for(std::size_t j=0;j<dimension;++j)middle.q[j]=(a.q[j]+b.q[j])/2;
                            const auto& input=seed.trajectory.points;
                            const auto upper=std::upper_bound(input.begin(),input.end(),middle.time,[](double t,const auto& p){return t<p.time;});
                            const auto edge=std::min(input.size()-2,std::size_t(upper-input.begin()-1));
                            const double f=(middle.time-input[edge].time)/(input[edge+1].time-input[edge].time);
                            if(!options.surfaceProject(middle.q,edge,f)) {
                                result.diagnostics.push_back({"apf_surface_refinement_failed",nlohmann::json{{"reason","surface IK projection"},{"time_s",middle.time},{"q_rad",middle.q},{"audit",surfaceAudit(middle.q,middle.time)}}.dump()});
                                return false;
                            }
                            return refine(a,middle,depth+1) && refine(middle,b,depth+1);
                        };
                        for(std::size_t i=1;i<coarse.size() && refined;++i)refined=refine(coarse[i-1],coarse[i],0);
                        return refined;
                    };
                    if(request.pipelineStage=="apf" && spray && !paths.success) {
                        // Try the projected connection before constructing a distance
                        // field. A GA-cleared path often only needs surface-correct
                        // support nodes, with no repulsive iterations. Every resulting
                        // knot AND chord is replayed below with the normal checks.
                        auto supported=seed.trajectory;supported.points.clear();
                        supported.points.push_back(seed.trajectory.points.front());bool projected=true;
                        for(std::size_t i=1;i<seed.trajectory.points.size() && projected && !stop();++i) {
                            const auto& a=seed.trajectory.points[i-1];const auto& b=seed.trajectory.points[i];
                            double delta=0;for(std::size_t j=0;j<dimension;++j)delta=std::max(delta,std::abs(b.q[j]-a.q[j]));
                            const auto edges=seed.trajectory.points.size()-1;
                            
                            const auto budget=request.preferredOutputWaypoints?std::max(edges,request.preferredOutputWaypoints-1):0;
                            // Balanced original intervals: counts differ by at most one.
                            // The independent collision/process sampling step is unchanged.
                            const auto count=request.preferredOutputWaypoints?
                                budget/edges+((i* (budget%edges))/edges-((i-1)*(budget%edges))/edges):
                                static_cast<std::size_t>(std::max(2.,std::ceil(delta/request.validationJointStep)));
                            if(count>request.maxSamplesPerEdge){projected=false;break;}
                            for(std::size_t s=1;s<count;++s) {
                                if(stop()){projected=false;break;}
                                const double fraction=double(s)/count;auto point=a;point.time=a.time+fraction*(b.time-a.time);
                                for(std::size_t j=0;j<dimension;++j)point.q[j]=a.q[j]+fraction*(b.q[j]-a.q[j]);
                                if(!options.surfaceProject(point.q,i-1,fraction)){projected=false;break;}
                                supported.points.push_back(std::move(point));
                            }
                            supported.points.push_back(b);
                        }
                        if(projected && !stop()) {
                            projected=refineProcess(supported);
                            // Preserve the refined curve, then redistribute its nodes over
                            // original intervals. Add nodes only when process chords need them.
                            for(int pass=0;projected && request.preferredOutputWaypoints && pass<4 && !stop();++pass) {
                                const auto curve=supported.points;const auto& original=seed.trajectory.points;
                                std::vector<std::size_t> counts(original.size()-1,0);
                                for(std::size_t k=1;k<curve.size();++k){const auto upper=std::lower_bound(original.begin()+1,original.end(),curve[k].time-1e-9,[](const auto& v,double t){return v.time<t;});
                                    ++counts[std::min(counts.size()-1,std::size_t(upper-original.begin()-1))];}
                                const auto bounds=std::minmax_element(counts.begin(),counts.end());
                                if(*bounds.second-*bounds.first<=1)break;
                                const auto divisions=*bounds.second;
                                supported.points={curve.front()};
                                for(std::size_t edge=0;edge<counts.size() && projected;++edge){
                                    for(std::size_t s=1;s<divisions;++s){const double f=double(s)/divisions;auto point=original[edge];point.time=(1-f)*original[edge].time+f*original[edge+1].time;
                                        const auto upper=std::upper_bound(curve.begin(),curve.end(),point.time,[](double t,const auto& v){return t<v.time;});
                                        const auto& a=*(upper-1);const auto& b=*upper;const double u=(point.time-a.time)/(b.time-a.time);
                                        for(std::size_t j=0;j<dimension;++j)point.q[j]=(1-u)*a.q[j]+u*b.q[j];
                                        if(!options.surfaceProject(point.q,edge,f)){projected=false;break;}
                                        supported.points.push_back(std::move(point));}
                                    supported.points.push_back(original[edge+1]);
                                }
                                if(projected)projected=refineProcess(supported);
                            }
                        }
                        const auto diagnosticStart=result.diagnostics.size();
                        if(projected && !stop() && repairedValid(supported)) {
                            if(request.preferredOutputWaypoints)result.diagnostics.push_back({"balanced_output_nodes",nlohmann::json{{"points",supported.points.size()},{"budget",request.preferredOutputWaypoints},{"original_intervals",seed.trajectory.points.size()-1},{"safety_joint_step_rad",request.validationJointStep}}.dump()});
                            result.plan=source;result.plan.trajectory=std::move(supported);result.success=true;recordVelocities();
                            result.diagnostics.push_back({"apf_projected_input_path_validated",
                                "Surface-mapped support path passed fresh knot, interior collision/baffle, process, singularity, jump and velocity checks. APF terminates with zero repulsive iterations; independent whole-path replay remains required."});
                            return finish();
                        }
                        if(std::any_of(result.diagnostics.begin()+diagnosticStart,result.diagnostics.end(),[](const auto& d){return d.code=="apf_velocity_retime_required";}))
                            return fail("apf_timing_required","Projected support path needs longer durations; resubmit under unchanged limits.");
                    }
                    options.previewState=[&](const std::vector<double>& q) {
                        if(request.progress) request.progress(nlohmann::json{{"type","preview_state"},{"q_rad",q},
                            {"validation","search_candidate_not_accepted"}}.dump());
                    };
                    options.progress=[&](const std::string& message){
                        if(stop()) throw std::runtime_error("Planning cancelled.");
                        if(request.progress) request.progress(message);
                    };
                    options.apfMaxTcpDeviation=request.maximumTcpPositionDeviation;
                    if(!request.pipelineStage.empty() && spray) {
                        const double distance=*std::max_element(request.sprayDistances.begin(),request.sprayDistances.end());
                        options.apfMaxTcpDeviation+=2*distance*std::sin(request.pointSearchMaxAngleDeg*std::acos(-1.)/360);
                        // Surface correspondence, including interpolated samples, remains authoritative.
                        options.tcpStateAdmissible=[&](const std::vector<double>& q,const Eigen::Vector3d&) {return stateValid(q,false);};
                    }
                    options.safetyMargin=request.minimumClearance; options.targetClearance=0;
                    options.distanceThreshold=std::numeric_limits<double>::max();
                    options.maxIterations=request.optimizationIterations; options.queryWorkers=0;
                    options.retimeOutput=false;
                    options.validationMaxJointStep=request.validationJointStep;
                    auto repaired=ProjectCdfQpTrajectoryRepairService{}.repair(privateDocument,basePath,
                        source.robotId,source.jointNames,seed.trajectory,options);
                    for(int surfacePass=0;surfacePass<4 && !stop();++surfacePass) {
                        for(const auto& diagnostic:repaired.diagnostics)result.diagnostics.push_back(diagnostic);
                        if(!repaired.success)break;
                        const auto replayStart=result.diagnostics.size();
                        if(request.pipelineStage=="apf" && spray) {
                            auto refined=repaired.plan.trajectory;
                            if(!refineProcess(refined))break;
                            repaired.plan.trajectory=std::move(refined);
                        }
                        if(repairedValid(repaired.plan.trajectory)) {
                            result.plan=source; result.plan.trajectory=repaired.plan.trajectory; result.success=true;recordVelocities();
                            result.diagnostics.push_back({"native_optimizer_statistics",nlohmann::json{
                                {"iterations",repaired.statistics.iterations},{"qp_iterations",repaired.statistics.qpIterations},
                                {"accepted_qp_steps",repaired.statistics.acceptedQpSteps},
                                {"initial_minimum_phi_m",repaired.statistics.initialMinimumPhi},
                                {"final_minimum_phi_m",repaired.statistics.finalMinimumPhi},
                                {"invalid_segments",repaired.statistics.invalidSegmentCount}}.dump()});
                            result.diagnostics.push_back({request.pipelineStage.empty()?"avoidance_optimized":request.pipelineStage+"_stage_validated",
                                "Requested native stage passed independent collision, baffle, singularity, velocity and timed process validation."});
                            return finish();
                        }
                        if(request.pipelineStage!="apf" || !spray || surfacePass==3 ||
                            std::any_of(result.diagnostics.begin()+replayStart,result.diagnostics.end(),[](const auto& d){return d.code=="apf_velocity_retime_required";}))break;
                        // Projecting a chord onto its fixed surface manifold can
                        // introduce a collision at a NEW support node. Feed those
                        // real nodes back to APF, rather than declaring its earlier
                        // mesh-only success to be the final process certificate.
                        seed.trajectory=repaired.plan.trajectory;
                        result.diagnostics.push_back({"apf_surface_collision_repair",nlohmann::json{{"pass",surfacePass+2},{"knots",seed.trajectory.points.size()},
                            {"acceptance","pending; rerun APF after surface support refinement, then independently check geometry/process/speed again"}}.dump()});
                        if(request.progress)request.progress("APF surface-support repair pass "+std::to_string(surfacePass+2));
                        repaired=ProjectCdfQpTrajectoryRepairService{}.repair(privateDocument,basePath,
                            source.robotId,source.jointNames,seed.trajectory,options);
                    }
                    result.diagnostics.push_back({"avoidance_attempt_rejected","Repair failed or independent timed TCP/singularity/velocity validation rejected its output."});
                }
                if(!request.pipelineStage.empty())return fail(request.pipelineStage=="apf"?"apf_stage_failed":"cdf_stage_failed",
                    "Requested stage did not produce an independently admissible path; no implicit stage fallback.");
                // A valid original graph path remains a safe fallback if optimization cannot improve it.
            }
            if(!paths.success) { return fail(paths.cancelled?"cancelled":"no_valid_candidate_path",paths.message,!paths.cancelled); }
            if(stop()) { return fail("cancelled","Evaluation cancelled."); }
            if(!ProjectTrajectoryInverseKinematics::selectMultiIkTrajectory(ik,paths.paths.front().selections,result.plan,error)) {
                return fail("selection_failed",error);
            }
            if(request.pipelineStage=="ga")for(std::size_t i=0;i<result.plan.trajectory.points.size();++i)
                result.plan.cartesianControlPoints.points[i].tcpPose=fk(result.plan.trajectory.points[i].q);
            result.success=true;recordVelocities();
            result.diagnostics.push_back({request.controlPointsOnly?"control_points_valid":"sampled_path_valid",
                request.controlPointsOnly?"Only original control points checked; segment interiors remain unchecked.":
                "Actual-model IK and sampled collision/singularity/TCP checks passed. Acceleration, controller dynamics and continuous collision are not certified."});
            result.diagnostics.push_back({"ik_search_scope",ik.message});
            return finish();
        } catch(const std::exception& e) { return fail("evaluation_failed",e.what()); }
    }
}
