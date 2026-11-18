#include <ProjectMotionPlanning/GaApfCdfPipeline.h>
#include <ProjectMotionPlanning/CartesianMotion.h>
#include <ProjectMotionPlanning/CartesianPathRefinement.h>
#include <ProjectMotionPlanning/TrajectoryImport.h>
#include <ProjectMotionPlanning/CdfJointAngleImport.h>
#include <ProjectMotionPlanning/TrajectoryControlPointEditing.h>
#include <SimulationProject/ProjectIO.h>
#include <RobotInstance/RobotInstance.h>
#include <nlohmann/json.hpp>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <chrono>
#include <algorithm>
#include <cmath>
#include "../../src/CdfQueryBatch.h"
#include "../../src/GpuSerialIk.h"
#include "../../src/ApfLocalPlanner.h"

using namespace motion_planning;
using Json=nlohmann::json;
using Clock=std::chrono::steady_clock;
static double seconds(Clock::time_point began) {return std::chrono::duration<double>(Clock::now()-began).count();}
static void write(const std::filesystem::path& path,const Json& data) {
    std::ofstream stream(path);if(!stream)throw std::runtime_error("Cannot write result file");stream<<data.dump(2)<<'\n';
}
static bool exportChecked(const simulation_project::ProjectDocument& document,const std::filesystem::path& base,
    const ExternalTrajectoryEvaluationRequest& request,const StoredMotionPlan& plan,const std::filesystem::path& output,Json& report,
    bool enforcePositionLimits=true)
{
    std::string error;ProjectPlanningRequest query;query.robotId=plan.robotId;query.jointNames=plan.jointNames;
    query.collisionDetectorIds=request.collisionDetectorIds;
    query.enforceJointPositionLimits=enforcePositionLimits;
    auto scene=ProjectPlanningSceneBuilder::build(document,base,query,&error);if(!scene)throw std::runtime_error(error);
    Json points=Json::array(),checks=Json::array();bool valid=true;double minimum=1e100;
    std::ofstream csv(output/"optimized_joints.csv");csv<<std::setprecision(17)<<"time_s";
    for(const auto& name:plan.jointNames)csv<<","<<name<<"_rad";csv<<'\n';
    for(const auto& point:plan.trajectory.points) {
        if(!scene->setState(point.q,&error))throw std::runtime_error(error);
        const auto distance=scene->nearestDistance(point.q);std::string reason;
        const bool baffle=scene->validateVirtualBaffles(request.minimumClearance,reason);
        const auto state=scene->validateState(point.q);
        const bool accepted=state.valid && distance.valid && !distance.inCollision && baffle;valid &= accepted;
        if(distance.valid)minimum=std::min(minimum,distance.distanceMeters);
        points.push_back({{"time_s",point.time},{"q_rad",point.q}});
        checks.push_back({{"valid",accepted},{"time_s",point.time},{"distance_m",distance.distanceMeters},{"baffle_clear",baffle}});
        csv<<point.time;for(double q:point.q)csv<<","<<q;csv<<'\n';
    }
    if(!valid)throw std::runtime_error("Fresh exported-node audit rejected a node.");
    std::string matrices;if(!ProjectTrajectoryControlPointEditor::serializeCartesianMatrices(plan,matrices,&error))throw std::runtime_error(error);
    std::ofstream tcp(output/"optimized_tcp_mm.txt");tcp<<matrices;tcp.close();
    TrajectoryImportOptions importOptions;importOptions.robotId=plan.robotId;importOptions.jointNames=plan.jointNames;
    std::string jointTable;
    if(!ProjectTrajectoryControlPointEditor::serializeJointTable(plan,jointTable,&error))throw std::runtime_error(error);
    {std::ofstream file(output/"optimized_joint_angles.txt");file<<jointTable;}
    const auto jointImport=ProjectTrajectoryImporter::importFile(output/"optimized_joint_angles.txt",importOptions);
    const auto cdfImport=ProjectCdfJointAngleImporter::importFile(output/"optimized_joint_angles.txt");
    bool jointIdentical=jointImport.success && cdfImport.success && cdfImport.hasMotionMetadata &&
        jointImport.plan.trajectory.points.size()==plan.trajectory.points.size() &&
        cdfImport.plan.trajectory.points.size()==plan.trajectory.points.size() &&
        CartesianMotionSampler::isMoveL(jointImport.plan)==CartesianMotionSampler::isMoveL(plan) &&
        CartesianMotionSampler::isMoveL(cdfImport.plan)==CartesianMotionSampler::isMoveL(plan);
    if(jointIdentical)for(std::size_t i=0;i<plan.trajectory.points.size();++i) {
        const auto& expected=plan.trajectory.points[i];
        jointIdentical &= jointImport.plan.trajectory.points[i].q==expected.q &&
            jointImport.plan.trajectory.points[i].time==expected.time &&
            cdfImport.plan.trajectory.points[i].q==expected.q && cdfImport.plan.trajectory.points[i].time==expected.time;
    }
    report["joint_txt_exact_roundtrip"]=jointIdentical;
    if(!jointIdentical)throw std::runtime_error("Joint TXT changed verified commands, times or motion law.");
    const auto reimport=ProjectTrajectoryImporter::importFile(output/"optimized_tcp_mm.txt",importOptions);
    if(!reimport.success || !reimport.plan.trajectory.empty())throw std::runtime_error("TCP roundtrip import rejected.");
    CartesianIkOptions ik;ik.robotId=plan.robotId;ik.jointNames=plan.jointNames;ik.seedJoints.assign(plan.jointNames.size(),0.);
    if(enforcePositionLimits)for(const auto& bound:scene->jointBounds()){ik.lower.push_back(bound.lower);ik.upper.push_back(bound.upper);}
    ik.worldForwardKinematics=[&](const auto& q) {
        if(!scene->setState(q,&error))throw std::runtime_error(error);
        return Eigen::Isometry3d(scene->linkWorldTransform(request.tcpLinkName)*request.linkFromTcp);
    };
    const auto restored=ProjectTrajectoryInverseKinematics::solveCartesianControlPoints(document,reimport.plan,ik);
    bool identical=restored.success && restored.plan.trajectory.points.size()==plan.trajectory.points.size();
    if(identical)for(std::size_t i=0;i<plan.trajectory.points.size();++i)
        identical &= restored.plan.trajectory.points[i].q==plan.trajectory.points[i].q && restored.plan.trajectory.points[i].time==plan.trajectory.points[i].time;
    report["tcp_reimport_exact_joint_roundtrip"]=identical;
    if(!identical)throw std::runtime_error("TCP roundtrip changed verified joint configurations or turns.");
    if(CartesianMotionSampler::isMoveL(plan)) {
        if(!CartesianMotionSampler::isMoveL(restored.plan))throw std::runtime_error("TCP roundtrip lost the MoveL motion law.");
        const auto sampler=CartesianMotionSampler::create(*scene,request.tcpLinkName,request.linkFromTcp,error);
        if(!sampler)throw std::runtime_error(error);
        std::size_t frames=0;double maximumTcpResidual=0.;
        for(std::size_t i=1;i<plan.trajectory.points.size();++i) {
            const auto& a=plan.trajectory.points[i-1].q;const auto& b=plan.trajectory.points[i].q;
            const auto from=ik.worldForwardKinematics(a),to=ik.worldForwardKinematics(b);
            for(double fraction:{.25,.5,.75}) {
                std::vector<double> before,after,jointFrame;
                if(!sampler->sample(a,b,fraction,before,error) ||
                    !sampler->sample(restored.plan.trajectory.points[i-1].q,restored.plan.trajectory.points[i].q,fraction,after,error) || before!=after ||
                    !sampler->sample(jointImport.plan.trajectory.points[i-1].q,jointImport.plan.trajectory.points[i].q,fraction,jointFrame,error) || before!=jointFrame)
                    throw std::runtime_error("TCP roundtrip changed a MoveL playback frame: "+error);
                const auto actual=ik.worldForwardKinematics(after);
                const double position=(actual.translation()-((1-fraction)*from.translation()+fraction*to.translation())).norm();
                const double angle=Eigen::Quaterniond(actual.linear()).angularDistance(
                    Eigen::Quaterniond(from.linear()).slerp(fraction,Eigen::Quaterniond(to.linear())));
                maximumTcpResidual=std::max({maximumTcpResidual,position,angle});++frames;
                if(position>1e-8 || angle>1e-8)throw std::runtime_error("Exported MoveL frame differs from actual-model TCP straight line.");
            }
        }
        report["joint_txt_exact_movel_playback"]=true;report["tcp_reimport_exact_movel_playback"]=true;report["roundtrip_interior_frames"]=frames;
        report["maximum_movel_fk_residual"]=maximumTcpResidual;
    }
    write(output/"optimized_joints.json",{{"joint_names",plan.jointNames},{"points",points},{"interpolation",CartesianMotionSampler::isMoveL(plan)?"cartesian_linear":"joint_linear"}});
    write(output/"verification_nodes.json",checks);
    report["output_points"]=points.size();report["export_node_checks_passed"]=valid;
    report["minimum_node_distance_m"]=minimum;report["virtual_baffles"]=scene->virtualBaffleStatus();return valid;
}
static int verifyExistingExport(char** argv)
{
    // Supplemental IO audit only. The original from-zero run and its report remain untouched.
    std::cout<<std::unitbuf;const auto began=Clock::now();
    const auto project=std::filesystem::absolute(std::filesystem::u8path(argv[2]));
    const auto accepted=std::filesystem::absolute(std::filesystem::u8path(argv[3]));
    const auto output=std::filesystem::absolute(std::filesystem::u8path(argv[4]));
    std::filesystem::create_directories(output);
    Json report={{"success",false},{"scope","Supplemental joint/TCP IO roundtrip and fresh node audit of a completed accepted run; no GA or APF rerun"}};
    try {
        simulation_project::ProjectDocument document,saved;std::string error;
        if(!simulation_project::loadProjectDocument(project,document,&error) || document.robots.empty() ||
            !simulation_project::loadProjectDocument(accepted,saved,&error))throw std::runtime_error(error);
        Json originalReport;{std::ifstream file(accepted.parent_path()/"report.json");file>>originalReport;}
        if(!originalReport.value("success",false) || !originalReport.value("pipeline_success",false))
            throw std::runtime_error("Supplemental export requires a completed accepted from-zero result.");
        const auto plans=MotionPlanningProjectStore::plans(saved);
        const auto found=std::find_if(plans.begin(),plans.end(),[&](const auto& plan){
            return plan.id==document.robots.front().id+"_cdf_qp_ga_apf";
        });
        if(found==plans.end())throw std::runtime_error("Accepted GA/APF plan is missing.");
        Json originalJoints;{std::ifstream file(accepted.parent_path()/"optimized_joints.json");file>>originalJoints;}
        const auto& rows=originalJoints.at("points");
        if(rows.size()!=found->trajectory.points.size())throw std::runtime_error("Accepted plan row count mismatch.");
        for(std::size_t i=0;i<rows.size();++i)if(rows[i].at("q_rad").get<std::vector<double>>()!=found->trajectory.points[i].q ||
            rows[i].at("time_s").get<double>()!=found->trajectory.points[i].time)
            throw std::runtime_error("Accepted plan commands differ from the original audited output.");
        ExternalTrajectoryEvaluationRequest request;request.plan=*found;
        ProjectCdfQpRepairOptions options;
        options.detectorIds=ProjectCdfQpTrajectoryRepairService::distanceDetectorIds(document,found->robotId);
        if(!ProjectCdfQpTrajectoryRepairService::ensureCollisionSetup(document,found->robotId,options))throw std::runtime_error("Collision setup failed.");
        request.collisionDetectorIds=ProjectCdfQpTrajectoryRepairService::distanceDetectorIds(document,found->robotId);
        if(!resolvePlanningTcp(document,found->robotId,request.tcpLinkName,request.linkFromTcp,error))throw std::runtime_error(error);
        const auto metadata=Json::parse(found->optimizationMetadata);
        if(metadata.value("joint_convention",std::string())!="runtime")throw std::runtime_error("Expected actual runtime joint convention.");
        request.minimumClearance=metadata.at("clearance_m").get<double>();
        report["source_accepted_project"]=accepted.generic_u8string();
        report["original_commands_exact"]=true;
        exportChecked(document,project.parent_path(),request,*found,output,report);
        report["success"]=true;
    } catch(const std::exception& error) {report["error"]=error.what();}
    report["total_seconds"]=seconds(began);write(output/"report.json",report);
    std::cout<<report.dump(2)<<'\n';return report.at("success").get<bool>()?0:1;
}

static int checkContinuity(int argc,char** argv)
{
    if(argc<5)throw std::runtime_error("--continuity project trajectory output [--joint-step value] [--workers value]");
    std::cout<<std::unitbuf;const auto began=Clock::now();
    const auto project=std::filesystem::absolute(std::filesystem::u8path(argv[2]));
    const auto output=std::filesystem::absolute(std::filesystem::u8path(argv[4]));
    std::filesystem::create_directories(output);
    Json report={{"success",false},{"scope","Actual six-axis GUI-compatible single IK followed by full MoveL replay; position limits disabled by user policy"}};
    try {
        double step=.001;int workers=8;bool refine=false,benchmark=false,benchmarkGuide=false;CartesianPathRefinementOptions refineOptions;
        for(int i=5;i<argc;++i) {
            const std::string flag=argv[i];
            if(flag=="--joint-step" && i+1<argc)step=std::stod(argv[++i]);
            else if(flag=="--workers" && i+1<argc)workers=std::stoi(argv[++i]);
            else if(flag=="--refine")refine=true;
            else if(flag=="--benchmark-kinematics")benchmark=true;
            else if(flag=="--benchmark-apf-guide")benchmarkGuide=true;
            else if(flag=="--sigma" && i+1<argc)refineOptions.smoothingSigma=std::stod(argv[++i]);
            else if(flag=="--bending-weight" && i+1<argc)refineOptions.bendingWeight=std::stod(argv[++i]);
            else if(flag=="--horizontal-weight" && i+1<argc)refineOptions.horizontalAxisWeight=std::stod(argv[++i]);
            else if(flag=="--adjust-endpoints")refineOptions.preserveEndpointEncoders=false;
            else if(flag=="--horizontal-preserve-axis" && i+1<argc) {
                const std::string axis=argv[++i];
                if(axis=="x")refineOptions.horizontalPreservedLocalAxis=Eigen::Vector3d::UnitX();
                else if(axis=="y")refineOptions.horizontalPreservedLocalAxis=Eigen::Vector3d::UnitY();
                else if(axis=="z")refineOptions.horizontalPreservedLocalAxis=Eigen::Vector3d::UnitZ();
                else throw std::runtime_error("Preserved horizontal axis must be x, y or z");
            }
            else if(flag=="--horizontal-axis" && i+1<argc) {
                const std::string axis=argv[++i];
                if(axis=="x")refineOptions.horizontalLocalAxis=Eigen::Vector3d::UnitX();
                else if(axis=="y")refineOptions.horizontalLocalAxis=Eigen::Vector3d::UnitY();
                else if(axis=="z")refineOptions.horizontalLocalAxis=Eigen::Vector3d::UnitZ();
                else throw std::runtime_error("Horizontal axis must be x, y or z");
            }
            else if(flag=="--max-position-mm" && i+1<argc)refineOptions.maximumPositionDeviation=.001*std::stod(argv[++i]);
            else if(flag=="--max-angle-deg" && i+1<argc)refineOptions.maximumOrientationDeviation=std::stod(argv[++i])*std::acos(-1.)/180.;
            else if(flag=="--refinement-iterations" && i+1<argc)refineOptions.maximumSearchIterations=std::stoi(argv[++i]);
            else throw std::runtime_error("Unknown continuity option: "+flag);
        }
        if(!std::isfinite(step) || step<=0 || step>.01)throw std::runtime_error("Invalid MoveL validation step");
        simulation_project::ProjectDocument document;std::string error;
        if(!simulation_project::loadProjectDocument(project,document,&error) || document.robots.empty())throw std::runtime_error(error);
        const auto& robot=document.robots.front();ExternalTrajectoryEvaluationRequest request;request.plan.robotId=robot.id;
        for(const auto& j:robot.initialJoints)request.plan.jointNames.push_back(j.jointName);
        TrajectoryImportOptions importOptions;importOptions.robotId=robot.id;importOptions.jointNames=request.plan.jointNames;
        const auto imported=ProjectTrajectoryImporter::importFile(std::filesystem::u8path(argv[3]),importOptions);
        if(!imported.success)throw std::runtime_error(imported.message());request.plan=imported.plan;
        request.collisionDetectorIds=ProjectCdfQpTrajectoryRepairService::distanceDetectorIds(document,robot.id);
        if(request.collisionDetectorIds.empty())throw std::runtime_error("No existing collision detectors");
        if(!resolvePlanningTcp(document,robot.id,request.tcpLinkName,request.linkFromTcp,error))throw std::runtime_error(error);
        ProjectPlanningRequest query;query.robotId=robot.id;query.jointNames=request.plan.jointNames;
        query.collisionDetectorIds=request.collisionDetectorIds;query.enforceJointPositionLimits=false;
        auto scene=ProjectPlanningSceneBuilder::build(document,project.parent_path(),query,&error);
        if(!scene)throw std::runtime_error(error);
        const auto sampler=CartesianMotionSampler::create(*scene,request.tcpLinkName,request.linkFromTcp,error);
        if(!sampler)throw std::runtime_error(error);
        const auto fk=[&](const auto& q){return sampler->forward(q);};
        const auto ikStart=Clock::now();
        if(request.plan.trajectory.empty()) {
            CartesianIkOptions ik;ik.robotId=robot.id;ik.jointNames=query.jointNames;
            for(const auto& j:robot.initialJoints)ik.seedJoints.push_back(j.value);
            ik.worldForwardKinematics=fk;ik.maxIterations=5000;ik.tolerance=1e-6;ik.stepSize=1.;ik.damping=.001;
            ik.worldJacobian=[sampler](const auto& q){return sampler->jacobian(q);};
            std::cout<<seconds(began)<<" Solving "<<request.plan.cartesianControlPoints.points.size()<<" Cartesian controls\n";
            const auto solved=ProjectTrajectoryInverseKinematics::solveCartesianControlPoints(document,request.plan,ik);
            report["ik_solved"]=solved.solvedPointCount();
            if(!solved.success) {
                report["ik_failures"]=Json::array();for(const auto& p:solved.points)if(!p.success)
                    report["ik_failures"].push_back({{"index",p.pointIndex},{"message",p.message}});
                throw std::runtime_error("Single IK did not solve every input point");
            }
            request.plan=solved.plan;
        }
        report["ik_seconds"]=seconds(ikStart);
        auto& plan=request.plan;plan.optimizationMetadata=Json{{"motion","cartesian_linear"}}.dump();
        // A joint-table replay has exact encoders but no Cartesian controls.
        // Rebuild only its export representation using the same verified TCP FK.
        if(plan.cartesianControlPoints.points.empty())for(const auto& point:plan.trajectory.points) {
            robottrajectory::TimedCartesianPoint tcp;tcp.time=point.time;tcp.tcpPose=fk(point.q);
            plan.cartesianControlPoints.points.push_back(tcp);
        }
        report["input_points"]=plan.trajectory.points.size();
        const auto qualityJson=[](const TrajectoryQuality& q){return Json{{"joint_length",q.jointLength},{"bending",q.bendingCost},
            {"maximum_corner_rad",q.maximumCornerRadians},{"peak_velocity",q.peakVelocity},{"peak_acceleration",q.peakAcceleration},
            {"duration",q.duration},{"tcp_length",q.tcpLength}};};
        std::vector<std::vector<double>> comparisonPath;for(const auto& p:plan.trajectory.points)comparisonPath.push_back(p.q);
        const auto comparisonParameters=jointPathParameters(comparisonPath);
        report["input_quality"]=qualityJson(evaluateTrajectoryQuality(plan.trajectory,scene->jointBounds(),1.0471975512,2.0943951024,comparisonParameters,fk));
        std::vector<std::vector<double>> path;Json joints=Json::array();
        for(const auto& p:plan.trajectory.points){path.push_back(p.q);joints.push_back({{"time_s",p.time},{"q_rad",p.q}});}
        write(output/"baseline_joints.json",{{"joint_names",plan.jointNames},{"points",joints}});
        if(benchmark || benchmarkGuide) {
            report["scope"]="Kinematic kernels on actual input encoder edges; this mode performs no collision acceptance";
            auto* robotRuntime=scene->simulationRuntime().robot(robot.id);std::vector<std::size_t> dofs;
            for(const auto& name:plan.jointNames)dofs.push_back(std::size_t(robotRuntime->model.joints.at(robotRuntime->model.jointNameToIndex.at(name)).dofIndex));
            const auto runtimeFk=[&](const auto& q) {
                for(std::size_t j=0;j<dofs.size();++j)robotRuntime->instance->setJoint(dofs[j],q.at(j));
                robotRuntime->instance->update();
                return Eigen::Isometry3d(scene->linkWorldTransform(request.tcpLinkName)*request.linkFromTcp);
            };
            if(benchmarkGuide) {
                report["scope"]="APF ordered guide kernel on every actual input edge; identical 0.001 rad uniform grid, no collision acceptance";
                detail::ApfGuidance guide;guide.maxDeviation=.01;
                for(const auto& q:path) {const auto pose=fk(q);guide.positions.push_back({pose.translation().x(),pose.translation().y(),pose.translation().z()});}
                std::string sampleError;
                guide.sampleMotion=[&](const auto& a,const auto& b,double f,auto& q){return sampler->sample(a,b,f,q,sampleError);};
                guide.position=[&](const auto& q){const auto p=runtimeFk(q).translation().eval();return detail::ApfState{p.x(),p.y(),p.z()};};
                std::vector<bool> cpu(path.size()-1),accelerated(path.size()-1);auto start=Clock::now();
                for(std::size_t i=0;i<cpu.size();++i)cpu[i]=detail::followsApfGuide(path[i],path[i+1],guide.positions[i],guide.positions[i+1],guide);
                report["reference_cpu_seconds"]=seconds(start);
                guide.position=[&](const auto& q){const auto p=fk(q).translation().eval();return detail::ApfState{p.x(),p.y(),p.z()};};
                guide.sampleMotionBatch=[&](const auto& a,const auto& b,std::size_t n,auto& samples){return sampler->uniformSamples(a,b,n,samples,sampleError);};
                start=Clock::now();
                for(std::size_t i=0;i<cpu.size();++i)accelerated[i]=detail::followsApfGuide(path[i],path[i+1],guide.positions[i],guide.positions[i+1],guide);
                report["batched_gpu_verified_fk_seconds"]=seconds(start);
                report["edges"]=cpu.size();report["matching_verdicts"]=cpu==accelerated;
                report["reference_rejected_edges"]=std::count(cpu.begin(),cpu.end(),false);
                report["accelerated_rejected_edges"]=std::count(accelerated.begin(),accelerated.end(),false);
                report["success"]=cpu==accelerated;report["total_seconds"]=seconds(began);
                write(output/"report.json",report);std::cout<<report.dump(2)<<'\n';return report.value("success",false)?0:1;
            }
            double residual=0;auto start=Clock::now();
            for(int repeat=0;repeat<4;++repeat)for(const auto& q:path)residual+=runtimeFk(q).translation().x();
            const double runtimeTime=seconds(start);start=Clock::now();
            for(int repeat=0;repeat<4;++repeat)for(const auto& q:path)residual+=fk(q).translation().x();
            report["fk_benchmark"]={{"runtime_seconds",runtimeTime},{"verified_poe_seconds",seconds(start)},{"poses",4*path.size()},{"checksum",residual}};
            detail::GpuSerialIk chain;CartesianMultiIkOptions config;config.model.worldForwardKinematics=runtimeFk;
            config.lower.assign(6,-std::numeric_limits<double>::infinity());config.upper.assign(6,std::numeric_limits<double>::infinity());
            if(!chain.configure(scene->simulationRuntime().robot(robot.id)->model,plan.jointNames,request.tcpLinkName,config))
                throw std::runtime_error(chain.status().dump());
            std::vector<std::size_t> edges;for(std::size_t i:{0,351,399,465,990,1190,1960,2278})if(i+1<path.size())edges.push_back(i);
            report["movel_benchmark"]=Json::array();
            for(std::size_t intervals:{32,64,128,256,512,1024}) {
                double cpuTime=0,gpuTime=0,maxJointDifference=0,maxFkResidual=0;bool valid=true;
                for(int repeat=0;repeat<3;++repeat)for(auto i:edges) {
                    std::vector<std::vector<double>> cpu(intervals+1),gpu;start=Clock::now();
                    for(std::size_t k=0;k<=intervals;++k)valid &= sampler->sample(path[i],path[i+1],double(k)/intervals,cpu[k],error);
                    cpuTime+=seconds(start);start=Clock::now();
                    const bool accelerated=chain.linearSamples(path[i],path[i+1],intervals,gpu);
                    const double gpuElapsed=seconds(start);
                    if(repeat==0 && i==edges.front()) { // Exclude and separately expose first process/device initialization.
                        if(!report.contains("gpu_first_call_seconds"))report["gpu_first_call_seconds"]=gpuElapsed;start=Clock::now();
                        valid &= chain.linearSamples(path[i],path[i+1],intervals,gpu);gpuTime+=seconds(start);
                    } else gpuTime+=gpuElapsed;
                    valid &= accelerated && gpu.size()==cpu.size();
                    if(gpu.size()==cpu.size())for(std::size_t k=0;k<cpu.size();++k)if(cpu[k].size()==6) {
                        for(int j=0;j<6;++j)maxJointDifference=std::max(maxJointDifference,std::abs(cpu[k][j]-gpu[k][j]));
                        maxFkResidual=std::max(maxFkResidual,(fk(cpu[k]).matrix()-fk(gpu[k]).matrix()).norm());
                    }
                }
                report["movel_benchmark"].push_back({{"intervals",intervals},{"edges",3*edges.size()},{"cpu_seconds",cpuTime},
                    {"gpu_seconds",gpuTime},{"both_solved",valid},{"maximum_joint_difference_rad",maxJointDifference},{"maximum_fk_difference",maxFkResidual}});
            }
            report["gpu_status"]=chain.status();report["success"]=true;report["total_seconds"]=seconds(began);
            write(output/"report.json",report);std::cout<<report.dump(2)<<'\n';return 0;
        }
        if(refine) {
            refineOptions.collisionDetectorIds=query.collisionDetectorIds;refineOptions.queryWorkers=workers;
            refineOptions.enforceJointPositionLimits=false;
            refineOptions.progress=[&](const auto& s){std::cout<<seconds(began)<<" "<<s<<'\n';};
            const auto refined=ProjectCartesianPathRefinementService{}.refine(document,project.parent_path(),plan,refineOptions);
            report["refinement"]=Json::parse(refined.statistics);write(output/"refinement.json",report["refinement"]);
            if(!refined.success)throw std::runtime_error(refined.message);
            plan=refined.plan;path.clear();for(const auto& p:plan.trajectory.points)path.push_back(p.q);
            report["output_quality"]=qualityJson(evaluateTrajectoryQuality(plan.trajectory,scene->jointBounds(),1.0471975512,2.0943951024,comparisonParameters,fk));
        }
        detail::CdfQueryBatch batch(*scene,document,project.parent_path(),query,workers,true);
        batch.setCartesianMotion(sampler);
        MotionValidationOptions validation;validation.maxJointStep=step;validation.maxSamples=1000000;validation.preserveJointTurns=true;
        std::cout<<seconds(began)<<" Verifying every MoveL edge, joint step="<<step<<"\n";
        const auto checkStart=Clock::now();
        const auto checks=batch.motions(path,validation,false,[&](std::size_t done,std::size_t total){
            std::cout<<seconds(began)<<" MoveL "<<done<<" / "<<total<<'\n';});
        report["validation_seconds"]=seconds(checkStart);report["validation_joint_step_rad"]=step;
        report["failed_edges"]=Json::array();
        for(std::size_t i=0;i<checks.size();++i)if(!checks[i].valid)
            report["failed_edges"].push_back({{"edge_zero_based",i},{"code",checks[i].diagnosticCode},{"message",checks[i].message}});
        report["query_profile"]=batch.summary();
        if(!report["failed_edges"].empty()) {
            const auto nodes=batch.states(path,0.);
            report["failed_nodes"]=Json::array();
            for(std::size_t i=0;i<nodes.size();++i)if(!nodes[i].validation.valid)
                report["failed_nodes"].push_back({{"index",i},{"code",nodes[i].validation.diagnosticCode}});
            report["failure_samples"]=Json::array();
            for(std::size_t i=0;i<checks.size();++i)if(!checks[i].valid) {
                std::vector<std::vector<double>> samples;
                if(!sampler->connection(path[i],path[i+1],step,1000000,samples,error))continue;
                const auto interior=batch.states(samples,0.);
                for(std::size_t k=0;k<interior.size();++k)if(!interior[k].validation.valid) {
                    const auto d=scene->nearestDistance(samples[k]);
                    report["failure_samples"].push_back({{"edge",i},{"fraction",double(k)/(interior.size()-1)},
                        {"q_rad",samples[k]},{"distance_m",d.distanceMeters},{"distance_valid",d.valid}});break;
                }
            }
            throw std::runtime_error("Input failed full MoveL verification");
        }
        report["full_movel_valid"]=true;
        exportChecked(document,project.parent_path(),request,plan,output,report,false);
        if(plan.id.empty())plan.id=robot.id+"_verified_continuity";
        if(!MotionPlanningProjectStore::upsertPlan(document,plan,&error) ||
            !simulation_project::saveProjectDocument(output/"accepted_project.sys.json",document,&error))throw std::runtime_error(error);
        report["accepted_project_saved"]=true;
        report["success"]=true;
    } catch(const std::exception& e){report["error"]=e.what();}
    report["total_seconds"]=seconds(began);write(output/"report.json",report);
    std::cout<<report.dump(2)<<'\n';return report.value("success",false)?0:1;
}

static int runCheck(int argc,char** argv)
{
    if(argc>=2 && std::string(argv[1])=="--continuity")return checkContinuity(argc,argv);
    if(argc==5 && std::string(argv[1])=="--verify-export")return verifyExistingExport(argv);
    if(argc<4) {std::cerr<<"project original-trajectory output-directory [ga-workers] [--spray-distance-mm value] [--fast-search] [--deadline-seconds value]\n";return 2;}
    std::cout<<std::unitbuf;const auto began=Clock::now();
    const auto project=std::filesystem::absolute(std::filesystem::u8path(argv[1]));
    const auto output=std::filesystem::absolute(std::filesystem::u8path(argv[3]));std::filesystem::create_directories(output);
    Json report={{"success",false},{"scope","From original input through the production GA/APF entry; no prior IK/GA/path checkpoint"},
        {"cdf_enabled",false},{"diagnostics",Json::array()},{"stage_timings",Json::array()}};
    try {
        simulation_project::ProjectDocument document;std::string error;
        if(!simulation_project::loadProjectDocument(project,document,&error) || document.robots.empty())throw std::runtime_error(error);
        const auto robot=document.robots.front();ExternalTrajectoryEvaluationRequest request;
        request.plan.robotId=robot.id;for(const auto& joint:robot.initialJoints)request.plan.jointNames.push_back(joint.jointName);
        TrajectoryImportOptions importedOptions;importedOptions.robotId=robot.id;importedOptions.jointNames=request.plan.jointNames;
        const auto imported=ProjectTrajectoryImporter::importFile(std::filesystem::u8path(argv[2]),importedOptions);
        if(!imported.success)throw std::runtime_error(imported.message());request.plan=imported.plan;
        if(!request.plan.trajectory.empty())throw std::runtime_error("From-zero check requires raw Cartesian input.");
        ProjectCdfQpRepairOptions options;
        options.detectorIds=ProjectCdfQpTrajectoryRepairService::distanceDetectorIds(document,robot.id);
        if(!ProjectCdfQpTrajectoryRepairService::ensureCollisionSetup(document,robot.id,options))throw std::runtime_error("Collision setup failed");
        GaSurfaceSearchOptions surface;
        double deadlineSeconds=0.;
        for(int i=4;i<argc;++i) {
            const std::string option=argv[i];
            if(option=="--spray-distance-mm" && i+1<argc)surface.sprayDistance=std::stod(argv[++i])*.001;
            else if(option=="--deadline-seconds" && i+1<argc)deadlineSeconds=std::stod(argv[++i]);
            else if(option=="--legacy-search")surface.latencyOptimized=false;
            else if(option=="--fast-search")surface.latencyOptimized=true;
            else if(i==4 && option.rfind("--",0)!=0)request.gaPointWorkers=std::stoi(option);
            else throw std::runtime_error("Unknown or incomplete argument: "+option);
        }
        request.cdfOptions=options;request.collisionDetectorIds=ProjectCdfQpTrajectoryRepairService::distanceDetectorIds(document,robot.id);
        if(!std::isfinite(deadlineSeconds) || deadlineSeconds<0)throw std::runtime_error("Invalid computation deadline.");
        request.cancelled=[&](){return deadlineSeconds>0 && seconds(began)>=deadlineSeconds;};
        report["deadline_seconds"]=deadlineSeconds;report["latency_optimized"]=surface.latencyOptimized;
        if(!resolvePlanningTcp(document,robot.id,request.tcpLinkName,request.linkFromTcp,error))throw std::runtime_error(error);
        request.progress=[&](const std::string& text) {
            std::cout<<seconds(began)<<" "<<text<<'\n';
            const auto event=Json::parse(text,nullptr,false);
            if(event.is_object() && event.value("type",std::string())=="pipeline_stage_timing")report["stage_timings"].push_back(event);
        };
        report["input_points"]=request.plan.cartesianControlPoints.points.size();report["detectors"]=request.collisionDetectorIds;
        report["spray_distance_m"]=surface.sprayDistance;
        const auto pipelineStart=Clock::now();const auto result=runGaApfCdfPipeline(document,project.parent_path(),request,false,surface);
        report["pipeline_seconds"]=seconds(pipelineStart);report["pipeline_success"]=result.success;
        for(const auto& diagnostic:result.diagnostics)report["diagnostics"].push_back({{"code",diagnostic.code},{"message",diagnostic.message}});
        for(std::size_t index=0;index<result.stages.size();++index) {
            Json points=Json::array();for(const auto& point:result.stages[index].plan.trajectory.points)points.push_back({{"time_s",point.time},{"q_rad",point.q}});
            write(output/("stage_"+std::to_string(index+1)+".json"),{{"name",result.stages[index].name},{"points",points}});
        }
        if(!result.success)throw std::runtime_error("Production GA/APF did not accept the complete trajectory; see diagnostics");
        report["source_reference_times_s"]=result.sourceReferenceTimes;report["selected_spray_distances_m"]=result.gaSelectedSprayDistances;
        const auto& originals=imported.plan.cartesianControlPoints.points;
        if(result.sourceReferenceTimes.size()!=result.plan.trajectory.points.size())throw std::runtime_error("Missing output/source station mapping.");
        std::vector<std::size_t> extra(originals.size()-1,0);std::vector<bool> retained(originals.size(),false);
        for(double time:result.sourceReferenceTimes) {
            const auto upper=std::lower_bound(originals.begin(),originals.end(),time-1e-8,[](const auto& p,double t){return p.time<t;});
            if(upper!=originals.end() && std::abs(upper->time-time)<=1e-8)retained[std::size_t(upper-originals.begin())]=true;
            else {
                const auto edge=std::upper_bound(originals.begin(),originals.end(),time,[](double t,const auto& p){return t<p.time;});
                if(edge==originals.begin() || edge==originals.end())throw std::runtime_error("Output outside source intervals.");
                ++extra[std::size_t(edge-originals.begin()-1)];
            }
        }
        if(std::any_of(extra.begin(),extra.end(),[](auto n){return n>128;}) ||
            std::any_of(retained.begin(),retained.end(),[](bool value){return !value;}))
            throw std::runtime_error("APF sampling lost an original control or exceeded the bounded dense retry budget.");
        report["all_original_controls_retained"]=true;report["max_intermediate_points_per_source_edge"]=*std::max_element(extra.begin(),extra.end());
        report["intermediate_points_per_source_edge"]=extra;
        exportChecked(document,project.parent_path(),request,result.plan,output,report);
        if(!MotionPlanningProjectStore::upsertPlan(document,result.plan,&error) ||
            !simulation_project::saveProjectDocument(output/"accepted_project.sys.json",document,&error))throw std::runtime_error(error);
        report["success"]=true;report["total_seconds"]=seconds(began);write(output/"report.json",report);
        std::cout<<"FINISHED success=true seconds="<<seconds(began)<<'\n';return 0;
    } catch(const std::exception& e) {
        report["success"]=false;report["error"]=e.what();report["total_seconds"]=seconds(began);
        write(output/"failure.json",report);if(!std::filesystem::exists(output/"report.json"))write(output/"report.json",report);
        std::cerr<<"FINISHED success=false "<<e.what()<<'\n';return 1;
    }
}

#ifdef _WIN32
int wmain(int argc,wchar_t** argv)
{
    std::vector<std::string> utf8;utf8.reserve(argc);
    for(int i=0;i<argc;++i)utf8.push_back(std::filesystem::path(argv[i]).u8string());
    std::vector<char*> arguments;arguments.reserve(argc);
    for(auto& text:utf8)arguments.push_back(text.data());
    return runCheck(argc,arguments.data());
}
#else
int main(int argc,char** argv) {return runCheck(argc,argv);}
#endif
