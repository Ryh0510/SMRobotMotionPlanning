#include <ProjectMotionPlanning/GaApfCdfPipeline.h>
#include <ProjectMotionPlanning/TrajectoryImport.h>
#include <ProjectMotionPlanning/TrajectoryControlPointEditing.h>
#include <SimulationProject/ProjectIO.h>
#include <nlohmann/json.hpp>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <chrono>
#include <algorithm>

using namespace motion_planning;
using Json=nlohmann::json;
using Clock=std::chrono::steady_clock;
static double seconds(Clock::time_point began) {return std::chrono::duration<double>(Clock::now()-began).count();}
static void write(const std::filesystem::path& path,const Json& data) {
    std::ofstream stream(path);if(!stream)throw std::runtime_error("Cannot write result file");stream<<data.dump(2)<<'\n';
}
static bool exportChecked(const simulation_project::ProjectDocument& document,const std::filesystem::path& base,
    const ExternalTrajectoryEvaluationRequest& request,const StoredMotionPlan& plan,const std::filesystem::path& output,Json& report)
{
    std::string error;ProjectPlanningRequest query;query.robotId=plan.robotId;query.jointNames=plan.jointNames;
    query.collisionDetectorIds=request.collisionDetectorIds;
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
    std::ofstream tcp(output/"optimized_tcp_mm.txt");tcp<<matrices;
    write(output/"optimized_joints.json",{{"joint_names",plan.jointNames},{"points",points},{"interpolation","joint_linear"}});
    write(output/"verification_nodes.json",checks);
    report["output_points"]=points.size();report["export_node_checks_passed"]=valid;
    report["minimum_node_distance_m"]=minimum;report["virtual_baffles"]=scene->virtualBaffleStatus();return valid;
}
int main(int argc,char** argv)
{
    if(argc!=4 && argc!=5) {std::cerr<<"project original-trajectory output-directory [downsample-target]\n";return 2;}
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
        if(!ProjectCdfQpTrajectoryRepairService::ensureCollisionSetup(document,robot.id,options))throw std::runtime_error("Collision setup failed");
        request.cdfOptions=options;request.collisionDetectorIds=ProjectCdfQpTrajectoryRepairService::distanceDetectorIds(document,robot.id);
        if(!resolvePlanningTcp(document,robot.id,request.tcpLinkName,request.linkFromTcp,error))throw std::runtime_error(error);
        request.progress=[&](const std::string& text) {
            std::cout<<seconds(began)<<" "<<text<<'\n';
            const auto event=Json::parse(text,nullptr,false);
            if(event.is_object() && event.value("type",std::string())=="pipeline_stage_timing")report["stage_timings"].push_back(event);
        };
        report["input_points"]=request.plan.cartesianControlPoints.points.size();report["detectors"]=request.collisionDetectorIds;
        const auto pipelineStart=Clock::now();const auto result=runGaApfCdfPipeline(document,project.parent_path(),request,false);
        report["pipeline_seconds"]=seconds(pipelineStart);report["pipeline_success"]=result.success;
        for(const auto& diagnostic:result.diagnostics)report["diagnostics"].push_back({{"code",diagnostic.code},{"message",diagnostic.message}});
        for(std::size_t index=0;index<result.stages.size();++index) {
            Json points=Json::array();for(const auto& point:result.stages[index].plan.trajectory.points)points.push_back({{"time_s",point.time},{"q_rad",point.q}});
            write(output/("stage_"+std::to_string(index+1)+".json"),{{"name",result.stages[index].name},{"points",points}});
        }
        if(!result.success)throw std::runtime_error("Production GA/APF did not accept the complete trajectory; see diagnostics");
        report["source_reference_times_s"]=result.sourceReferenceTimes;report["selected_spray_distances_m"]=result.gaSelectedSprayDistances;
        exportChecked(document,project.parent_path(),request,result.plan,output,report);
        if(!MotionPlanningProjectStore::upsertPlan(document,result.plan,&error) ||
            !simulation_project::saveProjectDocument(output/"accepted_project.sys.json",document,&error))throw std::runtime_error(error);
        report["success"]=true;report["total_seconds"]=seconds(began);write(output/"report.json",report);
        if(argc==5) {
            // Reload through the normal project store, just as the UI does.
            simulation_project::ProjectDocument reopened;
            if(!simulation_project::loadProjectDocument(output/"accepted_project.sys.json",reopened,&error))throw std::runtime_error(error);
            const auto plans=MotionPlanningProjectStore::plans(reopened);
            const auto saved=std::find_if(plans.begin(),plans.end(),[&](const auto& p){return p.id==result.plan.id;});
            if(saved==plans.end())throw std::runtime_error("Saved optimized plan missing");
            request.plan=*saved;request.preferredOutputWaypoints=std::stoull(argv[4]);
            if(!prepareGaApfDownsamplingRequest(request,error))throw std::runtime_error(error);
            const auto sampleDir=output/"downsampled";std::filesystem::create_directories(sampleDir);
            Json sampledReport={{"success",false},{"target_points",request.preferredOutputWaypoints},{"diagnostics",Json::array()}};
            const auto sampleStart=Clock::now();const auto sampled=ProjectExternalTrajectoryEvaluator::evaluate(document,project.parent_path(),request);
            sampledReport["downsample_seconds"]=seconds(sampleStart);
            for(const auto& d:sampled.diagnostics)sampledReport["diagnostics"].push_back({{"code",d.code},{"message",d.message}});
            write(sampleDir/"stage_report.json",sampledReport);
            if(!sampled.success)throw std::runtime_error("Safe downsampling rejected");
            auto replay=request;replay.pipelineStage="validate";replay.preferredOutputWaypoints=0;replay.verificationTrajectory=sampled.plan.trajectory;
            const auto replayStart=Clock::now();const auto checked=ProjectExternalTrajectoryEvaluator::evaluate(document,project.parent_path(),replay);
            sampledReport["replay_seconds"]=seconds(replayStart);sampledReport["independent_replay_passed"]=checked.success;
            if(!checked.success)throw std::runtime_error("Downsampled independent replay rejected");
            exportChecked(document,project.parent_path(),request,sampled.plan,sampleDir,sampledReport);
            sampledReport["success"]=true;sampledReport["total_seconds"]=seconds(sampleStart);write(sampleDir/"report.json",sampledReport);
        }
        std::cout<<"FINISHED success=true seconds="<<seconds(began)<<'\n';return 0;
    } catch(const std::exception& e) {
        report["success"]=false;report["error"]=e.what();report["total_seconds"]=seconds(began);
        write(output/"failure.json",report);if(!std::filesystem::exists(output/"report.json"))write(output/"report.json",report);
        std::cerr<<"FINISHED success=false "<<e.what()<<'\n';return 1;
    }
}
