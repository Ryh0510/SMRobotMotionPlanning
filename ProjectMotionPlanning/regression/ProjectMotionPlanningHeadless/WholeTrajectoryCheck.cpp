#include <ProjectMotionPlanning/GaApfCdfPipeline.h>
#include <ProjectMotionPlanning/CartesianMotion.h>
#include <ProjectMotionPlanning/TrajectoryImport.h>
#include <ProjectMotionPlanning/CdfJointAngleImport.h>
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
    for(const auto& bound:scene->jointBounds()){ik.lower.push_back(bound.lower);ik.upper.push_back(bound.upper);}
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

int main(int argc,char** argv)
{
    if(argc==5 && std::string(argv[1])=="--verify-export")return verifyExistingExport(argv);
    if(argc!=4 && argc!=5) {std::cerr<<"project original-trajectory output-directory [ga-workers]\n";return 2;}
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
        if(argc==5)request.gaPointWorkers=std::stoi(argv[4]);
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
        if(std::any_of(extra.begin(),extra.end(),[](auto n){return n>4;}) ||
            std::any_of(retained.begin(),retained.end(),[](bool value){return !value;}))
            throw std::runtime_error("Selective sampling lost an original control or exceeded four intermediate knots.");
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
