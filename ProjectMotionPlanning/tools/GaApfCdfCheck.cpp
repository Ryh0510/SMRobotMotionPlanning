#include <ProjectMotionPlanning/GaApfCdfPipeline.h>
#include <ProjectMotionPlanning/TrajectoryImport.h>
#include <SimulationProject/ProjectIO.h>
#include <nlohmann/json.hpp>
#include <fstream>
#include <iostream>
#include <chrono>
#include <algorithm>
#include <cmath>

int main(int argc,char** argv)
{
    if(argc<4) {std::cerr<<"Usage: GaApfCdfCheck project trajectory.txt report.json [--cdf|--checks] [--spray-distance-mm value]\n";return 2;}
    using namespace motion_planning;
    nlohmann::json report;
    try {
        const auto project=std::filesystem::absolute(std::filesystem::u8path(argv[1]));
        simulation_project::ProjectDocument document;std::string error;
        if(!simulation_project::loadProjectDocument(project,document,&error))throw std::runtime_error(error);
        if(document.robots.empty())throw std::runtime_error("No robot");
        const auto& robot=document.robots.front();
        ExternalTrajectoryEvaluationRequest request;request.plan.robotId=robot.id;
        for(const auto& j:robot.initialJoints)request.plan.jointNames.push_back(j.jointName);
        ProjectCdfQpRepairOptions options;options.detectorId="cdf_"+robot.id+"_A";
        bool checks=false,useCdf=false;GaSurfaceSearchOptions surface;
        for(int i=4;i<argc;++i) {
            const std::string option=argv[i];
            if(option=="--checks")checks=true;
            else if(option=="--cdf")useCdf=true;
            else if(option=="--spray-distance-mm" && i+1<argc)surface.sprayDistance=std::stod(argv[++i])*.001;
            else throw std::runtime_error("Unknown or incomplete argument: "+option);
        }
        if(checks) {
            auto missing=document;missing.objects.clear();
            missing.robots.erase(missing.robots.begin()+1,missing.robots.end());
            const bool noResurrection=!ProjectCdfQpTrajectoryRepairService::ensureCollisionSetup(missing,robot.id,options) && missing.objects.empty() && missing.robots.size()==1;
            if(!ProjectCdfQpTrajectoryRepairService::ensureCollisionSetup(document,robot.id,options))throw std::runtime_error("Cannot configure detector");
            const bool configured=ProjectCdfQpTrajectoryRepairService::collisionSetupConfigured(document,robot.id,options);
            document.collision.query.enabled=false;
            const bool disabled=!ProjectCdfQpTrajectoryRepairService::collisionSetupConfigured(document,robot.id,options) &&
                ProjectCdfQpTrajectoryRepairService::distanceDetectorIds(document,robot.id).empty();
            document.collision.query.enabled=true;
            const bool wrongRobot=!ProjectCdfQpTrajectoryRepairService::collisionSetupConfigured(document,"missing_robot",options);
            std::string link;Eigen::Isometry3d tcp;
            const bool savedTcp=resolvePlanningTcp(document,robot.id,link,tcp,error);
            request.cancelled=[] {return true;};
            request.collisionDetectorIds={options.detectorId};
            const auto cancelled=runGaApfCdfPipeline(document,project.parent_path(),request,true);
            report={{"success",noResurrection && configured && disabled && wrongRobot && savedTcp && !cancelled.success && cancelled.plan.trajectory.empty()},
                {"deleted_workpiece_not_recreated",noResurrection},{"selected_robot_detector",configured},
                {"global_disable_respected",disabled},{"wrong_robot_rejected",wrongRobot},{"saved_tool_tcp",savedTcp},
                {"failed_pipeline_not_published",!cancelled.success && cancelled.plan.trajectory.empty()}};
        } else {
            TrajectoryImportOptions importOptions;importOptions.robotId=robot.id;importOptions.jointNames=request.plan.jointNames;
            auto imported=ProjectTrajectoryImporter::importFile(std::filesystem::u8path(argv[2]),importOptions);
            if(!imported.success)throw std::runtime_error(imported.message());
            request.plan=imported.plan;
            if(!ProjectCdfQpTrajectoryRepairService::ensureCollisionSetup(document,robot.id,options))throw std::runtime_error("Cannot configure actual robot/workpiece collisions");
            request.collisionDetectorIds=ProjectCdfQpTrajectoryRepairService::distanceDetectorIds(document,robot.id);
            request.cdfOptions=options;
            const auto started=std::chrono::steady_clock::now();
            request.progress=[&](const std::string& text){std::cout<<std::chrono::duration<double>(std::chrono::steady_clock::now()-started).count()<<" "<<text<<std::endl;};
            const auto result=runGaApfCdfPipeline(document,project.parent_path(),request,useCdf,surface);
            report={{"success",result.success},{"cancelled",result.cancelled},{"source",argv[2]},
                {"robot",robot.id},{"input_cartesian_points",imported.plan.cartesianControlPoints.points.size()},
                {"surface_search",{{"spray_distance_mm",surface.sprayDistance*1000},{"half_angle_deg",80},
                    {"radius_evolution",true},{"axial_spin",true},{"preserve_safe_controls",true},{"single_point_time_limit_seconds",0}}},
                {"ga",{{"point_evaluations",request.gaPointEvaluations},{"initial_postures",request.gaInitialPostures},{"generations",request.gaGenerations},{"population",24},{"gpu",request.gaGpuEnabled}}},
                {"elapsed_seconds",std::chrono::duration<double>(std::chrono::steady_clock::now()-started).count()},
                {"diagnostics",nlohmann::json::array()},{"stages",nlohmann::json::array()},{"joint_points",nlohmann::json::array()}};
            report["selected_spray_distances_m"]=result.gaSelectedSprayDistances;
            for(const auto& d:result.diagnostics)report["diagnostics"].push_back({{"code",d.code},{"message",d.message}});
            for(const auto& s:result.stages)report["stages"].push_back({{"name",s.name},{"points",s.plan.trajectory.points.size()},{"timing_valid",s.quality.timingValid}});
            for(const auto& p:result.plan.trajectory.points)report["joint_points"].push_back({{"time_s",p.time},{"q_rad",p.q}});
            if(result.success) {
                ProjectPlanningRequest query;query.robotId=robot.id;query.jointNames=result.plan.jointNames;query.collisionDetectorIds=request.collisionDetectorIds;
                auto scene=ProjectPlanningSceneBuilder::build(document,project.parent_path(),query,&error);
                if(!scene)throw std::runtime_error(error);
                report["distances"]=nlohmann::json::array();bool valid=true;
                for(const auto& p:result.plan.trajectory.points) {
                    const auto distance=scene->nearestDistance(p.q);valid=valid && distance.valid && !distance.inCollision;
                    report["distances"].push_back({{"time_s",p.time},{"valid",distance.valid},{"distance_m",distance.distanceMeters},{"collision",distance.inCollision},{"message",distance.message}});
                }
                report["distance_all_valid_and_clear"]=valid;report["success"]=valid;
                std::string tcpLink;Eigen::Isometry3d tcpOffset;
                if(!resolvePlanningTcp(document,robot.id,tcpLink,tcpOffset,error))throw std::runtime_error(error);
                auto originals=imported.plan.cartesianControlPoints.points;
                if(originals.empty())for(const auto& joint:imported.plan.trajectory.points) {
                    if(!scene->setState(joint.q,&error))throw std::runtime_error(error);
                    robottrajectory::TimedCartesianPoint point;point.time=joint.time;
                    point.tcpPose=scene->linkWorldTransform(tcpLink)*tcpOffset;originals.push_back(point);
                }
                std::vector<std::size_t> intervalCounts(originals.size()-1,0);
                for(std::size_t i=1;i<result.sourceReferenceTimes.size();++i){const double time=result.sourceReferenceTimes[i];
                    const auto upper=std::lower_bound(originals.begin()+1,originals.end(),time-1e-8,[](const auto& v,double t){return v.time<t;});
                    ++intervalCounts[std::min(intervalCounts.size()-1,std::size_t(upper-originals.begin()-1))];}
                report["output_interval_subdivisions"]=intervalCounts;
                report["source_reference_times_s"]=result.sourceReferenceTimes;
                bool surfaceValid=result.sourceReferenceTimes.size()==result.plan.trajectory.points.size() && result.gaSelectedSprayDistances.size()==originals.size();
                report["surface_samples"]=nlohmann::json::array();
                for(std::size_t i=0;surfaceValid && i<result.plan.trajectory.points.size();++i) {
                    const double time=result.sourceReferenceTimes[i];
                    auto upper=std::upper_bound(originals.begin(),originals.end(),time,[](double value,const auto& p){return value<p.time;});
                    const auto edge=upper==originals.begin()?0:std::min(std::size_t(upper-originals.begin()-1),originals.size()-2);
                    const auto& a=originals[edge];const auto& b=originals[edge+1];
                    const double f=std::clamp((time-a.time)/(b.time-a.time),0.,1.);
                    const Eigen::Vector3d anchor=(1-f)*(a.tcpPose.translation()+surface.sprayDistance*a.tcpPose.linear().col(2))+
                        f*(b.tcpPose.translation()+surface.sprayDistance*b.tcpPose.linear().col(2));
                    const Eigen::Vector3d reference=Eigen::Quaterniond(a.tcpPose.linear()).slerp(f,Eigen::Quaterniond(b.tcpPose.linear()))*Eigen::Vector3d::UnitZ();
                    if(!scene->setState(result.plan.trajectory.points[i].q,&error))throw std::runtime_error(error);
                    const auto actual=scene->linkWorldTransform(tcpLink)*tcpOffset;
                    const double distance=(anchor-actual.translation()).dot(actual.linear().col(2));
                    const double miss=(actual.translation()+distance*actual.linear().col(2)-anchor).norm();
                    const double angle=std::acos(std::clamp(reference.dot(actual.linear().col(2)),-1.,1.))*180/std::acos(-1.);
                    const double selected=(1-f)*result.gaSelectedSprayDistances[edge]+f*result.gaSelectedSprayDistances[edge+1];
                    surfaceValid=std::abs(distance-selected)<=.001001 && miss<=.001001 && angle<=80.001 && distance>=.2*surface.sprayDistance-.001001 && distance<=1.8*surface.sprayDistance+.001001;
                    report["surface_samples"].push_back({{"source_time_s",time},{"selected_distance_mm",selected*1000},{"distance_mm",distance*1000},{"aim_error_mm",miss*1000},{"normal_tilt_deg",angle},{"valid",surfaceValid}});
                }
                report["surface_all_valid"]=surfaceValid;report["success"]=valid && surfaceValid;
            }
        }
    }catch(const std::exception& e){report={{"success",false},{"error",e.what()}};}
    std::ofstream out(std::filesystem::u8path(argv[3]));out<<report.dump(2)<<'\n';
    return report.value("success",false)?0:1;
}
