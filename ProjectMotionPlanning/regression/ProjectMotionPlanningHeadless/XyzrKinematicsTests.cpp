#include <ProjectMotionPlanning/XyzrKinematics.h>
#include <ProjectMotionPlanning/TrajectoryImport.h>
#include <SimulationRuntime/ProjectSimulationRuntime.h>
#include <SimulationProject/ProjectIO.h>
#include <RobotInstance/RobotInstance.h>
#include <iostream>
#include <fstream>
#include <stdexcept>
#include <limits>

namespace
{
    void require(bool ok,const std::string& message) {if(!ok)throw std::runtime_error(message);}
    constexpr double pi=3.14159265358979323846;
}

int main()
{
    using namespace motion_planning;
    try {
        const auto root=std::filesystem::u8path(XYZR_SOURCE_ROOT);
        simulation_project::ProjectDocument document;simulation_project::RobotDesc robot;
        robot.id="xyzr";robot.name="XYZR regression";robot.sourceType="urdf";
        robot.sourcePath=(root/"data/robots/xyzr_spray_robot_demo/urdf/xyzr_spray_robot_demo.urdf").generic_u8string();
        document.robots.push_back(robot);
        std::string error;auto model=XyzrKinematics::create(document,root,robot.id,CartesianIkToolMode::FixedTool,error);
        require(bool(model),error);
        require(model->jointNames()==std::vector<std::string>{"x_joint","y_joint","z_joint","yaw_joint"},"Actual model joint order with no initialJoints");
        TrajectoryImportOptions import;import.robotId=robot.id;import.planId="tongpian";import.jointNames=model->jointNames();
        const auto input=ProjectTrajectoryImporter::importFile(root/"data/trajectories/xyzr_tongpian.txt",import);
        require(input.success,input.message());require(input.plan.cartesianControlPoints.points.size()==8735,"Full original trajectory imports");
        CartesianIkOptions options;options.robotId=robot.id;options.jointNames=model->jointNames();
        const auto result=model->solve(input.plan,options);
        require(result.success,result.diagnostics.empty()?"Full XYZR solve failed":result.diagnostics.front().message);
        simulation_runtime::ProjectSimulationRuntime actual;
        require(actual.loadProject(document,root).success,"Independent runtime load");auto* item=actual.robot(robot.id);
        Eigen::Matrix3d calibration;calibration<<0,1,0,0,0,1,1,0,0;
        double maxPosition=0,maxRotation=0,maxDemoDifference=0;
        for(std::size_t i=0;i<result.plan.trajectory.points.size();++i) {
            const auto& point=result.plan.trajectory.points[i];const auto& original=input.plan.cartesianControlPoints.points[i];
            require(point.q.size()==4 && point.time==original.time,"Four SI joints and original timestamps");
            actual.setRobotJointValues(robot.id,model->jointNames(),point.q);item->instance->update();
            auto pose=item->instance->getLinkTransform("tool0");pose.linear()=pose.linear()*calibration;
            maxPosition=std::max(maxPosition,(pose.translation()-original.tcpPose.translation()).norm());
            maxRotation=std::max(maxRotation,Eigen::AngleAxisd(pose.linear()*original.tcpPose.linear().transpose()).angle());
            const auto& p=original.tcpPose.translation();const double yaw=std::atan2(-original.tcpPose.linear()(0,2),original.tcpPose.linear()(1,2));
            const std::vector<double> demo={p.x()+.08*std::sin(yaw),p.y()+.08-.08*std::cos(yaw),p.z()-.47,yaw};
            for(int j=0;j<4;++j)maxDemoDifference=std::max(maxDemoDifference,std::abs(demo[j]-point.q[j]));
            if(i)for(double f:{.25,.5,.75}) {
                std::vector<double> q;require(model->sample(result.plan.trajectory.points[i-1].q,point.q,f,q,error),error);
                const auto mid=model->forward(q);const auto& prev=input.plan.cartesianControlPoints.points[i-1];
                require((mid.translation()-((1-f)*prev.tcpPose.translation()+f*original.tcpPose.translation())).norm()<1e-9,"Internal frame remains on TCP straight line");
            }
        }
        require(maxPosition<1e-9 && maxRotation<1e-9 && maxDemoDifference<1e-9,"Independent actual FK and webpage analytic formula agree");
        auto savedDocument=document;
        require(MotionPlanningProjectStore::upsertPlan(savedDocument,result.plan,&error),error);
        const auto savedPath=std::filesystem::current_path()/"xyzr_roundtrip.scene.json";
        require(simulation_project::saveProjectDocument(savedPath,savedDocument,&error),error);
        simulation_project::ProjectDocument restored;
        require(simulation_project::loadProjectDocument(savedPath,restored,&error),error);
        const auto restoredPlans=MotionPlanningProjectStore::plans(restored);
        require(restoredPlans.size()==1 && restoredPlans[0].optimizationMetadata==result.plan.optimizationMetadata &&
            restoredPlans[0].trajectory.points.back().q==result.plan.trajectory.points.back().q,
            "Save/load preserves XYZR playback solver, TCP mode and exact joint values");
        require(XyzrKinematics::create(restored,root,robot.id,CartesianIkToolMode::FixedTool,error)!=nullptr,
            "Reloaded document can reconstruct actual-model playback sampler");
        auto bad=input.plan;bad.cartesianControlPoints.points[100].tcpPose.linear()=Eigen::AngleAxisd(.1,Eigen::Vector3d::UnitX()).toRotationMatrix()*bad.cartesianControlPoints.points[100].tcpPose.linear();
        const auto failed=model->solve(bad,options);
        require(!failed.success && failed.plan.trajectory.empty(),"Incompatible orientation cannot produce a partial executable path");
        std::vector<double> q;
        const auto fullTurn=model->forward({.01,.02,.03,4*pi+.2});
        require(model->solvePose(fullTurn,{0,0,0,4*pi},q,error) && std::abs(q[3]-4*pi-.2)<1e-9,"Continuous rotation retains turn nearest seed");
        auto nan=fullTurn;nan.translation().x()=std::numeric_limits<double>::quiet_NaN();
        require(!model->solvePose(nan,{},q,error),"Nonfinite pose rejected");
        auto strict=document;strict.robots[0].sourcePath=(root/"data/robots/xyzr_spray_robot_demo/urdf/xyzr_spray_robot_original_limits.urdf").generic_u8string();
        auto limited=XyzrKinematics::create(strict,root,robot.id,CartesianIkToolMode::FixedTool,error);require(bool(limited),error);
        const auto blocked=limited->solve(input.plan,options);
        require(!blocked.success && blocked.plan.trajectory.empty(),"Original URDF limits stay enforced");
        require(limited->solvePose(limited->forward({.02,.01,.03,.5}),{},q,error),"Original-limit robot solves reachable target");
        auto moved=document;moved.robots[0].baseTransform={};
        moved.robots[0].baseTransform.x=.3;moved.robots[0].baseTransform.y=-.2;moved.robots[0].baseTransform.z=.1;
        moved.robots[0].baseTransform.roll=.15;moved.robots[0].baseTransform.pitch=-.2;moved.robots[0].baseTransform.yaw=.4;
        auto transformed=XyzrKinematics::create(moved,root,robot.id,CartesianIkToolMode::FixedTool,error);require(bool(transformed),error);
        simulation_runtime::ProjectSimulationRuntime movedRuntime;require(movedRuntime.loadProject(moved,root).success,"Moved runtime loads");
        const std::vector<double> test={.02,.03,-.01,.7};movedRuntime.setRobotJointValues(robot.id,model->jointNames(),test);
        auto* movedItem=movedRuntime.robot(robot.id);movedItem->instance->update();auto target=movedItem->instance->getLinkTransform("tool0");target.linear()=target.linear()*calibration;
        require(transformed->solvePose(target,{},q,error),error);for(int j=0;j<4;++j)require(std::abs(q[j]-test[j])<1e-9,"Base transform applied exactly once");
        auto flange=XyzrKinematics::create(document,root,robot.id,CartesianIkToolMode::Flange,error);require(bool(flange),error);
        require(flange->solvePose(flange->forward(test),{},q,error),"Flange mode works independently of spray TCP");
        std::cout<<"8735 poses, 26202 internal frames, independent URDF FK; max position m="<<maxPosition
            <<", rotation rad="<<maxRotation<<", webpage joint difference="<<maxDemoDifference<<"\n";
        std::cout<<"Strict limits, unreachable poses, continuous turns, transformed base and flange checks passed.\n";
        return 0;
    } catch(const std::exception& e) {std::cerr<<e.what()<<'\n';return 1;}
}
