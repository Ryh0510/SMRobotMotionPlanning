#include <ProjectMotionPlanning/TrajectoryInverseKinematics.h>
#include <ProjectMotionPlanning/TrajectoryImport.h>
#include <ProjectMotionPlanning/GaApfCdfPipeline.h>
#include <SimulationRuntime/ProjectSimulationRuntime.h>
#include <SimulationProject/ProjectIO.h>
#include <RobotInstance/RobotInstance.h>
#include <iostream>
#include <stdexcept>
#include <set>

int main(int argc,char** argv)
{
    using namespace motion_planning;
    std::cout<<std::unitbuf;
    try {
        const auto root=std::filesystem::u8path(FANUC_SOURCE_ROOT);
        const auto path=root/"config/projects/ABB4600-Aburnner.sys.json";
        simulation_project::ProjectDocument document;std::string error;
        if(!simulation_project::loadProjectDocument(path,document,&error))throw std::runtime_error(error);
        // Resolve the fixture asset explicitly; do not depend on a developer's environment.
        document.robots.front().sourcePath=(root/"data/fanuc_m710ic50/m710ic_50_drake.urdf").generic_u8string();
        const auto id=document.robots.front().id;
        CartesianIkOptions options;options.robotId=id;
        for(const auto& j:document.robots.front().initialJoints)options.jointNames.push_back(j.jointName);
        options.seedJoints.assign(6,0);options.maxIterations=5000;options.stepSize=1;options.damping=.001;
        if(!ProjectTrajectoryInverseKinematics::readRevoluteJointLimits(document,path.parent_path(),id,
            options.jointNames,options.lower,options.upper,error))throw std::runtime_error(error);
        std::string link;Eigen::Isometry3d offset;
        if(!resolvePlanningTcp(document,id,link,offset,error))throw std::runtime_error(error);
        simulation_runtime::ProjectSimulationRuntime runtime;
        if(!runtime.loadProject(document,path.parent_path()).success)throw std::runtime_error("Fanuc runtime load failed");
        auto* item=runtime.robot(id);item->instance->setBaseTransform(Eigen::Isometry3d::Identity());
        options.worldForwardKinematics=[&](const std::vector<double>& q)->Eigen::Isometry3d {
            for(std::size_t j=0;j<q.size();++j) {
                const auto index=item->model.jointNameToIndex.at(options.jointNames[j]);
                item->instance->setJoint(std::size_t(item->model.joints[index].dofIndex),q[j]);
            }
            item->instance->update();
            return item->baseTransform*item->instance->getLinkTransform(link)*offset;
        };
        std::cout<<"Actual Fanuc TCP link "<<link<<" offset\n"<<offset.matrix()<<'\n';
        StoredMotionPlan plan;plan.robotId=id;plan.id="fanuc_regression";plan.jointNames=options.jointNames;
        if(argc>1) {
            TrajectoryImportOptions import;import.robotId=id;import.jointNames=options.jointNames;
            auto input=ProjectTrajectoryImporter::importFile(std::filesystem::u8path(argv[1]),import);
            if(!input.success)throw std::runtime_error(input.message());plan=input.plan;
        } else {
            TrajectoryImportOptions import;import.robotId=id;import.jointNames=options.jointNames;
            const auto fixture=ProjectTrajectoryImporter::importFile(root/
                "SMRobotMotionPlanning/ProjectMotionPlanning/regression/ProjectMotionPlanningHeadless/fanuc_limit_branches.txt",import);
            if(!fixture.success)throw std::runtime_error(fixture.message());
            for(auto p:fixture.plan.cartesianControlPoints.points) {
                p.time=double(plan.cartesianControlPoints.points.size());plan.cartesianControlPoints.points.push_back(p);
            }
        }
        const auto lower=options.lower,upper=options.upper;
        options.lower.clear();options.upper.clear();
        const auto result=ProjectTrajectoryInverseKinematics::solveCartesianControlPoints(document,plan,options);
        std::cout<<"Position limits ignored: "<<result.solvedPointCount()<<"/"<<result.points.size()<<'\n';
        for(const auto& d:result.diagnostics)std::cout<<d.code<<": "<<d.message<<'\n';
        if(!result.success)throw std::runtime_error("Unbounded basic IK did not solve every target");
        double maxPosition=0,maxRotation=0;
        for(std::size_t i=0;i<result.points.size();++i) {
            const auto actual=options.worldForwardKinematics(result.points[i].joints);
            const auto& target=plan.cartesianControlPoints.points[i].tcpPose;
            maxPosition=std::max(maxPosition,(actual.translation()-target.translation()).norm());
            maxRotation=std::max(maxRotation,Eigen::AngleAxisd(actual.linear()*target.linear().transpose()).angle());
            auto single=plan;single.cartesianControlPoints.points={plan.cartesianControlPoints.points[i]};
            if(!ProjectTrajectoryInverseKinematics::solveCartesianControlPoints(document,single,options).success)
                throw std::runtime_error("Unbounded single-point IK failed: "+std::to_string(i+1));
        }
        std::cout<<"All independent zero-seed single-point solves pass; max FK position m="<<maxPosition
            <<", rotation rad="<<maxRotation<<'\n';
        if(maxPosition>options.tolerance || maxRotation>options.tolerance)throw std::runtime_error("FK residual exceeds tolerance");
        if(argc==1 || (argc>2 && std::string(argv[2])=="multi")) {
            CartesianMultiIkOptions multi;multi.model=options;multi.lower=lower;multi.upper=upper;
            multi.progress=[](std::size_t done,std::size_t total){if(done%100==0)std::cout<<"Multi IK "<<done<<"/"<<total<<'\n';};
            auto multiPlan=plan;
            const auto all=ProjectTrajectoryInverseKinematics::solveAllCartesianControlPoints(multiPlan,multi);
            const std::set<std::size_t> expectedUnreachable={4,5,6};
            std::size_t found=0;
            for(std::size_t i=0;i<all.layers.size();++i) {
                const bool solved=!all.layers[i].candidates.empty();if(solved)++found;
                if(argc==1 && solved==bool(expectedUnreachable.count(i)))
                    throw std::runtime_error("Wrong bounded All IK result: "+std::to_string(i+1));
                for(const auto& c:all.layers[i].candidates)for(std::size_t j=0;j<6;++j)
                    if(c.joints[j]<lower[j]-1e-9 || c.joints[j]>upper[j]+1e-9)
                        throw std::runtime_error("All IK must retain original physical limits");
            }
            if(all.layers.size()!=multiPlan.cartesianControlPoints.points.size())throw std::runtime_error("All IK returned incomplete layers");
            std::cout<<"All IK with original position limits: "<<found<<"/"<<all.layers.size()<<'\n';
        }
        auto unreachable=plan;unreachable.cartesianControlPoints.points.resize(1);
        unreachable.cartesianControlPoints.points[0].tcpPose.translation().x()=100;
        options.maxIterations=160;
        if(ProjectTrajectoryInverseKinematics::solveCartesianControlPoints(document,unreachable,options).success)
            throw std::runtime_error("Unreachable target must fail");
        return 0;
    }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 2;}
}

