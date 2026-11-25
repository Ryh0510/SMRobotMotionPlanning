#include <ProjectMotionPlanning/SprayAxisContinuity.h>
#include <ProjectMotionPlanning/CdfQpTrajectoryRepair.h>
#include <ProjectMotionPlanning/TrajectoryImport.h>
#include <ProjectMotionPlanning/TrajectoryControlPointEditing.h>
#include <ProjectMotionPlanning/CdfJointAngleImport.h>
#include <ProjectMotionPlanning/CartesianMotion.h>
#include <ProjectMotionPlanning/GaApfCdfPipeline.h>
#include "../src/SpraySurfaceReference.h"
#include "../src/SprayMoveLBounds.h"
#include <Collision/CollisionGeometryBuilder.h>
#include <SimulationProject/ProjectIo.h>
#include <nlohmann/json.hpp>
#include <fstream>
#include <iostream>

namespace {
void require(bool condition,const char* message) {
    if(!condition)throw std::runtime_error(message);
    std::cout<<"[OK] "<<message<<'\n';
}
void boundTests(const simulation_project::ProjectDocument& document,const std::filesystem::path& base,
    const motion_planning::StoredMotionPlan& plan,const std::vector<std::string>& detectors) {
    using namespace motion_planning;
    collision::CollisionGeometryBuildRequest mesh;mesh.shape.type=collision::CollisionShapeType::TriangleMesh;
    mesh.shape.vertices={{0,0,0},{1,0,0},{0,1,0}};mesh.shape.indices={0,1,2};
    const auto built=collision::CollisionGeometryBuilder::build(mesh);require(built.success(),"Triangle fixture builds");
    detail::MeshSeparation separation(*built.geometry);require(separation.valid(),"Mesh separating bounds supported");
    auto tf=Eigen::Isometry3d::Identity();
    require(!separation.separated(separation,tf,0),"Coincident triangles are never certified clear");
    tf.translation().z()=.01;
    require(separation.separated(separation,tf,.009),"Known parallel triangle gap proves smaller displacement ball");
    require(!separation.separated(separation,tf,.011),"Displacement crossing the plane refuses separation");
    tf.translation()=Eigen::Vector3d(2,0,0);
    require(separation.separated(separation,tf,.9),"Coplanar separated triangles have a valid projection bound");
    ProjectPlanningRequest request;request.robotId=plan.robotId;request.jointNames=plan.jointNames;request.collisionDetectorIds=detectors;request.enforceJointPositionLimits=false;
    std::string error,link;Eigen::Isometry3d tcp;
    auto scene=ProjectPlanningSceneBuilder::build(document,base,request,&error);require(bool(scene),"Actual scene loads for derivative tests");
    require(resolvePlanningTcp(document,plan.robotId,link,tcp,error),"TCP resolves for derivative tests");
    const auto sampler=CartesianMotionSampler::create(*scene,link,tcp,error);require(bool(sampler),"Actual-model sampler builds for derivative tests");
    for(const auto& id:detectors)for(const auto& pair:scene->collisionRuntime().pairDiagnostics(id))if(pair.effective)for(const auto* endpoint:{&pair.first,&pair.second}) {
        const auto& info=endpoint->info;
        std::cout<<"Object "<<info.object<<" link="<<info.linkName<<" element="<<info.elementName<<" model="<<info.modelId<<" source="<<info.source<<'\n';
    }
    for(const auto& attachment:scene->simulationRuntime().mountedAttachments())std::cout<<"Attachment="<<attachment.documentId<<" owner="<<attachment.owner.linkName<<" asset="<<attachment.assetId<<'\n';
    for(const auto& object:scene->simulationRuntime().objects())std::cout<<"Runtime object="<<object.documentId<<" runtimeId="<<object.runtimeId<<'\n';
    const auto* robot=scene->simulationRuntime().robot(plan.robotId);std::array<int,6> depth{};
    for(int j=0;j<6;++j) {
        auto child=robot->model.joints.at(robot->model.jointNameToIndex.at(plan.jointNames[j])).child;
        for(std::size_t k=0;k<robot->model.links.size() && child!=robot->model.root;++k) {
            const auto joint=std::find_if(robot->model.joints.begin(),robot->model.joints.end(),[&](const auto& x){return x.child==child;});
            require(joint!=robot->model.joints.end(),"Derivative chain is complete");child=joint->parent;++depth[j];
        }
    }
    double maximum=0;const double h=2e-4,reach=5.;
    for(const std::vector<double> q:std::vector<std::vector<double>>{{.4,-.6,-1.1,1.8,-.6,.2},{-.3,.2,-.7,.9,.3,-.4}}) {
        const auto center=sampler->forward(q);const auto jac=sampler->jacobian(q);
        const auto chart=[&](const std::vector<double>& x) {
            const auto pose=sampler->forward(x);const Eigen::Matrix3d r=center.linear().transpose()*pose.linear();
            Eigen::Matrix<double,6,1> value;value.head<3>()=pose.translation()/reach;
            value.tail<3>()=Eigen::Vector3d(r(2,1)-r(1,2),r(0,2)-r(2,0),r(1,0)-r(0,1))*.5;return value;
        };
        for(int j=0;j<6;++j)for(int k=0;k<6;++k) {
            auto pp=q,pm=q,mp=q,mm=q;pp[j]+=h;pp[k]+=h;pm[j]+=h;pm[k]-=h;mp[j]-=h;mp[k]+=h;mm[j]-=h;mm[k]-=h;
            const Eigen::Matrix<double,6,1> numeric=(chart(pp)-chart(pm)-chart(mp)+chart(mm))/(4*h*h);
            const int ancestor=depth[j]<=depth[k]?j:k,descendant=ancestor==j?k:j;
            maximum=std::max(maximum,(numeric-detail::moveLChartHessian(jac,center.linear(),reach,ancestor,descendant)).norm());
        }
    }
    std::cout<<"Maximum chart Hessian finite-difference error="<<maximum<<'\n';
    require(maximum<1e-6,"Analytic actual-model chart Hessian agrees with finite differences");
}
void roundTrip(const std::filesystem::path& project,const std::filesystem::path& folder) {
    simulation_project::ProjectDocument document;std::string error;
    require(simulation_project::loadProjectDocument(project,document,&error),"Result project reloads");
    const auto plans=motion_planning::MotionPlanningProjectStore::plans(document);
    const auto found=std::find_if(plans.begin(),plans.end(),[](const auto& p){
        const auto j=nlohmann::json::parse(p.optimizationMetadata,nullptr,false);
        return j.is_object() && j.contains("spray_axis_continuity");
    });
    require(found!=plans.end(),"Certified result is present");const auto& plan=*found;
    std::filesystem::create_directories(folder);
    std::string joints,tcp;
    require(motion_planning::ProjectTrajectoryControlPointEditor::serializeJointTable(plan,joints,&error),"Joint export succeeds");
    require(motion_planning::ProjectTrajectoryControlPointEditor::serializeCartesianMatrices(plan,tcp,&error),"TCP export succeeds");
    const auto jointFile=folder/"optimized_joint_angles.txt",tcpFile=folder/"optimized_tcp_mm.txt";
    {std::ofstream file(jointFile);file<<joints;}{std::ofstream file(tcpFile);file<<tcp;}
    const auto imported=motion_planning::ProjectCdfJointAngleImporter::importFile(jointFile);
    require(imported.success && imported.hasMotionMetadata,"Joint export reimports with execution metadata");
    require(motion_planning::CartesianMotionSampler::isMoveL(imported.plan)==motion_planning::CartesianMotionSampler::isMoveL(plan),"Execution motion survives roundtrip");
    require(imported.plan.trajectory.points.size()==plan.trajectory.points.size(),"Joint count survives roundtrip");
    bool exact=true;
    for(std::size_t i=0;i<plan.trajectory.points.size();++i)
        exact &= imported.plan.trajectory.points[i].q==plan.trajectory.points[i].q &&
            imported.plan.trajectory.points[i].time==plan.trajectory.points[i].time;
    require(exact,"All exact joint/time rows survive roundtrip");
    const auto originalMetadata=nlohmann::json::parse(plan.optimizationMetadata);
    const auto importedMetadata=nlohmann::json::parse(imported.plan.optimizationMetadata);
    for(const char* key:{"spray_enabled","source_stations"})
        require(originalMetadata.at(key)==importedMetadata.at(key),"Spray/transfer flags and stations survive joint roundtrip");
    require(!importedMetadata.contains("spray_axis_continuity"),"Import does not claim a fresh scene certificate");
    const std::string marker="# RWS_JOINT_TABLE_V1 ";const auto markerStart=joints.find(marker);
    const auto markerEnd=joints.find('\n',markerStart);
    auto damagedHeader=nlohmann::json::parse(joints.substr(markerStart+marker.size(),markerEnd-markerStart-marker.size()));
    auto& damagedFlags=damagedHeader["spray_execution"]["spray_enabled"];damagedFlags.erase(damagedFlags.end()-1);
    auto damagedText=joints;damagedText.replace(markerStart,markerEnd-markerStart,marker+damagedHeader.dump());
    const auto damagedFile=folder/"negative_truncated_spray_mask.txt";
    {std::ofstream file(damagedFile);file<<damagedText;}
    require(!motion_planning::ProjectCdfJointAngleImporter::importFile(damagedFile).success,"Damaged spray mask is rejected");
    motion_planning::TrajectoryImportOptions options;options.robotId=plan.robotId;options.jointNames=plan.jointNames;
    const auto cartesian=motion_planning::ProjectTrajectoryImporter::importFile(tcpFile,options);
    require(cartesian.success,"TCP export reimports");
    const auto tcpMetadata=nlohmann::json::parse(cartesian.plan.optimizationMetadata);
    require(tcpMetadata.at("motion")==originalMetadata.at("motion") && tcpMetadata.at("points").size()==plan.trajectory.points.size(),"TCP retains exact joint seeds and motion type");
    require(tcpMetadata.at("spray_enabled")==originalMetadata.at("spray_enabled") &&
        tcpMetadata.at("source_stations")==originalMetadata.at("source_stations"),"TCP retains spray/transfer flags");
    double poseError=0;
    for(std::size_t i=0;i<plan.trajectory.points.size();++i) {
        exact &= tcpMetadata["points"][i]["q"].get<std::vector<double>>()==plan.trajectory.points[i].q;
        poseError=std::max(poseError,(cartesian.plan.cartesianControlPoints.points[i].tcpPose.matrix()-plan.cartesianControlPoints.points[i].tcpPose.matrix()).cwiseAbs().maxCoeff());
    }
    require(exact,"All TCP exact seed rows survive roundtrip");
    require(poseError<1e-12,"TCP poses survive roundtrip to numerical precision");
    auto edited=plan;
    require(motion_planning::ProjectTrajectoryControlPointEditor::removeCartesianControlPoint(edited,1,&error),"Control editing succeeds");
    const auto editedMetadata=nlohmann::json::parse(edited.optimizationMetadata);
    require(edited.trajectory.empty() && !editedMetadata.contains("spray_axis_continuity"),"Editing invalidates solved joints and continuous certificate");
    std::cout<<"Roundtrip points="<<plan.trajectory.points.size()<<" maximumPoseError="<<poseError<<'\n';
}
}

int main(int argc,char** argv) {
    try {
        if(argc>=4 && std::string(argv[1])=="--roundtrip") {
            roundTrip(std::filesystem::u8path(argv[2]),std::filesystem::u8path(argv[3]));return 0;
        }
        const bool failureTests=argc>=2 && std::string(argv[1])=="--failure-tests";
        const bool inspect=argc>=2 && std::string(argv[1])=="--inspect";
        const bool bounds=argc>=2 && std::string(argv[1])=="--bound-tests";
        if(failureTests || inspect || bounds){--argc;++argv;}
        if(argc<4)throw std::runtime_error("Usage: SprayAxisContinuityCheck project trajectory output [joint-step-rad] [clearance-mm] [max-roll-deg] [max-tilt-deg] [tcp-adjustment-mm] [maximum-output-controls] [fixed|pivot|flexible]");
        std::cout<<std::unitbuf;
        const auto project=std::filesystem::absolute(std::filesystem::u8path(argv[1]));
        const auto output=std::filesystem::absolute(std::filesystem::u8path(argv[3]));std::filesystem::create_directories(output);
        simulation_project::ProjectDocument document;std::string error;
        if(!simulation_project::loadProjectDocument(project,document,&error) || document.robots.empty())throw std::runtime_error(error);
        motion_planning::TrajectoryImportOptions import;import.robotId=document.robots.front().id;
        for(const auto& joint:document.robots.front().initialJoints)import.jointNames.push_back(joint.jointName);
        const auto source=motion_planning::ProjectTrajectoryImporter::importFile(std::filesystem::u8path(argv[2]),import);
        if(!source.success)throw std::runtime_error(source.message());
        motion_planning::SprayAxisContinuityOptions options;
        options.collisionDetectorIds=motion_planning::ProjectCdfQpTrajectoryRepairService::distanceDetectorIds(document,import.robotId);
        if(argc>4)options.validationJointStep=std::stod(argv[4]);
        if(argc>5)options.clearance=std::stod(argv[5])*.001;
        if(argc>6)options.maximumRollRadians=std::stod(argv[6])*std::acos(-1.)/180.;
        if(argc>7)options.maximumTiltRadians=std::stod(argv[7])*std::acos(-1.)/180.;
        if(argc>8)options.maximumTcpAdjustment=std::stod(argv[8])*.001;
        if(argc>9)options.maximumOutputPoints=std::stoull(argv[9]);
        if(argc>10) {
            const std::string mode=argv[10];
            if(mode=="fixed")options.poseConstraint=motion_planning::SprayPoseConstraint::FixedTcp;
            else if(mode=="pivot")options.poseConstraint=motion_planning::SprayPoseConstraint::SurfacePivotFixedDistance;
            else if(mode=="flexible")options.poseConstraint=motion_planning::SprayPoseConstraint::SurfacePivotAdjustableDistance;
            else throw std::runtime_error("Pose mode must be fixed, pivot or flexible.");
        }
        options.progress=[](const std::string& message){std::cout<<message<<'\n';};
        if(bounds){boundTests(document,project.parent_path(),source.plan,options.collisionDetectorIds);return 0;}
        if(inspect) {
            motion_planning::ProjectPlanningRequest request;request.robotId=import.robotId;request.jointNames=import.jointNames;
            request.collisionDetectorIds=options.collisionDetectorIds;request.enforceJointPositionLimits=false;
            auto scene=motion_planning::ProjectPlanningSceneBuilder::build(document,project.parent_path(),request,&error);
            if(!scene)throw std::runtime_error(error);
            nlohmann::json report;report["colliding_nodes"]=nlohmann::json::array();
            for(std::size_t i=0;i<source.plan.trajectory.points.size();++i) {
                const auto valid=scene->validateState(source.plan.trajectory.points[i].q);
                if(!valid.valid)report["colliding_nodes"].push_back({{"index",i},{"code",valid.diagnosticCode}});
            }
            report["nodes"]=source.plan.trajectory.points.size();
            if(source.plan.trajectory.points.size()<20) {
                std::string link;Eigen::Isometry3d tcp;
                if(!motion_planning::resolvePlanningTcp(document,import.robotId,link,tcp,error))throw std::runtime_error(error);
                auto sampler=motion_planning::CartesianMotionSampler::create(*scene,link,tcp,error);
                motion_planning::detail::SpraySurfaceReference surface(scene->simulationRuntime(),{});
                for(std::size_t edge=0;edge+1<source.plan.trajectory.points.size();++edge)
                    for(int k=0;k<=20;++k) {
                        std::vector<double> q;const double u=k/20.;
                        if(!sampler->sample(source.plan.trajectory.points[edge].q,source.plan.trajectory.points[edge+1].q,u,q,error))continue;
                        const auto pose=sampler->forward(q);const auto valid=scene->validateState(q);double distance;
                        const bool hit=surface.hit(pose.translation(),pose.linear().col(2),distance);
                        nlohmann::json row={{"edge",edge},{"u",u},{"safe",valid.valid},{"surface_hit",hit},
                            {"position",{pose.translation().x(),pose.translation().y(),pose.translation().z()}}};
                        if(hit)row["distance_m"]=distance;
                        for(const auto& id:options.collisionDetectorIds) {
                            const auto* result=scene->collisionRuntime().resultOf(id);
                            if(result && result->inCollision())for(const auto& contact:result->contacts)
                                row["contacts"].push_back({contact.infoA.linkName,contact.infoB.linkName,contact.penetrationDepth});
                        }
                        report["edge_samples"].push_back(row);
                    }
            }
            {std::ofstream file(output/"inspection.json");file<<report.dump(2);}
            std::cout<<report.dump(2)<<'\n';return 0;
        }
        if(failureTests) {
            auto invalid=options;invalid.collisionDetectorIds.clear();
            auto failed=motion_planning::ProjectSprayAxisContinuityService::repair(document,project.parent_path(),source.plan,invalid);
            require(!failed.success && failed.plan.trajectory.empty(),"No collision detector refuses output");
            auto cancelled=options;cancelled.cancelled=[]{return true;};
            failed=motion_planning::ProjectSprayAxisContinuityService::repair(document,project.parent_path(),source.plan,cancelled);
            require(!failed.success && failed.plan.trajectory.empty(),"Cancellation refuses partial output");
            options.maximumRollRadians=0;options.maximumTiltRadians=0;options.maximumTcpAdjustment=0;
            failed=motion_planning::ProjectSprayAxisContinuityService::repair(document,project.parent_path(),source.plan,options);
            {std::ofstream file(output/"zero_freedom_rejection.json");file<<failed.statistics;}
            require(!failed.success && failed.plan.trajectory.empty() && failed.message.find("exhausted")!=std::string::npos,
                "Known endpoint-safe interior collision is rejected with no posture freedom");
            return 0;
        }
        const auto result=motion_planning::ProjectSprayAxisContinuityService::repair(document,project.parent_path(),source.plan,options);
        {std::ofstream file(output/"report.json");file<<result.statistics;}
        if(!result.success){std::cerr<<result.message<<'\n';return 1;}
        if(!result.plan.processMetadata.empty()) {
            motion_planning::ProjectPlanningRequest request;request.robotId=import.robotId;request.jointNames=import.jointNames;
            request.collisionDetectorIds=options.collisionDetectorIds;request.enforceJointPositionLimits=false;
            auto scene=motion_planning::ProjectPlanningSceneBuilder::build(document,project.parent_path(),request,&error);
            require(bool(scene),"Process validation scene rebuilds");
            std::string link;Eigen::Isometry3d tool;
            require(motion_planning::resolvePlanningTcp(document,import.robotId,link,tool,error),"Process validation TCP resolves");
            const auto sampler=motion_planning::CartesianMotionSampler::create(*scene,link,tool,error);
            motion_planning::detail::SpraySurfaceReference surface(scene->simulationRuntime(),{});
            const auto process=nlohmann::json::parse(result.plan.processMetadata);
            require(process["points"].size()==result.plan.trajectory.points.size(),"Process rows follow sparse output controls");
            for(std::size_t i=0;i<result.plan.trajectory.points.size();++i) {
                const auto pose=sampler->forward(result.plan.trajectory.points[i].q);double distance;
                if(surface.hit(pose.translation(),pose.linear().col(2),distance))
                    require(std::abs(process["points"][i].at("distance").get<double>()-distance)<1e-8,"Process spray distance matches the adjusted output pose");
                else require(!process["points"][i].contains("distance") && !process["points"][i].value("spray",true),"Transfer row clears stale spray distance");
            }
        }
        std::string joints,tcp;
        if(!motion_planning::ProjectTrajectoryControlPointEditor::serializeJointTable(result.plan,joints,&error) ||
            !motion_planning::ProjectTrajectoryControlPointEditor::serializeCartesianMatrices(result.plan,tcp,&error))throw std::runtime_error(error);
        {std::ofstream file(output/"optimized_joint_angles.txt");file<<joints;}
        {std::ofstream file(output/"optimized_tcp_mm.txt");file<<tcp;}
        if(!motion_planning::MotionPlanningProjectStore::upsertPlan(document,result.plan,&error) ||
            !simulation_project::saveProjectDocument(output/"accepted_project.sys.json",document,&error))throw std::runtime_error(error);
        std::cout<<result.message<<'\n';return 0;
    }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 2;}
}
