#include <ProjectMotionPlanning/ProjectMotionPlanning.h>
#include <SimulationProject/ProjectIo.h>
#include <iostream>
#include <exception>
int main(int argc,char** argv) {try {
    if(argc!=2)return 1;
    const auto path=std::filesystem::u8path(argv[1]);simulation_project::ProjectDocument doc;std::string error;
    if(!simulation_project::loadProjectDocument(path,doc,&error)){std::cerr<<error;return 2;}
    motion_planning::ProjectPlanningRequest request;request.robotId="Red4600";request.collisionDetectorIds={"default_collision"};
    auto scene=motion_planning::ProjectPlanningSceneBuilder::build(doc,path.parent_path(),request,&error);
    if(!scene){std::cerr<<error;return 3;}
    const auto* robot=scene->simulationRuntime().robot(request.robotId);auto original=robot->instance->getState().q;
    std::vector<double> values;
    for(const auto& name:scene->jointNames())values.push_back(original[robot->model.joints.at(robot->model.jointNameToIndex.at(name)).dofIndex]);
    collision::MotionQueryRequest budget;budget.maxEvaluations=0;
    // Explicitly declared geometry-free reference links only, preserving coverage checks.
    std::vector<std::string> freeLinks;
    for(const auto& link:robot->model.links)if(link.second.collisions.empty())freeLinks.push_back(link.first);
    scene->useConservativeMotionValidation(budget,freeLinks);
    const auto result=scene->validateMotion(values,values,request.validation);
    if(result.valid||result.diagnosticCode!="motion_indeterminate"){std::cerr<<"Uncertified edge accepted: "<<result.message;return 4;}
    if(robot->instance->getState().q!=original)return 5;
    const auto bad=scene->collisionRuntime().queryRobotJointMotion(scene->simulationRuntime(),"missing",{},{},{"missing"},{},{});
    if(bad.empty()||bad[0].verdict==collision::MotionVerdict::Clear)return 6;
    scene->useSampledMotionValidation();
    std::cout<<"Conservative planning rejects uncertified edges and preserves snapshot state.\n";return 0;
} catch(const std::exception& e){std::cerr<<e.what();return 7;} }
