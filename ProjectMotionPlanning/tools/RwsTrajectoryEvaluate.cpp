#include <ProjectMotionPlanning/ExternalPlanningJson.h>
#include <ProjectMotionPlanning/VirtualBaffleEnvironment.h>
#include <SimulationRuntime/ProjectSimulationRuntime.h>
#include <SimulationProject/ProjectIO.h>
#include <nlohmann/json.hpp>
#include <fstream>
#include <iostream>
#include <chrono>
int evaluateFile(int argc,char** argv)
{
    if(argc!=4 && argc!=5) {std::cerr<<"Usage: RwsTrajectoryEvaluate project.sys.json request.json result.json [progress.jsonl]\n";return 2;}
    nlohmann::json report;
    try {
        const auto project=std::filesystem::absolute(std::filesystem::u8path(argv[1]));
        if(std::string(argv[2])=="--baffle-diagnostics") {
            simulation_runtime::ProjectSimulationRuntime runtime;
            const auto loaded=runtime.loadProjectFile(project);
            if(!loaded.success)throw std::runtime_error(loaded.message);
            motion_planning::VirtualBaffleEnvironment baffles;baffles.build(runtime);std::string reason;
            const bool valid=baffles.validState(runtime,0,reason);report=baffles.status();report["success"]=report.value("error",std::string()).empty();report["state_valid"]=valid;report["state_diagnostic"]=reason;
        } else {
        // runExternalPlanningJson builds its own isolated real-model scene.
        // Loading another full simulation runtime here duplicated mesh/collision
        // construction for every stage of every scan line.
        simulation_project::ProjectDocument document;std::string loadError;
        if(!simulation_project::loadProjectDocument(project,document,&loadError))throw std::runtime_error(loadError);
        std::ifstream input(std::filesystem::u8path(argv[2]));
        if(!input) throw std::runtime_error("Cannot read request file.");
        const std::string request((std::istreambuf_iterator<char>(input)),{});
        std::ofstream progressFile;
        if(argc==5) {
            progressFile.open(std::filesystem::u8path(argv[4]));
            if(!progressFile)throw std::runtime_error("Cannot write progress file.");
        }
        const auto started=std::chrono::steady_clock::now();
        report=nlohmann::json::parse(motion_planning::runExternalPlanningJson(document,
            project.parent_path(),request,std::filesystem::absolute(std::filesystem::u8path(argv[2])).parent_path(),{},
            [&](const std::string& message) {
                if(!progressFile.is_open())return;
                progressFile<<nlohmann::json{{"elapsed_s",std::chrono::duration<double>(std::chrono::steady_clock::now()-started).count()},
                    {"message",message}}.dump()<<'\n'<<std::flush;
            }));
        }
    } catch(const std::exception& e) {report={{"success",false},{"retry_workpiece_pose",false},{"error",e.what()}};}
    std::ofstream output(std::filesystem::u8path(argv[3]));
    if(!output) return 2;
    output<<report.dump(2)<<'\n';
    if(!output) return 2;
    return report.value("success",false)?0:(report.contains("error")?2:1);
}

int main(int argc,char** argv)
{
    // Offline SDK sessions may reuse libraries and immutable mesh caches.
    // Every request still constructs a fresh isolated planning scene and validator.
    if(argc==3 && std::string(argv[2])=="--server") {
        std::string line;
        while(std::getline(std::cin,line)) {
            try {
                const auto command=nlohmann::json::parse(line);
                std::vector<std::string> arguments={argv[0],argv[1],command.at("request").get<std::string>(),command.at("result").get<std::string>()};
                if(command.contains("progress"))arguments.push_back(command.at("progress").get<std::string>());
                std::vector<char*> raw;for(auto& value:arguments)raw.push_back(value.data());
                const int status=evaluateFile(int(raw.size()),raw.data());
                std::cout<<"RWS_EVALUATION_DONE "<<nlohmann::json{{"exit_code",status}}.dump()<<std::endl;
            }catch(const std::exception& error) {
                std::cout<<"RWS_EVALUATION_DONE "<<nlohmann::json{{"exit_code",2},{"error",error.what()}}.dump()<<std::endl;
            }
        }
        return 0;
    }
    return evaluateFile(argc,argv);
}
