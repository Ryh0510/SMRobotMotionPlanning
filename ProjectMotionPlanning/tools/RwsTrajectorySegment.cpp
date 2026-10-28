#include <ProjectMotionPlanning/ZigzagTrajectorySegmentation.h>
#include <fstream>
#include <iostream>
int main(int argc,char** argv) {
    try {
        if(argc!=3) throw std::runtime_error("Usage: RwsTrajectorySegment trajectory.json new_output_directory");
        std::ifstream file(std::filesystem::u8path(argv[1]),std::ios::binary);if(!file)throw std::runtime_error("Cannot open trajectory");
        const std::string source((std::istreambuf_iterator<char>(file)),{});
        const auto result=motion_planning::segmentZigzagTrajectoryJson(source,std::filesystem::absolute(std::filesystem::u8path(argv[2])));
        std::cout<<result<<std::endl;return 0;
    }catch(const std::exception& e){std::cerr<<e.what()<<std::endl;return 1;}
}
