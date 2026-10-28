#include <ProjectMotionPlanning/ZigzagTrajectorySegmentation.h>
#include <nlohmann/json.hpp>
#include <fstream>
#include <iostream>
#include <chrono>
using namespace motion_planning;
int main() try {
    int failures=0;auto check=[&](bool v,const char* text){std::cout<<(v?"PASS ":"FAIL ")<<text<<std::endl;if(!v)++failures;};
    std::vector<Eigen::Vector3d> p;
    for(int row=0;row<200;++row)for(int j=0;j<=10;++j) p.emplace_back(row%2?1.-j*.1:j*.1,row*.01,0);
    const auto segments=segmentZigzagTrajectory(p);
    check(segments.size()==200,"200 reciprocating scans produce 200 segments");
    std::size_t edges=0;bool coverage=segments.front().first==0 && segments.back().last==p.size()-1;
    for(std::size_t i=0;i<segments.size();++i){edges+=segments[i].last-segments[i].first;if(i)coverage=coverage && segments[i-1].last==segments[i].first;}
    check(coverage && edges==p.size()-1,"Shared endpoints cover every original edge and turn connector exactly once");
    std::vector<Eigen::Vector3d> noisy={{0,0,0},{.2,0,0},{.1999,0,0},{.4,0,0},{.4,0,0},{.6,0,0}};
    check(segmentZigzagTrajectory(noisy).size()==1,"Duplicates and sub-threshold backward noise do not create scans");
    bool rejected=false;try{segmentZigzagTrajectory({Eigen::Vector3d::Zero(),Eigen::Vector3d::Zero()});}catch(...){rejected=true;}
    check(rejected,"Stationary paths fail explicitly");
    const auto dir=std::filesystem::temp_directory_path()/("rws-segments-"+std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
    using J=nlohmann::json;J source={{"frame","user"},{"points",J::array()}};
    for(std::size_t i=0;i<p.size();++i)source["points"].push_back({{"position_m",{p[i].x(),p[i].y(),p[i].z()}},{"quaternion_xyzw",{0,0,0,1}},{"time_s",double(i)*.1},{"point_type",i%3},{"custom",i}});
    const auto manifest=J::parse(segmentZigzagTrajectoryJson(source.dump(),dir));
    J disk;std::ifstream(dir/"segment-2.json")>>disk;
    check(manifest["complete"]==true && disk["frame"]=="user" && disk["points"][0]==source["points"][manifest["segments"][1]["first"].get<int>()],"Persistence preserves times, roles, frames, poses and custom metadata");
    const auto cancelled=dir/"cancelled";bool stop=false;
    // Cancel after the first incrementally committed segment becomes visible.
    try{segmentZigzagTrajectoryJson(source.dump(),cancelled,{},[&](){return std::filesystem::exists(cancelled/"segment-1.json");});}catch(...){stop=true;}
    J partial;std::ifstream(cancelled/"manifest.json")>>partial;
    check(stop && partial["complete"]==false && partial["segments"].size()==1,"Cancellation retains completed segment and an honest partial manifest");
    std::filesystem::remove_all(dir);return failures?1:0;
}

catch(const std::exception& error) { std::cerr << "Segmentation regression error: " << error.what() << std::endl; return 1; }
