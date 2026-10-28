#include <ProjectMotionPlanning/ZigzagTrajectorySegmentation.h>
#include <Eigen/Eigenvalues>
#include <nlohmann/json.hpp>
#include <cmath>
#include <chrono>
#include <thread>
#include <fstream>
#include <stdexcept>
#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#endif

namespace motion_planning {
std::vector<ZigzagTrajectorySegment> segmentZigzagTrajectory(
    const std::vector<Eigen::Vector3d>& p, const ZigzagSegmentationOptions& o,
    std::function<void(const ZigzagTrajectorySegment&)> ready, std::function<bool()> cancelled)
{
    if(p.size()<2 || !std::isfinite(o.directionAngleDegrees) || o.directionAngleDegrees<5 ||
       o.directionAngleDegrees>85 || !std::isfinite(o.minimumScanLength) || o.minimumScanLength<=0)
        throw std::runtime_error("Segmentation requires at least two points, angle 5..85 degrees and a positive minimum scan length.");
    Eigen::Matrix3d tensor=Eigen::Matrix3d::Zero();
    for(std::size_t i=0;i<p.size();++i) {
        if(!p[i].allFinite()) throw std::runtime_error("Non-finite trajectory position.");
        if(i) { const Eigen::Vector3d d=p[i]-p[i-1]; tensor+=d*d.transpose(); }
    }
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(tensor);
    if(eig.info()!=Eigen::Success || eig.eigenvalues().maxCoeff()<1e-18)
        throw std::runtime_error("Trajectory has no measurable scan direction.");
    const Eigen::Vector3d axis=eig.eigenvectors().col(2);
    const double cosine=std::cos(o.directionAngleDegrees*3.14159265358979323846/180.0);
    std::vector<ZigzagTrajectorySegment> result;
    std::size_t first=0, reversal=0;
    int direction=0, pending=0; double accumulated=0;
    auto emit=[&](std::size_t last) {
        ZigzagTrajectorySegment s; s.first=first;s.last=last;
        for(std::size_t j=first+1;j<=last;++j) s.length+=(p[j]-p[j-1]).norm();
        result.push_back(s); if(ready) ready(s); first=last;
    };
    for(std::size_t i=1;i<p.size();++i) {
        if(cancelled && cancelled()) throw std::runtime_error("Segmentation cancelled; completed segments retained.");
        const Eigen::Vector3d d=p[i]-p[i-1]; const double length=d.norm();
        if(length<1e-12 || std::abs(d.dot(axis))<cosine*length) continue;
        const int sign=d.dot(axis)>0?1:-1;
        if(!direction) {direction=sign;continue;}
        if(sign==direction) {pending=0;accumulated=0;continue;}
        if(pending!=sign) {pending=sign;accumulated=0;reversal=i-1;}
        accumulated+=std::abs(d.dot(axis));
        if(accumulated>=o.minimumScanLength) {
            if(reversal>first) emit(reversal);
            direction=sign;pending=0;accumulated=0;
        }
    }
    emit(p.size()-1);
    return result;
}

std::string segmentZigzagTrajectoryJson(const std::string& text,const std::filesystem::path& directory,
    const ZigzagSegmentationOptions& options,std::function<bool()> cancelled)
{
    using Json=nlohmann::json;
    const auto source=Json::parse(text); const auto& points=source.at("points");
    std::vector<Eigen::Vector3d> positions;
    for(const auto& p:points) {
        const auto v=p.at("position_m").get<std::vector<double>>();
        if(v.size()!=3) throw std::runtime_error("position_m must contain three coordinates.");
        positions.emplace_back(v[0],v[1],v[2]);
    }
    if(!directory.is_absolute()) throw std::runtime_error("Absolute segment output directory required.");
    std::filesystem::create_directories(directory);
    if(std::filesystem::exists(directory/"manifest.json")) throw std::runtime_error("Use a new segmentation output directory to preserve previous results.");
    const auto write=[&](const std::filesystem::path& path,const Json& value) {
        auto temporary=path;temporary+=".tmp";
        {std::ofstream f(temporary,std::ios::binary|std::ios::trunc);f<<value.dump(2);f.flush();if(!f)throw std::runtime_error("Cannot save trajectory segment.");}
        // Keep old completed artifacts until the replacement is fully written.
#ifdef _WIN32
        // Windows indexers/scanners may briefly hold the previous manifest.
        // Retry only transient access/lock errors and keep the old artifact
        // intact if replacement still fails. Never delete it as a fallback.
        for(int attempt=0;;++attempt) {
            if(MoveFileExW(temporary.c_str(),path.c_str(),MOVEFILE_REPLACE_EXISTING|MOVEFILE_WRITE_THROUGH)) break;
            const DWORD error=GetLastError();
            const bool transient=error==ERROR_ACCESS_DENIED || error==ERROR_SHARING_VIOLATION || error==ERROR_LOCK_VIOLATION;
            if(!transient || attempt>=19) {
                throw std::runtime_error("Cannot atomically replace segment artifact (Windows error " +
                    std::to_string(error) + "): " + path.u8string());
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
#else
        std::filesystem::rename(temporary,path);
#endif
    };
    Json manifest={{"version",1},{"method","reciprocating_scan_lines"},{"complete",false},
        {"source_point_count",points.size()},{"source",source},{"segments",Json::array()},
        {"direction_angle_deg",options.directionAngleDegrees},{"minimum_scan_length_m",options.minimumScanLength},
        {"boundary_policy","shared point; connector belongs to preceding scan; whole path verification required"}};
    write(directory/"manifest.json",manifest);
    segmentZigzagTrajectory(positions,options,[&](const ZigzagTrajectorySegment& s) {
        const auto index=manifest["segments"].size();const auto filename="segment-"+std::to_string(index+1)+".json";
        auto trajectory=source;trajectory["points"]=Json::array();
        for(auto i=s.first;i<=s.last;++i) trajectory["points"].push_back(points[i]);
        trajectory["source_range"]={s.first,s.last};
        write(directory/std::filesystem::u8path(filename),trajectory);
        manifest["segments"].push_back({{"index",index},{"first",s.first},{"last",s.last},
            {"point_count",s.last-s.first+1},{"length_m",s.length},{"file",filename}});
        write(directory/"manifest.json",manifest);
    },cancelled);
    manifest["complete"]=true;write(directory/"manifest.json",manifest);
    return manifest.dump();
}
}
