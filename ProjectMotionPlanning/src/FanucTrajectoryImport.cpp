#include "FanucTrajectoryImport.h"
#include <nlohmann/json.hpp>
#include <Eigen/Geometry>
#include <fstream>
#include <sstream>
#include <regex>
#include <map>
#include <cmath>
#include <stdexcept>

namespace motion_planning { namespace detail {
TrajectoryImportResult importFanucLs(const std::filesystem::path& path, const TrajectoryImportOptions& options)
{
    TrajectoryImportResult result;
    try {
        std::ifstream file(path, std::ios::binary);
        if(!file) throw std::runtime_error("Cannot open FANUC LS file.");
        const std::string text((std::istreambuf_iterator<char>(file)), {});
        const auto mn = text.find("/MN"), pos = text.find("/POS"), end = text.find("/END");
        if(mn == std::string::npos || pos == std::string::npos || end == std::string::npos || !(mn < pos && pos < end))
            throw std::runtime_error("FANUC LS requires /MN, /POS and /END sections.");
        if(!std::isfinite(options.defaultTimeStep) || options.defaultTimeStep <= 0)
            throw std::runtime_error("Default time step must be positive.");
        struct Position { Eigen::Isometry3d pose = Eigen::Isometry3d::Identity(); int uf=0, ut=0; std::string config; };
        std::map<int, Position> positions;
        const std::string section = text.substr(pos + 4, end - pos - 4);
        const std::regex block(R"(P\[\s*(\d+)(?::[^\]]*)?\s*\]\s*\{([^}]*)\}\s*;)");
        const std::string number = R"(([+-]?(?:\d+(?:\.\d*)?|\.\d+)(?:[Ee][+-]?\d+)?))";
        for(auto it=std::sregex_iterator(section.begin(),section.end(),block); it!=std::sregex_iterator(); ++it) {
            const int id=std::stoi((*it)[1]); const std::string body=(*it)[2]; Position p;
            if(body.find("GP1:")==std::string::npos || std::regex_search(body,std::regex(R"(GP[2-9]\s*:|\b(?:J|E)\d+\s*=)")))
                throw std::runtime_error("Only GP1 Cartesian positions without external axes are supported: P["+std::to_string(id)+"].");
            std::smatch match;
            if(!std::regex_search(body,match,std::regex(R"(UF\s*:\s*(\d+)\s*,\s*UT\s*:\s*(\d+))")))
                throw std::runtime_error("Missing UF/UT in P["+std::to_string(id)+"].");
            p.uf=std::stoi(match[1]); p.ut=std::stoi(match[2]);
            if(std::regex_search(body,match,std::regex(R"(CONFIG\s*:\s*'([^']*)')"))) p.config=match[1];
            double values[6]; const char* names[]={"X","Y","Z","W","P","R"};
            for(int axis=0;axis<6;++axis) {
                if(!std::regex_search(body,match,std::regex(std::string("\\b")+names[axis]+"\\s*=\\s*"+number+"\\s*"+(axis<3?"mm":"deg"))))
                    throw std::runtime_error("Missing/invalid Cartesian coordinate in P["+std::to_string(id)+"].");
                values[axis]=std::stod(match[1]);
                if(!std::isfinite(values[axis])) throw std::runtime_error("Non-finite LS coordinate.");
            }
            p.pose.translation()=Eigen::Vector3d(values[0],values[1],values[2])*.001;
            constexpr double radians=3.14159265358979323846/180.;
            p.pose.linear()=(Eigen::AngleAxisd(values[5]*radians,Eigen::Vector3d::UnitZ()) *
                Eigen::AngleAxisd(values[4]*radians,Eigen::Vector3d::UnitY()) *
                Eigen::AngleAxisd(values[3]*radians,Eigen::Vector3d::UnitX())).toRotationMatrix();
            if(!positions.emplace(id,p).second) throw std::runtime_error("Duplicate LS position P["+std::to_string(id)+"].");
        }
        const std::regex motion(R"(^L\s+P\[\s*(\d+)(?::[^\]]*)?\s*\]\s+([0-9]+(?:\.[0-9]*)?)\s*mm/sec\s+(FINE|CNT\d+)\s*$)",std::regex::icase);
        const std::regex frame(R"(^(UFRAME_NUM|UTOOL_NUM)\s*=\s*(\d+)\s*$)");
        int activeUf=-1,activeUt=-1,referenceUf=-1,referenceUt=-1;
        nlohmann::json moves=nlohmann::json::array();
        std::istringstream instructions(text.substr(mn+3,pos-mn-3)); std::string line;
        while(std::getline(instructions,line)) {
            const auto colon=line.find(':'); if(colon==std::string::npos) continue;
            const std::string lineNumber=line.substr(0,colon);
            std::string command=line.substr(colon+1);
            const auto semicolon=command.find(';'); if(semicolon!=std::string::npos) command.resize(semicolon);
            const auto first=command.find_first_not_of(" \t\r"); if(first==std::string::npos) continue;
            command=command.substr(first); command.resize(command.find_last_not_of(" \t\r")+1);
            if(command[0]=='!') continue;
            std::smatch match;
            if(std::regex_match(command,match,frame)) {
                (match[1]=="UFRAME_NUM"?activeUf:activeUt)=std::stoi(match[2]); continue;
            }
            if(!std::regex_match(command,match,motion))
                throw std::runtime_error("Unsupported LS instruction at line "+lineNumber+": "+command+". Import requires straight L/P Cartesian moves.");
            const int id=std::stoi(match[1]); const auto found=positions.find(id);
            if(found==positions.end()) throw std::runtime_error("Undefined LS position P["+std::to_string(id)+"].");
            const auto& p=found->second;
            if((activeUf>=0 && activeUf!=p.uf) || (activeUt>=0 && activeUt!=p.ut))
                throw std::runtime_error("Active UF/UT differs from the referenced position.");
            if(referenceUf<0) {referenceUf=p.uf;referenceUt=p.ut;}
            if(referenceUf!=p.uf || referenceUt!=p.ut)
                throw std::runtime_error("Mixed UF/UT requires explicit coordinate conversion before importing.");
            const double speed=std::stod(match[2])*.001;
            if(!std::isfinite(speed) || speed<=0) throw std::runtime_error("LS speed must be positive.");
            robottrajectory::TimedCartesianPoint point; point.tcpPose=p.pose; point.linearSpeed=speed;
            const auto& points=result.plan.cartesianControlPoints.points;
            if(!points.empty()) {
                const double distance=(point.tcpPose.translation()-points.back().tcpPose.translation()).norm();
                point.time=points.back().time+(distance>1e-12 ? distance/speed : options.defaultTimeStep);
            } else point.time=0.;
            result.plan.cartesianControlPoints.points.push_back(point);
            moves.push_back({{"point",id},{"line",std::stoi(lineNumber)},{"uf",p.uf},{"ut",p.ut},
                {"config",p.config},{"speed_mm_s",speed*1000.},{"termination",match[3].str()}});
        }
        if(moves.empty()) throw std::runtime_error("No Cartesian L moves found in FANUC LS.");
        result.plan.robotId=options.robotId; result.plan.jointNames=options.jointNames;
        result.plan.optimizationMetadata=nlohmann::json({{"motion","cartesian_linear"},{"fanuc_ls",{
            {"source",path.filename().generic_u8string()},{"coordinate_interpretation","raw_uf_like_txt"},
            {"timing","nominal_distance_over_commanded_speed"},{"stationary_dt_s",options.defaultTimeStep},{"moves",moves}}}}).dump();
        result.success=true; result.dataKind=TrajectoryImportDataKind::CartesianControlPoints;
        result.diagnostics.push_back({"fanuc_ls_nominal","FANUC LS imported in /MN order; WPR=Rz(R)*Ry(P)*Rx(W). Raw UF coordinates use the same convention as TXT; UF/UT definitions are not included. Times are nominal length/feed (zero-distance moves use default step), not controller FINE/CNT execution times."});
    } catch(const std::exception& e) {result={};result.diagnostics.push_back({"fanuc_ls_parse_failed",e.what()});}
    return result;
}
}}
