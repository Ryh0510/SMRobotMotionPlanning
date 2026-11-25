#include "TrajectoryTextMetadata.h"
#include <cmath>
#include <stdexcept>

namespace motion_planning { namespace detail {
void exportTrajectoryProcessMetadata(const StoredMotionPlan& plan,nlohmann::json& header) {
    const auto metadata=nlohmann::json::parse(plan.optimizationMetadata,nullptr,false);
    if(metadata.is_object() && metadata.contains("spray_enabled")) {
        auto& execution=header["spray_execution"];execution=nlohmann::json::object();
        for(const char* key:{"spray_enabled","source_stations","spray_axis"})
            if(metadata.contains(key))execution[key]=metadata[key];
    }
    if(!plan.processMetadata.empty())header["process_metadata"]=nlohmann::json::parse(plan.processMetadata);
}
void importTrajectoryProcessMetadata(const nlohmann::json& header,StoredMotionPlan& plan,std::size_t count) {
    auto metadata=nlohmann::json::parse(plan.optimizationMetadata,nullptr,false);
    if(!metadata.is_object())metadata=nlohmann::json::object();
    if(header.contains("spray_execution")) {
        const auto& execution=header.at("spray_execution");
        const auto& flags=execution.at("spray_enabled");const auto& stations=execution.at("source_stations");
        if(!flags.is_array() || !stations.is_array() || flags.size()!=count || stations.size()!=count)
            throw std::runtime_error("Spray execution point count mismatch.");
        double previous=-1.;
        for(std::size_t i=0;i<count;++i) {
            if(!flags[i].is_boolean() || !stations[i].is_number())throw std::runtime_error("Invalid spray execution row.");
            const double station=stations[i].get<double>();
            if(!std::isfinite(station) || station<0 || station<=previous)throw std::runtime_error("Invalid source station order.");
            previous=station;
        }
        metadata["spray_enabled"]=flags;metadata["source_stations"]=stations;
        if(execution.contains("spray_axis")) {
            const auto axis=execution.at("spray_axis").get<std::vector<double>>();
            if(axis.size()!=3 || !std::isfinite(axis[0]+axis[1]+axis[2]) ||
                std::abs(axis[0]*axis[0]+axis[1]*axis[1]+axis[2]*axis[2]-1)>1e-8)
                throw std::runtime_error("Invalid spray axis.");
            metadata["spray_axis"]=axis;
        }
    }
    if(header.contains("process_metadata")) {
        const auto& process=header.at("process_metadata");
        if(!process.is_object() || process.value("schema",std::string())!="smrobot.planning-process.v1" ||
            !process.contains("points") || !process.at("points").is_array() || process.at("points").size()!=count)
            throw std::runtime_error("Process metadata point count/schema mismatch.");
        plan.processMetadata=process.dump();
    }
    plan.optimizationMetadata=metadata.dump();
}
}}
