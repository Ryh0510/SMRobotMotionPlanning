#include <ProjectMotionPlanning/CdfJointAngleImport.h>

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <locale>

namespace motion_planning
{
namespace
{

constexpr double kPi = 3.14159265358979323846;

std::string trim(const std::string& text)
{
    const auto first = std::find_if_not(
        text.begin(),
        text.end(),
        [](unsigned char ch) { return std::isspace(ch) != 0; });
    const auto last = std::find_if_not(
        text.rbegin(),
        text.rend(),
        [](unsigned char ch) { return std::isspace(ch) != 0; }).base();
    if(first >= last) {
        return {};
    }
    return std::string(first, last);
}

std::string lowerText(std::string text)
{
    std::transform(
        text.begin(),
        text.end(),
        text.begin(),
        [](unsigned char ch) {
            return static_cast<char>(std::tolower(ch));
        });
    return text;
}

std::vector<std::string> splitFields(std::string line)
{
    for(char& ch : line) {
        if(ch == ',' || ch == ';' || ch == '\t') {
            ch = ' ';
        }
    }

    std::istringstream stream(line);
    stream.imbue(std::locale::classic());
    std::vector<std::string> fields;
    std::string field;
    while(stream >> field) {
        fields.push_back(field);
    }
    return fields;
}

bool parseNumber(const std::string& token, double& value)
{
    std::istringstream input(token);
    input.imbue(std::locale::classic());
    input >> value;
    return input && input.peek() == std::char_traits<char>::eof() && std::isfinite(value);
}

bool isNumericRow(const std::vector<std::string>& fields)
{
    if(fields.size() < 2) {
        return false;
    }
    for(const std::string& field : fields) {
        double value = 0.0;
        if(!parseNumber(field, value)) {
            return false;
        }
    }
    return true;
}

std::vector<std::string> jointNamesFromHeader(const std::vector<std::string>& fields)
{
    std::vector<std::string> names;
    if(fields.size() < 2) {
        return names;
    }

    const std::string first = lowerText(fields.front());
    if(first.find("time") == std::string::npos) {
        return {};
    }

    names.reserve(fields.size() - 1);
    for(std::size_t index = 1; index < fields.size(); ++index) {
        std::string name = fields[index];
        const std::string lower = lowerText(name);
        const std::size_t degreeSuffix = lower.find("_deg");
        const std::size_t radianSuffix = lower.find("_rad");
        if(degreeSuffix != std::string::npos) {
            name = name.substr(0, degreeSuffix);
        } else if(radianSuffix != std::string::npos) {
            name = name.substr(0, radianSuffix);
        }
        if(name.empty()) {
            name = "J" + std::to_string(index);
        }
        names.push_back(name);
    }
    return names;
}

std::vector<std::string> defaultJointNames(std::size_t count)
{
    std::vector<std::string> names;
    names.reserve(count);
    for(std::size_t index = 0; index < count; ++index) {
        names.push_back("J" + std::to_string(index + 1));
    }
    return names;
}

void appendDiagnostic(CdfJointAngleImportResult& result, const std::string& message)
{
    result.diagnostics.push_back(message);
}

} // namespace

std::string CdfJointAngleImportResult::message() const
{
    if(success) {
        return "Imported " + std::to_string(points.size()) + " CDF joint angle points.";
    }
    if(!diagnostics.empty()) {
        return diagnostics.front();
    }
    return "CDF joint angle import failed.";
}

CdfJointAngleImportResult ProjectCdfJointAngleImporter::importFile(const std::filesystem::path& path)
{
    CdfJointAngleImportResult result;
    result.sourceName = path.filename().string();

    std::ifstream file(path);
    if(!file.is_open()) {
        appendDiagnostic(result, "Failed to open CDF joint angle file.");
        return result;
    }

    std::string line;
    std::size_t lineNumber = 0;
    std::size_t expectedFieldCount = 0;
    nlohmann::json exact;
    bool hasMarker = false;
    while(std::getline(file, line)) {
        ++lineNumber;
        line = trim(line);
        if(line.empty()) {
            continue;
        }

        const std::string marker = "# RWS_JOINT_TABLE_V1 ";
        if(line.rfind("# RWS_JOINT_TABLE", 0) == 0) {
            if(hasMarker || line.rfind(marker, 0) != 0) {
                appendDiagnostic(result, "Duplicate or unsupported joint table metadata.");
                return result;
            }
            hasMarker = true;
            exact = nlohmann::json::parse(line.substr(marker.size()), nullptr, false);
            if(!exact.is_object()) {
                appendDiagnostic(result, "Malformed joint table metadata.");
                return result;
            }
            continue;
        }

        const std::string lower = lowerText(line);
        if(lower.find("joint angle unit") != std::string::npos) {
            if(lower.find("rad") != std::string::npos) {
                result.sourceUnit = CdfJointAngleUnit::Radians;
            } else if(lower.find("deg") != std::string::npos) {
                result.sourceUnit = CdfJointAngleUnit::Degrees;
            }
            continue;
        }

        if(!line.empty() && line.front() == '#') {
            continue;
        }

        const std::vector<std::string> fields = splitFields(line);
        if(fields.empty()) {
            continue;
        }

        if(!isNumericRow(fields)) {
            const std::vector<std::string> names = jointNamesFromHeader(fields);
            if(!names.empty()) {
                const bool radians = std::all_of(fields.begin() + 1, fields.end(), [](const auto& field) {
                    const auto lower = lowerText(field);
                    return lower.size() >= 4 && lower.substr(lower.size() - 4) == "_rad";
                });
                const bool degrees = std::all_of(fields.begin() + 1, fields.end(), [](const auto& field) {
                    const auto lower = lowerText(field);
                    return lower.size() >= 4 && lower.substr(lower.size() - 4) == "_deg";
                });
                if(radians) result.sourceUnit = CdfJointAngleUnit::Radians;
                if(degrees) result.sourceUnit = CdfJointAngleUnit::Degrees;
                result.jointNames = names;
                expectedFieldCount = result.jointNames.size() + 1;
            }
            continue;
        }

        if(expectedFieldCount == 0) {
            expectedFieldCount = fields.size();
        }
        if(fields.size() != expectedFieldCount) {
            appendDiagnostic(
                result,
                "Line " + std::to_string(lineNumber) +
                    " has " + std::to_string(fields.size()) +
                    " fields; expected " + std::to_string(expectedFieldCount) + ".");
            continue;
        }

        CdfJointAnglePoint point;
        if(!parseNumber(fields.front(), point.timeSeconds)) {
            appendDiagnostic(result, "Line " + std::to_string(lineNumber) + " has an invalid timestamp.");
            continue;
        }

        point.jointAnglesDegrees.reserve(fields.size() - 1);
        bool rowValid = true;
        for(std::size_t index = 1; index < fields.size(); ++index) {
            double value = 0.0;
            if(!parseNumber(fields[index], value)) {
                appendDiagnostic(
                    result,
                    "Line " + std::to_string(lineNumber) +
                        " has an invalid joint angle value.");
                rowValid = false;
                break;
            }
            if(result.sourceUnit == CdfJointAngleUnit::Radians) {
                value = value * 180.0 / kPi;
            }
            point.jointAnglesDegrees.push_back(value);
        }

        if(rowValid) {
            result.points.push_back(std::move(point));
        }
    }

    if(result.points.empty()) {
        appendDiagnostic(result, "The CDF joint angle file does not contain any numeric joint angle rows.");
        return result;
    }

    const std::size_t jointCount = result.points.front().jointAnglesDegrees.size();
    if(result.jointNames.empty()) {
        result.jointNames = defaultJointNames(jointCount);
    }
    if(result.jointNames.size() != jointCount) {
        appendDiagnostic(result, "CDF joint name count does not match joint angle count.");
        result.success = false;
        return result;
    }

    if(hasMarker) {
        try {
            const auto names = exact.at("joint_names").get<std::vector<std::string>>();
            const auto motion = exact.at("motion").get<std::string>();
            const auto convention = exact.at("joint_convention").get<std::string>();
            const auto& rows = exact.at("points");
            if(exact.at("kind") != "joint_table_v1" || names.size() != jointCount ||
                !rows.is_array() || rows.size() != result.points.size() ||
                (motion != "cartesian_linear" && motion != "joint_linear") ||
                (convention != "runtime" && convention != "irb4600_stored" && convention != "caller") ||
                !result.diagnostics.empty() || result.sourceUnit != CdfJointAngleUnit::Degrees) {
                throw std::runtime_error("Invalid joint table metadata, units or row count.");
            }
            for(std::size_t j = 0; j < jointCount; ++j) {
                if(names[j].empty() || result.jointNames[j] != "J" + std::to_string(j + 1) ||
                    std::find(names.begin(), names.begin() + j, names[j]) != names.begin() + j)
                    throw std::runtime_error("Joint table header or names do not match metadata.");
            }
            result.plan.robotId = exact.at("robot_id").get<std::string>();
            result.plan.jointNames = names;
            result.plan.trajectory.interpolation = robottrajectory::TrajectoryInterpolation::Linear;
            for(std::size_t i = 0; i < rows.size(); ++i) {
                robottrajectory::TimedJointPoint point;
                point.time = rows[i].at("time").get<double>();
                point.q = rows[i].at("q").get<std::vector<double>>();
                if(!std::isfinite(point.time) || point.q.size() != jointCount ||
                    (i && point.time <= result.plan.trajectory.points.back().time) ||
                    std::abs(point.time - result.points[i].timeSeconds) > 5.01e-7)
                    throw std::runtime_error("Joint table timestamps do not match metadata.");
                for(std::size_t j = 0; j < jointCount; ++j) {
                    const double degrees = point.q[j] * 180.0 / kPi;
                    if(!std::isfinite(degrees) ||
                        std::abs(degrees - result.points[i].jointAnglesDegrees[j]) > 5.01e-7)
                        throw std::runtime_error("Joint table angles do not match metadata.");
                    result.points[i].jointAnglesDegrees[j] = degrees;
                }
                result.points[i].timeSeconds = point.time;
                result.plan.trajectory.points.push_back(std::move(point));
            }
            result.plan.optimizationMetadata = nlohmann::json{
                {"motion", motion}, {"joint_convention", convention}}.dump();
            result.jointNames = names;
            result.hasMotionMetadata = true;
        } catch(const std::exception& error) {
            result.plan = {};
            appendDiagnostic(result, std::string("Joint table provenance rejected: ") + error.what());
            return result;
        }
    }

    std::sort(
        result.points.begin(),
        result.points.end(),
        [](const CdfJointAnglePoint& lhs, const CdfJointAnglePoint& rhs) {
            return lhs.timeSeconds < rhs.timeSeconds;
        });

    result.success = true;
    return result;
}

} // namespace motion_planning
