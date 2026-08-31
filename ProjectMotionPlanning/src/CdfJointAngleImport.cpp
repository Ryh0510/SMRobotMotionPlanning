#include <ProjectMotionPlanning/CdfJointAngleImport.h>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <fstream>
#include <sstream>

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
    std::vector<std::string> fields;
    std::string field;
    while(stream >> field) {
        fields.push_back(field);
    }
    return fields;
}

bool parseNumber(const std::string& token, double& value)
{
    try {
        std::size_t consumed = 0;
        value = std::stod(token, &consumed);
        return consumed == token.size() && std::isfinite(value);
    } catch(const std::exception&) {
        return false;
    }
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
    while(std::getline(file, line)) {
        ++lineNumber;
        line = trim(line);
        if(line.empty()) {
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
