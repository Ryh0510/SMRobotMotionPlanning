#include <ProjectMotionPlanning/TrajectoryImport.h>

#include <nlohmann/json.hpp>

#include <Eigen/Geometry>

#include <algorithm>
#include <cctype>
#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iterator>
#include <regex>
#include <sstream>
#include <string>
#include <utility>

namespace motion_planning
{
    namespace
    {
        using Json = nlohmann::json;

        struct JointTable
        {
            std::vector<std::string> jointNames;
            robottrajectory::JointTrajectory trajectory;
        };

        std::string pathToString(const std::filesystem::path& path)
        {
            return path.generic_u8string();
        }

        std::string trim(std::string value)
        {
            const auto first = std::find_if_not(
                value.begin(),
                value.end(),
                [](unsigned char ch) { return std::isspace(ch) != 0; });
            const auto last = std::find_if_not(
                value.rbegin(),
                value.rend(),
                [](unsigned char ch) { return std::isspace(ch) != 0; }).base();
            if(first >= last) {
                return {};
            }
            value = std::string(first, last);
            if(value.size() >= 3 &&
                static_cast<unsigned char>(value[0]) == 0xEF &&
                static_cast<unsigned char>(value[1]) == 0xBB &&
                static_cast<unsigned char>(value[2]) == 0xBF) {
                value.erase(0, 3);
            }
            return value;
        }

        std::string lowerCopy(std::string value)
        {
            std::transform(value.begin(), value.end(), value.begin(), [](unsigned char ch) {
                return static_cast<char>(std::tolower(ch));
            });
            return value;
        }

        bool startsWith(const std::string& value, const std::string& prefix)
        {
            return value.size() >= prefix.size() &&
                std::equal(prefix.begin(), prefix.end(), value.begin());
        }

        bool isCommentOrEmpty(const std::string& line)
        {
            const std::string value = trim(line);
            return value.empty() ||
                startsWith(value, "#") ||
                startsWith(value, "//") ||
                startsWith(value, "!");
        }

        std::vector<std::string> splitTokens(const std::string& line)
        {
            std::string normalized;
            normalized.reserve(line.size());
            for(char ch : line) {
                switch(ch) {
                case ',':
                case ';':
                case '\t':
                case '[':
                case ']':
                case '(':
                case ')':
                    normalized.push_back(' ');
                    break;
                default:
                    normalized.push_back(ch);
                    break;
                }
            }

            std::istringstream stream(normalized);
            std::vector<std::string> tokens;
            std::string token;
            while(stream >> token) {
                while(!token.empty() &&
                    (token.back() == ':' || token.back() == '=' || token.back() == '\"' ||
                        token.back() == '\'')) {
                    token.pop_back();
                }
                while(!token.empty() && (token.front() == '\"' || token.front() == '\'')) {
                    token.erase(token.begin());
                }
                if(!token.empty()) {
                    tokens.push_back(token);
                }
            }
            return tokens;
        }

        bool parseDouble(const std::string& token, double& value)
        {
            std::string cleaned = trim(token);
            while(!cleaned.empty() &&
                (cleaned.back() == ',' || cleaned.back() == ';' || cleaned.back() == ']')) {
                cleaned.pop_back();
            }
            while(!cleaned.empty() && (cleaned.front() == '[' || cleaned.front() == '(')) {
                cleaned.erase(cleaned.begin());
            }
            if(cleaned.empty()) {
                return false;
            }

            errno = 0;
            char* end = nullptr;
            const double parsed = std::strtod(cleaned.c_str(), &end);
            if(end == cleaned.c_str() || errno == ERANGE) {
                return false;
            }
            while(end != nullptr && *end != '\0') {
                if(std::isspace(static_cast<unsigned char>(*end)) == 0) {
                    return false;
                }
                ++end;
            }
            value = parsed;
            return std::isfinite(value);
        }

        std::vector<double> numericValues(const std::string& line)
        {
            std::vector<double> values;
            for(const std::string& token : splitTokens(line)) {
                double value = 0.0;
                if(parseDouble(token, value)) {
                    values.push_back(value);
                }
            }
            return values;
        }

        std::vector<double> preciseMatrixRowValues(const std::string& line)
        {
            std::vector<std::string> parts;
            {
                std::istringstream stream(line);
                std::string part;
                while(stream >> part) {
                    parts.push_back(part);
                }
            }

            std::vector<double> values;
            if(parts.size() >= 4) {
                values.reserve(parts.size());
                bool allParsed = true;
                for(std::string part : parts) {
                    part.erase(
                        std::remove(part.begin(), part.end(), ','),
                        part.end());
                    double value = 0.0;
                    if(!parseDouble(part, value)) {
                        allParsed = false;
                        break;
                    }
                    values.push_back(value);
                }
                if(allParsed && values.size() >= 4) {
                    return values;
                }
            }

            return numericValues(line);
        }

        bool allTokensNumeric(const std::vector<std::string>& tokens)
        {
            if(tokens.empty()) {
                return false;
            }
            for(const std::string& token : tokens) {
                double value = 0.0;
                if(!parseDouble(token, value)) {
                    return false;
                }
            }
            return true;
        }

        std::vector<std::string> readLines(
            const std::filesystem::path& path,
            std::string& errorMessage)
        {
            std::ifstream input(path);
            if(!input) {
                errorMessage = "Failed to open trajectory file: " + pathToString(path);
                return {};
            }

            std::vector<std::string> lines;
            std::string line;
            while(std::getline(input, line)) {
                lines.push_back(line);
            }
            return lines;
        }

        bool readTextFile(
            const std::filesystem::path& path,
            std::string& content,
            std::string& errorMessage)
        {
            std::ifstream input(path);
            if(!input) {
                errorMessage = "Failed to open trajectory file: " + pathToString(path);
                return false;
            }
            std::ostringstream stream;
            stream << input.rdbuf();
            content = stream.str();
            return true;
        }

        TrajectoryImportResult failure(std::string code, std::string message)
        {
            TrajectoryImportResult result;
            result.diagnostics.push_back({ std::move(code), std::move(message) });
            return result;
        }

        void addWarning(
            TrajectoryImportResult& result,
            std::string code,
            std::string message)
        {
            result.diagnostics.push_back({ std::move(code), std::move(message) });
        }

        double normalizedCartesianPositionScale(const TrajectoryImportOptions& options)
        {
            return std::isfinite(options.cartesianPositionScale) && options.cartesianPositionScale > 0.0
                ? options.cartesianPositionScale
                : 1.0;
        }

        std::string sanitizedId(std::string value)
        {
            value = trim(value);
            for(char& ch : value) {
                const unsigned char c = static_cast<unsigned char>(ch);
                if(std::isalnum(c) == 0 && ch != '_' && ch != '-') {
                    ch = '_';
                }
            }
            value.erase(
                std::remove(value.begin(), value.end(), '.'),
                value.end());
            if(value.empty()) {
                value = "imported_trajectory";
            }
            return value;
        }

        std::vector<std::string> generatedJointNames(std::size_t count)
        {
            std::vector<std::string> result;
            result.reserve(count);
            for(std::size_t index = 0; index < count; ++index) {
                result.push_back("joint_" + std::to_string(index + 1));
            }
            return result;
        }

        bool looksLikeTimeColumn(const std::vector<std::vector<double>>& rows)
        {
            if(rows.size() < 2 || rows.front().empty()) {
                return false;
            }
            if(std::abs(rows.front().front()) > 1.0e-9) {
                return false;
            }
            bool changed = false;
            double previous = rows.front().front();
            for(std::size_t index = 1; index < rows.size(); ++index) {
                if(rows[index].empty() || rows[index].front() < previous) {
                    return false;
                }
                changed = changed || rows[index].front() > previous;
                previous = rows[index].front();
            }
            return changed;
        }

        std::size_t firstNumericRowColumnCount(const std::vector<std::string>& lines)
        {
            for(const std::string& rawLine : lines) {
                if(isCommentOrEmpty(rawLine)) {
                    continue;
                }
                const std::vector<double> values = numericValues(rawLine);
                if(!values.empty()) {
                    return values.size();
                }
            }
            return 0U;
        }

        bool matchesSelectedJointShape(
            const std::size_t columnCount,
            const TrajectoryImportOptions& options)
        {
            if(columnCount == 0U || options.jointNames.empty()) {
                return false;
            }
            return columnCount == options.jointNames.size() ||
                columnCount == options.jointNames.size() + 1U;
        }

        bool parseJointTable(
            const std::vector<std::string>& lines,
            const TrajectoryImportOptions& options,
            JointTable& table,
            std::string& errorMessage)
        {
            std::vector<std::vector<std::string>> tokenRows;
            for(const std::string& rawLine : lines) {
                if(isCommentOrEmpty(rawLine)) {
                    continue;
                }
                const std::vector<std::string> tokens = splitTokens(rawLine);
                if(!tokens.empty()) {
                    tokenRows.push_back(tokens);
                }
            }
            if(tokenRows.empty()) {
                errorMessage = "Trajectory file contains no table rows.";
                return false;
            }

            bool hasHeader = !allTokensNumeric(tokenRows.front());
            bool hasTimeColumn = false;
            std::vector<std::string> jointNames;
            std::size_t firstDataRow = 0;
            if(hasHeader) {
                const std::vector<std::string>& header = tokenRows.front();
                if(header.empty()) {
                    errorMessage = "Trajectory header is empty.";
                    return false;
                }
                const std::string first = lowerCopy(header.front());
                hasTimeColumn = first == "time" || first == "t" || first == "timestamp";
                const std::size_t firstJointColumn = hasTimeColumn ? 1 : 0;
                for(std::size_t index = firstJointColumn; index < header.size(); ++index) {
                    jointNames.push_back(header[index]);
                }
                firstDataRow = 1;
            }

            std::vector<std::vector<double>> numericRows;
            for(std::size_t rowIndex = firstDataRow; rowIndex < tokenRows.size(); ++rowIndex) {
                std::vector<double> row;
                row.reserve(tokenRows[rowIndex].size());
                for(const std::string& token : tokenRows[rowIndex]) {
                    double value = 0.0;
                    if(!parseDouble(token, value)) {
                        errorMessage = "Trajectory table contains non-numeric data: " + token;
                        return false;
                    }
                    row.push_back(value);
                }
                if(!row.empty()) {
                    numericRows.push_back(std::move(row));
                }
            }

            if(numericRows.empty()) {
                errorMessage = "Trajectory table contains no numeric points.";
                return false;
            }

            const std::size_t columnCount = numericRows.front().size();
            for(const std::vector<double>& row : numericRows) {
                if(row.size() != columnCount) {
                    errorMessage = "Trajectory table rows do not have a consistent column count.";
                    return false;
                }
            }

            if(!hasHeader) {
                if(!options.jointNames.empty() && columnCount == options.jointNames.size() + 1) {
                    hasTimeColumn = true;
                } else if(!options.jointNames.empty() && columnCount == options.jointNames.size()) {
                    hasTimeColumn = false;
                } else {
                    hasTimeColumn = columnCount > 1 && looksLikeTimeColumn(numericRows);
                }
            }

            const std::size_t jointValueCount = hasTimeColumn ? columnCount - 1 : columnCount;
            if(jointValueCount == 0) {
                errorMessage = "Trajectory table has no joint value columns.";
                return false;
            }

            if(jointNames.empty()) {
                jointNames = options.jointNames.size() == jointValueCount
                    ? options.jointNames
                    : generatedJointNames(jointValueCount);
            }
            if(jointNames.size() != jointValueCount) {
                errorMessage = "Trajectory joint names do not match the imported joint value count.";
                return false;
            }

            robottrajectory::JointTrajectory trajectory;
            trajectory.interpolation = robottrajectory::TrajectoryInterpolation::Linear;
            trajectory.points.reserve(numericRows.size());
            for(std::size_t rowIndex = 0; rowIndex < numericRows.size(); ++rowIndex) {
                const std::vector<double>& row = numericRows[rowIndex];
                robottrajectory::TimedJointPoint point;
                point.time = hasTimeColumn
                    ? row.front()
                    : static_cast<double>(rowIndex) * std::max(1.0e-6, options.defaultTimeStep);
                point.q.assign(
                    row.begin() + static_cast<std::ptrdiff_t>(hasTimeColumn ? 1 : 0),
                    row.end());
                trajectory.points.push_back(std::move(point));
            }
            trajectory.sortByTime();

            table.jointNames = std::move(jointNames);
            table.trajectory = std::move(trajectory);
            return true;
        }

        bool isFinitePose(const Eigen::Isometry3d& pose)
        {
            const Eigen::Matrix4d matrix = pose.matrix();
            for(int row = 0; row < 4; ++row) {
                for(int column = 0; column < 4; ++column) {
                    if(!std::isfinite(matrix(row, column))) {
                        return false;
                    }
                }
            }
            return true;
        }

        Eigen::Isometry3d makePose(const Eigen::Matrix4d& matrix)
        {
            Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
            pose.matrix() = matrix;
            return pose;
        }

        Eigen::Isometry3d makePoseFromPositionQuaternion(
            double x,
            double y,
            double z,
            double w,
            double qx,
            double qy,
            double qz)
        {
            Eigen::Quaterniond quaternion(w, qx, qy, qz);
            if(quaternion.norm() > 1.0e-12) {
                quaternion.normalize();
            } else {
                quaternion = Eigen::Quaterniond::Identity();
            }

            Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
            pose.linear() = quaternion.toRotationMatrix();
            pose.translation() = Eigen::Vector3d(x, y, z);
            return pose;
        }

        void appendCartesianPoint(
            robottrajectory::CartesianTrajectory& trajectory,
            double time,
            const Eigen::Isometry3d& pose,
            double positionScale = 1.0)
        {
            Eigen::Isometry3d scaledPose = pose;
            scaledPose.translation() *= positionScale;
            if(!isFinitePose(scaledPose)) {
                return;
            }
            robottrajectory::TimedCartesianPoint point;
            point.time = time;
            point.tcpPose = scaledPose;
            trajectory.points.push_back(std::move(point));
        }

        bool parseStructuredCartesianText(
            const std::vector<std::string>& lines,
            const TrajectoryImportOptions& options,
            robottrajectory::CartesianTrajectory& trajectory)
        {
            bool foundStructuredMarker = false;
            bool readingMatrix = false;
            int matrixRow = 0;
            Eigen::Matrix4d matrix = Eigen::Matrix4d::Identity();
            std::vector<Eigen::Matrix4d> poses;

            for(const std::string& rawLine : lines) {
                const std::string line = trim(rawLine);
                if(line.empty()) {
                    continue;
                }
                if(line.find("=== Point") != std::string::npos) {
                    foundStructuredMarker = true;
                    readingMatrix = false;
                    matrixRow = 0;
                    matrix = Eigen::Matrix4d::Identity();
                    continue;
                }
                if(line.find("Pose (4x4 matrix)") != std::string::npos ||
                    line.find("Backup Point") != std::string::npos) {
                    foundStructuredMarker = true;
                    readingMatrix = true;
                    matrixRow = 0;
                    matrix = Eigen::Matrix4d::Identity();
                    continue;
                }
                if(!readingMatrix) {
                    continue;
                }

                const std::vector<double> values = preciseMatrixRowValues(line);
                if(values.size() < 4) {
                    continue;
                }
                for(int column = 0; column < 4; ++column) {
                    matrix(matrixRow, column) = values[static_cast<std::size_t>(column)];
                }
                ++matrixRow;
                if(matrixRow == 4) {
                    poses.push_back(matrix);
                    readingMatrix = false;
                    matrixRow = 0;
                    matrix = Eigen::Matrix4d::Identity();
                }
            }

            if(poses.empty()) {
                return false;
            }

            const double positionScale = normalizedCartesianPositionScale(options);
            const double totalTime = std::max(
                options.defaultDuration,
                static_cast<double>(poses.size()) * std::max(1.0e-6, options.defaultTimeStep));
            const double timeInterval = totalTime / static_cast<double>(poses.size());
            for(std::size_t index = 0; index < poses.size(); ++index) {
                appendCartesianPoint(
                    trajectory,
                    static_cast<double>(index) * timeInterval,
                    makePose(poses[index]),
                    positionScale);
            }
            return foundStructuredMarker && !trajectory.points.empty();
        }

        bool parseGenericMatrixText(
            const std::vector<std::string>& lines,
            const TrajectoryImportOptions& options,
            robottrajectory::CartesianTrajectory& trajectory)
        {
            int matrixRow = 0;
            double currentTime = 0.0;
            bool hasPendingTime = false;
            Eigen::Matrix4d matrix = Eigen::Matrix4d::Identity();
            const double positionScale = normalizedCartesianPositionScale(options);

            for(const std::string& rawLine : lines) {
                if(isCommentOrEmpty(rawLine)) {
                    continue;
                }
                const std::vector<double> values = preciseMatrixRowValues(rawLine);
                if(values.empty()) {
                    continue;
                }
                if(values.size() == 1 && matrixRow == 0) {
                    currentTime = values.front();
                    hasPendingTime = true;
                    continue;
                }
                if(values.size() < 4) {
                    matrixRow = 0;
                    matrix = Eigen::Matrix4d::Identity();
                    hasPendingTime = false;
                    continue;
                }
                for(int column = 0; column < 4; ++column) {
                    matrix(matrixRow, column) = values[static_cast<std::size_t>(column)];
                }
                ++matrixRow;
                if(matrixRow == 4) {
                    const double pointTime = hasPendingTime
                        ? currentTime
                        : static_cast<double>(trajectory.points.size()) *
                            std::max(1.0e-6, options.defaultTimeStep);
                    appendCartesianPoint(trajectory, pointTime, makePose(matrix), positionScale);
                    matrixRow = 0;
                    matrix = Eigen::Matrix4d::Identity();
                    hasPendingTime = false;
                }
            }

            return !trajectory.points.empty();
        }

        bool parseNumericPoseRows(
            const std::vector<std::string>& lines,
            const TrajectoryImportOptions& options,
            robottrajectory::CartesianTrajectory& trajectory)
        {
            const double positionScale = normalizedCartesianPositionScale(options);
            for(const std::string& rawLine : lines) {
                if(isCommentOrEmpty(rawLine)) {
                    continue;
                }
                const std::vector<double> values = numericValues(rawLine);
                if(values.size() == 8) {
                    const double time = values[7] == 50.0
                        ? static_cast<double>(trajectory.points.size())
                        : values[7];
                    appendCartesianPoint(
                        trajectory,
                        time,
                        makePoseFromPositionQuaternion(
                            values[0], values[1], values[2],
                            values[3], values[4], values[5], values[6]),
                        positionScale);
                } else if(values.size() >= 12) {
                    const double time = values[9] == 0.0
                        ? static_cast<double>(trajectory.points.size())
                        : values[9];
                    appendCartesianPoint(
                        trajectory,
                        time,
                        makePoseFromPositionQuaternion(
                            values[0], values[1], values[2],
                            values[3], values[4], values[5], values[6]),
                        positionScale);
                }
            }
            return !trajectory.points.empty();
        }

        bool parseCartesianText(
            const std::vector<std::string>& lines,
            const TrajectoryImportOptions& options,
            robottrajectory::CartesianTrajectory& trajectory)
        {
            trajectory = robottrajectory::CartesianTrajectory();
            trajectory.interpolation = robottrajectory::TrajectoryInterpolation::Linear;
            if(parseStructuredCartesianText(lines, options, trajectory)) {
                trajectory.sortByTime();
                return true;
            }
            if(parseGenericMatrixText(lines, options, trajectory)) {
                trajectory.sortByTime();
                return true;
            }
            if(parseNumericPoseRows(lines, options, trajectory)) {
                trajectory.sortByTime();
                return true;
            }
            return false;
        }

        bool parseKfBinary(
            const std::filesystem::path& path,
            const TrajectoryImportOptions& options,
            robottrajectory::CartesianTrajectory& trajectory,
            std::string& errorMessage)
        {
            std::ifstream input(path, std::ios::binary);
            if(!input) {
                errorMessage = "Failed to open keyframe file: " + pathToString(path);
                return false;
            }

            std::size_t count = 0;
            input.read(reinterpret_cast<char*>(&count), sizeof(count));
            if(!input || count > 1000000) {
                errorMessage = "Invalid keyframe file header.";
                return false;
            }

            trajectory = robottrajectory::CartesianTrajectory();
            trajectory.interpolation = robottrajectory::TrajectoryInterpolation::Linear;
            trajectory.points.reserve(count);
            const double positionScale = normalizedCartesianPositionScale(options);
            for(std::size_t index = 0; index < count; ++index) {
                float timestamp = 0.0f;
                Eigen::Matrix4f matrix = Eigen::Matrix4f::Identity();
                input.read(reinterpret_cast<char*>(&timestamp), sizeof(timestamp));
                input.read(
                    reinterpret_cast<char*>(matrix.data()),
                    static_cast<std::streamsize>(matrix.size() * sizeof(float)));
                if(!input) {
                    errorMessage = "Keyframe file ended before all poses were read.";
                    return false;
                }
                appendCartesianPoint(
                    trajectory,
                    static_cast<double>(timestamp),
                    makePose(matrix.cast<double>()),
                    positionScale);
            }
            trajectory.sortByTime();
            return !trajectory.points.empty();
        }

        bool parseAbbTargets(
            const std::filesystem::path& path,
            const TrajectoryImportOptions& options,
            robottrajectory::CartesianTrajectory& trajectory,
            std::string& errorMessage)
        {
            std::string content;
            if(!readTextFile(path, content, errorMessage)) {
                return false;
            }

            const std::regex targetRegex(
                R"(CONST\s+robtarget\s+([A-Za-z_][A-Za-z0-9_]*)\s*:=\s*\[\s*\[\s*([-+0-9.Ee]+)\s*,\s*([-+0-9.Ee]+)\s*,\s*([-+0-9.Ee]+)\s*\]\s*,\s*\[\s*([-+0-9.Ee]+)\s*,\s*([-+0-9.Ee]+)\s*,\s*([-+0-9.Ee]+)\s*,\s*([-+0-9.Ee]+)\s*\])",
                std::regex::icase);

            trajectory = robottrajectory::CartesianTrajectory();
            trajectory.interpolation = robottrajectory::TrajectoryInterpolation::Linear;
            const double positionScale = normalizedCartesianPositionScale(options);
            for(std::sregex_iterator it(content.begin(), content.end(), targetRegex), end;
                it != end;
                ++it) {
                const std::smatch& match = *it;
                const std::string targetName = match.str(1);
                double timestamp = static_cast<double>(trajectory.points.size() + 1) * 10.0;
                const std::regex numberRegex(R"((\d+))");
                std::smatch numberMatch;
                if(std::regex_search(targetName, numberMatch, numberRegex)) {
                    timestamp = std::stod(numberMatch.str(1));
                }

                appendCartesianPoint(
                    trajectory,
                    timestamp,
                    makePoseFromPositionQuaternion(
                        std::stod(match.str(2)),
                        std::stod(match.str(3)),
                        std::stod(match.str(4)),
                        std::stod(match.str(5)),
                        std::stod(match.str(6)),
                        std::stod(match.str(7)),
                        std::stod(match.str(8))),
                    positionScale);
            }
            trajectory.sortByTime();
            if(trajectory.points.empty()) {
                errorMessage = "No ABB robtarget definitions were found.";
                return false;
            }
            return true;
        }

        Eigen::Isometry3d readPoseJson(const Json& json)
        {
            Eigen::Matrix4d matrix = Eigen::Matrix4d::Identity();
            if(json.is_array() && json.size() >= 4 && json.front().is_array()) {
                for(std::size_t row = 0; row < 4 && row < json.size(); ++row) {
                    for(std::size_t column = 0; column < 4 && column < json.at(row).size(); ++column) {
                        matrix(static_cast<int>(row), static_cast<int>(column)) =
                            json.at(row).at(column).get<double>();
                    }
                }
            } else if(json.is_array() && json.size() >= 16) {
                for(std::size_t index = 0; index < 16; ++index) {
                    matrix(static_cast<int>(index / 4), static_cast<int>(index % 4)) =
                        json.at(index).get<double>();
                }
            }
            return makePose(matrix);
        }

        bool jsonScaleValue(const Json& json, double& scale)
        {
            static constexpr const char* kScaleKeys[] = {
                "cartesianPositionScale",
                "positionScale",
                "linearPositionScale"
            };
            for(const char* key : kScaleKeys) {
                if(json.contains(key) && json.at(key).is_number()) {
                    scale = json.at(key).get<double>();
                    return std::isfinite(scale) && scale > 0.0;
                }
            }
            return false;
        }

        bool jsonUnitValue(const Json& json, std::string& unit)
        {
            static constexpr const char* kUnitKeys[] = {
                "cartesianPositionUnit",
                "positionUnit",
                "linearPositionUnit",
                "unit"
            };
            for(const char* key : kUnitKeys) {
                if(json.contains(key) && json.at(key).is_string()) {
                    unit = lowerCopy(json.at(key).get<std::string>());
                    return true;
                }
            }
            if(json.contains("units") && json.at("units").is_object()) {
                return jsonUnitValue(json.at("units"), unit);
            }
            return false;
        }

        double jsonCartesianPositionScale(
            const Json& json,
            const Json* parentJson = nullptr,
            double fallbackScale = 1.0)
        {
            double scale = 1.0;
            if(json.is_object() && jsonScaleValue(json, scale)) {
                return scale;
            }
            if(parentJson != nullptr && parentJson->is_object() && jsonScaleValue(*parentJson, scale)) {
                return scale;
            }

            std::string unit;
            if(json.is_object() && jsonUnitValue(json, unit)) {
                if(unit == "mm" || unit == "millimeter" || unit == "millimeters") {
                    return 0.001;
                }
                if(unit == "m" || unit == "meter" || unit == "meters") {
                    return 1.0;
                }
            }
            if(parentJson != nullptr && parentJson->is_object() && jsonUnitValue(*parentJson, unit)) {
                if(unit == "mm" || unit == "millimeter" || unit == "millimeters") {
                    return 0.001;
                }
                if(unit == "m" || unit == "meter" || unit == "meters") {
                    return 1.0;
                }
            }
            return fallbackScale;
        }

        robottrajectory::JointTrajectory readJointTrajectoryJson(const Json& json)
        {
            robottrajectory::JointTrajectory trajectory;
            trajectory.name = json.value("name", std::string());
            trajectory.interpolation = json.value("interpolation", std::string("linear")) == "step"
                ? robottrajectory::TrajectoryInterpolation::Step
                : robottrajectory::TrajectoryInterpolation::Linear;
            if(!json.contains("points") || !json.at("points").is_array()) {
                return trajectory;
            }
            for(const Json& pointJson : json.at("points")) {
                robottrajectory::TimedJointPoint point;
                if(pointJson.is_array()) {
                    if(pointJson.empty()) {
                        continue;
                    }
                    point.time = pointJson.size() > 1 ? pointJson.at(0).get<double>() : 0.0;
                    const std::size_t firstJoint = pointJson.size() > 1 ? 1 : 0;
                    for(std::size_t index = firstJoint; index < pointJson.size(); ++index) {
                        point.q.push_back(pointJson.at(index).get<double>());
                    }
                } else if(pointJson.is_object()) {
                    point.time = pointJson.value("time", 0.0);
                    point.q = pointJson.value("q", std::vector<double>());
                    point.qd = pointJson.value("qd", std::vector<double>());
                    point.qdd = pointJson.value("qdd", std::vector<double>());
                }
                if(!point.q.empty()) {
                    trajectory.points.push_back(std::move(point));
                }
            }
            trajectory.sortByTime();
            return trajectory;
        }

        robottrajectory::CartesianTrajectory readCartesianTrajectoryJson(
            const Json& json,
            double positionScale = 1.0)
        {
            robottrajectory::CartesianTrajectory trajectory;
            trajectory.name = json.value("name", std::string());
            trajectory.interpolation = json.value("interpolation", std::string("linear")) == "step"
                ? robottrajectory::TrajectoryInterpolation::Step
                : robottrajectory::TrajectoryInterpolation::Linear;
            if(!json.contains("points") || !json.at("points").is_array()) {
                return trajectory;
            }
            for(const Json& pointJson : json.at("points")) {
                if(!pointJson.is_object() || !pointJson.contains("tcpPose")) {
                    continue;
                }
                appendCartesianPoint(
                    trajectory,
                    pointJson.value("time", static_cast<double>(trajectory.points.size())),
                    readPoseJson(pointJson.at("tcpPose")),
                    positionScale);
            }
            trajectory.sortByTime();
            return trajectory;
        }

        const Json* selectPlanJson(const Json& root, const TrajectoryImportOptions& options)
        {
            if(!root.is_object() || !root.contains("plans") || !root.at("plans").is_array()) {
                return &root;
            }
            const Json& plans = root.at("plans");
            if(plans.empty()) {
                return nullptr;
            }
            if(!options.robotId.empty()) {
                for(const Json& plan : plans) {
                    if(plan.is_object() && plan.value("robotId", std::string()) == options.robotId) {
                        return &plan;
                    }
                }
            }
            return &plans.front();
        }

        TrajectoryImportResult finalizeImportedPlan(
            const std::filesystem::path& path,
            const TrajectoryImportOptions& options,
            StoredMotionPlan plan,
            TrajectoryImportDataKind kind)
        {
            TrajectoryImportResult result;
            const std::string effectiveRobotId = plan.robotId.empty()
                ? options.robotId
                : plan.robotId;
            const std::string generatedBaseId = effectiveRobotId.empty()
                ? path.stem().generic_u8string()
                : effectiveRobotId + "_" + path.stem().generic_u8string();
            plan.id = !options.planId.empty()
                ? sanitizedId(options.planId)
                : sanitizedId(generatedBaseId);
            plan.name = !options.planName.empty()
                ? options.planName
                : path.stem().generic_u8string();
            if(plan.name.empty()) {
                plan.name = plan.id;
            }
            if(plan.robotId.empty()) {
                plan.robotId = effectiveRobotId;
            }
            if(plan.robotId.empty()) {
                return failure("robot_required", "Select a robot or import a file that declares robotId.");
            }
            if(plan.jointNames.empty() && !options.jointNames.empty()) {
                plan.jointNames = options.jointNames;
            }
            if(!plan.trajectory.empty() && plan.jointNames.empty()) {
                const std::size_t jointCount = plan.trajectory.points.front().q.size();
                plan.jointNames = generatedJointNames(jointCount);
                addWarning(
                    result,
                    "generated_joint_names",
                    "Imported trajectory did not declare joint names; generated generic joint names.");
            }
            if(!plan.trajectory.empty()) {
                plan.trajectory.name = plan.name;
            }
            if(!plan.cartesianControlPoints.empty()) {
                plan.cartesianControlPoints.name = plan.name;
            }

            result.success = true;
            result.dataKind = kind;
            result.plan = std::move(plan);
            return result;
        }

        TrajectoryImportResult importJsonFile(
            const std::filesystem::path& path,
            const TrajectoryImportOptions& options)
        {
            std::string content;
            std::string errorMessage;
            if(!readTextFile(path, content, errorMessage)) {
                return failure("open_failed", errorMessage);
            }

            Json root;
            try {
                root = Json::parse(content);
            } catch(const std::exception& exception) {
                return failure("json_parse_failed", exception.what());
            }

            const Json* selected = selectPlanJson(root, options);
            if(selected == nullptr || !selected->is_object()) {
                return failure("json_plan_missing", "JSON file does not contain an importable trajectory plan.");
            }
            const Json& planJson = *selected;

            if(planJson.contains("trajectory") &&
                planJson.at("trajectory").is_object() &&
                planJson.at("trajectory").contains("trajectory_file")) {
                const std::filesystem::path referenced =
                    path.parent_path() / planJson.at("trajectory").value("trajectory_file", std::string());
                TrajectoryImportOptions nestedOptions = options;
                return ProjectTrajectoryImporter::importFile(referenced.lexically_normal(), nestedOptions);
            }
            if(planJson.contains("trajectory_file")) {
                const std::filesystem::path referenced =
                    path.parent_path() / planJson.value("trajectory_file", std::string());
                return ProjectTrajectoryImporter::importFile(referenced.lexically_normal(), options);
            }

            StoredMotionPlan plan;
            plan.robotId = planJson.value("robotId", std::string());
            plan.jointNames = planJson.value("jointNames", std::vector<std::string>());

            TrajectoryImportDataKind kind = TrajectoryImportDataKind::Unknown;
            if(planJson.contains("trajectory") && planJson.at("trajectory").is_object()) {
                plan.trajectory = readJointTrajectoryJson(planJson.at("trajectory"));
                if(!plan.trajectory.empty()) {
                    kind = TrajectoryImportDataKind::JointTrajectory;
                }
            }
            if(planJson.contains("cartesianControlPoints") &&
                planJson.at("cartesianControlPoints").is_object()) {
                const double positionScale =
                    jsonCartesianPositionScale(planJson.at("cartesianControlPoints"), &planJson);
                plan.cartesianControlPoints =
                    readCartesianTrajectoryJson(planJson.at("cartesianControlPoints"), positionScale);
                if(!plan.cartesianControlPoints.empty() &&
                    kind == TrajectoryImportDataKind::Unknown) {
                    kind = TrajectoryImportDataKind::CartesianControlPoints;
                }
            }
            if(planJson.contains("points") && planJson.at("points").is_array()) {
                if(!planJson.at("points").empty() &&
                    planJson.at("points").front().is_object() &&
                    planJson.at("points").front().contains("tcpPose")) {
                    plan.cartesianControlPoints =
                        readCartesianTrajectoryJson(
                            planJson,
                            jsonCartesianPositionScale(
                                planJson,
                                nullptr,
                                normalizedCartesianPositionScale(options)));
                    kind = TrajectoryImportDataKind::CartesianControlPoints;
                } else {
                    plan.trajectory = readJointTrajectoryJson(planJson);
                    kind = TrajectoryImportDataKind::JointTrajectory;
                }
            }

            if(plan.trajectory.empty() && plan.cartesianControlPoints.empty()) {
                return failure("json_trajectory_missing", "JSON file does not contain joint trajectory or cartesian control points.");
            }
            return finalizeImportedPlan(path, options, std::move(plan), kind);
        }

        TrajectoryImportResult importJointTableFile(
            const std::filesystem::path& path,
            const TrajectoryImportOptions& options,
            const std::vector<std::string>& lines)
        {
            JointTable table;
            std::string errorMessage;
            if(!parseJointTable(lines, options, table, errorMessage)) {
                return failure("joint_table_parse_failed", errorMessage);
            }

            StoredMotionPlan plan;
            plan.robotId = options.robotId;
            plan.jointNames = std::move(table.jointNames);
            plan.trajectory = std::move(table.trajectory);
            return finalizeImportedPlan(
                path,
                options,
                std::move(plan),
                TrajectoryImportDataKind::JointTrajectory);
        }

        TrajectoryImportResult importCartesianFile(
            const std::filesystem::path& path,
            const TrajectoryImportOptions& options,
            robottrajectory::CartesianTrajectory trajectory)
        {
            if(trajectory.empty()) {
                return failure("cartesian_points_missing", "No valid cartesian trajectory control points were imported.");
            }

            StoredMotionPlan plan;
            plan.robotId = options.robotId;
            plan.jointNames = options.jointNames;
            plan.cartesianControlPoints = std::move(trajectory);
            return finalizeImportedPlan(
                path,
                options,
                std::move(plan),
                TrajectoryImportDataKind::CartesianControlPoints);
        }
    }

    std::string TrajectoryImportResult::message() const
    {
        return diagnostics.empty() ? std::string() : diagnostics.front().message;
    }

    TrajectoryImportResult ProjectTrajectoryImporter::importFile(
        const std::filesystem::path& path,
        const TrajectoryImportOptions& options)
    {
        if(path.empty()) {
            return failure("path_required", "Trajectory import path is empty.");
        }
        std::error_code existsError;
        if(!std::filesystem::exists(path, existsError) || existsError) {
            return failure("path_not_found", "Trajectory file does not exist: " + pathToString(path));
        }

        const std::string extension = lowerCopy(path.extension().generic_u8string());
        if(extension == ".json") {
            return importJsonFile(path, options);
        }

        if(extension == ".kf") {
            robottrajectory::CartesianTrajectory trajectory;
            std::string errorMessage;
            if(!parseKfBinary(path, options, trajectory, errorMessage)) {
                return failure("kf_parse_failed", errorMessage);
            }
            return importCartesianFile(path, options, std::move(trajectory));
        }

        if(extension == ".mod") {
            robottrajectory::CartesianTrajectory trajectory;
            std::string errorMessage;
            if(!parseAbbTargets(path, options, trajectory, errorMessage)) {
                return failure("abb_parse_failed", errorMessage);
            }
            return importCartesianFile(path, options, std::move(trajectory));
        }

        std::string errorMessage;
        std::vector<std::string> lines = readLines(path, errorMessage);
        if(lines.empty() && !errorMessage.empty()) {
            return failure("open_failed", errorMessage);
        }

        if(extension == ".csv") {
            return importJointTableFile(path, options, lines);
        }

        robottrajectory::CartesianTrajectory cartesianTrajectory;
        if(parseStructuredCartesianText(lines, options, cartesianTrajectory)) {
            cartesianTrajectory.sortByTime();
            return importCartesianFile(path, options, std::move(cartesianTrajectory));
        }

        const bool selectedJointShape =
            matchesSelectedJointShape(firstNumericRowColumnCount(lines), options);
        if(!selectedJointShape && parseCartesianText(lines, options, cartesianTrajectory)) {
            return importCartesianFile(path, options, std::move(cartesianTrajectory));
        }

        TrajectoryImportResult jointResult = importJointTableFile(path, options, lines);
        if(jointResult.success) {
            return jointResult;
        }

        if(parseCartesianText(lines, options, cartesianTrajectory)) {
            return importCartesianFile(path, options, std::move(cartesianTrajectory));
        }

        return failure(
            "unsupported_trajectory_format",
            "Trajectory file is not a supported joint table, pose matrix, ABB robtarget, JSON, or KF file.");
    }
}
