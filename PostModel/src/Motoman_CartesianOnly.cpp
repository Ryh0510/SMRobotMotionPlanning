#include <PostModel/Motoman_CartesianOnly.h>
#include <PostModel/robomath.h>
#include <PostModel/ftpsend.h>
#include <algorithm>
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <algorithm>
#include <cmath>
#include <vector>
#include <map>
#include <cctype>
#include <windows.h>
#include <shellapi.h>
#include <cstdio>
#pragma execution_character_set("utf-8")

namespace Motoman_CartesianOnly {
    using namespace robomath;
    // 常量定义
    const std::vector<std::string> MOTOMAN_RESERVED_NAMES = {
        "PROG", "ATTR", "OWNER", "COMMENT", "CREATE", "MODIFIED", "FILE_NAME",
        "VERSION", "LINE_COUNT", "MEMORY_SIZE", "PROTECT", "TCD", "DEFAULT_GROUP"
    };

    // 工具函数
    std::string RobotPost::getSafeName(const std::string& name) {
        int max_chars = config_.MAX_NAME_CHARS;
        std::string result = name;
        const std::string forbidden = " -[]/\\;,><&*:%=+@!#^()|?^";

        // 移除非法字符
        for (char c : forbidden) {
            result.erase(std::remove(result.begin(), result.end(), c), result.end());
        }

        // 确保非空
        if (result.empty()) {
            result = "Program";
        }

        // 确保不以数字开头
        if (!result.empty() && std::isdigit(result[0])) {
            result = "P" + result;
        }

        // 限制最大字符数
        if (result.size() > max_chars && max_chars >= 4) {
            result = result.substr(0, max_chars);
        }
        else if (result.size() > max_chars) {
            result = result.substr(0, 8);
        }

        return result;
    }

    std::vector<double> RobotPost::Pose_2_Motoman(const robomath::Mat& pose) {
        Vector xyzrpw = pose_2_xyzrpw(pose);
        return { xyzrpw[0], xyzrpw[1], xyzrpw[2], xyzrpw[3], xyzrpw[4], xyzrpw[5] };
    }

    void RobotPost::setPulsesXDeg(const std::vector<double>& pulses) {
        if (!pulses.empty()) {
            config_.PULSES_X_DEG = pulses;
            // 确保向量大小足够，用默认值填充
            config_.PULSES_X_DEG.resize(10, 1.0);
        }
    }
    // 构造函数
    RobotPost::RobotPost(const std::string robotPost, const std::string robotName, int robotAxes, std::vector<std::string> axesType, const std::string ip_com, const Config config)
        : robotPost_(robotPost), robotName_(robotName), robotAxes_(robotAxes), axesType_(axesType), ip_com_(ip_com), config_(config) {
        // 添加缺失的初始化
        progName_ = "unknown";
        progNameCurrent_ = "unknown";
        nPages_ = 0;
        nProgs_ = 0;
        lastMoveType_ = -1;
        poseFrame_ = robomath::Mat::Mat();
        speedBackup_ = "";
        // 初始化LAST_CONFDATA的C++等价物
        lastConfData_ = { "", "", "", "" };

        // 初始化轴类型相关变量
        hasTrack_ = false;
        hasTurntable_ = false;
        axesTrack_.clear();
        axesTurntable_.clear();
        // 初始化轴类型
        for (size_t i = 0; i < axesType_.size(); ++i) {
            if (axesType_[i] == "T") {
                axesTrack_.push_back(i);
                hasTrack_ = true;
            }
            else if (axesType_[i] == "J") {
                axesTurntable_.push_back(i);
                hasTurntable_ = true;
            }
        }

        poseFrame_ = robomath::Mat::Mat();
    }

    // 程序控制方法
    void RobotPost::ProgStart(const std::string& progname, bool new_page) {
        std::string safeName = getSafeName(progname);
        std::string progname_i = safeName;

        lastMoveType_ = -1; // 重要：重置运动类型标志

        if (new_page) {
            if (nPages_ == 0) {
                if (!progNamesMain_.empty()) {
                    addLog("Can't split " + progname + ": Two or more programs are split into smaller programs");
                    throw std::runtime_error("Only one program at a time can be split into smaller programs");
                }
                progNamesMain_.push_back(progName_);
                nPages_++;
            }
            nPages_++;
            progname_i = progName_ + std::to_string(nPages_);
            progNamesMain_.push_back(progname_i);
        }
        else {
            if (nProgs_ > 1 && !config_.INCLUDE_SUB_PROGRAMS) {
                return;
            }
            progName_ = safeName;
            nProgs_++;
        }

        progNameCurrent_ = progname_i;
        progNames_.push_back(progname_i);

        // 重置当前程序数据
        currentProg_ = ProgramData(progname_i);
        currentProg_.lineCount = 0;
        currentProg_.pCount = 0;
        currentProg_.cCount = 0;
        currentProg_.ecCount = 0;
    }

    void RobotPost::ProgFinish(const std::string& progname, bool new_page) {

        if (!new_page) {
            nPages_ = 0;
        }

        if (nProgs_ > 1 && !config_.INCLUDE_SUB_PROGRAMS) {
            return;
        }

        std::string progname_used = currentProg_.name;

        if (nPages_ == 0) {
            nPages_ = 0;
        }

        // 构建程序头 - 匹配Python版本格式
        std::string header = "/JOB\n";
        header += "//NAME " + progname_used + "\n";
        header += "//POS\n";

        // 修改NPOS格式：Python版本是 cCount, bcCount, ecCount, pCount
        header += "///NPOS " + std::to_string(currentProg_.cCount) + "," +
            std::to_string(currentProg_.bcCount) + "," +
            std::to_string(currentProg_.ecCount) + "," +
            std::to_string(currentProg_.pCount) + ",0,0";

        // 插入目标部分
        if (!currentProg_.targets.empty()) {
            header += "\n";
            for (const auto& target : currentProg_.targets) {
                header += target + "\n";
            }
        }

        // 构建指令头 - 匹配Python版本
        std::string header_ins = "//INST\n";

        // 使用当前时间，而不是固定日期
        header_ins += "///DATE " + getCurrentTimestamp() + "\n";
        header_ins += "///COMM Generated using Hejun\n";  

        if (config_.USE_RELATIVE_JOB) {
            header_ins += "///ATTR SC,RW,RJ\n";
            if (config_.USE_MFRAME && config_.ACTIVE_FRAME >= 0) {
                header_ins += "////FRAME USER " + std::to_string(config_.ACTIVE_FRAME) + "\n";
            }
        }
        else {
            header_ins += "///ATTR SC,RW\n";
        }

        header_ins += "///GROUP1 RB1";
        if (hasTrack_) {
            header_ins += ",BS1";  // 添加直线轴组
        }
        header_ins += "\n";

        if (hasTurntable_) {
            header_ins += "///GROUP2 ST1\n";
        }
        header_ins += "NOP";

        // 合并所有部分 - 注意顺序要与Python版本一致
        std::vector<std::string> fullProgram;
        fullProgram.push_back(header);

        // 添加BC目标点（如果有）
        if (!currentProg_.targetsBC.empty()) {
            for (const auto& targetBC : currentProg_.targetsBC) {
                fullProgram.push_back(targetBC);
            }
        }

        // 添加EC目标点（如果有）
        if (!currentProg_.targetsEC.empty()) {
            for (const auto& targetEC : currentProg_.targetsEC) {
                fullProgram.push_back(targetEC);
            }
        }

        fullProgram.push_back(header_ins);
        for (const auto& line : currentProg_.lines) {
            fullProgram.push_back(line);
        }
        fullProgram.push_back("END");

        currentProg_.lines = fullProgram;
        progList_.push_back(currentProg_);

        // 重置当前程序
        currentProg_ = ProgramData();
        nPages_ = 0;
        lastConfData_ = { "", "", "", "" };
    }

    void RobotPost::ProgSave(const std::string& folder, const std::string& progname, bool askUser, const std::string& showResult) {
        std::string safeName = getSafeName(progname);

        if (progList_.size() >= 1) {
            if (currentProg_.lineCount > 0) {
                addLog("Warning: ProgFinish was not called properly");
                ProgFinish(currentProg_.name);
            }

            if (progNamesMain_.size() > 1) {
                std::string progname_main = "M_" + progNamesMain_[0];
                ProgramData mainProg(progname_main);

                for (const auto& prog_call : progNamesMain_) {
                    mainProg.lines.push_back("CALL JOB:" + prog_call);
                }

                progList_.push_back(mainProg);
                progNames_.push_back(progname_main);
            }

            // 保存所有程序
            for (size_t i = 0; i < progList_.size(); ++i) {
                saveProgram(folder, progList_[i].name, askUser, showResult);
            }
        }
        else {
            if (!currentProg_.name.empty()) {
                saveProgram(folder, currentProg_.name, askUser, showResult);
            }
            else {
                saveProgram(folder, safeName, askUser, showResult);
            }
        }

        if (!showResult.empty() && !log_.empty()) {
            MessageBoxA(NULL, ("Program generation LOG:\n\n" + log_).c_str(),
                "Program Log", MB_OK | MB_ICONINFORMATION);
        }
    }

    void RobotPost::ProgSendRobot(const std::string& remote_path, const std::string& ftp_user, const std::string& ftp_pass) {
        std::cout << "Motoman program send functionality would be implemented here" << std::endl;
        addLog("Program send to robot not fully implemented for Motoman");
    }

    // 运动指令
    void RobotPost::MoveJ(const robomath::Mat& pose, const std::vector<double>& joints, const std::vector<double>& confRLF) {
        pageSizeControl();

        auto target_ids = (pose != robomath::Mat() && config_.MOVEJ_IN_CARTESIAN)
            ? addTargetCartesian(poseFrame_ * pose, joints, confRLF)
            : addTargetJoints(joints);

        int target_id = std::get<0>(target_ids);
        int target_bc = std::get<1>(target_ids);
        int target_ec = std::get<2>(target_ids);

        std::string bc_modifier = "";
        if (target_bc != -1) {
            std::stringstream bc_ss;
            bc_ss << std::setw(5) << std::setfill('0') << target_bc;
            bc_modifier = " BC" + bc_ss.str();
        }

        std::stringstream target_ss;
        target_ss << std::setw(5) << std::setfill('0') << target_id;
        std::string target_str = target_ss.str();

        if (target_ec == -1) {
            addLine("MOVJ C" + target_str + bc_modifier + " " + strVJ_ + strPL_, 'J');
        }
        else if (config_.EXTAXES_USE_MOVJ) {
            std::stringstream ec_ss;
            ec_ss << std::setw(5) << std::setfill('0') << target_ec;
            addLine("MOVJ C" + target_str + bc_modifier + " " + strVJ_ + strPL_ +
                "  +MOVJ EC" + ec_ss.str() + " " + externAxeSpeed_, 'J');
        }
        else {
            std::stringstream ec_ss;
            ec_ss << std::setw(5) << std::setfill('0') << target_ec;
            addLine("MOVJ C" + target_str + bc_modifier + " EC" + ec_ss.str() +
                " " + strVJ_ + strPL_, 'J');
        }

        lastPose_ = pose;
        lastJoints_ = joints;
        lastMoveType_ = 0;
    }

    void RobotPost::MoveL(const robomath::Mat& pose, const std::vector<double>& joints, const std::vector<double>& confRLF) {
        pageSizeControl();

        auto target_ids = (pose == robomath::Mat())
            ? addTargetJoints(joints)
            : addTargetCartesian(poseFrame_ * pose, joints, confRLF);

        int target_id = std::get<0>(target_ids);
        int target_bc = std::get<1>(target_ids);
        int target_ec = std::get<2>(target_ids);

        std::string bc_modifier = "";
        if (target_bc != -1) {
            std::stringstream bc_ss;
            bc_ss << std::setw(5) << std::setfill('0') << target_bc;
            bc_modifier = " BC" + bc_ss.str();
        }

        std::stringstream target_ss;
        target_ss << std::setw(5) << std::setfill('0') << target_id;
        std::string target_str = target_ss.str();

        if (target_ec == -1) {
            addLine("MOVL C" + target_str + bc_modifier + " " + strV_ + strPL_, 'L');
        }
        else if (config_.EXTAXES_USE_SMOVL) {
            std::stringstream ec_ss;
            ec_ss << std::setw(5) << std::setfill('0') << target_ec;
            addLine("SMOVL C" + target_str + bc_modifier + " " + strV_ + strPL_ +
                "  +MOVJ EC" + ec_ss.str(), 'L');
        }
        else {
            std::stringstream ec_ss;
            ec_ss << std::setw(5) << std::setfill('0') << target_ec;
            addLine("MOVL C" + target_str + bc_modifier + " " + strV_ + strPL_ +
                "  +MOVJ EC" + ec_ss.str() + " " + externAxeSpeed_, 'L');
        }

        lastPose_ = pose;
        lastJoints_ = joints;
        lastMoveType_ = 1;
    }

    void RobotPost::MoveC(const robomath::Mat& pose1, const std::vector<double>& joints1,
        const robomath::Mat& pose2, const std::vector<double>& joints2,
        const std::vector<double>& confRLF1, const std::vector<double>& confRLF2) {
        pageSizeControl(4);

        auto target_id0 = std::make_tuple(-1, -1, -1);
        auto target_id1 = std::make_tuple(-1, -1, -1);
        auto target_id2 = std::make_tuple(-1, -1, -1);

        // 处理圆弧起始点
        if (lastMoveType_ != 2 && lastPose_ != robomath::Mat() && lastJoints_.size() > 0) {
            if (pose1 == robomath::Mat() || pose2 == robomath::Mat()) {
                target_id0 = addTargetJoints(lastJoints_);
            }
            else {
                target_id0 = addTargetCartesian(poseFrame_ * lastPose_, lastJoints_, confRLF1);
            }

            int target_id0_val = std::get<0>(target_id0);
            int target_bc0 = std::get<1>(target_id0);
            int target_ec0 = std::get<2>(target_id0);

            std::string bc_modifier0 = "";
            if (target_bc0 != -1) {
                std::stringstream bc0_ss;
                bc0_ss << std::setw(5) << std::setfill('0') << target_bc0;
                bc_modifier0 = " BC" + bc0_ss.str();
            }

            std::stringstream id0_ss;
            id0_ss << std::setw(5) << std::setfill('0') << target_id0_val;
            std::string target0_str = id0_ss.str();

            if (target_ec0 == -1) {
                if (lastMoveType_ < 0) {
                    addLine("MOVL C" + target0_str + bc_modifier0 + " " + strV_ + strPL_, 'L');
                }
                addLine("MOVC C" + target0_str + bc_modifier0 + " " + strV_ + strPL_, 'C');
            }
            else if (config_.EXTAXES_USE_MOVJ) {
                std::stringstream ec0_ss;
                ec0_ss << std::setw(5) << std::setfill('0') << target_ec0;
                addLine("MOVC C" + target0_str + bc_modifier0 + " " + strV_ + strPL_ +
                    "  +MOVJ EC" + ec0_ss.str() + " " + externAxeSpeed_, 'C');
            }
            else {
                std::stringstream ec0_ss;
                ec0_ss << std::setw(5) << std::setfill('0') << target_ec0;
                addLine("SMOVC C" + target0_str + bc_modifier0 + " EC" + ec0_ss.str() +
                    " " + strV_ + strPL_, 'C');
            }
        }

        // 添加圆弧中间点和结束点
        if (pose1 == robomath::Mat() || pose2 == robomath::Mat()) {
            target_id1 = addTargetJoints(joints1);
            target_id2 = addTargetJoints(joints2);
        }
        else {
            target_id1 = addTargetCartesian(poseFrame_ * pose1, joints1, confRLF1);
            target_id2 = addTargetCartesian(poseFrame_ * pose2, joints2, confRLF2);
        }

        // 处理中间点
        int target_id1_val = std::get<0>(target_id1);
        int target_bc1 = std::get<1>(target_id1);
        int target_ec1 = std::get<2>(target_id1);

        std::string bc_modifier1 = "";
        if (target_bc1 != -1) {
            std::stringstream bc1_ss;
            bc1_ss << std::setw(5) << std::setfill('0') << target_bc1;
            bc_modifier1 = " BC" + bc1_ss.str();
        }

        std::stringstream id1_ss;
        id1_ss << std::setw(5) << std::setfill('0') << target_id1_val;
        std::string target1_str = id1_ss.str();

        if (target_ec1 == -1) {
            addLine("MOVC C" + target1_str + bc_modifier1 + " " + strV_ + strPL_, 'C');
        }
        else if (config_.EXTAXES_USE_MOVJ) {
            std::stringstream ec1_ss;
            ec1_ss << std::setw(5) << std::setfill('0') << target_ec1;
            addLine("MOVC C" + target1_str + bc_modifier1 + " " + strV_ + strPL_ +
                "  +MOVJ EC" + ec1_ss.str() + " " + externAxeSpeed_, 'C');
        }
        else {
            std::stringstream ec1_ss;
            ec1_ss << std::setw(5) << std::setfill('0') << target_ec1;
            addLine("SMOVC C" + target1_str + bc_modifier1 + " EC" + ec1_ss.str() +
                " " + strV_ + strPL_, 'C');
        }

        // 处理结束点
        int target_id2_val = std::get<0>(target_id2);
        int target_bc2 = std::get<1>(target_id2);
        int target_ec2 = std::get<2>(target_id2);

        std::string bc_modifier2 = "";
        if (target_bc2 != -1) {
            std::stringstream bc2_ss;
            bc2_ss << std::setw(5) << std::setfill('0') << target_bc2;
            bc_modifier2 = " BC" + bc2_ss.str();
        }

        std::stringstream id2_ss;
        id2_ss << std::setw(5) << std::setfill('0') << target_id2_val;
        std::string target2_str = id2_ss.str();

        if (target_ec2 == -1) {
            addLine("MOVC C" + target2_str + bc_modifier2 + " " + strV_ + strPL_, 'C');
        }
        else if (config_.EXTAXES_USE_MOVJ) {
            std::stringstream ec2_ss;
            ec2_ss << std::setw(5) << std::setfill('0') << target_ec2;
            addLine("MOVC C" + target2_str + bc_modifier2 + " " + strV_ + strPL_ +
                "  +MOVJ EC" + ec2_ss.str() + " " + externAxeSpeed_, 'C');
        }
        else {
            std::stringstream ec2_ss;
            ec2_ss << std::setw(5) << std::setfill('0') << target_ec2;
            addLine("SMOVC C" + target2_str + bc_modifier2 + " EC" + ec2_ss.str() +
                " " + strV_ + strPL_, 'C');
        }

        lastPose_ = pose2;
        lastJoints_ = joints2;
        lastMoveType_ = 2;
    }

    // 设置函数
    void RobotPost::setFrame(const robomath::Mat& pose, int frameId, const std::string& frameName) {
        std::vector<double> xyzwpr = Pose_2_Motoman(pose);

        if (!config_.USE_MFRAME) {
            poseFrame_ = pose;
            RunMessage("Using " + frameName + " (targets wrt base):", true);
            std::string poseStr = std::to_string(xyzwpr[0]) + "," + std::to_string(xyzwpr[1]) + "," +
                std::to_string(xyzwpr[2]) + "," + std::to_string(xyzwpr[3]) + "," +
                std::to_string(xyzwpr[4]) + "," + std::to_string(xyzwpr[5]);
            RunMessage(poseStr, true);
        }
        else {
            poseFrame_ = robomath::Mat::Mat();
            if (frameId < 0) {
                RunMessage("Setting Frame " + std::to_string(config_.ACTIVE_FRAME) + " (" + frameName + "):", true);
                // 简化的MFRAME设置
            }
            else {
                RunMessage("Frame " + std::to_string(frameId) + " (" + frameName + ") should be close to:", true);
                std::string poseStr = std::to_string(xyzwpr[0]) + "," + std::to_string(xyzwpr[1]) + "," +
                    std::to_string(xyzwpr[2]) + "," + std::to_string(xyzwpr[3]) + "," +
                    std::to_string(xyzwpr[4]) + "," + std::to_string(xyzwpr[5]);
                RunMessage(poseStr, true);
            }
        }
    }

    void RobotPost::setTool(const robomath::Mat& pose, int toolId, const std::string& toolName) {
        std::vector<double> xyzwpr = Pose_2_Motoman(pose);

        if (toolId < 0) {
            if (!config_.USE_SETTOOL) {
                RunMessage("Tool " + toolName + " should be close to:", true);
            }
            else {
                RunMessage("Setting Tool " + std::to_string(config_.ACTIVE_TOOL) + " (" + toolName + "):", true);
                // 简化的SETTOOL实现
            }
        }
        else {
            RunMessage("Tool " + std::to_string(toolId) + " (" + toolName + ") should be close to:", true);
        }

        std::string poseStr = std::to_string(xyzwpr[0]) + "," + std::to_string(xyzwpr[1]) + "," +
            std::to_string(xyzwpr[2]) + "," + std::to_string(xyzwpr[3]) + "," +
            std::to_string(xyzwpr[4]) + "," + std::to_string(xyzwpr[5]);
        RunMessage(poseStr, true);
    }

    void RobotPost::setSpeed(double speed_mms) {
        double speedl = 0;
        if (config_.SPEED_CM_MIN) {
            double speed_cm_min = speed_mms * 60 / 10;
            speedl = (speed_cm_min < 0.01) ? 0.01 : ((speed_cm_min > 200.0) ? 200.0 : speed_cm_min);
        }
        else {
            speedl = (speed_mms < 0.01) ? 0.01 : ((speed_mms > 2000.0) ? 2000.0 : speed_mms);
        }

        std::stringstream ss;
        ss << std::fixed << std::setprecision(1) << speedl;
        strV_ = "V=" + ss.str() + strA_;
    }

    void RobotPost::setAcceleration(double accel_mmss) {
        double accel = (accel_mmss < 0) ? 0 : ((accel_mmss > 100.0) ? 100.0 : accel_mmss);
        strA_ = "";
        if (accel > 0) {
            std::stringstream ss;
            ss << std::fixed << std::setprecision(0) << accel;
            strA_ = " ACC=" + ss.str() + " DEC=" + ss.str();
        }
        // 更新速度字符串中的加速度
        size_t pos = strV_.find(" ACC=");
        if (pos != std::string::npos) {
            strV_ = strV_.substr(0, pos);
        }
        strV_ += strA_;
    }

    void RobotPost::setSpeedJoints(double speed_degs) {
        double speedj = (speed_degs > 100.0) ? 100.0 : ((speed_degs < 0.01) ? 0.01 : speed_degs);
        strVJ_ = "VJ=" + std::to_string(static_cast<int>(speedj));
    }

    void RobotPost::setAccelerationJoints(double accel_degss) {
        double accel = (accel_degss < 0) ? 0 : ((accel_degss > 100.0) ? 100.0 : accel_degss);
        strAJ_ = "";
        if (accel > 0) {
            std::stringstream ss;
            ss << std::fixed << std::setprecision(0) << accel;
            strAJ_ = " ACC=" + ss.str() + " DEC=" + ss.str();
        }
        // 更新关节速度字符串中的加速度
        size_t pos = strVJ_.find(" ACC=");
        if (pos != std::string::npos) {
            strVJ_ = strVJ_.substr(0, pos);
        }
        strVJ_ += strAJ_;
    }

    void RobotPost::setZoneData(double zone_mm) {
        if (zone_mm < 0) {
            strPL_ = "";
        }
        else {
            // 完全匹配Python版本的逻辑
            if (zone_mm > 0) {
                strPL_ = " PL=" + std::to_string(static_cast<int>(zone_mm));
            }
            else {
                strPL_ = "";
            }
        }
    }

    // I/O 操作
    void RobotPost::setDO(const std::string& io_var, const std::string& io_value) {
        std::string var_name = io_var;
        std::string value_str = io_value;

        if (std::isdigit(io_var[0])) {
            var_name = "OT#(" + io_var + ")";
        }

        if (std::isdigit(io_value[0])) {
            int value = std::stoi(io_value);
            value_str = (value > 0) ? "ON" : "OFF";
        }

        addLine("DOUT " + var_name + " " + value_str);
    }

    std::string RobotPost::joinStrings(const std::vector<std::string>& strings, const std::string& delimiter) {
        if (strings.empty()) return "";
        std::string result = strings[0];
        for (size_t i = 1; i < strings.size(); ++i) {
            result += delimiter + strings[i];
        }
        return result;
    }
    void RobotPost::setAO(const std::string& io_var, const std::string& io_value) {
        setDO(io_var, io_value);
    }

    void RobotPost::waitDI(const std::string& io_var, const std::string& io_value, int timeout_ms) {
        std::string var_name = io_var;
        std::string value_str = io_value;

        if (std::isdigit(io_var[0])) {
            var_name = "IN#(" + io_var + ")";
        }

        if (std::isdigit(io_value[0])) {
            int value = std::stoi(io_value);
            value_str = (value > 0) ? "ON" : "OFF";
        }

        if (timeout_ms <= 0) {
            addLine("WAIT " + var_name + "=" + value_str);
        }
        else {
            addLine("WAIT " + var_name + "=" + value_str + " T=" +
                std::to_string(static_cast<double>(timeout_ms) * 0.001));
        }
    }

    // 代码生成
    void RobotPost::RunCode(const std::string& code, bool is_function_call) {
        if (is_function_call) {
            // 处理speedex函数
            if (code.size() >= 8 && code.substr(0, 8) == "speedex(") {
                std::string strText = code.substr(8, code.size() - 9);
                try {
                    double speedValue = std::stod(strText);
                    double speedj = (speedValue < 0.01) ? 0.01 : ((speedValue > 100.0) ? 100.0 : speedValue);
                    std::stringstream ss;
                    ss << std::fixed << std::setprecision(2) << speedj;
                    externAxeSpeed_ = "VJ=" + ss.str();
                    return;
                }
                catch (...) {
                    addLog("Invalid speedex value: " + code);
                }
            }

            std::string code_safe = getSafeName(code);
            std::replace(code_safe.begin(), code_safe.end(), ' ', '_');
            addLine("CALL JOB:" + code_safe);
        }
        else {
            addLine(code);
        }
    }

    void RobotPost::RunMessage(const std::string& message, bool is_comment) {
        if (is_comment) {
            // 匹配Python版本的29字符换行
            for (size_t i = 0; i < message.length(); i += 29) {
                size_t end = (i + 29 < message.length()) ? (i + 29) : message.length();
                addLine("'" + message.substr(i, end - i));
            }
        }
        else {
            // 非注释消息使用MSG指令
            for (size_t i = 0; i < message.length(); i += 25) {
                size_t end = (i + 25 < message.length()) ? (i + 25) : message.length();
                addLine("MSG \"" + message.substr(i, end - i) + "\"");
            }
        }
    }

    void RobotPost::Pause(int time_ms) {
        if (time_ms <= 0) {
            addLine("PAUSE");
        }
        else {
            addLine("TIMER T=" + std::to_string(time_ms * 0.001));
        }
    }

    // Motoman特定功能
    void RobotPost::Macro(int number, const std::string& mf, const std::vector<double>& args) {
        std::string macro_line = "MACRO" + std::to_string(number) + " MJ#(" + mf + ")";

        if (args.size() > 16) {
            addLog("Macro supports only 16 arguments");
            return;
        }

        for (size_t i = 0; i < args.size(); ++i) {
            macro_line += " ARGF" + std::to_string(i + 1);
        }

        addLine(macro_line);
    }

    void RobotPost::Arcon(int asf_number) {
        if (asf_number == 0) {
            addLine("ARCON");
        }
        else {
            addLine("ARCON ASF#(" + std::to_string(asf_number) + ")");
        }
    }

    void RobotPost::Arcof(int aef_number) {
        if (aef_number == 0) {
            addLine("ARCOF");
        }
        else {
            addLine("ARCOF AEF#(" + std::to_string(aef_number) + ")");
        }
    }

    // 私有方法实现
    void RobotPost::addLine(const std::string& newline, char moveType) {
        if (nProgs_ > 1 && !config_.INCLUDE_SUB_PROGRAMS) {
            return;
        }

        pageSizeControl();

        currentProg_.lineCount++;
        std::string line_ok = newline;
        currentProg_.lines.push_back(line_ok);
    }

    void RobotPost::addLineTargets(const std::string& newline) {
        currentProg_.targets.push_back(newline);
    }

    void RobotPost::addLineTargetsEC(const std::string& newline) {
        currentProg_.targetsEC.push_back(newline);
    }

    void RobotPost::addLog(const std::string& newline) {
        if (nProgs_ > 1 && !config_.INCLUDE_SUB_PROGRAMS) {
            return;
        }
        log_ += newline + "\n";
    }

    void RobotPost::pageSizeControl(int reserved_lines) {
        if ((currentProg_.lineCount + reserved_lines) >= config_.MAX_LINES_X_PROG) {
            ProgFinish(currentProg_.name, true);
            ProgStart(currentProg_.name, true);
        }
    }

    std::tuple<int, int, int> RobotPost::addTargetJoints(const std::vector<double>& joints) {
        if (nProgs_ > 1 && !config_.INCLUDE_SUB_PROGRAMS) {
            return std::make_tuple(-1, -1, -1);
        }

        setPulses();

        int cid = currentProg_.cCount;
        int bcid = -1;
        int ecid = -1;
        currentProg_.cCount++;

        std::vector<std::string> str_pulses;
        std::vector<std::string> str_bc;
        std::vector<std::string> str_ec;

        for (size_t i = 0; i < joints.size() && i < config_.AXIS_INDEX.size(); ++i) {
            int axis_idx = (i < config_.AXIS_INDEX.size()) ? config_.AXIS_INDEX[i] : i;
            if (axis_idx < joints.size()) {
                double pulse_value_d = joints[axis_idx] * config_.PULSES_X_DEG[i];
                std::string pulse_value = std::to_string(static_cast<int>(std::round(pulse_value_d)));

                if (i < axesType_.size()) {
                    if (axesType_[i] == "R") {
                        str_pulses.push_back(pulse_value);
                    }
                    else if (axesType_[i] == "T") {
                        str_bc.push_back(pulse_value);
                    }
                    else {
                        str_ec.push_back(pulse_value);
                    }
                }
            }
        }

        // 处理4轴机器人特殊情况
        if (str_pulses.size() == 4) {
            str_pulses = { str_pulses[0], str_pulses[1], str_pulses[2], "0", "0", str_pulses[3] };
        }

        // 格式化CID为5位数
        std::stringstream cid_ss;
        cid_ss << std::setw(5) << std::setfill('0') << cid;
        std::string cid_str = cid_ss.str();

        addLineTargets("C" + cid_str + "=" + joinStrings(str_pulses, ","));

        // 处理直线轴(BC)
        if (!str_bc.empty()) {
            bcid = currentProg_.bcCount;
            currentProg_.bcCount++;

            std::stringstream bcid_ss;
            bcid_ss << std::setw(5) << std::setfill('0') << bcid;
            std::string bcid_str = bcid_ss.str();

            addLineTargetsBC("BC" + bcid_str + "=" + joinStrings(str_bc, ","));
        }

        // 处理外部轴(EC)
        if (!str_ec.empty()) {
            ecid = currentProg_.ecCount;
            currentProg_.ecCount++;

            std::stringstream ecid_ss;
            ecid_ss << std::setw(5) << std::setfill('0') << ecid;
            std::string ecid_str = ecid_ss.str();

            addLineTargetsEC("EC" + ecid_str + "=" + joinStrings(str_ec, ","));
        }

        return std::make_tuple(cid, bcid, ecid);
    }


    std::tuple<int, int, int> RobotPost::addTargetCartesian(const robomath::Mat& pose, const std::vector<double>& joints, const std::vector<double>& confRLF) {
        if (nProgs_ > 1 && !config_.INCLUDE_SUB_PROGRAMS) {
            return std::make_tuple(-1, -1, -1);
        }

        if (!config_.USE_RELATIVE_JOB) {
            auto result = addTargetJoints(joints);
            return result;
        }

        std::vector<double> xyzwpr = Pose_2_Motoman(pose);

        // 配置处理
        std::vector<int> turns = { 0, 0, 0 };
        std::vector<int> conf = { 0, 0, 0 };

        if (!confRLF.empty() && confRLF.size() >= 3) {
            conf = {
                static_cast<int>(confRLF[0]),
                static_cast<int>(confRLF[1]),
                static_cast<int>(confRLF[2])
            };
        }

        // 处理关节转角
        if (joints.size() >= 6) {
            // 计算转角 - 根据Python版本逻辑
            turns[0] = static_cast<int>((joints[3] + 180.0) / 360.0); // J4
            turns[1] = static_cast<int>((joints[5] + 180.0) / 360.0); // J6  
            turns[2] = static_cast<int>((joints[0] + 180.0) / 360.0); // J1
        }

        // 构建配置字符串 - 根据Python版本格式
        std::string confdata = std::to_string(conf[2]) + "," + std::to_string(conf[1]) + "," +
            std::to_string(conf[0]) + "," + std::to_string(turns[0]) + "," +
            std::to_string(turns[1]) + "," + std::to_string(turns[2]) + ",0,0";

        setCartesian(confdata);

        int cid = currentProg_.cCount;
        currentProg_.cCount++;
        int bcid = -1;
        int ecid = -1;

        // 处理直线轴(BC) - 对应Python版本中的T类型轴
        if (hasTrack_) {
            std::vector<std::string> str_bc;
            for (size_t i = 0; i < joints.size() && i < axesType_.size(); ++i) {
                if (axesType_[i] == "T") {  // 直线轴
                    double pulse_value_d = joints[i] * config_.PULSES_X_DEG[i];
                    str_bc.push_back(std::to_string(static_cast<int>(std::round(pulse_value_d))));
                }
            }

            if (!str_bc.empty()) {
                bcid = currentProg_.bcCount;
                currentProg_.bcCount++;

                std::stringstream bcid_ss;
                bcid_ss << std::setw(5) << std::setfill('0') << bcid;
                std::string bcid_str = bcid_ss.str();

                addLineTargetsBC("BC" + bcid_str + "=" + joinStrings(str_bc, ","));
            }
        }

        // 格式化CID为5位数
        std::stringstream cid_ss;
        cid_ss << std::setw(5) << std::setfill('0') << cid;
        std::string cid_str = cid_ss.str();

        // 构建坐标字符串
        std::stringstream coord_ss;
        coord_ss << std::fixed << std::setprecision(3);
        coord_ss << xyzwpr[0] << "," << xyzwpr[1] << "," << xyzwpr[2] << ",";
        coord_ss << std::fixed << std::setprecision(2);
        coord_ss << xyzwpr[3] << "," << xyzwpr[4] << "," << xyzwpr[5];

        addLineTargets("C" + cid_str + "=" + coord_ss.str());

        // 处理外部轴(EC) - 对应Python版本中的J类型轴（转台轴）
        if (hasTurntable_) {
            std::vector<std::string> str_ec;
            for (size_t i = 0; i < joints.size() && i < axesType_.size(); ++i) {
                if (axesType_[i] == "J") {  // 转台轴
                    double pulse_value_d = joints[i] * config_.PULSES_X_DEG[i];
                    str_ec.push_back(std::to_string(static_cast<int>(std::round(pulse_value_d))));
                }
            }

            if (!str_ec.empty()) {
                ecid = currentProg_.ecCount;
                currentProg_.ecCount++;

                std::stringstream ecid_ss;
                ecid_ss << std::setw(5) << std::setfill('0') << ecid;
                std::string ecid_str = ecid_ss.str();

                addLineTargetsEC("EC" + ecid_str + "=" + joinStrings(str_ec, ","));
            }
        }

        return std::make_tuple(cid, bcid, ecid);
    }

    // 添加新的直线轴目标点添加函数
    void RobotPost::addLineTargetsBC(const std::string& newline) {
        currentProg_.targetsBC.push_back(newline);
    }

    // 添加焊接相关函数（空实现，保持接口一致）
    void RobotPost::ArcStart(const std::string& data) {
        addLog("ArcStart not implemented");
    }

    void RobotPost::ArcContinue(const std::string& data) {
        addLog("ArcContinue not implemented");
    }

    void RobotPost::ArcEnd(const std::string& data) {
        addLog("ArcEnd not implemented");
    }

    void RobotPost::Arcset(int asf_number, int acond_number) {
        if (acond_number == 0) {
            addLine("ARCSET ASF#(" + std::to_string(asf_number) + ")");
        }
        else {
            addLine("ARCSET ASF#(" + std::to_string(asf_number) + ") ACOND=" + std::to_string(acond_number));
        }
    }

    void RobotPost::saveProgram(const std::string& folder, const std::string& progname, bool askUser, const std::string& showResult) {
        std::string safeName = getSafeName(progname);
        std::string filesave = folder + "/" + safeName + "." + progExt_;

        std::cout << "Saving program: " << safeName << " to: " << filesave << std::endl;

        // 查找对应的程序数据
        ProgramData* programToSave = nullptr;

        // 首先尝试在 progList_ 中查找
        for (auto& prog : progList_) {
            if (prog.name == safeName) {
                programToSave = &prog;
                break;
            }
        }

        // 如果没有找到，使用 currentProg_
        if (!programToSave && !currentProg_.name.empty() && currentProg_.name == safeName) {
            programToSave = &currentProg_;
        }

        if (!programToSave) {
            std::cerr << "Warning: Program " << safeName << " not found in progList_ or currentProg_" << std::endl;
            return;
        }

        std::ofstream fid(filesave);
        if (!fid.is_open()) {
            throw std::runtime_error("无法打开文件: " + filesave);
        }

        // 写入程序内容
        for (const auto& line : programToSave->lines) {
            fid << line << "\n";
        }

        fid.close();

        std::cout << "SAVED: " << filesave << " with " << programToSave->lines.size() << " lines\n";
        progFiles_.push_back(filesave);

        // 显示结果
        if (!showResult.empty()) {
            ShellExecuteA(NULL, "open", filesave.c_str(), NULL, NULL, SW_SHOW);
        }
    }

    void RobotPost::setCartesian(const std::string& confdata) {
        // 更新配置数据
        if (config_.USE_MFRAME && config_.ACTIVE_FRAME >= 0 &&
            std::to_string(config_.ACTIVE_FRAME) != lastConfData_[1]) {
            addLineTargets("///USER " + std::to_string(config_.ACTIVE_FRAME));
            lastConfData_[1] = std::to_string(config_.ACTIVE_FRAME);
        }

        if (std::to_string(config_.ACTIVE_TOOL) != lastConfData_[2]) {
            addLineTargets("///TOOL " + std::to_string(config_.ACTIVE_TOOL));
            lastConfData_[2] = std::to_string(config_.ACTIVE_TOOL);
        }

        if (lastConfData_[0] != "2") {
            if (config_.USE_MFRAME && config_.ACTIVE_FRAME >= 0) {
                addLineTargets("///POSTYPE USER");
            }
            else {
                addLineTargets("///POSTYPE BASE");
            }

            addLineTargets("///RECTAN");
            addLineTargets("///RCONF " + confdata);
            lastConfData_[3] = confdata;
        }
        else if (lastConfData_[3] != confdata) {
            addLineTargets("///RCONF " + confdata);
            lastConfData_[3] = confdata;
        }

        lastConfData_[0] = "2";
    }

    void RobotPost::setPulses() {
        if (lastConfData_[0].empty()) {
            addLineTargets("///TOOL " + std::to_string(config_.ACTIVE_TOOL));
            lastConfData_[2] = std::to_string(config_.ACTIVE_TOOL);
        }

        if (lastConfData_[0] != "1") {
            addLineTargets("///POSTYPE PULSE");
            addLineTargets("///PULSE");
            lastConfData_[0] = "1";
        }

        lastConfData_[1] = "";
        lastConfData_[2] = "";
        lastConfData_[3] = "";
    }

    // 辅助函数：连接字符串
    std::string joinStrings(const std::vector<std::string>& strings, const std::string& delimiter) {
        if (strings.empty()) return "";

        std::string result = strings[0];
        for (size_t i = 1; i < strings.size(); ++i) {
            result += delimiter + strings[i];
        }
        return result;
    }

    // 测试函数
    void test_post(std::string path) {
        auto Pose = robomath::PosePP;

        // 创建Motoman机器人后处理器
        RobotPost robot("Motoman_CartesianOnly", "Motoman Robot",6);
        // 必须先设置脉冲系数信息，也可直接修改模版中此信息
        robot.setPulsesXDeg({ 1492.764444, 1517.037037, 1264.197531, 910.222222, 910.222222, 568.888889 });
        robot.ProgStart("Curve");
        robot.RunMessage("Program generated by Hejun for Motoman MH5LF on " + robot.getCurrentTimestamp(), true);
        robot.RunMessage("Using nominal kinematics.", true);
        robot.RunCode("Pause", true);
        robot.setZoneData(1.000);
        robot.setSpeed(100.000);
        robot.setFrame(Pose(971.355, 225.235, 35.886, 180, 30, 0), -1, "Part");
        robot.setTool(Pose(0, 0, 90.865, 0, 0, 0), -1, "RobotiQ Sanding Kit");
        robot.RunMessage("Show RobotiQ Sanding Kit", true);
        robot.MoveJ(Pose(0,0,0,0,0,0), { 11.4367,58.4153,58.9996,6.49399,-61.2352,-103.073 }, {});
        robot.MoveL(Pose(25.0003, 60, 44.9998, 90, 0, -180), { 11.1056,66.1178,67.3057,6.27395,-61.7982,-102.622 }, { 0,0,0 });
        robot.MoveL(Pose(50.0003, 60, 19.9998, 90, 0, -180), { 11.2246,62.2551,55.2016,6.94297,-53.6249,-103.883 }, { 0,0,0 });
        robot.setSpeed(50.000);
        robot.MoveL(Pose(350, 60, 19.9997, 90, 0, -180), { 16.0908,35.5305,-17.4629,45.1981,-11.2627,-148.667 }, { 0,0,0 });
        robot.MoveL(Pose(350, 94.233, 19.9997, 90, 0, -180), { 12.882,34.6844,-19.0198,42.9366,-9.4184,-143.752 }, { 0,0,0 });
        robot.MoveL(Pose(347.348, 95.2783, 19.9997, 90, 0, -180), { 12.7332,34.7377,-18.6435,41.3706,-9.59856,-142.043 }, { 0,0,0 });
        robot.MoveL(Pose(332.253, 102.483, 19.9997, 90, 0, -180), { 11.7882,35.058,-16.5067,33.3935,-10.6957,-133.179 }, { 0,0,0 });
        robot.MoveL(Pose(318.126, 111.437, 19.9997, 90, 0, -180), { 10.7321,35.3829,-14.5122,27.0758,-11.8036,-125.904 }, { 0,0,0 });
        robot.MoveL(Pose(305.167, 122.011, 19.9997, 90, 0, -180), { 9.57929,35.7012,-12.6856,21.9447,-12.8646,-119.76 }, { 0,0,0 });
        robot.MoveL(Pose(293.562, 134.056, 19.9997, 90, 0, -180), { 8.34372,36.0021,-11.0507,17.6578,-13.8391,-114.414 }, { 0,0,0 });
        robot.MoveL(Pose(283.476, 147.398, 19.9997, 90, 0, -180), { 7.03853,36.2752,-9.62985,13.9726,-14.6991,-109.636 }, { 0,0,0 });
        robot.MoveL(Pose(275.053, 161.848, 19.9997, 90, 0, -180), { 5.67633,36.5114,-8.44272,10.7162,-15.4241,-105.259 }, { 0,0,0 });
        robot.MoveL(Pose(268.413, 177.2, 19.9997, 90, 0, -180), { 4.26925,36.7027,-7.50641,7.76138,-15.9989,-101.163 }, { 0,0,0 });
        robot.MoveL(Pose(263.652, 193.233, 19.9997, 90, 0, -180), { 2.82905,36.8425,-6.83475,5.01065,-16.4124,-97.258 }, { 0,0,0 });
        robot.MoveL(Pose(260.837, 209.72, 19.9997, 90, 0, -180), { 1.36725,36.9259,-6.43784,2.38542,-16.6569,-93.4696 }, { 0,0,0 });
        robot.MoveL(Pose(260.008, 226.426, 19.9997, 90, 0, -180), { -0.104822,36.9502,-6.32177,-0.182089,-16.7282,-89.7348 }, { 0,0,0 });
        robot.MoveL(Pose(261.177, 243.11, 19.9997, 90, 0, -180), { -1.57591,36.9143,-6.48831,-2.75479,-16.6249,-85.9953 }, { 0,0,0 });
        robot.MoveL(Pose(264.327, 259.537, 19.9997, 90, 0, -180), { -3.03474,36.8197,-6.93492,-5.39607,-16.3488,-82.1921 }, { 0,0,0 });
        robot.MoveL(Pose(269.414, 275.47, 19.9997, 90, 0, -180), { -4.46997,36.6695,-7.65477,-8.17513,-15.9048,-78.2608 }, { 0,0,0 });
        robot.MoveL(Pose(276.365, 290.683, 19.9997, 90, 0, -180), { -5.87004,36.4688,-8.63705,-11.1733,-15.3012,-74.1253 }, { 0,0,0 });
        robot.MoveL(Pose(285.08, 304.959, 19.9997, 90, 0, -180), { -7.22312,36.2246,-9.86733,-14.4922,-14.5496,-69.6901 }, { 0,0,0 });
        robot.MoveL(Pose(295.436, 318.093, 19.9997, 90, 0, -180), { -8.51701,35.945,-11.328,-18.2661,-13.666,-64.8285 }, { 0,0,0 });
        robot.MoveL(Pose(307.284, 329.899, 19.9997, 90, 0, -180), { -9.73901,35.6395,-12.9988,-22.6785,-12.672,-59.3653 }, { 0,0,0 });
        robot.MoveL(Pose(320.456, 340.207, 19.9997, 90, 0, -180), { -10.8758,35.3186,-14.8573,-27.9876,-11.5973,-53.0523 }, { 0,0,0 });
        robot.MoveL(Pose(334.762, 348.871, 19.9997, 90, 0, -180), { -11.9135,34.993,-16.8792,-34.5586,-10.484,-45.5362 }, { 0,0,0 });
        robot.MoveL(Pose(350, 355.767, 19.9997, 90, 0, -180), { -12.8373,34.6741,-19.0387,-42.8911,-9.39409,-36.3307 }, { 0,0,0 });
        robot.MoveL(Pose(350, 390, 19.9997, 90, 0, -180), { -16.0474,35.5176,-17.4867,-45.179,-11.2365,-31.3875 }, { 0,0,0 });
        robot.MoveL(Pose(50.0003, 390, 19.9998, 90, 0, -180), { -11.1935,62.2282,55.1514,-6.92628,-53.598,-76.152 }, { 0,0,0 });
        robot.MoveL(Pose(50.0003, 123.5, 19.9998, 90, 0, -180), { 6.96624,59.5195,50.1147,4.48406,-50.8647,-98.874 }, { 0,0,0 });
        robot.MoveL(Pose(223.62, 123.5, 19.9997, 90, 0, -180), { 8.48113,40.0526,2.22087,10.963,-22.8151,-107.482 }, { 0,0,0 });
        robot.MoveL(Pose(218.5, 132.968, 19.9997, 90, 0, -180), { 7.65267,40.2329,2.9603,9.71192,-23.2473,-105.574 }, { 0,0,0 });
        robot.MoveL(Pose(210.282, 151.385, 19.9997, 90, 0, -180), { 6.07571,40.5282,4.15157,7.49242,-23.9447,-102.121 }, { 0,0,0 });
        robot.MoveL(Pose(203.927, 170.526, 19.9997, 90, 0, -180), { 4.47327,40.7614,5.07654,5.39873,-24.4867,-98.7916 }, { 0,0,0 });
        robot.MoveL(Pose(199.498, 190.202, 19.9997, 90, 0, -180), { 2.85232,40.9264,5.72303,3.39222,-24.8657,-95.5491 }, { 0,0,0 });
        robot.MoveL(Pose(197.038, 210.219, 19.9997, 90, 0, -180), { 1.21969,41.0186,6.08226,1.43895,-25.0762,-92.3597 }, { 0,0,0 });
        robot.MoveL(Pose(196.572, 230.382, 19.9997, 90, 0, -180), { -0.417856,41.0354,6.14922,-0.492245,-25.1152,-89.1924 }, { 0,0,0 });
        robot.MoveL(Pose(198.103, 250.491, 19.9997, 90, 0, -180), { -2.05359,40.9765,5.92298,-2.43144,-24.9822,-86.0172 }, { 0,0,0 });
        robot.MoveL(Pose(201.618, 270.351, 19.9997, 90, 0, -180), { -3.68078,40.8433,5.4067,-4.40909,-24.6789,-82.8035 }, { 0,0,0 });
        robot.MoveL(Pose(207.081, 289.765, 19.9997, 90, 0, -180), { -5.29263,40.6396,4.60748,-6.45766,-24.2098,-79.5193 }, { 0,0,0 });
        robot.MoveL(Pose(214.439, 308.543, 19.9997, 90, 0, -180), { -6.88223,40.3709,3.53601,-8.61335,-23.5816,-76.1291 }, { 0,0,0 });
        robot.MoveL(Pose(223.62, 326.5, 19.9997, 90, 0, -180), { -8.44251,40.0441,2.20609,-10.9183,-22.8031,-72.5921 }, { 0,0,0 });
        robot.MoveL(Pose(113.5, 326.5, 19.9998, 90, 0, -180), { -7.41947,49.7878,27.9067,-5.95624,-38.4779,-78.8962 }, { 0,0,0 });
        robot.MoveL(Pose(113.5, 187, 19.9998, 90, 0, -180), { 2.81489,48.8697,26.2743,2.31394,-37.4571,-94.2755 }, { 0,0,0 });
        robot.MoveL(Pose(159.484, 187, 19.9997, 90, 0, -180), { 2.96656,44.2074,14.6038,2.92574,-30.4622,-95.0921 }, { 0,0,0 });
        robot.MoveL(Pose(158.527, 193.651, 19.9998, 90, 0, -180), { 2.44849,44.2495,14.7511,2.40879,-30.5464,-94.1956 }, { 0,0,0 });
        robot.MoveL(Pose(156.68, 215.636, 19.9998, 90, 0, -180), { 0.742953,44.3307,15.0352,0.727426,-30.7086,-91.2689 }, { 0,0,0 });
        robot.MoveL(Pose(156.832, 237.698, 19.9998, 90, 0, -180), { -0.964728,44.323,15.01,-0.944971,-30.694,-88.3519 }, { 0,0,0 });
        robot.MoveL(Pose(158.979, 259.655, 19.9997, 90, 0, -180), { -2.66947,44.2265,14.6761,-2.62957,-30.5029,-85.4217 }, { 0,0,0 });
        robot.setSpeed(100.000);
        robot.MoveL(Pose(161.412, 284.537, 44.9997, 90, 0, -180), { -4.6849,41.1852,12.5608,-4.47807,-31.5366,-82.1217 }, { 0,0,0 });
        std::cout << "Done instruction: 50" << std::endl;
        std::cout.flush();
        robot.MoveL(Pose(161.412, 284.537, 94.9997, 90, 0, -180), { -4.85176,35.335,9.44003,-4.30711,-34.2694,-82.2342 }, { 0,0,0 });
        robot.ProgFinish("Curve");
        robot.ProgStart("Pause");
        robot.Pause(4000.0);
        robot.ProgFinish("Pause");
        
        // 遍历PROG_LIST
        for (const auto& prgi : robot.progList_)
        {
            for (const auto& line : prgi.lines) {
                std::cout << line << std::endl;
            }
            std::cout << std::endl << std::endl;
        }

        // 检查LOG并显示消息框
        if (robot.log_.length() > 0)
        {
            MessageBoxA(NULL, ("Program generation LOG:\n\n" + robot.log_).c_str(),
                "Program Log", MB_OK | MB_ICONINFORMATION);
        }

        robot.ProgSave(path, "Prog1", false, "");
        std::cout << "Program generation completed successfully!" << std::endl;
    }
}
