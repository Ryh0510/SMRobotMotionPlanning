#include <PostModel/Fanuc_RJ3_DripFeed.h>
#include <PostModel/robomath.h>
#include <PostModel/ftpsend.h>
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
#include <wininet.h>
#include <string>
#include <iostream>
#include <stdexcept>
#pragma comment(lib, "shell32.lib")  // 添加这行
#pragma comment(lib, "wininet.lib")  // 已有的
#pragma execution_character_set("utf-8")

namespace Fanuc_RJ3_DripFeed{
    using namespace robomath;

    // 常量定义
    const std::vector<std::string> FANUC_RESERVED_NAMES = {
        "PROG", "ATTR", "OWNER", "COMMENT", "CREATE", "MODIFIED", "FILE_NAME",
        "VERSION", "LINE_COUNT", "MEMORY_SIZE", "PROTECT", "TCD", "DEFAULT_GROUP",
        "CONTROL_CODE", "APPL", "MN", "POS", "END"
    };

    // 工具函数
    std::string RobotPost::getSafeName(const std::string& name, int max_chars) {
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
            result = result.substr(0, 8); // 确保有足够长度
        }

        return result;
    }

    std::string RobotPost::pose2Fanuc(const robomath::Mat& pose) {
        Vector xyzrpw = pose_2_xyzrpw(pose);
        std::stringstream ss;
        ss << std::fixed << std::setprecision(3);
        ss << "X=" << xyzrpw[0] << "mm, Y=" << xyzrpw[1] << "mm, Z=" << xyzrpw[2] << "mm, ";
        ss << "W=" << xyzrpw[3] << "deg, P=" << xyzrpw[4] << "deg, R=" << xyzrpw[5] << "deg";
        return ss.str();
    }

    // 构造函数
    RobotPost::RobotPost(const std::string robotPost, const std::string robotName, int robotAxes, std::vector<std::string> axesType, const std::string ip_com, const Config config) : robotPost_(robotPost), robotName_(robotName), robotAxes_(robotAxes), axesType_(axesType), ip_com_(ip_com), config_(config)
    {
        // 初始化轴类型
        for (size_t i = 0; i < axesType_.size(); ++i) {
            if (axesType_[i] == "T") {
                axesTrack_.push_back(i);
                hasTrack_ = true;
                hasExtAxes_ = true;
            }
            else if (axesType_[i] == "J") {
                axesTurntable_.push_back(i);
                hasTurntable_ = true;
                hasExtAxes_ = true;
            }
        }
    }

    // 程序控制方法
    void RobotPost::ProgStart(const std::string& progname) {
        std::string safeName = getSafeName(progname,12);
        nProgs_++;

        if (nProgs_ > 1 && !config_.includeSubPrograms) {
            std::cout << "Skipping subprogram: " << safeName << std::endl;
            return;
        }

        if (nPages_ > 0) {
            if (nPages_ == 0) {
                if (!progNamesMain_.empty()) {
                    addLog("Can't split " + progname + ": Two or more programs are split into smaller programs");
                    throw std::runtime_error("Only one program at a time can be split into smaller programs");
                }
                progNamesMain_.push_back(currentProg_.name);
                nPages_++;
            }
            nPages_++;
            std::string progname_i = currentProg_.name + std::to_string(nPages_);
            progNamesMain_.push_back(progname_i);
        }
        else {
            currentProg_.name = safeName;
            progNames_.push_back(safeName);
        }

        currentProg_ = ProgramData(safeName);
        currentProg_.lineCount = 0;
        currentProg_.pCount = 0;
    }

    void RobotPost::ProgFinish(const std::string& progname) {
        if (nProgs_ > 1 && !config_.includeSubPrograms) {
            return;
        }

        // 构建程序头
        std::string header = "/PROG  " + currentProg_.name + "\n";
        header += "/ATTR\n";
        header += "OWNER\t\t= MNEDITOR;\n";
        header += "COMMENT\t\t= \"RobotSDK sequence\";\n";
        header += "PROG_SIZE\t= 0;\n";
        header += "CREATE\t\t= DATE 31-12-14  TIME 12:00:00;\n";
        header += "MODIFIED\t= DATE 31-12-14  TIME 12:00:00;\n";
        header += "FILE_NAME\t= " + currentProg_.name + ";\n";
        header += "VERSION\t\t= 0;\n";
        header += "LINE_COUNT\t= " + std::to_string(currentProg_.lineCount) + ";\n";
        header += "MEMORY_SIZE\t= 0;\n";
        header += "PROTECT\t\t= READ_WRITE;\n";
        header += "TCD:  STACK_SIZE\t= 0,\n";
        header += "      TASK_PRIORITY\t= 50,\n";
        header += "      TIME_SLICE\t= 0,\n";
        header += "      BUSY_LAMP_OFF\t= 0,\n";
        header += "      ABORT_REQUEST\t= 0,\n";
        header += "      PAUSE_REQUEST\t= 0;\n";

        if (config_.fanucRJ3Compatible) {
            header += "DEFAULT_GROUP\t= 1,*,*,*,*;\n";
        }
        else {
            header += "DEFAULT_GROUP\t= 1,*,*,*,*,*,*;\n";
        }

        header += "CONTROL_CODE\t= 00000000 00000000;\n";

        if (hasTrack_) {
            header += "/APPL\n";
            header += "LINE_TRACK;\n";
            header += "LINE_TRACK_SCHEDULE_NUMBER      : 0;\n";
            header += "LINE_TRACK_BOUNDARY_NUMBER      : 0;\n";
            header += "CONTINUE_TRACK_AT_PROG_END      : FALSE;\n";
        }

        header += "/MN";

        // 插入程序头到 lines 的开头
        currentProg_.lines.insert(currentProg_.lines.begin(), header);

        // 添加目标部分
        currentProg_.lines.push_back("/POS");
        for (const auto& target : currentProg_.targets) {
            currentProg_.lines.push_back(target);
        }
        currentProg_.lines.push_back("/END");

        // 保存程序到列表
        progList_.push_back(currentProg_);

        std::cout << "Program finished: " << currentProg_.name
            << ", lines: " << currentProg_.lines.size()
            << ", targets: " << currentProg_.targets.size() << std::endl;

        // 重置当前程序
        currentProg_ = ProgramData();
        nPages_ = 0;
    }

    void RobotPost::ProgSave(const std::string& folder, const std::string& progname,
        bool askUser, const std::string& showResult) {
        std::string safeName = getSafeName(progname, 16);
        // 修改查找逻辑，避免使用原始指针
        auto it = std::find_if(progList_.begin(), progList_.end(),
            [&safeName](const ProgramData& prog) { return prog.name == safeName; });

        if (it != progList_.end()) {
            // 使用迭代器访问
            ProgramData& programToSave = *it;
            // 处理程序保存
        }
        else if (!currentProg_.name.empty() && currentProg_.name == safeName) {
            // 使用currentProg_
            ProgramData& programToSave = currentProg_;
            // 处理程序保存
        }
        else {
            std::cerr << "Warning: Program " << safeName << " not found" << std::endl;
            return;
        }
        if (progList_.size() >= 1) {
            if (currentProg_.lineCount > 0) {
                addLog("Warning: ProgFinish was not called properly");
                // 确保当前程序也被保存
                ProgFinish(currentProg_.name);
            }

            if (progNamesMain_.size() > 1) {
                // 生成主程序调用子程序
                std::string progname_main = "M_" + progNamesMain_[0];
                ProgramData mainProg(progname_main);

                for (const auto& prog_call : progNamesMain_) {
                    mainProg.lines.push_back("CALL " + prog_call + ";");
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
            // 如果没有程序在列表中，尝试保存当前程序
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

    void RobotPost::ProgSendRobot(const std::string& remote_path,const std::string& ftp_user, const std::string& ftp_pass) {
        // 如果配置了drip feed模式，保留原有逻辑
        if (!config_.dripFeedFileName.empty()) {
            std::vector<std::string> ftpParams = { ip_com_, remote_path, ftp_user, ftp_pass };
            generateDripFeedScript("", progNames_, config_.dripFeedFileName,
                currentProg_.name, ftpParams);
            return;
        }

        // 直接FTP上传所有程序文件
        std::cout << "开始FTP上传到机器人..." << std::endl;
        std::cout << "服务器: " << ip_com_ << std::endl;
        std::cout << "远程路径: " << remote_path << std::endl;
        std::cout << "用户名: " << ftp_user << std::endl;

        int successCount = 0;
        int totalFiles = progFiles_.size();

        for (const auto& localFile : progFiles_) {
            std::string errorMsg;
            if (FTPUploadFile(ip_com_, ftp_user, ftp_pass, remote_path,
                localFile, errorMsg)) {
                successCount++;
                addLog("上传成功: " + localFile);
            }
            else {
                addLog("上传失败: " + localFile + " - " + errorMsg);
                std::cerr << "错误: " << errorMsg << std::endl;
            }
        }

        // 输出上传结果
        if (successCount == totalFiles) {
            std::cout << "所有文件上传成功! (" << successCount << "/" << totalFiles << ")" << std::endl;
            addLog("所有程序文件已成功上传到机器人");
        }
        else {
            std::cout << "部分文件上传失败。成功: " << successCount << "/" << totalFiles << std::endl;
            addLog("警告: 部分文件上传失败");
        }

        // 如果启用了TP文件编译，也上传TP文件
        if (!config_.pathMakeTP.empty()) {
            std::vector<std::string> tpFiles;
            for (const auto& lsFile : progFiles_) {
                std::string tpFile = lsFile;
                size_t dotPos = tpFile.find_last_of('.');
                if (dotPos != std::string::npos) {
                    tpFile = tpFile.substr(0, dotPos) + ".TP";
                    if (std::ifstream(tpFile).good()) {
                        tpFiles.push_back(tpFile);
                    }
                }
            }

            if (!tpFiles.empty()) {
                std::cout << "上传编译后的TP文件..." << std::endl;
                for (const auto& tpFile : tpFiles) {
                    std::string errorMsg;
                    if (FTPUploadFile(ip_com_, ftp_user, ftp_pass, remote_path,
                        tpFile, errorMsg)) {
                        addLog("上传成功: " + tpFile);
                    }
                    else {
                        addLog("上传失败: " + tpFile + " - " + errorMsg);
                    }
                }
            }
        }
    }


    // 运动指令
    void RobotPost::MoveJ(const robomath::Mat& pose, const std::vector<double>& joints,
        const std::vector<double>& confRLF) {
        pageSizeControl();
        int target_id = addTargetJoints(pose, joints);
        std::string move_ins = "P[" + std::to_string(target_id) + "] " +
            config_.jointSpeed + " " + config_.cntValue + " ;";
        addLine(move_ins, 'J');
        lastPose_ = pose;
    }

    void RobotPost::MoveL(const robomath::Mat& pose, const std::vector<double>& joints,
        const std::vector<double>& confRLF) {
        // 检查是否与上一个位置相同
        if (lastPose_ != robomath::Mat() && pose != robomath::Mat()) {
            Vector pos1 = lastPose_.Pos();
            Vector pos2 = pose.Pos();
            double dist = distance(pos1, pos2);
            double angle = angle3(lastPose_.VX(), pose.VX());

            if (dist < 0.001 && angle < 0.001) {
                return;
            }
        }

        pageSizeControl();

        int target_id;
        if (pose == robomath::Mat()) {
            target_id = addTargetJoints(pose, joints);
        }
        else {
            target_id = addTargetCartesian(pose, joints, confRLF);
        }

        std::string move_ins = "P[" + std::to_string(target_id) + "] " +
            config_.speed + " " + config_.cntValue + " ;";
        addLine(move_ins, 'L');
        lastPose_ = pose;
    }

    void RobotPost::MoveC(const robomath::Mat& pose1, const std::vector<double>& joints1,
        const robomath::Mat& pose2, const std::vector<double>& joints2,
        const std::vector<double>& confRLF1, const std::vector<double>& confRLF2) {
        pageSizeControl();

        int target_id1 = addTargetCartesian(pose1, joints1, confRLF1);
        int target_id2 = addTargetCartesian(pose2, joints2, confRLF2);

        std::string move_ins = "P[" + std::to_string(target_id1) + "] \n       P[" +
            std::to_string(target_id2) + "] " + config_.speed + " " + config_.cntValue + " ;";
        addLine(move_ins, 'C');
        lastPose_ = pose2;
    }

    // 设置函数
    void RobotPost::setFrame(const robomath::Mat& pose, int frameId, const std::string& frameName) {
        Vector xyzwpr = pose_2_xyzrpw(pose);

        if (frameId < 0 || frameId > 9) {
            for (int i = 0; i < 6; ++i) {
                addLine("PR[" + std::to_string(config_.sparePR) + "," + std::to_string(i + 1) +
                    "]=" + std::to_string(xyzwpr[i]) + " ;");
            }
            for (int i = 6; i < robotAxes_; ++i) {
                addLine("PR[" + std::to_string(config_.sparePR) + "," + std::to_string(i + 1) + "]=0.0 ;");
            }
            addLine("UFRAME[" + config_.activeUF + "]=PR[" + std::to_string(config_.sparePR) + "] ;");
            addLine("UFRAME_NUM=" + config_.activeUF + " ;");
        }
        else {
            config_.activeUF = std::to_string(frameId);
            addLine("UFRAME_NUM=" + config_.activeUF + " ;");
            RunMessage("UF" + std::to_string(frameId) + ":" +
                std::to_string(xyzwpr[0]) + "," + std::to_string(xyzwpr[1]) + "," +
                std::to_string(xyzwpr[2]) + "," + std::to_string(xyzwpr[3]) + "," +
                std::to_string(xyzwpr[4]) + "," + std::to_string(xyzwpr[5]), true);
        }
    }

    void RobotPost::setTool(const robomath::Mat& pose, int toolId, const std::string& toolName) {
        Vector xyzwpr = pose_2_xyzrpw(pose);

        if (toolId < 0 || toolId > 9) {
            for (int i = 0; i < 6; ++i) {
                addLine("PR[" + std::to_string(config_.sparePR) + "," + std::to_string(i + 1) +
                    "]=" + std::to_string(xyzwpr[i]) + " ;");
            }
            for (int i = 6; i < robotAxes_; ++i) {
                addLine("PR[" + std::to_string(config_.sparePR) + "," + std::to_string(i + 1) + "]=0.0 ;");
            }
            addLine("UTOOL[" + config_.activeUT + "]=PR[" + std::to_string(config_.sparePR) + "] ;");
            addLine("UTOOL_NUM=" + config_.activeUT + " ;");
        }
        else {
            config_.activeUT = std::to_string(toolId);
            addLine("UTOOL_NUM=" + config_.activeUT + " ;");
            RunMessage("UT" + std::to_string(toolId) + ":" +
                std::to_string(xyzwpr[0]) + "," + std::to_string(xyzwpr[1]) + "," +
                std::to_string(xyzwpr[2]) + "," + std::to_string(xyzwpr[3]) + "," +
                std::to_string(xyzwpr[4]) + "," + std::to_string(xyzwpr[5]), true);
        }
    }

    void RobotPost::setSpeed(double speed_mms) {
        if (speedBackup_.empty()) {
            config_.speed = std::to_string(static_cast<int>((speed_mms>0.01)? speed_mms: 0.01)) + "mm/sec";
            double temp = 100.0 * speed_mms / 5000.0;
            double clamped_value = (temp > 100.0) ? 100.0 : (temp < 1.0) ? 1.0 : temp;
            config_.jointSpeed = std::to_string(static_cast<int>(clamped_value)) + "%";
        }
        else {
            speedBackup_ = std::to_string(static_cast<int>((speed_mms>0.01)? speed_mms: 0.01)) + "mm/sec";
        }
    }

    void RobotPost::setAcceleration(double accel_mmss) {
        addLog("setAcceleration not defined");
    }

    void RobotPost::setSpeedJoints(double speed_degs) {
        double temp = 100.0 * speed_degs / 200.0;
        double clamped_value = (temp > 100.0) ? 100.0 : (temp < 1.0) ? 1.0 : temp;
        config_.jointSpeed = std::to_string(static_cast<int>(clamped_value)) + "%";
    }

    void RobotPost::setAccelerationJoints(double accel_degss) {
        addLog("setAccelerationJoints not defined");
    }

    void RobotPost::setZoneData(double zone_mm) {
        if (zone_mm < 0) {
            config_.cntValue = "FINE";
        }
        else {
            config_.cntValue = "CNT" + std::to_string(static_cast<int>((zone_mm<100.0)?zone_mm :100.0));
        }
    }

    // I/O 操作
    void RobotPost::setDO(const std::string& io_var, const std::string& io_value) {
        std::string var_name = io_var;
        std::string value_str = io_value;

        if (std::isdigit(io_var[0])) {
            var_name = "DO[" + io_var + "]";
        }

        if (std::isdigit(io_value[0])) {
            int value = std::stoi(io_value);
            value_str = (value > 0) ? "ON" : "OFF";
        }

        addLine(var_name + "=" + value_str + " ;");
    }

    void RobotPost::setAO(const std::string& io_var, const std::string& io_value) {
        setDO(io_var, io_value);
    }

    void RobotPost::waitDI(const std::string& io_var, const std::string& io_value, int timeout_ms) {
        std::string var_name = io_var;
        std::string value_str = io_value;

        if (std::isdigit(io_var[0])) {
            var_name = "DI[" + io_var + "]";
        }

        if (std::isdigit(io_value[0])) {
            int value = std::stoi(io_value);
            value_str = (value > 0) ? "ON" : "OFF";
        }

        if (timeout_ms < 0) {
            addLine("WAIT " + var_name + "=" + value_str + " ;");
        }
        else {
            static int lbl_count = 0;
            lbl_count++;
            addLine("$WAITTMOUT=" + std::to_string(timeout_ms) + " ;");
            addLine("WAIT " + var_name + "=" + value_str + " TIMEOUT, LBL[" +
                std::to_string(lbl_count) + "] ;");
            addLine("MESSAGE[Timed out for LBL " + std::to_string(lbl_count) + "] ;");
            addLine("PAUSE ;");
            addLine("LBL[" + std::to_string(lbl_count) + "] ;");
        }
    }

    // 代码生成
    void RobotPost::RunCode(const std::string& code, bool is_function_call) {
        if (is_function_call) {
            std::string code_lower = code;
            std::transform(code_lower.begin(), code_lower.end(), code_lower.begin(), ::tolower);

            if (code_lower.find("arcstart") != std::string::npos) {
                arcParams_ = code.substr(9);
                if (arcParams_.empty()) {
                    arcParams_ = "2.0Hz,8.0mm,0.075s,0.075";
                }

                // 强制特定焊接速度
                speedBackup_ = config_.speed;
                config_.speed = "R[20]";

                addLine("Arc Start[11]");
                addLine("Weave Sine[" + arcParams_ + "] ;");
                return;
            }
            else if (code_lower.find("arcend") != std::string::npos) {
                addLine("Arc End[11]");

                // 恢复速度
                if (!speedBackup_.empty()) {
                    config_.speed = speedBackup_;
                    speedBackup_.clear();
                }

                arcParams_.clear();
                return;
            }

            std::string code_safe = getSafeName(code, 12);
            std::replace(code_safe.begin(), code_safe.end(), ' ', '_');
            addLine("CALL " + code_safe + " ;");
        }
        else {
            if (!code.empty() && code.back() != ';') {
                addLine(code + ";");
            }
            else {
                addLine(code);
            }
        }
    }

    void RobotPost::RunMessage(const std::string& message, bool is_comment) {
        if (is_comment) {
            // 分割长注释
            for (size_t i = 0; i < message.length(); i += 20) {
                size_t end = ((i + 20)<message.length())? (i + 20):message.length();
                addLine("! " + message.substr(i, end - i) + " ;");
            }
        }
        else {
            // 分割长消息
            for (size_t i = 0; i < message.length(); i += 20) {
                size_t end = ((i + 20)<message.length())? (i + 20) : message.length();
                addLine("MESSAGE[" + message.substr(i, end - i) + "] ;");
            }
        }
    }

    void RobotPost::Pause(int time_ms) {
        if (time_ms <= 0) {
            addLine("PAUSE ;");
        }
        else {
            std::stringstream ss;
            ss << "WAIT  " << std::fixed << std::setprecision(2) << (time_ms * 0.001) << "(sec) ;";
            addLine(ss.str());
        }
    }

    // 私有方法实现
    void RobotPost::addLine(const std::string& newline, char moveType) {
        if (nProgs_ > 1 && !config_.includeSubPrograms) {
            return;
        }

        pageSizeControl();

        currentProg_.lineCount++;
        std::string line_ok = std::to_string(currentProg_.lineCount) + ":" +
            std::string(1, moveType) + " " + newline;
        currentProg_.lines.push_back(line_ok);
    }

    void RobotPost::addLineTargets(const std::string& newline) {
        currentProg_.targets.push_back(newline);
    }

    void RobotPost::addLog(const std::string& newline) {
        if (nProgs_ > 1 && !config_.includeSubPrograms) {
            return;
        }
        log_ += newline + "\n";
    }

    void RobotPost::pageSizeControl() {
        if (currentProg_.lineCount >= config_.maxLinesPerProgram) {
            ProgFinish(currentProg_.name);
            ProgStart(currentProg_.name);
        }
    }

    int RobotPost::addTargetJoints(const robomath::Mat& pose, const std::vector<double>& joints) {
        if (nProgs_ > 1 && !config_.includeSubPrograms) {
            return 0;
        }

        currentProg_.pCount++;
        std::string add_comma = hasTrack_ ? "," : "";

        std::stringstream ss;
        ss << "P[" << currentProg_.pCount << "]{\n";
        ss << "   GP1:\n";
        ss << "    UF : " << config_.activeUF << ", UT : " << config_.activeUT << ",    \n";
        ss << "\tJ1=    " << std::fixed << std::setprecision(3) << joints[0] << " deg,\t";
        ss << "J2=    " << joints[1] << " deg,\t";
        ss << "J3=    " << joints[2] << " deg,\n";
        ss << "\tJ4=    " << joints[3] << " deg,\t";
        ss << "J5=    " << joints[4] << " deg,\t";
        ss << "J6=    " << joints[5] << " deg" << add_comma << "\n";

        if (hasTrack_) {
            for (size_t i = 0; i < axesTrack_.size(); ++i) {
                ss << "\tE" << (i + 1) << "=" << std::setw(10) << joints[axesTrack_[i]] << "  mm";
                if (i < axesTrack_.size() - 1) ss << ",";
            }
            ss << "\n";
        }

        if (hasTurntable_) {
            ss << "   GP2:\n";
            ss << "    UF : " << config_.activeUF << ", UT : " << config_.activeUT << ",\n";
            for (size_t i = 0; i < axesTurntable_.size(); ++i) {
                ss << "\tJ" << (i + 1) << "=" << std::setw(10) << joints[axesTurntable_[i]] << " deg";
                if (i < axesTurntable_.size() - 1) ss << ",";
            }
            ss << "\n";
        }

        ss << "};";
        addLineTargets(ss.str());

        return currentProg_.pCount;
    }

    int RobotPost::addTargetCartesian(const robomath::Mat& pose, const std::vector<double>& joints,
        const std::vector<double>& confRLF) {
        if (nProgs_ > 1 && !config_.includeSubPrograms) {
            return 0;
        }

        auto angle_2_turn = [](double angle) {
            if (angle >= 0.0) {
                return +std::floor((angle + 180.0) / 360.0);
            }
            else {
                return -std::floor((-angle + 180.0) / 360.0);
            }
        };

        Vector xyzwpr = pose_2_xyzrpw(pose);

        // 配置处理
        std::vector<std::string> config = { "N", "U", "T" };
        if (!confRLF.empty() && confRLF.size() >= 3) {
            if (confRLF[2] > 0) config[0] = "F";
            if (confRLF[1] > 0) config[1] = "D";
            if (confRLF[0] > 0) config[2] = "B";
        }

        int turnJ1 = angle_2_turn(joints[0]);
        int turnJ4 = angle_2_turn(joints[3]);
        int turnJ6 = angle_2_turn(joints[5]);

        currentProg_.pCount++;
        std::string add_comma = hasTrack_ ? "," : "";

        std::stringstream ss;
        ss << "P[" << currentProg_.pCount << "]{\n";
        ss << "   GP1:\n";
        ss << "    UF : " << config_.activeUF << ", UT : " << config_.activeUT << ",        ";
        ss << "CONFIG : '" << config[0] << " " << config[1] << " " << config[2] << ", ";
        ss << turnJ1 << ", " << turnJ4 << ", " << turnJ6 << "',\n";
        ss << "\tX =" << std::setw(10) << std::fixed << std::setprecision(3) << xyzwpr[0] << "  mm,\t";
        ss << "Y =" << std::setw(10) << xyzwpr[1] << "  mm,\t";
        ss << "Z =" << std::setw(10) << xyzwpr[2] << "  mm,\n";
        ss << "\tW =" << std::setw(10) << std::setprecision(3) << xyzwpr[3] << " deg,\t";
        ss << "P =" << std::setw(10) << xyzwpr[4] << " deg,\t";
        ss << "R =" << std::setw(10) << xyzwpr[5] << " deg" << add_comma << "\n";

        if (hasTrack_) {
            for (size_t i = 0; i < axesTrack_.size(); ++i) {
                ss << "\tE" << (i + 1) << "=" << std::setw(10) << joints[axesTrack_[i]] << "  mm";
                if (i < axesTrack_.size() - 1) ss << ",";
            }
            ss << "\n";
        }

        if (hasTurntable_) {
            ss << "   GP2:\n";
            ss << "    UF : " << config_.activeUF << ", UT : " << config_.activeUT << ",\n";
            for (size_t i = 0; i < axesTurntable_.size(); ++i) {
                ss << "\tJ" << (i + 1) << "=" << std::setw(10) << joints[axesTurntable_[i]] << " deg";
                if (i < axesTurntable_.size() - 1) ss << ",";
            }
            ss << "\n";
        }

        ss << "};";
        addLineTargets(ss.str());

        return currentProg_.pCount;
    }

    void RobotPost::saveProgram(const std::string& folder, const std::string& progname,
        bool askUser, const std::string& showResult) {
        std::string safeName = getSafeName(progname, 16);
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

        std::cout << "Program lines count: " << programToSave->lines.size() << std::endl;
        std::cout << "Program targets count: " << programToSave->targets.size() << std::endl;

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

        // 删除TP文件（如果存在）
        std::string filesave_TP = filesave.substr(0, filesave.find_last_of('.')) + ".TP";
        if (std::ifstream(filesave_TP)) {
            std::remove(filesave_TP.c_str());
        }

        // TP文件编译（如果配置了路径）
        if (!config_.pathMakeTP.empty() && std::ifstream(config_.pathMakeTP + "MakeTP.exe")) {
            std::cout << "POPUP: Compiling LS file with MakeTP.exe: " << safeName << "...\n";

            std::string command = config_.pathMakeTP + "MakeTP.exe \"" +
                filesave + "\" \"" + filesave_TP +
                "\" /config \"" + config_.pathMakeTP + "robot.ini\"";

            FILE* pipe = _popen(command.c_str(), "r");
            if (pipe) {
                char buffer[128];
                while (fgets(buffer, sizeof(buffer), pipe) != nullptr) {
                    log_ += buffer;
                    std::cout << "POPUP: " << buffer;
                }
                _pclose(pipe);
            }
            log_ += "\n";
        }

        // 显示结果
        if (!showResult.empty()) {
            ShellExecuteA(NULL, "open", filesave.c_str(), NULL, NULL, SW_SHOW);
        }
    }

    void RobotPost::generateDripFeedScript(const std::string& folder, const std::vector<std::string>& prognames,
        const std::string& scriptName, const std::string& progMain,
        const std::vector<std::string>& ftpParams) {
        std::string filename = folder + "/" + scriptName;

        std::string sFileExtention = ".LS";
        if (!config_.pathMakeTP.empty() && std::ifstream(config_.pathMakeTP + "MakeTP.exe")) {
            sFileExtention = ".TP";
        }

        std::vector<std::string> localProgNames;
        for (const auto& progName : prognames) {
            localProgNames.push_back(progName + sFileExtention);
        }

        std::vector<std::string> param_list = ftpParams;
        param_list.push_back(progMain + sFileExtention);

        std::stringstream progListStr;
        progListStr << "[";
        for (size_t i = 0; i < localProgNames.size(); ++i) {
            if (i > 0) progListStr << ",";
            progListStr << "\"" << localProgNames[i] << "\"";
        }
        progListStr << "]";
        param_list.push_back(progListStr.str());

        // 这里应该实现DRIPFEED_FILE模板的填充
        // 由于Python模板较长，这里简化处理
        std::ofstream fid(filename);
        fid << "# Drip Feed script generated by RobotSDK\n";
        fid << "RobotFTPIP = \"" << ftpParams[0] << "\"\n";
        fid << "RobotFTPPath = \"" << ftpParams[1] << "\"\n";
        fid << "RobotFTPUsername = \"" << ftpParams[2] << "\"\n";
        fid << "RobotFTPPassword = \"" << ftpParams[3] << "\"\n";
        fid << "fMainFile = \"" << progMain + sFileExtention << "\"\n";
        fid << "flFilesToSend = " << progListStr.str() << "\n";
        fid.close();

        std::cout << "POPUP: Running dripfeeder\n";
        std::string command = "start python \"" + filename + "\"";
        system(command.c_str());
    }
    bool RobotPost::FTPUploadFile(const std::string& server, const std::string& username,
        const std::string& password, const std::string& remotePath,
        const std::string& localFile, std::string& errorMsg) {

        HINTERNET hInternet = nullptr;
        HINTERNET hFtpSession = nullptr;
        HINTERNET hFtpFile = nullptr;
        HANDLE hLocalFile = INVALID_HANDLE_VALUE;
        bool success = false;

        // 从本地文件路径中提取文件名
        std::string fileName = localFile;
        size_t lastSlash = fileName.find_last_of("/\\");
        if (lastSlash != std::string::npos) {
            fileName = fileName.substr(lastSlash + 1);
        }

        // 构建远程文件路径
        std::string remoteFile = remotePath;
        if (!remoteFile.empty() && remoteFile.back() != '/') {
            remoteFile += "/";
        }
        remoteFile += fileName;

        // 打开本地文件
        hLocalFile = CreateFileA(localFile.c_str(), GENERIC_READ, FILE_SHARE_READ,
            NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);
        if (hLocalFile == INVALID_HANDLE_VALUE) {
            errorMsg = "无法打开本地文件: " + localFile + ", 错误代码: " + std::to_string(GetLastError());
            return false;
        }

        try {
            // 初始化WinINet
            hInternet = InternetOpenA("RobotSDK_FTP_Client", INTERNET_OPEN_TYPE_DIRECT, NULL, NULL, 0);
            if (!hInternet) {
                errorMsg = "InternetOpen失败: " + std::to_string(GetLastError());
                throw std::runtime_error("InternetOpen失败");
            }

            // 连接到FTP服务器
            hFtpSession = InternetConnectA(hInternet, server.c_str(), INTERNET_DEFAULT_FTP_PORT,
                username.c_str(), password.c_str(),
                INTERNET_SERVICE_FTP, INTERNET_FLAG_PASSIVE, 0);
            if (!hFtpSession) {
                errorMsg = "连接到FTP服务器失败: " + std::to_string(GetLastError());
                throw std::runtime_error("FTP连接失败");
            }

            // 设置传输模式为二进制
            if (!FtpSetCurrentDirectoryA(hFtpSession, remotePath.c_str())) {
                // 如果目录不存在，尝试创建
                if (!FtpCreateDirectoryA(hFtpSession, remotePath.c_str())) {
                    errorMsg = "无法创建或切换到远程目录: " + remotePath + ", 错误代码: " + std::to_string(GetLastError());
                    throw std::runtime_error("目录操作失败");
                }
                // 再次尝试切换目录
                if (!FtpSetCurrentDirectoryA(hFtpSession, remotePath.c_str())) {
                    errorMsg = "无法切换到远程目录: " + remotePath + ", 错误代码: " + std::to_string(GetLastError());
                    throw std::runtime_error("目录切换失败");
                }
            }

            // 打开远程文件用于写入
            hFtpFile = FtpOpenFileA(hFtpSession, fileName.c_str(), GENERIC_WRITE,
                FTP_TRANSFER_TYPE_BINARY, 0);
            if (!hFtpFile) {
                errorMsg = "无法打开远程文件: " + fileName + ", 错误代码: " + std::to_string(GetLastError());
                throw std::runtime_error("打开远程文件失败");
            }

            // 读取本地文件并上传
            const DWORD bufferSize = 4096;
            char buffer[bufferSize];
            DWORD bytesRead = 0;
            DWORD bytesWritten = 0;

            while (ReadFile(hLocalFile, buffer, bufferSize, &bytesRead, NULL) && bytesRead > 0) 
            {
                if (!InternetWriteFile(hFtpFile, buffer, bytesRead, &bytesWritten)) 
                {
                    errorMsg = "文件上传失败: " + std::to_string(GetLastError());
                    throw std::runtime_error("上传失败");
                }
                if (bytesWritten != bytesRead) 
                {
                    errorMsg = "写入字节数不匹配: 读取 " + std::to_string(bytesRead) +", 写入 " + std::to_string(bytesWritten);
                    throw std::runtime_error("write incomplete");
                }
            }

            success = true;
            std::cout << "成功上传: " << localFile << " -> " << remoteFile << std::endl;

        }
        catch (const std::exception& e) {
            std::cerr << "FTP上传错误: " << e.what() << std::endl;
            if (!errorMsg.empty()) {
                std::cerr << "错误详情: " << errorMsg << std::endl;
            }
            success = false;
        }

        // 清理资源
        if (hFtpFile) InternetCloseHandle(hFtpFile);
        if (hFtpSession) InternetCloseHandle(hFtpSession);
        if (hInternet) InternetCloseHandle(hInternet);
        if (hLocalFile != INVALID_HANDLE_VALUE) {
            CloseHandle(hLocalFile);
        }

        return success;
    }


    // 测试函数
    void test_post(std::string path) {
        auto Pose = robomath::PosePP;
        RobotPost robot("Fanuc_RJ3_DripFeed", "Fanuc Robot");

        robot.ProgStart("Curve3");
        robot.RunMessage("Program generated by me for Fanuc ARC Mate 120iC on 10/11/2018 10:34:26", true);
        robot.RunMessage("Using nominal kinematics.", true);

        // 设置坐标系和工具
        robot.setFrame(Pose(-13114.312225, 1641.490176, 3032.967400, -89.429405, -0.828158, 0.547393), -1, "Part Local Ref");
        robot.setTool(Pose(6.579694, -0.871557, 381.879694, 0.000000, 45.000000, 0.000000), -1, "Weld gun");

        robot.setSpeed(1000.000);

        // 初始MoveJ
        std::vector<double> joints1 = { 83.882578, -15.754442, -46.523597, 49.296707, 69.533346, -29.398461, 2984.678090, 2570.163700, 563.830565 };
        robot.MoveJ(robomath::Mat(), joints1, {});

        // MoveL序列
        std::vector<double> joints2 = { 83.882578, -15.754442, -46.523597, 49.296707, 69.533346, -29.398461, 2984.678090, 2570.163700, 563.830565 };
        robot.MoveL(Pose(-128.772331, 15775.117798, -2955.672119, 142.328568, 35.043147, -175.776553),
            joints2, { 0.0, 0.0, 1.0 });

        // 开始焊接
        robot.RunCode("ArcStart(2.0Hz,8.0mm,0.075s,0.075)", true);

        std::vector<double> joints3 = { 87.328162, -15.924572, -40.474499, 48.176067, 63.664228, -32.552985, 2984.678090, 2570.163700, 563.830565 };
        robot.MoveL(Pose(-87.947502, 15734.292969, -3037.321777, 142.328568, 35.043147, -175.776553),
            joints3, { 0.0, 0.0, 1.0 });

        robot.setSpeed(50.000);

        std::vector<double> joints4 = { 84.993829, -15.820725, -37.736589, 57.497002, 62.008852, -34.964581, 2984.678090, 2570.163700, 563.830565 };
        robot.MoveL(Pose(-87.947502, 15740.364258, -3038.315186, 143.296306, 39.222486, -169.641516),
            joints4, { 0.0, 0.0, 1.0 });

        std::vector<double> joints5 = { 83.865207, -17.204132, -23.104629, 94.375731, 49.260470, -53.900868, 2984.678090, 2570.163700, 563.830565 };
        robot.MoveL(Pose(-87.947502, 15753.342773, -3056.371826, 162.364741, 57.321011, -139.122540),
            joints5, { 0.0, 0.0, 1.0 });

        std::cout << "Done instruction: 10" << std::endl;

        // 重复位置（可能是停留）
        robot.MoveL(Pose(-87.947502, 15753.342773, -3056.371826, 162.364741, 57.321011, -139.122540),
            joints5, { 0.0, 0.0, 1.0 });

        robot.setSpeed(50.000);

        std::vector<double> joints6 = { 91.280214, -24.514099, -7.482821, 94.264844, 27.576637, 52.827242, 2984.678090, 2570.163700, 563.830565 };
        robot.MoveL(Pose(-87.947502, 15753.342773, -3323.071777, 178.450701, -27.782697, -41.102688),
            joints6, { 0.0, 0.0, 1.0 });

        std::vector<double> joints7 = { 82.536012, -9.623914, -3.656630, 110.966378, 66.076926, 91.743683, 2984.678090, 2570.163700, 563.830565 };
        robot.MoveL(Pose(-87.947502, 15734.292969, -3342.121826, 115.212292, -23.920623, 8.299772),
            joints7, { 0.0, 0.0, 1.0 });

        robot.setSpeed(50.000);

        std::vector<double> joints8 = { 109.719609, 2.464938, 10.538370, 161.900353, 67.639292, 65.645182, 2984.678090, 2570.163700, 563.830565 };
        robot.MoveL(Pose(-87.947502, 15467.592773, -3342.121826, 106.861708, 12.533873, 22.086494),
            joints8, { 0.0, 0.0, 1.0 });

        std::cout << "Done instruction: 20" << std::endl;

        robot.setSpeed(1000.000);

        // 结束焊接
        robot.RunCode("ArcEnd", true);

        // 返回移动
        robot.MoveL(Pose(-87.947502, 15467.592773, -3342.121826, 106.861708, 12.533873, 22.086494),
            joints8, { 0.0, 0.0, 1.0 });

        std::vector<double> joints9 = { 79.404467, -8.562284, -21.927019, 140.074273, 53.809210, 114.164583, 2984.678090, 2570.163700, 563.830565 };
        robot.MoveL(Pose(-93.977729, 15961.562547, -2860.212507, 106.861708, 12.533873, 22.086494),
            joints9, { 0.0, 0.0, 1.0 });

        robot.ProgFinish("Curve3");
        for (const auto& prgi : robot.progList_) {
            for (const auto& line : prgi.lines) {
                std::cout << line << std::endl;
            }
            std::cout << std::endl << std::endl;
        }
        // 调试输出
        std::cout << "=== Program Content ===" << std::endl;
        std::cout << "Lines count: " << robot.currentProg_.lines.size() << std::endl;
        std::cout << "Targets count: " << robot.currentProg_.targets.size() << std::endl;

        if (!robot.progList_.empty()) {
            std::cout << "Program list size: " << robot.progList_.size() << std::endl;
            std::cout << "First program lines: " << robot.progList_[0].lines.size() << std::endl;
        }

        if (!robot.log_.empty()) {
            std::cout << "Program generation LOG:\n" << robot.log_ << std::endl;
        }

        // 保存程序
        robot.ProgSave(path, "Curve3", false, "");
        std::cout << "Program generation completed successfully!" << std::endl;
    }
}