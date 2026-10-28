#include <PostModel/ABB_RAPID_IRC5.h>
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
#pragma execution_character_set("utf-8")

namespace ABB_RAPID_IRC5 {
    using namespace robomath;

    // 常量定义
    const std::vector<int> KNOWN_ZONEDATA = { 0,1,5,10,15,20,30,40,50,60,80,100,150,200 };
    const std::vector<int> KNOWN_SPEEDS = { 5,10,20,30,40,50,60,80,100,150,200,300,400,500,600,800,1000,1500,2000,2500,3000,4000,5000,6000,7000 };
    const std::vector<std::string> RESERVED_NAMES = { "tool0","work0","work1","clock1","reg1","reg2","reg3","reg4","reg5","alias","and","backward","case","connect","const","default","div","do","else","elseif","endfor","endfunc","endif","endmodule","endproc","endrecord","endtest","endtrap","endwhile","error","exit","false","for","from","func","goto","if","inout","local","mod","module","nostepin","not","noview","or","pers","proc","raise","readonly","record","retry","return","step","sysmodule","test","then","to","trap","true","trynext","undo","var","viewonly","while","with","xor" };
    const std::vector<std::string> MODULE_PREHEAD = { "%%%","  VERSION:1","  LANGUAGE:ENGLISH","%%%" };
    const std::string CUSTOM_HEADER = "    ! -------------------------------\n    ! Define customized variables here\n    ! ...";

    // 工具函数
    std::string RobotPost::FilterName(const std::string& namefilter, char safechar,
        const std::vector<std::string>& reserved_names) {
        std::string result = namefilter;
        const std::string forbidden = " .-[]/\\;,><&*:%=+@!#^|?^";

        // 移除非法字符
        for (char c : forbidden) {
            result.erase(std::remove(result.begin(), result.end(), c), result.end());
        }

        // 移除非ASCII字符
        result.erase(std::remove_if(result.begin(), result.end(),
            [](char c) { return c < 0 || c >= 127; }), result.end());

        // 确保非空
        if (result.empty()) {
            result = std::string(1, safechar);
        }

        // 确保不以数字开头
        if (!result.empty() && std::isdigit(result[0])) {
            result = std::string(1, safechar) + result;
        }

        // 避免保留名称冲突
        if (!reserved_names.empty()) {
            std::string result_lower = result;
            std::transform(result_lower.begin(), result_lower.end(), result_lower.begin(), ::tolower);

            int cnt = 1;
            std::string original = result;
            auto is_reserved = [&](const std::string& name) {
                std::string name_lower = name;
                std::transform(name_lower.begin(), name_lower.end(), name_lower.begin(), ::tolower);
                return name_lower == result_lower;
            };

            while (std::find_if(reserved_names.begin(), reserved_names.end(), is_reserved) != reserved_names.end()) {
                result = original + std::to_string(cnt);
                result_lower = result;
                std::transform(result_lower.begin(), result_lower.end(), result_lower.begin(), ::tolower);
                cnt++;
            }
        }

        return result;
    }

    std::string RobotPost::pose2Str(const Mat& pose) {
        std::vector<double> abb_pose = Pose_2_ABB(pose);
        std::stringstream ss;
        ss << std::fixed << std::setprecision(3);
        ss << "[" << abb_pose[0] << "," << abb_pose[1] << "," << abb_pose[2] << "],";
        ss << std::setprecision(8);
        ss << "[" << abb_pose[3] << "," << abb_pose[4] << "," << abb_pose[5] << "," << abb_pose[6] << "]";
        return ss.str();
    }

    std::string RobotPost::angles2Str(const std::vector<double>& angles) {
        std::vector<double> extended_angles = angles;
        if (extended_angles.size() < 6) {
            extended_angles.resize(6, 0.0);
        }

        std::stringstream ss;
        ss << "[";
        for (size_t i = 0; i < 6; ++i) {
            if (i > 0) ss << ",";
            ss << std::fixed << std::setprecision(6) << extended_angles[i];
        }
        ss << "]";
        return ss.str();
    }

    std::string RobotPost::extAxes2Str(const std::vector<double>& angles, const std::vector<double>& ratio)
    {
        if (angles.size() <= 6) {
            return "[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]";
        }

        std::vector<double> adjusted_angles(angles.begin() + 6, angles.end());
        if (!ratio.empty() && ratio.size() >= angles.size()) {
            for (size_t i = 6; i < angles.size(); ++i) {
                adjusted_angles[i - 6] = ratio[i] * angles[i];
            }
        }

        std::stringstream ss;
        ss << "[";
        for (size_t i = 0; i < adjusted_angles.size(); ++i) {
            if (i > 0) ss << ",";
            ss << std::fixed << std::setprecision(6) << adjusted_angles[i];
        }

        // 填充剩余的轴为9E9
        for (size_t i = adjusted_angles.size(); i < 6; ++i) {
            ss << ",9E9";
        }
        ss << "]";
        return ss.str();
    }

    // 构造函数
    //RobotPost::RobotPost(const std::string robotPost,const std::string robotName,int robotAxes,std::vector<std::string> axesType, const std::string ip_com, const Config config) : robotPost_(robotPost), robotName_(robotName), robotAxes_(robotAxes), axesType_(axesType), ip_com_(ip_com), config_(config)
    RobotPost::RobotPost(const std::string robotPost, const std::string robotName,
        int robotAxes, std::vector<std::string> axesType,
        const std::string ip_com, const Config config)
        : robotPost_(robotPost),
        robotName_(robotName),
        robotAxes_(robotAxes),
        axesType_(axesType),
        ip_com_(ip_com),
        config_(config),
        hasTrack_(false),
        hasTurntable_(false)
    {
        // 初始化其他变量
        nProgs_ = 0;
        tab_ = "";
        speedMMS_ = 500.0;
        speedData_ = "v500";
        zoneData_ = "z1";
        toolData_ = "tool0";
        wobjData_ = "wobj0";

        cladOn_ = false;
        arcOn_ = false;

        printPoseLast_ = Mat();
        printELast_ = 0.0;
        printENew_ = 0.0;
        printLastSignal_ = 0.0;
        frExternalPose_ = Mat();

        // 初始化工具和坐标系定义
        toolDef_.push_back("\t! Tool variables: ");
        frameDef_.push_back("\t! Reference variables:");
        // 初始化轴类型相关的变量
        for (size_t i = 0; i < axesType_.size(); ++i)
        {
            if (axesType_[i] == "T")
            {
                axesTrack_.push_back(i);
                hasTrack_ = true;
            }
            else if (axesType_[i] == "J")
            {
                axesTurntable_.push_back(i);
                hasTurntable_ = true;
            }
        }
    }

    // 程序控制方法
    void RobotPost::ProgStart(const std::string& progname) {
        std::string safeName = FilterName(progname, 'P', RESERVED_NAMES);
        nProgs_++;

        if (nProgs_ > 1 && !config_.includeSubPrograms) {
            return;
        }

        progNames_.push_back(safeName);
        currentProg_.name = safeName;

        tab_ = "\t\t"; // 两个制表符

        if (hasTrack_ || hasTurntable_) {
            addLine("! ActUnit " + config_.mechanicalUnitName + ";");
        }

        addLine("ConfJ \\On;");
        addLine("ConfL \\Off;");
    }

    void RobotPost::ProgFinish(const std::string& progname) {
        if (nProgs_ > 1 && !config_.includeSubPrograms) {
            return;
        }

        progList_.push_back(currentProg_);
        currentProg_ = ProgramData();
    }

    void RobotPost::saveModule(const std::string& filesave, const std::vector<std::string>& prog,
        const std::string& module_name) {
        std::ofstream fid(filesave);
        if (!fid.is_open()) {
            throw std::runtime_error("无法打开文件: " + filesave);
        }

        for (const auto& line : MODULE_PREHEAD) {
            fid << line << "\n";
        }

        fid << "MODULE " << module_name << "\n\n";
        for (const auto& line : prog) {
            fid << line << "\n";
        }
        fid << "\nENDMODULE\n";
        fid.close();

        progFiles_.push_back(filesave);
    }

    void RobotPost::ProgSave(const std::string& folder, const std::string& progname,
        bool askUser, const std::string& showResult) {
        // 这里实现程序保存逻辑，由于代码较长，只展示关键部分
        std::string safeName = FilterName(progname, 'P', RESERVED_NAMES);

        // 构建程序内容
        std::vector<std::string> module_addtop;
        std::vector<ProgramData> module_list_prog;
        std::vector<std::string> module_list_names;

        // 处理子程序
        size_t max_subprog_lines = ((config_.maxLinesPerProgram) < (config_.maxSubprogLines)) ? (config_.maxLinesPerProgram) : (config_.maxSubprogLines);
        for (size_t i = 0; i < progList_.size(); ++i) {
            if (i > 0 && progList_[i].lines.size() < max_subprog_lines) {
                std::string name_i = progNames_[i];
                std::vector<std::string> subprog_header = { "\tPROC " + name_i + "()" };
                std::vector<std::string> subprog_footer = { "\tENDPROC\n" };
                std::vector<std::string> subprog_complete = subprog_header;
                subprog_complete.insert(subprog_complete.end(), progList_[i].lines.begin(), progList_[i].lines.end());
                subprog_complete.insert(subprog_complete.end(), subprog_footer.begin(), subprog_footer.end());
                module_addtop.insert(module_addtop.end(), subprog_complete.begin(), subprog_complete.end());
            }
            else {
                module_list_prog.push_back(progList_[i]);
                module_list_names.push_back(progNames_[i]);
            }
        }

        // 保存模块文件
        for (size_t i = 0; i < module_list_prog.size(); ++i) {
            std::string prgi_name = module_list_names[i];
            std::string module_name = "MOD_" + prgi_name;
            std::string filesave = folder + "/" + prgi_name;

            if (i == 0 && config_.firstProgAsMain) {
                prgi_name = "Main";
            }

            // 构建程序头
            std::vector<std::string> prgi_header;
            if (i == 0) {
                prgi_header.push_back(CUSTOM_HEADER);
                prgi_header.push_back("");
                prgi_header.insert(prgi_header.end(), toolDef_.begin(), toolDef_.end());
                prgi_header.push_back("");
                prgi_header.insert(prgi_header.end(), frameDef_.begin(), frameDef_.end());
                prgi_header.push_back("");
            }

            prgi_header.insert(prgi_header.end(), module_addtop.begin(), module_addtop.end());
            prgi_header.push_back("\tPROC " + prgi_name + "()");

            std::vector<std::string> prgi_footer = { "\tENDPROC" };
            std::vector<std::string> prgi_complete = prgi_header;
            prgi_complete.insert(prgi_complete.end(), module_list_prog[i].lines.begin(), module_list_prog[i].lines.end());
            prgi_complete.insert(prgi_complete.end(), prgi_footer.begin(), prgi_footer.end());

            std::string filesave_complete = filesave + "." + config_.progExt;
            saveModule(filesave_complete, prgi_complete, module_name);

            std::cout << "SAVED: " << filesave_complete << "\n";
        }

        // 显示结果
        if (!showResult.empty()) {
            // 这里可以添加打开文件的逻辑
        }

        if (!log_.empty()) {
            std::cout << "Program generation LOG:\n\n" << log_ << "\n";
        }
    }

    void RobotPost::ProgSendRobot(const std::string& remote_path, const std::string& ftp_user, const std::string& ftp_pass) {
        // 这里实现FTP上传逻辑
        std::cout << "Sending programs to robot " << ip_com_ << std::endl;
        bool success = UploadFTP(progFiles_, ip_com_, remote_path, ftp_user, ftp_pass);

        if (success) {
            std::cout << "Upload successful!" << std::endl;
        }
        else {
            std::cout << "Upload failed!" << std::endl;
        }
        // 实际实现需要FTP库支持
    }

    // 运动指令
    void RobotPost::MoveJ(const Mat& pose, const std::vector<double>& joints,
        const std::vector<double>& confRLF) {
        std::string target = "[" + angles2Str(joints) + "," +
            extAxes2Str(joints, config_.axesRatio) + "]";
        addLine("MoveAbsJ " + target + "," + speedData_ + "," +
            zoneData_ + "," + toolData_ + " \\WObj:=" + wobjData_ + ";");
    }

    void RobotPost::MoveL(const Mat& pose, const std::vector<double>& joints,
        const std::vector<double>& confRLF) {
        std::string target;

        if (pose == Mat()) {
            target = "CalcRobT([" + angles2Str(joints) + "," +
                extAxes2Str(joints, config_.axesRatio) + "]," +
                toolData_ + " \\WObj:=" + wobjData_ + ")";
        }
        else {
            // 处理3D打印挤出机
            newMove(pose);

            // 构建目标字符串
            int cf1 = 0, cf4 = 0, cf6 = 0;
            if (joints.size() >= 6) {
                cf1 = static_cast<int>(std::floor(joints[0] / 90.0));
                cf4 = static_cast<int>(std::floor(joints[3] / 90.0));
                cf6 = static_cast<int>(std::floor(joints[5] / 90.0));
            }

            int cfx = 0;
            if (!confRLF.empty() && confRLF.size() >= 3) {
                cfx = 4 * static_cast<int>(confRLF[0]) +
                    2 * static_cast<int>(confRLF[1]) +
                    static_cast<int>(confRLF[2]);
            }

            target = "[" + pose2Str(pose) + ",[" +
                std::to_string(cf1) + "," + std::to_string(cf4) + "," +
                std::to_string(cf6) + "," + std::to_string(cfx) + "]," +
                extAxes2Str(joints, config_.axesRatio) + "]";
        }

        if (arcOn_) {
            addLine("ArcL " + target + "," + speedData_ + "," +
                arcSeamData_ + "," + arcWeldData_ +
                ",\\Weave:=" + arcWeaveData_ + "," +
                zoneData_ + "," + toolData_ + " \\WObj:=" + wobjData_ + ";");
        }
        else if (cladOn_) {
            addLine("CladL " + target + "," + speedData_ + "," +
                cladData_ + "," + zoneData_ + "," +
                toolData_ + " \\WObj:=" + wobjData_ + ";");
        }
        else {
            addLine("MoveL " + target + "," + speedData_ + "," +
                zoneData_ + "," + toolData_ + " \\WObj:=" + wobjData_ + ";");
        }
    }

    void RobotPost::MoveC(const Mat& pose1, const std::vector<double>& joints1, const Mat& pose2, const std::vector<double>& joints2, const std::vector<double>& confRLF1, const std::vector<double>& confRLF2)
    {
        std::string target1, target2;
        // 构建第一个目标点
        if (pose1 == Mat())
        {
            target1 = "CalcRobT([" + angles2Str(joints1) + "," +
                extAxes2Str(joints1, config_.axesRatio) + "]," +
                toolData_ + " \\WObj:=" + wobjData_ + ")";
        }
        else {
            int cf1_1 = 0, cf4_1 = 0, cf6_1 = 0;
            if (joints1.size() >= 6)
            {
                cf1_1 = static_cast<int>(std::floor(joints1[0] / 90.0));
                cf4_1 = static_cast<int>(std::floor(joints1[3] / 90.0));
                cf6_1 = static_cast<int>(std::floor(joints1[5] / 90.0));
            }

            int cfx_1 = 0;
            if (!confRLF1.empty() && confRLF1.size() >= 3)
            {
                cfx_1 = 4 * static_cast<int>(confRLF1[0]) +
                    2 * static_cast<int>(confRLF1[1]) +
                    static_cast<int>(confRLF1[2]);
            }

            target1 = "[" + pose2Str(pose1) + ",[" +
                std::to_string(cf1_1) + "," + std::to_string(cf4_1) + "," +
                std::to_string(cf6_1) + "," + std::to_string(cfx_1) + "]," +
                extAxes2Str(joints1, config_.axesRatio) + "]";
        }

        // 构建第二个目标点
        if (pose2 == Mat()) {
            target2 = "CalcRobT([" + angles2Str(joints2) + "," +
                extAxes2Str(joints2, config_.axesRatio) + "]," +
                toolData_ + " \\WObj:=" + wobjData_ + ")";
        }
        else {
            int cf1_2 = 0, cf4_2 = 0, cf6_2 = 0;
            if (joints2.size() >= 6) {
                cf1_2 = static_cast<int>(std::floor(joints2[0] / 90.0));
                cf4_2 = static_cast<int>(std::floor(joints2[3] / 90.0));
                cf6_2 = static_cast<int>(std::floor(joints2[5] / 90.0));
            }

            int cfx_2 = 0;
            if (!confRLF2.empty() && confRLF2.size() >= 3) {
                cfx_2 = 4 * static_cast<int>(confRLF2[0]) +
                    2 * static_cast<int>(confRLF2[1]) +
                    static_cast<int>(confRLF2[2]);
            }

            target2 = "[" + pose2Str(pose2) + ",[" +
                std::to_string(cf1_2) + "," + std::to_string(cf4_2) + "," +
                std::to_string(cf6_2) + "," + std::to_string(cfx_2) + "]," +
                extAxes2Str(joints2, config_.axesRatio) + "]";
        }

        if (arcOn_) {
            addLine("ArcC " + target1 + "," + target2 + "," +
                speedData_ + "," + arcSeamData_ + "," +
                arcWeldData_ + " \\Weave:=" + arcWeaveData_ +
                "," + zoneData_ + "," + toolData_ + " \\WObj:=" + wobjData_ + ";");
        }
        else if (cladOn_) {
            addLine("CladC " + target1 + "," + target2 + "," +
                speedData_ + "," + cladData_ + "," +
                zoneData_ + "," + toolData_ + " \\WObj:=" + wobjData_ + ";");
        }
        else {
            addLine("MoveC " + target1 + "," + target2 + "," +
                speedData_ + "," + zoneData_ + "," +
                toolData_ + " \\WObj:=" + wobjData_ + ";");
        }
    }

    // 设置函数
    void RobotPost::setFrame(const Mat& pose, int frameId, const std::string& frameName) {
        if (!frameName.empty()) {
            wobjData_ = FilterName(frameName, 'w', RESERVED_NAMES);
        }

        std::string set_data;
        if (hasTurntable_) {
            if (config_.turntableIgnore && frExternalPose_ != Mat()) {
                Mat adjusted_pose = frExternalPose_ * pose;
                set_data = wobjData_ + " := [FALSE, TRUE, \"\", [" +
                    pose2Str(adjusted_pose) + "],[[0,0,0],[1,0,0,0]]];";
            }
            else {
                set_data = wobjData_ + " := [FALSE, FALSE, \"" +
                    config_.mechanicalUnitName + "\", [" +
                    pose2Str(pose) + "],[[0,0,0],[1,0,0,0]]];";
            }
        }
        else {
            set_data = wobjData_ + " := [FALSE, TRUE, \"\", [" +
                pose2Str(pose) + "],[[0,0,0],[1,0,0,0]]];";
        }

        if (std::find(frameList_.begin(), frameList_.end(), wobjData_) != frameList_.end()) {
            addLine(set_data);
        }
        else {
            frameList_.push_back(wobjData_);
            frameDef_.push_back("\tPERS wobjdata " + set_data);
        }
    }

    void RobotPost::setTool(const Mat& pose, int toolId, const std::string& toolName) {
        if (!toolName.empty()) {
            toolData_ = FilterName(toolName, 't', RESERVED_NAMES);
        }

        std::string set_data = toolData_ + " := [TRUE,[" + pose2Str(pose) +
            "],[1,[0,0,20],[1,0,0,0],0,0,0.005]];";

        if (std::find(toolList_.begin(), toolList_.end(), toolData_) != toolList_.end()) {
            addLine(set_data);
        }
        else {
            toolList_.push_back(toolData_);
            toolDef_.push_back("\tPERS tooldata " + set_data);
        }
    }

    void RobotPost::setSpeed(double speed_mms) {
        speedMMS_ = speed_mms;

        // 检查是否为已知速度
        auto it = std::find(KNOWN_SPEEDS.begin(), KNOWN_SPEEDS.end(), static_cast<int>(speed_mms));
        if (it != KNOWN_SPEEDS.end()) {
            speedData_ = "v" + std::to_string(static_cast<int>(speed_mms));
        }
        else {
            std::stringstream ss;
            ss << "[" << std::fixed << std::setprecision(2) << speed_mms << ",500,5000,1000]";
            speedData_ = ss.str();
        }
    }

    void RobotPost::setAcceleration(double accel_mmss) {
        addLog("setAcceleration is not defined");
    }

    void RobotPost::setSpeedJoints(double speed_degs) {
        addLog("setSpeedJoints not defined");
    }

    void RobotPost::setAccelerationJoints(double accel_degss) {
        addLog("setAccelerationJoints not defined");
    }

    void RobotPost::setZoneData(double zone_mm) {
        if (zone_mm < 0) {
            zoneData_ = "fine";
        }
        else {
            // 检查是否为已知区域数据
            auto it = std::find(KNOWN_ZONEDATA.begin(), KNOWN_ZONEDATA.end(), static_cast<int>(zone_mm));
            if (it != KNOWN_ZONEDATA.end()) {
                zoneData_ = "z" + std::to_string(static_cast<int>(zone_mm));
            }
            else {
                double zone_mm_O = zone_mm * 1.5;
                double zone_mm_O2 = 0.1 * zone_mm_O;
                std::stringstream ss;
                ss << "[FALSE," << zone_mm << "," << zone_mm_O << "," << zone_mm_O << ","
                    << zone_mm_O2 << "," << zone_mm_O << "," << zone_mm_O2 << "]";
                zoneData_ = ss.str();
            }
        }
    }

    // I/O 操作
    void RobotPost::setDO(const std::string& io_var, const std::string& io_value) {
        std::string var_name = io_var;
        std::string value_str = io_value;

        // 处理数字IO
        if (std::isdigit(io_var[0])) {
            var_name = "D_OUT_" + io_var;
        }

        // 处理数字值
        if (std::isdigit(io_value[0])) {
            int value = std::stoi(io_value);
            value_str = (value > 0) ? "1" : "0";
        }

        addLine("SetDO " + var_name + ", " + value_str + ";");
    }

    void RobotPost::setAO(const std::string& io_var, const std::string& io_value) {
        setDO(io_var, io_value);
    }

    void RobotPost::waitDI(const std::string& io_var, const std::string& io_value,
        int timeout_ms) {
        std::string var_name = io_var;
        std::string value_str = io_value;

        // 处理数字IO
        if (std::isdigit(io_var[0])) {
            var_name = "D_IN_" + io_var;
        }

        // 处理数字值
        if (std::isdigit(io_value[0])) {
            int value = std::stoi(io_value);
            value_str = (value > 0) ? "1" : "0";
        }

        if (timeout_ms < 0) {
            addLine("WaitDI " + var_name + ", " + value_str + ";");
        }
        else {
            std::stringstream ss;
            ss << "WaitDI " << var_name << ", " << value_str
                << ", \\MaxTime:=" << (timeout_ms * 0.001) << ";";
            addLine(ss.str());
        }
    }

    void RobotPost::RunCode(const std::string& code, bool is_function_call)
    {
        if (is_function_call) {
            std::string code_lower = code;
            std::transform(code_lower.begin(), code_lower.end(), code_lower.begin(), ::tolower);

            if (code_lower.find("arclstart") != std::string::npos) {
                arcOn_ = true;
            }
            else if (code_lower.find("arclend") != std::string::npos) {
                arcOn_ = false;
            }
            else if (code_lower.find("cladlstart") != std::string::npos) {
                cladOn_ = true;
            }
            else if (code_lower.find("cladlend") != std::string::npos) {
                cladOn_ = false;
            }
            else if (code_lower.find("extruder(") != std::string::npos) {
                // 处理挤出机指令
                size_t start = code.find('(');
                size_t end = code.find(')');
                if (start != std::string::npos && end != std::string::npos && end > start + 1) {
                    std::string value_str = code.substr(start + 1, end - start - 1);
                    try {
                        printENew_ = std::stod(value_str);
                    }
                    catch (...) {
                        printENew_ = 0.0;
                    }
                }
                return; // 不生成程序调用
            }

            std::string code_safe = FilterName(code);
            addLine(code_safe + ";");
        }
        else {
            // 处理代码块
            if (code.find("END") == 0 || code.find("ELSEIF") == 0) {
                // 移除制表符
                if (tab_.size() >= 2) {
                    tab_ = tab_.substr(0, tab_.size() - 2);
                }
            }

            std::string processed_code = code;
            // 替换制表符为两个空格
            size_t pos = 0;
            while ((pos = processed_code.find('\t', pos)) != std::string::npos) {
                processed_code.replace(pos, 1, "  ");
                pos += 2;
            }
            addLine(processed_code);

            if (code.find("IF ") == 0 || code.find("ELSEIF ") == 0 || code.find("WHILE ") == 0) {
                // 添加制表符
                tab_ += "\t";
            }
        }
    }

    void RobotPost::RunMessage(const std::string& message, bool is_comment)
    {
        if (is_comment)
        {
            addLine("! " + message);
        }
        else
        {
            addLine("TPWrite \"" + message + "\";");
        }
    }

    void RobotPost::Pause(int time_ms)
    {
        if (time_ms <= 0)
        {
            addLine("STOP;");
        }
        else {
            std::stringstream ss;
            ss << "WaitTime " << std::fixed << std::setprecision(3) << (time_ms * 0.001) << ";";
            addLine(ss.str());
        }
    }

    // 私有方法实现
    void RobotPost::addLine(const std::string& newline)
    {
        if (nProgs_ > 1 && !config_.includeSubPrograms)
        {
            return;
        }

        currentProg_.lines.push_back(tab_ + newline);
    }

    void RobotPost::addLog(const std::string& newline)
    {
        if (nProgs_ > 1 && !config_.includeSubPrograms)
        {
            return;
        }
        log_ += newline + "\n";
    }

    double RobotPost::calculateTime(double distance, double Vmax, double Amax) {
        if (Amax < 0) {
            // 假设恒定速度
            return distance / Vmax;
        }
        else {
            // 假设加速和减速
            double tacc = Vmax / Amax;
            double Xacc = 0.5 * Amax * tacc * tacc;

            if (distance <= 2 * Xacc) {
                // 未达到Vmax
                tacc = std::sqrt(distance / Amax);
                return tacc * 2;
            }
            else {
                // 达到Vmax
                double Xvmax = distance - 2 * Xacc;
                double Tvmax = Xvmax / Vmax;
                return 2 * tacc + Tvmax;
            }
        }
    }

    void RobotPost::newMove(const Mat& new_pose)
    {
        if (printENew_ == 0.0 || new_pose == Mat())
        {
            return;
        }

        // 跳过第一次移动并记住位姿
        if (printPoseLast_ == Mat()) {
            printPoseLast_ = new_pose;
            return;
        }

        // 计算下一个运动的材料增加量
        double add_material = printENew_ - printELast_;
        printELast_ = printENew_;

        // 计算机器人速度和挤出机信号
        double extruder_signal = 0;
        if (add_material > 0)
        {
            // 计算距离（简化实现）
            double dx = new_pose(0, 3) - printPoseLast_(0, 3);
            double dy = new_pose(1, 3) - printPoseLast_(1, 3);
            double dz = new_pose(2, 3) - printPoseLast_(2, 3);
            double distance_mm = std::sqrt(dx * dx + dy * dy + dz * dz);

            // 计算运动时间（秒）
            double time_s = calculateTime(distance_mm, speedMMS_, printAccelMMSS_);

            // 避免除以零
            if (time_s > 0) {
                double speed_mms = distance_mm / time_s;
                extruder_signal = speed_mms * printSpeed2Signal_;
            }
        }
        // 确保信号在可接受范围内
        extruder_signal = (((0.0) > (((printFlowMaxSignal_) < (extruder_signal)) ? (printFlowMaxSignal_) : (extruder_signal))) ? (0.0) : (((printFlowMaxSignal_) < (extruder_signal)) ? (printFlowMaxSignal_) : (extruder_signal)));
        // 在需要时更新挤出机速度
        if (printLastSignal_ == 0.0 || std::abs(extruder_signal - printLastSignal_) > 1e-6)
        {
            printLastSignal_ = extruder_signal;
            std::stringstream ss;
            ss << "ExtruderSpeed " << std::fixed << std::setprecision(3) << extruder_signal << ";";
            addLine(ss.str());
        }
        // 记住最后位姿
        printPoseLast_ = new_pose;
    }

    void RobotPost::prog2List() {
        // 将当前程序数据添加到程序列表中
        if (!currentProg_.lines.empty()) {
            progList_.push_back(currentProg_);

            // 重置当前程序数据，但保留名称
            std::string name = currentProg_.name;
            currentProg_ = ProgramData();
            currentProg_.name = name;

            // 重新初始化制表符
            tab_ = "\t\t";
        }
    }

    // 测试函数
    void test_post(std::string path) {
        auto Pose = robomath::PosePP;
        //"Test the post with a basic program"
        RobotPost robot("ABB_RAPID_IRC5", "ABB IRB 6700-155/2.85");
        robot.ProgStart("Prog1");
        robot.RunMessage("Program generated by me for ABB IRB 6700-155/2.85 on 18/05/2017 11:02:41", true);
        robot.RunMessage("Using nominal kinematics.", true);
        robot.setFrame(Pose(0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000), -1, "ABB IRB 6700-155/2.85 Base");
        robot.setTool(Pose(380.000000, 0.000000, 200.000000, 0.000000, 90.000000, 0.000000), 1, "Tool 1");
        robot.setSpeed(2000.000);
        robot.MoveJ(Pose(2103.102861, 0.000000, 1955.294643, -180.000000, -3.591795, -180.000000), { 0.00000, 3.93969, -14.73451, 0.00000, 14.38662, -0.00000 }, { 0.0, 0.0, 0.0 });
        robot.MoveJ(Pose(2065.661612, 700.455189, 1358.819971, 180.000000, -3.591795, -180.000000), { 22.50953, 5.58534, 8.15717, 67.51143, -24.42689, -64.06258 }, { 0.0, 0.0, 1.0 });
        robot.Pause(500.0);
        robot.setSpeed(100.000);
        //robot.RunCode(r'ArcLStart', true);
        robot.MoveL(Pose(2065.661612, 1074.197508, 1358.819971, 149.453057, -3.094347, -178.175378), { 36.19352, 22.86988, -12.37860, 88.83085, -66.57439, -81.72795 }, { 0.0, 0.0, 1.0 });
        robot.MoveC(Pose(2468.239418, 1130.614560, 1333.549802, -180.000000, -3.591795, -180.000000), { 28.37934, 35.45210, -28.96667, 85.54799, -28.41204, -83.00289 }, Pose(2457.128674, 797.241647, 1156.545094, 180.000000, -37.427062, -180.000000), { 18.58928, 43.77805, -40.05410, 155.58093, -37.76022, -148.70252 }, { 0.0, 0.0, 1.0 }, { 0.0, 0.0, 1.0 });
        robot.MoveL(Pose(2457.128674, 797.241647, 1156.545094, 180.000000, -37.427062, -180.000000), { 18.58928, 43.77805, -40.05410, 155.58093, -37.76022, -148.70252 }, { 0.0, 0.0, 1.0 });
        robot.MoveL(Pose(2469.684137, 397.051453, 1356.565545, -180.000000, -3.591795, -180.000000), { 10.73523, 21.17902, -10.22963, 56.13802, -12.93695, -54.77268 }, { 0.0, 0.0, 1.0 });
        robot.MoveL(Pose(2494.452316, 404.343933, 1751.146172, -180.000000, -3.591795, -180.000000), { 10.80299, 25.05092, -31.54821, 132.79244, -14.76878, -133.06820 }, { 0.0, 0.0, 1.0 });
        robot.MoveL(Pose(2494.452316, 834.649436, 1751.146172, -180.000000, -3.591795, -180.000000), { 21.49850, 33.45974, -43.37980, 121.21995, -25.32130, -122.42907 }, { 0.0, 0.0, 1.0 });
        robot.setZoneData(5.000);
        robot.MoveL(Pose(2147.781731, 834.649436, 1772.906995, -180.000000, -3.591795, -180.000000), { 25.21677, 13.65153, -17.95808, 107.03387, -26.40518, -107.19412 }, { 0.0, 0.0, 1.0 });
        robot.MoveL(Pose(2147.781731, 375.769504, 1772.906995, -180.000000, -3.591795, -180.000000), { 11.97030, 5.74930, -8.96838, 119.55454, -13.76610, -119.51539 }, { 0.0, 0.0, 1.0 });
        robot.MoveL(Pose(2147.781731, 61.363728, 1772.906995, -180.000000, -3.591795, -180.000000), { 1.98292, 3.75693, -6.84136, -16.54793, 6.96416, 16.55673 }, { 0.0, 0.0, 0.0 });
        //robot.RunCode(r'ArcLEnd', True)
        robot.MoveL(Pose(2147.781731, 275.581430, 1772.906995, -180.000000, -3.591795, -180.000000), { 8.83799, 4.80606, -7.95436, 127.27676, -11.11070, -127.24243 }, { 0.0, 0.0, 1.0 });
        robot.ProgFinish("Prog1");

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



