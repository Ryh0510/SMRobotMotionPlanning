#include <PostModel/Kawasaki.h>
#include <PostModel/robomath.h>
#include <PostModel/ftpsend.h>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <iostream>
#include <cctype>
#include <windows.h>
#include <shellapi.h>
#include <windows.h>
#pragma execution_character_set("utf-8")

namespace Kawasaki{
    using namespace robomath;
    // 工具函数
    std::vector<double> Pose_2_Adept(const Mat& pose) {
        return pose_2_xyzrpw(pose);
    }

    // 构造函数
    RobotPost::RobotPost(const std::string robotPost, const std::string robotName,
        int robotAxes, std::vector<std::string> axesType,
        const std::string ip_com, const Config config)
        : robotPost_(robotPost),
        robotName_(robotName),
        robotAxes_(robotAxes),
        axesType_(axesType),
        ip_com_(ip_com),
        config_(config)
    {
        // 初始化其他变量
        refFrame_ = robomath::eye(4);
        nProgs_ = 0;
        tab_ = "";
        weldParam_ = false;
        weldPoseLast_ = "";
        progList_.clear();
        progNames_.clear();
        progFiles_.clear();
        currentProg_ = ProgramData();
    }

    // 程序控制方法
    void RobotPost::ProgStart(const std::string& progname) {
        nProgs_++;
        if (nProgs_ > 1 && !config_.includeSubPrograms) {
            return;
        }
        progNames_.push_back(progname);
        //currentProg_ = ProgramData();  // 重置为默认构造
        //currentProg_.name = progname;
        addLine(".PROGRAM " + progname + "()");
        tab_ = "  ";
    }

    void RobotPost::ProgFinish(const std::string& progname) {
        if (nProgs_ > 1 && !config_.includeSubPrograms) {
            return;
        }

        weld_close();
        tab_ = "";
        addLine(".END");

        progList_.push_back(currentProg_);
        currentProg_ = ProgramData();
        nProgs_--;
    }

    void RobotPost::ProgSave(const std::string& folder, const std::string& progname,
        bool askUser, const std::string& showResult) {

        std::string filename = progname + "." + config_.progExt;
        std::string fullpath = folder + "/" + filename;

        std::ofstream fid(fullpath);
        if (!fid.is_open()) {
            throw std::runtime_error("无法打开文件: " + fullpath);
        }

        const ProgramData* program = nullptr;
        for (const ProgramData& candidate : progList_) {
            if(candidate.name == progname) {
                program = &candidate;
                break;
            }
        }
        if(program == nullptr && !progList_.empty()) {
            program = &progList_.back();
        }
        if(program == nullptr) {
            program = &currentProg_;
        }
        for (const auto& line : program->lines) {
            fid << line << "\n";
        }

        fid.close();
        progFiles_.push_back(fullpath);
        std::cout << "SAVED: " << fullpath << "\n";

        if (!log_.empty()) {
            std::cout << "Program generation LOG:\n\n" << log_ << "\n";
        }
    }

    void RobotPost::ProgSendRobot(const std::string& remote_path,
        const std::string& ftp_user, const std::string& ftp_pass) {
        // 这里实现FTP上传逻辑
        std::cout << "Sending programs to robot " << ip_com_ << std::endl;
        bool success = UploadFTP(progFiles_,ip_com_,remote_path,ftp_user,ftp_pass);

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
        if (weldParam_) {
            addLog("Welding with joint interpolation not supported! Use a linear movement instead.");
            return;
        }
        addLine("JMOVE #" + joints2Str(joints));
    }

    void RobotPost::MoveL(const Mat& pose, const std::vector<double>& joints,
        const std::vector<double>& confRLF) {
        if (weldParam_) {
            weldPoseLast_ = pose2Str(pose, &joints);
            if (weldParam_) {
                addLine("LWC " + weldPoseLast_ + ",1");
            }
            else {
                addLine("WC LINEAR OX= WX=  " + weldPoseLast_);
            }
            return;
        }
        addLine("LMOVE " + pose2Str(pose, &joints));
    }

    void RobotPost::MoveC(const Mat& pose1, const std::vector<double>& joints1,
        const Mat& pose2, const std::vector<double>& joints2,
        const std::vector<double>& confRLF1, const std::vector<double>& confRLF2) {

        if (weldParam_) {
            weldPoseLast_ = pose2Str(pose2, &joints2);
            if (weldParam_) {
                addLine("C1WC " + weldPoseLast_ + ",1");
                addLine("C2WC " + weldPoseLast_ + ",1");
            }
            else {
                addLine("WC CIR1 OX= WX=  " + pose2Str(pose1, &joints1));
                addLine("WC CIR2 OX= WX=  " + weldPoseLast_);
            }
            return;
        }
        addLine("C1MOVE " + pose2Str(pose1, &joints1));
        addLine("C2MOVE " + pose2Str(pose2, &joints2));
    }

    // 设置函数
    void RobotPost::setFrame(const Mat& pose, int frameId, const std::string& frameName) {
        addLine("BASE " + pose2Str(pose));
    }

    void RobotPost::setTool(const Mat& pose, int toolId, const std::string& toolName) {
        addLine("TOOL " + pose2Str(pose));
    }

    void RobotPost::setSpeed(double speed_mms) {
        speed_mms = (5000.0<speed_mms)? 5000.0:speed_mms;
        std::stringstream ss;
        ss << "SPEED " << std::fixed << std::setprecision(1) << speed_mms << " MM/S ALWAYS";
        addLine(ss.str());
    }

    void RobotPost::setAcceleration(double accel_mmss) {
        double temp = (0.0 > accel_mmss) ? 0.0 : accel_mmss;
        double accel_percentage = (100.0<temp)? 100.0: temp;
        std::stringstream ss;
        ss << "ACCEL " << std::fixed << std::setprecision(1) << accel_percentage << " " << accel_percentage;
        addLine(ss.str());
    }

    void RobotPost::setSpeedJoints(double speed_degs) {
        double speed_percentage = (100.0<(100.0 * speed_degs / 500.0))? 100.0 : (100.0 * speed_degs / 500.0);
        std::stringstream ss;
        ss << "SPEED " << std::fixed << std::setprecision(1) << speed_percentage << " ALWAYS";
        addLine(ss.str());
    }

    void RobotPost::setAccelerationJoints(double accel_degss) {
        addLog("setAccelerationJoints not defined");
    }

    void RobotPost::setZoneData(double zone_mm) {
        std::stringstream ss;
        ss << "ACCURACY " << std::fixed << std::setprecision(3) << zone_mm << " ALWAYS";
        addLine(ss.str());
    }

    // I/O 操作
    void RobotPost::setDO(const std::string& io_var, const std::string& io_value) {
        std::string var_name = io_var;
        std::string value_str = io_value;

        if (std::isdigit(io_var[0])) {
            var_name = io_var;
        }

        if (std::isdigit(io_value[0])) {
            int value = std::stoi(io_value);
            value_str = (value > 0) ? "" : "-";
        }

        addLine("SIGNAL " + value_str + var_name);
    }

    void RobotPost::setAO(const std::string& io_var, const std::string& io_value) {
        setDO(io_var, io_value);
    }

    void RobotPost::waitDI(const std::string& io_var, const std::string& io_value, int timeout_ms) {
        std::string var_name = io_var;
        std::string value_str = io_value;

        if (std::isdigit(io_var[0])) {
            var_name = io_var;
        }

        if (std::isdigit(io_value[0])) {
            int value = std::stoi(io_value);
            value_str = (value > 0) ? "" : "-";
        }

        if (timeout_ms > 0) {
            RunMessage("Timeout not implemented", true);
        }

        addLine("WAIT SIG(" + value_str + var_name + ")");
    }

    // 代码生成
    void RobotPost::RunCode(const std::string& code, bool is_function_call) {
        if (is_function_call) {
            std::string code_lower = code;
            std::transform(code_lower.begin(), code_lower.end(), code_lower.begin(), ::tolower);

            if (code_lower.find("weldstart") != std::string::npos) {
                std::string params = code.substr(9);
                if (!params.empty() && params[0] == '(' && params[params.size() - 1] == ')') {
                    params = params.substr(1, params.size() - 2);
                }

                if (params.empty()) {
                    weldParam_ = true;
                }
                else if (std::isdigit(params[0])) {
                    weldParam_ = true; // 使用数字参数
                }
                else {
                    weldParam_ = false; // 使用字符串参数
                }
                return;
            }
            else if (code_lower.find("weldend") != std::string::npos) {
                weld_close();
                weldParam_ = false;
                return;
            }

            std::string code_safe = FilterName(code);
            if (code_safe.find('(') == std::string::npos) {
                code_safe += "()";
            }
            addLine("CALL " + code_safe);
        }
        else {
            addLine(code);
        }
    }

    void RobotPost::RunMessage(const std::string& message, bool is_comment) {
        if (is_comment) {
            addLine("; " + message);
        }
        else {
            addLine("TYPE \"" + message + "\"");
        }
    }

    void RobotPost::Pause(int time_ms) {
        if (time_ms <= 0) {
            addLine("PAUSE");
        }
        else {
            std::stringstream ss;
            ss << "TWAIT " << std::fixed << std::setprecision(3) << (time_ms * 0.001);
            addLine(ss.str());
        }
    }

    // 工具函数
    std::string RobotPost::FilterName(const std::string& namefilter, char safechar,
        const std::vector<std::string>& reserved_names) {

        std::string result = namefilter;
        const std::string forbidden = " .-[]/\\;,><&*:%=+@!#^|?^";

        for (char c : forbidden) {
            result.erase(std::remove(result.begin(), result.end(), c), result.end());
        }

        result.erase(std::remove_if(result.begin(), result.end(),
            [](char c) { return c < 0 || c >= 127; }), result.end());

        if (result.empty()) {
            result = std::string(1, safechar);
        }

        if (!result.empty() && std::isdigit(result[0])) {
            result = std::string(1, safechar) + result;
        }

        return result;
    }

    std::string RobotPost::pose2Str(const Mat& pose, const std::vector<double>* joints, const Mat* reference) {
        if (pose == Mat()) {
            return joints2Str(*joints);
        }

        Mat final_pose = pose;
        if (reference != nullptr) {
            final_pose = *reference * pose;
        }

        std::vector<double> adept_pose = Pose_2_Adept(final_pose);
        std::stringstream ss;
        ss << "TRANS(";
        for (size_t i = 0; i < adept_pose.size(); ++i) {
            if (i > 0) ss << ",";
            ss << std::fixed << std::setprecision(3) << adept_pose[i];
        }
        ss << ")";
        return ss.str();
    }

    std::string RobotPost::joints2Str(const std::vector<double>& joints) {
        std::stringstream ss;
        ss << "[";
        for (size_t i = 0; i < joints.size(); ++i) {
            if (i > 0) ss << ",";
            ss << std::fixed << std::setprecision(5) << joints[i];
        }
        ss << "]";
        return ss.str();
    }

    // 私有方法
    void RobotPost::addLine(const std::string& newline) {
        if (nProgs_ > 1 && !config_.includeSubPrograms) {
            return;
        }
        currentProg_.lines.push_back(tab_ + newline);
    }

    void RobotPost::addLog(const std::string& newline) {
        if (nProgs_ > 1 && !config_.includeSubPrograms) {
            return;
        }
        log_ += newline + "\n";
    }

    void RobotPost::weld_close() {
        if (!weldPoseLast_.empty()) {
            if (weldParam_) {
                addLine("LWE " + weldPoseLast_ + ",1,1");
            }
            weldPoseLast_.clear();
        }
    }

    void test_post(std::string path) {
        auto Pose = robomath::PosePP;
        //"Test the post with a basic program"
        RobotPost robot("Kawasaki", "Kawasaki RS007N");
        robot.ProgStart("Oil");
        robot.RunMessage("Program generated by RobotSDK", true);
        robot.setFrame(Pose(807.766544, -963.699898, 41.478944, 0, 0, 0));
        robot.setTool(Pose(62.5, -108.253175, 100, -60, 90, 0));
        robot.MoveJ(Pose(200, 200, 500, 180, 0, 180), { -46.18419, -6.77518, -20.54925, 71.38674, 49.58727, -302.54752 });
        robot.MoveL(Pose(200, 250, 348.734575, 180, 0, -150), { -41.62707, -8.89064, -30.01809, 60.62329, 49.66749, -258.98418 });
        robot.MoveL(Pose(200, 200, 262.132034, 180, 0, -150), { -43.73892, -3.91728, -35.77935, 58.57566, 54.11615, -253.81122 });
        robot.RunMessage("Setting air valve 1 on");
        robot.RunCode("TCP_On", true);
        robot.Pause(1000);
        robot.MoveL(Pose(200, 250, 348.734575, 180, 0, -150), { -41.62707, -8.89064, -30.01809, 60.62329, 49.66749, -258.98418 });
        robot.MoveL(Pose(250, 300, 278.023897, 180, 0, -150), { -37.52588, -6.32628, -34.59693, 53.52525, 49.24426, -251.44677 });
        robot.MoveL(Pose(250, 250, 191.421356, 180, 0, -150), { -39.75778, -1.04537, -40.37883, 52.09118, 54.15317, -246.94403 });
        robot.RunMessage("Setting air valve off");
        robot.RunCode("TCP_Off(55)", true);
        robot.Pause(1000);
        robot.MoveL(Pose(250, 300, 278.023897, 180, 0, -150), { -37.52588, -6.32628, -34.59693, 53.52525, 49.24426, -251.44677 });
        robot.MoveL(Pose(250, 200, 278.023897, 180, 0, -150), { -41.85389, -1.95619, -34.89154, 57.43912, 52.34162, -253.73403 });
        robot.MoveL(Pose(250, 150, 191.421356, 180, 0, -150), {-43.82111, 3.29703, -40.29493, 56.02402, 56.61169, -249.23532 });
        robot.ProgFinish("Program");
        for (const auto& prgi : robot.progList_) {
            for (const auto& line : prgi.lines) {
                std::cout << line << std::endl;
            }
            std::cout << std::endl << std::endl;
        }
        if (!robot.log_.empty()) {
            std::cout << "\n******************************************" << std::endl;
            std::cout << "PROGRAM GENERATION LOG:" << std::endl;
            std::cout << "******************************************" << std::endl;
            std::cout << robot.log_ << std::endl;
            std::cout << "******************************************" << std::endl;
        }
        robot.ProgSave(path, "Oil", true, "");
        std::cout << "Program generation completed successfully!" << std::endl;
    }
}
