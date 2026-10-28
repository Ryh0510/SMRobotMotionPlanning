#include <PostModel/KUKA_custom.h>
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

namespace KUKA_custom {
    using namespace robomath;

    // 常量定义
    const std::string HEADER =
    ";FOLD INI\n"
    ";FOLD BASISTECH INI\n"
    "GLOBAL INTERRUPT DECL 3 WHEN $STOPMESS==TRUE DO IR_STOPM ( )\n"
    "INTERRUPT ON 3 \n"
    "BAS (#INITMOV,0 )\n"
    ";ENDFOLD (BASISTECH INI)\n"
    ";ENDFOLD (INI)\n"
    "\n"
    ";FOLD STARTPOSITION PTP, VEL 50%, A1 5,A2 -90,A3 100,A4 5,A5 10,A6 -5,E1 0\n"
    "$BWDSTART = FALSE\n"
    "PDAT_ACT = {VEL 50,ACC 100,APO_DIST 10}\n"
    "BAS(#PTP_DAT)\n"
    "FDAT_ACT = {TOOL_NO 0,BASE_NO 0,IPO_FRAME #BASE}\n"
    "BAS(#FRAMES)\n"
    "BAS(#VEL_PTP,50)\n"
    ";PTP {A1 -22.431,A2 -98.536,A3 76.935,A4 121.862,A5 -38.504,A6 -77.334,E1 44.368 }\n"
    "\n"
    ";ENDFOLD\n"
    "\n"
    ";FOLD SET SPEED TO 50% PTP AND 0.2 m/sec. LIN\n"
    "$VEL.CP=0.5\n"
    "BAS(#VEL_PTP,50)\n"
    "BAS(#TOOL,0)\n"
    "BAS(#BASE,0)\n"
    ";ENDFOLD\n"
    "\n"
    "\n"
    "$BWDSTART = FALSE\n"
    "PDAT_ACT = {VEL 50,ACC 100,APO_DIST 10}\n"
    "FDAT_ACT = {TOOL_NO 0,BASE_NO 0,IPO_FRAME #BASE}\n"
    "BAS(#FRAMES)\n"
    "BAS(#VEL_PTP,50)\n"
    "$ADVANCE = 5\n"
    "\n"
    "\n"
    "; ---- PORGRAM START ----\n";

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
        nProgs_ = 0;
        tab_ = "";
        PROG_ = "";
        LOG_ = "";
        APO_VALUE_ = 0.0;
        C_DIS_ = "";
        C_PTP_ = "";
        progList_.clear();
        progNames_.clear();
        progFiles_.clear();
        currentProg_ = ProgramData();
    }

    // 程序控制方法
    void RobotPost::ProgStart(const std::string& progname) {
        nProgs_++;
        progNames_.push_back(progname);
        currentProg_ = ProgramData();
        currentProg_.name = progname;

        // 添加DEF行
        addLine("DEF " + progname + " ( )");

        // 添加HEADER内容
        std::istringstream header_stream(HEADER);
        std::string line;
        while (std::getline(header_stream, line)) {
            addLine(line);
        }
    }

    void RobotPost::ProgFinish(const std::string& progname) {
        addLine("END");
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

        // 保存程序内容
        fid << PROG_;
        fid.close();

        progFiles_.push_back(fullpath);
        std::cout << "SAVED: " << fullpath << "\n";

        if (!LOG_.empty()) {
            std::cout << "Program generation LOG:\n\n" << LOG_ << "\n";
        }
    }

    void RobotPost::ProgSendRobot(const std::string& remote_path,
        const std::string& ftp_user, const std::string& ftp_pass) {
        // FTP上传逻辑需要额外实现
        // 这里实现FTP上传逻辑
        std::cout << "Sending programs to robot " << ip_com_ << std::endl;
        bool success = UploadFTP(progFiles_,ip_com_,remote_path,ftp_user,ftp_pass);

        if (success) {
            std::cout << "Upload successful!" << std::endl;
        }
        else {
            std::cout << "Upload failed!" << std::endl;
        }
    }

    // 运动指令
    void RobotPost::MoveJ(const Mat& pose, const std::vector<double>& joints,
        const std::vector<double>& confRLF) {
        addLine("PTP {" + angles_2_str(joints) + "}");
    }

    void RobotPost::MoveL(const Mat& pose, const std::vector<double>& joints,
        const std::vector<double>& confRLF) {
        addLine("LIN {" + pose_2_str_ext(pose, joints) + "}");
    }

    void RobotPost::MoveC(const Mat& pose1, const std::vector<double>& joints1,
        const Mat& pose2, const std::vector<double>& joints2,
        const std::vector<double>& confRLF1, const std::vector<double>& confRLF2) {
        addLog("Circular move is not implemented");
    }

    // 设置函数
    void RobotPost::setFrame(const Mat& pose, int frameId, const std::string& frameName) {
        if (robotAxes_ <= 6) {
            addLine("$BASE = {FRAME: " + pose_2_str(pose) + "}");
        }
        else {
            addLine("$BASE = EK (MACHINE_DEF[2].ROOT, MACHINE_DEF[2].MECH_TYPE, { " + pose_2_str(pose) + " })");
        }
    }

    void RobotPost::setTool(const Mat& pose, int toolId, const std::string& toolName) {
        addLine("$TOOL = {FRAME: " + pose_2_str(pose) + "}");
    }

    void RobotPost::setSpeed(double speed_mms) {
        std::stringstream ss;
        ss << "$VEL.CP = " << std::fixed << std::setprecision(5) << (speed_mms / 1000.0);
        addLine(ss.str());
    }

    void RobotPost::setAcceleration(double accel_mmss) {
        std::stringstream ss;
        ss << "$ACC.CP = " << std::fixed << std::setprecision(5) << (accel_mmss / 1000.0);
        addLine(ss.str());
    }

    void RobotPost::setSpeedJoints(double speed_degs) {
        std::stringstream ss1, ss2;
        ss1 << "$VEL.ORI1 = " << std::fixed << std::setprecision(5) << speed_degs;
        ss2 << "$VEL.ORI2 = " << std::fixed << std::setprecision(5) << speed_degs;
        addLine(ss1.str());
        addLine(ss2.str());
    }

    void RobotPost::setAccelerationJoints(double accel_degss) {
        std::stringstream ss1, ss2;
        ss1 << "$ACC.ORI1 = " << std::fixed << std::setprecision(5) << accel_degss;
        ss2 << "$ACC.ORI2 = " << std::fixed << std::setprecision(5) << accel_degss;
        addLine(ss1.str());
        addLine(ss2.str());
    }

    void RobotPost::setZoneData(double zone_mm) {
        APO_VALUE_ = zone_mm;
        if (zone_mm >= 0) {
            std::stringstream ss1, ss2;
            ss1 << "$APO.CPTP = " << std::fixed << std::setprecision(3) << zone_mm;
            ss2 << "$APO.CDIS = " << std::fixed << std::setprecision(3) << zone_mm;
            addLine(ss1.str());
            addLine(ss2.str());
            C_DIS_ = " C_DIS";
            C_PTP_ = " C_PTP";
        }
        else {
            C_DIS_ = "";
            C_PTP_ = "";
        }
    }

    // I/O 操作
    void RobotPost::setDO(const std::string& io_var, const std::string& io_value) {
        std::string var_name = io_var;
        std::string value_str = io_value;

        if (std::isdigit(io_var[0])) {
            var_name = "$OUT[" + io_var + "]";
        }

        if (std::isdigit(io_value[0])) {
            int value = std::stoi(io_value);
            value_str = (value > 0) ? "TRUE" : "FALSE";
        }

        addLine(var_name + "=" + value_str);
    }

    void RobotPost::setAO(const std::string& io_var, const std::string& io_value) {
        setDO(io_var, io_value);
    }

    void RobotPost::waitDI(const std::string& io_var, const std::string& io_value, int timeout_ms) {
        std::string var_name = io_var;
        std::string value_str = io_value;

        if (std::isdigit(io_var[0])) {
            var_name = "$IN[" + io_var + "]";
        }

        if (std::isdigit(io_value[0])) {
            int value = std::stoi(io_value);
            value_str = (value > 0) ? "TRUE" : "FALSE";
        }

        if (timeout_ms < 0) {
            addLine("WAIT FOR (" + var_name + "==" + value_str + ")");
        }
        else {
            addLine("START_TIMER:");
            addLine("$TIMER_STOP[1]=TRUE");
            addLine("$TIMER_FLAG[1]=FALSE");
            std::stringstream ss;
            ss << "$TIMER[1]=" << std::fixed << std::setprecision(3) << (float(timeout_ms) * 0.001);
            addLine(ss.str());
            addLine("$TIMER_STOP[1]=FALSE");
            addLine("WAIT FOR (" + var_name + "==" + value_str + " OR $TIMER_FLAG[1])");
            addLine("$TIMER_STOP[1]=TRUE");
            addLine("IF $TIMER_FLAG[1]== TRUE THEN");
            addLine("    HALT ; Timed out!");
            addLine("    GOTO START_TIMER");
            addLine("ENDIF");
        }
    }

    // 代码生成
    void RobotPost::RunCode(const std::string& code, bool is_function_call) {
        if (is_function_call) {
            std::string code_safe = code;
            std::replace(code_safe.begin(), code_safe.end(), ' ', '_');

            // 检查字符串是否以")"结尾
            if (code_safe.size() == 0 || code_safe[code_safe.size() - 1] != ')') {
                code_safe += "()";
            }
            addLine(code_safe);
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
            addLine("Wait for StrClear($LOOP_MSG[])");
            addLine("$LOOP_CONT = TRUE");
            addLine("$LOOP_MSG[] = \"" + message + "\"");
        }
    }

    void RobotPost::Pause(int time_ms) {
        if (time_ms <= 0) {
            addLine("HALT");
        }
        else {
            std::stringstream ss;
            ss << "WAIT SEC " << std::fixed << std::setprecision(3) << (time_ms * 0.001);
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

    std::string RobotPost::pose2Str(const Mat& pose) {
        return pose_2_str(pose);
    }

    std::string RobotPost::joints2Str(const std::vector<double>& joints) {
        return angles_2_str(joints);
    }

    // 私有方法
    void RobotPost::addLine(const std::string& newline) {
        PROG_ = PROG_ + newline + "\n";
        if (nProgs_ > 0) {
            currentProg_.lines.push_back(newline);
        }
    }

    void RobotPost::addLog(const std::string& newline) {
        LOG_ = LOG_ + newline + "\n";
    }

    std::string RobotPost::pose_2_str(const Mat& pose) {
        const std::vector<double> xyzrpw = robomath::pose_2_xyzrpw(pose);
        std::stringstream ss;
        ss << "X " << std::fixed << std::setprecision(3) << xyzrpw[0]
            << ", Y " << xyzrpw[1]
            << ", Z " << xyzrpw[2]
            << ", A " << xyzrpw[5]
            << ", B " << xyzrpw[4]
            << ", C " << xyzrpw[3];
        return ss.str();
    }

    std::string RobotPost::pose_2_str_ext(const Mat& pose, const std::vector<double>& joints) {
        if (joints.size() <= 6) {
            return pose_2_str(pose);
        }
        else {
            std::string extaxes;
            for (size_t i = 0; i < joints.size() - 6; ++i) {
                std::stringstream ss;
                ss << ", E" << (i + 1) << " " << std::fixed << std::setprecision(5) << joints[i + 6];
                extaxes += ss.str();
            }
            return pose_2_str(pose) + extaxes;
        }
    }

    std::string RobotPost::angles_2_str(const std::vector<double>& angles) {
        std::stringstream ss;
        std::vector<std::string> data = { "A1", "A2", "A3", "A4", "A5", "A6", "E1", "E2", "E3", "E4", "E5", "E6" };

        for (size_t i = 0; i < angles.size() && i < data.size(); ++i) {
            ss << data[i] << " " << std::fixed << std::setprecision(5) << angles[i] << ",";
        }

        std::string result = ss.str();
        if (!result.empty()) {
            result.pop_back(); // 移除最后一个逗号
        }
        return result;
    }

    // 测试函数
    void test_post(std::string path) {
        auto Pose = robomath::PosePP;
        //"Test the post with a basic program"
        RobotPost robot("Kuka_custom", "Generic Kuka");
        robot.ProgStart("Program");
        robot.RunMessage("Program generated by hejun", true);
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
        robot.RunCode("TCP_Off", true);
        robot.Pause(1000);
        robot.setDO("12", "1");
        robot.waitDI("$IN[1]", "TRUE", 10);
        robot.MoveL(Pose(250, 300, 278.023897, 180, 0, -150), { -37.52588, -6.32628, -34.59693, 53.52525, 49.24426, -251.44677 });
        robot.MoveL(Pose(250, 200, 278.023897, 180, 0, -150), { -41.85389, -1.95619, -34.89154, 57.43912, 52.34162, -253.73403 });
        robot.MoveL(Pose(250, 150, 191.421356, 180, 0, -150), { -43.82111, 3.29703, -40.29493, 56.02402, 56.61169, -249.23532 });
        robot.ProgFinish("Program");

        for (const auto& prgi : robot.progList_) {
            for (const auto& line : prgi.lines) {
                std::cout << line << std::endl;
            }
            std::cout << std::endl << std::endl;
        }

        if (!robot.LOG_.empty()) {
            std::cout << "\n******************************************" << std::endl;
            std::cout << "PROGRAM GENERATION LOG:" << std::endl;
            std::cout << "******************************************" << std::endl;
            std::cout << robot.LOG_ << std::endl;
            std::cout << "******************************************" << std::endl;
        }

        robot.ProgSave(path, "Program", true, "");
        std::cout << "Program generation completed successfully!" << std::endl;
    }
}
