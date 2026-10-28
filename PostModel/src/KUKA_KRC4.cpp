#include <PostModel/KUKA_KRC4.h>
#include <PostModel/ftpsend.h>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <iostream>
#include <windows.h>
#include <shellapi.h>


namespace KUKA_KRC4{
    using namespace std;

    // 常量定义
    const vector<string> RESERVED_NAMES = {
        "tool0", "work0", "work1", "clock1", "reg1", "reg2", "reg3", "reg4", "reg5"
    };

    const string RobotPost::HEADER = R"(;FOLD INI
    BAS (#INITMOV,0 )
    ;ENDFOLD (INI)

    ;FOLD STARTPOS
    $BWDSTART = FALSE
    PDAT_ACT = {VEL 50,ACC 100,APO_DIST 10}
    BAS(#PTP_DAT)
    FDAT_ACT = {TOOL_NO 0,BASE_NO 0,IPO_FRAME #BASE}
    BAS (#FRAMES)
    BAS (#VEL_PTP,50)
    ;ENDFOLD

    ;FOLD SET DEFAULT SPEED
    $VEL.CP=0.2
    BAS(#VEL_PTP,50)
    BAS(#TOOL,0)
    BAS(#BASE,0)
    ;ENDFOLD

    ;FOLD PTP FIRST POSITION
    $BWDSTART = FALSE
    PDAT_ACT = {VEL 50,ACC 100,APO_DIST 10}
    FDAT_ACT = {TOOL_NO 0,BASE_NO 0,IPO_FRAME #BASE}
    BAS(#FRAMES)
    $ADVANCE = 5
    ;ENDFOLD

    PTP $AXIS_ACT ; skip BCO quickly
    )";

    // 构造函数
    RobotPost::RobotPost(const string robotPost, const string robotName,
        int robotAxes, vector<string> axesType,
        const string ip_com, const Config config)
        : robotPost_(robotPost),
        robotName_(robotName),
        robotAxes_(robotAxes),
        axesType_(axesType),
        ip_com_(ip_com),
        config_(config)
    {
        nProgs_ = 0;
        nLines_ = 0;
        APO_VALUE_ = 1;
        C_DIS_ = "";
        C_PTP_ = "";
        toolData_ = "tool0";
        wobjData_ = "wobj0";
    }

    // 工具函数
    string RobotPost::FilterName(const string& namefilter, char safechar,
        const vector<string>& reserved_names) {
        string result = namefilter;
        const string forbidden = " .-[]/\\;,><&*:%=+@!#^|?^";

        // 移除非法字符
        for (char c : forbidden) {
            result.erase(remove(result.begin(), result.end(), c), result.end());
        }

        // 移除非ASCII字符
        result.erase(remove_if(result.begin(), result.end(),
            [](char c) { return c < 0 || c >= 127; }), result.end());

        // 确保非空
        if (result.empty()) {
            result = string(1, safechar);
        }

        // 确保不以数字开头
        if (!result.empty() && isdigit(result[0])) {
            result = string(1, safechar) + result;
        }

        // 避免保留名称冲突
        if (!reserved_names.empty()) {
            string result_lower = result;
            transform(result_lower.begin(), result_lower.end(), result_lower.begin(), ::tolower);

            int cnt = 1;
            string original = result;
            auto is_reserved = [&](const string& name) {
                string name_lower = name;
                transform(name_lower.begin(), name_lower.end(), name_lower.begin(), ::tolower);
                return name_lower == result_lower;
            };

            while (find_if(reserved_names.begin(), reserved_names.end(), is_reserved) != reserved_names.end()) {
                result = original + to_string(cnt);
                result_lower = result;
                transform(result_lower.begin(), result_lower.end(), result_lower.begin(), ::tolower);
                cnt++;
            }
        }

        return result;
    }

    string RobotPost::pose2Str(const robomath::Mat& pose) {
        vector<double> xyzrpw = robomath::pose_2_xyzrpw(pose);
        stringstream ss;
        ss << fixed << setprecision(3);
        ss << "X " << xyzrpw[0] << ",Y " << xyzrpw[1] << ",Z " << xyzrpw[2]
            << ",A " << xyzrpw[5] << ",B " << xyzrpw[4] << ",C " << xyzrpw[3];
        return ss.str();
    }

    string RobotPost::angles2Str(const vector<double>& angles) {
        stringstream ss;
        ss << "AXIS: ";
        vector<string> data = { "A1","A2","A3","A4","A5","A6","E1","E2","E3","E4","E5","E6" };

        for (size_t i = 0; i < angles.size(); ++i) {
            if (i > 0) ss << ",";
            ss << data[i] << " " << fixed << setprecision(5) << angles[i];
        }

        return ss.str();
    }

    string RobotPost::extAxes2Str(const vector<double>& angles, const vector<double>& ratio) {
        if (angles.size() <= 6) {
            return "";
        }

        stringstream ss;
        for (size_t i = 6; i < angles.size(); ++i) {
            if (i > 6) ss << ",";
            ss << "E" << (i - 5) << " " << fixed << setprecision(5) << angles[i];
        }

        return ss.str();
    }

    // 程序控制方法
    void RobotPost::ProgStart(const string& progname) {
        string safeName = FilterName(progname, 'P', RESERVED_NAMES);
        nProgs_++;

        if (nProgs_ > 1 && !config_.includeSubPrograms) {
            return;
        }

        progNames_.push_back(safeName);
        currentProg_.name = safeName;

        PROG_ = "DEF " + safeName + " ( )\n";
        PROG_ += HEADER;

        if (robotAxes_ > 6) {
            addLine("$ACT_EX_AX = " + to_string(robotAxes_ - 6));
        }
    }

    void RobotPost::ProgFinish(const string& progname) {
        if (nProgs_ > 1 && !config_.includeSubPrograms) {
            return;
        }

        addLine("END");
        currentProg_.lines.push_back(PROG_);
        progList_.push_back(currentProg_);
        PROG_.clear();
        nLines_ = 0;
    }

    void RobotPost::ProgSave(const string& folder, const string& progname,
        bool askUser, const string& showResult) {
        saveProgram(folder, progname, askUser, showResult);
    }

    void RobotPost::ProgSendRobot(const string& remote_path,
        const string& ftp_user, const string& ftp_pass) {
        // 这里实现FTP上传逻辑
        std::cout << "Sending programs to robot " << ip_com_ << std::endl;
        bool success = UploadFTP(progFiles_, ip_com_, remote_path, ftp_user, ftp_pass);

        if (success) {
            std::cout << "Upload successful!" << std::endl;
        }
        else {
            std::cout << "Upload failed!" << std::endl;
        }
    }

    // 运动指令
    void RobotPost::MoveJ(const robomath::Mat& pose, const vector<double>& joints,
        const vector<double>& confRLF) {
        addLine("PTP {" + angles2Str(joints) + "}" + C_PTP_);
    }

    void RobotPost::MoveL(const robomath::Mat& pose, const vector<double>& joints,
        const vector<double>& confRLF) {
        string extAxes = extAxes2Str(joints);
        string poseStr = pose2Str(pose);

        if (!extAxes.empty()) {
            poseStr += "," + extAxes;
        }

        addLine("LIN {" + poseStr + "}" + C_DIS_);
    }

    void RobotPost::MoveC(const robomath::Mat& pose1, const vector<double>& joints1,
        const robomath::Mat& pose2, const vector<double>& joints2,
        const vector<double>& confRLF1, const vector<double>& confRLF2) {
        string extAxes1 = extAxes2Str(joints1);
        string poseStr1 = pose2Str(pose1);
        string extAxes2 = extAxes2Str(joints2);
        string poseStr2 = pose2Str(pose2);

        if (!extAxes1.empty()) {
            poseStr1 += "," + extAxes1;
        }
        if (!extAxes2.empty()) {
            poseStr2 += "," + extAxes2;
        }

        addLine("CIRC {" + poseStr1 + "},{" + poseStr2 + "}" + C_DIS_);
    }

    // 设置函数
    void RobotPost::setFrame(const robomath::Mat& pose, int frameId, const string& frameName) {
        if (robotAxes_ <= 6) {
            addLine("$BASE = {FRAME: " + pose2Str(pose) + "}");
        }
        else {
            addLine("$BASE = EK (MACHINE_DEF[2].ROOT, MACHINE_DEF[2].MECH_TYPE, { " + pose2Str(pose) + " })");
        }
    }

    void RobotPost::setTool(const robomath::Mat& pose, int toolId, const string& toolName) {
        addLine("$TOOL = {FRAME: " + pose2Str(pose) + "}");
    }

    void RobotPost::setSpeed(double speed_mms) {
        addLine("$VEL.CP = " + to_string(speed_mms / 1000.0));
    }

    void RobotPost::setAcceleration(double accel_mmss) {
        addLine("$ACC.CP = " + to_string(accel_mmss / 1000.0));
    }

    void RobotPost::setSpeedJoints(double speed_degs) {
        addLine("$VEL.ORI1 = " + to_string(speed_degs));
        addLine("$VEL.ORI2 = " + to_string(speed_degs));
    }

    void RobotPost::setAccelerationJoints(double accel_degss) {
        addLine("$ACC.ORI1 = " + to_string(accel_degss));
        addLine("$ACC.ORI2 = " + to_string(accel_degss));
    }

    void RobotPost::setZoneData(double zone_mm) {
        APO_VALUE_ = zone_mm;
        if (zone_mm >= 0) {
            addLine("$APO.CPTP = " + to_string(zone_mm));
            addLine("$APO.CDIS = " + to_string(zone_mm));
            C_DIS_ = " C_DIS";
            C_PTP_ = " C_PTP";
        }
        else {
            C_DIS_ = "";
            C_PTP_ = "";
        }
    }

    // I/O 操作
    void RobotPost::setDO(const string& io_var, const string& io_value) {
        string var_name = io_var;
        string value_str = io_value;

        if (isdigit(io_var[0])) {
            var_name = "$OUT[" + io_var + "]";
        }

        if (isdigit(io_value[0])) {
            int value = stoi(io_value);
            value_str = (value > 0) ? "TRUE" : "FALSE";
        }

        addLine(var_name + "=" + value_str);
    }

    void RobotPost::setAO(const string& io_var, const string& io_value) {
        setDO(io_var, io_value);
    }

    void RobotPost::waitDI(const string& io_var, const string& io_value, int timeout_ms) {
        string var_name = io_var;
        string value_str = io_value;

        if (isdigit(io_var[0])) {
            var_name = "$IN[" + io_var + "]";
        }

        if (isdigit(io_value[0])) {
            int value = stoi(io_value);
            value_str = (value > 0) ? "TRUE" : "FALSE";
        }

        if (timeout_ms < 0) {
            addLine("WAIT FOR (" + var_name + "==" + value_str + ")");
        }
        else {
            addLine("START_TIMER:");
            addLine("$TIMER_STOP[1]=TRUE");
            addLine("$TIMER_FLAG[1]=FALSE");
            addLine("$TIMER[1]=" + to_string(timeout_ms * 0.001));
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
    void RobotPost::RunCode(const string& code, bool is_function_call) {
        if (is_function_call) {
            string code_safe = code;
            replace(code_safe.begin(), code_safe.end(), ' ', '_');
            if (!code_safe.empty() && code_safe.back() != ')') {
                code_safe += "()";
            }
            addLine(code_safe);
        }
        else {
            addLine(code);
        }
    }

    void RobotPost::RunMessage(const string& message, bool is_comment) {
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
            addLine("WAIT SEC " + to_string(time_ms * 0.001));
        }
    }

    // 私有方法
    void RobotPost::addLine(const string& newline) {
        if (nProgs_ > 1 && !config_.includeSubPrograms) {
            return;
        }

        if (nLines_ > config_.maxLinesPerProgram) {
            nLines_ = 0;
            ProgFinish(currentProg_.name);
            ProgStart(currentProg_.name);
        }

        PROG_ += newline + "\n";
        nLines_++;
    }

    void RobotPost::addLog(const string& newline) {
        if (nProgs_ > 1 && !config_.includeSubPrograms) {
            return;
        }
        LOG_ += newline + "\n";
    }

    void RobotPost::saveProgram(const string& folder, const string& progname,
        bool askUser, const string& showResult) {
        string filename = folder + "/" + progname + "." + config_.progExt;

        ofstream file(filename);
        if (!file.is_open()) {
            throw runtime_error("无法打开文件: " + filename);
        }

        file << "&ACCESS RVP\n";
        file << "&REL 1\n";
        file << "&COMMENT Generated by RobotSDK\n";
        file << "&PARAM TEMPLATE = C:\\KRC\\Roboter\\Template\\vorgabe\n";
        file << "&PARAM EDITMASK = *\n";

        for (const auto& prog : progList_) {
            for (const auto& line : prog.lines) {
                file << line << "\n";
            }
        }

        file.close();
        cout << "SAVED: " << filename << endl;
        progFiles_.push_back(filename);
    }

    // 测试函数
    void test_post(std::string path) {
        auto Pose = robomath::PosePP;

        RobotPost robot("Kuka_KRC4", "Generic Kuka");

        robot.ProgStart("Program2");
        robot.RunMessage("Program generated by hejun", true);
        robot.setFrame(Pose(807.766544, -963.699898, 41.478944, 0, 0, 0));
        robot.setTool(Pose(62.5, -108.253175, 100, -60, 90, 0));
        robot.MoveJ(Pose(200, 200, 500, 180, 0, 180), { -46.18419, -6.77518, -20.54925, 71.38674, 49.58727, -302.54752 });
        robot.MoveL(Pose(200, 250, 348.734575, 180, 0, -150), {-41.62707, -8.89064, -30.01809, 60.62329, 49.66749, -258.98418});
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
        robot.MoveL(Pose(250, 300, 278.023897, 180, 0, -150), { -37.52588, -6.32628, -34.59693, 53.52525, 49.24426, -251.44677 });
        robot.MoveL(Pose(250, 200, 278.023897, 180, 0, -150), { -41.85389, -1.95619, -34.89154, 57.43912, 52.34162, -253.73403 });
        robot.MoveL(Pose(250, 150, 191.421356, 180, 0, -150), { -43.82111, 3.29703, -40.29493, 56.02402, 56.61169, -249.23532 });
        robot.ProgFinish("Program2");
        // 遍历PROG_LIST
        for (const auto& prgi : robot.progList_)
        {
            for (const auto& line : prgi.lines) {
                std::cout << line << std::endl;
            }
            std::cout << std::endl << std::endl;
        }

        robot.ProgSave(path, "Program2", false, "");

        if (!robot.LOG_.empty()) {
            MessageBoxA(NULL, ("Program generation LOG:\n\n" + robot.LOG_).c_str(),
                "Program Log", MB_OK | MB_ICONINFORMATION);
        }
    }
}