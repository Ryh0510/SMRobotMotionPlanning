#include <PostModel/KUKA_IIWA.h>
#include <PostModel/robomath.h>
#include <PostModel/ftpsend.h>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <iostream>
#include <windows.h>
#include <shellapi.h>
#pragma execution_character_set("utf-8")

namespace KUKA_IIWA{
    using namespace std;

    using namespace robomath;
    #ifndef M_PI
    #define M_PI 3.14159265358979323846
    #endif
    // 常量定义

    const vector<string> RESERVED_NAMES = {
        "tool0", "work0", "work1", "clock1", "reg1", "reg2", "reg3", "reg4", "reg5"
    };

    const string RobotPost::HEADER = R"(package application;

    import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
    import static com.kuka.roboticsAPI.motionModel.BasicMotions.*;

    import com.kuka.roboticsAPI.deviceModel.LBR;
    import com.kuka.roboticsAPI.deviceModel.LBRE1Redundancy;
    import com.kuka.roboticsAPI.geometricModel.Frame;
    import com.kuka.roboticsAPI.geometricModel.Tool;
    import com.kuka.roboticsAPI.geometricModel.math.Transformation;

    import com.kuka.roboticsAPI.motionModel.LIN;
    import com.kuka.roboticsAPI.motionModel.PTP;
    import com.kuka.roboticsAPI.motionModel.Spline;
    import com.kuka.roboticsAPI.uiModel.ApplicationDialogType;

    public class %s extends RoboticsAPIApplication {
    \tprivate LBR robot;
    \tprivate Tool TOOL;
    \tSpline move_curve;
    \t// private Frame FRAME;
    \t
    \tpublic void initialize() {		
    \t\trobot = getContext().getDeviceFromType(LBR.class);
    \t}
    \tpublic void run() {
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
        MAIN_DONE_ = false;
        TAB_ = "\t\t";
        MOVE_OBJECT_ = "robot";
        REF_FRAME_ = robomath::eye(4);
        TARGET_PTP_id_ = 0;
        TARGET_LIN_id_ = 0;
        TARGET_CIRC_id_ = 0;
        SPEED_MMS_ = 300.0;
        SPEED_RADS_ = 0.250;
        ACCEL_MMSS_ = 500.0;
        SPLINE_COUNT_ = 0;
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

    string RobotPost::pose2Str(const robomath::Mat& pose, bool rot_in_deg) {
        double torad = M_PI / 180.0;
        if (rot_in_deg) {
            torad = 1.0;
        }

        vector<double> xyzrpw = robomath::pose_2_xyzrpw(pose);
        stringstream ss;
        ss << fixed << setprecision(3);
        ss << xyzrpw[0] << "," << xyzrpw[1] << "," << xyzrpw[2] << ",";
        ss << fixed << setprecision(5);
        ss << xyzrpw[5] * torad << "," << xyzrpw[4] * torad << "," << xyzrpw[3] * torad;
        return ss.str();
    }

    string RobotPost::angles2Str(const vector<double>& angles) {
        stringstream ss;
        for (size_t i = 0; i < angles.size(); ++i) {
            if (i > 0) ss << ",";
            ss << fixed << setprecision(5) << angles[i] * M_PI / 180.0;
        }
        return ss.str();
    }

    string RobotPost::pose2StrExt(const robomath::Mat& pose, const vector<double>& joints) {
        size_t njoints = joints.size();
        if (njoints <= 7) {
            return pose2Str(pose);
        }

        string result = pose2Str(pose);
        for (size_t i = 0; i < njoints - 7; ++i) {
            result += "," + to_string(joints[i + 7]);
        }
        return result;
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

        if (MAIN_DONE_) {
            PROG_ = "public void " + safeName + "() {\n";
            TAB_ = "\t\t";
        }
        else {
            MAIN_DONE_ = true;
            char buffer[1024];
            snprintf(buffer, sizeof(buffer), HEADER.c_str(), safeName.c_str());
            PROG_ = buffer;
        }
    }

    void RobotPost::ProgFinish(const string& progname) {
        splineFlush();
        TAB_ = "\t";
        addLine("}");

        if (nProgs_ > 1 && !config_.includeSubPrograms) {
            return;
        }

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
        cout << "Sending programs to robot " << ip_com_ << endl;
        bool success = UploadFTP(progFiles_, ip_com_, remote_path, ftp_user, ftp_pass);

        if (success) {
            cout << "Upload successful!" << endl;
        }
        else {
            cout << "Upload failed!" << endl;
        }
    }

    // 运动指令
    void RobotPost::MoveJ(const robomath::Mat& pose, const vector<double>& joints,
        const vector<double>& confRLF) {
        splineFlush();
        TARGET_PTP_id_++;
        string targetname = "MovePTP_" + to_string(TARGET_PTP_id_);

        addLine("");
        addLine("PTP " + targetname + " = ptp(" + angles2Str(joints) + ");");
        addLine(MOVE_OBJECT_ + ".move(" + targetname + ");");
    }

    void RobotPost::MoveL(const robomath::Mat& pose, const vector<double>& joints,
        const vector<double>& confRLF) {
        TARGET_LIN_id_++;
        string targetname = "TargetLIN_" + to_string(TARGET_LIN_id_);
        string framename = "FrameLIN_" + to_string(TARGET_LIN_id_);

        if (pose.rows() == 4 && pose.cols() == 4) {
            robomath::Mat poseabs = REF_FRAME_ * pose;
            addLine("Frame " + framename + " = new Frame(" + pose2Str(poseabs) + ");");

            if (config_.MOVE_Spline) {
                splineAddBuffer(framename);
            }
            else {
                addLine(MOVE_OBJECT_ + ".move(lin(" + framename + ").setCartVelocity(" +
                    to_string(SPEED_MMS_) + ").setBlendingCart(" + to_string(config_.BLENDING_MM) + "));");
            }
        }
        else {
            splineFlush();
            addLine("LIN " + targetname + " = lin(" + angles2Str(joints) + ");");
            addLine(MOVE_OBJECT_ + ".move(" + targetname + ");");
        }
    }

    void RobotPost::MoveC(const robomath::Mat& pose1, const vector<double>& joints1,
        const robomath::Mat& pose2, const vector<double>& joints2,
        const vector<double>& confRLF1, const vector<double>& confRLF2) {
        splineFlush();
        robomath::Mat pose1abs = REF_FRAME_ * pose1;
        robomath::Mat pose2abs = REF_FRAME_ * pose2;

        TARGET_CIRC_id_++;
        string framename1 = "FrameCIRC_" + to_string(TARGET_CIRC_id_);
        TARGET_CIRC_id_++;
        string framename2 = "FrameCIRC_" + to_string(TARGET_CIRC_id_);

        addLine("");
        addLine("getLogger().info(\"Move circular to " + framename1 + " and " + framename2 + "\");");
        addLine("Frame " + framename1 + " = new Frame(" + pose2Str(pose1abs) + ");");
        addLine("Frame " + framename2 + " = new Frame(" + pose2Str(pose2abs) + ");");
        addLine(MOVE_OBJECT_ + ".move(circ(" + framename1 + ", " + framename2 +
            ").setCartVelocity(" + to_string(SPEED_MMS_) + ").setBlendingCart(" +
            to_string(config_.BLENDING_MM) + "));");
    }

    // 设置函数
    void RobotPost::setFrame(const robomath::Mat& pose, int frameId, const string& frameName) {
        string name = frameName.empty() ? "Reference" : frameName;
        REF_FRAME_ = pose;
        splineFlush();
        addLine("// Using Reference Frame: " + name);
        addLine("// robot.addDefaultMotionFrame(\"" + name + "\", Transformation.ofDeg(" +
            pose2Str(pose, true) + ");");
    }

    void RobotPost::setTool(const robomath::Mat& pose, int toolId, const string& toolName) {
        string name = toolName.empty() ? "Tool" : toolName;
        splineFlush();
        addLine("// Using TCP: " + name);
        addLine("TOOL = new Tool(\"" + name + "\");");
        addLine("TOOL.attachTo(robot.getFlange(), Transformation.ofDeg(" +
            pose2Str(pose, true) + "));");
        MOVE_OBJECT_ = "TOOL";
    }

    void RobotPost::setSpeed(double speed_mms) {
        splineFlush();
        SPEED_MMS_ = speed_mms;
        addLine("// Linear speed set to " + to_string(SPEED_MMS_) + " mm/s");
    }

    void RobotPost::setAcceleration(double accel_mmss) {
        ACCEL_MMSS_ = accel_mmss;
        addLine("// Warning: set linear acceleration to " + to_string(ACCEL_MMSS_) + " mm/ss has no effect");
    }

    void RobotPost::setSpeedJoints(double speed_degs) {
        SPEED_RADS_ = speed_degs * M_PI / 180.0;
        addLine("// Joint speed set to " + to_string(SPEED_RADS_) + " rad/s");
    }

    void RobotPost::setAccelerationJoints(double accel_degss) {
        addLine("// Warning: set angular acceleration to " + to_string(accel_degss) + " deg/s has no effect");
    }

    void RobotPost::setZoneData(double zone_mm) {
        splineFlush();
        if (zone_mm < 0) {
            zone_mm = 0;
        }
        config_.BLENDING_MM = zone_mm;
        addLine("// smoothing/blending set to " + to_string(config_.BLENDING_MM) + " mm");
    }

    // I/O 操作
    void RobotPost::setDO(const string& io_var, const string& io_value) {
        string var_name = io_var;
        string value_str = io_value;

        if (isdigit(io_var[0])) {
            var_name = "OUT[" + io_var + "]";
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
            var_name = "IN[" + io_var + "]";
        }

        if (isdigit(io_value[0])) {
            int value = stoi(io_value);
            value_str = (value > 0) ? "TRUE" : "FALSE";
        }

        if (timeout_ms < 0) {
            addLine("WAIT FOR " + var_name + "==" + value_str);
        }
        else {
            addLine("WAIT FOR " + var_name + "==" + value_str + " TIMEOUT=" +
                to_string(timeout_ms * 0.001));
        }
    }

    // 代码生成
    void RobotPost::RunCode(const string& code, bool is_function_call) {
        splineFlush();
        if (is_function_call) {
            string code_safe = FilterName(code);
            if (code_safe.find('(') == string::npos) {
                code_safe += "()";
            }
            addLine(code_safe + ";");
        }
        else {
            addLine(code);
        }
    }

    void RobotPost::RunMessage(const string& message, bool is_comment) {
        splineFlush();
        addLine("");
        if (is_comment) {
            addLine("// " + message);
        }
        else {
            addLine("getLogger().info(\"" + message + "\");");
        }
    }

    void RobotPost::Pause(int time_ms) {
        splineFlush();
        if (time_ms < 0) {
            addLine("if (getApplicationUI().displayModalDialog(ApplicationDialogType.QUESTION, \"Program paused. Select Cancel to stop the program.\", \"OK\", \"Cancel\")== 1)");
            addLine("{");
            addLine("\treturn;");
            addLine("}");
        }
        else if (time_ms > 0) {
            addLine("try {");
            addLine("\tThread.sleep(" + to_string(time_ms) + ");");
            addLine("} catch (InterruptedException e){");
            addLine("\tgetLogger().info(\"Pause Skipped\");");
            addLine("}");
        }
    }

    // 私有方法
    void RobotPost::addLine(const string& newline) {
        if (nProgs_ > 1 && !config_.includeSubPrograms) {
            return;
        }

        PROG_ += TAB_ + newline + "\n";
        nLines_++;
    }

    void RobotPost::addLog(const string& newline) {
        if (nProgs_ > 1 && !config_.includeSubPrograms) {
            return;
        }
        LOG_ += newline + "\n";
    }

    void RobotPost::splineAddBuffer(const string& frame) {
        SPLINE_ += "spl(" + frame + "),";
        SPLINE_COUNT_++;
        SPLINE_LAST_FRAME_ = frame;
    }

    void RobotPost::splineFlush() {
        if (SPLINE_COUNT_ == 1) {
            addLine(MOVE_OBJECT_ + ".move(lin(" + SPLINE_LAST_FRAME_ + "));");
        }
        else if (SPLINE_COUNT_ > 1) {
            PROG_ += TAB_ + "move_curve = new Spline(" + SPLINE_.substr(0, SPLINE_.length() - 1) + ");\n";
            PROG_ += TAB_ + MOVE_OBJECT_ + ".move(move_curve);\n";
        }

        SPLINE_.clear();
        SPLINE_COUNT_ = 0;
        SPLINE_LAST_FRAME_.clear();
    }

    void RobotPost::saveProgram(const string& folder, const string& progname,
        bool askUser, const string& showResult) {
        string filename = folder + "/" + progname + "." + config_.PROG_EXT;

        ofstream file(filename);
        if (!file.is_open()) {
            throw runtime_error("无法打开文件: " + filename);
        }

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

        RobotPost robot("Kuka_IIWA", "Generic Kuka IIWA", 7);

        robot.ProgStart("ProgramIIWA");
        robot.RunMessage("Program generated by hejun", true);
        robot.setFrame(Pose(807.766544, -963.699898, 41.478944, 0, 0, 0 ));
        robot.setDO("123", "1");
        robot.waitDI("125", "1", 1000);
        robot.waitDI("126", "1", -1);
        robot.setTool(Pose(62.5, -108.253175, 100, -60, 90, 0 ));
        robot.MoveJ(Pose(200, 200, 500, 180, 0, 180 ), { -46.18419, -6.77518, -20.54925, 71.38674, 49.58727, -302.54752 });
        robot.MoveL(Pose(200, 250, 348.734575, 180, 0, -150 ), { -41.62707, -8.89064, -30.01809, 60.62329, 49.66749, -258.98418 });
        robot.MoveL(Pose(200, 200, 262.132034, 180, 0, -150 ), { -43.73892, -3.91728, -35.77935, 58.57566, 54.11615, -253.81122 });
        robot.RunMessage("Setting air valve 1 on");
        robot.RunCode("TCP_On", true);
        robot.Pause(1000);
        robot.MoveL(Pose(200, 250, 348.734575, 180, 0, -150 ), { -41.62707, -8.89064, -30.01809, 60.62329, 49.66749, -258.98418 });
        robot.MoveL(Pose(250, 300, 278.023897, 180, 0, -150 ), { -37.52588, -6.32628, -34.59693, 53.52525, 49.24426, -251.44677 });
        robot.MoveL(Pose(250, 250, 191.421356, 180, 0, -150 ), { -39.75778, -1.04537, -40.37883, 52.09118, 54.15317, -246.94403 });
        robot.RunMessage("Setting air valve off");
        robot.RunCode("TCP_Off", true);
        robot.Pause(1000);
        robot.MoveL(Pose(250, 300, 278.023897, 180, 0, -150 ), { -37.52588, -6.32628, -34.59693, 53.52525, 49.24426, -251.44677 });
        robot.MoveL(Pose(250, 200, 278.023897, 180, 0, -150), { -41.85389, -1.95619, -34.89154, 57.43912, 52.34162, -253.73403 });
        robot.MoveL(Pose(250, 150, 191.421356, 180, 0, -150), { -43.82111, 3.29703, -40.29493, 56.02402, 56.61169, -249.23532 });
        robot.ProgFinish("ProgramIIWA");

        for (const auto& prgi : robot.progList_) {
            for (const auto& line : prgi.lines) {
                std::cout << line << std::endl;
            }
            std::cout << std::endl << std::endl;
        }

        robot.ProgSave(path, "ProgramIIWA", false, "");

        if (!robot.LOG_.empty()) {
            MessageBoxA(NULL, ("Program generation LOG:\n\n" + robot.LOG_).c_str(),
                "Program Log", MB_OK | MB_ICONINFORMATION);
        }
    }
}