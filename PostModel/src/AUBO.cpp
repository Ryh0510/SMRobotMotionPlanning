#include <PostModel/AUBO.h>
#include <PostModel/robomath.h>
#include <PostModel/ftpsend.h>

#include <fstream>
#include <sstream>
#include <iomanip>
#include <iostream>
#include <random>
#include <cctype>
#include <functional>
#include <algorithm>
#include <cstring>
#include <windows.h>
#include <shellapi.h>
#pragma execution_character_set("utf-8")

namespace AUBO {
    using namespace std;
    using namespace robomath;
    #ifndef M_PI
    #define M_PI 3.14159265358979323846
    #endif
    // XML 常量定义
    const string RobotPost::XML_HEADER = R"(<?xml version="1.0" encoding="UTF-8"?>
    <ProcedureProgram>
        <Condition ConditionType="Procedure_Program" ConditionAlias="Procedure_Program" ConditionUuid="%s">
            <Attribute/>
    )";

    const string RobotPost::XML_CLOSING = R"(    </Condition>
    </ProcedureProgram>
    )";

    const string RobotPost::XML_MOVEJ = R"(        <Condition ConditionType="Move" ConditionAlias="HOME" ConditionUuid="%s">
                <Attribute>
                    <MoveConditionMode>MoveJoint</MoveConditionMode>
                    <ArrivalAheadThreshold>0</ArrivalAheadThreshold>
                    <ArrivalAheadType>None</ArrivalAheadType>
                    <EnableArrivalAhead>false</EnableArrivalAhead>
                    <EnableOffset>false</EnableOffset>
                    <JointAcc>%s,%s,%s,%s,%s,%s</JointAcc>
                    <JointSpeed>%s,%s,%s,%s,%s,%s</JointSpeed>
                    <OffsetCoordName></OffsetCoordName>
                    <OffsetX></OffsetX>
                    <OffsetY></OffsetY>
                    <OffsetZ></OffsetZ>
                </Attribute>
                <Condition ConditionType="Waypoint" ConditionAlias="Waypoint" ConditionUuid="%s">
                    <Attribute>
                        <WaypointConditionMode>FixedPoint</WaypointConditionMode>
                        <FixedPoint>%s</FixedPoint>
                    </Attribute>
                </Condition>
            </Condition>)";

    const string RobotPost::XML_MOVEL = R"(        <Condition ConditionType="Move" ConditionAlias="Move" ConditionUuid="%s">
                <Attribute>
                    <MoveConditionMode>MoveLine</MoveConditionMode>
                    <ArrivalAheadThreshold>0</ArrivalAheadThreshold>
                    <ArrivalAheadType>None</ArrivalAheadType>
                    <EnableArrivalAhead>false</EnableArrivalAhead>
                    <EnableOffset>false</EnableOffset>
                    <EndLinearAcc>%s</EndLinearAcc>
                    <EndLinearSpeed>%s</EndLinearSpeed>
                    <OffsetCoordName>Base</OffsetCoordName>
                    <OffsetCoordType>BaseCoord</OffsetCoordType>
                    <OffsetRX></OffsetRX>
                    <OffsetRY></OffsetRY>
                    <OffsetRZ></OffsetRZ>
                    <OffsetX></OffsetX>
                    <OffsetY></OffsetY>
                    <OffsetZ></OffsetZ>
                </Attribute>
                <Condition ConditionType="Waypoint" ConditionAlias="Waypoint" ConditionUuid="%s">
                    <Attribute>
                        <WaypointConditionMode>FixedPoint</WaypointConditionMode>
                        <FixedPoint>%s</FixedPoint>
                    </Attribute>
                </Condition>
            </Condition>)";

    const string RobotPost::XML_SETDO = R"(        <Condition ConditionType="Set" ConditionAlias="Set" ConditionUuid="%s">
                <Attribute>
                    <SetConditionMode>IO</SetConditionMode>
                    <IOCategory>User</IOCategory>
                    <IOName>%s</IOName>
                    <IOValue>%s</IOValue>
                    <RobotIOType>RobotBoardUserDO</RobotIOType>
                </Attribute>
            </Condition>)";

    const string RobotPost::XML_WAITDI = R"(        <Condition ConditionType="Wait" ConditionAlias="Wait" ConditionUuid="%s">
                <Attribute>
                    <WaitConditionMode>Exp</WaitConditionMode>
                    <WaitConditionExp>%s == %s</WaitConditionExp>
                </Attribute>
            </Condition>)";

    const string RobotPost::XML_PAUSE = R"(        <Condition ConditionType="Wait" ConditionAlias="Wait" ConditionUuid="%s">
                <Attribute>
                    <WaitConditionMode>Time</WaitConditionMode>
                    <WaitTime>%s</WaitTime>
                </Attribute>
            </Condition>)";

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
        TAB_ = "";
        PROG.clear();
        PROG_XML.clear();
        LOG.clear();
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
        ss << fixed << setprecision(6);
        ss << "{" << xyzrpw[0] << "," << xyzrpw[1] << "," << xyzrpw[2] << "},"
            << "{" << xyzrpw[3] << "," << xyzrpw[4] << "," << xyzrpw[5] << "}";
        return ss.str();
    }

    string RobotPost::joints2Str(const vector<double>& angles) {
        stringstream ss;
        ss << fixed << setprecision(6);
        for (size_t i = 0; i < angles.size(); ++i) {
            if (i > 0) ss << ", ";
            ss << angles[i] * M_PI / 180.0;
        }
        return ss.str();
    }

    string RobotPost::new_uuid() {
        random_device rd;
        mt19937 gen(rd());
        uniform_int_distribution<> dis(0, 15);
        uniform_int_distribution<> dis2(8, 11);

        stringstream ss;
        ss << hex;
        for (int i = 0; i < 32; i++) {
            if (i == 8 || i == 12 || i == 16 || i == 20) {
                ss << "-";
            }
            if (i == 12) {
                ss << "4";
            }
            else if (i == 16) {
                ss << dis2(gen);
            }
            else {
                ss << dis(gen);
            }
        }
        string uuid_str = ss.str();
        uuid_str.erase(remove(uuid_str.begin(), uuid_str.end(), '-'), uuid_str.end());
        return uuid_str;
    }

    // 程序控制方法
    void RobotPost::ProgStart(const string& progname) {
        nProgs_++;
        if (nProgs_ <= 1) {
            TAB_ = "  ";
            config_.LAST_POS = robomath::vec3(0.0, 0.0, 0.0);
        }
        else {
            addLine("  -- Subprogram " + progname);
            addLine("  def " + progname + "():");
            TAB_ = "    ";
        }
    }

    void RobotPost::ProgFinish(const string& progname) {
        TAB_ = "";
        if (nProgs_ <= 1) {
            addLine("--End of main program");
        }
        else {
            addLine("--End of main program");
        }
        addLine("");
    }

    void RobotPost::ProgSave(const string& folder, const string& progname,
        bool askUser, const string& showResult) {
        // 这里实现文件保存逻辑
        string filename = folder + "/" + progname + "." + config_.PROG_EXT;

        ofstream file(filename);
        if (file.is_open()) {
            for (const auto& line : PROG) {
                file << line << endl;
            }
            file.close();
            cout << "SAVED: " << filename << endl;
        }

        string xml_filename = folder + "/" + progname + ".xml";
        ofstream xml_file(xml_filename);
        if (xml_file.is_open()) {
            char buffer[1024];
            snprintf(buffer, sizeof(buffer), XML_HEADER.c_str(), new_uuid().c_str());
            xml_file << buffer;

            for (const auto& block : PROG_XML) {
                xml_file << block << endl;
            }

            xml_file << XML_CLOSING;
            xml_file.close();
            cout << "SAVED: " << xml_filename << endl;
        }
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
        if (pose.rows() == 0) {
            config_.LAST_POS = robomath::vec3(0.0, 0.0, 0.0);
        }

        string randid = new_uuid();
        string randid2 = new_uuid();

        // 正确：先创建string对象，然后使用.c_str()
        string aj_str = to_string(round(config_.ACCEL_RADSS_CURR / 17.296 * 100));
        string sj_str = to_string(round(config_.SPEED_RADS_CURR / 2.618 * 100));
        string joints_str = joints2Str(joints);

        char buffer[4096];
        snprintf(buffer, sizeof(buffer), XML_MOVEJ.c_str(),
            randid.c_str(),
            aj_str.c_str(), aj_str.c_str(), aj_str.c_str(), aj_str.c_str(), aj_str.c_str(), aj_str.c_str(),
            sj_str.c_str(), sj_str.c_str(), sj_str.c_str(), sj_str.c_str(), sj_str.c_str(), sj_str.c_str(),
            randid2.c_str(),
            joints_str.c_str());
        addXML(buffer);

        addLine("--(Logic tree item : Move) Move");
        addLine("init_global_move_profile()");
        double j_vel = config_.SPEED_RADS_CURR;
        double j_acc = config_.ACCEL_RADSS_CURR;

        char vel_buffer[256];
        snprintf(vel_buffer, sizeof(vel_buffer), "set_joint_maxvelc({%f,%f,%f,%f,%f,%f})",
            j_vel, j_vel, j_vel, j_vel * 121 / 101, j_vel * 121 / 101, j_vel * 121 / 101);
        addLine(vel_buffer);

        char acc_buffer[256];
        snprintf(acc_buffer, sizeof(acc_buffer), "set_joint_maxacc({%f,%f,%f,%f,%f,%f})",
            j_acc, j_acc, j_acc, j_acc * 121 / 101, j_acc * 121 / 101, j_acc * 121 / 101);
        addLine(acc_buffer);

        addLine("  --(Logic tree item : Waypoint) Waypoint");
        addLine("  set_current_logic_tree_item(\"" + randid2 + "\")");
        addLine("  move_joint({" + joints2Str(joints) + "}, true)");
        addLine("  set_step_breakpoint()");
    }

    void RobotPost::MoveL(const robomath::Mat& pose, const vector<double>& joints,
        const vector<double>& confRLF) {
        string randid = new_uuid();
        string randid2 = new_uuid();

        string target;
        if (pose.rows() == 0) {
            config_.LAST_POS = robomath::vec3(0.0, 0.0, 0.0);
            target = joints2Str(joints);
        }
        else {
            robomath::Mat pose_wrt_base = config_.REF_FRAME * pose;
            target = pose2Str(pose_wrt_base);
        }

        double al = config_.ACCEL_MSS_CURR * 1000;
        double sl = config_.SPEED_MS_CURR * 1000;

        // 将double转换为string
        string al_str = to_string(al);
        string sl_str = to_string(sl);
        string joints_str = joints2Str(joints);

        char buffer[2048];  // 建议使用更大的缓冲区
        snprintf(buffer, sizeof(buffer), XML_MOVEL.c_str(),
            randid.c_str(),
            al_str.c_str(),      // %s - EndLinearAcc
            sl_str.c_str(),      // %s - EndLinearSpeed
            randid2.c_str(),
            joints_str.c_str());
        addXML(buffer);

        addLine("--(Logic tree item : Move) Move");
        addLine("init_global_move_profile()");
        double l_vel = config_.SPEED_MS_CURR * 20;
        double l_acc = config_.ACCEL_MSS_CURR * 20;

        addLine("set_end_maxvelc(" + to_string(l_vel) + ")");
        addLine("set_end_maxacc(" + to_string(l_acc) + ")");
        addLine("  --(Logic tree item : Waypoint) Waypoint");
        addLine("  set_current_logic_tree_item(\"" + randid2 + "\")");
        addLine("  move_line({" + joints2Str(joints) + "}, true)");
        addLine("  set_step_breakpoint()");
    }

    void RobotPost::MoveC(const robomath::Mat& pose1, const vector<double>& joints1,
        const robomath::Mat& pose2, const vector<double>& joints2,
        const vector<double>& confRLF1, const vector<double>& confRLF2) {
        // 这里实现圆弧运动逻辑
        addLine("-- Circular move not fully implemented");
        MoveL(pose2, joints2, confRLF2);
    }

    // 设置函数
    void RobotPost::setFrame(const robomath::Mat& pose, int frameId, const string& frameName) {
        config_.REF_FRAME = pose;
        //addLine("-- set_reference(" + pose2Str(pose) + ")");
    }

    void RobotPost::setTool(const robomath::Mat& pose, int toolId, const string& toolName) {
        addLine("set_tool_kinematics_param(" + pose2Str(pose) + ")");
    }

    void RobotPost::setSpeed(double speed_mms) {
        if (speed_mms >= 2000) {
            speed_mms = 2000;
        }
        config_.SPEED_MS_CURR = speed_mms / 1000.0;
    }

    void RobotPost::setAcceleration(double accel_mmss) {
        if (accel_mmss >= 2000) {
            accel_mmss = 2000;
        }
        config_.ACCEL_MSS_CURR = accel_mmss / 1000.0;
    }

    void RobotPost::setSpeedJoints(double speed_degs) {
        double max_speed_degs = 2.59 * 180 / M_PI;
        if (speed_degs >= max_speed_degs) {
            speed_degs = max_speed_degs;
        }
        config_.SPEED_RADS_CURR = speed_degs * M_PI / 180;
    }

    void RobotPost::setAccelerationJoints(double accel_degss) {
        if (accel_degss >= 991) {
            accel_degss = 991;
        }
        config_.ACCEL_RADSS_CURR = accel_degss * M_PI / 180;
    }

    void RobotPost::setZoneData(double zone_mm) {
        if (zone_mm < 0) {
            zone_mm = 0;
        }
        config_.BLEND_RADIUS_M = zone_mm / 1000.0;
    }

    // I/O 操作
    void RobotPost::setDO(const string& io_var, const string& io_value) {
        string randid = new_uuid();
        addLine("--(Logic tree item : Set) Set");
        addLine("set_current_logic_tree_item(\"" + randid + "\")");

        string value_str = io_value;
        if (isdigit(io_value[0])) {
            if (stoi(io_value) > 0) {
                value_str = "1";
            }
            else {
                value_str = "0";
            }
        }
        else {
            if (io_value == "true" || io_value == "TRUE") {
                value_str = "1";
            }
            else {
                value_str = "0";
            }
        }

        string var_name = io_var;
        string newline;
        if (isdigit(io_var[0])) {
            if (stoi(io_var) < 10) {
                var_name = "U_DO_0" + io_var;
                newline = "set_robot_io_status(RobotIOType.RobotBoardUserDO, \"U_DO_0" + io_var + "\", " + value_str + ")";
            }
            else {
                var_name = "U_DO_" + io_var;
                newline = "set_robot_io_status(RobotIOType.RobotBoardUserDO, \"U_DO_" + io_var + "\", " + value_str + ")";
            }
        }
        else {
            newline = "set_robot_io_status(RobotIOType.RobotBoardUserDO, \"" + io_var + "\", " + value_str + ")";
        }

        char buffer[1024];
        snprintf(buffer, sizeof(buffer), XML_SETDO.c_str(),
            randid.c_str(), var_name.c_str(), value_str.c_str());
        addXML(buffer);

        addLine(newline);
        addLine("set_step_breakpoint()");
    }

    void RobotPost::setAO(const string& io_var, const string& io_value) {
        setDO(io_var, io_value);
    }

    void RobotPost::waitDI(const string& io_var, const string& io_value, int timeout_ms) {
        string randid = new_uuid();
        addLine("--(Logic tree item : Wait) Wait");
        addLine("set_current_logic_tree_item(\"" + randid + "\")");

        string value_str = io_value;
        if (isdigit(io_value[0])) {
            if (stoi(io_value) > 0) {
                value_str = "1";
            }
            else {
                value_str = "0";
            }
        }
        else {
            if (io_value == "true" || io_value == "TRUE") {
                value_str = "1";
            }
            else {
                value_str = "0";
            }
        }

        string var_name = io_var;
        string io_call;
        if (isdigit(io_var[0])) {
            if (stoi(io_var) < 10) {
                var_name = "(U,DI)U_DI_0" + io_var;
                io_call = "get_robot_io_status(RobotIOType.RobotBoardUserDI, \"U_DI_0" + io_var + "\")";
            }
            else {
                var_name = "(U,DI)U_DI_" + io_var;
                io_call = "get_robot_io_status(RobotIOType.RobotBoardUserDI, \"U_DI_" + io_var + "\")";
            }
        }
        else {
            var_name = "(U,DI)" + io_var;
            io_call = "get_robot_io_status(RobotIOType.RobotBoardUserDI, \"" + io_var + "\")";
        }

        char buffer[1024];
        snprintf(buffer, sizeof(buffer), XML_WAITDI.c_str(),
            randid.c_str(), var_name.c_str(), value_str.c_str());
        addXML(buffer);

        if (timeout_ms < 0) {
            addLine("while (not (" + io_call + " == " + value_str + ")) do");
            addLine("  sleep(0.01)");
            addLine("end");
        }
    }

    // 代码生成
    void RobotPost::RunCode(const string& code, bool is_function_call) {
        if (is_function_call) {
            string code_safe = FilterName(code);
            if (code_safe.find('(') == string::npos) {
                code_safe += "()";
            }
            addLine(code_safe);
        }
        else {
            addLine("-- " + code);
        }
    }

    void RobotPost::RunMessage(const string& message, bool is_comment) {
        if (is_comment) {
            addLine("-- " + message);
        }
        else {
            addLine("popup(\"" + message + "\",\"Message\",False,False,blocking=True)");
        }
    }

    void RobotPost::Pause(int time_ms) {
        string randid = new_uuid();

        if (time_ms <= 0) {
            addLine("halt() # reimplement this function to force stop");
        }
        else {
            char buffer[1024];
            snprintf(buffer, sizeof(buffer), XML_PAUSE.c_str(),
                randid.c_str(), to_string(time_ms * 0.001).c_str());
            addXML(buffer);

            addLine("--(Logic tree item : Wait) Wait");
            addLine("set_current_logic_tree_item(\"" + randid + "\")");
            addLine("sleep(" + to_string(time_ms * 0.001) + ")");
            addLine("set_step_breakpoint()");
        }
    }

    // 私有方法
    void RobotPost::addLine(const string& newline) {
        PROG.push_back(TAB_ + newline);
    }

    void RobotPost::addXML(const string& newblock) {
        PROG_XML.push_back(newblock);
    }

    void RobotPost::addLog(const string& newline) {
        LOG += newline + "\n";
    }

    void RobotPost::saveProgram(const string& folder, const string& progname,
        bool askUser, const string& showResult) {
        // 实现文件保存逻辑
        ProgSave(folder, progname, askUser, showResult);
    }

    // 测试函数
    void test_post(string path) {
        auto Pose = robomath::PosePP;

        RobotPost robot("AUBO", "Generic AUBO robot", 6);
        robot.ProgStart("Program");
        robot.RunMessage("Program generated by hejun", true);
        robot.setFrame(Pose(807.766544, -963.699898, 41.478944, 0, 0, 0));
        robot.setTool(Pose(62.5, -108.253175, 100, -60, 90, 0));
        robot.setSpeed(100);
        robot.setAcceleration(3000);
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
        robot.MoveL(Pose(250, 300, 278.023897, 180, 0, -150), { -37.52588, -6.32628, -34.59693, 53.52525, 49.24426, -251.44677 });
        robot.MoveL(Pose(250, 200, 278.023897, 180, 0, -150), { -41.85389, -1.95619, -34.89154, 57.43912, 52.34162, -253.73403 });
        robot.MoveL(Pose(250, 150, 191.421356, 180, 0, -150), { -43.82111, 3.29703, -40.29493, 56.02402, 56.61169, -249.23532 });
        robot.ProgFinish("Program");

        for (const auto& line : robot.PROG) {
            std::cout << line << std::endl;
        }

        if (!robot.LOG.empty()) {
            cout << "Program generation LOG:\n" << robot.LOG << endl;
        }

        robot.ProgSave(path, "Program", false, "");
    }
}