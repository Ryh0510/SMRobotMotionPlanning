#include <PostModel/Staubli_VAL3.h>
#include <PostModel/robomath.h>
#include <PostModel/ftpsend.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <cstdio>
#include <cstdlib>
#include <algorithm>
#include <cmath>
#include <vector>
#include <map>
#include <cctype>
#include <windows.h>
#include <direct.h>
#include <sys/stat.h>
#include <functional>
#include <wininet.h>
#pragma execution_character_set("utf-8")

namespace Staubli_VAL3{
    using namespace robomath;
    using namespace std;

    // XML模板定义
    const string RobotPost::PROGRAM_PJX = "<?xml version=\"1.0\" encoding=\"utf-8\"?>\n<Project xmlns=\"http://www.staubli.com/robotics/VAL3/Project/3\">\n<Parameters version=\"s7.5.3\" stackSize=\"5000\" millimeterUnit=\"true\" />\n<Programs>\n<Program file=\"MoveJoint.pgx\" />\n<Program file=\"MoveLine.pgx\" />\n%s<Program file=\"start.pgx\" />\n<Program file=\"stop.pgx\" />\n<Program file=\"main.pgx\" />\n</Programs>\n<Database>\n<Data file=\"%s.dtx\" />\n</Database>\n<Libraries />\n</Project>\n";

    const string RobotPost::PROGRAM_PJX_CS8V7 = "<?xml version=\"1.0\" encoding=\"utf-8\"?>\n<Project xmlns=\"http://www.staubli.com/robotics/VAL3/Project/3\">\n  <Parameters version=\"s7.11.2\" stackSize=\"5000\" millimeterUnit=\"true\" />\n<Programs>\n<Program file=\"MoveJoint.pgx\" />\n<Program file=\"MoveLine.pgx\" />\n%s<Program file=\"start.pgx\" />\n<Program file=\"stop.pgx\" />\n<Program file=\"main.pgx\" />\n</Programs>\n<Database>\n<Data file=\"%s.dtx\" />\n</Database>\n<Libraries />\n</Project>\n";

    const string RobotPost::PROGRAM_PJX_CS8V6 = "<?xml version=\"1.0\" encoding=\"utf-8\"?>\n<project xmlns=\"ProjectNameSpace\">\n  <parameters version=\"s6.9.1\" stackSize=\"5000\" millimeterUnit=\"true\" />\n  <programSection>\n    <program file=\"start.pgx\" />\n    <program file=\"stop.pgx\" />\n  </programSection>\n  <dataSection>\n    <data file=\"main.dtx\" />\n  </dataSection>\n  <aliasSection>\n    <alias name=\"io\" interface=\"io\" autoload=\"true\" />\n    <alias name=\"vg\" interface=\"Disk://vg\" autoload=\"true\" />\n    <alias name=\"rp\" interface=\"Disk://appli\" autoload=\"true\" />\n    <alias name=\"us\" interface=\"Disk://usinage\" autoload=\"true\" />\n  </aliasSection>\n</project>\n";

    const string RobotPost::MODULES_ADD = "";

    const string RobotPost::PROGRAM_DTX = "<?xml version=\"1.0\" encoding=\"utf-8\"?>\n<Database xmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\" xmlns=\"http://www.staubli.com/robotics/VAL3/Data/2\">\n  <Datas>\n    <Data name=\"mNomSpeed\" access=\"private\" xsi:type=\"array\" type=\"mdesc\" size=\"1\" />\n    <Data name=\"fFrame1\" access=\"private\" xsi:type=\"array\" type=\"frame\" size=\"1\">\n      <Value key=\"0\" %s fatherId=\"world[0]\" />\n    </Data>\n    <Data name=\"Input\" access=\"private\" xsi:type=\"array\" type=\"dio\" size=\"16\">\n      <Value key=\"0\" link=\"BasicIO-1\\%%I0\" />\n      <Value key=\"1\" link=\"BasicIO-1\\%%I1\" />\n      <Value key=\"2\" link=\"BasicIO-1\\%%I2\" />\n      <Value key=\"3\" link=\"BasicIO-1\\%%I3\" />\n      <Value key=\"4\" link=\"BasicIO-1\\%%I4\" />\n      <Value key=\"5\" link=\"BasicIO-1\\%%I5\" />\n      <Value key=\"6\" link=\"BasicIO-1\\%%I6\" />\n      <Value key=\"7\" link=\"BasicIO-1\\%%I7\" />\n      <Value key=\"8\" link=\"BasicIO-1\\%%I8\" />\n      <Value key=\"9\" link=\"BasicIO-1\\%%I9\" />\n      <Value key=\"10\" link=\"BasicIO-1\\%%I10\" />\n      <Value key=\"11\" link=\"BasicIO-1\\%%I11\" />\n      <Value key=\"12\" link=\"BasicIO-1\\%%I12\" />\n      <Value key=\"13\" link=\"BasicIO-1\\%%I13\" />\n      <Value key=\"14\" link=\"BasicIO-1\\%%I14\" />\n      <Value key=\"15\" link=\"BasicIO-1\\%%I15\" />\n    </Data>\n    <Data name=\"Output\" access=\"private\" xsi:type=\"array\" type=\"dio\" size=\"16\">\n      <Value key=\"0\" link=\"BasicIO-1\\%%Q0\" />\n      <Value key=\"1\" link=\"BasicIO-1\\%%Q1\" />\n      <Value key=\"2\" link=\"BasicIO-1\\%%Q2\" />\n      <Value key=\"3\" link=\"BasicIO-1\\%%Q3\" />\n      <Value key=\"4\" link=\"BasicIO-1\\%%Q4\" />\n      <Value key=\"5\" link=\"BasicIO-1\\%%Q5\" />\n      <Value key=\"6\" link=\"BasicIO-1\\%%Q6\" />\n      <Value key=\"7\" link=\"BasicIO-1\\%%Q7\" />\n      <Value key=\"8\" link=\"BasicIO-1\\%%Q8\" />\n      <Value key=\"9\" link=\"BasicIO-1\\%%Q9\" />\n      <Value key=\"10\" link=\"BasicIO-1\\%%Q10\" />\n      <Value key=\"11\" link=\"BasicIO-1\\%%Q11\" />\n      <Value key=\"12\" link=\"BasicIO-1\\%%Q12\" />\n      <Value key=\"13\" link=\"BasicIO-1\\%%Q13\" />\n      <Value key=\"14\" link=\"BasicIO-1\\%%Q14\" />\n      <Value key=\"15\" link=\"BasicIO-1\\%%Q15\" />\n    </Data>\n    <Data name=\"tCurrentTool\" access=\"private\" xsi:type=\"array\" type=\"tool\" size=\"1\">\n      <Value key=\"0\" %s fatherId=\"flange[0]\" ioLink=\"valve1\" />\n    </Data>\n    <Data name=\"mCurrentSpeed\" access=\"private\" xsi:type=\"array\" type=\"mdesc\" size=\"1\" />\n    <Data name=\"pPointRx\" access=\"private\" xsi:type=\"array\" type=\"pointRx\" size=\"1\">\n      <Value key=\"0\" fatherId=\"fFrame1[0]\" />\n    </Data>\n    <Data name=\"pPointRx2\" access=\"private\" xsi:type=\"array\" type=\"pointRx\" size=\"1\">\n      <Value key=\"0\" fatherId=\"fFrame1[0]\" />\n    </Data>\n  </Datas>\n</Database>\n";

    const string RobotPost::MOVELINE_PGX = "<?xml version=\"1.0\" encoding=\"utf-8\"?>\n<Programs xmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\" xmlns=\"http://www.staubli.com/robotics/VAL3/Program/2\">\n  <Program name=\"MoveLine\">\n    <Parameters xmlns=\"http://www.staubli.com/robotics/VAL3/Param/1\">\n      <Parameter name=\"x_nX\" type=\"num\" xsi:type=\"element\" />\n      <Parameter name=\"x_nY\" type=\"num\" xsi:type=\"element\" />\n      <Parameter name=\"x_nZ\" type=\"num\" xsi:type=\"element\" />\n      <Parameter name=\"x_nRx\" type=\"num\" xsi:type=\"element\" />\n      <Parameter name=\"x_nRy\" type=\"num\" xsi:type=\"element\" />\n      <Parameter name=\"x_nRz\" type=\"num\" xsi:type=\"element\" />\n    </Parameters>\n    <Code><![CDATA[begin\n  pPointRx.trsf.x = x_nX\n  pPointRx.trsf.y = x_nY\n  pPointRx.trsf.z = x_nZ\n  pPointRx.trsf.rx = x_nRx\n  pPointRx.trsf.ry = x_nRy\n  pPointRx.trsf.rz = x_nRz\n  movel(pPointRx,tCurrentTool,mCurrentSpeed)\nend]]></Code>\n  </Program>\n</Programs>\n";

    const string RobotPost::MOVECIRC_PGX = "<?xml version=\"1.0\" encoding=\"utf-8\"?>\n<Programs xmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\" xmlns=\"http://www.staubli.com/robotics/VAL3/Program/2\">\n  <Program name=\"MoveCirc\">\n    <Parameters xmlns=\"http://www.staubli.com/robotics/VAL3/Param/1\">\n      <Parameter name=\"x_nX\" type=\"num\" xsi:type=\"element\" />\n      <Parameter name=\"x_nY\" type=\"num\" xsi:type=\"element\" />\n      <Parameter name=\"x_nZ\" type=\"num\" xsi:type=\"element\" />\n      <Parameter name=\"x_nRx\" type=\"num\" xsi:type=\"element\" />\n      <Parameter name=\"x_nRy\" type=\"num\" xsi:type=\"element\" />\n      <Parameter name=\"x_nRz\" type=\"num\" xsi:type=\"element\" />\n      <Parameter name=\"x_nX2\" type=\"num\" xsi:type=\"element\" />\n      <Parameter name=\"x_nY2\" type=\"num\" xsi:type=\"element\" />\n      <Parameter name=\"x_nZ2\" type=\"num\" xsi:type=\"element\" />\n      <Parameter name=\"x_nRx2\" type=\"num\" xsi:type=\"element\" />\n      <Parameter name=\"x_nRy2\" type=\"num\" xsi:type=\"element\" />\n      <Parameter name=\"x_nRz2\" type=\"num\" xsi:type=\"element\" />\n    </Parameters>\n    <Code><![CDATA[begin\n  pPointRx.trsf.x = x_nX\n  pPointRx.trsf.y = x_nY\n  pPointRx.trsf.z = x_nZ\n  pPointRx.trsf.rx = x_nRx\n  pPointRx.trsf.ry = x_nRy\n  pPointRx.trsf.rz = x_nRz\n  pPointRx2.trsf.x = x_nX2\n  pPointRx2.trsf.y = x_nY2\n  pPointRx2.trsf.z = x_nZ2\n  pPointRx2.trsf.rx = x_nRx2\n  pPointRx2.trsf.ry = x_nRy2\n  pPointRx2.trsf.rz = x_nRz2\n  movec(pPointRx,pPointRx2,tCurrentTool,mCurrentSpeed)\nend]]></Code>\n  </Program>\n</Programs>\n";

    const string RobotPost::MOVEJOINT_PGX = "<?xml version=\"1.0\" encoding=\"utf-8\"?>\n<Programs xmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\" xmlns=\"http://www.staubli.com/robotics/VAL3/Program/2\">\n  <Program name=\"MoveJoint\">\n    <Parameters xmlns=\"http://www.staubli.com/robotics/VAL3/Param/1\">\n      <Parameter name=\"x_nJ1\" type=\"num\" xsi:type=\"element\" />\n      <Parameter name=\"x_nJ2\" type=\"num\" xsi:type=\"element\" />\n      <Parameter name=\"x_nJ3\" type=\"num\" xsi:type=\"element\" />\n      <Parameter name=\"x_nJ4\" type=\"num\" xsi:type=\"element\" />\n      <Parameter name=\"x_nJ5\" type=\"num\" xsi:type=\"element\" />\n      <Parameter name=\"x_nJ6\" type=\"num\" xsi:type=\"element\" />\n    </Parameters>\n    <Locals>\n      <Local name=\"l_jJoint\" type=\"joint\" xsi:type=\"array\" size=\"1\" />\n    </Locals>\n    <Code><![CDATA[begin\n  l_jJoint.j1 = x_nJ1\n  l_jJoint.j2 = x_nJ2\n  l_jJoint.j3 = x_nJ3\n  l_jJoint.j4 = x_nJ4\n  l_jJoint.j5 = x_nJ5\n  l_jJoint.j6 = x_nJ6\n  movej(l_jJoint,tCurrentTool ,mCurrentSpeed)\nend]]></Code>\n  </Program>\n</Programs>\n";

    const string RobotPost::MAIN_PGX = "<?xml version=\"1.0\" encoding=\"utf-8\"?>\n<Programs xmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\" xmlns=\"http://www.staubli.com/robotics/VAL3/Program/2\">\n  <Program name=\"main\">\n    <Code><![CDATA[begin\n  close(tCurrentTool)\n  pPointRx.config.shoulder = lefty\n  pPointRx.config.elbow = epositive\n  pPointRx.config.wrist = wpositive\n  pPointRx2.config.shoulder = lefty\n  pPointRx2.config.elbow = epositive\n  pPointRx2.config.wrist = wpositive\n  mCurrentSpeed.tvel = 100.00\n  mCurrentSpeed.blend = joint\n  mCurrentSpeed.reach = 0.010\n  mCurrentSpeed.leave = 0.010\n%s\nend]]></Code>\n  </Program>\n</Programs>";

    const string RobotPost::START_PGX = "<?xml version=\"1.0\" encoding=\"utf-8\"?>\n<Programs xmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\" xmlns=\"http://www.staubli.com/robotics/VAL3/Program/2\">\n  <Program name=\"start\">\n    <Code><![CDATA[begin\n  call main()\nend]]></Code>\n  </Program>\n</Programs>\n";

    const string RobotPost::STOP_PGX = "<?xml version=\"1.0\" encoding=\"utf-8\"?>\n<Programs xmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\" xmlns=\"http://www.staubli.com/robotics/VAL3/Program/2\">\n  <Program name=\"stop\">\n    <Code><![CDATA[begin\nend]]></Code>\n  </Program>\n</Programs>\n";

    const string RobotPost::LOAD_NEXT_ONE = "<?xml version=\"1.0\" encoding=\"utf-8\" ?>\n<Programs xmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\" xmlns=\"http://www.staubli.com/robotics/VAL3/Program/2\">\n  <Program name=\"loadNextOne\" access=\"private\">\n    <Parameters xmlns=\"http://www.staubli.com/robotics/VAL3/Param/1\">\n      <Parameter name=\"x_sName\" type=\"string\" xsi:type=\"element\" />\n    </Parameters>\n    <Code><![CDATA[\nbegin\n  prog_swap:libLoad(x_sName)\nend\n      ]]></Code>\n  </Program>\n</Programs>";

    const string RobotPost::START_PGX_MAIN = "<?xml version=\"1.0\" encoding=\"utf-8\" ?>\n<Programs xmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\" xmlns=\"http://www.staubli.com/robotics/VAL3/Program/2\">\n  <Program name=\"start\" access=\"public\">\n    <Code><![CDATA[\nbegin\n%s\nend\n      ]]></Code>\n  </Program>\n</Programs>\n";

    const string RobotPost::PROGRAM_PJX_MAIN = "<?xml version=\"1.0\" encoding=\"utf-8\" ?>\n<Project xmlns=\"http://www.staubli.com/robotics/VAL3/Project/3\">\n  <Parameters version=\"s7.3.1\" stackSize=\"5000\" millimeterUnit=\"true\" />\n  <Programs>\n    <Program file=\"loadNextOne.pgx\" />\n    <Program file=\"start.pgx\" />\n    <Program file=\"stop.pgx\" />\n  </Programs>\n  <Database>\n    <Data file=\"%s.dtx\" />\n  </Database>\n  <Libraries>\n    <Library alias=\"prog\" path=\"./%s.pjx\" />\n    <Library alias=\"prog_swap\" path=\"./%s.pjx\" />\n    <Library alias=\"tooldata\" path=\"saveChangeTool\" />\n  </Libraries>\n</Project>";

    // 构造函数
    RobotPost::RobotPost(const string robotPost, const string robotName,
        int robotAxes,
        vector<string> axesType,
        const string ip_com,
        const Config config)
        : config_(config),
        ROBOT_POST(robotPost),
        ROBOT_NAME(robotName),
        nAxes(robotAxes),
        ip_com_(ip_com)
    {
        // 初始化所有成员变量
        PROG_MOVE_COUNT = 0;
        TAB_PGX = "  ";
        PROG_NAME = "unknown";
        MAIN_FOLDER = "ProgRobot";
        PROG_PGX = "";
        LOG = "";
        REF = eye(4);
        TOOL = eye(4);
        REF_COUNT = 0;
        TOOL_COUNT = 0;
        SPEED = -1;
        CONF_shoulder = 0;
        CONF_lowerarm = 0;
        CONF_flip = 0;
        MOVEC_USED = false;
    }

    // 程序开始
    void RobotPost::ProgStart(const string& progname) {
        PROG_NAME = FilterName(progname, 'P', {});
        currentProg_ = ProgramData(PROG_NAME);
        addLine("// Program " + progname + " start");
    }

    // 程序结束
    void RobotPost::ProgFinish(const string& progname) {
        addLine("");
        addLine("waitEndMove()");
        addLine("// Program " + progname + " end");
        prog2List();
    }

    // 关节运动
    void RobotPost::MoveJ(const Mat& pose, const vector<double>& joints, const vector<double>& confRLF) {
        controlProgSize();
        addLine("call MoveJoint(" + angles2Str(joints) + ")");
    }

    // 直线运动
    void RobotPost::MoveL(const Mat& pose, const vector<double>& joints, const vector<double>& confRLF) {
        if (pose == Mat()) {
            string msg = "Linear movement using joint targets is not supported. Change the target type to cartesian or use a joint movement.";
            addLog(msg);
            RunMessage(msg, true);
            return;
        }

        controlProgSize();

        int rear, lowerarm, flip;
        if (confRLF.empty()) {
            rear = CONF_shoulder;
            lowerarm = CONF_lowerarm;
            flip = CONF_flip;
        }
        else {
            rear = confRLF[0];
            lowerarm = confRLF[1];
            flip = confRLF[2];
        }

        if (rear != CONF_shoulder) {
            addLine("pPointRx.config.shoulder = " + string(rear > 0 ? "righty" : "lefty"));
            CONF_shoulder = rear;
        }
        if (lowerarm != CONF_lowerarm) {
            addLine("pPointRx.config.elbow = " + string(lowerarm > 0 ? "enegative" : "epositive"));
            CONF_lowerarm = lowerarm;
        }
        if (flip != CONF_flip) {
            addLine("pPointRx.config.wrist = " + string(flip > 0 ? "wnegative" : "wpositive"));
            CONF_flip = flip;
        }

        addLine("call MoveLine(" + pose2Str(pose) + ")");
    }

    // 圆弧运动
    void RobotPost::MoveC(const Mat& pose1, const vector<double>& joints1, const Mat& pose2,
        const vector<double>& joints2, const vector<double>& confRLF1, const vector<double>& confRLF2) {
        if (pose1 == Mat() || pose2 == Mat()) {
            string msg = "Circular movement using joint targets is not supported. Change the target type to cartesian or use a joint movement.";
            addLog(msg);
            RunMessage(msg, true);
            return;
        }

        MOVEC_USED = true;
        controlProgSize();
        addLine("call MoveCirc(" + pose2Str(pose1) + "," + pose2Str(pose2) + ")");
    }

    // 设置坐标系
    void RobotPost::setFrame(const Mat& pose, int frameId, const string& frameName) {
        if (REF_COUNT > 1) {
            RunMessage("Warning: This post processor is meant to use one reference frame. Errors might follow.", true);
            addLog("This post processor is meant to use one reference frame. Errors might follow.");
        }

        controlProgSize();
        REF = pose;
        REF_COUNT++;
    }

    // 设置工具
    void RobotPost::setTool(const Mat& pose, int toolId, const string& toolName) {
        if (TOOL_COUNT > 1) {
            RunMessage("Warning: Only one tool allowed per program. Tool change skipped.", true);
            addLog("Only one tool allowed per program");
            return;
        }

        controlProgSize();
        TOOL = pose;
        TOOL_COUNT++;
    }

    // 设置速度
    void RobotPost::setSpeed(double speed_mms) {
        if (SPEED != speed_mms) {
            SPEED = speed_mms;
            addLine("mCurrentSpeed.tvel = " + to_string(speed_mms));
        }
    }

    void RobotPost::setAcceleration(double accel_mmss) {
        config_.ACCEL_MMSS = accel_mmss;
    }

    void RobotPost::setSpeedJoints(double speed_degs) {
        config_.SPEED_DEGS = speed_degs;
    }

    void RobotPost::setAccelerationJoints(double accel_degss) {
        config_.ACCEL_DEGSS = accel_degss;
    }

    void RobotPost::setZoneData(double zone_mm) {
        addLine("mCurrentSpeed.reach = " + to_string(zone_mm));
        addLine("mCurrentSpeed.leave = " + to_string(zone_mm));
    }

    // I/O 操作
    void RobotPost::setDO(const string& io_var, const string& io_value) {
        string var_name = io_var;
        string value_str = io_value;

        // 处理数字IO值
        if (isdigit(io_var[0])) {
            var_name = "Output[" + io_var + "]";
        }

        // 处理数字值
        if (isdigit(io_value[0])) {
            int value = stoi(io_value);
            value_str = value > 0 ? "true" : "false";
        }

        addLine(var_name + "=" + value_str);
    }

    void RobotPost::setAO(const string& io_var, const string& io_value) {
        setDO(io_var, io_value);
    }

    void RobotPost::waitDI(const string& io_var, const string& io_value, int timeout_ms) {
        string var_name = io_var;
        string value_str = io_value;

        // 处理数字IO值
        if (isdigit(io_var[0])) {
            var_name = "Input[" + io_var + "]";
        }

        // 处理数字值
        if (isdigit(io_value[0])) {
            int value = stoi(io_value);
            value_str = value > 0 ? "true" : "false";
        }

        if (timeout_ms < 0) {
            addLine("watch(" + var_name + "==" + value_str + ")");
        }
        else {
            addLine("watch(" + var_name + "==" + value_str + "," + to_string(timeout_ms * 0.001) + ")");
        }
    }

    // 代码生成
    void RobotPost::RunCode(const string& code, bool is_function_call) {
        controlProgSize();
        if (is_function_call) {
            if (!code.empty() && code.find('(') == string::npos) {
                addLine("//call prog:" + code+"()");
            }
            else {
                addLine("call " + code);
            }
        }
        else {
            addLine(code);
        }
    }

    void RobotPost::RunMessage(const string& message, bool is_comment) {
        controlProgSize();
        if (is_comment) {
            addLine("// " + message);
        }
        else {
            addLine("popUpMsg(\"" + message + "\")");
        }
    }

    void RobotPost::Pause(int time_ms) {
        controlProgSize();
        if (time_ms < 0) {
            addLine("popUpMsg(\"Paused. Press OK to continue\")");
        }
        else {
            addLine("delay(" + to_string(time_ms * 0.001) + ")");
        }
    }

    // 工具函数
    string RobotPost::FilterName(const string& namefilter, char safechar, const vector<string>& reserved_names) {
        string result = namefilter;
        string invalid_chars = " .-[]/\\;,><&*:%=+@!#^|?^";

        // 移除无效字符
        for (char c : invalid_chars) {
            result.erase(remove(result.begin(), result.end(), c), result.end());
        }

        // 只保留ASCII字符
        result.erase(remove_if(result.begin(), result.end(), [](char c) { return c < 0 || c > 127; }), result.end());

        if (result.empty()) {
            result = string(1, safechar);
        }

        // 如果以数字开头，添加安全字符
        if (!result.empty() && isdigit(result[0])) {
            result = string(1, safechar) + result;
        }

        // 检查保留名称
        if (!reserved_names.empty()) {
            vector<string> reserved_lower;
            for (const auto& name : reserved_names) {
                string lower_name = name;
                transform(lower_name.begin(), lower_name.end(), lower_name.begin(), ::tolower);
                reserved_lower.push_back(lower_name);
            }

            string result_lower = result;
            transform(result_lower.begin(), result_lower.end(), result_lower.begin(), ::tolower);

            int cnt = 1;
            while (find(reserved_lower.begin(), reserved_lower.end(), result_lower) != reserved_lower.end()) {
                if (result.length() > 12) {
                    result = result.substr(0, 12);
                }
                result = result + to_string(cnt);
                result_lower = result;
                transform(result_lower.begin(), result_lower.end(), result_lower.begin(), ::tolower);
                cnt++;
            }
        }

        return result;
    }

    string RobotPost::pose2Str(const Mat& pose) {
        vector<double> xyzrpw = Pose_2_Staubli(pose);
        stringstream ss;
        ss << fixed << setprecision(3);
        ss << xyzrpw[0] << "," << xyzrpw[1] << "," << xyzrpw[2] << ","
            << xyzrpw[3] << "," << xyzrpw[4] << "," << xyzrpw[5];
        return ss.str();
    }

    string RobotPost::angles2Str(const vector<double>& angles) {
        stringstream ss;
        ss << fixed << setprecision(5);
        for (size_t i = 0; i < angles.size(); ++i) {
            if (i > 0) ss << ",";
            ss << angles[i];
        }
        return ss.str();
    }

    string RobotPost::pose2StrDtx(const Mat& pose) {
        vector<double> xyzrpw = Pose_2_Staubli(pose);
        stringstream ss;
        ss << fixed << setprecision(3);
        ss << "x=\"" << xyzrpw[0] << "\" y=\"" << xyzrpw[1] << "\" z=\"" << xyzrpw[2] << "\" "
            << "rx=\"" << xyzrpw[3] << "\" ry=\"" << xyzrpw[4] << "\" rz=\"" << xyzrpw[5] << "\"";
        return ss.str();
    }

    // 私有方法实现
    void RobotPost::addLine(const string& newline) {
        currentProg_.lines.push_back(TAB_PGX + newline);
    }

    void RobotPost::addLog(const string& newline) {
        LOG += newline + "\n";
    }

    void RobotPost::controlProgSize() {
        PROG_MOVE_COUNT++;
        if (PROG_MOVE_COUNT > config_.PROG_MOVE_COUNT_MAX) {
            closeModule();
        }
    }

    void RobotPost::closeModule() {
        if (PROG_MOVE_COUNT == 0) {
            return;
        }

        // 移除添加数字后缀的逻辑，直接使用原始程序名
        string progname = PROG_NAME;

        PROG_PGX_LIST.push_back(currentProg_);
        PROG_NAME_LIST.push_back(progname);

        // 重置当前程序
        currentProg_ = ProgramData(progname);
        PROG_MOVE_COUNT = 0;

        // 重新设置速度
        if (SPEED > 0) {
            addLine("mCurrentSpeed.tvel = " + to_string(SPEED));
        }
    }

    bool RobotPost::DirExists(const string& path) {
        struct _stat info;
        return _stat(path.c_str(), &info) == 0 && (info.st_mode & _S_IFDIR);
    }

    bool RobotPost::CreateDir(const string& path) {
        if (DirExists(path)) {
            return true;
        }

        string current;
        for (char c : path) {
            current += c;
            if (c == '\\' || c == '/') {
                if (!DirExists(current) && _mkdir(current.c_str()) != 0) {
                    return false;
                }
            }
        }

        return _mkdir(path.c_str()) == 0 || DirExists(path);
    }

    string RobotPost::FilterNameS6(const string& name) {
        string invalid_chars = "_ .-[]/\\;,><&*:%=+@!#^|?^";
        string result = name;

        for (char c : invalid_chars) {
            result.erase(remove(result.begin(), result.end(), c), result.end());
        }

        if (result.empty()) {
            result = "P";
        }

        return result;
    }

    // 在ProgSave方法中修改程序保存逻辑
    // 修改后的 ProgSave 方法
    void RobotPost::ProgSave(const string& folder, const string& progname, bool askUser, const string& showResult) {
        string progname_filtered = FilterName(progname, 'P', {});
        closeModule();

        string foldersave = folder;
        size_t nprogs = PROG_PGX_LIST.size();

        // 直接使用程序名作为文件夹名
        string folderprog = foldersave + "/" + progname_filtered;
        MAIN_FOLDER = progname_filtered;

        if (!CreateDir(folderprog)) {
            cerr << "Error creating directory: " << folderprog << endl;
            return;
        }

        vector<string> show_file_list;
        string added_modules = MODULES_ADD;

        if (MOVEC_USED) {
            added_modules += "<Program file=\"MoveCirc.pgx\" />\n";
        }

        // 合并所有程序的代码到一个main.pgx文件
        string main_prog_content;
        for (size_t i = 0; i < nprogs; i++) {
            for (const auto& line : PROG_PGX_LIST[i].lines) {
                main_prog_content += line + "\n";
            }
        }

        // 确保内容正确结束
        if (!main_prog_content.empty() && main_prog_content.back() == ',') {
            main_prog_content.pop_back();
        }

        // 保存main.pgx（包含所有程序代码）
        string main_data = folderprog + "/main.pgx";
        show_file_list.push_back(main_data);
        ofstream fid_main(main_data, ios::out | ios::binary);
        if (fid_main.is_open()) {
            fid_main << "\xEF\xBB\xBF";

            // 替换 snprintf 为字符串操作
            string main_content = MAIN_PGX;
            size_t pos = main_content.find("%s");
            if (pos != string::npos) {
                main_content.replace(pos, 2, main_prog_content);
            }
            fid_main << main_content;
            fid_main.close();
        }

        // 保存start.pgx
        string start_file = folderprog + "/start.pgx";
        ofstream fid_start(start_file, ios::out | ios::binary);
        if (fid_start.is_open()) {
            fid_start << "\xEF\xBB\xBF";
            fid_start << START_PGX;
            fid_start.close();
        }

        // 保存stop.pgx
        string stop_file = folderprog + "/stop.pgx";
        ofstream fid_stop(stop_file, ios::out | ios::binary);
        if (fid_stop.is_open()) {
            fid_stop << "\xEF\xBB\xBF";
            fid_stop << STOP_PGX;
            fid_stop.close();
        }

        // 保存MoveJoint.pgx
        string movej_file = folderprog + "/MoveJoint.pgx";
        ofstream fid_movej(movej_file, ios::out | ios::binary);
        if (fid_movej.is_open()) {
            fid_movej << "\xEF\xBB\xBF";
            fid_movej << MOVEJOINT_PGX;
            fid_movej.close();
        }

        // 保存MoveLine.pgx
        string movel_file = folderprog + "/MoveLine.pgx";
        ofstream fid_movel(movel_file, ios::out | ios::binary);
        if (fid_movel.is_open()) {
            fid_movel << "\xEF\xBB\xBF";
            fid_movel << MOVELINE_PGX;
            fid_movel.close();
        }

        if (MOVEC_USED) {
            string movec_file = folderprog + "/MoveCirc.pgx";
            ofstream fid_movec(movec_file, ios::out | ios::binary);
            if (fid_movec.is_open()) {
                fid_movec << "\xEF\xBB\xBF";
                fid_movec << MOVECIRC_PGX;
                fid_movec.close();
            }
        }

        // 只为最后一个程序保存.pjx和.dtx文件（Pause）
        if (nprogs > 0) {
            size_t last_index = nprogs - 1;
            string last_prog_name = PROG_NAME_LIST[last_index];

            // 保存项目文件（Pause.pjx）
            string project_file = folderprog + "/" + last_prog_name + ".pjx";
            ofstream fid_pjx(project_file, ios::out | ios::binary);
            if (fid_pjx.is_open()) {
                fid_pjx << "\xEF\xBB\xBF";

                // 替换 snprintf 为字符串操作
                string project_content = PROGRAM_PJX;
                size_t pos1 = project_content.find("%s");
                if (pos1 != string::npos) {
                    project_content.replace(pos1, 2, added_modules);
                    size_t pos2 = project_content.find("%s", pos1);
                    if (pos2 != string::npos) {
                        project_content.replace(pos2, 2, last_prog_name);
                    }
                }
                fid_pjx << project_content;
                fid_pjx.close();
            }

            // 保存数据文件（Pause.dtx）
            string program_data = folderprog + "/" + last_prog_name + ".dtx";
            show_file_list.push_back(program_data);
            ofstream fid_dtx(program_data, ios::out | ios::binary);
            if (fid_dtx.is_open()) {
                fid_dtx << "\xEF\xBB\xBF";

                // 替换 snprintf 为字符串操作
                string data_content = PROGRAM_DTX;
                size_t pos = data_content.find("%s");
                if (pos != string::npos) {
                    string pose_str = pose2StrDtx(REF);
                    data_content.replace(pos, 2, pose_str);
                }
                fid_dtx << data_content;
                fid_dtx.close();
            }
        }

        PROG_FILES.clear();
        PROG_FILES.push_back(folderprog);
    }

    // 在prog2List方法中修改程序命名逻辑
    void RobotPost::prog2List() {
        if (currentProg_.lines.empty()) {
            return;
        }

        // 对于所有程序，使用原始名称（不添加数字后缀）
        string progName = currentProg_.name;

        PROG_PGX_LIST.push_back(currentProg_);
        PROG_NAME_LIST.push_back(progName);

        // 重置当前程序
        currentProg_ = ProgramData(progName);
        currentProg_.lines.clear(); // 清空行
    }


    // 发送程序到机器人
    void RobotPost::ProgSendRobot(const string& remote_path,
        const string& ftp_user, const string& ftp_pass)
    {
        if (PROG_FILES.empty()) {
            cerr << "No program files to send. Please save program first." << endl;
            return;
        }

        cout << "Sending programs to robot " << ip_com_ << endl;
        bool success = UploadFTP(PROG_FILES[0], ip_com_, remote_path, ftp_user, ftp_pass);

        if (success) {
            cout << "Upload successful!" << endl;
        }
        else {
            cout << "Upload failed!" << endl;
        }
    }

    // 公共访问器
    const vector<string> RobotPost::getProgramLines() const {
        vector<string> allLines;
        for (const auto& prog : PROG_PGX_LIST) {
            allLines.insert(allLines.end(), prog.lines.begin(), prog.lines.end());
        }
        allLines.insert(allLines.end(), currentProg_.lines.begin(), currentProg_.lines.end());
        return allLines;
    }

    // 测试函数
    void test_post(const string& path) {
        auto Pose = PosePP;
        cout << "Testing Staubli VAL3 Post Processor" << endl;
        cout << "===================================" << endl;

        // 创建机器人后处理器
        RobotPost robot("Staubli_VAL3", "TX200");

        //// 开始程序
        //robot.ProgStart("TestProgram");
        //robot.RunMessage("Program generated by hejun using Staubli VAL3 post processor", true);

        //// 设置坐标系和工具
        //robot.setFrame(Pose(807.766544, -963.699898, 41.478944, 0, 0, 0));
        //robot.setTool(Pose(62.5, -108.253175, 100, -60, 90, 0));

        //// 关节运动
        //robot.MoveJ(Pose(200, 200, 500, 180, 0, 180),
        //    { -46.18419, -6.77518, -20.54925, 71.38674, 49.58727, -302.54752 });

        //// 直线运动
        //robot.MoveL(Pose(200, 250, 348.734575, 180, 0, -150),
        //    { -41.62707, -8.89064, -30.01809, 60.62329, 49.66749, -258.98418 });

        //robot.MoveL(Pose(200, 200, 262.132034, 180, 0, -150),
        //    { -43.73892, -3.91728, -35.77935, 58.57566, 54.11615, -253.81122 });

        //// 设置IO和消息
        //robot.RunMessage("Setting air valve 1 on");
        //robot.RunCode("TCP_On", true);
        //robot.Pause(1000);

        //// 更多运动
        //robot.MoveL(Pose(200, 250, 348.734575, 180, 0, -150),
        //    { -41.62707, -8.89064, -30.01809, 60.62329, 49.66749, -258.98418 });

        //robot.MoveL(Pose(250, 300, 278.023897, 180, 0, -150),
        //    { -37.52588, -6.32628, -34.59693, 53.52525, 49.24426, -251.44677 });

        //robot.MoveL(Pose(250, 250, 191.421356, 180, 0, -150),
        //    { -39.75778, -1.04537, -40.37883, 52.09118, 54.15317, -246.94403 });

        //// 设置IO
        //robot.RunMessage("Setting air valve off");
        //robot.RunCode("TCP_Off", true);
        //robot.Pause(1000);

        //// 最后几个运动
        //robot.MoveL(Pose(250, 300, 278.023897, 180, 0, -150),
        //    { -37.52588, -6.32628, -34.59693, 53.52525, 49.24426, -251.44677 });

        //robot.MoveL(Pose(250, 200, 278.023897, 180, 0, -150),
        //    { -41.85389, -1.95619, -34.89154, 57.43912, 52.34162, -253.73403 });

        //robot.MoveL(Pose(250, 150, 191.421356, 180, 0, -150),
        //    { -43.82111, 3.29703, -40.29493, 56.02402, 56.61169, -249.23532 });

        //// 结束程序
        //robot.ProgFinish("TestProgram");
        robot.ProgStart("Curve");
        robot.RunMessage("Program generated by hejun for Staubli RX90B on 01/10/2025 11:20:46", true);
        robot.RunMessage("Using nominal kinematics.", true);
        robot.RunCode("Pause", true);
        robot.setZoneData(1.000);
        robot.setSpeed(100.000);
        robot.setFrame(Pose(971.355, 225.235, -114.112, 180, 30, 0), -1, "Part");
        robot.setTool(Pose(0, 0, 90.865, 0, 0, 0), -1, "RobotiQ Sanding Kit");
        robot.RunMessage("Show RobotiQ Sanding Kit", true);

        robot.MoveJ(robomath::Mat(), { 11.4709, 61.1794, 42.834, -7.84591, 46.7536, -74.6394 });
        robot.MoveL(Pose(25.0003, 60, 44.9998, 90, 0, -180), { 11.1379, 68.0668, 35.2204, -7.53604, 47.43, -75.21 });
        robot.MoveL(Pose(50.0003, 60, 19.9998, 90, 0, -180), { 11.2575, 68.1662, 39.5386, -8.21781, 43.0698, -74.1969 });

        robot.setSpeed(50.000);
        robot.MoveL(Pose(350, 60, 19.9997, 90, 0, -180), { 16.1577, 53.6746, 95.4069, -76.6684, 8.22117, 0.620307 });
        robot.MoveL(Pose(350, 94.233, 19.9997, 90, 0, -180), { 12.9365, 53.116, 96.8499, -84.0923, 6.46138, 5.30564 });
        robot.MoveL(Pose(347.348, 95.2783, 19.9997, 90, 0, -180), { 12.7869, 53.0984, 96.5596, -81.4334, 6.42553, 2.49969 });
        robot.MoveL(Pose(332.253, 102.483, 19.9997, 90, 0, -180), { 11.8371, 53.0173, 94.9111, -66.2367, 6.43446, -13.6094 });
        robot.MoveL(Pose(318.126, 111.437, 19.9997, 90, 0, -180), { 10.7759, 52.9694, 93.3721, -52.726, 6.74666, -28.1064 });
        robot.MoveL(Pose(305.167, 122.011, 19.9997, 90, 0, -180), { 9.61787, 52.9479, 91.9626, -41.4885, 7.24435, -40.3905 });
        robot.MoveL(Pose(293.562, 134.056, 19.9997, 90, 0, -180), { 8.37693, 52.946, 90.7016, -32.3646, 7.82086, -50.6095 });
        robot.MoveL(Pose(283.476, 147.398, 19.9997, 90, 0, -180), { 7.06627, 52.9572, 89.6062, -24.9105, 8.39699, -59.1972 });
        robot.MoveL(Pose(275.053, 161.848, 19.9997, 90, 0, -180), { 5.69852, 52.9752, 88.6915, -18.6771, 8.91876, -66.5942 });
        robot.MoveL(Pose(268.413, 177.2, 19.9997, 90, 0, -180), { 4.28583, 52.9948, 87.9705, -13.2961, 9.35047, -73.161 });
        robot.MoveL(Pose(263.652, 193.233, 19.9997, 90, 0, -180), { 2.83999, 53.0117, 87.4536, -8.48211, 9.66895, -79.1767 });
        robot.MoveL(Pose(260.837, 209.72, 19.9997, 90, 0, -180), { 1.37252, 53.0226, 87.1484, -4.01054, 9.85981, -84.8598 });
        robot.MoveL(Pose(260.008, 226.426, 19.9997, 90, 0, -180), { -0.105226, 53.0256, 87.0593, 0.30555, 9.9153, -90.3921 });
        robot.MoveL(Pose(261.177, 243.11, 19.9997, 90, 0, -180), { -1.58199, 53.0202, 87.1876, 4.63612, 9.83321, -95.9384 });
        robot.MoveL(Pose(264.327, 259.537, 19.9997, 90, 0, -180), { -3.04649, 53.0071, 87.5315, 9.1528, 9.61664, -101.665 });
        robot.MoveL(Pose(269.414, 275.47, 19.9997, 90, 0, -180), { -4.48736, 52.9886, 88.0859, 14.048, 9.27437, -107.76 });
        robot.MoveL(Pose(276.365, 290.683, 19.9997, 90, 0, -180), { -5.89302, 52.968, 88.8427, 19.5558, 8.82208, -114.45 });
        robot.MoveL(Pose(285.08, 304.959, 19.9997, 90, 0, -180), { -7.25164, 52.95, 89.7911, 25.9767, 8.28480, -122.029 });
        robot.MoveL(Pose(295.436, 318.093, 19.9997, 90, 0, -180), { -8.55098, 52.94, 90.9176, 33.6969, 7.70096, -130.877 });
        robot.MoveL(Pose(307.284, 329.899, 19.9997, 90, 0, -180), { -9.77833, 52.9442, 92.2067, 43.1799, 7.12860, -141.447 });
        robot.MoveL(Pose(320.456, 340.207, 19.9997, 90, 0, -180), { -10.9204, 52.9692, 93.641, 54.8531, 6.65225, -154.157 });
        robot.MoveL(Pose(334.762, 348.871, 19.9997, 90, 0, -180), { -11.9631, 53.022, 95.2013, 68.783, 6.38315, -169.06 });
        robot.MoveL(Pose(350, 355.767, 19.9997, 90, 0, -180), { -12.8917, 53.1092, 96.8674, 84.2057, 6.43795, -185.381 });
        robot.MoveL(Pose(350, 390, 19.9997, 90, 0, -180), { -16.1141, 53.6661, 95.429, 76.7609, 8.19629, -180.676 });
        robot.MoveL(Pose(50.0003, 390, 19.9998, 90, 0, -180), { -11.2263, 68.1487, 39.573, 8.19842, 43.0487, -105.764 });
        robot.MoveL(Pose(50.0003, 123.5, 19.9998, 90, 0, -180), { 6.98701, 66.3261, 43.1691, -5.33961, 40.8126, -79.8953 });
        robot.MoveL(Pose(223.62, 123.5, 19.9997, 90, 0, -180), { 8.51186, 54.7018, 79.9615, -15.4174, 16.1635, -67.7796 });
        robot.MoveL(Pose(218.5, 132.968, 19.9997, 90, 0, -180), { 7.6803, 54.7526, 79.3964, -13.5977, 16.5126, -70.2822 });
        robot.MoveL(Pose(210.282, 151.385, 19.9997, 90, 0, -180), { 6.09752, 54.8393, 78.4875, -10.4184, 17.0793, -74.746 });
        robot.MoveL(Pose(203.927, 170.526, 19.9997, 90, 0, -180), { 4.48925, 54.9107, 77.7829, -7.46888, 17.5222, -78.9842 });
        robot.MoveL(Pose(199.498, 190.202, 19.9997, 90, 0, -180), { 2.86248, 54.9626, 77.2912, -4.67683, 17.8328, -83.0675 });
        robot.MoveL(Pose(197.038, 210.219, 19.9997, 90, 0, -180), { 1.22403, 54.9919, 77.0184, -1.98018, 18.0054, -87.0566 });
        robot.MoveL(Pose(196.572, 230.382, 19.9997, 90, 0, -180), { -0.419341, 54.997, 76.9677, 0.677169, 18.0371, -91.0071 });
        robot.MoveL(Pose(198.103, 250.491, 19.9997, 90, 0, -180), { -2.0609, 54.9775, 77.1399, 3.34888, 17.9272, -94.9716 });
        robot.MoveL(Pose(201.618, 270.351, 19.9997, 90, 0, -180), { -3.69391, 54.9346, 77.5328, 6.08952, 17.6777, -99.0041 });
        robot.MoveL(Pose(207.081, 289.765, 19.9997, 90, 0, -180), { -5.31158, 54.8706, 78.1416, 8.95799, 17.2929, -103.162 });
        robot.MoveL(Pose(214.439, 308.543, 19.9997, 90, 0, -180), { -6.907, 54.7893, 78.9589, 12.0214, 16.78, -107.512 });
        robot.MoveL(Pose(223.62, 326.5, 19.9997, 90, 0, -180), { -8.47311, 54.6954, 79.9752, 15.3597, 16.1492, -112.132 });
        robot.MoveL(Pose(113.5, 326.5, 19.9998, 90, 0, -180), { -7.44313, 60.3841, 59.4134, 7.30666, 30.6164, -102.752 });
        robot.MoveL(Pose(113.5, 187, 19.9998, 90, 0, -180), { 2.82396, 59.6675, 60.8479, -2.86343, 29.5454, -85.0623 });
        robot.MoveL(Pose(159.484, 187, 19.9997, 90, 0, -180), { 2.97663, 56.825, 70.1196, -3.78923, 23.1341, -83.9363 });
        robot.MoveL(Pose(158.527, 193.651, 19.9998, 90, 0, -180), { 2.45679, 56.8416, 70.0094, -3.11849, 23.2025, -85.0053 });
        robot.MoveL(Pose(156.68, 215.636, 19.9998, 90, 0, -180), { 0.74547, 56.8736, 69.7973, -0.941051, 23.334, -88.4903 });
        robot.MoveL(Pose(156.832, 237.698, 19.9998, 90, 0, -180), { -0.967997, 56.8701, 69.8164, 1.22258, 23.3218, -91.961 });
        robot.MoveL(Pose(158.979, 259.655, 19.9997, 90, 0, -180), { -2.67853, 56.8314, 70.0664, 3.40514, 23.1659, -95.4512 });

        robot.setSpeed(100.000);
        robot.MoveL(Pose(161.412, 284.537, 44.9997, 90, 0, -180), { -4.70107, 53.7921, 72.9771, 5.91628, 23.4258, -99.5052 });
        std::cout << "Done instruction: 50" << std::endl;
        //std::cout.flush();
        robot.MoveL(Pose(161.412, 284.537, 94.9997, 90, 0, -180), { -4.8691, 47.779, 77.928, 5.87482, 24.4962, -99.5685 });

        robot.ProgFinish("Curve");
        robot.ProgStart("Pause");
        robot.Pause(4000.0);
        robot.ProgFinish("Pause");
    
        // 输出程序内容
        cout << "\nGenerated program content:" << endl;
        cout << "==========================" << endl;

        vector<string> programLines = robot.getProgramLines();
        for (const auto& line : programLines) {
            cout << line << endl;
        }

        // 保存程序
        robot.ProgSave(path, "Curve", false, "");

        cout << "\nProgram generation completed successfully!" << endl;
        cout << "Output saved to: " << path << endl;

        // 显示日志信息
        string log = robot.getLog();
        if (!log.empty()) {
            cout << "\nGeneration log:" << endl;
            cout << log << endl;
        }
    }
}