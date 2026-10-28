#include <PostModel/Staubli_S6.h>
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
#pragma comment(lib, "shell32.lib")  // 添加这行
#pragma comment(lib, "wininet.lib")  // 已有的
#pragma execution_character_set("utf-8")

namespace Staubli_S6 {
    using namespace robomath;
    using namespace std;

    // XML模板定义
    const string PROGRAM_PJX = "<?xml version=\"1.0\" encoding=\"utf-8\" ?>\n"
    "<project xmlns=\"ProjectNameSpace\" >\n"
    "  <parameters version=\"s6.4.2\" stackSize=\"5000\" millimeterUnit=\"true\" />\n"
    "  <programSection>\n"
    "    <program file=\"start.pgx\" />\n"
    "    <program file=\"stop.pgx\" />\n"
    "  </programSection>\n"
    "  <dataSection>\n"
    "    <data file=\"%s.dtx\" />\n"
    "  </dataSection>\n"
    "  <aliasSection>\n"
    "  </aliasSection>\n"
    "</project>\n";

    const string PROGRAM_PJX_MAIN = "<?xml version=\"1.0\" encoding=\"utf-8\" ?>\n"
    "<project xmlns=\"ProjectNameSpace\" >\n"
    "  <parameters version=\"s6.4.2\" stackSize=\"5000\" millimeterUnit=\"true\" />\n"
    "  <programSection>\n"
    "    <program file=\"start.pgx\" />\n"
    "    <program file=\"stop.pgx\" />\n"
    "  </programSection>\n"
    "  <dataSection>\n"
    "    <data file=\"%s.dtx\" />\n"
    "  </dataSection>\n"
    "  <aliasSection>\n"
    "  </aliasSection>\n"
    "</project>\n";

    const string DATA_DTX = "<?xml version=\"1.0\" encoding=\"utf-8\" ?>\n"
    "<dataList xmlns=\"DataNameSpace\" >\n"
    "  <aioSection />\n"
    "  <boolSection />\n"
    "  <configRsSection />\n"
    "  <configSection />\n"
    "  <configVrbSection />\n"
    "  <dioSection />\n"
    "  <frameSection>\n"
    "    <frame name=\"world\" public=\"false\" privilege=\"0\" >\n"
    "      <fFather alias=\"\" name=\"\" fatherIndex=\"0\" />\n"
    "      <valueFrame index=\"0\" >\n"
    "        <tfValue x=\"0\" y=\"0\" z=\"0\" rx=\"0\" ry=\"0\" rz=\"0\" />\n"
    "      </valueFrame>\n"
    "    </frame>%s\n"
    "  </frameSection>\n"
    "  <jointRsSection />\n"
    "  <jointSection>%s\n"
    "  </jointSection>\n"
    "  <jointVrbSection />\n"
    "  <mdescSection>\n"
    "    <mdesc name=\"mNomSpeed\" public=\"true\" privilege=\"0\" >\n"
    "      <valueMdesc index=\"0\" >\n"
    "        <mdescValue accel=\"100\" vel=\"100\" decel=\"100\" tmax=\"99999\" rmax=\"99999\" blend=\"off\" leave=\"50\" reach=\"50\" />\n"
    "      </valueMdesc>\n"
    "    </mdesc>\n"
    "  </mdescSection>\n"
    "  <numSection />\n"
    "  <pointRsSection />\n"
    "  <pointSection>%s\n"
    "  </pointSection>\n"
    "  <pointVrbSection />\n"
    "  <sioSection />\n"
    "  <stringSection />\n"
    "  <toolSection>\n"
    "    <tool name=\"flange\" public=\"false\" privilege=\"0\" >\n"
    "      <tFather alias=\"\" name=\"\" fatherIndex=\"0\" />\n"
    "      <valueTool index=\"0\" >\n"
    "        <ttValue x=\"0\" y=\"0\" z=\"0\" rx=\"0\" ry=\"0\" rz=\"0\" />\n"
    "        <io alias=\"io\" name=\"valve1\" ioIndex=\"0\" open=\"0\" close=\"0\" />\n"
    "      </valueTool>\n"
    "    </tool>%s\n"
    "  </toolSection>\n"
    "  <trsfSection />\n"
    "</dataList>\n";

    const string START_PGX = "<?xml version=\"1.0\" encoding=\"utf-8\" ?>\n"
    "<programList xmlns=\"ProgramNameSpace\" >\n"
    "  <program name=\"start\" public=\"false\" >\n"
    "    <description />\n"
    "    <paramSection/>\n"
    "    <localSection/>\n"
    "    <source>\n"
    "      <code>begin\n"
    "  do\n"
    "  %s\n"
    "  until (false)\n"
    "end\n"
    "      </code>\n"
    "    </source>\n"
    "  </program>\n"
    "</programList>\n";

    const string STOP_PGX = "<?xml version=\"1.0\" encoding=\"utf-8\" ?>\n"
    "<programList xmlns=\"ProgramNameSpace\" >\n"
    "  <program name=\"stop\" public=\"false\" >\n"
    "    <description />\n"
    "    <paramSection/>\n"
    "    <localSection/>\n"
    "    <source>\n"
    "      <code>begin\n"
    "  popUpMsg(\"Pending movement commands have been canceled\")\n"
    "  resetMotion()\n"
    "end\n"
    "      </code>\n"
    "    </source>\n"
    "  </program>\n"
    "</programList>\n";

    const string LOAD_NEXT_ONE = "<?xml version=\"1.0\" encoding=\"utf-8\" ?>\n"
    "<programList xmlns=\"ProgramNameSpace\" >\n"
    "  <program name=\"stop\" public=\"false\" >\n"
    "    <description />\n"
    "    <paramSection/>\n"
    "    <localSection/>\n"
    "    <source>\n"
    "      <code>begin\n"
    "  prog_swap:libLoad(x_sName)\n"
    "end\n"
    "      </code>\n"
    "    </source>\n"
    "  </program>\n"
    "</programList>\n";

    const string LTX_FILE = "<?xml version=\"1.0\" encoding=\"utf-8\" ?>\n"
    "<externList>\n"
    "</externList>";

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
        // 初始化所有成员变量
        jointCount_ = 0;
        pointCount_ = 0;
        toolCount_ = 0;
        frameCount_ = 0;
        speedCount_ = 0;
        progMoveCount_ = 0;
        nProgs_ = 0;
        tab_ = "  ";
        speedMMS_ = config.DEFAULT_SPEED;
        toolDataCurrent_ = "tProg";
        wobjData_ = "world";
        speedDataCurrent_ = "mNomSpeed";
        currentFrame_ = robomath::eye(4);
        currentTool_ = robomath::eye(4);
        hasTrack_ = false;
        hasTurntable_ = false;
        cladOn_ = false;
        arcOn_ = false;
        printELast_ = 0.0;
        printENew_ = 0.0;
        printLastSignal_ = 0.0;
        printAccelMMSS_ = 100.0;
        printSpeed2Signal_ = 1.0;
        printFlowMaxSignal_ = 100.0;
        frExternalPose_ = robomath::Mat();
        printPoseLast_ = robomath::Mat();

        // 字符串初始化
        jointData_ = "";
        pointData_ = "";
        toolData_ = "";
        frameData_ = "";
        speedData_ = "";
        zoneData_ = "z1";
        log_ = "";
        MAIN_FOLDER_ = "ProgRobot";
        PROG_PGX = "";
        LAST_PG_NAME = "";
        // 容器初始化
        toolDef_ = { "\t! Tool variables: " };
        frameDef_ = { "\t! Reference variables:" };
        axesTrack_ = vector<int>();
        axesTurntable_ = vector<int>();
        toolList_ = vector<string>();
        frameList_ = vector<string>();
        progList_ = vector<ProgramData>();
        progNames_ = vector<string>();
        progFiles_ = vector<string>();

        // XML模板数据列表初始化
        PROG_PGX_LIST = vector<string>();
        PROG_DTX_LIST = vector<string>();
        PROG_PJX_LIST = vector<string>();

        // 对象初始化
        currentProg_ = ProgramData();
    
        // 初始化轴类型相关的变量
        for (size_t i = 0; i < axesType_.size(); ++i) {
            if (axesType_[i] == "T") {
                axesTrack_.push_back(static_cast<int>(i));
                hasTrack_ = true;
            }
            else if (axesType_[i] == "J") {
                axesTurntable_.push_back(static_cast<int>(i));
                hasTurntable_ = true;
            }
        }
    }

    // 程序开始
    void RobotPost::ProgStart(const string& progname)
    {
        string filteredName = FilterNameS6(progname);
        transform(filteredName.begin(), filteredName.end(), filteredName.begin(), ::tolower);
        currentProg_.name = filteredName;
        nProgs_++;
        addLine("// Program " + progname + " start");
    }

    // 程序结束
    void RobotPost::ProgFinish(const string& progname)
    {
        addLine("");
        addLine("waitEndMove()");
        addLine("// Program " + progname + " end");
        /*progList_.push_back(currentProg_);
        progNames_.push_back(progname);*/
    }

    // 发送程序到机器人
    void RobotPost::ProgSendRobot(const string& remote_path,
        const string& ftp_user, const string& ftp_pass)
    {
        if (progFiles_.empty()) {
            cerr << "No program files to send. Please save program first." << endl;
            return;
        }

        cout << "Sending programs to robot " << ip_com_ << endl;
        bool success = UploadFTP(progFiles_[0], ip_com_, remote_path, ftp_user, ftp_pass);

        if (success) {
            cout << "Upload successful!" << endl;
        }
        else {
            cout << "Upload failed!" << endl;
        }
    }

    // 关节运动
    void RobotPost::MoveJ(const robomath::Mat& pose, const vector<double>& joints,
        const vector<double>& confRLF)
    {
        controlProgSize();
        jointCount_++;

        string jointStr;
        for (size_t i = 0; i < joints.size(); ++i) {
            if (i > 0) jointStr += " ";
            jointStr += "j" + to_string(i + 1) + "=\"" +
                to_string(joints[i]) + "\"";
        }

        jointData_ += "\n    <joint name=\"t" + to_string(jointCount_) +
            "\" public=\"false\" privilege=\"0\" >\n      <valueJoint index=\"0\" >\n        <jointValue " +
            jointStr + " />\n      </valueJoint>\n    </joint>";

        addLine("movej(t" + to_string(jointCount_) + "," +
            toolDataCurrent_ + "," + speedDataCurrent_ + ")");
    }

    // 直线运动
    void RobotPost::MoveL(const robomath::Mat& pose, const vector<double>& joints,
        const vector<double>& confRLF)
    {
        if (pose==Mat()) {
            string msg = "Linear movement using joint targets is not supported. Change the target type to cartesian or use a joint movement.";
            addLog(msg);
            RunMessage(msg, true);
            return;
        }

        controlProgSize();
        pointCount_++;

        string poseStr = pose2Str(pose);
        pointData_ += "\n    <point name=\"p" + to_string(pointCount_) +
            "\" public=\"false\" privilege=\"0\" >\n      <pFather alias=\"\" name=\"" + wobjData_ + "\" fatherIndex=\"0\" />\n      <valuePoint index=\"0\" >\n        <tpValue " +
            poseStr + " />\n        <cpValue shoulder=\"ssame\" elbow=\"esame\" wrist=\"wsame\"/>\n      </valuePoint>\n    </point>";

        addLine("movel(p" + to_string(pointCount_) + "," +
            toolDataCurrent_ + "," + speedDataCurrent_ + ")");
    }

    // 圆弧运动
    void RobotPost::MoveC(const robomath::Mat& pose1, const vector<double>& joints1,
        const robomath::Mat& pose2, const vector<double>& joints2,
        const vector<double>& confRLF1,
        const vector<double>& confRLF2)
    {
        if (pose1==Mat() || pose2 == Mat()) {
            string msg = "Circular movement using joint targets is not supported. Change the target type to cartesian or use a joint movement.";
            addLog(msg);
            RunMessage(msg, true);
            return;
        }

        controlProgSize();

        // 创建第一个点
        pointCount_++;
        string pose1StrValue = pose2Str(pose1);
        pointData_ += "\n    <point name=\"p" + to_string(pointCount_) +
            "\" public=\"false\" privilege=\"0\" >\n      <pFather alias=\"\" name=\"" + wobjData_ + "\" fatherIndex=\"0\" />\n      <valuePoint index=\"0\" >\n        <tpValue " +
            pose1StrValue + " />\n        <cpValue shoulder=\"ssame\" elbow=\"esame\" wrist=\"wsame\"/>\n      </valuePoint>\n    </point>";

        // 创建第二个点
        pointCount_++;
        string pose2StrValue = pose2Str(pose2);
        pointData_ += "\n    <point name=\"p" + to_string(pointCount_) +
            "\" public=\"false\" privilege=\"0\" >\n      <pFather alias=\"\" name=\"" + wobjData_ + "\" fatherIndex=\"0\" />\n      <valuePoint index=\"0\" >\n        <tpValue " +
            pose2StrValue + " />\n        <cpValue shoulder=\"ssame\" elbow=\"esame\" wrist=\"wsame\"/>\n      </valuePoint>\n    </point>";

        addLine("movec(p" + to_string(pointCount_ - 1) + ",p" +
            to_string(pointCount_) + "," + toolDataCurrent_ + "," +
            speedDataCurrent_ + ")");
    }

    // 设置坐标系
    void RobotPost::setFrame(const robomath::Mat& pose, int frameId, const string& frameName)
    {
        currentFrame_ = pose;
        frameCount_++;
        wobjData_ = "fFrame" + to_string(frameCount_);

        string frameStr = pose2Str(pose);
        frameData_ += "\n    <frame name=\"" + wobjData_ +
            "\" public=\"false\" privilege=\"0\" >\n      <fFather alias=\"\" name=\"world\" fatherIndex=\"0\" />\n      <valueFrame index=\"0\" >\n        <tfValue " +
            frameStr + " />\n      </valueFrame>\n    </frame>";
    }

    // 设置工具
    void RobotPost::setTool(const robomath::Mat& pose, int toolId, const string& toolName)
    {
        currentTool_ = pose;
        toolCount_++;
        toolDataCurrent_ = "tTool" + to_string(toolCount_);

        string toolPoseStr = pose2Str(pose);
        toolData_ += "\n    <tool name=\"" + toolDataCurrent_ +
            "\" public=\"false\" privilege=\"0\" >\n      <tFather alias=\"\" name=\"flange\" fatherIndex=\"0\" />\n      <valueTool index=\"0\" >\n        <ttValue " +
            toolPoseStr + " />\n        <io alias=\"io\" name=\"valve1\" ioIndex=\"0\" open=\"0\" close=\"0\" />\n      </valueTool>\n    </tool>";
    }

    // 设置速度
    void RobotPost::setSpeed(double speed_mms)
    {
        if (speedMMS_ != speed_mms) {
            speedMMS_ = speed_mms;
            addLine("mNomSpeed.tvel = " + to_string(speed_mms));
        }
    }

    void RobotPost::setAcceleration(double accel_mmss)
    {
        // 设置笛卡尔加速度（mm/s²）
        if (accel_mmss > 0) {
            addLine("// Acceleration set to: " + to_string(accel_mmss) + " mm/s²");
        }
    }

    void RobotPost::setSpeedJoints(double speed_degs)
    {
        // 设置关节速度（deg/s）
        if (speed_degs > 0) {
            addLine("// Joint speed set to: " + to_string(speed_degs) + " deg/s");
            addLine("mNomSpeed.rvel = " + to_string(speed_degs));
        }
    }

    void RobotPost::setAccelerationJoints(double accel_degss)
    {
        // 设置关节加速度（deg/s²）
        if (accel_degss > 0) {
            addLine("// Joint acceleration set to: " + to_string(accel_degss) + " deg/s²");
            addLine("mNomSpeed.raccel = " + to_string(accel_degss));
        }
    }

    void RobotPost::setZoneData(double zone_mm)
    {
        // 设置区域数据（平滑过渡距离）
        controlProgSize();
        if (zone_mm >= 0) {
            addLine("// Zone data set to: " + to_string(zone_mm) + " mm");
        }
    }

    void RobotPost::setDO(const string& io_var, const string& io_value)
    {
        addLine(io_var + " = " + io_value);
    }

    void RobotPost::setAO(const string& io_var, const string& io_value)
    {
        setDO(io_var, io_value);
    }

    void RobotPost::waitDI(const string& io_var, const string& io_value, int timeout_ms)
    {
        if (timeout_ms < 0) {
            addLine("WAIT FOR " + io_var + " == " + io_value);
        }
        else {
            addLine("WAIT FOR " + io_var + " == " + io_value + " TIMEOUT=" +
                to_string(timeout_ms * 0.001));
        }
    }

    void RobotPost::RunCode(const string& code, bool is_function_call)
    {
        controlProgSize();
        if (is_function_call) {
            string filteredCode = FilterNameS6(code);
            addLine("//call prog:" + filteredCode);
        }
        else {
            addLine(code);
        }
    }

    void RobotPost::RunMessage(const string& message, bool is_comment)
    {
        controlProgSize();
        if (is_comment) {
            addLine("// " + message);
        }
        else {
            addLine("popUpMsg(\"" + message + "\")");
        }
    }

    void RobotPost::Pause(int time_ms)
    {
        controlProgSize();
        if (time_ms < 0) {
            addLine("popUpMsg(\"Paused. Press OK to continue\")");
        }
        else {
            stringstream ss;
            ss << fixed << setprecision(3);
            ss << "delay(" << (time_ms * 0.001) << ")";
            addLine(ss.str());
        }
    }

    // 工具函数
    string RobotPost::FilterName(const string& namefilter, char safechar,
        const vector<string>& reserved_names)
    {
        string result = namefilter;
        result.erase(remove_if(result.begin(), result.end(),
            [](char c) { return !isalnum(c); }), result.end());

        if (result.empty()) {
            result = string(1, safechar);
        }

        return result;
    }

    string RobotPost::pose2Str(const robomath::Mat& pose)
    {
        vector<double> xyzrpw = robomath::Pose_2_Staubli(pose);
        stringstream ss;
        ss << fixed << setprecision(3);
        ss << "x=\"" << xyzrpw[0] << "\" y=\"" << xyzrpw[1] << "\" z=\"" << xyzrpw[2] << "\" "
            << "rx=\"" << xyzrpw[3] << "\" ry=\"" << xyzrpw[4] << "\" rz=\"" << xyzrpw[5] << "\"";
        return ss.str();
    }

    string RobotPost::angles2Str(const vector<double>& angles)
    {
        stringstream ss;
        ss << fixed << setprecision(5);
        for (size_t i = 0; i < angles.size(); ++i) {
            if (i > 0) ss << " ";
            ss << "j" << (i + 1) << "=\"" << angles[i] << "\"";
        }
        return ss.str();
    }

    string RobotPost::extAxes2Str(const vector<double>& angles,
        const vector<double>& ratio)
    {
        stringstream ss;
        ss << fixed << setprecision(5);
        ss << "[";
        for (size_t i = 0; i < angles.size(); ++i) {
            if (i > 0) ss << ",";
            ss << angles[i];
        }
        // 填充到6个轴
        for (size_t i = angles.size(); i < 6; ++i) {
            ss << ",9E+09";
        }
        ss << "]";
        return ss.str();
    }

    // 私有方法实现
    void RobotPost::addLine(const string& newline)
    {
        PROG_PGX += tab_ + newline + "\n"; // 按照Python版本累积到PROG_PGX
        currentProg_.lines.push_back(tab_ + newline + "\n");
    }

    void RobotPost::addLog(const string& newline)
    {
        log_ += newline + "\n";
    }

    double RobotPost::calculateTime(double distance, double Vmax, double Amax)
    {
        // 简化的时间计算：距离/速度
        return distance / Vmax;
    }

    void RobotPost::newMove(const robomath::Mat& new_pose)
    {
        controlProgSize();
        addLog("New move to position: " + pose2Str(new_pose));

        if (speedMMS_ != config_.DEFAULT_SPEED) {
            addLine("// Speed adjusted to: " + to_string(speedMMS_) + " mm/s");
        }
    }


    void RobotPost::controlProgSize()
    {
        progMoveCount_++;
        if (progMoveCount_ >= config_.PROG_MOVE_COUNT_MAX) {
            closeModule();
        }
    }


    bool RobotPost::DirExists(const string& path)
    {
        struct stat info;
        return stat(path.c_str(), &info) == 0 && (info.st_mode & S_IFDIR);
    }

    bool RobotPost::CreateDir(const string& path)
    {
        if (DirExists(path)) {
            return true;
        }

        string current;
        for (char c : path) {
            current += c;
            if (c == '\\' || c == '/') {
                if (!DirExists(current) && mkdir(current.c_str()) != 0) {
                    return false;
                }
            }
        }

        return mkdir(path.c_str()) == 0 || DirExists(path);
    }


    // 修改closeModule方法 - 按照Python版本逻辑
    void RobotPost::closeModule()
    {
        if (progMoveCount_ == 0) {
            return;
        }

        string progname = currentProg_.name;
        size_t nprogs = progNames_.size();

        if (nprogs > 0) {
            progname = progname + to_string(nprogs + 1);
        }

        // 生成DTX内容 - 按照Python版本
        string dtx_data = DATA_DTX;

        // 正确替换所有占位符
        size_t pos = dtx_data.find("%s");
        if (pos != string::npos) {
            dtx_data.replace(pos, 2, frameData_);
            pos = dtx_data.find("%s", pos + 2);
            if (pos != string::npos) {
                dtx_data.replace(pos, 2, jointData_);
                pos = dtx_data.find("%s", pos + 2);
                if (pos != string::npos) {
                    dtx_data.replace(pos, 2, pointData_);
                    pos = dtx_data.find("%s", pos + 2);
                    if (pos != string::npos) {
                        dtx_data.replace(pos, 2, toolData_);
                    }
                }
            }
        }
        PROG_DTX_LIST.push_back(dtx_data);

        // 保存到程序列表
        progNames_.push_back(progname);
        progList_.push_back(currentProg_);

        // 重置当前程序
        currentProg_ = ProgramData(progname);
        PROG_PGX = ""; // 重置PGX内容

        // 关键：像Python版本一样重置所有数据变量和计数器！
        jointData_.clear();
        pointData_.clear();
        toolData_.clear();
        frameData_.clear();
        speedData_.clear();

        jointCount_ = 0;
        pointCount_ = 0;
        toolCount_ = 0;
        frameCount_ = 0;
        speedCount_ = 0;
        progMoveCount_ = 0;

        // 重新设置当前坐标系和工具 - 按照Python版本
        setFrame(currentFrame_);
        setTool(currentTool_);
        setSpeed(speedMMS_);
    }

    // 修改ProgSave方法 - 按照Python版本逻辑
    void RobotPost::ProgSave(const string& folder, const string& progname,
        bool askUser, const string& showResult)
    {
        string prognameLower = FilterNameS6(progname);
        transform(prognameLower.begin(), prognameLower.end(), prognameLower.begin(), ::tolower);

        if (progMoveCount_ > 0) {
            closeModule();
        }

        string foldersave = folder;

        // 处理文件夹选择 - 按照Python版本
        if (askUser || !DirExists(folder)) {
            // 这里需要实现getSaveFolder功能
            // foldersave = getSaveFolder(folder, "Save program as...");
            // 简化实现，直接使用提供的文件夹
        }

        size_t nprogs = progNames_.size();
        cout << "Saving " << nprogs << " programs..." << endl;

        string main_progname;
        string folderprog;

        // 多程序处理 - 按照Python版本
        if (nprogs > 1) {
            main_progname = "Main" + prognameLower;
            MAIN_FOLDER_ = main_progname;
            folderprog = foldersave + "/" + main_progname;
        }
        else {
            if (!LAST_PG_NAME.empty()) {
                main_progname = LAST_PG_NAME;
            }
            else {
                main_progname = prognameLower;
            }
            folderprog = foldersave + "/" + main_progname;
            MAIN_FOLDER_ = main_progname;
        }

        if (!CreateDir(folderprog)) {
            cerr << "Error creating directory: " << folderprog << endl;
            return;
        }

        vector<string> show_file_list;

        // 多程序的主程序处理 - 按照Python版本
        if (nprogs > 1) {
            string call_sequence;
            for (size_t i = 0; i < nprogs; i++) {
                call_sequence += "  if prog:libLoad(\"./" + progNames_[i] + "\")!=0\n";
                call_sequence += "    logMsg(\"Error Loading RobotSDK library\")\n";
                call_sequence += "    popUpMsg(\"Error Loading RobotSDK library\")\n";
                call_sequence += "  endIf\n";
                call_sequence += "  wait(taskStatus(\"loading\")==-1)\n";

                if (i < nprogs - 1) {
                    call_sequence += "  taskCreate \"loading\",10,loadNextOne(\"./" + progNames_[i + 1] + "\")\n";
                }

                call_sequence += "  prog:fPartReal.trsf=fPartCad.trsf*fCadToReal.trsf\n";
                call_sequence += "  prog:tCad.trsf=prog:tCad.trsf*{0,0,tooldata:nLength,0,0,0}\n";
                call_sequence += "  call prog:start()\n";
                call_sequence += "  \n";
            }

            // 保存主程序文件...
            string start_file = folderprog + "/start.pgx";
            ofstream fid_start(start_file, ios::out | ios::binary);
            if (fid_start.is_open()) {
                fid_start << "\xEF\xBB\xBF";
                string start_content = START_PGX;
                size_t pos = start_content.find("%s");
                if (pos != string::npos) {
                    start_content.replace(pos, 2, call_sequence);
                }
                fid_start << start_content;
                fid_start.close();
            }

            // 其余主程序保存代码...
        }

        // 保存所有程序文件...
        // 保存所有程序文件
        for (size_t i = 0; i < progNames_.size(); i++) {
            string folderprog_final;
            if (progNames_.size() > 1) {
                folderprog_final = folderprog + "/" + progNames_[i];
            }
            else {
                folderprog_final = folderprog;
            }

            if (!CreateDir(folderprog_final)) {
                cerr << "Error creating program directory: " << folderprog_final << endl;
                continue;
            }

            // 修改：传递模块索引
            saveModule(folderprog_final, progList_[i].lines, progNames_[i], i);
        }

        progFiles_.clear();
        progFiles_.push_back(folderprog);
    }

    // 添加缺失的工具函数实现
    string RobotPost::FilterNameS6(const string& name)
    {
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

    // 修改后的 saveModule 方法
    void RobotPost::saveModule(const string& folderprog_final,
        const vector<string>& prog_lines,
        const string& progname,
        size_t moduleIndex)
    {
        // 生成PGX内容
        string pgx_content;
        for (const auto& line : prog_lines) {
            pgx_content += line;
        }

        // 保存start.pgx - 使用字符串替换
        string start_file = folderprog_final + "/start.pgx";
        ofstream fid_start(start_file, ios::out | ios::binary);
        if (fid_start.is_open()) {
            fid_start << "\xEF\xBB\xBF";
            string start_content = START_PGX;
            size_t pos = start_content.find("%s");
            if (pos != string::npos) {
                start_content.replace(pos, 2, pgx_content);
            }
            fid_start << start_content;
            fid_start.close();
        }

        // 保存stop.pgx
        string stop_file = folderprog_final + "/stop.pgx";
        ofstream fid_stop(stop_file, ios::out | ios::binary);
        if (fid_stop.is_open()) {
            fid_stop << "\xEF\xBB\xBF";
            fid_stop << STOP_PGX;
            fid_stop.close();
        }

        // 保存项目文件 - 使用字符串替换
        string project_file = folderprog_final + "/" + progname + ".pjx";
        ofstream fid_pjx(project_file, ios::out | ios::binary);
        if (fid_pjx.is_open()) {
            fid_pjx << "\xEF\xBB\xBF";
            string project_content = PROGRAM_PJX;
            size_t pos = project_content.find("%s");
            if (pos != string::npos) {
                project_content.replace(pos, 2, progname);
            }
            fid_pjx << project_content;
            fid_pjx.close();
        }
        cout << "SAVED: " << project_file << endl;

        // 保存数据文件 - 使用对应模块的DTX数据
        if (moduleIndex < PROG_DTX_LIST.size()) {
            string program_data = folderprog_final + "/" + progname + ".dtx";
            ofstream fid_dtx(program_data, ios::out | ios::binary);
            if (fid_dtx.is_open()) {
                fid_dtx << "\xEF\xBB\xBF";
                fid_dtx << PROG_DTX_LIST[moduleIndex];
                fid_dtx.close();
            }
        }
        else {
            cerr << "Error: Module index " << moduleIndex << " out of range for DTX data." << endl;
        }

        // 保存ltx文件
        string ltx_file = folderprog_final + "/" + progname + ".ltx";
        ofstream fid_ltx(ltx_file, ios::out | ios::binary);
        if (fid_ltx.is_open()) {
            fid_ltx << "\xEF\xBB\xBF";
            fid_ltx << LTX_FILE;
            fid_ltx.close();
        }

        // 如果是多程序模式，保存loadNextOne.pgx
        if (progNames_.size() > 1) {
            string load_next_file = folderprog_final + "/loadNextOne.pgx";
            ofstream fid_load(load_next_file, ios::out | ios::binary);
            if (fid_load.is_open()) {
                fid_load << "\xEF\xBB\xBF";
                fid_load << LOAD_NEXT_ONE;
                fid_load.close();
            }
        }
    }


    // 修改后的 prog2List 方法
    void RobotPost::prog2List()
    {
        if (currentProg_.lines.empty()) {
            return;
        }

        // 确保关闭当前模块
        if (progMoveCount_ > 0) {
            closeModule();
        }
    }


    // 公共访问器实现
    const vector<string> RobotPost::getProgramLines() const
    {
        vector<string> allLines;
        for (const auto& prog : progList_) {
            allLines.insert(allLines.end(), prog.lines.begin(), prog.lines.end());
        }
        // 添加当前程序的代码行
        allLines.insert(allLines.end(), currentProg_.lines.begin(), currentProg_.lines.end());
        return allLines;
    }

    // 在文件末尾添加 test_post 函数实现
    void test_post(const std::string& path) {
        auto Pose = robomath::PosePP;

        std::cout << "Total instructions: 51" << std::endl;
        RobotPost robot("Staubli_S6", "TX200");

        //// 开始程序
        //robot.ProgStart("TestProgram");
        //robot.RunMessage("Program generated by hejun for Staubli S6", true);
        //
        //// 设置坐标系和工具
        //robot.setTool(Pose(62.5, -108.253175, 100, -60, 90, 0));
        //robot.MoveJ(Pose(200, 200, 500, 180, 0, 180), { -46.18419, -6.77518, -20.54925, 71.38674, 49.58727, -302.54752 });
        //robot.MoveL(Pose(200, 250, 348.734575, 180, 0, -150), { -41.62707, -8.89064, -30.01809, 60.62329, 49.66749, -258.98418 });
        //robot.MoveL(Pose(200, 200, 262.132034, 180, 0, -150), { -43.73892, -3.91728, -35.77935, 58.57566, 54.11615, -253.81122 });
        //robot.RunMessage("Setting air valve 1 on");
        //robot.RunCode("TCP_On", true);
        //robot.Pause(1000);
        //robot.MoveL(Pose(200, 250, 348.734575, 180, 0, -150), { -41.62707, -8.89064, -30.01809, 60.62329, 49.66749, -258.98418 });
        //robot.MoveL(Pose(250, 300, 278.023897, 180, 0, -150), { -37.52588, -6.32628, -34.59693, 53.52525, 49.24426, -251.44677 });
        //robot.MoveL(Pose(250, 250, 191.421356, 180, 0, -150), { -39.75778, -1.04537, -40.37883, 52.09118, 54.15317, -246.94403 });
        //robot.RunMessage("Setting air valve off");
        //robot.RunCode("TCP_Off", true);
        //robot.Pause(1000);
        //robot.MoveL(Pose(250, 300, 278.023897, 180, 0, -150), { -37.52588, -6.32628, -34.59693, 53.52525, 49.24426, -251.44677 });
        //robot.MoveL(Pose(250, 200, 278.023897, 180, 0, -150), { -41.85389, -1.95619, -34.89154, 57.43912, 52.34162, -253.73403 });
        //robot.MoveL(Pose(250, 150, 191.421356, 180, 0, -150), { -43.82111, 3.29703, -40.29493, 56.02402, 56.61169, -249.23532 });
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
        robot.MoveL(Pose(161.412, 284.537, 94.9997, 90, 0, -180), { -4.8691, 47.779, 77.928, 5.87482, 24.4962, -99.5685 });
        robot.ProgFinish("Curve");

        robot.ProgStart("Pause");
        robot.Pause(4000.0);
        robot.ProgFinish("Pause");
    
        // 输出程序内容
        std::cout << "\nGenerated program content:" << std::endl;
        std::cout << "==========================" << std::endl;

        for (const auto& prog : robot.progList_) {
            for (const auto& line : prog.lines) {
                std::cout << line << std::endl;
            }
            std::cout << std::endl << std::endl;
        }

        // 保存程序
        robot.ProgSave(path, "Curve", false, "");
        std::cout << "Program generation completed successfully!" << std::endl;
        std::cout << "Output saved to: " << path << std::endl;
    }
}