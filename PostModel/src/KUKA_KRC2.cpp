#include <PostModel/KUKA_KRC2.h>
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

namespace KUKA_KRC2{
    using namespace robomath;

    // 常量定义
    const std::string HEADER =
    "EXT BAS (BAS_COMMAND :IN,REAL :IN )\n"
    "\n"
    "GLOBAL INTERRUPT DECL 3 WHEN $STOPMESS==TRUE DO IR_STOPM ( )\n"
    "INTERRUPT ON 3\n"
    "\n"
    ";FOLD Initialise and set default speed\n"
    "BAS (#INITMOV,0)\n"
    "BAS (#VEL_PTP,100)\n"
    "BAS (#ACC_PTP,20)\n"
    "$VEL.CP=0.2\n"
    "BAS (#TOOL,0)\n"
    "BAS (#BASE,0)\n"
    ";ENDFOLD\n"
    "\n"
    ";;FOLD STARTPOS\n"
    ";$BWDSTART = FALSE\n"
    ";PDAT_ACT = PDEFAULT\n"
    ";BAS(#PTP_DAT)\n"
    ";FDAT_ACT = {TOOL_NO 0,BASE_NO 0,IPO_FRAME #BASE}\n"
    ";BAS(#FRAMES)\n"
    "; ENDFOLD\n"
    "\n"
    "$ADVANCE = 5\n"
    "\n"
    ";FOLD ---- Quickly skip BCO ----\n"
    "; PTP $AXIS_ACT \n"
    ";ENDFOLD\n"
    "\n"
    ";FOLD ---- GO HOME ----\n"
    "; PTP {A1 0.000, A2 -90.000, A3 90.000, A4 0.000, A5 0.000, A6 0.000, E1 0, E2 0, E3 0, E4 0, E5 0, E6 0}\n"
    ";ENDFOLD\n";

    const std::string CODE_TOOL_ON =
    ";FOLD TURN_ON WELDING\n"
    ";--- WELDING TURN_ON ---\n"
    "$OUT[1] = TRUE\n"
    ";---CONFIRM WELDING ON ---\n"
    "WAIT FOR $IN[1] == TRUE\n"
    ";--- PLASMA WELDING_ON ---\n"
    ";ENDFOLD\n";

    const std::string CODE_TOOL_OFF =
    ";FOLD TURN_OFF WELDING\n"
    ";--- WELDING TURN_OFF ---\n"
    "$OUT[1] = FALSE\n"
    ";---CONFIRM WELDING OFF ---\n"
    "WAIT FOR $IN[1] == FALSE\n"
    "WAIT SEC 1\n"
    ";--- WELDING TURN_FALSE ---\n"
    ";ENDFOLD\n";

    const std::vector<std::string> AXES_DATA = { "A1","A2","A3","A4","A5","A6","E1","E2","E3","E4","E5","E6" };

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
        nLines_ = 0;
        tab_ = "";
        PROG_ = "";
        LOG_ = "";
        APO_VALUE_ = 0.0;
        C_DIS_ = "";
        C_PTP_ = "";
        SPEED_MMS_ = 500.0;
        ARC_ON_ = false;

        PRINT_POSE_LAST_ = Mat();
        PRINT_E_LAST_ = 0.0;
        PRINT_E_NEW_ = 0.0;
        PRINT_E_NEW_MOTOR_ = 0.0;
        PRINT_LAST_SIGNAL_ = 0.0;

        progList_.clear();
        progNames_.clear();
        progFiles_.clear();
        PROG_CALLS_.clear();
        PROG_CALLS_LIST_.clear();
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

        std::string filename = progname + "." + config_.PROG_EXT;
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
        addLine("PTP {" + angles_2_str(joints) + "}" + C_PTP_);
    }

    void RobotPost::MoveL(const Mat& pose, const std::vector<double>& joints,
        const std::vector<double>& confRLF) {
        new_move(pose);

        std::string str_config = "";
        if (config_.ADD_CONFIG_TURN) {
            str_config = " S " + conf_2_str(confRLF) + ", T " + joints_2_turn_str(joints);
        }

        addLine("LIN {" + pose_2_str_ext(pose, joints) + str_config + "}" + C_DIS_);
    }

    void RobotPost::MoveC(const Mat& pose1, const std::vector<double>& joints1,
        const Mat& pose2, const std::vector<double>& joints2,
        const std::vector<double>& confRLF1, const std::vector<double>& confRLF2) {
        new_move(pose2);

        std::string str_config1 = "";
        std::string str_config2 = "";
        if (config_.ADD_CONFIG_TURN) {
            str_config1 = " S " + conf_2_str(confRLF1) + ", T " + joints_2_turn_str(joints1);
            str_config2 = " S " + conf_2_str(confRLF2) + ", T " + joints_2_turn_str(joints2);
        }

        addLine("CIRC {" + pose_2_str_ext(pose1, joints1) + str_config1 + "},{" +
            pose_2_str_ext(pose2, joints2) + str_config2 + "}" + C_DIS_);
    }

    // 设置函数
    void RobotPost::setFrame(const Mat& pose, int frameId, const std::string& frameName) {
        if (robotAxes_ > 6) {
            addLine("; Using external axes");
            addLine("; $BASE=EK(EX_AX_DATA[1].ROOT,EX_AX_DATA[1].EX_KIN,EX_AX_DATA[1].OFFSET)");
            addLine("; $ACT_EX_AX= " + std::to_string(robotAxes_ - 6));
        }

        if (frameName.find("Base") != std::string::npos) {
            addLine("$BASE = {FRAME: " + pose_2_str(pose) + "}");
            addLine("; BASE_DATA[0] = {FRAME: " + pose_2_str(pose) + "}");
            addLine("; $BASE = BASE_DATA[0]");
        }
        else if (frameId >= 0) {
            addLine("$BASE = {FRAME: " + pose_2_str(pose) + "}");
            addLine("; BASE_DATA[" + std::to_string(frameId) + "] = {FRAME: " + pose_2_str(pose) + "}");
            addLine("; $BASE = BASE_DATA[" + std::to_string(frameId) + "]");
        }
        else {
            addLine("$BASE = {FRAME: " + pose_2_str(pose) + "}");
        }
    }

    void RobotPost::setTool(const Mat& pose, int toolId, const std::string& toolName) {
        if (toolId >= 0) {
            addLine("$TOOL = {FRAME: " + pose_2_str(pose) + "}");
            addLine("; TOOL_DATA[" + std::to_string(toolId) + "] = {FRAME: " + pose_2_str(pose) + "}");
            addLine("; $TOOL = TOOL_DATA[" + std::to_string(toolId) + "]");
        }
        else {
            addLine("$TOOL = {FRAME: " + pose_2_str(pose) + "}");
        }
    }

    void RobotPost::setSpeed(double speed_mms) {
        SPEED_MMS_ = speed_mms;
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
            ss1 << "$APO.CPTP = " << std::fixed << std::setprecision(3) << ((100.0<zone_mm)?100.0:zone_mm);
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
        std::string var_name = io_var;
        if (std::isdigit(io_var[0])) {
            var_name = "$ANOUT[" + io_var + "]";
        }
        setDO(var_name, io_value);
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
            std::string code_lower = code;
            std::transform(code_lower.begin(), code_lower.end(), code_lower.begin(), ::tolower);

            if (code_lower.find("extruder(") == 0) {
                // 处理挤出机命令
                size_t start = code.find('(') + 1;
                size_t end = code.find(')');
                if (start != std::string::npos && end != std::string::npos) {
                    std::string value_str = code.substr(start, end - start);
                    double e_new = std::stod(value_str);

                    if (e_new > 0.0 && !ARC_ON_) {
                        addLine(CODE_TOOL_ON);
                        ARC_ON_ = true;
                    }
                    else if (e_new <= 0.0 && ARC_ON_) {
                        addLine(CODE_TOOL_OFF);
                        ARC_ON_ = false;
                    }

                    PRINT_E_NEW_ = e_new;
                    PRINT_E_NEW_MOTOR_ += (0.0>(PRINT_E_NEW_- PRINT_E_LAST_))? 0.0:(PRINT_E_NEW_ - PRINT_E_LAST_);
                }
                return;
            }
            else if (code_lower == "arc_on") {
                if (!ARC_ON_) {
                    ARC_ON_ = true;
                    addLine(CODE_TOOL_ON);
                }
                return;
            }
            else if (code_lower == "arc_off") {
                if (ARC_ON_) {
                    ARC_ON_ = false;
                    addLine(CODE_TOOL_OFF);
                }
                return;
            }

            std::string code_safe = code;
            std::replace(code_safe.begin(), code_safe.end(), ' ', '_');

            // 检查字符串是否以")"结尾
            if (code_safe.size() == 0 || code_safe[code_safe.size() - 1] != ')') {
                code_safe += "()";
            }

            // 提取函数名
            std::string fcn_def = code_safe;
            size_t paren_pos = fcn_def.find('(');
            if (paren_pos != std::string::npos) {
                fcn_def = fcn_def.substr(0, paren_pos);
            }

            // 检查是否已存在
            if (std::find(PROG_CALLS_.begin(), PROG_CALLS_.end(), fcn_def) == PROG_CALLS_.end()) {
                PROG_CALLS_.push_back(fcn_def);
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
        nLines_++;
    }

    void RobotPost::addLog(const std::string& newline) {
        LOG_ = LOG_ + newline + "\n";
    }

    std::string RobotPost::pose_2_str(const Mat& pose) {
        // 简化的Pose到KUKA格式转换
        std::vector<double> xyzrpw = pose_2_xyzrpw(pose);
        std::stringstream ss;
        ss << "X " << std::fixed << std::setprecision(3) << xyzrpw[0]
            << ",Y " << xyzrpw[1]
            << ",Z " << xyzrpw[2]
            << ",A " << xyzrpw[3]
            << ",B " << xyzrpw[4]
            << ",C " << xyzrpw[5];
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
                ss << "," << AXES_DATA[i + 6] << " " << std::fixed << std::setprecision(5) << joints[i + 6];
                extaxes += ss.str();
            }
            return pose_2_str(pose) + extaxes;
        }
    }

    std::string RobotPost::angles_2_str(const std::vector<double>& angles) {
        std::stringstream ss;
        if (config_.KRC_VERSION >= 4) 
        {
            ss << "AXIS: ";
        }

        for (size_t i = 0; i < angles.size() && i < AXES_DATA.size(); ++i) {
            ss << AXES_DATA[i] << " " << std::fixed << std::setprecision(5) << angles[i] << ",";
        }

        std::string result = ss.str();
        if (!result.empty()) {
            result.pop_back(); // 移除最后一个逗号
        }
        return result;
    }

    std::string RobotPost::conf_2_str(const std::vector<double>& confRLF) {
        if (confRLF.empty() || confRLF.size() < 3) {
            return "'B010'";
        }

        std::string strconf = "";
        if (confRLF[2] > 0) {
            strconf += '1';
        }
        else {
            strconf += '0';
        }

        if (confRLF[1] == 0) {
            strconf += '1';
        }
        else {
            strconf += '0';
        }

        if (confRLF[0] > 0) {
            strconf += '1';
        }
        else {
            strconf += '0';
        }

        return "'B" + strconf + "'";
    }

    std::string RobotPost::joints_2_turn_str(const std::vector<double>& joints) {
        if (joints.empty()) {
            return "'B000000'";
        }

        std::string strturn = "";
        for (double joint : joints) {
            if (joint < 0) {
                strturn = '1' + strturn;
            }
            else {
                strturn = '0' + strturn;
            }
        }
        return "'B" + strturn + "'";
    }

    void RobotPost::new_move(const Mat& new_pose) {
        // 3D打印相关逻辑，这里简化实现
        if (PRINT_E_NEW_ <= 0 || (new_pose== Mat()))
        {
            return;
        }

        // 记住最后一个位姿
        PRINT_POSE_LAST_ = new_pose;
    }

    double RobotPost::calculate_time(double distance, double Vmax, double Amax) {
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

    // 测试函数
    void test_post(std::string path) {
        auto Pose = robomath::PosePP;
        //"Test the post with a basic program"
        RobotPost robot("Kuka_KRC2", "Generic Kuka");
        robot.ProgStart("Program1");
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
        robot.ProgFinish("Program1");

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

        robot.ProgSave(path, "Program1", false, "");
        std::cout << "Program generation completed successfully!" << std::endl;
    }
}