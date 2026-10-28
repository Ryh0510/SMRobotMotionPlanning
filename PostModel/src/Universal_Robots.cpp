#include <PostModel/Universal_Robots.h>
#include <cmath>
#include <algorithm>
#include <cctype>
#include <cstring>
#include <PostModel/robomath.h>
#include <PostModel/ftpsend.h>


#pragma execution_character_set("utf-8")

namespace Universal_Robots{
    using namespace std;

    #ifndef M_PI
    #define M_PI 3.14159265358979323846
    #endif
    // 构造函数
    RobotPost::RobotPost(const std::string robotPost, const std::string robotName,
        int robotAxes,
        std::vector<std::string> axesType,
        const std::string ip_com,
        const Config config)
        : robotPost_(robotPost),
        robotName_(robotName),
        robotAxes_(robotAxes),
        axesType_(axesType),
        ip_com_(ip_com),
        config_(config)
    {
        // 初始化所有成员变量
        nPROGS_ = 0;
        TAB_ = "";
        SPEED_MS_ = config.SPEED_MS;
        SPEED_RADS_ = config.SPEED_RADS;
        ACCEL_MSS_ = config.ACCEL_MSS;
        ACCEL_RADSS_ = config.ACCEL_RADSS;
        BLEND_RADIUS_M_ = config.BLEND_RADIUS_M;
        MAX_LINES_X_PROG_ = config.MAX_LINES_X_PROG;
        MAX_SPEED_DEGS_ = config.MAX_SPEED_DEGS;
        MAX_ACCEL_DEGSS_ = config.MAX_ACCEL_DEGSS;
        BLEND_RADIUS_ = "";
        REF_FRAME_ = robomath::eye(4);
        LAST_POS_ABS_ = robomath::Mat();
        LAST_DIR_ = robomath::Mat();
        PROG_EXT_ = "script";
        MAIN_PROGNAME_ = "unknown";
        NATIVE_NAME_ = robotName;

        // 根据机器人类型设置参数
        if (NATIVE_NAME_ == "UR3" || NATIVE_NAME_ == "UR3e") {
            MAX_SPEED_DEGS_ = 360;
            MAX_ACCEL_DEGSS_ = 4583.7;
        }
        else {
            MAX_SPEED_DEGS_ = config.MAX_SPEED_DEGS;
            MAX_ACCEL_DEGSS_ = config.MAX_ACCEL_DEGSS;
        }

        // 设置混合检查
        if (NATIVE_NAME_ == "UR3e" || NATIVE_NAME_ == "UR5e" ||
            NATIVE_NAME_ == "UR10e" || NATIVE_NAME_ == "UR16e" ||
            NATIVE_NAME_ == "UR20") {
            BLENDING_CHECK_ = false;
        }
        else if (NATIVE_NAME_ == "UR3" || NATIVE_NAME_ == "UR5" ||
            NATIVE_NAME_ == "UR10") {
            BLENDING_CHECK_ = true;
        }
        else {
            BLENDING_CHECK_ = config.BLENDING_CHECK;
        }
    }

    // 程序控制
    void RobotPost::ProgStart(const std::string& progname) {
        std::string filteredName = FilterName(progname);
        nPROGS_++;
        if (nPROGS_ <= 1) {
            TAB_ = "";
            vars_update();
            MAIN_PROGNAME_ = filteredName;
        }
        else {
            addLine("# Subprogram " + filteredName);
            addLine("def " + filteredName + "():");
            TAB_ = "  ";
        }
    }

    void RobotPost::ProgFinish(const std::string& progname) {
        std::string filteredName = FilterName(progname);
        TAB_ = "";
        if (nPROGS_ <= 1) {
            addLine("# End of main program");
        }
        else {
            addLine("end");
            addLine("");
        }
    }

    void RobotPost::ProgSave(const std::string& folder, const std::string& progname,
        bool askUser, const std::string& showResult) {
        std::string filteredName = FilterName(progname);
        std::string filesave = folder + "/" + filteredName + "." + PROG_EXT_;

        prog_2_list();

        std::ofstream fid(filesave);
        if (fid.is_open()) {
            fid << "def " << MAIN_PROGNAME_ << "():\n";
            fid << "  # Global parameters:\n";
            for (const auto& line : VARS_LIST_[0]) {
                fid << "  " << line << "\n";
            }
            fid << "  ";

            if (config_.INCLUDE_HEADER) {
                fid << HEADER_;
                fid << "  \n";
            }

            for (const auto& line : SUBPROG_) {
                fid << "  " << line << "\n";
            }
            fid << "  \n";
            fid << "  # Main program:\n";

            for (const auto& prog : PROG_LIST_) {
                for (const auto& line : prog) {
                    fid << "  " << line << "\n";
                }
            }
            fid << "end\n\n";
            fid << MAIN_PROGNAME_ << "()\n";
            fid.close();
        }

        // 生成URP文件
        std::string filesave_urp = folder + "/" + filteredName;

        // 读取刚刚保存的script文件内容
        std::ifstream scriptFile(filesave);
        std::stringstream buffer;
        buffer << scriptFile.rdbuf();
        std::string prog_final = buffer.str();
        scriptFile.close();

        // 转义HTML特殊字符
        std::string prog_final_ok;
        for (char c : prog_final) {
            switch (c) {
            case '&': prog_final_ok += "&amp;"; break;
            case '<': prog_final_ok += "&lt;"; break;
            case '>': prog_final_ok += "&gt;"; break;
            case '"': prog_final_ok += "&quot;"; break;
            case '\'': prog_final_ok += "&#39;"; break;
            default: prog_final_ok += c; break;
            }
        }

        // 创建URP XML内容
        std::stringstream urp_xml;
        urp_xml << R"(<URProgram createdIn="3.0.0" lastSavedIn="3.0.0" name=")"
            << MAIN_PROGNAME_ << R"(" directory="." installation="default">)"
            << R"(<children><MainProgram runOnlyOnce="true" motionType="MoveJ" speed="1.0471975511965976" acceleration="1.3962634015954636" useActiveTCP="false">)"
            << R"(<children><Script type="File"><cachedContents>)"
            << prog_final_ok << R"(</cachedContents><file>)"
            << MAIN_PROGNAME_ << ".script</file></Script></children></MainProgram></children></URProgram>";

        // 保存为gzip压缩的URP文件
        std::string urp_filename = filesave_urp + ".urp";
        std::ofstream urp_file(urp_filename, std::ios::binary);
        if (urp_file.is_open()) {
            std::string xml_content = urp_xml.str();

            // 直接写入XML内容（不压缩）
            urp_file.write(xml_content.c_str(), xml_content.length());
            urp_file.close();

            std::cout << "SAVED URP: " << urp_filename << std::endl;
            PROG_FILES_.push_back(urp_filename);
        }
        else {
            std::cerr << "Failed to create URP file: " << urp_filename << std::endl;
        }
        std::cout << "SAVED: " << filesave << std::endl;
        PROG_FILES_.push_back(filesave);
    }

    void RobotPost::ProgSendRobot(const std::string& remote_path,
        const std::string& ftp_user, const std::string& ftp_pass) {
        // 实现程序发送到机器人的逻辑
        if (PROG_FILES_.empty()) {
            cerr << "No program files to send. Please save program first." << endl;
            return;
        }

        cout << "Sending programs to robot " << ip_com_ << endl;
        bool success = UploadFTP(PROG_FILES_[0], ip_com_, remote_path, ftp_user, ftp_pass);

        if (success) {
            cout << "Upload successful!" << endl;
        }
        else {
            cout << "Upload failed!" << endl;
        }
    }

    // 运动指令
    void RobotPost::MoveJ(const robomath::Mat& pose, const std::vector<double>& joints,const std::vector<double>& confRLF) {
        if (pose == robomath::Mat()) {
            BLEND_RADIUS_ = "0";
            LAST_POS_ABS_ = robomath::Mat();
            LAST_DIR_ = robomath::Mat();
        }
        else {
            robomath::Mat pose_abs = REF_FRAME_ * pose;
            if (BLENDING_CHECK_) {
                BLEND_RADIUS_ = blend_radius_check(pose_abs);
            }
            else {
                BLEND_RADIUS_ = std::to_string(BLEND_RADIUS_M_);
            }
            LAST_POS_ABS_ = pose_abs;
        }

        if (joints.size() < 6) {
            RunMessage("Move axes to: " + angles_2_str(joints), true);
        }
        else {
            addLine("movej(" + angles_2_str(joints) + ",accel_radss,speed_rads,0," + BLEND_RADIUS_ + ")");
        }
    }

    void RobotPost::MoveL(const robomath::Mat& pose, const std::vector<double>& joints,
        const std::vector<double>& confRLF) {
        if (pose == robomath::Mat()) {
            BLEND_RADIUS_ = "0";
            LAST_POS_ABS_ = robomath::Mat();
            std::string target = angles_2_str(joints);
        }
        else {
            robomath::Mat pose_tgt;
            if (!config_.USE_RELATIVE_TARGETS) {
                robomath::Mat pose_abs = REF_FRAME_ * pose;
                pose_tgt = pose_abs;
            }
            else {
                robomath::Mat pose_abs = REF_FRAME_ * pose;
                pose_tgt = pose;
            }

            if (BLENDING_CHECK_) {
                BLEND_RADIUS_ = blend_radius_check(pose_tgt);
            }
            else {
                BLEND_RADIUS_ = std::to_string(BLEND_RADIUS_M_);
            }
            LAST_POS_ABS_ = pose_tgt;

            std::string target = pose_2_str(pose_tgt);
            if (config_.USE_RELATIVE_TARGETS) {
                target = "pose_trans(ref_frame," + target + ")";
            }

            if (config_.USE_MOVEP) {
                addLine("movep(" + target + ",accel_mss,speed_ms," + BLEND_RADIUS_ + ")");
            }
            else {
                addLine("movel(" + target + ",accel_mss,speed_ms,0," + BLEND_RADIUS_ + ")");
            }
        }
    }

    void RobotPost::MoveC(const robomath::Mat& pose1, const std::vector<double>& joints1,
        const robomath::Mat& pose2, const std::vector<double>& joints2,
        const std::vector<double>& confRLF1,
        const std::vector<double>& confRLF2) {
        if (pose1 == robomath::Mat() || pose2 == robomath::Mat()) {
            std::string msg = "Circular movement using joint targets is not supported. Change the target type to cartesian or use a joint movement.";
            addLog(msg);
            RunMessage(msg, true);
            return;
        }

        robomath::Mat pose1_abs = REF_FRAME_ * pose1;
        robomath::Mat pose2_abs = REF_FRAME_ * pose2;

        if (LAST_POS_ABS_ == robomath::Mat()) {
            MoveL(pose2, joints2, confRLF2);
            return;
        }

        double radius = circle_radius(LAST_POS_ABS_, pose1_abs, pose2_abs);
        std::cout << "MoveC Radius: " << radius << " mm" << std::endl;

        if (radius < config_.MOVEC_MIN_RADIUS || radius > config_.MOVEC_MAX_RADIUS) {
            MoveL(pose2, joints2, confRLF2);
            return;
        }

        BLEND_RADIUS_ = std::to_string(BLEND_RADIUS_M_);
        LAST_POS_ABS_ = pose2_abs;

        std::string target1, target2;
        if (config_.USE_RELATIVE_TARGETS) {
            target1 = "pose_trans(ref_frame," + pose_2_str(pose1) + ")";
            target2 = "pose_trans(ref_frame," + pose_2_str(pose2) + ")";
        }
        else {
            target1 = pose_2_str(pose1_abs);
            target2 = pose_2_str(pose2_abs);
        }

        addLine("movec(" + target1 + "," + target2 + ",accel_mss,speed_ms," + BLEND_RADIUS_ + ")");
    }

    // 设置函数
    void RobotPost::setFrame(const robomath::Mat& pose, int frameId, const std::string& frameName) {
        REF_FRAME_ = pose;
        if (config_.USE_RELATIVE_TARGETS) {
            addLine("ref_frame = " + pose_2_str(pose));
        }
        else {
            addLine("# set_reference(" + pose_2_str(pose) + ")");
        }
    }

    void RobotPost::setTool(const robomath::Mat& pose, int toolId, const std::string& toolName) {
        addLine("set_tcp(" + pose_2_str(pose) + ")");
    }

    void RobotPost::setSpeed(double speed_mms) {
        double new_speed = (config_.MAX_SPEED_MS<(speed_mms / 1000.0))? config_.MAX_SPEED_MS:(speed_mms / 1000.0);
        if (std::abs(SPEED_MS_ - new_speed) < 1e-06) {
            return;
        }
        make_last_move_accurate();
        SPEED_MS_ = new_speed;
        addLine("speed_ms    = " + std::to_string(SPEED_MS_));
    }

    void RobotPost::setAcceleration(double accel_mmss) {
        ACCEL_MSS_ = (config_.MAX_ACCEL_MSS<(accel_mmss / 1000.0))? config_.MAX_ACCEL_MSS:(accel_mmss / 1000.0);
        addLine("accel_mss   = " + std::to_string(ACCEL_MSS_));
    }

    void RobotPost::setSpeedJoints(double speed_degs) {
        SPEED_RADS_ = ((MAX_SPEED_DEGS_<speed_degs)? MAX_SPEED_DEGS_ : speed_degs)* M_PI / 180;
        addLine("speed_rads  = " + std::to_string(SPEED_RADS_));
    }

    void RobotPost::setAccelerationJoints(double accel_degss) {
        ACCEL_RADSS_ = ((MAX_ACCEL_DEGSS_< accel_degss)? MAX_ACCEL_DEGSS_:accel_degss) * M_PI / 180;
        addLine("accel_radss = " + std::to_string(ACCEL_RADSS_));
    }

    void RobotPost::setZoneData(double zone_mm) {
        if (zone_mm < 0) {
            zone_mm = 0;
        }
        BLEND_RADIUS_M_ = zone_mm / 1000.0;
        addLine("blend_radius_m = " + std::to_string(BLEND_RADIUS_M_));
    }

    // I/O 操作
    void RobotPost::setDO(const std::string& io_var, const std::string& io_value) {
        make_last_move_accurate();
        std::string newline;
        try {
            int io_var_int = std::stoi(io_var);
            std::string io_value_str = io_value;
            if (io_value != "true" && io_value != "false") {
                try {
                    int io_value_int = std::stoi(io_value);
                    io_value_str = (io_value_int > 0) ? "True" : "False";
                }
                catch (...) {
                    io_value_str = "False";
                }
            }

            if (io_var_int < 10) {
                newline = "set_standard_digital_out(" + io_var + ", " + io_value_str + ")";
            }
            else if (io_var_int < 20) {
                int io_var_adj = io_var_int - 10;
                newline = "set_tool_digital_out(" + std::to_string(io_var_adj) + ", " + io_value_str + ")";
            }
            else {
                int io_var_adj = io_var_int - 20;
                newline = "set_configurable_digital_out(" + std::to_string(io_var_adj) + ", " + io_value_str + ")";
            }
        }
        catch (...) {
            newline = io_var + " = " + io_value;
        }
        addLine(newline);
    }

    void RobotPost::setAO(const std::string& io_var, const std::string& io_value) {
        make_last_move_accurate();
        std::string newline;
        try {
            int io_var_int = std::stoi(io_var);
            newline = "set_standard_analog_out(" + io_var + ", " + io_value + ")";
        }
        catch (...) {
            newline = io_var + " = " + io_value;
        }
        addLine(newline);
    }

    void RobotPost::waitDI(const std::string& io_var, const std::string& io_value, int timeout_ms) {
        make_last_move_accurate();
        std::string io_var_adj = io_var;
        std::string io_value_adj = io_value;

        try {
            int io_var_int = std::stoi(io_var);
            if (io_var_int < 10) {
                io_var_adj = "get_standard_digital_in(" + io_var + ")";
            }
            else if (io_var_int < 20) {
                int io_var_adj_int = io_var_int - 10;
                io_var_adj = "get_tool_digital_in(" + std::to_string(io_var_adj_int) + ")";
            }
            else {
                int io_var_adj_int = io_var_int - 20;
                io_var_adj = "get_configurable_digital_in(" + std::to_string(io_var_adj_int) + ")";
            }

            if (io_value != "true" && io_value != "false") {
                try {
                    int io_value_int = std::stoi(io_value);
                    io_value_adj = (io_value_int > 0) ? "True" : "False";
                }
                catch (...) {
                    io_value_adj = "False";
                }
            }
        }
        catch (...) {
            // 保持原样
        }

        addLine("while (" + io_var_adj + " != " + io_value_adj + "):");
        addLine("  sync()");
        addLine("end");
    }

    // 代码生成
    void RobotPost::RunCode(const std::string& code, bool is_function_call) {
        make_last_move_accurate();
        if (is_function_call) {
            std::string code_processed = code;
            size_t idx_fcn = code.find('(');
            if (idx_fcn != std::string::npos) {
                code_processed = FilterName(code.substr(0, idx_fcn)) + code.substr(idx_fcn);
            }
            else {
                code_processed = FilterName(code);
            }

            if (code_processed == "usemovel") {
                config_.USE_MOVEP = false;
            }
            else if (code_processed == "usemovep") {
                config_.USE_MOVEP = true;
            }
            else {
                if (code_processed.find('(') == std::string::npos) {
                    code_processed += "()";
                }
                addLine(code_processed);
            }
        }
        else if (code.find('\n') == std::string::npos) {
            std::string code_filtered = FilterBackSlash(code);
            addLine(code_filtered);
        }
        else {
            std::istringstream iss(code);
            std::string line;
            while (std::getline(iss, line)) {
                std::string line_filtered = FilterBackSlash(line);
                addLine(line_filtered);
            }
        }
    }

    void RobotPost::RunMessage(const std::string& message, bool iscomment) {
        if (iscomment) {
            addLine("# " + message);
        }
        else {
            addLine("popup(\"" + message + "\",\"Message\",False,False,blocking=False)");
        }
    }

    void RobotPost::Pause(int time_ms) {
        if (time_ms <= 0) {
            addLine("halt() # reimplement this function to force stop");
        }
        else {
            addLine("sleep(" + std::to_string(time_ms * 0.001) + ")");
        }
    }

    // 工具函数
    std::string RobotPost::FilterName(const std::string& namefilter, char safechar,
        const std::vector<std::string>& reserved_names) {
        std::string result = namefilter;
        result.erase(std::remove_if(result.begin(), result.end(),
            [](char c) { return !std::isalnum(c); }), result.end());

        if (result.empty()) {
            result = std::string(1, safechar);
        }

        return result;
    }

    std::string RobotPost::pose_2_str(const robomath::Mat& pose) {
        std::vector<double> xyzrpw = robomath::Pose_2_Staubli(pose);
        std::stringstream ss;
        ss << std::fixed << std::setprecision(6);
        ss << "p[" << xyzrpw[0] * 0.001 << ", " << xyzrpw[1] * 0.001 << ", " << xyzrpw[2] * 0.001
            << ", " << xyzrpw[3] << ", " << xyzrpw[4] << ", " << xyzrpw[5] << "]";
        return ss.str();
    }

    std::string RobotPost::angles_2_str(const std::vector<double>& angles) {
        if (angles.size() != 6) {
            return "this post only supports 6 joints";
        }

        std::stringstream ss;
        ss << std::fixed << std::setprecision(6);
        ss << "[" << angles[0] * M_PI / 180.0 << ", " << angles[1] * M_PI / 180.0 << ", "
            << angles[2] * M_PI / 180.0 << ", " << angles[3] * M_PI / 180.0 << ", "
            << angles[4] * M_PI / 180.0 << ", " << angles[5] * M_PI / 180.0 << "]";
        return ss.str();
    }

    std::string RobotPost::extAxes2Str(const std::vector<double>& angles,
        const std::vector<double>& ratio) {
        std::stringstream ss;
        ss << "[";
        for (size_t i = 0; i < angles.size(); ++i) {
            if (i > 0) ss << ",";
            ss << angles[i];
        }
        for (size_t i = angles.size(); i < 6; ++i) {
            ss << ",9E+09";
        }
        ss << "]";
        return ss.str();
    }

    // 公共访问器
    const std::vector<std::string> RobotPost::getProgramLines() const {
        std::vector<std::string> allLines;
        for (const auto& prog : PROG_LIST_) {
            allLines.insert(allLines.end(), prog.begin(), prog.end());
        }
        allLines.insert(allLines.end(), PROG_.begin(), PROG_.end());
        return allLines;
    }

    // 私有方法
    void RobotPost::addLine(const std::string& newline) {
        if (nPROGS_ <= 1) {
            if (PROG_.size() > MAX_LINES_X_PROG_) {
                prog_2_list();
            }
            PROG_.push_back(TAB_ + newline);
        }
        else {
            SUBPROG_.push_back(TAB_ + newline);
        }
    }

    void RobotPost::addLog(const std::string& newline) {
        LOG_ += newline + "\n";
    }

    std::string RobotPost::blend_radius_check(const robomath::Mat& pose_abs, double ratio_check) {
        std::string blend_radius = "blend_radius_m";

        if (LAST_POS_ABS_ == robomath::Mat()) {
            blend_radius = "0";
            return blend_radius;
        }

        // 简化的实现，实际需要更复杂的几何计算
        // 这里返回默认的混合半径值
        return std::to_string(BLEND_RADIUS_M_);
    }

    double RobotPost::circle_radius(const robomath::Mat& p0, const robomath::Mat& p1, const robomath::Mat& p2) {
        // 简化的实现，返回一个默认值
        // 实际需要计算三点确定的圆的半径
        return 100.0;
    }

    void RobotPost::make_last_move_accurate() {
        if (!config_.MOVE_ACCURATE_CALLS || BLEND_RADIUS_M_ <= 0) {
            return;
        }

        if (nPROGS_ <= 1) {
            if (!PROG_.empty() && PROG_.back().find("move") != std::string::npos) {
                std::string lastline = PROG_.back();
                size_t id_last = lastline.rfind(',');
                if (id_last != std::string::npos) {
                    PROG_.back() = lastline.substr(0, id_last) + ",0) # end trace";
                }
            }
        }
        else if (!SUBPROG_.empty()) {
            if (SUBPROG_.back().find("move") != std::string::npos) {
                std::string lastline = SUBPROG_.back();
                size_t id_last = lastline.rfind(',');
                if (id_last != std::string::npos) {
                    SUBPROG_.back() = lastline.substr(0, id_last) + ",0) # end trace";
                }
            }
        }
    }

    void RobotPost::vars_update() {
        VARS_.clear();
        VARS_.push_back("global speed_ms    = " + std::to_string(SPEED_MS_));
        VARS_.push_back("global speed_rads  = " + std::to_string(SPEED_RADS_));
        VARS_.push_back("global accel_mss   = " + std::to_string(ACCEL_MSS_));
        VARS_.push_back("global accel_radss = " + std::to_string(ACCEL_RADSS_));
        VARS_.push_back("global blend_radius_m = " + std::to_string(BLEND_RADIUS_M_));
        if (config_.USE_RELATIVE_TARGETS) {
            VARS_.push_back("global ref_frame = p[0,0,0,0,0,0]");
        }
    }

    void RobotPost::prog_2_list() {
        if (PROG_.size() > 1) {
            PROG_LIST_.push_back(PROG_);
            PROG_.clear();
            VARS_LIST_.push_back(VARS_);
            VARS_.clear();
            vars_update();
        }
    }

    std::string RobotPost::FilterBackSlash(const std::string& code) {
        std::string result = code;
        size_t pos = 0;
        while ((pos = result.find("\\", pos)) != std::string::npos) {
            result.erase(pos, 1);
        }
        return result;
    }

    // While循环控制
    void RobotPost::EnterWhile(const std::vector<int>& conditionPair) {
        make_last_move_accurate();

        if (conditionPair.size() < 2) {
            addLine("# Error: EnterWhile requires a condition pair [lhs, rhs]");
            return;
        }

        int lhs = conditionPair[0];
        int rhs = conditionPair[1];

        std::string lhs_str;
        std::string rhs_str;

        // 处理左侧条件
        if (lhs < 10) {
            lhs_str = "get_standard_digital_in(" + std::to_string(lhs) + ")";
        }
        else if (lhs < 20) {
            int lhs_adj = lhs - 10;
            lhs_str = "get_tool_digital_in(" + std::to_string(lhs_adj) + ")";
        }
        else {
            int lhs_adj = lhs - 20;
            lhs_str = "get_configurable_digital_in(" + std::to_string(lhs_adj) + ")";
        }

        // 处理右侧条件
        if (rhs > 0) {
            rhs_str = "True";
        }
        else {
            rhs_str = "False";
        }

        addLine("while (" + lhs_str + " != " + rhs_str + "):");
        TAB_ += "  ";
    }

    void RobotPost::ExitWhile() {
        if (TAB_.size() >= 2) {
            TAB_ = TAB_.substr(0, TAB_.size() - 2);
        }
        addLine("end");
    }

    // If条件控制
    void RobotPost::EnterIf(const std::vector<int>& conditionPair) {
        make_last_move_accurate();

        if (conditionPair.size() < 2) {
            addLine("//Error: EnterIf requires a condition pair {lhs, rhs}");
            return;
        }

        int lhs = conditionPair[0];
        int rhs = conditionPair[1];

        std::string lhs_str;
        std::string rhs_str;

        // 处理左侧条件
        if (lhs < 10) {
            lhs_str = "get_standard_digital_in(" + std::to_string(lhs) + ")";
        }
        else if (lhs < 20) {
            int lhs_adj = lhs - 10;
            lhs_str = "get_tool_digital_in(" + std::to_string(lhs_adj) + ")";
        }
        else {
            int lhs_adj = lhs - 20;
            lhs_str = "get_configurable_digital_in(" + std::to_string(lhs_adj) + ")";
        }

        // 处理右侧条件
        if (rhs > 0) {
            rhs_str = "True";
        }
        else {
            rhs_str = "False";
        }

        addLine("if (" + lhs_str + " != " + rhs_str + "):");
        TAB_ += "  ";
    }

    void RobotPost::ExitIf() {
        if (TAB_.size() >= 2) {
            TAB_ = TAB_.substr(0, TAB_.size() - 2);
        }
        addLine("end");
    }

    //// 字符串版本的If条件控制（可选）
    //void RobotPost::EnterIf(const std::string& condition) {
    //    make_last_move_accurate();
    //    addLine("if (" + condition + "):");
    //    TAB_ += "  ";
    //}
    //
    //// 字符串版本的While循环控制（可选）
    //void RobotPost::EnterWhile(const std::string& condition) {
    //    make_last_move_accurate();
    //    addLine("while (" + condition + "):");
    //    TAB_ += "  ";
    //}

    void test_post(const std::string& path) {
        // 创建机器人后处理器实例
        auto Pose = robomath::PosePP;
        RobotPost robot("Universal_Robots", "UR10");
    
        //// 开始测试程序
        //robot.ProgStart("TestProgram");
        //robot.RunMessage("Program generated by hejun", true);
        //robot.RunMessage("Init");

        //// 测试While循环
        //robot.EnterWhile({ 5, 1 });
        //robot.RunMessage("Main while loop test");
        //robot.Pause(20000);
        //robot.ExitWhile();
        //robot.RunMessage("Loop exit");
        //robot.EnterWhile({ 6, 1 });
        //robot.RunMessage("Nested if's while iteration");
        //robot.Pause(20000);
        //robot.EnterIf({ 7, 1 });
        //robot.RunMessage("Outer if true");
        //robot.EnterIf({ 8, 1 });
        //robot.RunMessage("Inner if true");
        //robot.ExitIf();
        //robot.ExitIf();
        //robot.ExitWhile();

        //// 设置坐标系和工具
        //robot.setFrame(Pose(807.766544, -963.699898, 41.478944, 0, 0, 0));
        //robot.setTool(Pose(62.5, -108.253175, 100, -60, 90, 0));
        //robot.setSpeed(100);
        //robot.setAcceleration(3000);

        //// 测试运动指令
        //robot.MoveJ(Pose(200, 200, 500, 180, 0, 180), { -46.18419, -6.77518, -20.54925, 71.38674, 49.58727, -302.54752 });
        //robot.MoveL(Pose(200, 250, 348.734575, 180, 0, -150), { -41.62707, -8.89064, -30.01809, 60.62329, 49.66749, -258.98418 });
        //robot.MoveL(Pose(200, 200, 262.132034, 180, 0, -150), { -43.73892, -3.91728, -35.77935, 58.57566, 54.11615, -253.81122 });

        //// 测试I/O操作
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

        //// 结束程序
        //robot.ProgFinish("TestProgram");
        robot.ProgStart("AllPath");
        robot.RunMessage("Program generated by hejun for UR10e on 05/10/2025 08:58:40", true);
        robot.RunMessage("Using nominal kinematics.", true);
        robot.setZoneData(1.000);
        robot.setSpeed(1000.000);
        robot.setFrame(Pose(350, 375, -5, -90, 0, 0), -1, "Frame Dispensing");
        robot.setTool(Pose(-43.401, 135.45, 290.032, 0, 0, 0), -1, "FerRobotics ATK 201-L-015 Taping Tool");
        robot.RunMessage("Show FerRobotics ATK 201-L-015 Taping Tool", true);
        robot.MoveJ(Pose(0,0,0,0,0,0), { 31.9253, -119.461, -80.7634, -69.776, 90, -148.075 }, {});
        robot.MoveL(Pose(60, 690, 15, -180, 0, -180), { 31.9253, -123.374, -85.4786, -61.1476, 90, -148.075 }, { 0, 1, 0 });
        robot.RunCode("TapeOn", true);
        robot.setSpeed(50.000);
        robot.MoveL(Pose(82.8, 690, 15, 180, 0, -180), { 30.7547, -122.694, -86.6136, -60.6928, 90, -149.245 }, { 0, 1, 0 });
        robot.MoveL(Pose(105.6, 690, 15, -180, 0, 180), { 29.5594, -122.056, -87.6715, -60.2722, 90, -150.441 }, { 0, 1, 0 });
        robot.MoveL(Pose(128.4, 690, 15, -180, 0, -180), { 28.3401, -121.462, -88.6532, -59.8851, 90, -151.66 }, { 0, 1, 0 });
        robot.MoveL(Pose(151.2, 690, 15, -180, 0, 180), { 27.0971, -120.91, -89.5592, -59.5305, 90, -152.903 }, { 0, 1, 0 });
        robot.MoveL(Pose(174, 690, 15, -180, 0, 180), { 25.8311, -120.402, -90.3901, -59.2077, 90, -154.169 }, { 0, 1, 0 });
        robot.MoveL(Pose(196.8, 690, 15, -180, 0, 180), { 24.5431, -119.938, -91.1463, -58.9159, 90, -155.457 }, { 0, 1, 0 });
        robot.MoveL(Pose(219.6, 690, 15, -180, 0, 180), { 23.2337, -119.517, -91.828, -58.6546, 90, -156.766 }, { 0, 1, 0 });
        robot.MoveL(Pose(242.4, 690, 15, 180, 0, 180), { 21.9041, -119.141, -92.4355, -58.4231, 90, -158.096 }, { 0, 1, 0 });
        robot.MoveL(Pose(265.2, 690, 15, -180, 0, 180), { 20.5553, -118.81, -92.9689, -58.2209, 90, -159.445 }, { 0, 1, 0 });
        robot.MoveL(Pose(288, 690, 15, -180, 0, -180), { 19.1886, -118.524, -93.4283, -58.0476, 90, -160.811 }, { 0, 1, 0 });
        robot.MoveL(Pose(310.8, 690, 15, 180, 0, 180), { 17.8053, -118.283, -93.8138, -57.9028, 90, -162.195 }, { 0, 1, 0 });
        robot.MoveL(Pose(333.6, 690, 15, 180, 0, 180), { 16.407, -118.088, -94.1254, -57.7862, 90, -163.593 }, { 0, 1, 0 });
        robot.MoveL(Pose(356.4, 690, 15, -180, 0, 180), { 14.9951, -117.939, -94.3631, -57.6975, 90, -165.005 }, { 0, 1, 0 });
        robot.MoveL(Pose(379.2, 690, 15, -180, 0, -180), { 13.5712, -117.837, -94.527, -57.6364, 90, -166.429 }, { 0, 1, 0 });
        robot.MoveL(Pose(402, 690, 15, 180, 0, 180), { 12.1372, -117.78, -94.6171, -57.6029, 90, -167.863 }, { 0, 1, 0 });
        robot.MoveL(Pose(424.8, 690, 15, -180, 0, 180), { 10.6947, -117.77, -94.6333, -57.5969, 90, -169.305 }, { 0, 1, 0 });
        robot.MoveL(Pose(447.6, 690, 15, -180, 0, -180), { 9.24555, -117.806, -94.5756, -57.6183, 90, -170.754 }, { 0, 1, 0 });
        robot.MoveL(Pose(470.4, 690, 15, -180, 0, 180), { 7.79166, -117.889, -94.4441, -57.6673, 90, -172.208 }, { 0, 1, 0 });
        robot.MoveL(Pose(493.2, 690, 15, -180, 0, 180), { 6.3349, -118.017, -94.2388, -57.7439, 90, -173.665 }, { 0, 1, 0 });
        robot.MoveL(Pose(516, 690, 15, 180, 0, 180), { 4.87715, -118.192, -93.9596, -57.8482, 90, -175.123 }, { 0, 1, 0 });
        robot.MoveL(Pose(538.8, 690, 15, -180, 0, -180), { 3.42029, -118.413, -93.6065, -57.9806, 90, -176.58 }, { 0, 1, 0 });
        robot.MoveL(Pose(561.6, 690, 15, -180, 0, 180), { 1.96622, -118.679, -93.1795, -58.1414, 90, -178.034 }, { 0, 1, 0 });
        robot.MoveL(Pose(584.4, 690, 15, 180, 0, 180), { 0.516732, -118.991, -92.6786, -58.3308, 90, -179.483 }, { 0, 1, 0 });
        robot.MoveL(Pose(607.2, 690, 15, 180, 0, 180), { -0.926368, -119.347, -92.1036, -58.5494, 90, -180.926 }, { 0, 1, 0 });
        robot.MoveL(Pose(630, 690, 15, 180, 0, -180), { -2.36137, -119.748, -91.4545, -58.7976, 90, -182.361 }, { 0, 1, 0 });
        robot.MoveL(Pose(630, 690, 15, 179.576, 0, 180), { -2.2977, -119.755, -91.4427, -58.8021, 90, -181.873 }, { 0, 1, 0 });
        robot.MoveL(Pose(630.89, 689.993, 15, 175.098, 0, -180), { -1.69218, -119.891, -91.222, -58.8868, 90, -176.79 }, { 0, 1, 0 });
        robot.MoveL(Pose(639.339, 689.269, 15, 166.993, 0, -180), { -1.08598, -120.392, -90.4073, -59.2011, 90, -168.079 }, { 0, 1, 0 });
        robot.MoveL(Pose(647.602, 687.36, 15, 158.889, 0, -180), { -0.566877, -121.026, -89.3702, -59.6043, 90, -159.455 }, { 0, 1, 0 });
        robot.MoveL(Pose(655.512, 684.306, 15, 150.784, 0, 180), { -0.144799, -121.779, -88.1302, -60.091, 90, -150.929 }, { 0, 1, 0 });
        robot.MoveL(Pose(662.914, 680.167, 15, 142.68, 0, -180), { 0.173421, -122.635, -86.7105, -60.6541, 90, -142.506 }, { 0, 1, 0 });
        robot.MoveL(Pose(669.657, 675.026, 15, 134.575, 0, -180), { 0.38419, -123.578, -85.1372, -61.2852, 90, -134.19 }, { 0, 1, 0 });
        robot.MoveL(Pose(675.609, 668.985, 15, 126.471, 0, -180), { 0.486734, -124.586, -83.4395, -61.9741, 90, -125.984 }, { 0, 1, 0 });
        robot.MoveL(Pose(680.649, 662.166, 15, 118.366, 0, -180), { 0.483061, -125.643, -81.648, -62.7095, 90, -117.883 }, { 0, 1, 0 });
        robot.MoveL(Pose(684.678, 654.704, 15, 110.261, 0, -180), { 0.377221, -126.726, -79.7958, -63.4784, 90, -109.884 }, { 0, 1, 0 });
        robot.MoveL(Pose(687.615, 646.749, 15, 102.157, 0, -180), { 0.175067, -127.816, -77.9172, -64.2668, 90, -101.982 }, { 0, 1, 0 });
        robot.MoveL(Pose(689.401, 638.459, 15, 94.0522, 0, -180), { -0.116144, -128.893, -76.0479, -65.0595, 90, -94.1684 }, { 0, 1, 0 });
        robot.MoveL(Pose(690, 630, 15, 94.0522, 0, -180), { -0.150969, -128.166, -77.3103, -64.5233, 90, -94.2032 }, { 0, 1, 0 });
        robot.MoveL(Pose(690, 630, 15, 90, 0, -180), { -0.302184, -129.05, -75.7735, -65.1765, 90, -90.3022 }, { 0, 1, 0 });
        robot.MoveL(Pose(690, 607.2, 15, 90, 0, -180), { -0.309063, -127.081, -79.1855, -63.7337, 90, -90.3091 }, { 0, 1, 0 });
        robot.MoveL(Pose(690, 584.4, 15, 90, 0, -180), { -0.316263, -125.157, -82.4731, -62.3697, 90, -90.3163 }, { 0, 1, 0 });
        robot.MoveL(Pose(690, 561.6, 15, 90, 0, -180), { -0.323806, -123.271, -85.6507, -61.0784, 90, -90.3238 }, { 0, 1, 0 });
        robot.MoveL(Pose(690, 538.8, 15, 90, 0, -180), { -0.331717, -121.415, -88.7298, -59.855, 90, -90.3317 }, { 0, 1, 0 });
        robot.MoveL(Pose(690, 516, 15, 90, 0, -180), { -0.340024, -119.584, -91.7202, -58.6958, 90, -90.34 }, { 0, 1, 0 });
        robot.MoveL(Pose(690, 493.2, 15, 90, 0, -180), { -0.348757, -117.772, -94.6299, -57.5982, 90, -90.3488 }, { 0, 1, 0 });
        robot.MoveL(Pose(690, 470.4, 15, 90, 0, -180), { -0.357951, -115.974, -97.4658, -56.5601, 90, -90.358 }, { 0, 1, 0 });
        robot.MoveL(Pose(690, 447.6, 15, 90, 0, -180), { -0.367642, -114.186, -100.234, -55.5805, 90, -90.3676 }, { 0, 1, 0 });
        robot.MoveL(Pose(690, 424.8, 15, 90, 0, -180), { -0.377873, -112.402, -102.939, -54.6588, 90, -90.3779 }, { 0, 1, 0 });
        robot.MoveL(Pose(690, 402, 15, 90, 0, -180), { -0.388688, -110.62, -105.585, -53.795, 90, -90.3887 }, { 0, 1, 0 });
        robot.MoveL(Pose(690, 379.2, 15, 90, 0, -180), { -0.40014, -108.834, -108.177, -52.9897, 90, -90.4001 }, { 0, 1, 0 });
        robot.MoveL(Pose(690, 356.4, 15, 90, 0, -180), { -0.412287, -107.039, -110.716, -52.2442, 90, -90.4123 }, { 0, 1, 0 });
        robot.MoveL(Pose(690, 333.6, 15, 90, 0, -180), { -0.425194, -105.233, -113.207, -51.5602, 90, -90.4252 }, { 0, 1, 0 });
        robot.MoveL(Pose(690, 310.8, 15, 90, 0, -180), { -0.438935, -103.41, -115.65, -50.9402, 90, -90.4389 }, { 0, 1, 0 });
        robot.MoveL(Pose(690, 288, 15, 90, 0, -180), { -0.453592, -101.565, -118.048, -50.3871, 90, -90.4536 }, { 0, 1, 0 });
        robot.MoveL(Pose(690, 265.2, 15, 90, 0, -180), { -0.469261, -99.6929, -120.402, -49.9049, 90, -90.4693 }, { 0, 1, 0 });
        robot.MoveL(Pose(690, 242.4, 15, 90, 0, -180), { -0.486049, -97.7885, -122.713, -49.4981, 90, -90.486 }, { 0, 1, 0 });
        robot.MoveL(Pose(690, 219.6, 15, 90, 0, -180), { -0.504082, -95.8455, -124.982, -49.1721, 90, -90.5041 }, { 0, 1, 0 });
        robot.MoveL(Pose(690, 196.8, 15, 90, 0, -180), { -0.523503, -93.8573, -127.209, -48.9334, 90, -90.5235 }, { 0, 1, 0 });
        robot.MoveL(Pose(690, 174, 15, 90, 0, -180), { -0.544478, -91.8165, -129.394, -48.7896, 90, -90.5445 }, { 0, 1, 0 });
        robot.MoveL(Pose(690, 151.2, 15, 90, 0, -180), { -0.567201, -89.7149, -131.535, -48.7496, 90, -90.5672 }, { 0, 1, 0 });
        robot.MoveL(Pose(690, 128.4, 15, 90, 0, -180), { -0.5919, -87.5433, -133.633, -48.8236, 90, -90.5919 }, { 0, 1, 0 });
        robot.MoveL(Pose(690, 105.6, 15, 90, 0, -180), { -0.618843, -85.2915, -135.685, -49.0238, 90, -90.6188 }, { 0, 1, 0 });
        robot.MoveL(Pose(690, 82.8, 15, 90, 0, -180), { -0.648351, -82.9478, -137.688, -49.3639, 90, -90.6484 }, { 0, 1, 0 });
        robot.MoveL(Pose(690, 60, 15, 90, 0, -180), { -0.680806, -80.4991, -139.641, -49.86, 90, -90.6808 }, { 0, 1, 0 });
        robot.setSpeed(1000.000);
        robot.RunCode("TapeOff", true);
        robot.MoveL(Pose(690, 60, 115, 90, 0, -180), { -0.680806, -73.1148, -132.869, -64.0166, 90, -90.6808 }, { 0, 1, 0 });
        robot.setSpeed(1000.000);
        robot.MoveL(Pose(561.381, 622.5, 115, 90, 0, -180), { 6.98807, -123.528, -74.2863, -72.1856, 90, -83.0119 }, { 0, 1, 0 });
        robot.MoveL(Pose(561.381, 622.5, 15, 90, 0, -180), { 6.98807, -127.132, -79.0968, -63.7708, 90, -83.0119 }, { 0, 1, 0 });
        robot.RunCode("TapeOn", true);
        robot.setSpeed(50.000);
        robot.MoveL(Pose(561.381, 595.831, 15, 90, 0, -180), { 7.17867, -124.851, -82.9915, -62.1572, 90, -82.8213 }, { 0, 1, 0 });
        robot.MoveL(Pose(561.381, 569.161, 15, 90, 0, -180), { 7.38002, -122.62, -86.7368, -60.6436, 90, -82.62 }, { 0, 1, 0 });
        robot.MoveL(Pose(561.381, 569.161, 15, 94.2724, 0, -180), { 7.63323, -121.764, -88.1555, -60.081, 90, -86.6392 }, { 0, 1, 0 });
        robot.MoveL(Pose(561.765, 564.017, 15, 102.339, 0, -180), { 8.02143, -119.715, -91.5073, -58.7773, 90, -94.3176 }, { 0, 1, 0 });
        robot.MoveL(Pose(562.863, 558.997, 15, 109.421, 0, -180), { 8.19126, -117.889, -94.4442, -57.6672, 90, -101.23 }, { 0, 1, 0 });
        robot.MoveL(Pose(564.594, 554.09, 15, 115.375, 0, -180), { 8.18493, -116.322, -96.9215, -56.7568, 90, -107.19 }, { 0, 1, 0 });
        robot.MoveL(Pose(566.872, 549.287, 15, 120.272, 0, -180), { 8.05262, -114.999, -98.9821, -56.0192, 90, -112.22 }, { 0, 1, 0 });
        robot.MoveL(Pose(569.623, 544.573, 15, 124.721, 0, -180), { 7.8234, -113.798, -100.827, -55.3751, 90, -116.898 }, { 0, 1, 0 });
        robot.MoveL(Pose(573.804, 538.54, 15, 128.473, 0, -180), { 7.46803, -112.655, -102.559, -54.7859, 90, -121.005 }, { 0, 1, 0 });
        robot.MoveL(Pose(578.501, 532.629, 15, 131.135, 0, -180), { 7.07554, -111.743, -103.925, -54.3323, 90, -124.059 }, { 0, 1, 0 });
        robot.MoveL(Pose(583.56, 526.837, 15, 133.054, 0, -180), { 6.66621, -110.986, -105.046, -53.9679, 90, -126.388 }, { 0, 1, 0 });
        robot.MoveL(Pose(590.649, 519.25, 15, 134.004, 0, -180), { 6.16442, -110.279, -106.085, -53.6365, 90, -127.84 }, { 0, 1, 0 });
        robot.MoveL(Pose(597.814, 511.831, 15, 133.749, 0, -180), { 5.71863, -109.795, -106.79, -53.4155, 90, -128.03 }, { 0, 1, 0 });
        robot.MoveL(Pose(604.762, 504.573, 15, 132.393, 0, -180), { 5.34769, -109.512, -107.199, -53.2885, 90, -127.046 }, { 0, 1, 0 });
        robot.MoveL(Pose(611.257, 497.458, 15, 129.987, 0, -180), { 5.05779, -109.424, -107.326, -53.2493, 90, -124.929 }, { 0, 1, 0 });
        robot.MoveL(Pose(616.581, 491.11, 15, 126.615, 0, -180), { 4.87413, -109.569, -107.118, -53.3136, 90, -121.741 }, { 0, 1, 0 });
        robot.MoveL(Pose(621.215, 484.875, 15, 122.04, 0, -180), { 4.74416, -109.954, -106.559, -53.4875, 90, -117.296 }, { 0, 1, 0 });
        robot.MoveL(Pose(625.055, 478.738, 15, 116.119, 0, -180), { 4.62625, -110.622, -105.582, -53.7959, 90, -111.493 }, { 0, 1, 0 });
        robot.MoveL(Pose(628.009, 472.713, 15, 108.762, 0, -180), { 4.45318, -111.604, -104.132, -54.2645, 90, -104.309 }, { 0, 1, 0 });
        robot.MoveL(Pose(630.021, 466.789, 15, 99.9927, 0, -180), { 4.13976, -112.891, -102.203, -54.9058, 90, -95.8529 }, { 0, 1, 0 });
        robot.MoveL(Pose(631.048, 460.965, 15, 90.4146, 0, -180), { 3.635, -114.335, -100.005, -55.6603, 90, -86.7796 }, { 0, 1, 0 });
        robot.MoveL(Pose(631.089, 455.229, 15, 80.4371, 0, -180), { 2.93598, -115.804, -97.7317, -56.4645, 90, -77.5012 }, { 0, 1, 0 });
        robot.MoveL(Pose(630.142, 449.607, 15, 70.9393, 0, -180), { 2.13444, -117.084, -95.7206, -57.195, 90, -68.8049 }, { 0, 1, 0 });
        robot.MoveL(Pose(628.228, 444.067, 15, 62.5814, 0, -180), { 1.35823, -118.042, -94.1989, -57.7587, 90, -61.2232 }, { 0, 1, 0 });
        robot.MoveL(Pose(625.398, 438.611, 15, 55.3721, 0, -180), { 0.671166, -118.696, -93.1526, -58.1515, 90, -54.7009 }, { 0, 1, 0 });
        robot.MoveL(Pose(621.522, 432.999, 15, 49.4492, 0, -180), { 0.147784, -119.036, -92.605, -58.3587, 90, -49.3014 }, { 0, 1, 0 });
        robot.MoveL(Pose(616.797, 427.476, 15, 44.8473, 0, -180), { -0.183427, -119.123, -92.465, -58.4119, 90, -45.0307 }, { 0, 1, 0 });
        robot.MoveL(Pose(611.319, 422.028, 15, 41.3414, 0, -180), { -0.335276, -119.023, -92.6261, -58.3507, 90, -41.6767 }, { 0, 1, 0 });
        robot.MoveL(Pose(605.213, 416.656, 15, 38.7467, 0, -180), { -0.327078, -118.788, -93.005, -58.2073, 90, -39.0738 }, { 0, 1, 0 });
        robot.MoveL(Pose(598.597, 411.347, 15, 36.924, 0, -180), { -0.177215, -118.452, -93.544, -58.0041, 90, -37.1012 }, { 0, 1, 0 });
        robot.MoveL(Pose(591.605, 406.092, 15, 35.5587, 0, -180), { 0.064682, -118.061, -94.1687, -57.77, 90, -35.494 }, { 0, 1, 0 });
        robot.MoveL(Pose(580.203, 397.942, 15, 35.1268, 0, -180), { 0.739153, -117.285, -95.4034, -57.3118, 90, -34.3876 }, { 0, 1, 0 });
        robot.MoveL(Pose(568.77, 389.898, 15, 36.1187, 0, -180), { 1.65218, -116.394, -96.808, -56.798, 90, -34.4665 }, { 0, 1, 0 });
        robot.MoveL(Pose(557.84, 381.923, 15, 37.9397, 0, -180), { 2.68276, -115.448, -98.2853, -56.2665, 90, -35.2569 }, { 0, 1, 0 });
        robot.MoveL(Pose(551.739, 377.166, 15, 40.0961, 0, -180), { 3.44976, -114.786, -99.310, -55.9036, 90, -36.6463 }, { 0, 1, 0 });
        robot.MoveL(Pose(546.102, 372.42, 15, 42.9954, 0, -180), { 4.31162, -114.047, -100.445, -55.5071, 90, -38.6838 }, { 0, 1, 0 });
        robot.MoveL(Pose(541.03, 367.692, 15, 46.7993, 0, -180), { 5.28485, -113.199, -101.738, -55.0634, 90, -41.5144 }, { 0, 1, 0 });
        robot.MoveL(Pose(536.596, 362.969, 15, 51.7924, 0, 180), { 6.40284, -112.18, -103.273, -54.5477, 90, -45.3896 }, { 0, 1, 0 });
        robot.MoveL(Pose(532.885, 358.256, 15, 57.9281, 0, -180), { 7.63797, -110.958, -105.088, -53.9542, 90, -50.2901 }, { 0, 1, 0 });
        robot.MoveL(Pose(529.992, 353.638, 15, 65.5117, 0, -180), { 9.00078, -109.448, -107.292, -53.2598, 90, -56.5109 }, { 0, 1, 0 });
        robot.MoveL(Pose(527.891, 349.026, 15, 74.5583, 0, -180), { 10.4431, -107.58, -109.959, -52.4618, 90, -64.1152 }, { 0, 1, 0 });
        robot.MoveL(Pose(526.612, 344.397, 15, 84.7428, 0, -180), { 11.8394, -105.353, -113.043, -51.6036, 90, -72.9034 }, { 0, 1, 0 });
        robot.MoveL(Pose(526.187, 339.779, 15, 95.6766, 0, -180), { 13.0397, -102.823, -116.421, -50.7559, 90, -82.6369 }, { 0, 1, 0 });
        robot.MoveL(Pose(526.683, 334.791, 15, 106.615, 0, -180), { 13.879, -100.128, -119.863, -50.0097, 90, -92.736 }, { 0, 1, 0 });
        robot.MoveL(Pose(528.172, 329.802, 15, 116.164, 0, -180), { 14.2363, -97.6552, -122.872, -49.4728, 90, -101.928 }, { 0, 1, 0 });
        robot.MoveL(Pose(530.616, 324.828, 15, 123.982, 0, -180), { 14.1901, -95.5431, -125.327, -49.1296, 90, -109.792 }, { 0, 1, 0 });
        robot.MoveL(Pose(533.968, 319.854, 15, 130.160, 0, -180), { 13.8605, -93.7994, -127.273, -48.9279, 90, -116.299 }, { 0, 1, 0 });
        robot.MoveL(Pose(538.177, 314.866, 15, 135.001, 0, -180), { 13.3443, -92.3631, -128.819, -48.818, 90, -121.657 }, { 0, 1, 0 });
        robot.MoveL(Pose(543.163, 309.881, 15, 138.875, 0, -180), { 12.6984, -91.1556, -130.079, -48.7652, 90, -126.176 }, { 0, 1, 0 });
        robot.MoveL(Pose(549.908, 303.992, 15, 141.890, 0, -180), { 11.8805, -90.0692, -131.182, -48.7486, 90, -130.01 }, { 0, 1, 0 });
        robot.MoveL(Pose(557.431, 298.09, 15, 143.845, 0, 180), { 11.0247, -89.2112, -132.032, -48.7563, 90, -132.821 }, { 0, 1, 0 });
        robot.MoveL(Pose(565.536, 292.169, 15, 144.947, 0, -180), { 10.1636, -88.5306, -132.694, -48.7754, 90, -134.783 }, { 0, 1, 0 });
        robot.MoveL(Pose(574.011, 286.222, 15, 145.246, 0, 180), { 9.33402, -88.0085, -133.194, -48.7979, 90, -135.912 }, { 0, 1, 0 });
        robot.MoveL(Pose(582.63, 280.243, 15, 144.761, 0, 180), { 8.57091, -87.6339, -133.548, -48.8182, 90, -136.19 }, { 0, 1, 0 });
        robot.MoveL(Pose(591.168, 274.21, 15, 143.493, 0, -180), { 7.90278, -87.4017, -133.766, -48.8325, 90, -135.59 }, { 0, 1, 0 });
        robot.MoveL(Pose(599.396, 268.12, 15, 141.323, 0, 180), { 7.36481, -87.334, -133.829, -48.837, 90, -133.958 }, { 0, 1, 0 });
        robot.MoveL(Pose(607.077, 261.972, 15, 138.475, 0, -180), { 6.93979, -87.3966, -133.771, -48.8329, 90, -131.535 }, { 0, 1, 0 });
        robot.MoveL(Pose(612.593, 257.088, 15, 135.087, 0, -180), { 6.75149, -87.6742, -133.51, -48.8159, 90, -128.335 }, { 0, 1, 0 });
        robot.MoveL(Pose(617.547, 252.149, 15, 130.712, 0, -180), { 6.65256, -88.1689, -133.041, -48.7903, 90, -124.059 }, { 0, 1, 0 });
        robot.MoveL(Pose(621.839, 247.161, 15, 125.152, 0, 180), { 6.60342, -88.9451, -132.292, -48.7624, 90, -118.549 }, { 0, 1, 0 });
        robot.MoveL(Pose(625.398, 242.107, 15, 118.152, 0, -180), { 6.525, -90.0793, -131.172, -48.7487, 90, -111.627 }, { 0, 1, 0 });
        robot.MoveL(Pose(628.229, 236.817, 15, 109.653, 0, -180), { 6.2867, -91.5783, -129.642, -48.7796, 90, -103.367 }, { 0, 1, 0 });
        robot.MoveL(Pose(630.142, 231.458, 15, 99.9004, 0, -180), { 5.7766, -93.3738, -127.736, -48.89, 90, -94.1238 }, { 0, 1, 0 });
        robot.MoveL(Pose(631.089, 226.035, 15, 89.5646, 0, -180), { 4.95859, -95.2498, -125.66, -49.0905, 90, -84.606 }, { 0, 1, 0 });
        robot.MoveL(Pose(631.047, 220.509, 15, 79.62, 0, -180), { 3.94132, -96.9237, -123.735, -49.3418, 90, -75.6786 }, { 0, 1, 0 });
        robot.MoveL(Pose(630.021, 214.912, 15, 70.5054, 0, -180), { 2.86206, -98.2801, -122.125, -49.5949, 90, -67.6434 }, { 0, 1, 0 });
        robot.MoveL(Pose(628.01, 209.228, 15, 62.8933, 0, -180), { 1.91333, -99.2007, -121.008, -49.7916, 90, -60.98 }, { 0, 1, 0 });
        robot.MoveL(Pose(625.055, 203.457, 15, 56.8109, 0, -180), { 1.18451, -99.7273, -120.36, -49.9131, 90, -55.6264 }, { 0, 1, 0 });
        robot.MoveL(Pose(621.214, 197.585, 15, 52.1438, 0, -180), { 0.711645, -99.9261, -120.113, -49.9606, 90, -51.4322 }, { 0, 1, 0 });
        robot.MoveL(Pose(616.581, 191.623, 15, 48.7302, 0, -180), { 0.496796, -99.8631, -120.192, -49.9454, 90, -48.2334 }, { 0, 1, 0 });
        robot.MoveL(Pose(611.257, 185.557, 15, 46.3044, 0, -180), { 0.507433, -99.6052, -120.51, -49.8843, 90, -45.7969 }, { 0, 1, 0 });
        robot.MoveL(Pose(604.64, 178.631, 15, 44.9762, 0, -180), { 0.836456, -99.0775, -121.158, -49.7641, 90, -44.1398 }, { 0, 1, 0 });
        robot.MoveL(Pose(597.556, 171.553, 15, 44.8166, 0, -180), { 1.44333, -98.3568, -122.033, -49.6105, 90, -43.3732 }, { 0, 1, 0 });
        robot.MoveL(Pose(590.257, 164.301, 15, 45.9487, 0, -180), { 2.34654, -97.4304, -123.138, -49.4312, 90, -43.6021 }, { 0, 1, 0 });
        robot.MoveL(Pose(583.056, 156.858, 15, 48.5675, 0, -180), { 3.57183, -96.2613, -124.504, -49.2342, 90, -44.9957 }, { 0, 1, 0 });
        robot.MoveL(Pose(576.28, 149.18, 15, 51.8356, 0, -180), { 4.92752, -94.9527, -125.994, -49.053, 90, -46.9081 }, { 0, 1, 0 });
        robot.MoveL(Pose(572.808, 144.762, 15, 55.0399, 0, -180), { 5.95611, -93.9687, -127.087, -48.9442, 90, -49.0838 }, { 0, 1, 0 });
        robot.MoveL(Pose(569.656, 140.255, 15, 59.1123, 0, -180), { 7.1426, -92.793, -128.361, -48.8456, 90, -51.9697 }, { 0, 1, 0 });
        robot.MoveL(Pose(566.894, 135.637, 15, 64.176, 0, -180), { 8.49725, -91.3676, -129.861, -48.7719, 90, -55.6788 }, { 0, 1, 0 });
        robot.MoveL(Pose(564.607, 130.911, 15, 70.3014, 0, -180), { 10.0073, -89.6315, -131.618, -48.7503, 90, -60.2941 }, { 0, 1, 0 });
        robot.MoveL(Pose(562.87, 126.059, 15, 77.5366, 0, -180), { 11.641, -87.5108, -133.664, -48.8256, 90, -65.8956 }, { 0, 1, 0 });
        robot.MoveL(Pose(561.767, 121.07, 15, 85.7066, 0, -180), { 13.3022, -84.9737, -135.964, -49.0621, 90, -72.4044 }, { 0, 1, 0 });
        robot.MoveL(Pose(561.381, 115.93, 15, 85.7066, 0, -180), { 13.4834, -84.4157, -136.449, -49.1352, 90, -72.2232 }, { 0, 1, 0 });
        robot.MoveL(Pose(561.381, 115.93, 15, 90, 0, -180), { 14.1801, -83.2771, -137.415, -49.3081, 90, -75.8199 }, { 0, 1, 0 });
        robot.MoveL(Pose(561.381, 87.9649, 15, 90, 0, -180), { 15.0466, -80.071, -139.968, -49.9613, 90, -74.9534 }, { 0, 1, 0 });
        robot.MoveL(Pose(561.381, 60, 15, 90, 0, -180), { 16.0296, -76.6253, -142.446, -50.9289, 90, -73.9704 }, { 0, 1, 0 });
        robot.setSpeed(1000.000);
        robot.RunCode("TapeOff", true);
        robot.MoveL(Pose(561.381, 60, 115, 90, 0, -180), { 16.0296, -69.1325, -135.324, -65.5433, 90, -73.9704 }, { 0, 1, 0 });
        robot.setSpeed(1000.000);
        robot.MoveL(Pose(457.5, 622.5, 115, 90, 0, -180), { 12.8407, -123.538, -74.2706, -72.1917, 90, -77.1593 }, { 0, 1, 0 });
        robot.MoveL(Pose(457.5, 622.5, 15, 90, 0, -180), { 12.8407, -127.141, -79.0814, -63.7773, 90, -77.1593 }, { 0, 1, 0 });
        robot.RunCode("TapeOn", true);
        robot.setSpeed(50.000);
        robot.MoveL(Pose(457.5, 599.063, 15, 90, 0, -180), { 13.1456, -125.134, -82.5126, -62.3535, 90, -76.8544 }, { 0, 1, 0 });
        robot.MoveL(Pose(457.5, 575.625, 15, 90, 0, -180), { 13.4653, -123.166, -85.8266, -61.0078, 90, -76.5347 }, { 0, 1, 0 });
        robot.MoveL(Pose(457.5, 552.188, 15, 90, 0, -180), { 13.8011, -121.229, -89.0363, -59.7348, 90, -76.1989 }, { 0, 1, 0 });
        robot.MoveL(Pose(457.5, 528.75, 15, 90, 0, -180), { 14.154, -119.317, -92.1526, -58.5307, 90, -75.846 }, { 0, 1, 0 });
        robot.MoveL(Pose(457.5, 505.313, 15, 90, 0, -180), { 14.5255, -117.423, -95.1846, -57.3925, 90, -75.4745 }, { 0, 1, 0 });
        robot.MoveL(Pose(457.5, 481.875, 15, 90, 0, -180), { 14.9172, -115.542, -98.14, -56.3183, 90, -75.0828 }, { 0, 1, 0 });
        robot.MoveL(Pose(457.5, 458.438, 15, 90, 0, -180), { 15.3306, -113.668, -101.025, -55.307, 90, -74.6694 }, { 0, 1, 0 });
        robot.MoveL(Pose(457.5, 435, 15, 90, 0, -180), { 15.7677, -111.796, -103.846, -54.3583, 90, -74.2323 }, { 0, 1, 0 });
        robot.MoveL(Pose(457.5, 411.563, 15, 90, 0, -180), { 16.2305, -109.921, -106.607, -53.4725, 90, -73.7695 }, { 0, 1, 0 });
        robot.MoveL(Pose(457.5, 388.125, 15, 90, 0, -180), { 16.7215, -108.037, -109.312, -52.6507, 90, -73.2785 }, { 0, 1, 0 });
        robot.MoveL(Pose(457.5, 364.688, 15, 90, 0, -180), { 17.2433, -106.14, -111.965, -51.8949, 90, -72.7567 }, { 0, 1, 0 });
        robot.MoveL(Pose(457.5, 341.25, 15, 90, 0, -180), { 17.7989, -104.223, -114.569, -51.2078, 90, -72.2011 }, { 0, 1, 0 });
        robot.MoveL(Pose(457.5, 317.813, 15, 90, 0, -180), { 18.3916, -102.281, -117.127, -50.5927, 90, -71.6084 }, { 0, 1, 0 });
        robot.MoveL(Pose(457.5, 294.375, 15, 90, 0, -180), { 19.0255, -100.306, -119.639, -50.0541, 90, -70.9745 }, { 0, 1, 0 });
        robot.MoveL(Pose(457.5, 270.938, 15, 90, 0, -180), { 19.7049, -98.2933, -122.109, -49.5976, 90, -70.2951 }, { 0, 1, 0 });
        robot.MoveL(Pose(457.5, 247.5, 15, 90, 0, -180), { 20.4351, -96.2332, -124.537, -49.2299, 90, -69.5649 }, { 0, 1, 0 });
        robot.MoveL(Pose(457.5, 224.063, 15, 90, 0, -180), { 21.2218, -94.1172, -126.924, -48.9591, 90, -68.7782 }, { 0, 1, 0 });
        robot.MoveL(Pose(457.5, 200.625, 15, 90, 0, -180), { 22.0721, -91.9354, -129.269, -48.7952, 90, -67.9279 }, { 0, 1, 0 });
        robot.MoveL(Pose(457.5, 177.188, 15, 90, 0, -180), { 22.9941, -89.6759, -131.574, -48.7499, 90, -67.0059 }, { 0, 1, 0 });
        robot.MoveL(Pose(457.5, 153.75, 15, 90, 0, -180), { 23.9973, -87.3255, -133.837, -48.8375, 90, -66.0027 }, { 0, 1, 0 });
        robot.MoveL(Pose(457.5, 130.312, 15, 90, 0, -180), { 25.0932, -84.8684, -136.056, -49.0753, 90, -64.9068 }, { 0, 1, 0 });
        robot.MoveL(Pose(457.5, 106.875, 15, 90, 0, -180), { 26.2953, -82.2862, -138.23, -49.4839, 90, -63.7047 }, { 0, 1, 0 });
        robot.MoveL(Pose(457.5, 83.4375, 15, 90, 0, -180), { 27.6201, -79.557, -140.355, -50.0885, 90, -62.3799 }, { 0, 1, 0 });
        robot.MoveL(Pose(457.5, 60, 15, 90, 0, -180), { 29.088, -76.6549, -142.426, -50.9195, 90, -60.912 }, { 0, 1, 0 });
        robot.setSpeed(1000.000);
        robot.RunCode("TapeOff", true);
        robot.MoveL(Pose(457.5, 60, 115, 90, 0, -180), { 29.088, -69.1626, -135.307, -65.5307, 90, -60.912 }, { 0, 1, 0 });
        robot.setSpeed(1000.000);
        robot.MoveL(Pose(397.5, 622.5, 115, 90, 0, -180), { 16.1538, -123.997, -73.5237, -72.4791, 90, -73.8462 }, { 0, 1, 0 });
        robot.MoveL(Pose(397.5, 622.5, 15, 90, 0, -180), { 16.1538, -127.566, -78.3497, -64.0846, 90, -73.8462 }, { 0, 1, 0 });
        robot.RunCode("TapeOn", true);
        robot.setSpeed(50.000);
        robot.MoveL(Pose(397.5, 599.063, 15, 90, 0, -180), { 16.5334, -125.559, -81.7907, -62.6506, 90, -73.4666 }, { 0, 1, 0 });
        robot.MoveL(Pose(397.5, 575.625, 15, 90, 0, -180), { 16.9313, -123.593, -85.1116, -61.2955, 90, -73.0687 }, { 0, 1, 0 });
        robot.MoveL(Pose(397.5, 552.188, 15, 90, 0, -180), { 17.3487, -121.66, -88.3259, -60.0138, 90, -72.6513 }, { 0, 1, 0 });
        robot.MoveL(Pose(397.5, 528.75, 15, 90, 0, -180), { 17.7872, -119.754, -91.4448, -58.8013, 90, -72.2128 }, { 0, 1, 0 });
        robot.MoveL(Pose(397.5, 505.313, 15, 90, 0, -180), { 18.2483, -117.868, -94.4776, -57.6548, 90, -71.7517 }, { 0, 1, 0 });
        robot.MoveL(Pose(397.5, 481.875, 15, 90, 0, -180), { 18.7339, -115.996, -97.432, -56.5723, 90, -71.2661 }, { 0, 1, 0 });
        robot.MoveL(Pose(397.5, 458.437, 15, 90, 0, -180), { 19.246, -114.133, -100.315, -55.5524, 90, -70.754 }, { 0, 1, 0 });
        robot.MoveL(Pose(397.5, 435, 15, 90, 0, -180), { 19.7868, -112.274, -103.131, -54.5946, 90, -70.2132 }, { 0, 1, 0 });
        robot.MoveL(Pose(397.5, 411.563, 15, 90, 0, -180), { 20.3587, -110.414, -105.887, -53.6991, 90, -69.6413 }, { 0, 1, 0 });
        robot.MoveL(Pose(397.5, 388.125, 15, 90, 0, -180), { 20.9646, -108.548, -108.585, -52.8667, 90, -69.0354 }, { 0, 1, 0 });
        robot.MoveL(Pose(397.5, 364.688, 15, 90, 0, -180), { 21.6075, -106.67, -111.231, -52.099, 90, -68.3925 }, { 0, 1, 0 });
        robot.MoveL(Pose(397.5, 341.25, 15, 90, 0, -180), { 22.2908, -104.777, -113.825, -51.3983, 90, -67.7092 }, { 0, 1, 0 });
        robot.MoveL(Pose(397.5, 317.813, 15, 90, 0, -180), { 23.0187, -102.861, -116.372, -50.7676, 90, -66.9813 }, { 0, 1, 0 });
        robot.MoveL(Pose(397.5, 294.375, 15, 90, 0, -180), { 23.7954, -100.917, -118.872, -50.2111, 90, -66.2046 }, { 0, 1, 0 });
        robot.MoveL(Pose(397.5, 270.938, 15, 90, 0, -180), { 24.6261, -98.9379, -121.329, -49.7334, 90, -65.3739 }, { 0, 1, 0 });
        robot.MoveL(Pose(397.5, 247.5, 15, 90, 0, -180), { 25.5166, -96.9174, -123.742, -49.3407, 90, -64.4834 }, { 0, 1, 0 });
        robot.MoveL(Pose(397.5, 224.062, 15, 90, 0, -180), { 26.4736, -94.8471, -126.113, -49.0402, 90, -63.5264 }, { 0, 1, 0 });
        robot.MoveL(Pose(397.5, 200.625, 15, 90, 0, -180), { 27.5047, -92.7182, -128.441, -48.8404, 90, -62.4953 }, { 0, 1, 0 });
        robot.MoveL(Pose(397.5, 177.188, 15, 90, 0, -180), { 28.6188, -90.5203, -130.728, -48.7519, 90, -61.3812 }, { 0, 1, 0 });
        robot.MoveL(Pose(397.5, 153.75, 15, 90, 0, -180), { 29.8264, -88.242, -132.971, -48.787, 90, -60.1736 }, { 0, 1, 0 });
        robot.MoveL(Pose(397.5, 130.313, 15, 90, 0, -180), { 31.1396, -85.8697, -135.17, -48.9605, 90, -58.8604 }, { 0, 1, 0 });
        robot.MoveL(Pose(397.5, 106.875, 15, 90, 0, -180), { 32.5728, -83.3881, -137.322, -49.2898, 90, -57.4272 }, { 0, 1, 0 });
        robot.MoveL(Pose(397.5, 83.4375, 15, 90, 0, -180), { 34.1432, -80.779, -139.425, -49.7962, 90, -55.8568 }, { 0, 1, 0 });
        robot.MoveL(Pose(397.5, 60, 15, 90, 0, -180), { 35.8712, -78.0214, -141.474, -50.5046, 90, -54.1288 }, { 0, 1, 0 });
        robot.setSpeed(1000.000);
        robot.RunCode("TapeOff", true);
        robot.MoveL(Pose(397.5, 60, 115, 90, 0, -180), { 35.8712, -70.5575, -134.479, -64.9636, 90, -54.1288 }, { 0, 1, 0 });
        robot.setSpeed(1000.000);
        robot.MoveL(Pose(330, 607.5, 115, 90, 0, -180), { 20.0794, -123.515, -74.3075, -72.1775, 90, -69.9206 }, { 0, 1, 0 });
        robot.MoveL(Pose(330, 607.5, 15, 90, 0, -180), { 20.0794, -127.12, -79.1176, -63.7621, 90, -69.9206 }, { 0, 1, 0 });
        robot.RunCode("TapeOn", true);
        robot.setSpeed(50.000);
        robot.MoveL(Pose(330, 584.348, 15, 90, 0, -180), { 20.5443, -125.167, -82.457, -62.3763, 90, -69.4557 }, { 0, 1, 0 });
        robot.MoveL(Pose(330, 561.196, 15, 90, 0, -180), { 21.031, -123.252, -85.6829, -61.0655, 90, -68.969 }, { 0, 1, 0 });
        robot.MoveL(Pose(330, 538.043, 15, 90, 0, -180), { 21.5411, -121.368, -88.8073, -59.8246, 90, -68.4589 }, { 0, 1, 0 });
        robot.MoveL(Pose(330, 514.891, 15, 90, 0, -180), { 22.0761, -119.51, -91.8405, -58.6498, 90, -67.9239 }, { 0, 1, 0 });
        robot.MoveL(Pose(330, 491.739, 15, 90, 0, -180), { 22.638, -117.671, -94.7907, -57.5384, 90, -67.362 }, { 0, 1, 0 });
        robot.MoveL(Pose(330, 468.587, 15, 90, 0, -180), { 23.2289, -115.846, -97.6653, -56.4884, 90, -66.7711 }, { 0, 1, 0 });
        robot.MoveL(Pose(330, 445.435, 15, 90, 0, -180), { 23.8509, -114.031, -100.47, -55.4985, 90, -66.1491 }, { 0, 1, 0 });
        robot.MoveL(Pose(330, 422.283, 15, 90, 0, -180), { 24.5066, -112.221, -103.211, -54.5683, 90, -65.4934 }, { 0, 1, 0 });
        robot.MoveL(Pose(330, 399.13, 15, 90, 0, -180), { 25.1987, -110.411, -105.891, -53.6978, 90, -64.8013 }, { 0, 1, 0 });
        robot.MoveL(Pose(330, 375.978, 15, 90, 0, -180), { 25.9304, -108.597, -108.515, -52.8878, 90, -64.0696 }, { 0, 1, 0 });
        robot.MoveL(Pose(330, 352.826, 15, 90, 0, -180), { 26.705, -106.774, -111.086, -52.1396, 90, -63.295 }, { 0, 1, 0 });
        robot.MoveL(Pose(330, 329.674, 15, 90, 0, -180), { 27.5263, -104.938, -113.607, -51.4551, 90, -62.4737 }, { 0, 1, 0 });
        robot.MoveL(Pose(330, 306.522, 15, 90, 0, -180), { 28.3986, -103.084, -116.079, -50.837, 90, -61.6014 }, { 0, 1, 0 });
        robot.MoveL(Pose(330, 283.37, 15, 90, 0, -180), { 29.3266, -101.207, -118.505, -50.2886, 90, -60.6734 }, { 0, 1, 0 });
        robot.MoveL(Pose(330, 260.217, 15, 90, 0, -180), { 30.3158, -99.3002, -120.886, -49.8141, 90, -59.6842 }, { 0, 1, 0 });
        robot.MoveL(Pose(330, 237.065, 15, 90, 0, -180), { 31.3723, -97.3593, -123.222, -49.4183, 90, -58.6277 }, { 0, 1, 0 });
        robot.MoveL(Pose(330, 213.913, 15, 90, 0, -180), { 32.5029, -95.3772, -125.516, -49.1072, 90, -57.4971 }, { 0, 1, 0 });
        robot.MoveL(Pose(330, 190.761, 15, 90, 0, -180), { 33.7154, -93.3468, -127.765, -48.8878, 90, -56.2846 }, { 0, 1, 0 });
        robot.MoveL(Pose(330, 167.609, 15, 90, 0, -180), { 35.0187, -91.2603, -129.971, -48.7683, 90, -54.9813 }, { 0, 1, 0 });
        robot.MoveL(Pose(330, 144.457, 15, 90, 0, -180), { 36.4232, -89.1088, -132.133, -48.7585, 90, -53.5768 }, { 0, 1, 0 });
        robot.MoveL(Pose(330, 121.304, 15, 90, 0, -180), { 37.9404, -86.8825, -134.248, -48.8694, 90, -52.0596 }, { 0, 1, 0 });
        robot.MoveL(Pose(330, 98.1522, 15, 90, 0, -180), { 39.5839, -84.5702, -136.316, -49.1142, 90, -50.4161 }, { 0, 1, 0 });
        robot.MoveL(Pose(330, 75, 15, 90, 0, -180), { 41.3694, -82.1594, -138.332, -49.5081, 90, -48.6306 }, { 0, 1, 0 });
        robot.MoveL(Pose(330, 75, 15, 85.5481, 0, -180), { 40.2068, -83.1062, -137.557, -49.3367, 90, -45.3413 }, { 0, 1, 0 });
        robot.MoveL(Pose(329.819, 72.6783, 15, 72.9869, 0, -180), { 37.038, -85.1733, -135.789, -49.0377, 90, -35.9489 }, { 0, 1, 0 });
        robot.MoveL(Pose(328.581, 68.6315, 15, 56.7671, 0, -180), { 32.994, -86.9785, -134.159, -48.8621, 90, -23.7731 }, { 0, 1, 0 });
        robot.MoveL(Pose(326.262, 65.0917, 15, 40.5479, 0, 180), { 29.0141, -87.97, -133.23, -48.7998, 90, -11.5338 }, { 0, 1, 0 });
        robot.MoveL(Pose(323.046, 62.3405, 15, 24.3288, 0, -180), { 25.2149, -88.1549, -133.054, -48.7909, 90, 0.886106 }, { 0, 1, 0 });
        robot.MoveL(Pose(319.19, 60.597, 15, 8.10962, 0, 180), { 21.7191, -87.5344, -133.641, -48.8242, 90, 13.6095 }, { 0, 1, 0 });
        robot.MoveL(Pose(315, 60, 15, 8.10962, 0, 180), { 22.169, -87.4998, -133.674, -48.8263, 90, 14.0594 }, { 0, 1, 0 });
        robot.MoveL(Pose(315, 60, 15, 0, 0, 180), { 20.3534, -86.9117, -134.221, -48.8671, 90, 20.3534 }, { 0, 1, 0 });
        robot.MoveL(Pose(291, 60, 15, 0, 0, -180), { 22.8228, -87.0419, -134.101, -48.8574, 90, 22.8228 }, { 0, 1, 0 });
        robot.MoveL(Pose(267, 60, 15, 0, 0, 180), { 25.2395, -87.2796, -133.88, -48.8406, 90, 25.2395 }, { 0, 1, 0 });
        robot.MoveL(Pose(243, 60, 15, 0, 0, -180), { 27.5958, -87.6223, -133.559, -48.8189, 90, 27.5958 }, { 0, 1, 0 });
        robot.MoveL(Pose(219, 60, 15, 0, 0, 180), { 29.885, -88.0659, -133.139, -48.7951, 90, 29.885 }, { 0, 1, 0 });
        robot.MoveL(Pose(195, 60, 15, 0, 0, 180), { 32.1016, -88.6058, -132.621, -48.7728, 90, 32.1016 }, { 0, 1, 0 });
        robot.MoveL(Pose(171, 60, 15, 0, 0, -180), { 34.2413, -89.2363, -132.008, -48.7558, 90, 34.2413 }, { 0, 1, 0 });
        robot.MoveL(Pose(147, 60, 15, 0, 0, 180), { 36.301, -89.9516, -131.3, -48.7486, 90, 36.301 }, { 0, 1, 0 });
        robot.MoveL(Pose(123, 60, 15, 0, 0, 180), { 38.2789, -90.7452, -130.499, -48.7555, 90, 38.2789 }, { 0, 1, 0 });
        robot.MoveL(Pose(99, 60, 15, 0, 0, -180), { 40.1741, -91.6107, -129.608, -48.7809, 90, 40.1741 }, { 0, 1, 0 });
        robot.MoveL(Pose(75, 60, 15, 0, 0, 180), { 41.9864, -92.5417, -128.629, -48.8289, 90, 41.9864 }, { 0, 1, 0 });
        robot.MoveL(Pose(75, 60, 15, -4.45195, 0, 180), { 41.4629, -91.8044, -129.406, -48.7891, 90, 45.9149 }, { 0, 1, 0 });
        robot.MoveL(Pose(72.6783, 60.1808, 15, -17.0134, 0, 180), { 40.5085, -89.5277, -131.721, -48.7514, 90, 57.522 }, { 0, 1, 0 });
        robot.MoveL(Pose(68.6315, 61.419, 15, -33.2327, 0, 180), { 40.2447, -86.0909, -134.971, -48.9384, 90, 73.4774 }, { 0, 1, 0 });
        robot.MoveL(Pose(65.0917, 63.7383, 15, -49.452, 0, 180), { 41.1826, -82.1348, -138.352, -49.5128, 90, 90.6346 }, { 0, 1, 0 });
        robot.MoveL(Pose(62.3405, 66.9541, 15, -65.6712, 0, 180), { 43.5328, -77.8475, -141.597, -50.555, 90, 109.204 }, { 0, 1, 0 });
        robot.MoveL(Pose(60.597, 70.8103, 15, -81.8904, 0, 180), { 47.3794, -73.5868, -144.414, -51.9994, 90, 129.27 }, { 0, 1, 0 });
        robot.MoveL(Pose(60, 75, 15, -81.8904, 0, 180), { 46.9965, -74.1471, -144.066, -51.7872, 90, 128.887 }, { 0, 1, 0 });
        robot.MoveL(Pose(60, 75, 15, -90, 0, 180), { 49.5877, -71.9289, -145.406, -52.665, 90, 139.588 }, { 0, 1, 0 });
        robot.MoveL(Pose(60, 98.1522, 15, -90, 0, 180), { 47.0825, -74.8543, -143.617, -51.5289, 90, 137.083 }, { 0, 1, 0 });
        robot.MoveL(Pose(60, 121.304, 15, -90, 0, 180), { 44.8051, -77.6186, -141.759, -50.6225, 90, 134.805 }, { 0, 1, 0 });
        robot.MoveL(Pose(60, 144.457, 15, -90, 0, 180), { 42.7273, -80.2407, -139.839, -49.9206, 90, 132.727 }, { 0, 1, 0 });
        robot.MoveL(Pose(60, 167.609, 15, -90, 0, 180), { 40.8254, -82.7381, -137.861, -49.4008, 90, 130.825 }, { 0, 1, 0 });
        robot.MoveL(Pose(60, 190.761, 15, -90, 0, 180), { 39.0789, -85.126, -135.831, -49.0434, 90, 129.079 }, { 0, 1, 0 });
        robot.MoveL(Pose(60, 213.913, 15, -90, 0, -180), { 37.4703, -87.4183, -133.75, -48.8315, 90, 127.47 }, { 0, 1, 0 });
        robot.MoveL(Pose(60, 237.065, 15, -90, 0, 180), { 35.9844, -89.6271, -131.623, -48.7503, 90, 125.984 }, { 0, 1, 0 });
        robot.MoveL(Pose(60, 260.217, 15, -90, 0, 180), { 34.6082, -91.7634, -129.449, -48.7873, 90, 124.608 }, { 0, 1, 0 });
        robot.MoveL(Pose(60, 283.37, 15, -90, 0, 180), { 33.3303, -93.8368, -127.232, -48.9315, 90, 123.33 }, { 0, 1, 0 });
        robot.MoveL(Pose(60, 306.522, 15, -90, 0, 180), { 32.1409, -95.8558, -124.971, -49.1736, 90, 122.141 }, { 0, 1, 0 });
        robot.MoveL(Pose(60, 329.674, 15, -90, 0, 180), { 31.0314, -97.8282, -122.666, -49.5057, 90, 121.031 }, { 0, 1, 0 });
        robot.MoveL(Pose(60, 352.826, 15, -90, 0, 180), { 29.9941, -99.7611, -120.318, -49.9211, 90, 119.994 }, { 0, 1, 0 });
        robot.MoveL(Pose(60, 375.978, 15, -90, 0, 180), { 29.0224, -101.661, -117.925, -50.414, 90, 119.022 }, { 0, 1, 0 });
        robot.MoveL(Pose(60, 399.13, 15, -90, 0, 180), { 28.1104, -103.533, -115.488, -50.9797, 90, 118.11 }, { 0, 1, 0 });
        robot.MoveL(Pose(60, 422.283, 15, -90, 0, 180), { 27.2529, -105.383, -113.003, -51.6143, 90, 117.253 }, { 0, 1, 0 });
        robot.MoveL(Pose(60, 445.435, 15, -90, 0, 180), { 26.4452, -107.216, -110.47, -52.3145, 90, 116.445 }, { 0, 1, 0 });
        robot.MoveL(Pose(60, 468.587, 15, -90, 0, 180), { 25.6832, -109.036, -107.886, -53.0781, 90, 115.683 }, { 0, 1, 0 });
        robot.MoveL(Pose(60, 491.739, 15, -90, 0, 180), { 24.9631, -110.849, -105.248, -53.903, 90, 114.963 }, { 0, 1, 0 });
        robot.MoveL(Pose(60, 514.891, 15, -90, 0, 180), { 24.2817, -112.659, -102.552, -54.7882, 90, 114.282 }, { 0, 1, 0 });
        robot.MoveL(Pose(60, 538.043, 15, -90, 0, 180), { 23.636, -114.471, -99.796, -55.7331, 90, 113.636 }, { 0, 1, 0 });
        robot.MoveL(Pose(60, 561.196, 15, -90, 0, 180), { 23.0232, -116.288, -96.9741, -56.7377, 90, 113.023 }, { 0, 1, 0 });
        robot.MoveL(Pose(60, 584.348, 15, -90, 0, 180), { 22.441, -118.116, -94.0812, -57.8027, 90, 112.441 }, { 0, 1, 0 });
        robot.MoveL(Pose(60, 607.5, 15, -90, 0, 180), { 21.8872, -119.96, -91.1109, -58.9296, 90, 111.887 }, { 0, 1, 0 });
        robot.MoveL(Pose(60, 607.5, 15, -94.452, 0, 180), { 22.3535, -119.191, -92.3555, -58.4535, 90, 116.805 }, { 0, 1, 0 });
        robot.MoveL(Pose(60.1808, 609.822, 15, -107.013, 0, 180), { 23.8465, -117.459, -95.1274, -57.4137, 90, 130.86 }, { 0, 1, 0 });
        robot.MoveL(Pose(61.419, 613.868, 15, -123.233, 0, 180), { 26.0971, -115.974, -97.4662, -56.56, 90, 149.33 }, { 0, 1, 0 });
        robot.MoveL(Pose(63.7383, 617.408, 15, -139.452, 0, 180), { 28.5172, -115.362, -98.4198, -56.2186, 90, 167.969 }, { 0, 1, 0 });
        robot.MoveL(Pose(66.9541, 620.159, 15, -155.671, 0, -180), { 30.825, -115.69, -97.9098, -56.4007, 90, 186.496 }, { 0, 1, 0 });
        robot.MoveL(Pose(70.8103, 621.903, 15, -171.89, 0, 180), { 32.7585, -116.921, -95.9785, -57.1004, 90, 204.648 }, { 0, 1, 0 });
        robot.MoveL(Pose(75, 622.5, 15, -171.89, 0, 180), { 32.5068, -116.843, -96.102, -57.0552, 90, 204.397 }, { 0, 1, 0 });
        robot.MoveL(Pose(75, 622.5, 15, -180, 0, 180), { 33.4037, -117.784, -94.6102, -57.6055, 90, 213.404 }, { 0, 1, 0 });
        robot.MoveL(Pose(99, 622.5, 15, -180, 0, -180), { 32.0944, -117.072, -95.7402, -57.1878, 90, 212.094 }, { 0, 1, 0 });
        robot.MoveL(Pose(123, 622.5, 15, -180, 0, -180), { 30.7527, -116.406, -96.7887, -56.805, 90, 210.753 }, { 0, 1, 0 });
        robot.MoveL(Pose(147, 622.5, 15, 180, 0, 180), { 29.3792, -115.788, -97.7559, -56.4558, 90, 209.379 }, { 0, 1, 0 });
        robot.MoveL(Pose(171, 622.5, 15, 180, 0, 180), { 27.9746, -115.218, -98.6419, -56.1396, 90, 207.975 }, { 0, 1, 0 });
        robot.MoveL(Pose(195, 622.5, 15, -180, 0, -180), { 26.5398, -114.698, -99.4467, -55.8555, 90, 206.54 }, { 0, 1, 0 });
        robot.MoveL(Pose(219, 622.5, 15, 180, 0, 180), { 25.0759, -114.227, -100.17, -55.6027, 90, 205.076 }, { 0, 1, 0 });
        robot.MoveL(Pose(243, 622.5, 15, -180, 0, -180), { 23.5842, -113.808, -100.812, -55.3804, 90, 203.584 }, { 0, 1, 0 });
        robot.MoveL(Pose(267, 622.5, 15, -180, 0, -180), { 22.0662, -113.44, -101.372, -55.1881, 90, 202.066 }, { 0, 1, 0 });
        robot.MoveL(Pose(291, 622.5, 15, -180, 0, -180), { 20.5236, -113.124, -101.851, -55.0251, 90, 200.524 }, { 0, 1, 0 });
        robot.MoveL(Pose(315, 622.5, 15, -180, 0, 180), { 18.9584, -112.862, -102.247, -54.8909, 90, 198.958 }, { 0, 1, 0 });
        robot.MoveL(Pose(315, 622.5, 15, 175.547, 0, -180), { 19.5507, -113.268, -101.632, -55.0994, 90, 204.003 }, { 0, 1, 0 });
        robot.MoveL(Pose(317.322, 622.319, 15, 162.987, 0, 180), { 20.8082, -114.725, -99.4054, -55.87, 90, 217.821 }, { 0, 1, 0 });
        robot.MoveL(Pose(321.368, 621.081, 15, 146.766, 0, -180), { 21.7473, -117.137, -95.6372, -57.2257, 90, 234.981 }, { 0, 1, 0 });
        robot.MoveL(Pose(324.908, 618.762, 15, 130.549, 0, -180), { 21.9944, -119.953, -91.1216, -58.9254, 90, 251.446 }, { 0, 1, 0 });
        robot.MoveL(Pose(327.66, 615.546, 15, 114.329, 0, -180), { 21.6053, -122.947, -86.1908, -60.8618, 90, 267.276 }, { 0, 1, 0 });
        robot.MoveL(Pose(329.403, 611.69, 15, 98.1094, 0, -180), { 20.6769, -125.904, -81.2023, -62.8937, 90, 282.568 }, { 0, 1, 0 });
        robot.MoveL(Pose(330, 607.5, 15, 98.1094, 0, -180), { 20.7306, -125.543, -81.8174, -62.6396, 90, 282.621 }, { 0, 1, 0 });
        robot.setSpeed(1000.000);
        robot.RunCode("TapeOff", true);
        robot.MoveL(Pose(330, 607.5, 115, 98.1094, 0, -180), { 20.7306, -121.808, -77.0554, -71.1368, 90, 282.621 }, { 0, 1, 0 });
        robot.RunCode("GoHome", true);
        robot.ProgFinish("AllPath");
        robot.ProgStart("GoHome");
        robot.setFrame(Pose(0, 0, 0, 0, 0, 0), -1, "UR10e Base");
        robot.MoveJ(Pose(0,0,0,0,0,0), { 11.3286, -55.3613, -116.557, -98.0816, 90, -258.671 }, {});
        robot.ProgFinish("GoHome");
 
        // 输出程序内容
        std::cout << "\nGenerated program content:" << std::endl;
        std::cout << "==========================" << std::endl;

        const std::vector<std::string> programLines = robot.getProgramLines();
        for (const auto& line : programLines) {
            std::cout << line << std::endl;
        }

        // 保存程序
        robot.ProgSave(path, "AllPath", false, "");
        std::cout << "Program generation completed successfully!" << std::endl;
        std::cout << "Output saved to: " << path << std::endl;

        // 如果有日志，显示日志
        if (!robot.getLog().empty()) {
            std::cout << "\nProgram generation LOG:" << std::endl;
            std::cout << robot.getLog() << std::endl;
        }
    }
}