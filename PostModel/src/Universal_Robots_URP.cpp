#include <PostModel/Universal_Robots_URP.h>
#include <PostModel/robomath.h>
#include <PostModel/ftpsend.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <algorithm>
#include <cmath>
#include <thread>
#include <chrono>
#include <stdexcept>
#include <cctype>
#include <vector>
#include <map>
#include <functional>
#include <windows.h>
#include <shellapi.h>
#pragma execution_character_set("utf-8")

namespace Universal_Robots_URP{
    using namespace robomath;
    #ifndef M_PI
    #define M_PI 3.14159265358979323846
    #endif

    // 常量定义
    const std::string DEFAULT_HEADER_SCRIPT = R"(
      #--------------------------
      # Add any default subprograms here
      # For example, to drive a gripper as a program call:
      # def Gripper_Open():
      #   ...
      # end
      #
      # Example to drive a spray gun:
      def SprayOn(value):
        # use the value as an output:
        DO_SPRAY = 5
        if value == 0:
          set_standard_digital_out(DO_SPRAY, False)
        else:
          set_standard_digital_out(DO_SPRAY, True)
        end
      end

      # Example to synchronize 2
      def Synchronize():
        # Use the following digital output to signal the state of the robot:
        DO_SYNC = 1
    
        # Use the following digital input to get the state of another robot:
        DI_SYNC = 1
    
        if (get_standard_digital_out(DO_SYNC) == get_standard_digital_in(DI_SYNC)):
          set_standard_digital_out(DO_SYNC, not (get_standard_digital_out(DI_SYNC)))
          sleep(0.1)
          thread Thread_wait_1():
            while (True):
              sleep(0.01)
            end
          end
          if (get_standard_digital_out(DO_SYNC) != get_standard_digital_in(DI_SYNC)):
            global thread_handler_1=run Thread_wait_1()
            while (get_standard_digital_out(DO_SYNC) != get_standard_digital_in(DI_SYNC)):
              sync()
            end
            kill thread_handler_1
          end
        else:
          if (get_standard_digital_out(DO_SYNC) != get_standard_digital_in(DI_SYNC)):
            set_standard_digital_out(DO_SYNC, not (get_standard_digital_out(DO_SYNC)))
          end
        end
      end

      #
      # Example to move an external axis
      def MoveAxis(value):
        # use the value as an output:
        DO_AXIS_1 = 1
        DI_AXIS_1 = 1
        if value <= 0:
          set_standard_digital_out(DO_AXIS_1, False)
      
          # Wait for digital input to change state
          #while (get_standard_digital_in(DI_AXIS_1) != False):
          #  sync()
          #end
        else:
          set_standard_digital_out(DO_AXIS_1, True)
      
          # Wait for digital input to change state
          #while (get_standard_digital_in(DI_AXIS_1) != True):
          #  sync()
          #end
        end
      end
      #--------------------------
    )";

    const std::string SCRIPT_URP = R"(<URProgram createdIn="3.0.0" lastSavedIn="3.0.0" name="%s" directory="/programs" installation="default">
      <children>
        <MainProgram runOnlyOnce="true" motionType="MoveJ" speed="1.0471975511965976" acceleration="1.3962634015954636" useActiveTCP="false">
          <children>
    %s      </children>
        </MainProgram>
    %s  </children>
    </URProgram>)";

    // 工具函数
    std::string RobotPost::getSafeName(const std::string& progname) {
        std::string result = progname;
        const std::string forbidden = "-[]/\\;,><&*:%=+@!#^|?^";
    
        for (char c : forbidden) {
            result.erase(std::remove(result.begin(), result.end(), c), result.end());
        }
    
        if (result.empty()) {
            result = "Program";
        }
    
        if (!result.empty() && std::isdigit(result[0])) {
            result = "P" + result;
        }
    
        return result;
    }

    std::string RobotPost::poseToString(const Mat& pose) 
    {
        // 简化的实现，实际需要完整的pose转字符串逻辑
        std::stringstream ss;
        ss << "p[";
        ss << std::fixed << std::setprecision(6) << pose(0, 3) * 0.001 << ", ";
        ss << std::fixed << std::setprecision(6) << pose(1, 3) * 0.001 << ", ";
        ss << std::fixed << std::setprecision(6) << pose(2, 3) * 0.001 << ", ";
        // 这里需要完整的旋转矩阵到欧拉角转换
        ss << "0.0, 0.0, 0.0]";
        return ss.str();
    }

    std::string RobotPost::anglesToString(const std::vector<double>& angles) 
    {
        std::stringstream ss;
        ss << "[";
        for (size_t i = 0; i < angles.size(); ++i) 
        {
            if (i > 0) ss << ", ";
            ss << std::fixed << std::setprecision(6) << angles[i] * M_PI/180.0;
        }
        ss << "]";
        return ss.str();
    }

    // 构造函数
    //RobotPost::RobotPost(const std::string& robotPost, const std::string& robotName, int robotAxes, const Config& config) : robotPost_(robotPost), robotName_(robotName),robotAxes_(robotAxes), config_(config)
    RobotPost::RobotPost(const std::string robotPost,
        const std::string robotName,
        int robotAxes,
        std::vector<std::string> axesType,
        const std::string ip_com,
        const Config config)
        : robotPost_(robotPost), robotName_(robotName), robotAxes_(robotAxes),
        axesType_(axesType), ip_com_(ip_com), config_(config)
    //RobotPost::RobotPost(const std::string robotPost, const std::string robotName): robotPost_(robotPost), robotName_(robotName), robotAxes_(6),axesType_({ "R", "R", "R", "R", "R", "R" }),ip_com_("192.168.241.127"), config_(Config())
    {
        // 初始化单位矩阵 - 使用Mat的默认构造函数会自动创建单位矩阵
        refFrame_ = Mat(); // 默认构造函数创建4x4单位矩阵
        toolFrame_ = Mat();
        toolFrameInv_ = Mat();
        nProgs_ = 0; 
        subProgCount_ = 0; 
        waypointCount_ = 0; 
        tab_ = ""; 
        tabURP_ = "";
        vars_update();
    }

    // 私有方法实现
    void RobotPost::vars_update() 
    {
        currentProg_.vars.clear();
        currentProg_.vars.push_back("global speed_ms    = " + std::to_string(config_.speedMS));
        currentProg_.vars.push_back("global speed_rads  = " + std::to_string(config_.speedRads));
        currentProg_.vars.push_back("global accel_mss   = " + std::to_string(config_.accelMSS));
        currentProg_.vars.push_back("global accel_radss = " + std::to_string(config_.accelRadSS));
        currentProg_.vars.push_back("global blend_radius_m = " + std::to_string(config_.blendRadiusM));
    }

    void RobotPost::prog_2_list() 
    {
        if (currentProg_.lines.size() > 1) {
            progList_.push_back(currentProg_);
            varsList_.push_back(currentProg_);
            currentProg_ = ProgramData();
            vars_update();
        }
    }

    void RobotPost::addline(const std::string& newline) 
    {
        if (nProgs_ <= 1) {
            if (currentProg_.lines.size() > config_.maxLinesPerProgram) {
                prog_2_list();
            }
            currentProg_.lines.push_back(tab_ + newline);
        } else {
            subProg_.push_back(tab_ + newline);
        }
    }

    void RobotPost::add_urp(const std::string& newline) {
        if (nProgs_ <= 1) {
            currentProg_.urpLines.push_back(tabURP_ + newline);
        } else {
            subProgURP_.push_back(tabURP_ + newline);
        }
    }

    void RobotPost::update_MotionParameters() {
        std::stringstream ss;
        ss << "<motionParameters";
        ss << " blendRadius=\"" << config_.blendRadiusM << "\"";
        ss << " jointSpeed=\"" << config_.speedRads << "\"";
        ss << " jointAcceleration=\"" << config_.accelRadSS << "\"";
        ss << " cartesianSpeed=\"" << config_.speedMS << "\"";
        ss << " cartesianAcceleration=\"" << config_.accelMSS << "\"";
        ss << "/>";
        motionParameters_ = ss.str();
    }

    void RobotPost::add_urp_move(MoveType movetype, const Mat& pose, 
                                         const std::vector<double>& joints, 
                                         const std::vector<double>& confRLF, 
                                         double blend_radius) {
        if (movetype != lastMoveType_) {
            urp_move_close();
        }

        if (lastMoveType_ == MoveType::MoveJ) {
            add_urp("<Move motionType=\"MoveJ\" speed=\"" + 
                    std::to_string(config_.speedRads) + "\" acceleration=\"" + 
                    std::to_string(config_.accelRadSS) + "\" useActiveTCP=\"true\">");
        } else if (movetype == MoveType::MoveL) {
            add_urp("<Move motionType=\"MoveL\" speed=\"" + 
                    std::to_string(config_.speedMS) + "\" acceleration=\"" + 
                    std::to_string(config_.accelMSS) + "\" useActiveTCP=\"true\">");
        } else if (movetype == MoveType::MoveP) {
            add_urp("<Move motionType=\"MoveP\" speed=\"" + 
                    std::to_string(config_.speedMS) + "\" acceleration=\"" + 
                    std::to_string(config_.accelMSS) + "\" blendRadius=\"" + 
                    std::to_string(config_.blendRadiusM) + "\" useActiveTCP=\"true\">");
        }

        add_urp("  <children>");
        lastMoveType_ = movetype;

        waypointCount_++;
        add_urp("    <Waypoint type=\"Fixed\" name=\"Waypoint_" + 
                std::to_string(waypointCount_) + "\">");

        if (!motionParameters_.empty()) {
            add_urp("      " + motionParameters_);
        } else if (movetype == MoveType::MoveL && config_.blendRadiusM > 0.0) {
            add_urp("      <motionParameters blendRadius=\"" + 
                    std::to_string(config_.blendRadiusM) + "\"/>");
        }

        if (pose == Mat()) {
            add_urp("      <position joints=\"" + anglesToString(joints).substr(1) + "\"/>");
        } else {
            Mat poseabs = refFrame_ * pose;
            std::string pose_str = poseToString(poseabs);
            add_urp("      <position joints=\"" + anglesToString(joints).substr(1) + 
                    "\" pose=\"" + pose_str.substr(2, pose_str.length()-3) + "\"/>");
            add_urp("      <poseInFeatureCoordinates pose=\"" + 
                    pose_str.substr(2, pose_str.length()-3) + "\"/>");
        }
        add_urp("    </Waypoint>");
    }

    void RobotPost::urp_move_close() {
        if (lastMoveType_ != MoveType::MoveJ) {
            add_urp("  </children>");
            add_urp("</Move>");
            lastMoveType_ = MoveType::MoveJ;
        }
    }

    double RobotPost::blend_radius_check(const Mat& pose_abs, double ratio_check) {
        double blend_radius = config_.blendRadiusM;
        if (lastPosAbs_.empty()) {
            blend_radius = 0;
        } else {
            // 简化的距离计算
            double dx = pose_abs(0, 3) - lastPosAbs_[0];
            double dy = pose_abs(1, 3) - lastPosAbs_[1];
            double dz = pose_abs(2, 3) - lastPosAbs_[2];
            double distance = std::sqrt(dx*dx + dy*dy + dz*dz);
        
            if (ratio_check * distance < config_.blendRadiusM * 1000) {
                blend_radius = std::round(ratio_check * distance * 0.001 * 1000) / 1000.0;
            }
        }
        return blend_radius;
    }

    void RobotPost::addlog(const std::string& newline) {
        log_ += newline + "\n";
    }

    // 公共方法实现
    void RobotPost::MoveJ(const Mat& pose, const std::vector<double>& joints, const std::vector<double>& confRLF) 
    {
        double blend_radius = 0;
        if (pose != Mat()) {
            Mat pose_abs = refFrame_ * pose;
            blend_radius = blend_radius_check(pose_abs);
            lastPosAbs_ = { pose_abs(0, 3),pose_abs(1, 3),
                          pose_abs(2, 3) };
        }

        if (joints.size() < 6) {
            RunMessage("Move axes to: " + anglesToString(joints), false);
        }

        RunCode("movej(" + anglesToString(joints) + "," + 
                std::to_string(config_.accelMSS) + "," + 
                std::to_string(config_.speedMS) + "," + 
                std::to_string(blend_radius) + ")", true);
    
        if (pose != Mat()) {
            add_urp_move(MoveType::MoveJ, pose, joints, confRLF, blend_radius);
        }
    }

    void RobotPost::MoveL(const Mat& pose, const std::vector<double>& joints, const std::vector<double>& confRLF) 
    {
        std::string target;
        double blend_radius = 0;

        if (pose==Mat()) {
            target = anglesToString(joints);
        } else {
            Mat pose_abs = refFrame_ * pose;
            blend_radius = blend_radius_check(pose_abs);
            target = poseToString(pose_abs);
            lastPosAbs_ = { pose_abs(0, 3),
                          pose_abs(1, 3),
                          pose_abs(2, 3) };
        }

        if (config_.useMoveP) {
            addline("movep(" + target + "," + 
                    std::to_string(config_.accelMSS) + "," + 
                    std::to_string(config_.speedMS) + "," + 
                    std::to_string(blend_radius) + ")");
            add_urp_move(MoveType::MoveP, pose, joints, confRLF, blend_radius);
        } else {
            addline("movel(" + target + "," + 
                    std::to_string(config_.accelMSS) + "," + 
                    std::to_string(config_.speedMS) + ",0," + 
                    std::to_string(blend_radius) + ")");
            add_urp_move(MoveType::MoveL, pose, joints, confRLF, blend_radius);
        }
    }

    void RobotPost::MoveC(const Mat& pose1, const std::vector<double>& joints1,
                                 const Mat& pose2, const std::vector<double>& joints2,
                                 const std::vector<double>& confRLF1,
                                 const std::vector<double>& confRLF2) {
        if (lastPosAbs_.empty()) {
            MoveL(pose2, joints2, confRLF2);
            return;
        }

        // 简化的圆弧半径计算
        Mat pose1_abs = refFrame_ * pose1;
        Mat pose2_abs = refFrame_ * pose2;
    
        double p0x = lastPosAbs_[0], p0y = lastPosAbs_[1], p0z = lastPosAbs_[2];
        double p1x = pose1_abs(0, 3), p1y = pose1_abs(1, 3), p1z = pose1_abs(2, 3);
        double p2x = pose2_abs(0, 3), p2y = pose2_abs(1, 3), p2z = pose2_abs(2, 3);
    
        double a = std::sqrt(std::pow(p0x-p1x,2) + std::pow(p0y-p1y,2) + std::pow(p0z-p1z,2));
        double b = std::sqrt(std::pow(p1x-p2x,2) + std::pow(p1y-p2y,2) + std::pow(p1z-p2z,2));
        double c = std::sqrt(std::pow(p2x-p0x,2) + std::pow(p2y-p0y,2) + std::pow(p2z-p0z,2));
    
        double radius = a * b * c / std::sqrt(std::pow(a*a+b*b+c*c,2) - 
                      2*(std::pow(a,4)+std::pow(b,4)+std::pow(c,4)));

        if (radius < config_.moveCMinRadius || radius > config_.moveCMaxRadius) {
            MoveL(pose2, joints2, confRLF2);
            return;
        }

        double blend_radius = blend_radius_check(pose1_abs, 0.2);
        lastPosAbs_ = { pose2_abs(0, 3),
                      pose2_abs(1, 3),
                      pose2_abs(2, 3) };

        RunMessage("Circular move (R" + std::to_string(radius) + " mm)", true);
        RunCode("movec(" + poseToString(pose1_abs) + "," + 
                poseToString(pose2_abs) + "," + 
                std::to_string(config_.accelMSS) + "," + 
                std::to_string(config_.speedMS) + "," + 
                std::to_string(blend_radius) + ")", true);
    }

    void RobotPost::setFrame(const Mat& pose, int frameId, 
                                    const std::string& frameName) {
        refFrame_ = pose;
        std::string ref_pose = poseToString(pose);
        urp_move_close();
        RunMessage("Using Ref. " + frameName + ": " + ref_pose, true);
        addline("# set_reference(" + ref_pose + ")");
    }

    void RobotPost::setTool(const Mat& pose, int toolId, const std::string& toolName) 
    {
        toolFrame_ = pose;
        // 需要根据Mat类的实现来求逆矩阵
        // 这里假设Mat类有inv()方法或者需要手动计算逆矩阵
        // 简化实现：暂时设为相同矩阵
        toolFrameInv_ = pose;
        std::string tcp_pose = poseToString(pose);
        urp_move_close();
        RunMessage("Using TCP " + toolName + ": " + tcp_pose, true);
        RunCode("set_tcp(" + tcp_pose + ")", true);
    }

    void RobotPost::setSpeed(double speed_mms) {
        config_.speedMS = speed_mms / 1000.0;
        addline("speed_ms    = " + std::to_string(config_.speedMS));
        urp_move_close();
        update_MotionParameters();
    }

    void RobotPost::setAcceleration(double accel_mmss) {
        config_.accelMSS = accel_mmss / 1000.0;
        addline("accel_mss   = " + std::to_string(config_.accelMSS));
        update_MotionParameters();
    }

    void RobotPost::setSpeedJoints(double speed_degs) {
        config_.speedRads = speed_degs * M_PI / 180.0;
        addline("speed_rads  = " + std::to_string(config_.speedRads));
        update_MotionParameters();
    }

    void RobotPost::setAccelerationJoints(double accel_degss) {
        config_.accelRadSS = accel_degss * M_PI / 180.0;
        addline("accel_radss = " + std::to_string(config_.accelRadSS));
        update_MotionParameters();
    }

    void RobotPost::setZoneData(double zone_mm) {
        if (zone_mm < 0) zone_mm = 0;
        config_.blendRadiusM = zone_mm / 1000.0;
        addline("blend_radius_m = " + std::to_string(config_.blendRadiusM));
    }

    void RobotPost::setDO(const std::string& io_var, const std::string& io_value) {
        std::string newline;
        if (std::isdigit(io_var[0])) {
            newline = "set_standard_digital_out(" + io_var + ", " + io_value + ")";
        } else {
            newline = io_var + " = " + io_value;
        }
        addline(newline);
    
        urp_move_close();
        add_urp("<Set type=\"DigitalOutput\">");
        add_urp("  <pin referencedName=\"" + io_var + "\"/>");
        add_urp("  <digitalValue>" + io_value + "</digitalValue>");
        add_urp("</Set>");
    }

    void RobotPost::setAO(const std::string& io_var, const std::string& io_value) {
        setDO(io_var, io_value);
    }

    void RobotPost::waitDI(const std::string& io_var, const std::string& io_value, 
                                  int timeout_ms) {
        std::string io_var_urp = io_var;
        if (std::isdigit(io_var[0])) {
            io_var_urp = "get_standard_digital_in(" + io_var + ")";
        }
    
        addline("while (" + io_var_urp + " != " + io_value + "):");
        addline("  sync()");
        addline("end");
    
        urp_move_close();
        add_urp("<Wait type=\"DigitalInput\">");
        add_urp("  <pin referencedName=\"" + io_var + "\"/>");
        add_urp("  <digitalValue>" + io_value + "</digitalValue>");
        add_urp("</Wait>");
    }

    void RobotPost::RunCode(const std::string& code, bool is_function_call) {
        if (is_function_call) {
            std::string clean_code = code;
            if (clean_code.substr(0, 1) == "#") {
                is_function_call = false;
            } else {
                if (!code.empty() && code.back() != ')') {
                    clean_code += "()";
                }
                addline(clean_code);
            
                std::string progname = clean_code.substr(0, clean_code.find('('));
                urp_move_close();
            
                subProgCount_++;
                if (std::find(subProgNames_.begin(), subProgNames_.end(), progname) == subProgNames_.end()) {
                    add_urp("<CallSubProgram>");
                    add_urp("  <subprogram name=\"" + progname + "\" keepHidden=\"false\" keepSynchronizedWithDisk=\"true\" trackProgramExecution=\"false\">");
                    add_urp("    <programFile resolves-to=\"file\">/programs/" + progname + ".urp</programFile>");
                    add_urp("    <children>");
                    add_urp("    </children>");
                    add_urp("  </subprogram>");
                    add_urp("</CallSubProgram>");
                
                    subProgNames_.push_back(progname);
                    std::string str_count = (subProgCount_ > 1) ? "[" + std::to_string(subProgCount_) + "]" : "";
                    footerURP_ += "  <SubProgram reference=\"../MainProgram/children/CallSubProgram" + str_count + "/subprogram\"/>\n";
                }
            }
        } else {
            addline(code);
            urp_move_close();
            add_urp("<Script type=\"Line\">");
            add_urp("  <expression>");
            for (char c : code) {
                add_urp("    <ExpressionChar character=\"" + std::string(1, c) + "\"/>");
            }
            add_urp("  </expression>");
            add_urp("</Script>");
        }
    }

    void RobotPost::RunMessage(const std::string& message, bool is_comment) {
        if (is_comment) {
            addline("# " + message);
            urp_move_close();
            add_urp("<Comment comment=\"" + message + "\"/>");
        } else {
            addline("popup(\"" + message + "\",\"Message\",False,False,blocking=True)");
            urp_move_close();
            add_urp("<Popup type=\"Message\" haltProgram=\"false\" message=\"" + message + "\"/>");
        }
    }

    void RobotPost::Pause(int time_ms) {
        if (time_ms <= 0) {
            addline("halt() # reimplement this function to force stop");
            urp_move_close();
            add_urp("<Halt/>");
        } else {
            addline("sleep(" + std::to_string(time_ms * 0.001) + ")");
            urp_move_close();
            add_urp("<Wait type=\"Sleep\">");
            add_urp("  <waitTime>" + std::to_string(time_ms * 0.001) + "</waitTime>");
            add_urp("</Wait>");
        }
    }

    // 网络通信方法（简化实现）
    bool RobotPost::UR_Wait_Ready( double percent_complete) {
        // 简化实现 - 实际需要socket通信
        std::cout << "Connecting to robot " << ip_com_ << ":30003" << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(2));
        std::cout << "Robot Ready (transfer " << percent_complete << "% completed)" << std::endl;
        return true;
    }

    RobotPost::RobotStatus RobotPost::UR_SendProgramRobot(const std::string& data) 
    {
        // 简化实现 - 实际需要socket通信
        std::cout << "Sending program to robot " << ip_com_ << std::endl;
        std::cout << "Program size: " << data.size() << " bytes" << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(1));

        // 模拟可能的错误情况
        if (ip_com_.empty())
        {
            std::cout << "POPUP: Robot connection problems..." << std::endl;
            return RobotPost::RobotStatus::NotConnected;
        }
        std::cout << "POPUP: Program sent. The program should be running on the robot." << std::endl;
        return RobotPost::RobotStatus::Ok;
    }

    RobotPost::RobotStatus RobotPost::ProgSendRobot()
    {
        //构建程序字符串
        //std::stringstream send_str;
        std::stringstream send_str;
        send_str << "def " << mainProgName_ << "():\n";
        send_str << "  # Global parameters:\n";
        for (const auto& var : currentProg_.vars)
        {
            send_str << "  " << var << "\n";
        }
        send_str << "  \n";

        // 添加自定义头文件
        send_str << DEFAULT_HEADER_SCRIPT;
        send_str << "  \n";

        // 添加子程序

        for (const auto& subline : subProg_)
        {
            send_str << "  " << subline << "\n";
        }
        send_str << "  \n";

        // 添加主代码
        send_str << "  # Main program:\n";
        for (const auto& prog : progList_)
        {
            for (const auto& progline : prog.lines)
            {
                send_str << "  " << progline << "\n";
            }
        }

        send_str << "end\n\n";
        send_str << mainProgName_ << "()\n";

        // 转换为字节数据
        //std::string program_content = send_str.str();
        std::string program_content = send_str.str();
        // 等待机器人准备就绪
        //int nprogs = static_cast<int>(progList_.size());
        int nprogs = static_cast<int>(progList_.size());
        RobotPost::RobotStatus status = RobotPost::RobotStatus::Ok;
    
        for (int i = 0; i < nprogs; ++i)
        {
            if (i > 0)
            {
                while (!UR_Wait_Ready(i * 100.0 / nprogs))
                {
                    std::cout << "POPUP: Connect robot to run the program..." << std::endl;
                    std::this_thread::sleep_for(std::chrono::seconds(2));
                }
            }

            // 发送程序到机器人
            do 
            {
                status = UR_SendProgramRobot(program_content);
                if (status == RobotPost::RobotStatus::NotConnected)
                {
                    std::cout << "POPUP: Connect robot to transfer program..." << std::endl;
                    std::this_thread::sleep_for(std::chrono::seconds(2));
                }
            } while (status == RobotPost::RobotStatus::NotConnected);

            if (status != RobotPost::RobotStatus::Ok)
            {
                return status;
            }
        }
        return RobotPost::RobotStatus::Ok;
    }

    void RobotPost::ProgStart(const std::string& progname) {
        std::string safeName = getSafeName(progname);
        nProgs_++;
        subProgCount_ = 0;
        subProgNames_.clear();

        if (nProgs_ <= 1) {
            tab_ = "";
            // 创建全局变量
            vars_update();
            mainProgName_ = safeName;
        }
        else {
            addline("# Subprogram " + safeName);
            addline("def " + safeName + "():");
            tab_ = "  ";
        }
    }

    void RobotPost::ProgFinish(const std::string& progname) {
        urp_move_close();
        std::string safeName = getSafeName(progname);
        tab_ = "";

        if (nProgs_ <= 1) {
            addline("# End of main program");
        }
        else {
            addline("end");
            addline("");
        }
    }

    void RobotPost::ProgSave(const std::string& folder, const std::string& progname, bool askUser, const std::string& showResult)
    {
        std::string safeName = getSafeName(progname);
        std::string filename = safeName + ".script";
        std::string filepath = folder + "/" + filename;

        // 确保程序数据已处理
        prog_2_list();
        // 写入脚本文件
        std::ofstream fid(filepath);
        if (!fid.is_open())
        {
            throw std::runtime_error("无法打开文件: " + filepath);
        }
        // 创建主程序调用
        // 调试输出，确认这行代码执行了
        std::cout << "Writing program header..." << std::endl;
        fid << "def " << mainProgName_ << "():\n";
        // 添加全局参数
        fid << "  # Global parameters:\n";
        if (!varsList_.empty() && !varsList_[0].vars.empty())
        {
            for (const auto& varLine : varsList_[0].vars)
            {
                fid << "  " << varLine << "\n";
            }
        }
        else if (!currentProg_.vars.empty())
        {
            for (const auto& varLine : currentProg_.vars) {
                fid << "  " << varLine << "\n";
            }
        }

        // 添加自定义头文件
        fid << DEFAULT_HEADER_SCRIPT;
        fid << "  \n";
        for (const auto& subline : subProg_)
        {
            fid << "  " << subline << "\n";
        }
        fid << "  \n";
        // 添加主代码
        std::cout << "Writing program header..." << std::endl;
        fid << "  # Main program:\n";
        if (!progList_.empty())
        {
            for (const auto& prog : progList_)
            {
                for (const auto& progLine : prog.lines)
                {
                    fid << "  " << progLine << "\n";
                }
            }
        }
        else {
            for (const auto& progLine : currentProg_.lines)
            {
                fid << "  " << progLine << "\n";
            }
        }

        fid << "end\n\n";
        fid << mainProgName_ << "()\n";
        fid.close();

        std::cout << "SAVED: " << filepath << "\n";
        progFiles_.push_back(filepath);

        // ---------------------------- 保存URP文件 --------------------------
        std::string filesaveUrp = filepath.substr(0, filepath.find_last_of('.'));
        std::string urpContent;

        // 添加URP内容
        for (const auto& urpLine : currentProg_.urpLines)
        {
            urpContent += urpLine + "\n";
        }

        // 构建URP XML内容
        char buffer[4096];
        snprintf(buffer, sizeof(buffer), SCRIPT_URP.c_str(), mainProgName_.c_str(), urpContent.c_str(), footerURP_.c_str());
        std::string progXml = buffer;

        // 写入XML文件
        std::string xmlPath = filesaveUrp + ".xml";
        std::ofstream xmlFile(xmlPath);
        if (xmlFile.is_open())
        {
            xmlFile << progXml;
            xmlFile.close();
            std::cout << "SAVED XML: " << xmlPath << "\n";
        }

        // 保存URP文件
        std::string urpPath = filesaveUrp + ".urp";
        std::ofstream urpFile(urpPath);
        if (urpFile.is_open())
        {
            urpFile << progXml;
            urpFile.close();
            std::cout << "SAVED URP: " << urpPath << "\n";
            progFiles_.push_back(urpPath);
        }

        // 打开结果文件（如果指定了显示结果）
        if (!showResult.empty())
        {
            std::cout << "Would open with: " << showResult << "\n";

            // 跨平台的文件打开方式
           #ifdef _WIN32
             ShellExecuteA(NULL, "open", filepath.c_str(), NULL, NULL, SW_SHOWNORMAL);
           #elif __APPLE__
             std::string command = "open \"" + filepath + "\"";
             system(command.c_str());
           #elif __linux__
             std::string command = "xdg-open \"" + filepath + "\"";
             system(command.c_str());
           #else
             std::cout << "File opening not supported on this platform" << std::endl;
           #endif
        }
        // 显示日志信息
        if (!log_.empty())
        {
            std::cout << "Program generation LOG:\n\n" << log_ << "\n";
        }
    }

    void test_post(const std::string& path) 
    {
        auto Pose = robomath::PosePP;
        std::cout << "Starting Universal Robots URP Post Processorobot..." << std::endl;

        // 正确调用构造函数
        //RobotPost robot(robotPost, robotName, 6, axesType, ipCom, config);
        RobotPost robot("Universal_Robots_URP", "UR10");
        // 开始程序生成
        robot.ProgStart("MainProg");
        robot.RunMessage("Program generated by RobotSDK for UR10", true);
        robot.RunMessage("Using nominal kinematics.", true);

        // 添加运动指令
        robot.RunCode("ApproachMove", true);
        robot.RunCode("SprayOn(1)", true);
        robot.RunCode("PaintTop", true);
        robot.RunCode("SprayOn(0)", true);
        robot.RunCode("Retract", true);

        // 完成程序生成
        robot.ProgFinish("MainProg");
        robot.ProgStart("ApproachMove");
        // 设置参考坐标系和工具坐标系
        robot.setFrame(Pose(-800.000000, 500.000000, 100.000000, 180.000000, 0.000000, 0.000000), 2, "Frame 2"); //2坐标系序号
        robot.setTool(Pose(50.000000, 0.000000, 450.000000, 0.000000, 30.000000, 0.000000), -1, "Paint gun");

        // 设置速度
        robot.MoveJ(Pose(66.7612, 615.117, 413.712, -178.178, 13.352, 179.141), { -151.986, -78.2225, -86.6417, -64.6022, 73.0619, 110.032 }, { 0, 1, 0 });
        robot.MoveJ(Pose(0, 0, 200, 180, 0, 180), { -208.691, -96.3136, -82.1526, -64.6733, 103.889, 64.6406 }, { 0, 1, 0 });
        robot.ProgFinish("ApproachMove");
        robot.ProgStart("PaintTop");
        robot.MoveJ(Pose(31.0721, 24.785, 35.3552, 165.193, -11.1497, 143.813), { -219.536, -114.993, -76.9458, -70.1368, 127.814, 81.2666 }, { 0, 1, 0 });
        robot.setSpeed(50.000);
        robot.MoveL(Pose(355.2, 25.8154, 36.1499, 169.477, -8.01712, 143.099), { -210.289, -143.985, -30.8176, -80.4272, 128.618, 88.0677 }, { 0, 1, 0 });
        robot.MoveL(Pose(364.047, 124.576, 30.8315, 162.952, 5.37625, -163.01), { -179.629, -118.486, -75.3081, -44.8851, 65.5622, 96.6411 }, { 0, 1, 0 });
        robot.MoveL(Pose(29.8829, 140.332, 26.469, 157.13, 6.06692, -165.933), { -177.894, -88.3544, -112.979, -39.1892, 64.0089, 104.945 }, { 0, 1, 0 });
        robot.MoveL(Pose(29.2498, 262.962, 27.3698, 168.185, -3.20484, 165.037), { -194.064, -91.5522, -105.349, -45.9171, 103.957, 95.5589 }, { 0, 1, 0 });
        robot.MoveL(Pose(379.268, 259.84, 26.5159, 171.754, -2.24614, 164.867), { -190.188, -122.622, -65.0462, -54.1053, 103.992, 95.9442 }, { 0, 1, 0 });
        robot.MoveL(Pose(357.907, 376.604, 34.2839, 173.791, 4.76792, -142.618), { -157.813, -118.488, -80.8979, -50.2913, 43.3912, 100.711 }, { 0, 1, 0 });
        robot.MoveL(Pose(25.9867, 366.613, 39.8246, 169.748, 4.87183, -154.848), { -153.215, -84.8372, -115.212, -52.7169, 50.6543, 115.012 }, { 0, 1, 0 });
        robot.ProgFinish("PaintTop");
        robot.ProgStart("Retract");
        robot.MoveL(Pose(25.987, 366.613, 169.825, 169.747, 4.87184, -154.848), { -153.215, -81.8965, -104.364, -66.5061, 50.6545, 115.012 }, { 0, 1, 0 });
        robot.MoveJ(Pose(66.7612, 615.117, 413.712, -178.178, 13.352, 179.141), { -151.986, -78.2225, -86.6417, -64.6022, 73.0619, 110.032 }, { 0, 1, 0 });
        robot.ProgFinish("Retract");
        for (const auto& prgi : robot.progList_) {
            for (const auto& line : prgi.lines) {
                std::cout << line << std::endl;
            }
            std::cout << std::endl << std::endl;
        }
        // 保存程序
        robot.ProgSave(path, "MainProg", false, "");
        std::cout << "Program generation completed successfully!" << std::endl;
    }
}
    