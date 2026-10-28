#pragma once
#include <string>
#include <vector>
#include <map>
#include <memory>
#include "robomath.h"
#include "PostInterface.h"
#include <fstream>    // 用于 std::ifstream
#include <sstream>    // 用于 std::stringstream
#include <iostream>
#include <windows.h> 
#include <ctime>
#include <chrono>
#include <sstream>
#pragma execution_character_set("utf-8")

namespace Motoman_Cartesian {
  class RobotPost {
    public:
        // 配置参数结构体
        struct Config {
            // 每个程序的最大行数限制,程序行数超过此限制时会自动分页（生成多个文件）
            int MAX_LINES_X_PROG = 2000;  
            //程序名字最大字符数
            int MAX_NAME_CHARS = 7;
            // 是否使用SETTOOL设置工具,设置为True时，JBI程序中将使用SETTOOL指令设置工具坐标,此项需启用安川提供的SETTOOL选配功能（付费选项）
            bool USE_SETTOOL = true; 
            // 激活的工具ID,指定默认使用的工具坐标系编号（寄存器编号）,也可使用数字编号工具（例如命名为"Tool 2"的工具将自动使用UTOOL 2）
            int ACTIVE_TOOL = 9;  
            // 是否使用相对作业,设置为False时始终使用脉冲坐标编程,若设为True需启用安川Relative Job选配功能（付费选项）
            bool USE_RELATIVE_JOB = true;    
            // 强制将关节运动转换为笛卡尔空间运动
            bool MOVEJ_IN_CARTESIAN = true;    
            // 是否生成子程序
            bool INCLUDE_SUB_PROGRAMS = true;  
            // 备用位置寄存器,指定用于计算的备用位置寄存器编号（工具/工件坐标系计算等）
            int SPARE_PR = 95;  
            // 是否使用MFRAME设置参考坐标系,设置为True时，在程序中自动使用MFRAME指令设置工件坐标系
            bool USE_MFRAME = false;   
            // 激活的坐标系ID,指定默认使用的工件坐标系编号（寄存器编号）,可使用数字编号参考系（例如命名为"Reference 4"的参考系将自动使用UFRAME 4）
            int ACTIVE_FRAME = 9;    
            // 外部轴是否使用EC变量,设置外部轴是否定义为独立变量单元（使用EC寄存器而非扩展默认的C寄存器）
            bool EXTAXES_USE_EC = true; 
            // 设置外部轴是否需通过独立的MOVJ指令运动,为True时输出格式：MOVL C00008 V=166.7 +MOVJ EC00008 VJ=100.00反之输出格式：MOVL C00008 EC00008 V = 166.7
            bool EXTAXES_USE_MOVJ = true;     
            // 在此指定各外部轴（第7/8轴及后续轴）的脉冲/角度换算比例
            std::vector<double> PULSES_X_DEG = { 1,1,1,1,1,1,1,1,1,1 };
            // 特定轴交换功能选项,当前为已交换后的最终轴序,[0,1,2,3,4,5,7,6]交换第7和第8轴（外部轴1-2）,数字对应轴
            std::vector<int> AXIS_INDEX = { 0,1,2,3,4,5,6,7,8,9,10,11,12,13,14 };
            // 速度单位是否为cm/min,通常为否
            bool SPEED_CM_MIN = false;  
            // 是否使用SMOVL
            bool EXTAXES_USE_SMOVL = false;  

            Config() = default;
        };

        // 构造函数
        RobotPost(const std::string robotPost, const std::string robotName,
            int robotAxes = 6,
            std::vector<std::string> axesType = std::vector<std::string>(6, "R"),
            const std::string ip_com = "192.168.241.127",
            const Config config = Config());

        // 程序控制
        void ProgStart(const std::string& progname, bool new_page = false);
        void ProgFinish(const std::string& progname, bool new_page = false);
        void ProgSave(const std::string& folder, const std::string& progname,
            bool askUser = false, const std::string& showResult = "");
        void ProgSendRobot(const std::string& remote_path, const std::string& ftp_user, const std::string& ftp_pass);

        // 运动指令
        void MoveJ(const robomath::Mat& pose, const std::vector<double>& joints,
            const std::vector<double>& confRLF = {});
        void MoveL(const robomath::Mat& pose, const std::vector<double>& joints,
            const std::vector<double>& confRLF = {});
        void MoveC(const robomath::Mat& pose1, const std::vector<double>& joints1,
            const robomath::Mat& pose2, const std::vector<double>& joints2,
            const std::vector<double>& confRLF1 = {},
            const std::vector<double>& confRLF2 = {});

        // 设置函数
        void setFrame(const robomath::Mat& pose, int frameId = -1, const std::string& frameName = "");
        void setTool(const robomath::Mat& pose, int toolId = -1, const std::string& toolName = "");
        void setSpeed(double speed_mms);
        void setAcceleration(double accel_mmss);
        void setSpeedJoints(double speed_degs);
        void setAccelerationJoints(double accel_degss);
        void setZoneData(double zone_mm);
        // 设置脉冲系数函数
        void setPulsesXDeg(const std::vector<double>& pulses); 

        // I/O 操作
        void setDO(const std::string& io_var, const std::string& io_value);
        void setAO(const std::string& io_var, const std::string& io_value);
        void waitDI(const std::string& io_var, const std::string& io_value, int timeout_ms = -1);

        // 代码生成
        void RunCode(const std::string& code, bool is_function_call = false);
        void RunMessage(const std::string& message, bool is_comment = false);
        void Pause(int time_ms);

        // Motoman特定功能
        void Macro(int number, const std::string& mf, const std::vector<double>& args);
        void Arcon(int asf_number = 0);
        void Arcof(int aef_number = 0);

        // 焊接相关功能
        void ArcStart(const std::string& data = "");
        void ArcContinue(const std::string& data = "");
        void ArcEnd(const std::string& data = "");
        void Arcset(int asf_number, int acond_number = 0);
        // 工具函数
        std::string getSafeName(const std::string& name);
        static std::string joinStrings(const std::vector<std::string>& strings, const std::string& delimiter);
        //友元函数
        friend void test_post(std::string path);
        //时间函数
        std::string getCurrentTimestamp() {
            auto now = std::chrono::system_clock::now();
            auto time_t = std::chrono::system_clock::to_time_t(now);

            std::tm tm = *std::localtime(&time_t);
            char buffer[80];
            std::strftime(buffer, sizeof(buffer), "%d/%m/%Y %H:%M:%S", &tm);

            return std::string(buffer);
        }
    private:
        // 内部数据结构
        struct ProgramData {
            std::vector<std::string> lines;
            std::vector<std::string> targets;
            std::vector<std::string> targetsEC;
            std::string name;
            int lineCount = 0;
            int pCount = 0;
            int cCount = 0;
            int ecCount = 0;
            std::vector<std::string> targetsBC;  // 新增：直线轴目标点
            int bcCount = 0;  // 新增：直线轴计数
            ProgramData() = default;
            ProgramData(const std::string& n) : name(n) {}
        };


        // 私有方法
        void addLine(const std::string& newline, char moveType = ' ');
        void addLineTargets(const std::string& newline);
        void addLineTargetsEC(const std::string& newline);
        void addLog(const std::string& newline);
        void addLineTargetsBC(const std::string& newline);  // 新增：添加直线轴目标点
        void pageSizeControl(int reserved_lines = 0);
        std::tuple<int, int, int> addTargetJoints(const std::vector<double>& joints);
        std::tuple<int, int, int> addTargetCartesian(const robomath::Mat& pose, const std::vector<double>& joints,
            const std::vector<double>& confRLF = {});
        void saveProgram(const std::string& folder, const std::string& progname,
            bool askUser, const std::string& showResult);
        void setCartesian(const std::string& confdata);
        void setPulses();
        static std::vector<double> Pose_2_Motoman(const robomath::Mat& pose);

        // 添加缺失的成员变量
        std::string progName_ = "unknown";           // 对应Python的PROG_NAME
        std::string progNameCurrent_ = "unknown";    // 对应Python的PROG_NAME_CURRENT
        int nPages_ = 0;                            // 对应Python的nPages
        std::vector<std::string> progNamesMain_;     // 对应Python的PROG_NAMES_MAIN

        // 成员变量
        Config config_;
        std::string robotPost_;
        std::string robotName_;
        int robotAxes_;
        std::vector<std::string> axesType_;
        std::string ip_com_;
        std::vector<int> axesTrack_;
        std::vector<int> axesTurntable_;
        bool hasTrack_ = false;
        bool hasTurntable_ = false;

        // 程序数据
        std::vector<ProgramData> progList_;
        std::vector<std::string> progNames_;
        std::vector<std::string> progFiles_;
        ProgramData currentProg_;
        int nProgs_ = 0;
        int lblIdCount_ = 0;  // 新增：标签ID计数

        // 状态变量
        std::string log_;
        std::string progExt_ = "JBI";
        std::string strV_ = "V=100.0";
        std::string strVJ_ = "VJ=50.00";
        std::string externAxeSpeed_ = "VJ=100.00";  // 新增：外部轴速度
        std::string strPL_ = "";
        std::string strA_ = "";    // 新增：加速度
        std::string strAJ_ = "";   // 新增：关节加速度
        int registerDigits_ = 5;
    
        robomath::Mat lastPose_;
        std::vector<double> lastJoints_;
        int lastMoveType_ = -1;
        robomath::Mat poseFrame_;
        std::vector<std::string> lastConfData_ = {"", "", "", ""}; // [pulses, base, tool, config]
        std::string speedBackup_;
        int arcWeldData_ = 0;     // 新增：焊接数据
        int arcCondData_ = 0;     // 新增：条件数据
    };

    // 常量定义
    extern const std::vector<std::string> MOTOMAN_RESERVED_NAMES;

    // 模板注册函数
    extern "C" __declspec(dllexport) inline void register_Motoman_Cartesian(PostInterface & manager)
    {
        // 读取源文件内容
        auto calculateCppPath = [](const std::string& header) -> std::string {
            // 假设固定目录结构，直接替换扩展名和部分路径
            std::string cppPath = header;

            // 找到最后一个斜杠位置
            size_t lastSlash = cppPath.find_last_of("\\/");
            if (lastSlash != std::string::npos) {
                // 回溯到项目根目录（假设结构固定）
                size_t rootPos = cppPath.find_last_of("\\/", lastSlash - 1);
                rootPos = cppPath.find_last_of("\\/", rootPos - 1);
                

                if (rootPos != std::string::npos) {
                    cppPath = cppPath.substr(0, rootPos + 1) +
                        "src/Motoman_Cartesian.cpp";
                }
            }

            return cppPath;
        };

        const std::string headerpath = __FILE__;
        const std::string cpppath = calculateCppPath(headerpath);
        std::cout << "headerpath:" << headerpath << std::endl;
        std::cout << "cpppath:" << cpppath << std::endl;

        std::stringstream buffer;
        std::ifstream file(headerpath);
        file.is_open();
        buffer << file.rdbuf();
        file.close();
        std::string headerContent = buffer.str();
        buffer.clear();

        std::stringstream bufferCpp;
        std::ifstream file1(cpppath);
        file1.is_open();
        bufferCpp << file1.rdbuf();
        file1.close();
        std::string cppContent = bufferCpp.str();
        bufferCpp.clear();

        manager.registerTemplate("Motoman_Cartesian",
            headerpath,
            cpppath,
            "Motoman_Cartesian::RobotPost",
            headerContent, cppContent);
    }
}
