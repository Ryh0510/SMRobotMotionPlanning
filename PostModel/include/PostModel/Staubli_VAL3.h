#pragma once
#include <string>
#include <vector>
#include <memory>
#include <map>
#include <cmath>
#include "robomath.h"
#include <fstream>    // 用于 std::ifstream
#include <sstream>    // 用于 std::stringstream
#include <string>
#include "PostInterface.h"

namespace Staubli_VAL3 {
    class RobotPost {
    public:
        // 配置参数结构体
        struct Config {
            //设置每个程序的最大行数限制
            int PROG_MOVE_COUNT_MAX = 200;
            //默认速度
            double DEFAULT_SPEED = 150.0;
            //默认平滑度
            double DEFAULT_SMOOTH = 0.1;
            //默认笛卡尔加速度
            double ACCEL_MMSS = 1000.0;
            //默认关节速度  
            double SPEED_DEGS = 100.0;
            //默认关节加速度
            double ACCEL_DEGSS = 500.0;

            Config() = default;
        };

        // 构造函数
        RobotPost(const std::string robotPost, const std::string robotName,
            int robotAxes = 6,
            std::vector<std::string> axesType = std::vector<std::string>(6, "R"),
            const std::string ip_com = "192.168.241.127",
            const Config config = Config());

        // 程序控制
        void ProgStart(const std::string& progname);
        void ProgFinish(const std::string& progname);
        void ProgSave(const std::string& folder, const std::string& progname,
            bool askUser = false, const std::string& showResult = "");
        void ProgSendRobot(const std::string& remote_path,
            const std::string& ftp_user, const std::string& ftp_pass);

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

        // I/O 操作
        void setDO(const std::string& io_var, const std::string& io_value);
        void setAO(const std::string& io_var, const std::string& io_value);
        void waitDI(const std::string& io_var, const std::string& io_value, int timeout_ms = -1);

        // 代码生成
        void RunCode(const std::string& code, bool is_function_call = false);
        void RunMessage(const std::string& message, bool is_comment = false);
        void Pause(int time_ms);

        // 工具函数
        static std::string FilterName(const std::string& namefilter, char safechar = 'P',
            const std::vector<std::string>& reserved_names = {});
        static std::string pose2Str(const robomath::Mat& pose);
        static std::string angles2Str(const std::vector<double>& angles);
        static std::string pose2StrDtx(const robomath::Mat& pose);

        // 公共访问器
        const std::vector<std::string> getProgramLines() const;
        const std::string& getLog() const { return LOG; }
        // 友元函数声明
        friend void test_post(const std::string& path);
    private:
        // 内部数据结构
        struct ProgramData {
            std::vector<std::string> lines;
            std::string name;
            ProgramData() = default;
            ProgramData(const std::string& n) : name(n) {}
        };

        // 私有方法
        void addLine(const std::string& newline);
        void addLog(const std::string& newline);
        void controlProgSize();
        void closeModule();
        bool DirExists(const std::string& path);
        bool CreateDir(const std::string& path);
        std::string FilterNameS6(const std::string& name);
        void prog2List();
        // 成员变量
        Config config_;
        std::string ROBOT_POST;
        std::string ROBOT_NAME;
        int nAxes;
        std::string ip_com_;

        // XML模板常量
        static const std::string PROGRAM_PJX;
        static const std::string PROGRAM_PJX_CS8V7;
        static const std::string PROGRAM_PJX_CS8V6;
        static const std::string MODULES_ADD;
        static const std::string PROGRAM_DTX;
        static const std::string MOVELINE_PGX;
        static const std::string MOVECIRC_PGX;
        static const std::string MOVEJOINT_PGX;
        static const std::string MAIN_PGX;
        static const std::string START_PGX;
        static const std::string STOP_PGX;
        static const std::string LOAD_NEXT_ONE;
        static const std::string START_PGX_MAIN;
        static const std::string PROGRAM_PJX_MAIN;

        // 程序数据
        std::vector<ProgramData> PROG_PGX_LIST;
        std::vector<std::string> PROG_NAME_LIST;
        ProgramData currentProg_;
        std::string PROG_PGX;
        std::string LOG;

        // 状态变量
        std::string TAB_PGX;
        int PROG_MOVE_COUNT;
        std::string PROG_NAME;
        std::string MAIN_FOLDER;
        std::vector<std::string> PROG_FILES;

        // 当前位姿和工具
        robomath::Mat REF;
        robomath::Mat TOOL;
        int REF_COUNT;
        int TOOL_COUNT;

        // 配置状态
        double SPEED;
        int CONF_shoulder;
        int CONF_lowerarm;
        int CONF_flip;
        bool MOVEC_USED;
    };

    // 模板注册函数
    extern "C" __declspec(dllexport) inline void register_Staubli_VAL3(PostInterface & manager)
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
                        "src/Staubli_VAL3.cpp";
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

        manager.registerTemplate("Staubli_VAL3",
            headerpath,
            cpppath,
            "Staubli_VAL3::RobotPost",
            headerContent, cppContent);
    }
}
