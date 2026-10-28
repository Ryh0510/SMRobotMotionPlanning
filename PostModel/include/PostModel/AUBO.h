#pragma once
#include <string>
#include <vector>
#include <map>
#include <cmath>
#include <algorithm>
#include <memory>
#include "robomath.h"
#include "PostInterface.h"
#include <fstream>    // 用于 std::ifstream
#include <sstream>    // 用于 std::stringstream
#include <string>

namespace AUBO {
    class RobotPost {
    public:
        // 配置参数结构体
        struct Config {
            //设置程序文件扩展名
            std::string PROG_EXT = "aubo";
            //线性运动的默认速度（米/秒）
            double SPEED_MS = 0.2501;
            //关节运动的默认速度（弧度/秒）  
            double SPEED_RADS = 0.3;
            // 线性运动的默认加速度（米/秒²）
            double ACCEL_MSS = 0.25;
            //关节运动的默认加速度（弧度/秒²）
            double ACCEL_RADSS = 1.1;
            //用户设置的当前速度（线性）
            double SPEED_MS_CURR = SPEED_MS;
            //用户设置的当前速度（关节）
            double SPEED_RADS_CURR = SPEED_RADS;
            //用户设置的当前加速度（线性）
            double ACCEL_MSS_CURR = ACCEL_MSS;
            //用户设置的当前加速度（关节）
            double ACCEL_RADSS_CURR = ACCEL_RADSS;
            //默认混合半径（米），用于拐角平滑
            double BLEND_RADIUS_M = 0.001;
            // 默认参考坐标系（机器人参考坐标系）
            robomath::Mat REF_FRAME = robomath::eye(4);
            //上一次的XYZ位置
            robomath::Mat LAST_POS;

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
        static std::string joints2Str(const std::vector<double>& angles);
        static std::string new_uuid();

        // 测试函数
        friend void test_post(std::string path);

        // 公共变量用于测试
        std::vector<std::string> PROG;
        std::vector<std::string> PROG_XML;
        std::string LOG;

    private:
        // 内部数据结构
        struct ProgramData
        {
            std::vector<std::string> lines;
            std::string name;
            ProgramData() = default;
            ProgramData(const std::string& n) : name(n) {}
        };

        // 私有方法
        void addLine(const std::string& newline);
        void addXML(const std::string& newblock);
        void addLog(const std::string& newline);
        void saveProgram(const std::string& folder, const std::string& progname,
            bool askUser, const std::string& showResult);

        // 成员变量
        Config config_;
        std::string robotPost_;
        std::string robotName_;
        int robotAxes_;
        std::vector<std::string> axesType_;
        std::string ip_com_;

        // 程序数据
        std::vector<ProgramData> progList_;
        std::vector<std::string> progNames_;
        std::vector<std::string> progFiles_;
        ProgramData currentProg_;
        int nProgs_;
        int nLines_;

        // 状态变量
        std::string TAB_;
        bool MAIN_DONE_;

        // XML 常量定义
        static const std::string XML_HEADER;
        static const std::string XML_CLOSING;
        static const std::string XML_MOVEJ;
        static const std::string XML_MOVEL;
        static const std::string XML_SETDO;
        static const std::string XML_WAITDI;
        static const std::string XML_PAUSE;
    };
    
    // 模板注册函数
    extern "C" __declspec(dllexport) inline void register_AUBO(PostInterface & manager)
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
                        "src/AUBO.cpp";
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

        manager.registerTemplate("AUBO",
            headerpath,
            cpppath,
            "AUBO::RobotPost",
            headerContent, cppContent);
    }
}
