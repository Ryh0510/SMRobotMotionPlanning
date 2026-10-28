#pragma once
#include <string>
#include <vector>
#include <memory>
#include <map>
#include <functional>
#include "robomath.h"
#include "PostInterface.h"
#include <fstream>    // 用于 std::ifstream
#include <sstream>    // 用于 std::stringstream
#include <string>
#pragma execution_character_set("utf-8")

namespace Universal_Robots_URP {
    class RobotPost {
    public:
        // 运动类型枚举
        enum class MoveType {
            MoveJ = 1,
            MoveL = 2,
            MoveP = 3,
            MoveC = 4
        };

        // 运行时状态枚举
        enum class RuntimeMode {
            Cancelled = 0,
            Ready = 1,
            Busy = 2
        };

        // 机器人状态枚举
        enum class RobotStatus {
            ProgramError = -1,
            NotConnected = 0,
            Ok = 1
        };

        // 配置参数
        struct Config {
            // 单个程序的最大行数限制。若超出此行数，程序将分步执行
            size_t maxLinesPerProgram = 5000;
            // 若设为 True，将尝试使用 SFTP 进行上传。需要安装 PYSFTP（通过 pip install pysftp 安装）。
            // 重要提示：此功能需要 Visual Studio Community C++ 环境支持
            bool uploadSFTP = false;
            // 文件扩展名：设置程序扩展名
            std::string programExtension = "script";
            // 线性运动的默认速度（米/秒）
            double speedMS = 0.250;
            // 关节运动的默认速度（弧度/秒）
            double speedRads = 0.75;
            // 线性运动的默认加速度（米/秒²）
            double accelMSS = 1.2003;
            // 关节运动的默认加速度（弧度/秒²）
            double accelRadSS = 1.1998;
            // 默认融合半径（单位：米），用于转角平滑处理
            double blendRadiusM = 0.001;
            // 输出圆弧运动的最小半径（单位：毫米）。此设置不考虑融合半径的影响
            double moveCMinRadius = 1.0;
            // 输出圆弧运动的最大半径（单位：毫米）。此设置不考虑融合半径的影响
            double moveCMaxRadius = 10000.0;
            // 设置为 True 时使用 MoveP 指令，设置为 False 时使用 MoveL 指令
            bool useMoveP = false;
            Config() = default;
        };

        // 构造函数
        RobotPost(const std::string robotPost,
            const std::string robotName,
            int robotAxes = 6,
            std::vector<std::string> axesType = std::vector<std::string>{ "R","R","R","R","R","R" },
            const std::string ip_com = "192.168.241.127",
            const Config config = Config());
        //RobotPost(const std::string robotPost, const std::string robotName);
        /*robotpost (str) – Name of the post processor
        robotname(str) – Name of the robot
        robot_axes(int) – Number of axes of the robot, including synchronized axes
        axes_type(list of str, optional) – Type of each axes of the robot(‘R’: Robot rotative, ‘T’ : Robot translation, ‘J’ : Ext.axis rotative, ‘L’ : Ext.axis linear)
        ip_com(str, optional) – IP address of the robot(“Connect” tab in RobotSDK)
         */
         // 程序控制
        void ProgStart(const std::string& progname);
        void ProgFinish(const std::string& progname);
        void ProgSave(const std::string& folder, const std::string& progname, bool askUser = false, const std::string& showResult = "");
        RobotStatus ProgSendRobot();

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
        // 添加重载方法接受本地Mat类型
        void setSpeed(double speed_mms);
        void setAcceleration(double accel_mmss);
        void setSpeedJoints(double speed_degs);
        void setAccelerationJoints(double accel_degss);
        void setZoneData(double zone_mm);

        // I/O 操作
        void setDO(const std::string& io_var, const std::string& io_value);
        void setAO(const std::string& io_var, const std::string& io_value);
        void waitDI(const std::string& io_var, const std::string& io_value,
            int timeout_ms = -1);

        // 代码生成
        void RunCode(const std::string& code, bool is_function_call = false);
        void RunMessage(const std::string& message, bool is_comment = false);
        void Pause(int time_ms);

        // 工具函数
        static std::string getSafeName(const std::string& progname);
        static std::string poseToString(const robomath::Mat& pose);
        static std::string anglesToString(const std::vector<double>& angles);

        // 网络通信
        bool UR_Wait_Ready(double percent_complete);
        RobotStatus UR_SendProgramRobot(const std::string& data);
        friend void test_post(const std::string& path);
    private:
        // 内部数据结构
        struct ProgramData {
            std::vector<std::string> lines;
            std::vector<std::string> vars;
            std::vector<std::string> urpLines;
            // 默认构造函数
            ProgramData() = default;
        };

        // 私有方法
        void vars_update();
        void prog_2_list();
        void addline(const std::string& newline);
        void add_urp(const std::string& newline);
        void update_MotionParameters();
        void add_urp_move(MoveType movetype, const robomath::Mat& pose,
            const std::vector<double>& joints,
            const std::vector<double>& confRLF, double blend_radius);
        void urp_move_close();
        double blend_radius_check(const robomath::Mat& pose_abs, double ratio_check = 0.5);
        void addlog(const std::string& newline);

        // 成员变量
        Config config_;
        std::string robotPost_;
        std::string robotName_;
        int robotAxes_;
        std::vector<std::string> axesType_;
        std::string ip_com_;

        robomath::Mat refFrame_;
        robomath::Mat toolFrame_;
        robomath::Mat toolFrameInv_;
        std::vector<double> lastPosAbs_;

        std::vector<std::string> progFiles_;
        std::string mainProgName_;

        int nProgs_;
        ProgramData currentProg_;
        std::vector<ProgramData> progList_;
        std::vector<ProgramData> varsList_;
        std::vector<std::string> subProg_;
        std::vector<std::string> subProgURP_;
        std::string footerURP_;

        int subProgCount_;
        std::vector<std::string> subProgNames_;
        MoveType lastMoveType_;
        int waypointCount_;
        std::string motionParameters_;

        std::string log_;
        std::string tab_;
        std::string tabURP_;
    };

    // 默认头文件脚本
    extern const std::string DEFAULT_HEADER_SCRIPT;
    extern const std::string SCRIPT_URP;

    // 模板注册函数
    extern "C" __declspec(dllexport) inline void register_Universal_Robots_URP(PostInterface & manager)
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
                        "src/Universal_Robots_URP.cpp";
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

        manager.registerTemplate("Universal_Robots_URP",
            headerpath,
            cpppath,
            "Universal_Robots_URP::RobotPost",
            headerContent, cppContent);
    }
}
