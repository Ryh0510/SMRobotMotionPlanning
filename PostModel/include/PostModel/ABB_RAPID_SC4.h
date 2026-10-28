#pragma once
#include <string>
#include <vector>
#include <map>
#include <memory>
#include "robomath.h"
#include "PostInterface.h"
#include <fstream>    // 用于 std::ifstream
#include <sstream>    // 用于 std::stringstream
#include <string>
#pragma execution_character_set("utf-8")

namespace ABB_RAPID_SC4 {
    class RobotPost {
    public:
        // 配置参数结构体
        struct Config {
            //设置程序文件扩展名: 'mod',IRC5 (较新的控制器);'prg',S4 (较旧的控制器).
            std::string progExt = "prg";
            //设置是否将主/第一个程序生成为主程序。主/第一个程序的名称将被替换为Main(),示例: PROC Main() 而不是 PROG Prog1().
            bool firstProgAsMain = true;
            //默认每个程序的最大行数。如果程序超过此值，将生成多个"页面"(文件).
            size_t maxLinesPerProgram = 5016;
            // 在主模块中包含子程序,设置为True以在同一模块中包含子程序.
            bool includeSubPrograms = true;
            //指定允许包含在主/第一个程序中的子程序的最大代码行数,如果子程序超过此代码行数，它将作为单独的模块生成.
            size_t maxSubprogLines = 500;
            //外部传输，如果您使用外部工具加载程序(如RAPBOX)，则设置为True，它不会生成主程序来加载子程序。每个子程序都将被称为Main()，这适用于RAPBOX。False 将生成一个主程序来加载子程序
            bool externalDripfeeder = false;
            //在机器人控制器中放置程序的远程路径，当程序分割发生时，我们需要此路径来动态加载程序.
            std::string rapidRemotePath = "/hd0a/Enter-Serial-Number/HOME";
            //设置是否要忽略控制器上转台(或外部轴)的设置，如果设置为True，意味着控制器将不知道该轴。外部轴的运动将不会与机器人臂的运动同步。
            bool turntableIgnore = false;
            //如果需要，指定线性轨道和/或转台的机械单元名称，此名称将添加到wobjdata变量中.
            std::string mechanicalUnitName = "Turntable_Mechanical_Unit_Name";
            //如果需要，输入外部轴的轴比.
            std::vector<double> axesRatio = { 1.0, 1.0, 1.0, 1.0, 1.0, 1.0 };

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
        static std::string extAxes2Str(const std::vector<double>& angles,
            const std::vector<double>& ratio = {});

        // 测试函数
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
        double calculateTime(double distance, double Vmax, double Amax);
        void newMove(const robomath::Mat& new_pose);
        void saveModule(const std::string& filesave, const std::vector<std::string>& prog,
            const std::string& module_name);
        void prog2List();

        // 成员变量
        Config config_;
        std::string robotPost_;
        std::string robotName_;
        int robotAxes_;
        std::vector<std::string> axesType_;
        std::string ip_com_;
        std::vector<int> axesTrack_;
        std::vector<int> axesTurntable_;
        bool hasTrack_;
        bool hasTurntable_;
        robomath::Mat frExternalPose_;

        // 程序数据
        std::vector<ProgramData> progList_;
        std::vector<std::string> progNames_;
        std::vector<std::string> progFiles_;
        ProgramData currentProg_;
        int nProgs_;

        // 状态变量
        std::string tab_;
        std::string log_;
        double speedMMS_;
        std::string speedData_;
        std::string zoneData_;
        std::string toolData_;
        std::string wobjData_;

        // 工具和坐标系定义
        std::vector<std::string> toolDef_;
        std::vector<std::string> frameDef_;
        std::vector<std::string> toolList_;
        std::vector<std::string> frameList_;

        // 工艺参数
        bool cladOn_;
        std::string cladData_;
        bool arcOn_;
        std::string arcWeldData_;
        std::string arcWeaveData_;
        std::string arcSeamData_;

        // 3D打印参数
        int printEAO_;
        double printSpeed2Signal_;
        double printFlowMaxSignal_;
        double printAccelMMSS_;

        // 3D打印状态
        robomath::Mat printPoseLast_;
        double printELast_;
        double printENew_;
        double printLastSignal_;
    };

    // 外部常量定义
    extern const std::vector<int> KNOWN_ZONEDATA;
    extern const std::vector<int> KNOWN_SPEEDS;
    extern const std::vector<std::string> RESERVED_NAMES;
    extern const std::vector<std::string> MODULE_PREHEAD;
    extern const std::string CUSTOM_HEADER;
    
    // 模板注册函数
    extern "C" __declspec(dllexport) inline void register_ABB_RAPID_SC4(PostInterface & manager)
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
                        "src/ABB_RAPID_SC4.cpp";
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

        manager.registerTemplate("ABB_RAPID_SC4",
            headerpath,
            cpppath,
            "ABB_RAPID_SC4::RobotPost",
            headerContent, cppContent);
    }
}
