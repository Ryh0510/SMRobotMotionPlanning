#pragma once
#include <string>
#include <vector>
#include <memory>
#include <map>
#include "robomath.h"
#include <fstream>    // 用于 std::ifstream
#include <sstream>    // 用于 std::stringstream
#include <string>
#include "PostInterface.h"

namespace Staubli_S6 {
    class RobotPost {
    public:
        // 配置参数结构体
        struct Config {
            //设置每个程序的最大行数限制
            int PROG_MOVE_COUNT_MAX = 2000;
            //默认速度
            double DEFAULT_SPEED = 150.0;
            //默认平滑度
            double DEFAULT_SMOOTH = 0.1;

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

        // 公共访问器
        const std::vector<std::string> getProgramLines() const;
        const std::string& getLog() const { return log_; }

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
        void saveModule(const std::string& folderprog_final, const std::vector<std::string>& prog_lines,
            const std::string& progname, size_t moduleIndex);
        void prog2List();
        void controlProgSize();
        void closeModule();
        bool DirExists(const std::string& path);
        bool CreateDir(const std::string& path);
        std::string FilterNameS6(const std::string& name);

        // 成员变量
        Config config_;
        std::string robotPost_;
        std::string robotName_;
        int robotAxes_;
        std::vector<std::string> axesType_;
        std::string ip_com_;

        // XML数据存储
        std::string jointData_;
        std::string pointData_;
        std::string toolData_;
        std::string frameData_;
        std::string speedData_;

        // 计数器
        size_t jointCount_;
        size_t pointCount_;
        size_t toolCount_;
        size_t frameCount_;
        size_t speedCount_;
        size_t progMoveCount_;
        size_t nProgs_;

        // 程序数据
        std::vector<ProgramData> progList_;
        std::vector<std::string> progNames_;
        std::vector<std::string> progFiles_;
        ProgramData currentProg_;

        // 状态变量
        std::string tab_;
        std::string log_;
        double speedMMS_;
        std::string speedDataCurrent_;
        std::string zoneData_;
        std::string toolDataCurrent_;
        std::string wobjData_;
        std::string MAIN_FOLDER_;

        // 当前位姿和工具
        robomath::Mat currentFrame_;
        robomath::Mat currentTool_;

        // XML模板数据列表
        std::vector<std::string> PROG_PGX_LIST;
        std::vector<std::string> PROG_DTX_LIST;
        std::vector<std::string> PROG_PJX_LIST;

        // 外部轴相关
        bool hasTrack_;
        bool hasTurntable_;
        std::vector<int> axesTrack_;
        std::vector<int> axesTurntable_;
        std::vector<std::string> toolList_;
        std::vector<std::string> frameList_;

        // 其他状态变量
        bool cladOn_;
        bool arcOn_;
        double printELast_;
        double printENew_;
        double printLastSignal_;
        double printAccelMMSS_;
        double printSpeed2Signal_;
        double printFlowMaxSignal_;
        robomath::Mat frExternalPose_;
        robomath::Mat printPoseLast_;
        std::vector<std::string> toolDef_;
        std::vector<std::string> frameDef_;
        // 添加Python版本中的类变量对应物
        std::string PROG_PGX;  // 当前程序的PGX内容
        std::string LAST_PG_NAME;

        // 添加常量定义
        const std::string TOOL_CURRENT = "tProg";
        const std::string REF_CURRENT = "world";
        const std::string SPEED_CURRENT = "mNomSpeed";
    };

    // XML模板常量
    extern const std::string PROGRAM_PJX;
    extern const std::string PROGRAM_PJX_MAIN;
    extern const std::string DATA_DTX;
    extern const std::string START_PGX;
    extern const std::string STOP_PGX;
    extern const std::string LOAD_NEXT_ONE;
    extern const std::string LTX_FILE;

    // 模板注册函数
    extern "C" __declspec(dllexport) inline void register_Staubli_S6(PostInterface & manager)
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
                        "src/Staubli_S6.cpp";
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

        manager.registerTemplate("Staubli_S6",
            headerpath,
            cpppath,
            "Staubli_S6::RobotPost",
            headerContent, cppContent);
    }
}
