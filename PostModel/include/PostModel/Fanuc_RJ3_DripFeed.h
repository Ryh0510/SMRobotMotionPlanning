#pragma once
#include <string>
#include <vector>
#include <map>
#include <memory>
#include "robomath.h"
#include <fstream>    // 用于 std::ifstream
#include <sstream>    // 用于 std::stringstream
#include <string>
#include "PostInterface.h"

namespace Fanuc_RJ3_DripFeed {
    class RobotPost {
    public:
        // 配置参数结构体
        struct Config {
            //每个程序的最大行数限制。超过此限制时将生成多个"页面"（文件）。
            int maxLinesPerProgram = 9999;
            //是否生成子程序
            bool includeSubPrograms = true;
            //设置默认关节速度（总速度的百分比）
            std::string jointSpeed = "20%";
            //设置默认笛卡尔运动速度 
            std::string speed = "200mm/sec";
            //设置默认CNT值（所有运动将保持此平滑值直至被修改）CNT_VALUE = 'CNT5' # 5%平滑度（可设置CNT1-CNT100）
            std::string cntValue = "FINE";
            //激活的用户坐标系ID（寄存器）
            std::string activeUF = "9";
            //激活的工具坐标系ID（寄存器）
            std::string activeUT = "9";
            //用于计算的备用位置寄存器（例如设置UFRAME和UTOOL）
            int sparePR = 9;
            //设置转台组（通常为GP2或GP3）
            std::string turntableGroup = "GP2";
            //设置为True可生成兼容RJ3控制器的程序（程序头有细微差异）
            bool fanucRJ3Compatible = true;
            //将LS程序编译为TP程序,设置Roboguide WinOLPC工具路径，若设置为None则不生成TP文件,如果路径不存在，此步骤将被忽略
            std::string pathMakeTP = "C:/Program Files (x86)/FANUC/WinOLPC/bin/";
            //生成 drip feeder 程序：将长程序分割为子程序，并在执行时动态加载到控制器,设置文件名以使用自动 dripfeeder：程序执行时将通过FTP发送文件
            std::string dripFeedFileName = "Program_DripFeed.py";

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

        // I/O 操作
        void setDO(const std::string& io_var, const std::string& io_value);
        void setAO(const std::string& io_var, const std::string& io_value);
        void waitDI(const std::string& io_var, const std::string& io_value, int timeout_ms = -1);

        // 代码生成
        void RunCode(const std::string& code, bool is_function_call = false);
        void RunMessage(const std::string& message, bool is_comment = false);
        void Pause(int time_ms);

        // 工具函数
        static std::string getSafeName(const std::string& name, int max_chars = 16);
        static std::string pose2Fanuc(const robomath::Mat& pose);
        bool FTPUploadFile(const std::string& server, const std::string& username,
            const std::string& password, const std::string& remotePath,
            const std::string& localFile, std::string& errorMsg);
        // 友元函数声明
        friend void test_post(std::string path);
    private:
        // 内部数据结构
        struct ProgramData {
            std::vector<std::string> lines;
            std::vector<std::string> targets;
            std::string name;
            int lineCount = 0;
            int pCount = 0;
            ProgramData() = default;
            ProgramData(const std::string& n) : name(n) {}
        };

        // 私有方法
        void addLine(const std::string& newline, char moveType = ' ');
        void addLineTargets(const std::string& newline);
        void addLog(const std::string& newline);
        void pageSizeControl();
        int addTargetJoints(const robomath::Mat& pose, const std::vector<double>& joints);
        int addTargetCartesian(const robomath::Mat& pose, const std::vector<double>& joints,
            const std::vector<double>& confRLF = {});
        void saveProgram(const std::string& folder, const std::string& progname,
            bool askUser, const std::string& showResult);
        void generateDripFeedScript(const std::string& folder, const std::vector<std::string>& prognames,
            const std::string& scriptName, const std::string& progMain,
            const std::vector<std::string>& ftpParams);

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
        bool hasExtAxes_ = false;

        // 程序数据
        std::vector<ProgramData> progList_;
        std::vector<std::string> progNames_;
        std::vector<std::string> progNamesMain_;
        std::vector<std::string> progFiles_;
        ProgramData currentProg_;
        int nProgs_ = 0;
        int nPages_ = 0;

        // 状态变量
        std::string log_;
        double speedMMS_ = 500.0;
        std::string progExt_ = "LS";
        bool firstProgAsMain_ = false;
        std::string mechanicalUnitName_ = "";
        robomath::Mat lastPose_;
        std::string speedBackup_;
        std::string arcParams_;

        // 工具和坐标系定义
        std::vector<std::string> toolDef_;
        std::vector<std::string> frameDef_;
    };

    // 常量定义
    extern const std::vector<std::string> FANUC_RESERVED_NAMES;

    // 模板注册函数
        extern "C" __declspec(dllexport) inline void register_Fanuc_RJ3_DripFeed(PostInterface & manager)
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
                        "src/Fanuc_RJ3_DripFeed.cpp";
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

        manager.registerTemplate("Fanuc_RJ3_DripFeed",
            headerpath,
            cpppath,
            "Fanuc_RJ3_DripFeed::RobotPost",
            headerContent, cppContent);
    }

}
