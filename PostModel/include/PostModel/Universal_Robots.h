#pragma once
#include <string>
#include <vector>
#include <memory>
#include <map>
#include <cmath>
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <cstdio>
#include <cstdlib>
#include <algorithm>
#include "robomath.h"
#include <string>
#include "PostInterface.h"

namespace Universal_Robots {
    class RobotPost {
    public:
        // 配置参数结构体
        struct Config {
            //是否包含自定义头文件到程序中
            bool INCLUDE_HEADER = true;
            //设置为True使用MoveP，设置为False使用MoveL
            bool USE_MOVEP = false;
            //设置为True将参考坐标系作为位姿并使用pose_trans预乘所有目标,设置为False将所有目标相对于机器人基座输出
            bool USE_RELATIVE_TARGETS = true;
            //如果为True，将尝试使用SFTP上传
            bool UPLOAD_SFTP = false;
            //设置为True自动过滤圆滑过渡(CB3或更早版本),设置为False由控制器根据需要调整过渡,保留默认值(None)以自动检测适当的操作
            bool BLENDING_CHECK = false;
            //在触发程序调用、速度变化或更改数字输出之前强制精确移动
            bool MOVE_ACCURATE_CALLS = true;
            //线性移动的默认速度(米/秒)
            double SPEED_MS = 0.25;
            //关节移动的默认速度(弧度/秒)
            double SPEED_RADS = 0.75;
            //线性移动的默认加速度(米/秒²)
            double ACCEL_MSS = 1.2;
            //关节移动的默认加速度(弧度/秒²)
            double ACCEL_RADSS = 1.2;
            //默认圆弧过渡半径(米,拐角平滑)
            double BLEND_RADIUS_M = 0.001;
            //每个程序的最大行数。如果行数超过限制，程序将逐步执行
            double MAX_LINES_X_PROG = 1000000000.0;
            //输出的最小圆弧半径(毫米),不考虑圆弧过渡半径
            double MOVEC_MIN_RADIUS = 1;
            //输出的最大圆弧半径(毫米)。不考虑圆弧过渡半径
            double MOVEC_MAX_RADIUS = 10000;
            //控制器允许的最大线速度(否则会抛出速度错误)
            double MAX_SPEED_MS = 3.0;
            //控制器允许的最大角速度(否则会抛出速度错误)
            double MAX_SPEED_DEGS = 180;
            //控制器允许的最大线加速度(否则会抛出速度错误)
            double MAX_ACCEL_MSS = 15.0;
            //控制器允许的最大角加速度(否则会抛出速度错误)
            double MAX_ACCEL_DEGSS = 2291.8;

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
        void ProgSave(const std::string& folder, const std::string& progname, bool askUser = false, const std::string& showResult = "");
        void ProgSendRobot(const std::string& remote_path, const std::string& ftp_user, const std::string& ftp_pass);

        // 运动指令
        void MoveJ(const robomath::Mat& pose, const std::vector<double>& joints, const std::vector<double>& confRLF = {});
        void MoveL(const robomath::Mat& pose, const std::vector<double>& joints, const std::vector<double>& confRLF = {});
        void MoveC(const robomath::Mat& pose1, const std::vector<double>& joints1, const robomath::Mat& pose2, const std::vector<double>& joints2, const std::vector<double>& confRLF1 = {}, const std::vector<double>& confRLF2 = {});

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
        void RunMessage(const std::string& message, bool iscomment = false);
        void Pause(int time_ms);

        // 工具函数
        static std::string FilterName(const std::string& namefilter, char safechar = 'P',
            const std::vector<std::string>& reserved_names = {});
        static std::string pose_2_str(const robomath::Mat& pose);
        static std::string angles_2_str(const std::vector<double>& angles);
        static std::string extAxes2Str(const std::vector<double>& angles,
            const std::vector<double>& ratio = {});

        // 公共访问器
        const std::vector<std::string> getProgramLines() const;
        const std::string& getLog() const { return LOG_; }
        // 循环等函数 ：
        void EnterWhile(const std::vector<int>& conditionPair);
        void ExitWhile();
        void EnterIf(const std::vector<int>& conditionPair);
        void ExitIf();
        //void EnterIf(const std::string& condition); // 可选
        //void EnterWhile(const std::string& condition); // 可选
        // 友元函数声明
        friend void test_post(const std::string& path);

    private:
        // 私有方法
        void addLine(const std::string& newline);
        void addLog(const std::string& newline);
        std::string blend_radius_check(const robomath::Mat& pose_abs, double ratio_check = 0.4);
        double circle_radius(const robomath::Mat& p0, const robomath::Mat& p1, const robomath::Mat& p2);
        void make_last_move_accurate();
        void vars_update();
        void prog_2_list();
        static std::string FilterBackSlash(const std::string& code);

        // 成员变量
        Config config_;
        std::string robotPost_;
        std::string robotName_;
        int robotAxes_;
        std::vector<std::string> axesType_;
        std::string ip_com_;
        std::string NATIVE_NAME_;


        // 程序数据存储
        std::vector<std::string> PROG_;
        std::vector<std::vector<std::string>> PROG_LIST_;
        std::vector<std::string> VARS_;
        std::vector<std::vector<std::string>> VARS_LIST_;
        std::vector<std::string> SUBPROG_;
        std::vector<std::string> PROG_FILES_;
        std::string TAB_;
        std::string LOG_;

        // 状态变量
        size_t nPROGS_;
        double SPEED_MS_;
        double SPEED_RADS_;
        double ACCEL_MSS_;
        double ACCEL_RADSS_;
        double BLEND_RADIUS_M_;
        double MAX_SPEED_DEGS_;
        double MAX_ACCEL_DEGSS_;
        bool BLENDING_CHECK_;
        std::string BLEND_RADIUS_;
        robomath::Mat REF_FRAME_;
        robomath::Mat LAST_POS_ABS_;
        robomath::Mat LAST_DIR_;
        std::string PROG_EXT_;
        std::string MAIN_PROGNAME_;
        double MAX_LINES_X_PROG_;
        // 常量
        const std::string HEADER_ = R"(
      #
      # TO REMOVE HEADER:
      # Go to "Program" -> "Post-Processor Editor"
      # Select "Universal Robots" 
      # Set "INCLUDE_HEADER" to "False"
      #
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

      # Example to drive an extruder:
      def Extruder(value):
        # use the value as an output:
        if value < 0:
          # stop extruder
        else:
          # start extruder
        end
      end
  
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
    };

    // 模板注册函数
    extern "C" __declspec(dllexport) inline void register_Universal_Robots(PostInterface & manager)
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
                        "src/Universal_Robots.cpp";
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

        manager.registerTemplate("Universal_Robots",
            headerpath,
            cpppath,
            "Universal_Robots::RobotPost",
            headerContent, cppContent);
    }
}
