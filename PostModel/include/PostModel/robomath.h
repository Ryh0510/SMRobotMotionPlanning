#pragma once
#include <vector>
#include <cmath>
#include <string>
#include <stdexcept>
#include <iostream>
#include <algorithm>
#include <type_traits>


#ifdef POSTMODEL_SHAREDLIB_EXPORTS
#define ROBOMATH_API __declspec(dllexport)
#else
#define ROBOMATH_API __declspec(dllimport)
#endif

namespace robomath {

    // Constants
    constexpr double PI = 3.14159265358979323846;

    // Forward declarations
    class Mat;
    using Vector = std::vector<double>;
    using Matrix = std::vector<Vector>;

    // Utility functions
    inline double deg2rad(double degrees) { return degrees * PI / 180.0; }
    inline double rad2deg(double radians) { return radians * 180.0 / PI; }

    // Basic math functions
    inline double sqrt(double value) { return std::sqrt(value); }
    inline double sqrtA(double value) { return value <= 0 ? 0 : std::sqrt(value); }
    inline double sin(double value) { return std::sin(value); }
    inline double cos(double value) { return std::cos(value); }
    inline double tan(double value) { return std::tan(value); }
    inline double asin(double value) { return std::asin(value); }
    inline double acos(double value) { return std::acos(value); }
    inline double atan2(double y, double x) { return std::atan2(y, x); }

    // Vector operations
    ROBOMATH_API double norm(const Vector& v);
    ROBOMATH_API Vector normalize3(const Vector& v);
    ROBOMATH_API Vector cross(const Vector& a, const Vector& b);
    ROBOMATH_API double dot(const Vector& a, const Vector& b);
    ROBOMATH_API double angle3(const Vector& a, const Vector& b);
    ROBOMATH_API Vector mult3(const Vector& v, double d);
    ROBOMATH_API Vector subs3(const Vector& a, const Vector& b);
    ROBOMATH_API Vector add3(const Vector& a, const Vector& b);
    ROBOMATH_API double distance(const Vector& a, const Vector& b);

    // 创建列向量（3x1矩阵）
    ROBOMATH_API Mat vec3(double x, double y, double z);
    ROBOMATH_API Mat vec3(const Vector& v);

    // 创建行向量（1x3矩阵）  
    ROBOMATH_API Mat rowVec3(double x, double y, double z);
    ROBOMATH_API Mat rowVec3(const Vector& v);

    // Transformation matrices
    ROBOMATH_API Mat rotx(double rx);
    ROBOMATH_API Mat roty(double ry);
    ROBOMATH_API Mat rotz(double rz);
    ROBOMATH_API Mat transl(double tx, double ty = 0, double tz = 0);
    ROBOMATH_API Mat transl(const Vector& translation);
    ROBOMATH_API Mat eye(int size = 4);

    // Pose conversions
    ROBOMATH_API Mat Pose(double x, double y, double z, double rxd, double ryd, double rzd);
    ROBOMATH_API Mat PosePP(double x, double y, double z, double r, double p, double w);
    ROBOMATH_API Vector pose_2_xyzrpw(const Mat& H);
    ROBOMATH_API Mat xyzrpw_2_pose(const Vector& xyzrpw);
    ROBOMATH_API Mat TxyzRxyz_2_Pose(const Vector& xyzrpw);
    ROBOMATH_API Vector Pose_2_TxyzRxyz(const Mat& H);

    // Robot-specific conversions
    ROBOMATH_API Vector Pose_2_Staubli(const Mat& H);
    ROBOMATH_API Mat Staubli_2_Pose(const Vector& xyzwpr);
    ROBOMATH_API Vector Pose_2_KUKA(const Mat& H);
    ROBOMATH_API Mat KUKA_2_Pose(const Vector& xyzrpw);
    ROBOMATH_API Vector Pose_2_ABB(const Mat& H);
    ROBOMATH_API Vector Pose_2_UR(const Mat& H);
    ROBOMATH_API Mat UR_2_Pose(const Vector& xyzwpr);

    // Denavit-Hartenberg
    ROBOMATH_API Mat dh(double rz, double tx = 0, double tz = 0, double rx = 0);
    ROBOMATH_API Mat dhm(double rx, double tx = 0, double tz = 0, double rz = 0);

    // Matrix class
    class ROBOMATH_API Mat {
    public:
        std::vector<std::vector<double>> data;

        // Constructors
        Mat();
        Mat(int rows, int cols);
        Mat(const std::vector<std::vector<double>>& rows);
        Mat(const std::vector<double>& row);

        // Copy and move
        Mat(const Mat& other);
        Mat(Mat&& other) noexcept;
        Mat& operator=(const Mat& other);
        Mat& operator=(Mat&& other) noexcept;

        // Access operators
        double& operator()(int row, int col);
        const double& operator()(int row, int col) const;
        std::vector<double> operator()(int row) const;

        // Matrix operations
        Mat operator+(const Mat& other) const;
        Mat operator-(const Mat& other) const;
        Mat operator*(const Mat& other) const;
        Mat operator*(double scalar) const;
        bool operator==(const Mat& other) const;
        bool operator!=(const Mat& other) const;

        // Transformations
        Mat tr() const;
        Mat invH() const;
        Mat inv() const;

        // Properties
        std::pair<int, int> size() const;
        int rows() const;
        int cols() const;
        bool isHomogeneous() const;

        // Pose operations
        Vector Pos() const;
        Vector VX() const;
        Vector VY() const;
        Vector VZ() const;
        Mat Rot33() const;

        void setPos(const Vector& newpos);
        void setVX(const Vector& v_xyz);
        void setVY(const Vector& v_xyz);
        void setVZ(const Vector& v_xyz);

        Mat RelTool(double x, double y, double z, double rx = 0, double ry = 0, double rz = 0) const;
        Mat Offset(double x, double y, double z, double rx = 0, double ry = 0, double rz = 0) const;

        // Utility
        std::vector<double> toVector() const;
        std::vector<std::vector<double>> toMatrix() const;

        // Output
        friend std::ostream& operator<<(std::ostream& os, const Mat& mat);
    };

    // Utility functions
    ROBOMATH_API Mat RelTool(const Mat& target_pose, double x, double y, double z, double rx = 0, double ry = 0, double rz = 0);
    ROBOMATH_API Mat Offset(const Mat& target_pose, double x, double y, double z, double rx = 0, double ry = 0, double rz = 0);

} // namespace robomath

