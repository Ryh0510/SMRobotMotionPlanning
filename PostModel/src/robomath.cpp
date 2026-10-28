#include <PostModel/robomath.h>
#include <cmath>
#include <algorithm>
#include <stdexcept>
#include <iostream>
#include <iomanip>

namespace robomath {

// Vector operations
double norm(const Vector& v) {
    if (v.size() != 3) throw std::invalid_argument("Vector must have 3 elements");
    return std::sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
}

Vector normalize3(const Vector& v) {
    double n = norm(v);
    if (n == 0) return {0, 0, 0};
    return {v[0]/n, v[1]/n, v[2]/n};
}

Vector cross(const Vector& a, const Vector& b) {
    if (a.size() != 3 || b.size() != 3) throw std::invalid_argument("Vectors must have 3 elements");
    return {
        a[1]*b[2] - a[2]*b[1],
        a[2]*b[0] - a[0]*b[2],
        a[0]*b[1] - a[1]*b[0]
    };
}

double dot(const Vector& a, const Vector& b) {
    if (a.size() != 3 || b.size() != 3) throw std::invalid_argument("Vectors must have 3 elements");
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}

double angle3(const Vector& a, const Vector& b) {
    Vector na = normalize3(a);
    Vector nb = normalize3(b);
    double cos_angle = dot(na, nb);
    cos_angle = std::max(-1.0, std::min(1.0, cos_angle));
    return std::acos(cos_angle);
}

Vector mult3(const Vector& v, double d) {
    return {v[0]*d, v[1]*d, v[2]*d};
}

Vector subs3(const Vector& a, const Vector& b) {
    return {a[0]-b[0], a[1]-b[1], a[2]-b[2]};
}

Vector add3(const Vector& a, const Vector& b) {
    return {a[0]+b[0], a[1]+b[1], a[2]+b[2]};
}

double distance(const Vector& a, const Vector& b) {
    return norm(subs3(a, b));
}

Mat vec3(double x, double y, double z) {
    // 明确创建3x1矩阵
    Mat result(3, 1);
    result(0, 0) = x;
    result(1, 0) = y;
    result(2, 0) = z;
    return result;
}

Mat vec3(const Vector& v) {
    if (v.size() != 3) throw std::invalid_argument("Vector must have 3 elements");
    return vec3(v[0], v[1], v[2]);
}

Mat rowVec3(double x, double y, double z) {
    return Mat({ {x, y, z} });
}

Mat rowVec3(const Vector& v) {
    if (v.size() != 3) throw std::invalid_argument("Vector must have 3 elements");
    return rowVec3(v[0], v[1], v[2]);
}

// Transformation matrices
Mat rotx(double rx) {
    double ct = cos(rx);
    double st = sin(rx);
    return Mat({
        {1, 0, 0, 0},
        {0, ct, -st, 0},
        {0, st, ct, 0},
        {0, 0, 0, 1}
    });
}

Mat roty(double ry) {
    double ct = cos(ry);
    double st = sin(ry);
    return Mat({
        {ct, 0, st, 0},
        {0, 1, 0, 0},
        {-st, 0, ct, 0},
        {0, 0, 0, 1}
    });
}

Mat rotz(double rz) {
    double ct = cos(rz);
    double st = sin(rz);
    return Mat({
        {ct, -st, 0, 0},
        {st, ct, 0, 0},
        {0, 0, 1, 0},
        {0, 0, 0, 1}
    });
}

Mat transl(double tx, double ty, double tz) {
    return Mat({
        {1, 0, 0, tx},
        {0, 1, 0, ty},
        {0, 0, 1, tz},
        {0, 0, 0, 1}
    });
}

Mat transl(const Vector& translation) {
    if (translation.size() != 3) throw std::invalid_argument("Translation vector must have 3 elements");
    return transl(translation[0], translation[1], translation[2]);
}

Mat eye(int size) {
    if (size == 4) {
        return Mat({
            {1, 0, 0, 0},
            {0, 1, 0, 0},
            {0, 0, 1, 0},
            {0, 0, 0, 1}
        });
    }
    Mat result(size, size);
    for (int i = 0; i < size; ++i) {
        result(i, i) = 1;
    }
    return result;
}

// Pose conversions
Mat Pose(double x, double y, double z, double rxd, double ryd, double rzd) {
    double rx = deg2rad(rxd);
    double ry = deg2rad(ryd);
    double rz = deg2rad(rzd);
    
    double srx = sin(rx);
    double crx = cos(rx);
    double sry = sin(ry);
    double cry = cos(ry);
    double srz = sin(rz);
    double crz = cos(rz);
    
    return Mat({
        {cry * crz, -cry * srz, sry, x},
        {crx * srz + crz * srx * sry, crx * crz - srx * sry * srz, -cry * srx, y},
        {srx * srz - crx * crz * sry, crz * srx + crx * sry * srz, crx * cry, z},
        {0, 0, 0, 1}
    });
}

Mat PosePP(double x, double y, double z, double r, double p, double w) {
    double a = deg2rad(r);
    double b = deg2rad(p);
    double c = deg2rad(w);
    
    double ca = cos(a);
    double sa = sin(a);
    double cb = cos(b);
    double sb = sin(b);
    double cc = cos(c);
    double sc = sin(c);
    
    return Mat({
        {cb * ca, ca * sc * sb - cc * sa, sc * sa + cc * ca * sb, x},
        {cb * sa, cc * ca + sc * sb * sa, cc * sb * sa - ca * sc, y},
        {-sb, cb * sc, cc * cb, z},
        {0.0, 0.0, 0.0, 1.0}
    });
}


Vector pose_2_xyzrpw(const Mat& H) {
    double x = H(0, 3);
    double y = H(1, 3);
    double z = H(2, 3);
    
    double r, p, w;
    if (H(2, 0) > (1.0 - 1e-10)) {
        p = -PI/2;
        r = 0;
        w = atan2(-H(1, 2), H(1, 1));
    } else if (H(2, 0) < -1.0 + 1e-10) {
        p = PI/2;
        r = 0;
        w = atan2(H(1, 2), H(1, 1));
    } else {
        p = atan2(-H(2, 0), sqrt(H(0, 0)*H(0, 0) + H(1, 0)*H(1, 0)));
        w = atan2(H(1, 0), H(0, 0));
        r = atan2(H(2, 1), H(2, 2));
    }
    
    return {x, y, z, rad2deg(r), rad2deg(p), rad2deg(w)};
}

Mat xyzrpw_2_pose(const Vector& xyzrpw) {
    if (xyzrpw.size() != 6) throw std::invalid_argument("XYZRPW vector must have 6 elements");
    
    double x = xyzrpw[0];
    double y = xyzrpw[1];
    double z = xyzrpw[2];
    double r = deg2rad(xyzrpw[3]);
    double p = deg2rad(xyzrpw[4]);
    double w = deg2rad(xyzrpw[5]);
    
    double ca = cos(r);
    double sa = sin(r);
    double cb = cos(p);
    double sb = sin(p);
    double cc = cos(w);
    double sc = sin(w);
    
    return Mat({
        {cb * cc, cc * sa * sb - ca * sc, sa * sc + ca * cc * sb, x},
        {cb * sc, ca * cc + sa * sb * sc, ca * sb * sc - cc * sa, y},
        {-sb, cb * sa, ca * cb, z},
        {0, 0, 0, 1}
    });
}


// Robot-specific conversions
Vector Pose_2_Staubli(const Mat& H) {
    Vector xyzwpr = Pose_2_TxyzRxyz(H);
    xyzwpr[3] = rad2deg(xyzwpr[3]);
    xyzwpr[4] = rad2deg(xyzwpr[4]);
    xyzwpr[5] = rad2deg(xyzwpr[5]);
    return xyzwpr;
}

Mat Staubli_2_Pose(const Vector& xyzwpr) {
    Vector temp = xyzwpr;
    temp[3] = deg2rad(temp[3]);
    temp[4] = deg2rad(temp[4]);
    temp[5] = deg2rad(temp[5]);
    return TxyzRxyz_2_Pose(temp);
}

Vector Pose_2_KUKA(const Mat& H) {
    double x = H(0, 3);
    double y = H(1, 3);
    double z = H(2, 3);
    
    double r, p, w;
    if (H(2, 0) > (1.0 - 1e-10)) {
        p = -PI/2;
        r = 0;
        w = atan2(-H(1, 2), H(1, 1));
    } else if (H(2, 0) < -1.0 + 1e-10) {
        p = PI/2;
        r = 0;
        w = atan2(H(1, 2), H(1, 1));
    } else {
        p = atan2(-H(2, 0), sqrt(H(0, 0)*H(0, 0) + H(1, 0)*H(1, 0)));
        w = atan2(H(1, 0), H(0, 0));
        r = atan2(H(2, 1), H(2, 2));
    }
    
    return {x, y, z, rad2deg(w), rad2deg(p), rad2deg(r)};
}

Mat KUKA_2_Pose(const Vector& xyzrpw) {
    if (xyzrpw.size() != 6) throw std::invalid_argument("XYZRPW vector must have 6 elements");
    
    double x = xyzrpw[0];
    double y = xyzrpw[1];
    double z = xyzrpw[2];
    double w = deg2rad(xyzrpw[3]);
    double p = deg2rad(xyzrpw[4]);
    double r = deg2rad(xyzrpw[5]);
    
    double ca = cos(r);
    double sa = sin(r);
    double cb = cos(p);
    double sb = sin(p);
    double cc = cos(w);
    double sc = sin(w);
    
    return Mat({
        {cb * ca, ca * sc * sb - cc * sa, sc * sa + cc * ca * sb, x},
        {cb * sa, cc * ca + sc * sb * sa, cc * sb * sa - ca * sc, y},
        {-sb, cb * sc, cc * cb, z},
        {0.0, 0.0, 0.0, 1.0}
    });
}

Vector pose_2_quaternion(const Mat& Ti) {
    if (!Ti.isHomogeneous()) {
        throw std::invalid_argument("Input matrix must be a homogeneous transformation matrix");
    }
    
    const double TOLERANCE_0 = 1e-9;
    const double TOLERANCE_180 = 1e-7;
    
    double cosangle = std::min(std::max(((Ti(0, 0) + Ti(1, 1) + Ti(2, 2) - 1.0) * 0.5), -1.0), 1.0);
    
    double q1, q2, q3, q4;
    
    if (cosangle > 1.0 - TOLERANCE_0) {
        // Identity matrix
        q1 = 1.0;
        q2 = 0.0;
        q3 = 0.0;
        q4 = 0.0;
    } else if (cosangle < -1.0 + TOLERANCE_180) {
        // 180 rotation around an axis
        std::vector<double> diag = {Ti(0, 0), Ti(1, 1), Ti(2, 2)};
        auto max_it = std::max_element(diag.begin(), diag.end());
        int k = std::distance(diag.begin(), max_it);
        
        std::vector<double> col = {Ti(0, k), Ti(1, k), Ti(2, k)};
        col[k] = col[k] + 1.0;
        
        double denominator = sqrtA(2.0 * (1.0 + diag[k]));
        std::vector<double> rotvector(3);
        for (int i = 0; i < 3; ++i) {
            rotvector[i] = col[i] / denominator;
        }
        
        q1 = 0.0;
        q2 = rotvector[0];
        q3 = rotvector[1];
        q4 = rotvector[2];
    } else {
        // No edge case, normal calculation
        double a = Ti(0, 0);
        double b = Ti(1, 1);
        double c = Ti(2, 2);
        
        double sign2 = 1.0;
        double sign3 = 1.0;
        double sign4 = 1.0;
        
        if (Ti(2, 1) - Ti(1, 2) < 0.0) {
            sign2 = -1.0;
        }
        if (Ti(0, 2) - Ti(2, 0) < 0.0) {
            sign3 = -1.0;
        }
        if (Ti(1, 0) - Ti(0, 1) < 0.0) {
            sign4 = -1.0;
        }
        
        q1 = sqrtA(a + b + c + 1.0) / 2.0;
        q2 = sign2 * sqrtA(a - b - c + 1.0) / 2.0;
        q3 = sign3 * sqrtA(-a + b - c + 1.0) / 2.0;
        q4 = sign4 * sqrtA(-a - b + c + 1.0) / 2.0;
    }
    
    return {q1, q2, q3, q4};
}

Mat quaternion_2_pose(const Vector& qin) {
    if (qin.size() != 4) {
        throw std::invalid_argument("Quaternion vector must have 4 elements");
    }
    
    double qnorm = sqrt(qin[0] * qin[0] + qin[1] * qin[1] + qin[2] * qin[2] + qin[3] * qin[3]);
    
    // Create a copy to avoid modifying the input
    Vector q = qin;
    if (qnorm > 0) {
        q[0] = q[0] / qnorm;
        q[1] = q[1] / qnorm;
        q[2] = q[2] / qnorm;
        q[3] = q[3] / qnorm;
    }
    
    return Mat({
        {1 - 2 * q[2] * q[2] - 2 * q[3] * q[3], 2 * q[1] * q[2] - 2 * q[3] * q[0], 2 * q[1] * q[3] + 2 * q[2] * q[0], 0},
        {2 * q[1] * q[2] + 2 * q[3] * q[0], 1 - 2 * q[1] * q[1] - 2 * q[3] * q[3], 2 * q[2] * q[3] - 2 * q[1] * q[0], 0},
        {2 * q[1] * q[3] - 2 * q[2] * q[0], 2 * q[2] * q[3] + 2 * q[1] * q[0], 1 - 2 * q[1] * q[1] - 2 * q[2] * q[2], 0},
        {0, 0, 0, 1}
    });
}

Vector Pose_2_ABB(const Mat& H) {
    if (!H.isHomogeneous()) {
        throw std::invalid_argument("Input matrix must be a homogeneous transformation matrix");
    }
    
    Vector q = pose_2_quaternion(H);
    return {H(0, 3), H(1, 3), H(2, 3), q[0], q[1], q[2], q[3]};
}

Vector Pose_2_UR(const Mat& H) {
    double x = H(0, 3);
    double y = H(1, 3);
    double z = H(2, 3);
    
    // Simplified rotation vector calculation
    double angle = acos((H(0, 0) + H(1, 1) + H(2, 2) - 1) / 2);
    Vector rxyz = {
        H(2, 1) - H(1, 2),
        H(0, 2) - H(2, 0),
        H(1, 0) - H(0, 1)
    };
    
    if (angle < 1e-8) {
        rxyz = {0, 0, 0};
    } else {
        rxyz = mult3(normalize3(rxyz), angle);
    }
    
    return {x, y, z, rxyz[0], rxyz[1], rxyz[2]};
}

Mat UR_2_Pose(const Vector& xyzwpr) {
    if (xyzwpr.size() != 6) throw std::invalid_argument("XYZA vector must have 6 elements");
    return xyzrpw_2_pose(xyzwpr);
}

// Denavit-Hartenberg
Mat dh(double rz, double tx, double tz, double rx) {
    double crx = cos(rx);
    double srx = sin(rx);
    double crz = cos(rz);
    double srz = sin(rz);
    
    return Mat({
        {crz, -srz * crx, srz * srx, tx * crz},
        {srz, crz * crx, -crz * srx, tx * srz},
        {0, srx, crx, tz},
        {0, 0, 0, 1}
    });
}

Mat dhm(double rx, double tx, double tz, double rz) {
    double crx = cos(rx);
    double srx = sin(rx);
    double crz = cos(rz);
    double srz = sin(rz);
    
    return Mat({
        {crz, -srz, 0, tx},
        {crx * srz, crx * crz, -srx, -tz * srx},
        {srx * srz, crz * srx, crx, tz * crx},
        {0, 0, 0, 1}
    });
}

// Matrix class implementation
Mat::Mat() : data(4, std::vector<double>(4, 0)) {
    for (int i = 0; i < 4; ++i) data[i][i] = 1;
}

Mat::Mat(int rows, int cols) : data(rows, std::vector<double>(cols, 0)) {}

Mat::Mat(const std::vector<std::vector<double>>& rows) : data(rows) {}

Mat::Mat(const std::vector<double>& row) : data(1, row) {}

Mat::Mat(const Mat& other) : data(other.data) {}

Mat::Mat(Mat&& other) noexcept : data(std::move(other.data)) {}

Mat& Mat::operator=(const Mat& other) {
    if (this != &other) {
        data = other.data;
    }
    return *this;
}

Mat& Mat::operator=(Mat&& other) noexcept {
    if (this != &other) {
        data = std::move(other.data);
    }
    return *this;
}

double& Mat::operator()(int row, int col) {
    if (row < 0 || row >= rows() || col < 0 || col >= cols()) {
        throw std::out_of_range("Matrix index out of range");
    }
    return data[row][col];
}

const double& Mat::operator()(int row, int col) const {
    if (row < 0 || row >= rows() || col < 0 || col >= cols()) {
        throw std::out_of_range("Matrix index out of range");
    }
    return data[row][col];
}

std::vector<double> Mat::operator()(int row) const {
    if (row < 0 || row >= rows()) {
        throw std::out_of_range("Matrix row index out of range");
    }
    return data[row];
}

Mat Mat::operator+(const Mat& other) const {
    if (rows() != other.rows() || cols() != other.cols()) {
        throw std::invalid_argument("Matrix dimensions must match for addition");
    }
    
    Mat result(rows(), cols());
    for (int i = 0; i < rows(); ++i) {
        for (int j = 0; j < cols(); ++j) {
            result(i, j) = data[i][j] + other(i, j);
        }
    }
    return result;
}

Mat Mat::operator-(const Mat& other) const {
    if (rows() != other.rows() || cols() != other.cols()) {
        throw std::invalid_argument("Matrix dimensions must match for subtraction");
    }
    
    Mat result(rows(), cols());
    for (int i = 0; i < rows(); ++i) {
        for (int j = 0; j < cols(); ++j) {
            result(i, j) = data[i][j] - other(i, j);
        }
    }
    return result;
}

Mat Mat::operator*(const Mat& other) const {
    if (cols() != other.rows()) {
        throw std::invalid_argument("Matrix dimensions must match for multiplication");
    }
    
    Mat result(rows(), other.cols());
    for (int i = 0; i < rows(); ++i) {
        for (int j = 0; j < other.cols(); ++j) {
            for (int k = 0; k < cols(); ++k) {
                result(i, j) += data[i][k] * other(k, j);
            }
        }
    }
    return result;
}

Mat Mat::operator*(double scalar) const {
    Mat result(rows(), cols());
    for (int i = 0; i < rows(); ++i) {
        for (int j = 0; j < cols(); ++j) {
            result(i, j) = data[i][j] * scalar;
        }
    }
    return result;
}

bool Mat::operator==(const Mat& other) const {
    if (rows() != other.rows() || cols() != other.cols()) return false;
    
    for (int i = 0; i < rows(); ++i) {
        for (int j = 0; j < cols(); ++j) {
            if (std::abs(data[i][j] - other(i, j)) > 1e-10) {
                return false;
            }
        }
    }
    return true;
}

bool Mat::operator!=(const Mat& other) const {
    return !(*this == other);
}

Mat Mat::tr() const {
    Mat result(cols(), rows());
    for (int i = 0; i < rows(); ++i) {
        for (int j = 0; j < cols(); ++j) {
            result(j, i) = data[i][j];
        }
    }
    return result;
}

Mat Mat::invH() const {
    if (!isHomogeneous()) {
        throw std::runtime_error("Matrix is not homogeneous");
    }
    
    Mat R = Rot33().tr();
    //Vector t = mult3(R, -1.0 * norm(Pos()));
    Vector pos = Pos();    // 获取位置向量

    // 计算 t = -R * pos
    // 首先需要实现矩阵与向量的乘法
    Vector rotated_pos(3, 0);
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            rotated_pos[i] += R(i, j) * pos[j];
        }
    }

    // 然后取反
    Vector t = mult3(rotated_pos, -1.0);
    
    Mat result(4, 4);
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            result(i, j) = R(i, j);
        }
        result(i, 3) = t[i];
    }
    result(3, 3) = 1;
    
    return result;
}

Mat Mat::inv() const {
    return invH();
}

std::pair<int, int> Mat::size() const {
    return {rows(), cols()};
}

int Mat::rows() const {
    return data.size();
}

int Mat::cols() const {
    return data.empty() ? 0 : data[0].size();
}

bool Mat::isHomogeneous() const {
    if (rows() != 4 || cols() != 4) return false;
    if (data[3][0] != 0 || data[3][1] != 0 || data[3][2] != 0 || data[3][3] != 1) return false;
    
    // Check if rotation matrix is orthogonal
    Mat R = Rot33();
    Mat RTR = R.tr() * R;
    Mat I = eye(3);
    
    double error = 0;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            error += std::abs(RTR(i, j) - I(i, j));
        }
    }
    
    return error < 1e-10;
}

Vector Mat::Pos() const 
{
    if (rows() < 3 || cols() < 4) {
        throw std::runtime_error("Matrix is not 4x4 homogeneous transformation matrix");
    }

    return { data[0][3], data[1][3], data[2][3] };
}

Mat TxyzRxyz_2_Pose(const Vector& xyzrpw) {
    if (xyzrpw.size() != 6) throw std::invalid_argument("XYZRPR vector must have 6 elements");
    double x = xyzrpw[0];
    double y = xyzrpw[1];
    double z = xyzrpw[2];
    double rx = xyzrpw[3];
    double ry = xyzrpw[4];
    double rz = xyzrpw[5];
    double srx = sin(rx);
    double crx = cos(rx);
    double sry = sin(ry);
    double cry = cos(ry);
    double srz = sin(rz);
    double crz = cos(rz);
    return Mat({
        {cry * crz, -cry * srz, sry, x},
        {crx * srz + crz * srx * sry, crx * crz - srx * sry * srz, -cry * srx, y},
        {srx * srz - crx * crz * sry, crz * srx + crx * sry * srz, crx * cry, z},
        {0, 0, 0, 1}
        });
}

// 实现 Pose_2_TxyzRxyz
Vector Pose_2_TxyzRxyz(const Mat& H) {
    double x = H(0, 3);
    double y = H(1, 3);
    double z = H(2, 3);
    double a = H(0, 0);
    double b = H(0, 1);
    double c = H(0, 2);
    double d = H(1, 2);
    double e = H(2, 2);
    double rx, ry, rz;
    if (c > (1.0 - 1e-10)) {
        ry = PI / 2;
        rx = 0;
        rz = atan2(H(1, 0), H(1, 1));
    }
    else if (c < (-1.0 + 1e-10)) {
        ry = -PI / 2;
        rx = 0;
        rz = atan2(H(1, 0), H(1, 1));
    }
    else {
        double sy = c;
        double cy = sqrt(1 - sy * sy);
        double sx = -d / cy;
        double cx = e / cy;
        double sz = -b / cy;
        double cz = a / cy;
        rx = atan2(sx, cx);
        ry = atan2(sy, cy);
        rz = atan2(sz, cz);
    }
    return { x, y, z, rx, ry, rz };
}

// 实现 Mat 类的成员函数
Vector Mat::VX() const {
    return { data[0][0], data[1][0], data[2][0] };
}

Vector Mat::VY() const {
    return { data[0][1], data[1][1], data[2][1] };
}

Vector Mat::VZ() const {
    return { data[0][2], data[1][2], data[2][2] };
}

Mat Mat::Rot33() const {
    return Mat({
        {data[0][0], data[0][1], data[0][2]},
        {data[1][0], data[1][1], data[1][2]},
        {data[2][0], data[2][1], data[2][2]}
        });
}

void Mat::setPos(const Vector& newpos) {
    if (newpos.size() != 3) throw std::invalid_argument("Position vector must have 3 elements");
    data[0][3] = newpos[0];
    data[1][3] = newpos[1];
    data[2][3] = newpos[2];
}

void Mat::setVX(const Vector& v_xyz) {
    if (v_xyz.size() != 3) throw std::invalid_argument("Vector must have 3 elements");
    double n = norm(v_xyz);
    if (n == 0) return;
    data[0][0] = v_xyz[0] / n;
    data[1][0] = v_xyz[1] / n;
    data[2][0] = v_xyz[2] / n;
}

void Mat::setVY(const Vector& v_xyz) {
    if (v_xyz.size() != 3) throw std::invalid_argument("Vector must have 3 elements");
    double n = norm(v_xyz);
    if (n == 0) return;
    data[0][1] = v_xyz[0] / n;
    data[1][1] = v_xyz[1] / n;
    data[2][1] = v_xyz[2] / n;
}

void Mat::setVZ(const Vector& v_xyz) {
    if (v_xyz.size() != 3) throw std::invalid_argument("Vector must have 3 elements");
    double n = norm(v_xyz);
    if (n == 0) return;
    data[0][2] = v_xyz[0] / n;
    data[1][2] = v_xyz[1] / n;
    data[2][2] = v_xyz[2] / n;
}

Mat Mat::RelTool(double x, double y, double z, double rx, double ry, double rz) const {
    return robomath::RelTool(*this, x, y, z, rx, ry, rz);
}

Mat Mat::Offset(double x, double y, double z, double rx, double ry, double rz) const {
    return robomath::Offset(*this, x, y, z, rx, ry, rz);
}

std::vector<double> Mat::toVector() const {
    std::vector<double> result;
    for (const auto& row : data) {
        for (double val : row) {
            result.push_back(val);
        }
    }
    return result;
}

std::vector<std::vector<double>> Mat::toMatrix() const {
    return data;
}

std::ostream& operator<<(std::ostream& os, const Mat& mat) {
    os << std::fixed << std::setprecision(3);
    for (int i = 0; i < mat.rows(); ++i) {
        os << "[ ";
        for (int j = 0; j < mat.cols(); ++j) {
            os << mat(i, j);
            if (j < mat.cols() - 1) os << ", ";
        }
        os << " ]" << std::endl;
    }
    return os;
}

// 实现工具函数 RelTool
Mat RelTool(const Mat& target_pose, double x, double y, double z, double rx, double ry, double rz) {
    if (!target_pose.isHomogeneous()) {
        throw std::runtime_error("Target pose is not homogeneous");
    }
    Mat translation = transl(x, y, z);
    Mat rotation = rotx(rx) * roty(ry) * rotz(rz);
    return target_pose * translation * rotation;
}

// 实现工具函数 Offset
Mat Offset(const Mat& target_pose, double x, double y, double z, double rx, double ry, double rz) 
{
    if (!target_pose.isHomogeneous()) 
    {
        throw std::runtime_error("Target pose is not homogeneous");
    }
    Mat translation = transl(x, y, z);
    Mat rotation = rotx(rx) * roty(ry) * rotz(rz);
    return translation * rotation * target_pose;
}
}