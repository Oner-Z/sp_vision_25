#ifndef TOOLS__MATH_TOOLS_HPP
#define TOOLS__MATH_TOOLS_HPP

#include <Eigen/Geometry>
#include <chrono>
#include <opencv2/opencv.hpp>

namespace tools
{
// 将弧度值限制在(-pi, pi]
double limit_rad(double angle);

// 四元数转欧拉角
// x = 0, y = 1, z = 2
// e.g. 先绕z轴旋转，再绕y轴旋转，最后绕x轴旋转：axis0=2, axis1=1, axis2=0
// 参考：https://github.com/evbernardes/quaternion_to_euler
Eigen::Vector3d eulers(
  Eigen::Quaterniond q, int axis0, int axis1, int axis2, bool extrinsic = false);

// 旋转矩阵转欧拉角
// x = 0, y = 1, z = 2
// e.g. 先绕z轴旋转，再绕y轴旋转，最后绕x轴旋转：axis0=2, axis1=1, axis2=0
Eigen::Vector3d eulers(Eigen::Matrix3d R, int axis0, int axis1, int axis2, bool extrinsic = false);

// 欧拉角转旋转矩阵
// zyx:先绕z轴旋转，再绕y轴旋转，最后绕x轴旋转
Eigen::Matrix3d rotation_matrix(const Eigen::Vector3d & ypr);

// 直角坐标系转球坐标系
// ypd为yaw、pitch、distance的缩写
Eigen::Vector3d xyz2ypd(const Eigen::Vector3d & xyz);

// 直角坐标系转球坐标系转换函数对xyz的雅可比矩阵
Eigen::MatrixXd xyz2ypd_jacobian(const Eigen::Vector3d & xyz);

// 球坐标系转直角坐标系
Eigen::Vector3d ypd2xyz(const Eigen::Vector3d & ypd);

// 球坐标系转直角坐标系转换函数对xyz的雅可比矩阵
Eigen::MatrixXd ypd2xyz_jacobian(const Eigen::Vector3d & ypd);

// 计算时间差a - b，单位：s
double delta_time(
  const std::chrono::steady_clock::time_point & a, const std::chrono::steady_clock::time_point & b);

/**
 * @brief 两个二维点间距离
 * @param[in] pt1
 * @param[in] pt2
 * @return double
 */
inline double pointPointDistance(const cv::Point2f& pt1, const cv::Point2f& pt2) noexcept {
  return std::sqrt((pt1.x - pt2.x) * (pt1.x - pt2.x) + (pt1.y - pt2.y) * (pt1.y - pt2.y));
}

/**
* @brief 两个三维点间距离
* @param[in] pt1
* @param[in] pt2
* @return double
*/
inline double pointPointDistance(const cv::Point3f& pt1, const cv::Point3f& pt2) noexcept {
  return std::sqrt((pt1.x - pt2.x) * (pt1.x - pt2.x) + (pt1.y - pt2.y) * (pt1.y - pt2.y) +
                   (pt1.z - pt2.z) * (pt1.z - pt2.z));
}

/**
* @brief 点到直线间距离
* @param[in] ptP
* @param[in] ptL1
* @param[in] ptL2
* @return double
*/
inline double pointLineDistance(const cv::Point2f& ptP, const cv::Point2f& ptL1,
                              const cv::Point2f& ptL2) noexcept {
  double A = ptL2.y - ptL1.y;
  double B = ptL1.x - ptL2.x;
  double C = ptL2.x * ptL1.y - ptL2.y * ptL1.x;
  double distance = fabs(A * ptP.x + B * ptP.y + C) / sqrt(A * A + B * B);
  return distance;
}

/**
* @brief 点到直线距离
* @param[in] pt
* @param[in] line
* @return double
*/
inline double pointLineDistance(const cv::Point2f& pt, const cv::Vec4f& line) {
  cv::Vec2f line_dir(line[0], line[1]);
  cv::Point2f line_pt(line[2], line[3]);
  return cv::norm((pt - line_pt).cross(line_dir));
}

/**
* @brief 求解一元二次方程，返回解的数值对，第一项小于第二项
* @param[in] a
* @param[in] b
* @param[in] c
* @return std::pair<double, double>
*/
inline std::pair<double, double> solveQuadraticEquation(double a, double b, double c) {
  std::pair<double, double> result((-b - sqrt((double)(b * b - 4 * a * c))) / (2 * a),
                                   (-b + sqrt((double)(b * b - 4 * a * c))) / (2 * a));
  return result;
}

/**
 * @brief 角度转弧度
 * @param[in] angle
 * @return double
 */
inline double angle2Radian(double angle) noexcept { return angle * CV_PI / 180; }

/**
 * @brief 弧度转角度
 * @param[in] radian
 * @return double
 */
inline double radian2Angle(double radian) noexcept { return radian * 180 / CV_PI; }

/**
 * @brief 判断某个值是否在一个范围内。
 * @tparam T
 * @param[in] val           判断的值
 * @param[in] lower         下限
 * @param[in] upper         上限
 * @return true
 * @return false
 */
template <typename T>
constexpr inline bool inRange(T val, T lower, T upper) {
    if (lower > upper) {
        std::swap(lower, upper);
    }
    return val >= lower && val <= upper;
}


}  // namespace tools

#endif  // TOOLS__MATH_TOOLS_HPP