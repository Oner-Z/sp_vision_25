#ifndef AUTO_BUFF__SOLVER_HPP
#define AUTO_BUFF__SOLVER_HPP

#include <yaml-cpp/yaml.h>

#include <Eigen/Dense>  // 必须在opencv2/core/eigen.hpp上面
#include <opencv2/core/eigen.hpp>
#include <optional>

#include "buff_type.hpp"
#include "tools/math_tools.hpp"
namespace auto_buff
{
const std::vector<cv::Point3f> OBJECT_POINTS = {
  cv::Point3f(0, 160e-3, 858.5e-3),
  cv::Point3f(0, -160e-3, 858.5e-3),
  cv::Point3f(0, -186e-3, 541.5e-3),
  cv::Point3f(0, 186e-3, 541.5e-3),
  cv::Point3f(0, 0, 0),
  cv::Point3f(0, 0, 700e-3)};  // 单位：米

class Solver
{
public:
  explicit Solver(const std::string & config_path);

  Eigen::Matrix3d R_gimbal2world() const;

  void set_R_gimbal2world(const Eigen::Quaterniond & q);
  void solve(std::optional<PowerRune> & ps) const;

  // 调试用
  cv::Point2f point_buff2pixel(cv::Point3f x);

private:
  cv::Mat camera_matrix_;
  cv::Mat distort_coeffs_;
  Eigen::Matrix3d R_gimbal2imubody_;
  Eigen::Matrix3d R_camera2gimbal_;
  Eigen::Vector3d t_camera2gimbal_;
  Eigen::Matrix3d R_gimbal2world_;

  cv::Vec3d rvec_, tvec_;
};
}  // namespace auto_buff
#endif  // AUTO_AIM__SOLVER_HPP