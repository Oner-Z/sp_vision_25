#ifndef AUTO_AIM__TARGET_HPP
#define AUTO_AIM__TARGET_HPP

#include <Eigen/Dense>
#include <chrono>
#include <string>
#include <vector>

#include "armor.hpp"
#include "tools/extended_kalman_filter.hpp"

namespace auto_aim
{

class Target
{
public:
  ArmorName name;
  ArmorType armor_type;
  bool jumped;
  int last_id;  // debug only

  Target() = default;
  Target(
    const Armor & armor, std::chrono::steady_clock::time_point t, double radius, int armor_num,
    Eigen::VectorXd P0_dig, double v1 = 100.0, double v2 = 400.0);

  void predict(std::chrono::steady_clock::time_point t);
  void update(const Armor & armor, std::chrono::steady_clock::time_point t);

  Eigen::VectorXd ekf_x() const;
  std::vector<Eigen::Vector4d> armor_xyza_list() const;

  bool diverged() const;

private:
  int armor_num_;
  tools::ExtendedKalmanFilter ekf_;
  std::chrono::steady_clock::time_point t_;
  std::chrono::steady_clock::time_point t_last_update_;

  double v1_;  // 加速度方差
  double v2_;  // 角加速度方差

  void update_ypda(const Armor & armor, int id);  // yaw pitch distance angle

  Eigen::Vector3d h_armor_xyz(const Eigen::VectorXd & x, int id) const;
  Eigen::MatrixXd h_jacobian(const Eigen::VectorXd & x, int id) const;
};

}  // namespace auto_aim

#endif  // AUTO_AIM__TARGET_HPP