#ifndef HANGING_SHOOTER_HPP
#define HANGING_SHOOTER_HPP
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "io/ros2/ros2.hpp"
#include "io/command.hpp"
namespace auto_base
{

class HangingShooter  //英雄吊射基地处理函数类
{
public:
  HangingShooter(const std::string & config_path);
  io::Command aim(Eigen::Quaterniond q,io::LocationInfo info,const double bullet_speed);
private:
  double g_;
  double m_;
  double BigBulletRadius_;  // 大弹丸半径
  double C_;                // 空气阻力系数
  double rho_;              // 空气密度
  double PI_;
  double hero_on_center_;   
  double hero_on_bottom_;
  // 目标在地图坐标系下位置
  std::vector<double>base_position_;
  io::LocationInfo location_; 
  double calculate_pitch(double x,double y,double z,double bullet_speed);  // 考虑空气阻力的弹道解算
  void RungeKutta_4(std::vector<double> & x_vals, std::vector<double> & y_vals, double angle, double bullet_speed,
    double height, double dt = 0.01,
    double max_time = 10);  // 使用4阶龙格库塔推导弹道方程
};


};  // namespace auto_aim
#endif
