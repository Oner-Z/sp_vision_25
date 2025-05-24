#include "hanging_shooter.hpp"

namespace auto_base
{
HangingShooter::HangingShooter(const std::string & config_path)
{
  auto yaml = YAML::LoadFile(config_path);
  g_ = yaml["g"].as<double>();
  m_ = yaml["m"].as<double>();
  C_ = yaml["C"].as<double>();
  rho_ = yaml["rho"].as<double>();
  BigBulletRadius_ = yaml["BigBulletRadius"].as<double>();
  PI_ = yaml["PI"].as<double>();
  base_position_ = yaml["base_position"].as<std::vector<double>>();
  hero_on_bottom_ = yaml["hero_on_bottom"].as<double>();
  hero_on_center_ = yaml["hero_on_center"].as<double>();
  location_ = {0.0,0.0,0.0};
}


/*
输入参数：
x_vals       打击距离数组
y_vals       打击高度数组     
angle        发射角度     单位：度
bullet_speed 发射速度     单位：m/s
dt           步长        单位： s
max_time     极限时间     单位：s

作用：根据输入角度和速度，使用四阶龙格库塔进行弹道解算，获得离散的弹道映射关系，结果存储在数组当中
*/
void HangingShooter::RungeKutta_4(
  std::vector<double> & x_vals, std::vector<double> & y_vals, double angle, double bullet_speed,
  double height, double dt, double max_time)
{
  angle = angle * PI_ / 180.0;  //先转化为弧度制
  double vx = bullet_speed * cos(angle);
  double vy = bullet_speed * sin(angle);
  // 显然初始位置为(0,0)
  x_vals.emplace_back(0);
  y_vals.emplace_back(0);
  double x = 0, y = 0;
  double t = 0;
  double S = PI_ * BigBulletRadius_ * BigBulletRadius_;  // 大弹丸截面积
  double D = C_ * rho_ * S / 2;  // 空气阻力的影响系数，推导请看知识库
  // 开始迭代求解
  bool is_fall = false;
  while (y >= 0 && t < max_time)  // 弹丸落地和超出时间范围停止迭代
  {
    if (y >= height) is_fall = true;
    if (y < height && is_fall)  //开始下落且已到达目标高度则停止迭代
      break;
    double k1x = vx;
    double k1y = vy;
    double k1vx = -D / m_ * k1x * k1x;
    double k1vy = -g_ - D / m_ * k1y * k1y;

    double k2x = k1x + 0.5 * dt * k1vx;
    double k2y = k1y + 0.5 * dt * k1vy;
    double k2vx = k1vx + 0.5 * dt * (-D / m_ * k2x * k2x);
    double k2vy = k1vy + 0.5 * dt * (-g_ - D / m_ * k2y * k2y);

    double k3x = vx + 0.5 * dt * k2vx;
    double k3y = vy + 0.5 * dt * k2vy;
    double k3vx = k2vx + 0.5 * dt * (-D / m_ * k3x * k3x);
    double k3vy = k2vy + 0.5 * dt * (-g_ - D / m_ * k3y * k3y);

    double k4x = vx + dt * k3vx;
    double k4y = vy + dt * k3vy;
    double k4vx = k3vx + 0.5 * dt * (-D / m_ * k4x * k4x);
    double k4vy = k3vy + 0.5 * dt * (-g_ - D / m_ * k4y * k4y);

    // 更新状态
    x += dt / 6 * (k1x + 2 * k2x + 2 * k3x + k4x);
    y += dt / 6 * (k1y + 2 * k2y + 2 * k3y + k4y);
    vx += dt / 6 * (k1vx + 2 * k2vx + 2 * k3vx + k4vx);
    vy += dt / 6 * (k1vy + 2 * k2vy + 2 * k3vy + k4vy);
    // 存储轨迹
    x_vals.emplace_back(x);
    y_vals.emplace_back(y);
    // 计算下一时间点x和y的值
    t += dt;
  }
}

// 考虑空气阻力计算吊射角度  ,具体的解算流程请见知识库
double HangingShooter::calculate_pitch(double distance,double bullet_speed)
{
  double height;
  if(distance>12)
    height = hero_on_bottom_;
  else
    height = hero_on_center_;
  tools::logger()->info("[cal_data] distance: {} height: {}",distance,height);
  std::vector<double> errors;  // 存储每个角度与实际打击位置的误差值
  // 开始迭代，存储每个角度对应的误差值
  for (double angle = 5.0; angle < 45.0; angle += 0.01)  // 初始角度设置为5度角，迭代到45度
  {
    std::vector<double> x_vals, y_vals;
    RungeKutta_4(x_vals, y_vals, angle, bullet_speed, height);
    double x_simple = x_vals.back(), y_simple = y_vals.back();
    double error = sqrt(
      (x_simple - distance) * (x_simple - distance) +
      (y_simple - height) * (y_simple - height));  
    errors.emplace_back(error);
  }

  auto min_iter = std::min_element(errors.begin(), errors.end());  // 找到落地点与实际位置误差最小的角度
  return (min_iter - errors.begin()) / 100.0 + 5.0;  // 得到最佳发射角度
  
}
io::Command HangingShooter::aim(Eigen::Quaterniond q,io::LocationInfo info,const double bullet_speed)
{
  Eigen::Vector3d ypr = tools::eulers(q,2, 1, 0);
  double pitch,distance;
  if(info.value)
  {
    location_ = info;
    distance = sqrt(
      (location_.x -base_position_[0]) * (location_.x - base_position_[0]) +
      (location_.y -base_position_[1]) * (location_.y -base_position_[1]));
  }
  else
    distance = 15.7;
  tools::logger()->info("bullet_speed : {}",bullet_speed);
  if(bullet_speed<15.0)
    pitch = HangingShooter::calculate_pitch(distance,15.6);
  else
    pitch = HangingShooter::calculate_pitch(distance,bullet_speed);
  pitch/=57.3;
  return {true, false, ypr[0], -pitch};
}
}  // namespace auto_aim