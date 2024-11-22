#include "hanging_shooter.hpp"
using namespace hanging_shooting;
/*
bullet_speed 单位：m/s
distance     单位：m      x
height       单位：m      y
yaw          单位：rad
*/
void h_solver::calculate_pitch_noAirResistance(
  double bullet_speed, double distance, double height, double & pitch)
{
  // 具体的解算流程请见知识库
  // 不考虑空气阻力就是个简单的一元二次方程，靠公式解算后选择飞行时间更短的即可。
  double a = 0.5 * g * (distance * distance) / (bullet_speed * bullet_speed);
  double b = -distance;
  double c = a - height;
  double result_1 = (-b + sqrt(b * b - 4 * a * c)) / (2 * a);
  double result_2 = (-b - sqrt(b * b - 4 * a * c)) / (2 * a);
  double pitch_1 = atan(result_1);
  double pitch_2 = atan(result_2);

  double t_1 = distance / (bullet_speed * cos(pitch_1));
  double t_2 = distance / (bullet_speed * cos(pitch_2));

  if (t_1 < t_2)
    pitch = pitch_1;
  else
    pitch = pitch_2;
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
void h_solver::RungeKutta_4(
  std::vector<double> & x_vals, std::vector<double> & y_vals, double angle, double bullet_speed,
  double height, double dt, double max_time)
{
  angle = angle * PI / 180.0;  //先转化为弧度制
  double vx = bullet_speed * cos(angle);
  double vy = bullet_speed * sin(angle);

  // 显然初始位置为(0,0)
  x_vals.emplace_back(0);
  y_vals.emplace_back(0);
  double x = 0, y = 0;
  double t = 0;
  double S = PI * BIG_BULLET_RADIUS * BIG_BULLET_RADIUS;  // 大弹丸截面积
  double D = C * rho * S / 2;  // 空气阻力的影响系数，推导请看知识库
  // 开始迭代求解
  bool is_fall = false;
  while (y >= 0 && t < max_time)  // 弹丸落地和超出时间范围停止迭代
  {
    if (y >= height) is_fall = true;
    if (y < height && is_fall)  //开始下落且已到达目标高度则停止迭代
      break;
    double k1x = vx;
    double k1y = vy;
    double k1vx = -D / m * vx;
    double k1vy = -g - D / m * vy;
    double k2x = vx + 0.5 * dt * k1vx;
    double k2y = vy + 0.5 * dt * k1vy;
    double k2vx = -D / m * (vx + 0.5 * dt * k1vx);
    double k2vy = -g - D / m * (vy + 0.5 * dt * k1vy);
    double k3x = vx + 0.5 * dt * k2vx;
    double k3y = vy + 0.5 * dt * k2vy;
    double k3vx = -D / m * (vx + 0.5 * dt * k2vx);
    double k3vy = -g - D / m * (vy + 0.5 * dt * k2vy);
    double k4x = vx + dt * k3vx;
    double k4y = vy + dt * k3vy;
    double k4vx = -D / m * (vx + dt * k3vx);
    double k4vy = -g - D / m * (vy + dt * k3vy);
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
    std::cout << dt << std::endl;
    // std::cout << t << std::endl;
    std::cout << max_time << std::endl;
  }
}

// 考虑空气阻力计算吊射角度
void h_solver::calculate_pitch(double bullet_speed, double distance, double height, double & pitch)
{
  std::vector<double> errors;  // 存储每个角度与实际打击位置的误差值

  // 开始迭代，存储每个角度对应的误差值
  for (double angle = 10.0; angle < 85; angle++)  // 初始角度设置为10度角，迭代到85度
  {
    std::vector<double> x_vals, y_vals;
    RungeKutta_4(x_vals, y_vals, angle, angle, bullet_speed, height);
    double x_simple = x_vals.back(), y_simple = y_vals.back();
    double error = sqrt(
      (x_simple - distance) * (x_simple - distance) +
      (y_simple - height) * (y_simple - height));  // 计算方差
    errors.emplace_back(error);
    auto min_iter =
      std::min_element(errors.begin(), errors.end());  // 找到落地点与实际位置误差的角度
    pitch = (min_iter - errors.begin() + 10.0) / 180.0 *
            PI;  // 得到最佳发射角度，初始值是10度所以要加上10再转化为弧度制
  }
}