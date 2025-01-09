#include "trajectory.hpp"

#include <cmath>

namespace tools
{
constexpr double g = 9.7946;
constexpr int MAX_ITER = 100;
/* 
  假设空气阻力 f = k1 * v, 下方是k1的组成
  C_d = 0.47   无量纲系数，一般球体都用这个值
  p = 1.169    空气密度（kg/m^3）
  r = 0.02125  弹丸半径（m）
  m = 0.041    弹丸质量（kg）
*/
constexpr double k1 = (0.47 * 1.169 * M_PI * 0.02125 * 0.02125) / (2 * 0.041);

Trajectory::Trajectory(const double v0, const double d, const double h)
{
  auto a = g * d * d / (2 * v0 * v0);
  auto b = -d;
  auto c = a + h;
  auto delta = b * b - 4 * a * c;

  if (delta < 0) {
    unsolvable = true;
    return;
  }

  unsolvable = false;
  auto tan_pitch_1 = (-b + std::sqrt(delta)) / (2 * a);
  auto tan_pitch_2 = (-b - std::sqrt(delta)) / (2 * a);
  auto pitch_1 = std::atan(tan_pitch_1);
  auto pitch_2 = std::atan(tan_pitch_2);
  auto t_1 = d / (v0 * std::cos(pitch_1));
  auto t_2 = d / (v0 * std::cos(pitch_2));

  pitch = (t_1 < t_2) ? pitch_1 : pitch_2;
  fly_time = (t_1 < t_2) ? t_1 : t_2;
}

Trajectory::Trajectory(const double v0, const double d, const double h, int mode)
{
  // 这是没考虑空气阻力的版本
  if (mode == 0) {
    auto a = g * d * d / (2 * v0 * v0);
    auto b = -d;
    auto c = a + h;
    auto delta = b * b - 4 * a * c;

    if (delta < 0) {
      unsolvable = true;
      return;
    }

    unsolvable = false;
    auto tan_pitch_1 = (-b + std::sqrt(delta)) / (2 * a);
    auto tan_pitch_2 = (-b - std::sqrt(delta)) / (2 * a);
    auto pitch_1 = std::atan(tan_pitch_1);
    auto pitch_2 = std::atan(tan_pitch_2);
    auto t_1 = d / (v0 * std::cos(pitch_1));
    auto t_2 = d / (v0 * std::cos(pitch_2));

    pitch = (t_1 < t_2) ? pitch_1 : pitch_2;
    fly_time = (t_1 < t_2) ? t_1 : t_2;

    return;
  }

  // 这是考虑空气阻力的版本 mode = 1
  if (d < 1e-6) {
    unsolvable = true;
    return;
  }
  double theta = std::atan(h / d);
  double delta_z;
  double center_distance = d;  // 平面距离
  double flyTime;
  for (int i = 0; i < MAX_ITER; i++) {
    // 计算炮弹的飞行时间
    flyTime = (pow(M_E, k1 * center_distance) - 1) / (k1 * v0 * cos(theta));
    delta_z = h - v0 * sin(theta) * flyTime / cos(theta) + 0.5 * g * flyTime * flyTime / cos(theta) / cos(theta);
    if (fabs(delta_z) < 1e-6) break;
    theta -=
      delta_z / (-(v0 * flyTime) / pow(cos(theta), 2) + g * flyTime * flyTime / (v0 * v0) * sin(theta) / pow(cos(theta), 3));
  }
  unsolvable = false;
  fly_time = flyTime;
  pitch = theta;
}

}  // namespace tools