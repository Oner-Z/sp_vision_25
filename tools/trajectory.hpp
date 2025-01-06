#ifndef TOOLS__TRAJECTORY_HPP
#define TOOLS__TRAJECTORY_HPP

namespace tools
{
struct Trajectory
{
  bool unsolvable;
  double fly_time;
  double pitch;  // 抬头为负

  // 不考虑空气阻力
  // v0 子弹初速度大小，单位：m/s
  // d 目标水平距离，单位：m
  // h 目标竖直高度，单位：m
  Trajectory(const double v0, const double d, const double h);// 已经废弃，不考虑空气阻力的已经整合，由mode决定

  // mode=1时考虑空气阻力，mode=0时不考虑
  Trajectory(const double v0, const double d, const double h, const int mode);
};

}  // namespace tools

#endif  // TOOLS__TRAJECTORY_HPP