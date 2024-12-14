#ifndef BUFF__TYPE_HPP
#define BUFF__TYPE_HPP

#include <algorithm>
#include <deque>
#include <eigen3/Eigen/Dense>  // 必须在opencv2/core/eigen.hpp上面
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

#include "tools/math_tools.hpp"
namespace auto_buff
{
const int INF = 1000000;
enum PowerRune_type { SMALL, BIG };
enum FanBlade_type { _target, _unlight, _shot };
enum Track_status { TRACK, TEM_LOSE, LOSE };
class FanBlade
{
public:
  std::string color;
  cv::Point2f center, r_center, top, bottom;
  std::vector<cv::Point2f> points;
  double angle, width, height;
  FanBlade_type type;
  explicit FanBlade() = default;
  explicit FanBlade(
    const std::vector<cv::Point2f> & kpt, cv::Point2f keypoints_center, FanBlade_type t);

  explicit FanBlade(
    const std::vector<cv::Point2f> & kpt, cv::Point2f keypoints_center, cv::Point2f r_center,
    FanBlade_type t);
};

class PowerRune
{
public:
  cv::Point2f r_center;
  FanBlade target;

  Eigen::Vector3d xyz_in_world;  // 单位：m
  Eigen::Vector3d ypr_in_world;  // 单位：rad
  Eigen::Vector3d ypd_in_world;  // 球坐标系

  Eigen::Vector3d blade_xyz_in_world;  // 单位：m
  Eigen::Vector3d blade_ypd_in_world;  // 球坐标系, 单位: m

  explicit PowerRune(const FanBlade & t);
  explicit PowerRune() = default;
};
}  // namespace auto_buff
#endif  // BUFF_TYPE_HPP
