#include "buff_type.hpp"
namespace auto_buff
{
FanBlade::FanBlade(
  const std::vector<cv::Point2f> & kpt, cv::Point2f keypoints_center, FanBlade_type t)
: center(keypoints_center), r_center(kpt[2]), type(t)
{
  points.emplace_back(kpt[0]);
  points.emplace_back(kpt[4]);
  points.emplace_back(kpt[3]);
  points.emplace_back(kpt[1]);
}

FanBlade::FanBlade(
  const std::vector<cv::Point2f> & kpt, cv::Point2f keypoints_center, cv::Point2f r_center,
  FanBlade_type t)
: center(keypoints_center), r_center(r_center), type(t)
{
  points.emplace_back(kpt[2]);
  points.emplace_back(kpt[1]);
  points.emplace_back(kpt[0]);
  points.emplace_back(kpt[3]);
}

PowerRune::PowerRune(const FanBlade & t) : r_center(t.r_center), target(t){};
}  // namespace auto_buff
