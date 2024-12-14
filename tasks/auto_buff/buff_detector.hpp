#ifndef AUTO_BUFF__TRACK_HPP
#define AUTO_BUFF__TRACK_HPP

#include <yaml-cpp/yaml.h>

#include <deque>
#include <optional>

#include "buff_type.hpp"
#include "tools/img_tools.hpp"
#include "yolov8kp.hpp"
const int LOSE_MAX = 10;  // 丢失的阙值
namespace auto_buff
{
class Buff_Detector
{
public:
  Buff_Detector(const std::string & config);

  std::optional<PowerRune> detect(cv::Mat & bgr_img);

private:
  void Handle_img(const cv::Mat & bgr_img, cv::Mat & dilated_img);

  YOLOV8KP MODE_;
  Track_status status_;
  int lose_;  // 丢失的次数
  double lastlen_;
};
}  // namespace auto_buff
#endif  // DETECTOR_HPP