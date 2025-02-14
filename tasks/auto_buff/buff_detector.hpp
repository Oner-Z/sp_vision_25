#ifndef AUTO_BUFF__TRACK_HPP
#define AUTO_BUFF__TRACK_HPP

#include <yaml-cpp/yaml.h>

#include <deque>
#include <optional>

#include "buff_type.hpp"
#include "tools/img_tools.hpp"

const int LOSE_MAX = 20;  // 丢失的阙值
namespace auto_buff
{
class Buff_Detector
{
public:
  Buff_Detector(const std::string & config);

  std::optional<PowerRune> detect_24(cv::Mat & bgr_img);

  std::optional<PowerRune> detect(cv::Mat & bgr_img);

private:
  void handle_img(const cv::Mat & bgr_img, cv::Mat & handled_img);

  std::optional<FanBlade> detect_fanblades(const cv::Mat & handled_img);

  bool detect_center_r(const cv::Mat & handled_img);

  cv::Point2f get_r_center(std::vector<FanBlade> & fanblades, cv::Mat & bgr_img);

  void handle_lose();

  std::string enemy_color_;
  int contrast_;
  int brightness_;
  int canny_low_threshold_;
  int canny_high_threshold_;
  float approx_epsilon_;
  int R_contours_min_area_;
  int R_contours_max_area_;
  int target_contours_min_area_;
  int target_contours_max_area_;
  int brightness_threshold_;
  cv::Mat standard_fanblade;
  cv::Size standard_fanblade_size;

  Track_status status_;
  int lose_;  // 丢失的次数
  double lastlen_;
  std::optional<PowerRune> last_powerrune_ = std::nullopt;
};
}  // namespace auto_buff
#endif  // DETECTOR_HPP