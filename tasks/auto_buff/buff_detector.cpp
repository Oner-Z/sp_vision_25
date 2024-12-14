#include "buff_detector.hpp"

namespace auto_buff
{
Buff_Detector::Buff_Detector(const std::string & config) : status_(LOSE), lose_(0), MODE_(config) {}

void Buff_Detector::Handle_img(const cv::Mat & bgr_img, cv::Mat & dilated_img)
{
  // 彩色图转灰度图
  cv::Mat gray_img;
  cv::cvtColor(bgr_img, gray_img, cv::COLOR_BGR2GRAY);  // 彩色图转灰度图
  // cv::imshow("gray", gray_img);  // 调试用

  // 进行二值化           :把高于100变成255，低于100变成0
  cv::Mat binary_img;
  cv::threshold(gray_img, binary_img, 100, 255, cv::THRESH_BINARY);
  // cv::imshow("binary", binary_img);  // 调试用

  // 膨胀
  cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));  // 使用矩形核
  cv::dilate(binary_img, dilated_img, kernel, cv::Point(-1, -1), 1);
  // cv::imshow("Dilated Image", dilated_img);  // 调试用
}

std::optional<PowerRune> Buff_Detector::detect(cv::Mat & bgr_img)
{
  /// onnx 模型检测

  std::vector<YOLOV8KP::Object> results = MODE_.work(bgr_img);

  /// 处理未获得的情况

  if (results.empty()) {
    lose_++;
    if (lose_ >= LOSE_MAX) status_ = LOSE;
    return std::nullopt;
  }

  cv::Point2f keypoints_center;  // 扇叶四点中心点
  cv::Point2f r_center;          // buff的R标中心点
  YOLOV8KP::Object result;
  result = results[0];
  keypoints_center = result.kpt[4];

  /// 处理error

  if (status_ == LOSE)
    lastlen_ = cv::norm(result.kpt[1] - result.kpt[0]);
  else {
    double len = cv::norm(result.kpt[1] - result.kpt[0]);
    double ratio = lastlen_ > len ? len / lastlen_ : lastlen_ / len;
    if (ratio < 0.8) {
      lose_++;
      if (lose_ >= LOSE_MAX) status_ = LOSE;
      return std::nullopt;
    }
  }

  /// 处理获得的情况

  // 计算r_center    先算出大概位置，然后mask选出大概范围，最后矩阵框筛选
  cv::Point2f kpt30 = (result.kpt[3] + result.kpt[0]) / 2;  // 算出大概位置
  cv::Point2f r_center_t = 4.7 * kpt30 - (4.7 - 1) * result.kpt[4];

  // 处理图片,mask选出大概范围
  cv::Mat dilated_img;
  Handle_img(bgr_img, dilated_img);
  double radius = cv::norm(result.kpt[0] - result.kpt[4]) * 0.8;
  cv::Mat mask = cv::Mat::zeros(dilated_img.size(), CV_8U);  // mask
  circle(mask, r_center_t, radius, cv::Scalar(255), -1);
  bitwise_and(dilated_img, mask, dilated_img);  // 将遮罩应用于二值化图像
  tools::draw_point(bgr_img, r_center_t, {255, 0, 0}, 5);
  // cv::imshow("Dilated Image", dilated_img);  // 调试用

  // 获取轮廓点,矩阵框筛选
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(
    dilated_img, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);  // external找外部区域
  double ratio_1 = INF;
  for (auto & it : contours) {
    auto rotated_rect = cv::minAreaRect(it);
    double ratio = rotated_rect.size.height > rotated_rect.size.width
                     ? rotated_rect.size.height / rotated_rect.size.width
                     : rotated_rect.size.width / rotated_rect.size.height;
    ratio += cv::norm(rotated_rect.center - r_center_t) / (radius / 3);
    if (ratio < ratio_1) {
      ratio_1 = ratio;
      r_center = rotated_rect.center;
    }
  }

  // 更新
  lose_ = 0;
  status_ = TRACK;

  // 返回power_rune
  FanBlade target(result.kpt, keypoints_center, r_center, _target);  // 生成FanBlade
  PowerRune power_rune(target);                                      // 生成PowerRune
  std::optional<PowerRune> P;
  P.emplace(power_rune);
  return P;
}
}  // namespace auto_buff