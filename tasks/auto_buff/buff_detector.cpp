#include "buff_detector.hpp"

#include "tools/logger.hpp"

namespace auto_buff
{
Buff_Detector::Buff_Detector(const std::string & config_path) : status_(LOSE), lose_(0)
{
  auto yaml = YAML::LoadFile(config_path);

  enemy_color_ = yaml["enemy_color"].as<std::string>();
  auto fsDetect = yaml["detect"];

  contrast_ = fsDetect["contrast"].as<int>();
  brightness_ = fsDetect["brightness"].as<int>();
  canny_low_threshold_ = fsDetect["canny_low_threshold"].as<int>();
  canny_high_threshold_ = fsDetect["canny_high_threshold"].as<int>();
  approx_epsilon_ = fsDetect["approx_epsilon"].as<float>();
  R_contours_min_area_ = fsDetect["R_contours_min_area"].as<int>();
  R_contours_max_area_ = fsDetect["R_contours_max_area"].as<int>();
  target_contours_min_area_ = fsDetect["target_contours_min_area"].as<int>();
  target_contours_max_area_ = fsDetect["target_contours_max_area"].as<int>();
}

void Buff_Detector::handle_img(const cv::Mat & bgr_img, cv::Mat & handled_img)
{
  // Step 1: 读取图像并调整亮度和对比度
  bgr_img.convertTo(bgr_img, -1, contrast_, brightness_);

  // Step 2: 提取颜色通道
  std::vector<cv::Mat> channels;
  cv::split(bgr_img, channels);  // 分离 BGR 通道
  cv::Mat blue = channels[0];    // 蓝色通道
  cv::Mat red = channels[2];     // 红色通道

  // Step 4: 转换为灰度图
  cv::Mat gray_image;
  cv::cvtColor(bgr_img, gray_image, cv::COLOR_BGR2GRAY);

  // Step 5: 边缘检测
  cv::Mat edges;
  cv::Canny(gray_image, edges, 200, 230);
  //   cv::imshow("Edges", edges);

  // Step 6: 闭运算
  cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2, 2));
  cv::morphologyEx(edges, edges, cv::MORPH_CLOSE, element);

  // Step 7: 查找轮廓
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(edges, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  // Step 8: 轮廓近似
  for (size_t i = 0; i < contours.size(); i++) {
    cv::approxPolyDP(contours[i], contours[i], 1.0, true);
  }

  // Step 9: 绘制并填充轮廓
  handled_img = cv::Mat::zeros(edges.size(), CV_8UC1);
  for (size_t i = 0; i < contours.size(); i++) {
    double area = cv::contourArea(contours[i]);
    if (
      (area > target_contours_min_area_ && area < target_contours_max_area_) ||
      (area > R_contours_min_area_ && area < R_contours_max_area_)) {
      cv::drawContours(handled_img, contours, static_cast<int>(i), cv::Scalar(255), cv::FILLED);
    }
  }

  //   cv::imshow("Filled Contours", filled_image);
}

cv::Point2f Buff_Detector::get_r_center(std::vector<FanBlade> & fanblades, cv::Mat & bgr_img)
{
  /// error

  if (fanblades.empty()) {
    tools::logger()->debug("[Buff_Detector] 无法计算r_center!");
    return {0, 0};
  }

  /// 算出大概位置

  cv::Point2f r_center_t = {0, 0};
  for (auto & fanblade : fanblades) {
    auto point5 = fanblade.points[4];  // point5是扇叶的中心
    auto point6 = fanblade.points[5];
    r_center_t += (point6 - point5) * 1.4 + point5;  // TODO
    // r_center_t += 4.7 * point - (4.7 - 1) * fanblade.center;
  }
  r_center_t /= float(fanblades.size());

  /// 处理图片,mask选出大概范围

  cv::Mat dilated_img;
  handle_img(bgr_img, dilated_img);
  double radius = cv::norm(fanblades[0].points[2] - fanblades[0].center) * 0.8;
  cv::Mat mask = cv::Mat::zeros(dilated_img.size(), CV_8U);  // mask
  circle(mask, r_center_t, radius, cv::Scalar(255), -1);
  bitwise_and(dilated_img, mask, dilated_img);               // 将遮罩应用于二值化图像
  tools::draw_point(bgr_img, r_center_t, {255, 255, 0}, 5);  // 调试用
  // cv::imshow("Dilated Image", dilated_img);                // 调试用

  /// 获取轮廓点,矩阵框筛选  TODO

  std::vector<std::vector<cv::Point>> contours;
  auto r_center = r_center_t;
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
  return r_center;
};

void Buff_Detector::handle_lose()
{
  lose_++;
  if (lose_ >= LOSE_MAX) {
    status_ = LOSE;
    last_powerrune_ = std::nullopt;
  }
  status_ = TEM_LOSE;
}

// 函数 1：获取扇叶轮廓的四个边缘点
std::vector<std::vector<cv::Point>> get_fanblade_corners(
  const cv::Mat & binary_img, double area_min, double area_max)
{
  std::vector<std::vector<cv::Point>> corners_list;

  // 查找所有轮廓
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(binary_img, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  for (const auto & contour : contours) {
    double area = cv::contourArea(contour);
    // 只考虑面积在范围内的轮廓
    if (area >= area_min && area <= area_max) {
      // 使用多边形逼近来找到近似的四个角点
      std::vector<cv::Point> approx;
      cv::approxPolyDP(contour, approx, 0.02 * cv::arcLength(contour, true), true);

      // 只保留四个顶点的轮廓
      if (approx.size() == 4) {
        corners_list.push_back(approx);
      }
    }
  }

  return corners_list;
}

std::optional<PowerRune> Buff_Detector::detect(cv::Mat & bgr_img)
{
  /// get filled_image
  cv::Mat handled_img;
  handle_img(bgr_img, handled_img);

  /// get fanblades

  /// get R_center

  /// 处理未获得的情况
  //   if (handled_img.empty()) {
  //     handle_lose();
  //     return std::nullopt;
  //   }

  //   /// results转扇叶FanBlade
  //   std::vector<FanBlade> fanblades;
  //   for (auto & result : results) fanblades.emplace_back(FanBlade(result.kpt, result.kpt[4], _light));

  //   /// 生成PowerRune
  //   auto r_center = get_r_center(fanblades, bgr_img);
  //   PowerRune powerrune(fanblades, r_center, last_powerrune_);

  //   /// handle error
  //   if (powerrune.is_unsolve()) {
  //     handle_lose();
  //     return std::nullopt;
  //   }

  status_ = TRACK;
  lose_ = 0;
  std::optional<PowerRune> P;
  //   P.emplace(powerrune);
  last_powerrune_ = P;
  return P;
}

}  // namespace auto_buff