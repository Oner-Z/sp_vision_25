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
  brightness_ = fsDetect["brightness"][enemy_color_].as<int>();
  canny_low_threshold_ = fsDetect["canny_low_threshold"].as<int>();
  canny_high_threshold_ = fsDetect["canny_high_threshold"].as<int>();
  approx_epsilon_ = fsDetect["approx_epsilon"].as<float>();
  R_contours_min_area_ = fsDetect["R_contours_min_area"].as<int>();
  R_contours_max_area_ = fsDetect["R_contours_max_area"].as<int>();
  target_contours_min_area_ = fsDetect["target_contours_min_area"].as<int>();
  target_contours_max_area_ = fsDetect["target_contours_max_area"].as<int>();
  brightness_threshold_ = fsDetect["brightness_threshold"][enemy_color_].as<int>();
  auto standard_fanblade_path = fsDetect["standard_fanblade_path"].as<std::string>();
  standard_fanblade = cv::imread(standard_fanblade_path, cv::IMREAD_GRAYSCALE);
  standard_fanblade_size = standard_fanblade.size();
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
  // imshow("Blue Channel", blue);
  // imshow("Red Channel", red);

  // Step 4: 转换为灰度图
  cv::Mat gray_image;
  if (enemy_color_ == "red")
    gray_image = red - blue * 0.7;
  else
    gray_image = blue - red * 0.8;
  imshow("Gray Image", gray_image);

  // 二值化
  cv::Mat threshold_image;
  cv::threshold(gray_image, threshold_image, brightness_threshold_, 255, cv::THRESH_BINARY);

  // Step 6: 闭运算
  cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(4, 4));
  cv::morphologyEx(threshold_image, threshold_image, cv::MORPH_CLOSE, element);

  // **膨胀操作** (让轮廓更完整)
  cv::dilate(threshold_image, threshold_image, element, cv::Point(-1, -1), 1);

  handled_img = threshold_image;

  // // Step 5: 边缘检测
  // cv::Mat edges;
  // cv::Canny(threshold_image, edges, canny_low_threshold_, canny_high_threshold_);
  // cv::imshow("Edges", edges);

  // // Step 7: 查找轮廓
  // std::vector<std::vector<cv::Point>> contours;
  // cv::findContours(edges, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  // // Step 8: 轮廓近似
  // for (size_t i = 0; i < contours.size(); i++) {
  //   cv::approxPolyDP(contours[i], contours[i], 1.0, true);
  // }

  // // 创建彩色图像（原图大小，3通道）
  // cv::Mat contour_img = cv::Mat::zeros(edges.size(), CV_8UC3);
  // cv::cvtColor(edges, contour_img, cv::COLOR_GRAY2BGR);  // 转换成 3 通道用于绘制彩色轮廓

  // // 随机颜色绘制轮廓并标注面积
  // for (size_t i = 0; i < contours.size(); i++) {
  //   double area = cv::contourArea(contours[i]);

  //   // 生成随机颜色
  //   cv::Scalar color = cv::Scalar(rand() % 256, rand() % 256, rand() % 256);

  //   // 绘制轮廓
  //   cv::drawContours(contour_img, contours, static_cast<int>(i), color, 2);

  //   // 仅在轮廓至少有1个点时添加文本
  //   if (!contours[i].empty()) {
  //     cv::putText(
  //       contour_img,
  //       std::to_string(static_cast<int>(area)),  // 显示整数面积
  //       contours[i][0],                          // 在轮廓第一个点的位置绘制文字
  //       cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 1);
  //   }
  // }

  // // 显示最终的轮廓图像
  // cv::imshow("Contours", contour_img);

  // // Step 9: 绘制并填充轮廓
  // handled_img = cv::Mat::zeros(edges.size(), CV_8UC1);
  // for (size_t i = 0; i < contours.size(); i++) {
  //   double area = cv::contourArea(contours[i]);
  //   if (
  //     (area > target_contours_min_area_ && area < target_contours_max_area_) ||
  //     (area > R_contours_min_area_ && area < R_contours_max_area_)) {
  //     cv::drawContours(handled_img, contours, static_cast<int>(i), cv::Scalar(255), cv::FILLED);
  //   }
  // }

  // cv::imshow("Filled Contours", handled_img);
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
std::vector<std::vector<cv::Point>> get_corners(
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
      corners_list.push_back(contour);
      // // 使用多边形逼近来找到近似的四个角点
      // std::vector<cv::Point> approx;
      // cv::approxPolyDP(contour, approx, 0.02 * cv::arcLength(contour, true), true);

      // // 只保留四个顶点的轮廓
      // if (approx.size() == 4) {
      //   corners_list.push_back(approx);
      // }
    }
  }

  return corners_list;
}

std::optional<PowerRune> Buff_Detector::detect(cv::Mat & bgr_img)
{
  /// get filled_image
  cv::Mat handled_img;
  handle_img(bgr_img, handled_img);
  cv::imshow("handled_img", handled_img);

  /// get fanblades
  auto fanblade = detect_fanblades(handled_img);
  // if ( == false) {
  // m_globalRoi = {
  //   0, 0, static_cast<float>(frame.m_image.cols), static_cast<float>(frame.m_image.rows)};
  // m_lightArmorNum = 0;
  // }
  /// get R_center
  // if (detect_center_r(handled_img) == false) {
  //   // m_status = Status::CENTER_FAILURE;
  // }

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

std::optional<FanBlade> Buff_Detector::detect_fanblades(const cv::Mat & handled_img)
{
  std::optional<FanBlade> F;
  auto corners_list =
    get_corners(handled_img, target_contours_min_area_, target_contours_max_area_);

  cv::Mat output;
  cv::cvtColor(handled_img, output, cv::COLOR_GRAY2BGR);  // 转换为彩色以绘制框

  /// 扇叶头
  double best_match_score = -1;  // 最高匹配度
  cv::Rect best_match_rect;      // 最佳匹配的矩形区域
  cv::Mat result;

  for (const auto & corners : corners_list) {
    cv::Rect bounding_box = cv::boundingRect(corners);

    // 检查矩形是否超出图像范围 跳过不符合宽高比的矩形
    if (
      bounding_box.x < 0 || bounding_box.y < 0 ||
      bounding_box.x + bounding_box.width > handled_img.cols ||
      bounding_box.y + bounding_box.height > handled_img.rows)
      continue;

    double aspect_ratio = (double)bounding_box.width / bounding_box.height;
    if (aspect_ratio < 0.5 || aspect_ratio > 2) continue;

    // 模板匹配
    cv::Mat roi = handled_img(bounding_box);
    cv::resize(roi, roi, standard_fanblade_size, 0, 0, cv::INTER_AREA);
    cv::matchTemplate(roi, standard_fanblade, result, cv::TM_CCOEFF_NORMED);
    double min_val, max_val;
    cv::minMaxLoc(result, &min_val, &max_val);
    if (max_val > best_match_score) {
      best_match_score = max_val;
      best_match_rect = bounding_box;
    }

    // 绘制所有检测的矩形
    cv::rectangle(output, bounding_box, cv::Scalar(255, 255, 0), 2);
    cv::putText(
      output, std::to_string(best_match_score), bounding_box.tl(), cv::FONT_HERSHEY_SIMPLEX, 0.5,
      cv::Scalar(255, 255, 0), 1);
  }

  // 如果没有找到匹配的扇叶
  if (best_match_score < 0.15) {
    tools::logger()->debug("[Buff_Detector] 未找到扇叶头!");
    cv::imshow("detect_fanblades", output);
    return F;
  }

  double best_rotation_angle = 0;  // 最佳匹配的旋转角度

  // cv::Mat roi = handled_img(best_match_rect);
  // cv::resize(roi, roi, standard_fanblade_size, 0, 0, cv::INTER_AREA);
  // // cv::normalize(roi, roi, 0, 255, cv::NORM_MINMAX, CV_8U);
  // // cv::imwrite("result.jpg", roi);
  // for (double angle = 0; angle <= 90; angle += 5) {  // 旋转范围：0° ~ 90°，步长1°
  //   cv::Mat rotated_template;
  //   cv::Mat rotation_matrix = cv::getRotationMatrix2D(
  //     cv::Point2f(standard_fanblade.cols / 2, standard_fanblade.rows / 2), angle, 1.0);
  //   cv::warpAffine(standard_fanblade, rotated_template, rotation_matrix, standard_fanblade.size());
  //   cv::matchTemplate(roi, rotated_template, result, cv::TM_CCOEFF_NORMED);
  //   double min_val, max_val;
  //   cv::minMaxLoc(result, &min_val, &max_val);
  //   // 更新最佳匹配
  //   if (max_val > best_match_score) {
  //     best_match_score = max_val;
  //     best_rotation_angle = angle;
  //   }
  // }

  // 绘制最佳匹配区域
  cv::rectangle(output, best_match_rect, cv::Scalar(0, 255, 0), 2);
  cv::putText(
    output, "Best: " + std::to_string(best_match_score), best_match_rect.tl(),
    cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2);

  cv::Point2f head_center = (best_match_rect.tl() + best_match_rect.br()) / 2;
  auto radius = (best_match_rect.width + best_match_rect.height) / 4;

  /// 扇叶杆
  cv::Point2f box[4];
  int angle = 0;
  bool found = false;

  corners_list = get_corners(handled_img, 1000, 3000);
  std::cout << corners_list.size() << std::endl;
  for (const auto & corners : corners_list) {
    cv::RotatedRect bounding_box = cv::minAreaRect(corners);

    cv::Point2f body_center = bounding_box.center;
    cv::Size2f size = bounding_box.size;

    if (size.width < size.height) {
      bounding_box.angle += 90;  // 调整角度范围，使其始终为 -180°~180°
      if (head_center.y < body_center.y) bounding_box.angle -= 180;
      std::swap(bounding_box.size.width, bounding_box.size.height);
      size = bounding_box.size;
    }

    bounding_box.points(box);
    for (int i = 0; i < 4; i++) {
      cv::line(output, box[i], box[(i + 1) % 4], cv::Scalar(255, 255, 0), 2);
    }

    // 检查矩形是否超出图像范围 跳过不符合宽高比的矩形
    float half_width = size.width / 2.0, half_height = size.height / 2.0;
    if (
      body_center.x - half_width < 0 || body_center.y - half_height < 0 ||
      body_center.x + half_width > handled_img.cols ||
      body_center.y + half_height > handled_img.rows) {
      continue;
    }

    if (
      size.width < radius * 1.5 || size.width > radius * 2.5 || size.height < radius * 0.3 ||
      size.height > radius * 0.8)
      continue;

    auto distance = cv::norm(body_center - head_center);
    if (distance < radius * 2 || distance > radius * 3) continue;

    found = true;
    angle = bounding_box.angle;
    if (head_center.y < body_center.y) {
      cv::swap(box[0], box[2]);
      cv::swap(box[1], box[3]);
    }
    break;
  }

  if (!found) {
    tools::logger()->debug("[Buff_Detector] 未找到扇叶杆!");
    cv::imshow("detect_fanblades", output);
    return F;
  }

  // for (int i = 0; i < 4; i++) {
  //   cv::line(output, box[i], box[(i + 1) % 4], cv::Scalar(0, 255, 0), 2);
  // }

  std::vector<cv::Point2f> kpt;
  cv::Point2f keypoints_center = head_center;
  // best_match_rect box angle
  cv::Mat mask = cv::Mat::zeros(handled_img.size(), CV_8U);
  cv::rectangle(mask, best_match_rect, cv::Scalar(255), cv::FILLED);
  cv::Mat masked_img;
  handled_img.copyTo(masked_img, mask);

  cv::Mat rotation_matrix = cv::getRotationMatrix2D(head_center, angle, 1.0);
  cv::Mat rotated_img;
  cv::warpAffine(masked_img, rotated_img, rotation_matrix, handled_img.size());

  // 去除小噪点
  cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
  cv::morphologyEx(rotated_img, rotated_img, cv::MORPH_OPEN, kernel);  // 形态学开运算
  cv::Mat labels, stats, centroids;                                    // 连通域分析
  int num_labels = cv::connectedComponentsWithStats(rotated_img, labels, stats, centroids);

  for (int i = 1; i < num_labels; i++) {
    int area = stats.at<int>(i, cv::CC_STAT_AREA);
    if (area < 50) rotated_img.setTo(0, labels == i);
  }

  // 寻找 `rotated_img` 中扇叶头的四个关键点
  std::vector<cv::Point> non_zero_pixels;
  cv::findNonZero(rotated_img, non_zero_pixels);

  cv::Point2f top = non_zero_pixels[0], bottom = non_zero_pixels[0];
  cv::Point2f left = non_zero_pixels[0], right = non_zero_pixels[0];

  for (const auto & p : non_zero_pixels) {
    if (p.y < top.y) top = p;
    if (p.y > bottom.y) bottom = p;
    if (p.x < left.x) left = p;
    if (p.x > right.x) right = p;
  }

  // 将 `rotated_img` 关键点转换回原图坐标
  std::vector<cv::Point2f> rotated_points = {top, right, bottom, left};
  std::vector<cv::Point2f> original_points;
  cv::Mat inverse_rotation_matrix;
  cv::invertAffineTransform(rotation_matrix, inverse_rotation_matrix);
  cv::transform(rotated_points, original_points, inverse_rotation_matrix);

  for (const auto & pt : original_points) {
    kpt.push_back(pt);
  }
  for (auto & point : box) {
    kpt.push_back(point);
  }

  // **可视化关键点**
  for (int i = 0; i < kpt.size(); i++) {
    cv::circle(output, kpt[i], 3, cv::Scalar(0, 0, 255), -1);  // 绘制红色点
    cv::putText(
      output, std::to_string(i), kpt[i], cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 1);
  }
  cv::imshow("detect_fanblades", output);

  FanBlade fanblade(kpt, keypoints_center, _light);
  F.emplace(fanblade);
  std::cout << "angle: " << angle << std::endl;
  return F;
}

bool Buff_Detector::detect_center_r(const cv::Mat & handled_img) { return false; }

// std::optional<PowerRune> Buff_Detector::detect(cv::Mat & bgr_img)
// {
//   /// onnx 模型检测

//   std::vector<YOLOV8KP::Object> results = MODE_.get_onecandidatebox(bgr_img);

//   /// 处理未获得的情况

//   if (results.empty()) {
//     handle_lose();
//     return std::nullopt;
//   }

//   /// results转扇叶FanBlade

//   std::vector<FanBlade> fanblades;
//   auto result = results[0];
//   fanblades.emplace_back(FanBlade(result.kpt, result.kpt[4], _light));

//   /// 生成PowerRune
//   auto r_center = get_r_center(fanblades, bgr_img);
//   PowerRune powerrune(fanblades, r_center, last_powerrune_);

//   /// handle error
//   if (powerrune.is_unsolve()) {
//     handle_lose();
//     return std::nullopt;
//   }

//   status_ = TRACK;
//   lose_ = 0;
//   std::optional<PowerRune> P;
//   P.emplace(powerrune);
//   last_powerrune_ = P;
//   return P;
// }

}  // namespace auto_buff