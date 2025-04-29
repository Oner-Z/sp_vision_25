#include "Boomeranger.hpp"

#include <yaml-cpp/yaml.h>

#include <Eigen/Dense>  // 必须在opencv2/core/eigen.hpp上面
#include <Eigen/Geometry>
#include <opencv2/core/eigen.hpp>
namespace auto_aim
{
Boomeranger::Boomeranger(const std::string config_path)
{
  auto yaml = YAML::LoadFile(config_path);
  auto camera_matrix_data = yaml["camera_matrix"].as<std::vector<double>>();
  auto distort_coeffs_data = yaml["distort_coeffs"].as<std::vector<double>>();
  Eigen::Matrix<double, 3, 3, Eigen::RowMajor> camera_matrix(camera_matrix_data.data());
  Eigen::Matrix<double, 1, 5> distort_coeffs(distort_coeffs_data.data());
  cv::eigen2cv(camera_matrix, camera_matrix_);
  cv::eigen2cv(distort_coeffs, distort_coeffs_);
}
bool Boomeranger::isCircle(const std::vector<cv::Point> & contour)  // 判断这个轮廓是不是圆形
{
  double area = cv::contourArea(contour);           // 计算该轮廓的面积
  if (area < MIN_AREA) return false;                // 排除干扰点
  double perimeter = cv::arcLength(contour, true);  // 计算该轮廓的周长，第二个>参数表示其是否封闭

  // 计算圆形度（PS：比较他两的面积）
  double circularity = (4 * PI * area) / (perimeter * perimeter);
  // 设定一个阈值，接近1的值表示更接近圆形
  return (area > DIVIDE_AREA)
           ? (circularity > BIG_FAMILIAR)
           : (circularity >
              SMALL_FAMILIAR);  // 大的区域圆形度更大，小的区域需要的圆形度更小（因为远距离探测到的光强会小）
}

std::vector<LightSpot> Boomeranger::detect(const cv::Mat & img)
{
  // 方案一，使用灰度图来寻找发光区域
  // cv::Mat gray_img;
  // cv::cvtColor(img, gray_img, cv::COLOR_BGR2GRAY);

  // cv::Mat test;
  // cv::threshold(gray_img, test, 120, 255, cv::THRESH_BINARY);
  // cv::imshow("test", test);

  // 方案二，使用图片绿通道来进行二值化，高斯滤波后找出发绿光且最圆的地方
  std::vector<cv::Mat> channels;
  cv::split(img, channels);
  cv::Mat green = channels.at(1);
  // cv::resize(green, green, {}, 0.5, 0.5);
  // cv::imshow("green", green);
  cv::Mat gauss;
  cv::GaussianBlur(green, gauss, cv::Size(15, 15), 0);  //降噪
  // cv::imshow("gauss", gauss);

  // 进行二值化
  cv::Mat binary_img;
  cv::threshold(gauss, binary_img, 120, 255, cv::THRESH_BINARY);

  // 显示二值图,调试用
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(binary_img, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
  // RETR_EXTERNAL 只测最外围，内侧轮廓忽略
  // CHAIN_APPROX_NONE 保存物体所有边界点信息

  // 把高于画面中心的筛除，这些明显是灯光之类的
  cv::Mat drawcontours = img.clone();
  double middle_height = drawcontours.rows / 2;

  // for (const auto & contour : contours) {
  //   tools::draw_points(drawcontours, contour);
  // }

  // cv::imshow("drawcontours", drawcontours);

  std::vector<std::vector<cv::Point>> circles;
  for (const auto & contour : contours)  // 找到圆形的轮廓保存到circles当中
  {
    if (isCircle(contour)) {
      double area = cv::contourArea(contour);  // 计算该轮廓的面积
      circles.emplace_back(contour);
      tools::draw_points(drawcontours, contour);
    }
  }
  // cv::imshow("drawcontours", drawcontours);

  std::vector<LightSpot> lightspots;
  for (const auto & circle : circles) {
    cv::Point2f center;
    float radius;
    cv::minEnclosingCircle(circle, center, radius);
    //使用OpenCV的函数来得到最小外接圆的半径和圆心
    auto lightspot = LightSpot(circle, radius, center);
    // if (lightspot.center.y > middle_height)  // 筛掉高于画面中心的
    lightspots.emplace_back(lightspot);
  }
  cv::resize(binary_img, binary_img, {}, 0.5, 0.5);
  cv::imshow("binary", binary_img);

  // 方案三，将图片使用用双边滤波后来平滑处理。
  // std::vector<cv::Mat> channels;
  // cv::split(img, channels);
  // cv::Mat green = channels.at(1);

  // cv::Mat median;
  // cv::medianBlur(green, median, 3);

  // cv::Mat bilate;
  // cv::bilateralFilter(median, bilate, 3, 255, 15);

  // cv::Mat binary_img;
  // cv::threshold(bilate, binary_img, 120, 255, cv::THRESH_BINARY);
  // std::vector<std::vector<cv::Point>> contours;
  // cv::findContours(binary_img, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
  // // RETR_EXTERNAL 只测最外围，内侧轮廓忽略
  // // CHAIN_APPROX_NONE 保存物体所有边界点信息

  // // 把高于画面中心的筛除，这些明显是灯光之类的
  // cv::Mat drawcontours = img.clone();
  // double middle_height = drawcontours.rows / 2;

  // // for (const auto & contour : contours) {
  // //   tools::draw_points(drawcontours, contour);
  // // }

  // // cv::imshow("drawcontours", drawcontours);

  // std::vector<std::vector<cv::Point>> circles;
  // for (const auto & contour : contours)  // 找到圆形的轮廓保存到circles当中
  // {
  //   if (isCircle(contour)) {
  //     circles.emplace_back(contour);
  //     tools::draw_points(drawcontours, contour);
  //   }
  // }
  // // cv::imshow("drawcontours", drawcontours);

  // std::vector<LightSpot> lightspots;
  // for (const auto & circle : circles) {
  //   cv::Point2f center;
  //   float radius;
  //   cv::minEnclosingCircle(circle, center, radius);
  //   //使用OpenCV的函数来得到最小外接圆的半径和圆心
  //   auto lightspot = LightSpot(circle, radius, center);
  //   // if (lightspot.center.y > middle_height)  // 筛掉高于画面中心的
  //   lightspots.emplace_back(lightspot);
  // }
  // cv::resize(binary_img, binary_img, {}, 0.5, 0.5);
  // cv::imshow("binary_img", binary_img);
  return lightspots;
}

void Boomeranger::solvePnP(
  const LightSpot lightspot, cv::Mat & tvec, cv::Mat & rvec)  //根据得到的指示灯图像进行坐标转化
{
  std::vector<cv::Point2f> img_points{
    lightspot.top, lightspot.right, lightspot.bottom, lightspot.left};
  cv::solvePnP(lightspot_points, img_points, camera_matrix_, distort_coeffs_, rvec, tvec);
}

//
}  // namespace auto_aim
