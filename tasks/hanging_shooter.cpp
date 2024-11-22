#include "hanging_shooter.hpp"

using namespace hanging_shooting;
bool HangingShooter::isCircle(const std::vector<cv::Point> & contour)  // 判断这个轮廓是不是圆形
{
  double area = cv::contourArea(contour);  // 计算该轮廓的面积
  if (area < MIN_AREA) return false;       // 排除干扰点

  double perimeter = cv::arcLength(contour, true);  // 计算该轮廓的周长，第二个>参数表示其是否封闭

  // 计算圆形度（PS：比较他两的面积）
  double circularity = (4 * PI * area) / (perimeter * perimeter);

  // 设定一个阈值，接近1的值表示更接近圆形(PS::鉴于分辨率有限，阀值并不是很大。>。。)
  return (circularity > FAMILIAR);  // 可以根据需要调整阈值
}

std::vector<LightSpot> HangingShooter::detect(const cv::Mat & img)
{
  // 方案一，暴力找指示灯颜色上下值，强行过滤。
  // 问题： 上下值会根据灯的不同而改变，鲁棒性存疑。
  // cv::Mat hsvImage;
  // cv::cvtColor(frame, hsvImage, cv::COLOR_BGR2HSV);

  // // 定义颜色范围
  // cv::Scalar lowerBound(20, 40, 40);    // 绿色下界   60 100 100
  // cv::Scalar upperBound(50, 255, 130);  // 绿色上界   120 255 255

  // cv::Mat mask;
  // // 应用颜色过滤
  // cv::inRange(hsvImage, lowerBound, upperBound, mask);
  // cv::resize(mask, mask, {}, 0.5, 0.5);
  // cv::imshow("mask", mask);

  // 方案二，直接灰度图找最亮的地方。
  cv::Mat gray_img;
  cv::cvtColor(img, gray_img, cv::COLOR_BGR2GRAY);

  // 进行二值化
  cv::Mat binary_img;
  cv::threshold(gray_img, binary_img, 120, 255, cv::THRESH_BINARY);
  // 显示二值图,调试用
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(binary_img, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
  // RETR_EXTERNAL 只测最外围，内侧轮廓忽略
  // CHAIN_APPROX_NONE 保存物体所有边界点信息

  cv::Mat drawcontours = img.clone();
  for (const auto & contour : contours) {
    tools::draw_points(drawcontours, contour);
  }

  //   cv::imshow("drawcontours", drawcontours);

  std::vector<std::vector<cv::Point>> circles;
  for (const auto & contour : contours)  // 找到圆形的轮廓保存到circles当中
  {
    if (isCircle(contour)) {
      circles.emplace_back(contour);
    }
  }
  std::vector<LightSpot> lightspots;
  for (const auto & circle : circles) {
    cv::Point2f center;
    float radius;
    cv::minEnclosingCircle(circle, center, radius);
    //使用OpenCV的函数来得到最小外接圆的半径和圆心
    auto lightspot = LightSpot(circle, radius, center);

    lightspots.emplace_back(lightspot);
  }
  cv::resize(binary_img, binary_img, {}, 0.5, 0.5);
  cv::imshow("binary", binary_img);
  return lightspots;
}

void HangingShooter::solvePnP(
  const LightSpot lightspot, cv::Mat & tvec,
  cv::Mat & rvec)  //根据得到的指示灯图像进行坐标转化
{
  std::vector<cv::Point2f> img_points{
    lightspot.top, lightspot.right, lightspot.bottom, lightspot.left};
  cv::solvePnP(lightspot_points, img_points, camera_matrix, distort_coeffs, rvec, tvec);
}

//