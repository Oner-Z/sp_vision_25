#ifndef TOOLS__IMG_TOOLS_HPP
#define TOOLS__IMG_TOOLS_HPP

#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

namespace tools
{
const cv::Scalar COLOR_RED = {0, 0, 255};
const cv::Scalar COLOR_GREEN = {0, 255, 0};
const cv::Scalar COLOR_BLUE = {255, 0, 0};
const cv::Scalar COLOR_YELLOW = {0, 255, 255};
const cv::Scalar COLOR_CYAN = {255, 255, 0};
const cv::Scalar COLOR_MAGENTA = {255, 0, 255};
const cv::Scalar COLOR_WHITE = {255, 255, 255};
const cv::Scalar COLOR_BLACK = {0, 0, 0};

void draw_point(
  cv::Mat & img, const cv::Point & point, const cv::Scalar & color = {0, 0, 255}, int radius = 3);

void draw_points(
  cv::Mat & img, const std::vector<cv::Point> & points, const cv::Scalar & color = {0, 0, 255},
  int thickness = 2);

void draw_points(
  cv::Mat & img, const std::vector<cv::Point2f> & points, const cv::Scalar & color = {0, 0, 255},
  int thickness = 2);

void draw_circle(
  cv::Mat & img, const cv::Point & center, const cv::Scalar & color = COLOR_GREEN, int radius = 10,
  int thickness = 6);

void draw_text(
  cv::Mat & img, const std::string & text, const cv::Point & point,
  const cv::Scalar & color = {0, 255, 255}, double font_scale = 1.0, int thickness = 2);

}  // namespace tools

#endif  // TOOLS__IMG_TOOLS_HPP