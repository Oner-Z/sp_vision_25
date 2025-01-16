#ifndef TOOLS__IMG_TOOLS_HPP
#define TOOLS__IMG_TOOLS_HPP

#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

namespace color
{
const cv::Scalar RED = {0, 0, 255};
const cv::Scalar GREEN = {0, 255, 0};
const cv::Scalar BLUE = {255, 0, 0};
const cv::Scalar YELLOW = {0, 255, 255};
const cv::Scalar CYAN = {255, 255, 0};
const cv::Scalar MAGENTA = {255, 0, 255};
const cv::Scalar WHITE = {255, 255, 255};
const cv::Scalar BLACK = {0, 0, 0};
}  // namespace color

namespace tools
{
void draw_point(
  cv::Mat & img, const cv::Point & point, const cv::Scalar & color = {0, 0, 255}, int radius = 3);

void draw_points(
  cv::Mat & img, const std::vector<cv::Point> & points, const cv::Scalar & color = {0, 0, 255},
  int thickness = 2);

void draw_points(
  cv::Mat & img, const std::vector<cv::Point2f> & points, const cv::Scalar & color = {0, 0, 255},
  int thickness = 2);

void draw_circle(
  cv::Mat & img, const cv::Point & center, const cv::Scalar & color = color::GREEN, int radius = 10,
  int thickness = 6);

void draw_text(
  cv::Mat & img, const std::string & text, const cv::Point & point,
  const cv::Scalar & color = {0, 255, 255}, double font_scale = 1.0, int thickness = 2);

}  // namespace tools

#endif  // TOOLS__IMG_TOOLS_HPP