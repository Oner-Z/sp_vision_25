#include <fmt/core.h>
#include <yaml-cpp/yaml.h>

#include <chrono>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>

#include "io/camera.hpp"
#include "io/cboard.hpp"
#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/yolov8.hpp"
#include "tasks/auto_dart/dart_detector.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"
#include "tools/recorder.hpp"
using namespace std::chrono;

const std::string keys =
  "{help h usage ? |                        | 输出命令行参数说明}"
  "{@config-path   | configs/dart.yaml | 位置参数，yaml配置文件路径 }";

int main(int argc, char * argv[])
{
  tools::Exiter exiter;
  tools::Plotter plotter;
  tools::Recorder recorder(100);  //根据实际帧率调整

  cv::CommandLineParser cli(argc, argv, keys);
  auto config_path = cli.get<std::string>(0);
  if (cli.has("help") || config_path.empty()) {
    cli.printMessage();
    return 0;
  }

  io::CBoard cboard(config_path);
  io::Camera camera(config_path);
  auto yaml = YAML::LoadFile(config_path);
  auto yaw_offset = yaml["yaw_offset"].as<double>();
  auto height_threshold = yaml["height_threshold"].as<double>();
  auto low_threshold = yaml["low_threshold"].as<double>();
  auto_dart::DartDetector detector(config_path);
  cv::Mat img;
  Eigen::Quaterniond q;
  std::chrono::steady_clock::time_point t;
  std::chrono::steady_clock::time_point last_t = std::chrono::steady_clock::now();

  while (!exiter.exit()) {
    camera.read(img, t);
    std::vector<auto_dart::LightSpot> lightspots = detector.detect(img);
    cv::Mat drawlightspot = img.clone();
    for (const auto & lightspot : lightspots) {
      tools::draw_points(drawlightspot, lightspot.contour);
      cv::Mat tvec, rvec;
      detector.solvePnP(lightspot, tvec, rvec);
      tools::draw_text(
        drawlightspot,
        fmt::format(
          "tvec:  x{: .2f} y{: .2f} z{: .2f}", tvec.at<double>(0), tvec.at<double>(1),
          tvec.at<double>(2)),
        cv::Point2f(10, 60), cv::Scalar(0, 255, 255));
      tools::draw_text(
        drawlightspot,
        fmt::format(
          "rvec:  x{: .2f} y{: .2f} z{: .2f}", rvec.at<double>(0), rvec.at<double>(1),
          rvec.at<double>(2)),
        cv::Point2f(10, 260), cv::Scalar(0, 255, 255));

      double yaw = atan2(tvec.at<double>(0), tvec.at<double>(2));
      io::DartCommand command{-yaw - yaw_offset / 57.3};
      cboard.send(command);
    }
    int cx = drawlightspot.cols / 2;
    cv::line(
      drawlightspot, cv::Point(cx, 0), cv::Point(cx, img.rows - 1), cv::Scalar(0, 0, 255), 2);
    double pixels_per_degree = drawlightspot.cols / 30.0;
    int offset_x = static_cast<int>(yaw_offset * pixels_per_degree);
    int line_x = cx + offset_x;
    cv::line(
      drawlightspot, cv::Point(line_x, 0), cv::Point(line_x, img.rows - 1), cv::Scalar(0, 0, 255),
      2);

    int y1 = static_cast<int>(drawlightspot.rows * height_threshold);
    cv::line(
      drawlightspot, cv::Point(0, y1), cv::Point(drawlightspot.cols - 1, y1), cv::Scalar(0, 255, 0),
      2);
    int y2 = static_cast<int>(drawlightspot.rows * low_threshold);
    cv::line(
      drawlightspot, cv::Point(0, y2), cv::Point(drawlightspot.cols - 1, y2), cv::Scalar(0, 255, 0),
      2);
    cv::resize(drawlightspot, drawlightspot, {}, 0.5, 0.5);

    cv::imshow("lightspot", drawlightspot);
    auto key = cv::waitKey(1);
    if (key == 'q') break;
  }

  return 0;
}