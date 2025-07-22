#include <fmt/core.h>
#include <yaml-cpp/yaml.h>

#include <chrono>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>

#include "io/camera.hpp"
#include "io/cboard.hpp"
#include "tasks/auto_dart/dart_detector.hpp"
#include "tools/dart_recorder.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"
using namespace std::chrono;

const std::string keys =
  "{help h usage ? |                        | 输出命令行参数说明}"
  "{@config-path   | configs/dart.yaml | 位置参数，yaml配置文件路径 }";

int main(int argc, char * argv[])
{
  tools::Exiter exiter;
  tools::Plotter plotter;
  tools::DartRecorder recorder(100);  //根据实际帧率调整

  cv::CommandLineParser cli(argc, argv, keys);
  auto config_path = cli.get<std::string>(0);
  if (cli.has("help") || config_path.empty()) {
    cli.printMessage();
    return 0;
  }

  io::CBoard cboard(config_path);
  io::Camera camera(config_path);
  auto_dart::DartDetector detector(config_path, true);
  cv::Mat img;
  std::chrono::steady_clock::time_point t;

  while (!exiter.exit()) {
    camera.read(img, t);
    recorder.record(img, t);
    std::vector<auto_dart::LightSpot> lightspots = detector.detect(img);

    if (!lightspots.empty()) {
      auto command = detector.aim(lightspots.front(), cboard.offset);
      cboard.send(command);
    }
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
    }
    detector.draw_detect_area(drawlightspot);
    cv::resize(drawlightspot, drawlightspot, {}, 0.5, 0.5);
    cv::imshow("lightspot", drawlightspot);
    auto key = cv::waitKey(1);
    if (key == 'q') break;
  }

  return 0;
}