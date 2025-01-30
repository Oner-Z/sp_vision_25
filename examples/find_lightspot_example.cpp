#include <fmt/format.h>

#include <opencv2/opencv.hpp>

#include "io/camera.hpp"
#include "tasks/hanging_shooter.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
using namespace hanging_shooting;
const std::string keys =
  "{help h usage ? |                     | 输出命令行参数说明}"
  "{config-path c  | configs/example.yaml | yaml配置文件路径 }"
  "{d display      |                     | 显示视频流       }";

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }
  tools::Exiter exiter;

  auto config_path = cli.get<std::string>("config-path");
  auto display = cli.has("display");
  io::Camera camera(config_path);

  cv::Mat frame;
  std::chrono::steady_clock::time_point timestamp;
  HangingShooter hangingshooter;
  while (!exiter.exit()) {
    camera.read(frame, timestamp);
    std::vector<LightSpot> lightspots = hangingshooter.detect(frame);
    cv::Mat drawlightspot = frame.clone();
    for (const auto & lightspot : lightspots) {
      // cv::circle(drawlightspot, lightspot.center, (int)lightspot.radius, cv::Scalar(0, 0, 255), 2);
      // 方案二：使用OpenCV的函数，根据圆心坐标和半径绘制圆
      // 与实际光圈会存在偏差，但不会有多余的识别点。
      tools::draw_points(drawlightspot, lightspot.contour);
      tools::draw_point(drawlightspot, lightspot.bottom);
      // 方案一：直接画出识别到的图
      // 更换相机后效果还挺好。
      cv::Mat tvec, rvec;
      hangingshooter.solvePnP(lightspot, tvec, rvec);

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
    cv::resize(drawlightspot, drawlightspot, {}, 0.5, 0.5);
    cv::imshow("lightspot", drawlightspot);

    // cv::imshow("camera", frame);
    if (cv::waitKey(1) == 'q') break;
    // test
  }
}
