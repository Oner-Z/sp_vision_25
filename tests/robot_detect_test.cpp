#include <fmt/core.h>

#include <chrono>
#include <opencv2/opencv.hpp>

#include "tasks/mono_loc/robot_detector.hpp"
#include "tools/exiter.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

const std::string keys =
  "{help h usage ? |                        | 输出命令行参数说明 }"
  "{start-index s  | 0                      | 视频起始帧下标    }"
  "{end-index e    | 0                      | 视频结束帧下标    }"
  "{@video_path    |                        | avi路径}"
  "{config-path c  | configs/newsentry.yaml | yaml配置文件的路径}";

int main(int argc, char * argv[])
{
  // 读取命令行参数
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }
  auto video_path = cli.get<std::string>(0);
  auto config_path = cli.get<std::string>("config-path");
  auto start_index = cli.get<int>("start-index");
  auto end_index = cli.get<int>("end-index");
  tools::logger()->debug("video_path: {}", video_path);
  tools::Exiter exiter;

  cv::VideoCapture video(video_path);

  mono_loc::RobotDetector detector(config_path);

  std::chrono::steady_clock::time_point timestamp;
  tools::logger()->debug("here");

  for (int frame_count = start_index; !exiter.exit(); frame_count++) {
    if (end_index > 0 && frame_count > end_index) break;

    cv::Mat img;
    video.read(img);
    if (img.empty()) break;

    auto l = detector.detect(img, frame_count);
    for (const auto & e : l) {
      cv::circle(img, e, 5, {0, 255, 0}, 2);
    }
    cv::imshow("aaaaa", img);
    auto key = cv::waitKey(0);
    if (key == 'q') break;
  }

  return 0;
}