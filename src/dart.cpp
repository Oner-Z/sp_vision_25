#include <fmt/core.h>

#include <chrono>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>

#include "io/camera.hpp"
#include "io/cboard.hpp"
#include "tools/dart_recorder.hpp"
#include "tools/exiter.hpp"
#include "tools/logger.hpp"

using namespace std::chrono;

const std::string keys =
  "{help h usage ? |      | 输出命令行参数说明}"
  "{@config-path   | configs/dart.yaml | 位置参数，yaml配置文件路径 }";

int main(int argc, char * argv[])
{
  tools::Exiter exiter;
  tools::DartRecorder recorder;

  cv::CommandLineParser cli(argc, argv, keys);
  auto config_path = cli.get<std::string>(0);
  if (cli.has("help") || config_path.empty()) {
    cli.printMessage();
    return 0;
  }

  io::CBoard cboard(config_path);
  io::Camera camera(config_path);

  cv::Mat img;
  std::chrono::steady_clock::time_point t;

  auto mode = io::Mode::idle;
  auto last_mode = io::Mode::idle;

  while (!exiter.exit()) {
    camera.read(img, t);
    if (cboard.control) {
      recorder.record(img, t);
    }
    // if (mode == io::Mode::idle) aimer.clear_last();

    recorder.record(img, t);
  }
  return 0;
}