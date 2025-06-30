#include <fmt/format.h>

#include <chrono>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>

#include "io/camera.hpp"
#include "io/cboard.hpp"
#include "tasks/auto_buff/buff_aimer.hpp"
#include "tasks/auto_buff/buff_detector.hpp"
#include "tasks/auto_buff/buff_solver.hpp"
#include "tasks/auto_buff/buff_target.hpp"
#include "tasks/auto_buff/buff_type.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"
#include "tools/recorder.hpp"
#include "tools/trajectory.hpp"

// 定义命令行参数
const std::string keys =
  "{help h usage ? |                        | 输出命令行参数说明 }"
  "{config-path c  | configs/standard4.yaml | yaml配置文件的路径}"
  "{start-index s  | 0                      | 视频起始帧下标    }"
  "{end-index e    | 0                      | 视频结束帧下标    }"
  "{show-iou i     | 1                      | 显示predict位置计算iou}"
  "{@input-path    |                        | avi和txt文件的路径}";

int main(int argc, char * argv[])
{
  // 读取命令行参数
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }
  auto input_path = cli.get<std::string>(0);
  auto config_path = cli.get<std::string>("config-path");
  auto start_index = cli.get<int>("start-index");
  auto end_index = cli.get<int>("end-index");
  auto show_iou = cli.get<bool>("show-iou");

  // 初始化绘图器和退出器
  tools::Plotter plotter;
  tools::Exiter exiter;

  // 构造视频和文本文件路径
  auto video_path = fmt::format("{}.avi", input_path);
  cv::VideoCapture video(video_path);

  // 初始化跟踪器、解算器、追踪器和瞄准器
  auto_buff::Buff_Detector detector(config_path);

  cv::Mat img;
  auto t0 = std::chrono::steady_clock::now();

  // 设置视频起始帧
  video.set(cv::CAP_PROP_POS_FRAMES, start_index);

  // 循环处理视频帧
  for (int frame_count = start_index; !exiter.exit(); frame_count++) {
    if (end_index > 0 && frame_count > end_index) break;

    video.read(img);
    if (img.empty()) break;

    // -------------- 打符核心逻辑 --------------

    // 检测buff
    auto detector_start = std::chrono::steady_clock::now();
    auto power_runes = detector.detect(img);

    // -------------- 调试输出 --------------

    auto finish = std::chrono::steady_clock::now();
    tools::logger()->info(
      "[{}] detector: {:.1f}ms", frame_count, tools::delta_time(finish, detector_start) * 1e3);

    // nlohmann::json data;

    // // buff原始观测数据
    // if (power_runes.has_value()) {
    //   const auto p = power_runes.value();
    // }
    // std::vector<cv::Point2f> image_points;
    // if (!target.is_unsolve()) {
    //   auto power_rune = power_runes.value();

    //   // 当前帧target更新后buff
    //   auto Rxyz_in_world_now = target.point_buff2world(Eigen::Vector3d(0.0, 0.0, 0.0));
    //   image_points = solver.reproject_buff(Rxyz_in_world_now, target.ekf_x()[4], target.ekf_x()[5]);
    //   tools::draw_points(
    //     img, std::vector<cv::Point2f>(image_points.begin(), image_points.begin() + 4), {0, 255, 0});
    //   tools::draw_points(
    //     img, std::vector<cv::Point2f>(image_points.begin() + 4, image_points.end()), {0, 255, 0});

    // }

    // cv::resize(img, img, cv::Size(img.size().width * 0.7, img.size().height * 0.7));
    // cv::imshow("result", img);

    int key = cv::waitKey(1);
    if (key == 'q') break;
    while (key == ' ') {
      int y = cv::waitKey(30);
      if (y == 'q') break;
    }
  }
  cv::destroyAllWindows();
  return 0;
}
