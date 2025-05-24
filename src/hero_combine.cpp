#include <fmt/core.h>
#include <sched.h>
#include <sys/resource.h>
#include <unistd.h>

#include <chrono>
#include <condition_variable>
#include <mutex>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <optional>
#include <thread>

#include "io/camera.hpp"
#include "io/cboard.hpp"
#include "io/ros2/ros2.hpp"
#include "tasks/auto_aim/classifier.hpp"
#include "tasks/auto_aim/detector.hpp"
#include "tasks/auto_aim/multithread/mt_detector.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/yolov8.hpp"
#include "tasks/auto_base/hanging_shooter.hpp"
#include "tasks/auto_outpost/command_executor.hpp"
#include "tasks/auto_outpost/shooter.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"
#include "tools/recorder.hpp"

using namespace std::chrono;

const std::string keys =
  "{help h usage ? |      | 输出命令行参数说明}"
  "{can c          | can0 | can端口名称     }"
  "{exposure-ms e  |  2.0 | 曝光时间，单位ms  }"
  "{gamma g        |  0.5 | 伽马值，基准为1   }"
  "{debug d        |      | 输出调试画面和信息 }"
  "{@config-path   | configs/hero.yaml | 位置参数，yaml配置文件路径 }";

int main(int argc, char * argv[])
{
  tools::Exiter exiter;
  tools::Plotter plotter;
  tools::Recorder recorder;

  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }
  auto debug = cli.has("debug");
  auto can = cli.get<std::string>("can");
  auto exposure_ms = cli.get<double>("exposure-ms");
  auto gamma = cli.get<double>("gamma");
  auto config_path = cli.get<std::string>(0);

  tools::logger()->info("Initialization Start");
  io::CBoard cboard(can);
  tools::logger()->info("Cboard Start Done");
  io::Camera camera(config_path);
  tools::logger()->info("Camera Start Done");
  io::ROS2 ros2;
  tools::logger()->info("ROS2 Start Done");
  // auto_aim::Classifier classifier(config_path);
  // auto_aim::Detector detector(config_path, debug);
  auto_aim::YOLOV8 detector(config_path, debug);
  // auto_aim::multithread::MultiThreadDetector detector(config_path, debug);
  tools::logger()->info("Detector Start Done");
  auto_aim::Solver solver(config_path);
  tools::logger()->info("Solver Start Done");
  auto_aim::Tracker tracker(config_path, solver);
  tools::logger()->info("Tracker Start Done");
  auto_outpost::Shooter shooter(config_path);
  tools::logger()->info("Shooter Start Done");
  auto_base::HangingShooter hangingshooter(config_path);
  tools::logger()->info("HangingShooter Start Done");

  // 独立决策线程
  auto_outpost::CommandExecutor executor(shooter, cboard);
  executor.start();
  tools::logger()->info("Seperate Command Thread Done");

  Eigen::Quaterniond q;

  cv::Mat img;
  std::chrono::steady_clock::time_point t;

  auto mode = io::Mode::idle;
  auto last_mode = io::Mode::idle;

  auto last_fps_time = std::chrono::steady_clock::now();
  int frame_counter = 0;
  double fps = 0.0;

  nlohmann::json data;

  tools::logger()->info("AUTO AIM START");
  for (int frame_count = 0; !exiter.exit(); frame_count++) {
    camera.read(img, t);

    q = cboard.imu_at(t - 1ms);
    if (!img.empty()) recorder.record(img, q, t);

    mode = cboard.mode;  // TODO
    if (last_mode != mode) tools::logger()->info("Switch to {}", io::MODES[mode]);
    last_mode = mode;
    if (io::MODES[mode] == "hanging_shooting") {
      io::Command command = hangingshooter.aim(q, ros2.subscribe(), cboard.bullet_speed);
      cboard.send(command);
    } else {
      solver.set_R_gimbal2world(q);
      auto armors = detector.detect(img);
      Eigen::Vector3d ypr = tools::eulers(solver.R_gimbal2world(), 2, 1, 0);
      auto targets = tracker.track(armors, t, ypr[0], true, mode);
      executor.push(targets, t, cboard.bullet_speed, ypr);
    }
  }

  executor.stop_thread();
  return 0;
}