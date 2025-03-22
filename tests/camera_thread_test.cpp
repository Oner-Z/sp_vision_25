#include <fmt/core.h>
#include <unistd.h>

#include <chrono>
#include <future>
#include <map>
#include <mutex>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <thread>

#include "io/camera.hpp"
#include "io/cboard.hpp"
#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/yolo11.hpp"
#include "tasks/auto_aim/yolov8.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"
#include "tools/recorder.hpp"
#include "tools/thread_pool.hpp"

const std::string keys =
  "{help h usage ? |                        | 输出命令行参数说明}"
  "{@config-path   | configs/ascento.yaml | 位置参数，yaml配置文件路径 }";

tools::ThreadSafeQueue<std::future<tools::Frame>> results(1000);

// 处理detect任务的线程函数
tools::Frame detect_frame(tools::Frame && frame, auto_aim::YOLOV8 & yolo)
{
  frame.armors = yolo.detect(frame.img).get();
  return frame;
}

int main(int argc, char * argv[])
{
  tools::Exiter exiter;
  tools::Plotter plotter;
  // tools::Recorder recorder(100);

  cv::CommandLineParser cli(argc, argv, keys);
  auto config_path = cli.get<std::string>(0);
  if (cli.has("help") || config_path.empty()) {
    cli.printMessage();
    return 0;
  }

  // 处理线程函数
  auto process_thread = std::thread([&]() {
    std::future<tools::Frame> out;
    while (!exiter.exit()) {
      results.pop_by_move(out);
      auto process_frame = out.get();
      auto img = process_frame.img;
      auto armors = process_frame.armors;
      auto t = process_frame.t;

      nlohmann::json data;
      data["armor_num"] = armors.size();

      plotter.plot(data);
      // cv::resize(img, img, {}, 0.5, 0.5);
      // cv::imshow("reprojection", img);
    }
  });

  auto_aim::YOLOV8 yolov8(config_path, true);
  io::Camera camera(config_path);

  cv::Mat img;
  Eigen::Quaterniond q;
  std::chrono::steady_clock::time_point t;
  std::chrono::steady_clock::time_point last_t = std::chrono::steady_clock::now();

  int frame_id = 0;

  while (!exiter.exit()) {
    camera.read(img, t);
    auto dt = tools::delta_time(t, last_t);
    last_t = t;

    // tools::logger()->info("{:.2f} fps", 1 / dt);
    // tools::draw_text(img, fmt::format("{:.2f} fps", 1/dt), {10, 60}, {255, 255, 255});
    nlohmann::json data;
    data["fps"] = 1 / dt;

    frame_id++;
    tools::Frame frame{frame_id, img.clone(), t};
    auto result = std::async(std::launch::async, detect_frame, std::move(frame), std::ref(yolov8));
    results.push_by_move(std::move(result));
    plotter.plot(data);

    auto key = cv::waitKey(1);
    if (key == 'q') break;
  }

  return 0;
}
