#include <fmt/core.h>

#include <chrono>
#include <condition_variable>
#include <mutex>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <optional>
#include <thread>

#include "io/camera.hpp"
#include "io/cboard.hpp"
#include "tasks/auto_aim/classifier.hpp"
#include "tasks/auto_aim/detector.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/yolov8.hpp"
#include "tasks/auto_outpost/shooter.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"
#include "tools/recorder.hpp"
using namespace std::chrono;

class CommandExecutor
{
public:
  CommandExecutor(auto_outpost::Shooter & shooter_ref, io::CBoard & cboard_ref)
  : shooter_(shooter_ref), cboard_(cboard_ref), stop_(false)
  {
  }

  void start() { thread_ = std::thread(&CommandExecutor::run, this); }

  void stop_thread()
  {
    {
      std::lock_guard<std::mutex> lock(mtx_);
      stop_ = true;
    }
    cv_.notify_all();
    if (thread_.joinable()) thread_.join();
  }

  void push(
    const std::list<auto_aim::Target> & targets, const std::chrono::steady_clock::time_point & t, double bullet_speed,
    const Eigen::Vector3d & ypr)
  {
    std::lock_guard<std::mutex> lock(mtx_);
    latest_ = {std::list<auto_aim::Target>(targets), t, bullet_speed, ypr};
    cv_.notify_one();
  }

private:
  struct Input
  {
    std::list<auto_aim::Target> targets;
    std::chrono::steady_clock::time_point t;

    double bullet_speed;
    Eigen::Vector3d ypr;
  };

  auto_outpost::Shooter & shooter_;
  io::CBoard & cboard_;
  std::optional<Input> latest_;
  std::mutex mtx_;
  std::condition_variable cv_;
  std::thread thread_;
  bool stop_;

  void run()
  {
    while (!stop_) {
      std::optional<Input> input;
      {
        std::lock_guard<std::mutex> lock(mtx_);
        if (latest_) {
          input = latest_;
        }
      }
      if (input) {
        // static auto last_time = std::chrono::steady_clock::now();
        // auto now = std::chrono::steady_clock::now();
        // double interval = std::chrono::duration<double>(now - last_time).count();
        // last_time = now;
        // std::cout << "[决策线程] 频率 = " << 1.0 / interval << " Hz" << std::endl;
        auto command = shooter_.shoot(input->targets, input->t, input->bullet_speed, true, input->ypr);
        cboard_.send(command);
        tools::Plotter plotter;
        nlohmann::json data;
        data["command_yaw"] = command.yaw * 57.3;
        data["command_pitch"] = -command.pitch * 57.3;
        plotter.plot(data);
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }
};

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

  io::CBoard cboard(can);
  io::Camera camera(config_path);

  auto_aim::Classifier classifier(config_path);
  // auto_aim::Detector detector(config_path, debug);
  auto_aim::YOLOV8 detector(config_path, debug);
  auto_aim::Solver solver(config_path);
  auto_aim::Tracker tracker(config_path, solver);
  auto_outpost::Shooter shooter(config_path);

  // 独立决策线程
  CommandExecutor executor(shooter, cboard);
  executor.start();

  cv::Mat img;
  Eigen::Quaterniond q;
  std::chrono::steady_clock::time_point t;

  auto mode = io::Mode::idle;
  auto last_mode = io::Mode::idle;

  for (int frame_count = 0; !exiter.exit(); frame_count++) {
    camera.read(img, t);
    q = cboard.imu_at(t - 1ms);
    recorder.record(img, q, t);

    mode = cboard.mode;  // TODO
    if (last_mode != mode) tools::logger()->info("Switch to {}", io::MODES[mode]);
    last_mode = mode;

    solver.set_R_gimbal2world(q);
    auto armors = detector.detect(img);
    Eigen::Vector3d ypr = tools::eulers(solver.R_gimbal2world(), 2, 1, 0);
    auto targets = tracker.track(armors, t, ypr[0], true, mode);

    // auto command = shooter.shoot(targets, t, cboard.bullet_speed, true, ypr);
    // cboard.send(command);

    executor.push(targets, t, cboard.bullet_speed, ypr);

    if (!debug) continue;

    nlohmann::json data;
    tools::draw_text(img, fmt::format("[{}] [{}]", frame_count, tracker.state_str()), {10, 30}, {255, 255, 255});
    int armor_id = 0;
    if (tracker.state()) {  // tracker.state() && targets.size()
      // 当前帧target更新后
      auto target = tracker.get_target();
      std::vector<Eigen::Vector4d> armor_xyza_list = target.armor_xyza_list();
      for (const Eigen::Vector4d & xyza : armor_xyza_list) {
        auto image_points = solver.reproject_armor(xyza.head(3), xyza[3], target.armor_type, target.name);
        tools::draw_points(img, image_points, {0, 255, 0});
        tools::draw_text(img, fmt::format("No: {}", armor_id), image_points[0], {0, 255, 255});
        armor_id++;
      }

      // aimer瞄准位置
      auto aim_point = shooter.debug_aim_point_;
      Eigen::Vector4d aim_xyza = aim_point.xyza;
      auto image_points = solver.reproject_armor(aim_xyza.head(3), aim_xyza[3], target.armor_type, target.name);
      if (aim_point.valid)
        tools::draw_points(img, image_points, {0, 0, 255});
      else
        tools::draw_points(img, image_points, {255, 0, 0});

      // 观测器内部数据
      Eigen::VectorXd x = target.ekf_x();
      // x vx y vy z a w r1 r2 h
      // a: angle
      // w: angular velocity
      data["x"] = x[0];
      data["vx"] = x[1];
      data["y"] = x[2];
      data["vy"] = x[3];
      data["z"] = x[4];
      data["vz"] = x[5];
      data["a"] = x[6] * 57.3;
      data["v"] = sqrt(x[1] * x[1] + x[3] * x[3]);
      data["w"] = x[7];
      data["r"] = x[8];
      data["l"] = x[9];
      data["h"] = x[10];
      data["last_id"] = target.last_id;
      data["bullet_speed"] = cboard.bullet_speed;
      data["d"] = sqrt(x[0] * x[0] + x[2] * x[2] + x[4] * x[4]);
      // data["command_yaw"] = command.yaw * 57.3;
      // data["command_pitch"] = -command.pitch * 57.3;
    }
    cv::resize(img, img, {}, 0.5, 0.5);  // 显示时缩小图片尺寸
    cv::imshow("reprojection", img);

    // 云台响应情况
    // Eigen::Vector3d ypr = tools::eulers(solver.R_gimbal2world(), 2, 1, 0);

    data["gimbal_yaw"] = ypr[0] * 57.3;
    data["gimbal_pitch"] = -ypr[1] * 57.3;

    plotter.plot(data);

    auto key = cv::waitKey(10);
    if (key == 'q') break;
  }

  executor.stop_thread();
  return 0;
}