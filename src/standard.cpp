#include <fmt/core.h>

#include <chrono>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>

#include "io/camera.hpp"
#include "io/cboard.hpp"
#include "tasks/auto_aim/auto_shoot_aimer.hpp"
#include "tasks/auto_aim/detector.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"
#include "tools/recorder.hpp"

using namespace std::chrono;

const std::string keys =
  "{help h usage ? |      | 输出命令行参数说明}"
  "{@config-path   | configs/standard3.yaml | 位置参数，yaml配置文件路径 }";

int main(int argc, char * argv[])
{
  tools::Exiter exiter;
  tools::Plotter plotter;
  tools::Recorder recorder;

  cv::CommandLineParser cli(argc, argv, keys);
  auto config_path = cli.get<std::string>(0);
  if (cli.has("help") || config_path.empty()) {
    cli.printMessage();
    return 0;
  }

  io::CBoard cboard(config_path);
  io::Camera camera(config_path);

  auto_aim::Detector detector(config_path, false);
  auto_aim::Solver solver(config_path);
  auto_aim::Tracker tracker(config_path, solver);
  auto_aim::Aimer aimer(config_path);

  cv::Mat img;
  Eigen::Quaterniond q;
  std::chrono::steady_clock::time_point t;

  while (!exiter.exit()) {
    camera.read(img, t);
    q = cboard.imu_at(t - 1ms);
    // recorder.record(img, q, t);

    solver.set_R_gimbal2world(q);

    auto real_yaw = tools::eulers(q, 2, 1, 0)[0];

    auto armors = detector.detect(img);
    // std::cout<<armors.size()<<std::endl;

    auto targets = tracker.track(armors, t);
    // std::cout<<targets.size()<<std::endl;

    auto command = aimer.aim(targets, t, cboard.bullet_speed);

    if (command.control && (std::abs(command.yaw - real_yaw) < 2.4 / 57.3) && command.shoot) {
      // command.shoot = true;
      tools::logger()->debug("####### shoot #######");
    } else {
      command.shoot = false;
    }
    tools::logger()->debug("{}", command.yaw);
    cboard.send(command);
    // cv::waitKey(30);

    nlohmann::json data;
    if (!targets.empty()) {
      auto target = targets.front();
      // 观测器内部数据
      Eigen::VectorXd x = target.ekf_x();
      data["x"] = x[0];
      data["vx"] = x[1];
      data["y"] = x[2];
      data["vy"] = x[3];
      data["z"] = x[4];
      data["vz"] = x[5];
      data["a"] = x[6] * 57.3;
      data["w"] = x[7];
      data["r"] = x[8];
      data["l"] = x[9];
      data["h"] = x[10];
      data["last_id"] = target.last_id;
      plotter.plot(data);
    }
  }

  return 0;
}