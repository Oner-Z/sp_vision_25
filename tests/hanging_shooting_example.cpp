#include <fmt/core.h>
#include <yaml-cpp/yaml.h>

#include <chrono>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>

#include "io/camera.hpp"
#include "io/cboard.hpp"
#include "tasks/hanging_shooter.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"
#include "tools/recorder.hpp"

using namespace std::chrono;
using namespace hanging_shooting;
const std::string keys =
  "{help h usage ? |      | 输出命令行参数说明}"
  "{@config-path   | configs/example.yaml | 位置参数，yaml配置文件路径 }";

int main(int argc, char * argv[])  // 测试已对准目标情况下吊射效果，example.yaml文件中修改目标位置
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
  // io::CBoard cboard(config_path);
  // io::Camera camera(config_path);
  Eigen::Quaterniond q;
  std::chrono::steady_clock::time_point t;
  h_solver solver;
  HangingShooter hangingshooter;
  auto yaml = YAML::LoadFile(config_path);
  auto t_distance = yaml["t_distance"].as<double>();
  auto t_height = yaml["t_height"].as<double>();
  auto bullet_speed = yaml["bullet_speed"].as<double>();
  double pitch = 0;
  solver.calculate_pitch(bullet_speed, t_distance, t_height, pitch);
  std::cout << pitch << std::endl;
  // while (!exiter.exit()) {
  //   q = cboard.imu_at(t - 1ms);
  //   double c_yaw = std::atan2(
  //     2.0f * (q.w() * q.z() + q.x() * q.y()), 1.0f - 2.0f * (q.y() * q.y() + q.z() * q.z()));
  //   double c_pitch = std::asin(2.0f * (q.w() * q.y() - q.x() * q.z()));
  //   double t_yaw = c_yaw;
  //   double t_pitch = -(c_pitch + pitch);
  //   io::Command command{1, 0, t_yaw, t_pitch};
  //   tools::logger()->info("c_pitch:{.2f} c_yaw:{.2f}", c_pitch, c_yaw);
  // }

  return 0;
}