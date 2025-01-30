#include <fmt/core.h>
#include <yaml-cpp/yaml.h>

#include <chrono>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>

#include "io/camera.hpp"
#include "io/cboard.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"
#include "tools/recorder.hpp"

using namespace std::chrono;
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
  io::CBoard cboard(config_path);
  io::Camera camera(config_path);
  Eigen::Quaterniond q;
  std::chrono::steady_clock::time_point t;
  nlohmann::json data;

  auto yaml = YAML::LoadFile(config_path);
  auto t_distance = yaml["t_distance"].as<double>();
  auto t_height = yaml["t_height"].as<double>();
  auto bullet_speed = yaml["bullet_speed"].as<double>();
  auto C = yaml["C"].as<double>();
  auto rho = yaml["rho"].as<double>();
  auto angle = yaml["angle"].as<double>();
  double pitch = angle / 180 * 3.1415926;
  cv::Mat img;
  while (!exiter.exit()) {
    camera.read(img, t);
    q = cboard.imu_at(t);
    Eigen::Vector3d eulers = tools::eulers(q, 2, 1, 0);
    double c_yaw = eulers[0];
    double c_pitch = eulers[1];
    double t_yaw = c_yaw;
    double t_pitch = pitch;  //
    io::Command command{1, 0, t_yaw, t_pitch};
    tools::logger()->info("pitch {:.2f} ", pitch);
    tools::logger()->info("c_pitch:{:.2f} c_yaw:{:.2f}", c_pitch, c_yaw);
    tools::logger()->info("t_pitch:{:.2f} t_yaw:{:.2f}", t_pitch, t_yaw);
    tools::logger()->info("bullet_speed:{:.2f}", cboard.bullet_speed);
    cboard.send(command);
    cv::imshow("camera", img);
  }
  return 0;
}