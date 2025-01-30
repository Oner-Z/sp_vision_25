#include <fmt/core.h>
#include <yaml-cpp/yaml.h>

#include <chrono>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>

#include "io/camera.hpp"
#include "io/cboard.hpp"
#include "tasks/hanging_shooter.hpp"
#include "tasks/pipe.hpp"
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
  h_solver solver;
  HangingShooter hangingshooter;
  NamedPipe pipe("/tmp/client_to_server", true);
  pipe.open();
  auto yaml = YAML::LoadFile(config_path);
  auto bullet_speed = 15.8;
  auto target_x = 19.309643;
  auto target_y = 5.228956;
  auto target_z = 0.42;
  auto shouyan_y = 0.15;
  auto shouyan_z = 0.2;
  auto C = 0.275;
  auto rho = 1.169;

  io::CBoard cboard(config_path);
  io::Camera camera(config_path);
  cv::Mat img;
  Eigen::Quaterniond q;
  std::chrono::steady_clock::time_point t;
  DataPacket current_position;
  pipe.open();
  while (!exiter.exit()) {
    camera.read(img, t);
    q = cboard.imu_at(t);
    Eigen::Vector3d eulers = tools::eulers(q, 2, 1, 0);
    double c_yaw = eulers[0];
    double c_pitch = eulers[2];
    if (pipe.read(&current_position, sizeof(current_position)))  //读取到数据
    {
      Eigen::Quaterniond turn_q;
      turn_q.x() = current_position.rotation_x;
      turn_q.y() = current_position.rotation_y;
      turn_q.z() = current_position.rotation_z;
      turn_q.w() = current_position.rotation_w;
      Eigen::Vector3d turn_eulers = tools::eulers(turn_q, 2, 1, 0);

      // 获取雷达坐标系下向目标位置的旋转角
      current_position.y -= shouyan_y;  // 雷达到发射中心偏移量
      auto t_distance = sqrt(
        (current_position.x - target_x) * (current_position.x - target_x) +
        (current_position.y - target_y) * (current_position.y - target_y));
      auto t_height = current_position.z - target_z + shouyan_z;

      double pitch = 0;
      solver.calculate_pitch(bullet_speed, t_distance, t_height, C, rho, pitch);
      // 计算发射角度

      double yaw_offset = atan2(t_distance, shouyan_y);

      double t_yaw = turn_eulers[0] - yaw_offset;
      double t_pitch = -(pitch);  // 此值正负与电控有关
      io::Command command{1, 0, t_yaw, t_pitch};
      cboard.send(command);
      tools::logger()->info("c_pitch:{.2f} c_yaw:{.2f}", c_pitch, c_yaw);
    }
  }

  return 0;
}