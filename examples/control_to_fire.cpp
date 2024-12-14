#include <chrono>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <thread>

#include "io/camera.hpp"
#include "io/cboard.hpp"
#include "io/socketcan.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"

using namespace std::chrono_literals;
using namespace std;

auto first_ = true;
auto last_rpm_ = 0.0;
auto drop = false;
std::chrono::steady_clock::time_point drop_time;

void callback(const can_frame & frame)
{
  auto receive_time = std::chrono::steady_clock::now();
  if (frame.can_id != 0x201) return;

  auto rpm = static_cast<double>(frame.data[2] << 8 | frame.data[3]);

  if (first_) {
    last_rpm_ = rpm;
    first_ = false;
    return;
  }

  // drop!
  if (std::abs(last_rpm_ - rpm) > 200.0 && !drop) {
    tools::logger()->warn("DROP!!! {} {}", last_rpm_, rpm);
    drop = true;
    drop_time = receive_time;
  }

  last_rpm_ = rpm;
}

int main()
{
  tools::Exiter exiter;
  tools::Plotter plotter;

  io::SocketCAN can("can0", callback);
  std::this_thread::sleep_for(500ms);
  io::CBoard cboard("can0");
  std::this_thread::sleep_for(500ms);

  Eigen::Quaterniond q = cboard.imu_at(std::chrono::steady_clock::now() - 5ms);
  Eigen::Vector3d eulers = tools::eulers(q, 2, 1, 0);

  auto cmd_yaw = eulers[0];
  auto cmd_pitch = -eulers[1];
  cboard.send({true, false, cmd_yaw, 15 / 57.3});
  std::this_thread::sleep_for(2s);

  auto cmd_time = std::chrono::steady_clock::now();
  cboard.send({true, true, cmd_yaw, 15 / 57.3});
  tools::logger()->warn("FIRE!");

  std::this_thread::sleep_for(3s);

  if (drop) {
    tools::logger()->info("control_to_fire: {:.6f}s", tools::delta_time(drop_time, cmd_time));
  }

  return 0;
}