#include "io/cboard.hpp"

#include <chrono>
#include <thread>

#include "tools/exiter.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

using namespace std::chrono_literals;

int main()
{
  tools::Exiter exiter;

  io::CBoard cboard("can0");

  while (!exiter.exit()) {
    auto timestamp = std::chrono::steady_clock::now();

    std::this_thread::sleep_for(1ms);

    Eigen::Quaterniond q = cboard.imu_at(timestamp);

    Eigen::Vector3d eulers = tools::eulers(q, 2, 1, 0) * 57.3;
    tools::logger()->info("z{:.2f} y{:.2f} x{:.2f} degree", eulers[0], eulers[1], eulers[2]);
    tools::logger()->info("bullet speed {:.2f} m/s", cboard.bullet_speed);
  }

  return 0;
}

// #include "io/cboard.hpp"

// #include <chrono>
// #include <thread>
// #include <cmath>

// #include "tools/exiter.hpp"
// #include "tools/logger.hpp"
// #include "tools/math_tools.hpp"

// using namespace std::chrono_literals;

// void log_and_send(io::CBoard &cboard, double yaw, double pitch) {
//   tools::logger()->info("Testing yaw = {:.2f}, pitch = {:.2f} radians", yaw, pitch);
//   cboard.send({true, true, yaw, pitch});
//   auto timestamp = std::chrono::steady_clock::now();
//   std::this_thread::sleep_for(1ms);
//   Eigen::Quaterniond q = cboard.imu_at(timestamp);
//   Eigen::Vector3d eulers = tools::eulers(q, 2, 1, 0) * 57.3;
//   tools::logger()->info("Command is yaw = {:.2f}, pitch = {:.2f} degrees", yaw * 57.3, pitch * 57.3);
//   tools::logger()->info("Present is yaw = {:.2f}, pitch = {:.2f} degrees", eulers[0], eulers[2]);
// }

// double normalize_yaw(double yaw) {
//   // 将 yaw 归一化到 [-π, π]
//   while (yaw > M_PI) yaw -= 2 * M_PI;
//   while (yaw < -M_PI) yaw += 2 * M_PI;
//   return yaw;
// }

// void yaw_pitch_test(io::CBoard &cboard, tools::Exiter &exiter) {
//   const double step = 0.1; // 步进值
//   const int delay_ms = 10; // 每次测试后等待硬件反应的时间（毫秒）

//   enum class Phase { PITCH, YAW, EIGHT };
//   Phase phase = Phase::YAW;
//   const int max = 20;
//   int count = 0;

//   while (!exiter.exit()) {
//     switch (phase) {
//       case Phase::PITCH: {
//         count = 0;
//         double yaw = 0.0, pitch = 0.0;
//         // 从 0 增加到 0.5π
//         while (pitch <= M_PI / 2 && !exiter.exit()) {
//           log_and_send(cboard, yaw, pitch);
//           if(count<max){
//             count++;
//           }
//           else{
//             pitch += step;
//             count = 0;
//           }
//           std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms));
//         }
//         // 从 0.5π 减小到 -0.5π
//         while (pitch >= -M_PI / 2 && !exiter.exit()) {
//           log_and_send(cboard, yaw, pitch);
//           if(count<max){
//             count++;
//           }
//           else{
//             pitch -= step;
//             count = 0;
//           }
//           std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms));
//         }
//         // 从 -0.5π 增加回 0
//         while (pitch <= 0 && !exiter.exit()) {
//           log_and_send(cboard, yaw, pitch);
//           if(count<max){
//             count++;
//           }
//           else{
//             pitch += step;
//             count = 0;
//           }
//           std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms));
//         }
//         tools::logger()->info("Completed pitch test.");
//         phase = Phase::YAW;
//         break;
//       }

//       case Phase::YAW: {
//         count = 0;
//         double yaw = 0.0, pitch = 0.0;
//         // 从 0 增加到 π，并越过 π
//         while (!exiter.exit()) {
//           log_and_send(cboard, yaw, pitch);
//           if(count<max){
//             count++;
//           }
//           else{
//             yaw += step;
//             count = 0;
//           }
//           yaw = normalize_yaw(yaw); // 始终保持 yaw 在 [-π, π] 范围内
//           if (std::abs(yaw) < step && yaw < 0) {
//             tools::logger()->info("Completed yaw full-circle test.");
//             break;
//           }
//           std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms));
//         }
//         phase = Phase::EIGHT;
//         break;
//       }

//       case Phase::EIGHT: {
//         count = 0;
//         double yaw, pitch;
//         double time = 0.0; // 时间参数
//         const double period = 2.0; // 单周期
//         while (!exiter.exit()) {
//           if(count<max){
//             count++;
//           }
//           else{
//             yaw = std::sin(2 * M_PI * time / period) * (M_PI / 2); // sin 函数生成 8 字形的 X 轴变化
//             pitch = std::sin(4 * M_PI * time / period) * (M_PI / 4); // sin 函数生成 8 字形的 Y 轴变化
//             count = 0;
//           }
//           log_and_send(cboard, yaw, pitch);
//           time += 0.02; // 时间步进
//           std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms));
//         }
//         tools::logger()->info("Completed eight-shape test.");
//         return; // 测试完成
//       }
//     }
//   }
// }

// int main() {
//   tools::Exiter exiter;
//   io::CBoard cboard("can0");

//   tools::logger()->info("----------Start Comprehensive Yaw-Pitch Test!!!----------");

//   yaw_pitch_test(cboard, exiter);

//   tools::logger()->info("----------End Comprehensive Yaw-Pitch Test!!!----------");

//   return 0;
// }
