#ifndef AUTO_OUTPOST__HERO_COMMAND_EXECUTOR_HPP
#define AUTO_OUTPOST__HERO_COMMAND_EXECUTOR_HPP

#include <chrono>
#include <string>
#include <vector>
#include <optional>
#include <thread>
#include "io/cboard.hpp"
#include "tasks/auto_aim/target.hpp"
#include "tools/thread_safe_queue.hpp"
#include "tools/plotter.hpp"
#include "shooter.hpp"
namespace auto_outpost
{

class CommandExecutor
{
public:
  CommandExecutor(auto_outpost::Shooter & shooter_ref, io::CBoard & cboard_ref)
  : shooter_(shooter_ref), cboard_(cboard_ref), stop_(false)
  {
  }

  void start();
  void stop_thread();
  void push(
    const std::list<auto_aim::Target> & targets, const std::chrono::steady_clock::time_point & t, double bullet_speed,
    const Eigen::Vector3d & ypr);

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

  void run();
};
}  // namespace auto_outpost
#endif