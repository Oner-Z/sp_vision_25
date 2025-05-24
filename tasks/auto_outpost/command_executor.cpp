#include "command_executor.hpp"
#include "tools/math_tools.hpp"
namespace auto_outpost
{

void CommandExecutor::run()
{
  while (!stop_) {
    std::optional<Input> input;
    {
      std::lock_guard<std::mutex> lock(mtx_);
      if (latest_) {
        input = latest_;
      }
    }
    if (input && (tools::delta_time(std::chrono::steady_clock::now(), input->t) <= 0.2)) {
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
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
}

void CommandExecutor::push(
  const std::list<auto_aim::Target> & targets, const std::chrono::steady_clock::time_point & t, double bullet_speed,
  const Eigen::Vector3d & ypr)
{
  std::lock_guard<std::mutex> lock(mtx_);
  latest_ = {std::list<auto_aim::Target>(targets), t, bullet_speed, ypr};
  cv_.notify_one();
}

void CommandExecutor::start() { thread_ = std::thread(&CommandExecutor::run, this); }

void CommandExecutor::stop_thread()
{
  {
    std::lock_guard<std::mutex> lock(mtx_);
    stop_ = true;
  }
  cv_.notify_all();
  if (thread_.joinable()) thread_.join();
}

}  // namespace auto_outpost
