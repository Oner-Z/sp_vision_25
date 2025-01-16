#ifndef IO__CBOARD_HPP
#define IO__CBOARD_HPP

#include <Eigen/Geometry>
#include <chrono>
#include <cmath>
#include <functional>
#include <string>
#include <vector>

#include "io/command.hpp"
#include "io/socketcan.hpp"
#include "tools/logger.hpp"
#include "tools/thread_safe_queue.hpp"

namespace io
{
enum Mode
{
  idle,
  auto_aim,
  small_buff,
  big_buff,
  outpost
};
const std::vector<std::string> MODES = {"idle", "auto_aim", "small_buff", "big_buff", "outpost"};

class CBoard
{
public:
  /// read-only
  double bullet_speed;
  Mode mode;

  Eigen::Quaterniond imu_at(std::chrono::steady_clock::time_point timestamp);
  Eigen::Quaterniond latest();

  void send(Command command) const;

  // 获取唯一实例，仅以首次调用时的path进行初始化
  static CBoard & getInstance(const std::string & path = "")
  {
    static CBoard instance(path);
    return instance;
  }

private:
  // std::string config_path_;

  // 禁止拷贝构造和赋值操作
  CBoard(const CBoard &) = delete;
  CBoard & operator=(const CBoard &) = delete;

  CBoard(const std::string & config_path);
  struct IMUData
  {
    Eigen::Quaterniond q;
    std::chrono::steady_clock::time_point timestamp;
  };

  tools::ThreadSafeQueue<IMUData> queue_;  // 必须在can_之前初始化，否则存在死锁的可能

  Eigen::Quaterniond latest_;
  std::mutex latest_lock_;

  SocketCAN can_;
  IMUData data_ahead_;
  IMUData data_behind_;

  int quaternion_canid_, bullet_speed_canid_, send_canid_;

  void callback(const can_frame & frame);

  std::string read_yaml(const std::string & config_path);
};

}  // namespace io

#endif  // IO__CBOARD_HPP