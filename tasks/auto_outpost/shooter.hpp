#ifndef AUTO_AIM__HERO_SHOOTER_HPP
#define AUTO_AIM__HERO_SHOOTER_HPP

#include <chrono>
#include <string>
#include <vector>

#include "io/cboard.hpp"
#include "tasks/auto_aim/target.hpp"
#include "tools/thread_safe_queue.hpp"

namespace auto_outpost
{
// {valid,y,p,id,shoot_time}
struct shooter_info
{
  bool valid;
  double yaw;
  double pitch;
  int aim_id;
  std::chrono::steady_clock::time_point shoot_time;
  shooter_info()
  : valid(false), yaw(0.0), pitch(0.0), aim_id(0), shoot_time(std::chrono::steady_clock::now())
  {
  }
  shooter_info(
    bool _valid, double _yaw, double _pitch, int _aim_id,
    std::chrono::steady_clock::time_point _shoot_time = std::chrono::steady_clock::time_point{})
  : valid(_valid), yaw(_yaw), pitch(_pitch), aim_id(_aim_id), shoot_time(_shoot_time)
  {
  }
};
// {valid，xyza}
struct AimPoint
{
  bool valid;
  Eigen::Vector4d xyza;
};

class Shooter
{
public:
  explicit Shooter(const std::string & config_path, io::CBoard & cboard);
  void schedule(auto_aim::Target target_at_t0);
  int get_next_armor(const auto_aim::Target & target,double flytime, std::chrono::steady_clock::time_point timestamp);
  void shoot(
    std::list<auto_aim::Target> targets, std::chrono::steady_clock::time_point timestamp,
    double bullet_speed, bool to_now);
  void test(
    std::list<auto_aim::Target> targets, std::chrono::steady_clock::time_point timestamp,
    double bullet_speed, bool to_now, double value);
  Eigen::Vector3d get_front(const auto_aim::Target & target);

  AimPoint choose_aim_point(const auto_aim::Target & target);

  ~Shooter();
  AimPoint debug_aim_point_;

private:
  double ctrl_to_fire_;
  double yaw_offset_;
  double pitch_offset_;
  int last_hit_id_ = -1;
  int lock_id_ = -1;
  bool exit_;
  io::CBoard & cboard_;
  std::thread when_to_fire_;
  tools::ThreadSafeQueue<shooter_info> queue_0_, queue_1_, queue_2_;
  int choose_armor(
    std::vector<std::chrono::steady_clock::time_point> send_times,
    std::chrono::steady_clock::time_point now)
  {
    auto erliest_send_time = now + 10s;
    int id = 0;
    for (int i = 0; i < send_times.size(); i++) {
      if (send_times[i] > now - 0.03s && send_times[i] < erliest_send_time) {
        erliest_send_time = send_times[i];
        id = i;
      }
    }
    return id;
  }
};
}  // namespace auto_outpost
#endif