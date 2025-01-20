#ifndef AUTO_AIM__AUTO_SHOOT_AIMER_HPP
#define AUTO_AIM__AUTO_SHOOT_AIMER_HPP

#include <Eigen/Dense>
#include <chrono>
#include <list>
#include <optional>

#include "io/command.hpp"
#include "target.hpp"

namespace auto_aim
{
class Aimer
{
public:
  // std::optional<AimPoint> debug_aim_point;
  explicit Aimer(const std::string & config_path);
  io::Command aim(
    const std::list<Target> & targets, std::list<Armor> & armors,
    std::chrono::steady_clock::time_point timestamp, double bullet_speed, bool to_now = true);
  void clear_last();

  std::optional<std::pair<double, double>> yp_should;

  int aim_id = -1;
  double yaw_offset_;
  double pitch_offset_;

private:
  double coming_angle_;
  double leaving_angle_;
  double delay_gimbal_;
  double delay_shoot_;
  int lock_id_ = -1;
  /// TODO: aim at next coming armor
  ArmorName last_target_name_ = not_armor;

  std::optional<Target> choose_target(const std::list<Target> & targets, std::list<Armor> & armors);
  std::optional<int> choose_armor(const Target & target);
  // AimPoint choose_coming_aim_point(const Target & target);
};

}  // namespace auto_aim

#endif  // AUTO_AIM__AIMER_HPP