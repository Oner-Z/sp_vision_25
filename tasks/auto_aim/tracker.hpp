#ifndef AUTO_AIM__TRACKER_HPP
#define AUTO_AIM__TRACKER_HPP

#include <Eigen/Dense>
#include <chrono>
#include <list>
#include <string>

#include "armor.hpp"
#include "solver.hpp"
#include "target.hpp"

namespace auto_aim
{
class Tracker
{
public:
  Tracker(const std::string & config_path, Solver & solver);

  std::vector<Target> track(
    std::list<Armor> & armors, std::chrono::steady_clock::time_point t_img,
    bool use_enemy_color = true);

private:
  Solver & solver_;
  Color enemy_color_;

  std::vector<Target> targets_;
  std::chrono::steady_clock::time_point last_timestamp_;

  bool update_targets(std::list<Armor> & armors, std::chrono::steady_clock::time_point t_img);
};

}  // namespace auto_aim

#endif  // AUTO_AIM__TRACKER_HPP