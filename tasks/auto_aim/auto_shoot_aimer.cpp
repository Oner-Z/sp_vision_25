#include "auto_shoot_aimer.hpp"

#include <yaml-cpp/yaml.h>

#include <cmath>
#include <vector>

#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/trajectory.hpp"

namespace auto_aim
{
Aimer::Aimer(const std::string & config_path)
{
  auto yaml = YAML::LoadFile(config_path);
  yaw_offset_ = yaml["yaw_offset"].as<double>() / 57.3;          // degree to rad
  pitch_offset_ = yaml["pitch_offset"].as<double>() / 57.3;      // degree to rad
  coming_angle_ = yaml["coming_angle"].as<double>() / 57.3;  // degree to rad
  leaving_angle_ = yaml["leaving_angle"].as<double>() / 57.3;
  delay_gimbal_ = yaml["delay_gimbal"].as<double>();
  delay_shoot_ = yaml["delay_shoot"].as<double>();
}

std::optional<Target> Aimer::choose_target(
  const std::list<Target> & targets, std::list<Armor> & armors)
{
  if (targets.empty()) return std::nullopt;

  Target chosen_target = targets.front();
  bool choose_last = false, choose_near = false;

  // 优先击打刚打过的目标
  if (last_target_name_ != not_armor) {
    for (auto & target : targets) {
      if (target.name == last_target_name_) {
        chosen_target = target;
        choose_last = true;
        last_target_name_ = target.name;
        break;
      }
    }
  }

  // 若没有刚击打过的记录，或者上次击打的目标不在tracking状态，则选取最靠近中心的
  if (!choose_last) {
    armors.sort([](const Armor & a, const Armor & b) {
      auto img_center_norm = cv::Point2f(0.5, 0.5);
      auto distance_1 = cv::norm(a.center_norm - img_center_norm);
      auto distance_2 = cv::norm(b.center_norm - img_center_norm);
      return distance_1 < distance_2;
    });

    for (auto & armor : armors) {
      for (auto & target : targets) {
        if (target.name == armor.name) {
          chosen_target = target;
          choose_near = true;
          last_target_name_ = target.name;
          break;
        }
      }
      if (choose_near) break;
    }
  }

  if (!choose_last && !choose_near) {
    // tools::logger()->warn("ERROR: targets not empty, but refused to aim!");
    return std::nullopt;
  }
  return chosen_target;
}

io::Command Aimer::aim(
  const std::list<Target> & targets, std::list<Armor> & armors,
  std::chrono::steady_clock::time_point t_img, double bullet_speed, bool to_now)
{
  /// 选车
  std::optional<Target> opt_target = choose_target(targets, armors);
  if (!opt_target.has_value()) return {false, false, 0, 0};
  auto chosen_target = opt_target.value();
  tools::logger()->debug("[Aimer] bullet_speed: {:.2f}", bullet_speed);
  if (bullet_speed < 10) bullet_speed = 23;
  if (to_now) {
    chosen_target.predict(std::chrono::steady_clock::now());
  }

  bool can_fire = true;

  /// 预选 装甲板
  std::optional<int> opt_rough_chosen_id = choose_armor(chosen_target);
  if (!opt_rough_chosen_id.has_value()) {
    tools::logger()->debug("pre_choose invalid");
    return {false, false, 0, 0};
  }
  int chosen_id_rough = opt_rough_chosen_id.value();

  /// 粗略计算弹道
  Eigen::Vector3d xyz_rough = chosen_target.armor_xyza_list()[chosen_id_rough].head(3);
  auto d_rough = std::sqrt(xyz_rough[0] * xyz_rough[0] + xyz_rough[1] * xyz_rough[1]);
  tools::Trajectory trajectory_rough(bullet_speed, d_rough, xyz_rough[2]);
  if (trajectory_rough.unsolvable) {
    tools::logger()->debug(
      "[Aimer] Unsolvable trajectory0: {:.2f} {:.2f} {:.2f}", bullet_speed, d_rough, xyz_rough[2]);
    yp_should = std::nullopt;
    return {false, false, 0, 0};
  }

  tools::logger()->debug("[Aimer] fly_time: {:.3f}", trajectory_rough.fly_time);

  /// 预测target，额外加入控制延时
  if (to_now) {
    auto t_hit = std::chrono::steady_clock::now() +
                 std::chrono::microseconds(int((delay_gimbal_ + trajectory_rough.fly_time) * 1e6));
    chosen_target.predict(t_hit);
  } else {
    auto t_hit =
      t_img + std::chrono::microseconds(int((delay_gimbal_ + trajectory_rough.fly_time) * 1e6));
    chosen_target.predict(t_hit);
  }

  /// 选择真要正击打的装甲板
  std::optional<int> opt_chosen_id = choose_armor(chosen_target);
  if (!opt_chosen_id.has_value()) {
    tools::logger()->debug("choose invalid");
    yp_should = std::nullopt;
    return {false, false, 0, 0};
  }
  int chosen_id = opt_chosen_id.value();
  aim_id = chosen_id;

  Eigen::Vector3d xyz_compensate = chosen_target.armor_xyza_list()[chosen_id].head(3);
  auto d_compensate =
    std::sqrt(xyz_compensate[0] * xyz_compensate[0] + xyz_compensate[1] * xyz_compensate[1]);
  tools::Trajectory trajectory_compensate(bullet_speed, d_compensate, xyz_compensate[2]);
  if (trajectory_compensate.unsolvable) {
    tools::logger()->debug(
      "[Aimer] Unsolvable trajectory1: {:.2f} {:.2f} {:.2f}", bullet_speed, d_compensate,
      xyz_compensate[2]);
    return {false, false, 0, 0};
  }

  /// TODO: maybe more strategies here

  double yaw_send = std::atan2(xyz_compensate[1], xyz_compensate[0]) + yaw_offset_;
  double pitch_send = trajectory_compensate.pitch + pitch_offset_;

  /// 预测target，不考虑控制延时
  if (to_now) {
    auto t_hit = std::chrono::steady_clock::now() +
                 std::chrono::microseconds(int((trajectory_rough.fly_time) * 1e6));
    chosen_target.predict(t_hit);
  } else {
    auto t_hit = t_img + std::chrono::microseconds(int((trajectory_rough.fly_time) * 1e6));
    chosen_target.predict(t_hit);
  }

  /// 理想弹道
  Eigen::Vector3d xyz_hit = chosen_target.armor_xyza_list()[chosen_id].head(3);
  auto d_hit = std::sqrt(xyz_hit[0] * xyz_hit[0] + xyz_hit[1] * xyz_hit[1]);
  tools::Trajectory trajectory_hit(bullet_speed, d_hit, xyz_hit[2]);
  if (trajectory_hit.unsolvable) {
    tools::logger()->debug(
      "[Aimer] Unsolvable trajectory1: {:.2f} {:.2f} {:.2f}", bullet_speed, d_hit, xyz_hit[2]);
    yp_should = std::nullopt;
    return {false, false, 0, 0};
  }

  auto time_error = trajectory_hit.fly_time - trajectory_rough.fly_time;
  if (std::abs(time_error) > 0.1) {
    tools::logger()->debug("[Aimer] Large time error: {:.3f}", time_error);
    // debug_aim_point.valid = false;
    return {false, false, 0, 0};
  }

  double yaw_should = std::atan2(xyz_hit[1], xyz_hit[0]) + yaw_offset_;
  double pitch_should = trajectory_hit.pitch + pitch_offset_;
  yp_should = std::make_pair(yaw_should, pitch_should);

  return {true, can_fire, yaw_send, pitch_send};
}

void Aimer::clear_last() { last_target_name_ = not_armor; }

std::optional<int> Aimer::choose_armor(const Target & target)
{
  Eigen::VectorXd ekf_x = target.ekf_x();
  std::vector<Eigen::Vector4d> armor_xyza_list = target.armor_xyza_list();
  auto armor_num = armor_xyza_list.size();

  // 如果装甲板未发生过跳变，则只有当前装甲板的位置已知
  if (!target.jumped) {
    tools::logger()->debug("[Aimer] aim at id 0, never jumped");
    return 0;
  }
  // 整车旋转中心的球坐标yaw
  auto center_yaw = std::atan2(ekf_x[2], ekf_x[0]);

  // 如果delta_angle为0，则该装甲板中心和整车中心的连线在世界坐标系的xy平面过原点
  std::vector<double> delta_angle_list;
  for (int i = 0; i < armor_num; i++) {
    auto delta_angle = tools::limit_rad(armor_xyza_list[i][3] - center_yaw);
    delta_angle_list.emplace_back(delta_angle);
  }

  // 不考虑小陀螺
  if (std::abs(ekf_x[7]) <= 1) {
    // 选择在可射击范围内的装甲板
    std::vector<int> id_list;
    for (int i = 0; i < armor_num; i++) {
      if (std::abs(delta_angle_list[i]) > 60 / 57.3) continue;
      id_list.push_back(i);
    }

    // 绝无可能
    if (id_list.empty()) {
      tools::logger()->warn("Empty id list!");
      return std::nullopt;
    }

    // 锁定模式：防止在两个都呈45度的装甲板之间来回切换
    if (id_list.size() > 1) {
      int id0 = id_list[0], id1 = id_list[1];

      // 未处于锁定模式时，选择delta_angle绝对值较小的装甲板，进入锁定模式
      if (lock_id_ != id0 && lock_id_ != id1)
        lock_id_ = (std::abs(delta_angle_list[id0]) < std::abs(delta_angle_list[id1])) ? id0 : id1;
      tools::logger()->debug("[Aimer] aim at id {}, lock mode", lock_id_);
      return lock_id_;
    }

    // 只有一个装甲板在可射击范围内时，退出锁定模式
    lock_id_ = -1;
    return id_list[0];
  }

  // 在小陀螺时，一侧的装甲板不断出现，另一侧的装甲板不断消失，显然前者被打中的概率更高
  for (int i = 0; i < armor_num; i++) {
    if (std::abs(delta_angle_list[i]) > coming_angle_) continue;
    if (ekf_x[7] > 0 && delta_angle_list[i] < leaving_angle_) return i;
    if (ekf_x[7] < 0 && delta_angle_list[i] > -leaving_angle_) return i;
  }
  return std::nullopt;
}

// AimPoint Aimer::choose_coming_aim_point(const Target & target)
// {
//   Eigen::VectorXd ekf_x = target.ekf_x();
//   Eigen::Vector3d center = {ekf_x[0], ekf_x[2], ekf_x[4]};
//   std::vector<Eigen::Vector4d> armor_xyza_list = target.armor_xyza_list();
//   auto armor_num = armor_xyza_list.size();

//   // 整车旋转中心的球坐标yaw
//   auto center_yaw = std::atan2(ekf_x[2], ekf_x[0]);

//   // 如果delta_angle为0，则该装甲板中心和整车中心的连线在世界坐标系的xy平面过原点
//   // delta_angle 为正，在车中心右侧
//   std::vector<double> delta_angle_list;
//   for (int i = 0; i < armor_num; i++) {
//     auto delta_angle = tools::limit_rad(armor_xyza_list[i][3] - center_yaw);
//     delta_angle_list.emplace_back(delta_angle);
//   }

//   if (ekf_x[7] > 0) {  // 向右旋转
//     int chosen_id = 0;
//     double min_val = -1e3;
//     for (int id = 0; id < armor_num; id++) {
//       if (
//         delta_angle_list[id] < -shooting_angle_ &&
//         delta_angle_list[id] > min_val)  // 选择左侧离击打范围最近的装甲板
//       {
//         chosen_id = id;
//         min_val = delta_angle_list[chosen_id];
//       }
//     }
//     auto use_l_h = (armor_num == 4) && (chosen_id == 1 || chosen_id == 3);
//     auto r = (use_l_h) ? ekf_x[8] + ekf_x[9] : ekf_x[8];
//     return {
//       true,
//       {center[0] - std::cos(-shooting_angle_ + center_yaw) * r,
//        center[1] - std::sin(-shooting_angle_ + center_yaw) * r, armor_xyza_list[chosen_id][2],
//        -shooting_angle_ + center_yaw}};
//   } else {
//     int chosen_id = 0;
//     double max_val = 1e3;
//     for (int id = 0; id < armor_num; id++) {
//       if (
//         delta_angle_list[id] > shooting_angle_ &&
//         delta_angle_list[id] < max_val)  // 选择右侧离击打范围最近的装甲板
//       {
//         chosen_id = id;
//         max_val = delta_angle_list[chosen_id];
//       }
//     }
//     auto use_l_h = (armor_num == 4) && (chosen_id == 1 || chosen_id == 3);
//     auto r = (use_l_h) ? ekf_x[8] + ekf_x[9] : ekf_x[8];
//     return {
//       true,
//       {center[0] - std::cos(shooting_angle_ + center_yaw) * r,
//        center[1] - std::sin(shooting_angle_ + center_yaw) * r, armor_xyza_list[chosen_id][2],
//        shooting_angle_ + center_yaw}};
//   }
//   return {0, armor_xyza_list[0]};  // 不会运行到这里
// }

}  // namespace auto_aim