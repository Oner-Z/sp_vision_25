#include "shooter.hpp"

#include <yaml-cpp/yaml.h>

#include <nlohmann/json.hpp>

#include "tasks/auto_aim/armor.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"
#include "tools/trajectory.hpp"
namespace auto_outpost
{
Shooter::Shooter(const std::string & config_path) : exit_{false}, queue_0_(500), queue_1_(500), queue_2_(500)
{
  auto yaml = YAML::LoadFile(config_path);
  ctrl_to_fire_ = yaml["shoot_delay"].as<double>();
  yaw_offset_ = yaml["yaw_offset"].as<double>() / 57.3;      // degree to rad
  pitch_offset_ = yaml["pitch_offset"].as<double>() / 57.3;  // degree to rad
  direction_ = yaml["direction"].as<double>();               // pitch正方向。气动为1，摩擦轮为-1
}

// 对于outpost，我们要打的位置AimPoint
Eigen::Vector3d Shooter::get_outpost_front(const auto_aim::Target & target_origin)
{
  auto ekf_x = target_origin.ekf_x();

  Eigen::Vector3d center = {ekf_x[0], ekf_x[2], ekf_x[4]};
  auto center_yaw = std::atan2(ekf_x[2], ekf_x[0]);  // 整车旋转中心的球坐标yaw

  Eigen::Vector2d center_xy = center.head(2);
  double horizonal_distance = center_xy.norm() - ekf_x[8];
  Eigen::Vector2d hit_point_xy = (horizonal_distance / center_xy.norm()) * center_xy;
  return {hit_point_xy[0], hit_point_xy[1], ekf_x[4]};
}

// 对于小陀螺的车，我们要打的位置AimPoint
Eigen::Vector3d Shooter::get_top_front(const auto_aim::Target & target_origin)
{
  // 这时候瞄准位置的z坐标是下一块来接紫蛋的装甲板的高度
  auto ekf_x = target_origin.ekf_x();

  Eigen::Vector3d center = {ekf_x[0], ekf_x[2], ekf_x[4]};
  auto center_yaw = std::atan2(ekf_x[2], ekf_x[0]);  // 整车旋转中心的球坐标yaw

  Eigen::Vector2d center_xy = center.head(2);
  double horizonal_distance = center_xy.norm() - ekf_x[8];
  Eigen::Vector2d hit_point_xy = (horizonal_distance / center_xy.norm()) * center_xy;

  auto armors = target_origin.armor_xyza_list();
  int armor_num = armors.size();

  int sig_w = ekf_x[7] > 0 ? 1 : -1;  // 旋转方向
  int armor_id;
  double nearst_angle = INFINITY;
  for (int aim_id = 0; aim_id < armor_num; aim_id++) {
    if (sig_w * (armors[aim_id][3] - center_yaw) < 0) {  // 说明该装甲板正在转过来吃紫蛋:)
      if (abs(armors[aim_id][3] - center_yaw) < nearst_angle) {
        armor_id = aim_id;
        nearst_angle = armors[aim_id][3];
      }
    }
  }

  return {hit_point_xy[0], hit_point_xy[1], armors[armor_id][2]};
}

io::Command Shooter::shoot(
  std::list<auto_aim::Target> targets, std::chrono::steady_clock::time_point timestamp, double bullet_speed, bool to_now)
{
  int mode = 1;  // debug使用，0表示不考虑空气阻力，1表示考虑空气阻力

  // 没有有效目标时：
  if (targets.empty()) {
    return {false, false, 0, 0};
  }

  // 得到有效目标时
  auto target = targets.front();
  auto target_predicted = target;
  auto t_img = timestamp;
  auto t_fire = timestamp;
  if (bullet_speed < 15.2 || bullet_speed > 16.5) bullet_speed = 16.3;
  auto ekf_x = target_predicted.ekf_x();

  if (std::abs(ekf_x[7]) > 1) {  // w 大于1就认为在旋转
    auto target_rotate = target;
    // 程序运行到这一句的时刻
    auto t_decide = tools::add_time(t_img, tools::delta_time(std::chrono::steady_clock::now(), t_img));
    target_rotate.predict(t_decide);  // 补上延迟，主要是神经网络
    ekf_x = target_rotate.ekf_x();

    Eigen::Vector3d xyz0;

    if (target.name == auto_aim::outpost) {  // 反前哨站。不存在高低装甲板
      tools::logger()->info("anti outpost mode");
      xyz0 = get_outpost_front(target_rotate);  // 瞄准点
    } else {                                    // 反小陀螺，需要应对高低装甲板
      // tools::logger()->info("anti top mode");
      xyz0 = get_top_front(target_rotate);  // 瞄准点
    }

    double yaw = std::atan2(xyz0[1], xyz0[0]) + yaw_offset_;  // yaw直接瞄准旋转中心

    double d0 = std::sqrt(xyz0[0] * xyz0[0] + xyz0[1] * xyz0[1]);
    tools::Trajectory trajectory0(bullet_speed, d0, xyz0[2], mode);
    if (trajectory0.unsolvable) {
      tools::logger()->debug("[Aimer] Unsolvable trajectory0: {:.2f} {:.2f} {:.2f}", bullet_speed, d0, xyz0[2]);
      return {false, false, 0, 0};
    }
    auto pitch = trajectory0.pitch + pitch_offset_;  // pitch瞄准装甲板正对时的位置

    debug_aim_point_ = {true, {xyz0[0], xyz0[1], xyz0[2], 0}};

    auto t_hit = tools::add_time(t_decide, (ctrl_to_fire_ + trajectory0.fly_time));
    target_rotate.predict(t_hit);  // 预测如果这时候发射，紫蛋到达时的情况
    auto armors_hit = target_rotate.armor_xyza_list();
    int armor_num = armors_hit.size();
    int sig = ekf_x[7] < 0 ? -1 : +1;
    auto center_yaw = std::atan2(ekf_x[2], ekf_x[0]);  // - sig * 0.015;
    auto armor_state = target.armor_state;
    for (int aim_id = 0; aim_id < armor_num; aim_id++) {
      if (GET_STATE(armor_state, aim_id) == ALLOW) {
        if (
          ((sig * (-armors_hit[aim_id][3] + center_yaw)) <= 0.08) &&
          (sig * (-armors_hit[aim_id][3] + center_yaw)) >= 0) {  // 在击打窗口内
          tools::logger()->info("########## fire ##########");
          io::Command command = {true, true, yaw, direction_ * pitch};
          armor_state = 0;
          if (target_rotate.name = auto_aim::outpost) {
            SET_STATE(armor_state, aim_id, SHOOTED);
            SET_STATE(armor_state, (aim_id - sig + armor_num) % armor_num, SKIP);
          }
          return command;
        }
      } else if (GET_STATE(armor_state, aim_id) == SKIP) {  // 上一块刚打过
        tools::logger()->info("---------- wait ----------");
        armor_state = 0;
        if (target_rotate.name = auto_aim::outpost) {
          SET_STATE(armor_state, aim_id, SHOOTED);
        }
      }
    }
    io::Command command = {true, false, yaw, direction_ * pitch};
    return command;
  }  // end of anti top/outpost
  else {  // 平动目标
    if (to_now) {
      auto dt = tools::delta_time(std::chrono::steady_clock::now(), timestamp) + ctrl_to_fire_;
      t_fire = tools::add_time(t_img, dt);
      // t_img + std::chrono::microseconds(int(dt * 1e6));  意义不明的一句 ???
      target_predicted.predict(t_fire);  // 预测紫蛋出膛时的目标状态
    }

    auto aim_point0 = choose_aim_point(target_predicted);  // 获取平动目标的击打目标点
    debug_aim_point_ = aim_point0;
    if (!aim_point0.valid) {
      // tools::logger()->debug("Invalid aim_point0.");
      io::Command command = {false, false, 0, 0};
      return command;
    }

    Eigen::Vector3d xyz0 = aim_point0.xyza.head(3);
    auto d0 = std::sqrt(xyz0[0] * xyz0[0] + xyz0[1] * xyz0[1]);
    tools::Trajectory trajectory0(bullet_speed, d0, xyz0[2], mode);
    if (trajectory0.unsolvable) {
      tools::logger()->debug("[Aimer] Unsolvable trajectory0: {:.2f} {:.2f} {:.2f}", bullet_speed, d0, xyz0[2]);
      debug_aim_point_.valid = false;
      io::Command command = {false, false, 0, 0};
      return command;
    }

    auto t_hit = tools::add_time(t_fire, trajectory0.fly_time);
    target_predicted.predict(t_hit);

    auto aim_point1 = choose_aim_point(target_predicted);
    debug_aim_point_ = aim_point1;
    if (!aim_point1.valid) {
      // tools::logger()->debug("Invalid aim_point1.");
      io::Command command = {false, false, 0, 0};
      return command;
    }

    Eigen::Vector3d xyz1 = aim_point1.xyza.head(3);
    auto d1 = std::sqrt(xyz1[0] * xyz1[0] + xyz1[1] * xyz1[1]);
    tools::Trajectory trajectory1(bullet_speed, d1, xyz1[2], mode);
    if (trajectory1.unsolvable) {
      tools::logger()->debug("[Aimer] Unsolvable trajectory1: {:.2f} {:.2f} {:.2f}", bullet_speed, d1, xyz1[2]);
      debug_aim_point_.valid = false;
      io::Command command = {false, false, 0, 0};
      return command;
    }

    auto time_error = trajectory1.fly_time - trajectory0.fly_time;
    if (std::abs(time_error) > 0.1) {
      tools::logger()->debug("[Aimer] Large time error: {:.3f}", time_error);
      debug_aim_point_.valid = false;
      io::Command command = {false, false, 0, 0};
      return command;
    }

    double yaw;
    yaw = std::atan2(xyz1[1], xyz1[0]) + yaw_offset_;
    tools::logger()->debug("xyz1[0] = {}, xyz1[1] = {}", xyz1[0], xyz1[1]);

    auto pitch = trajectory1.pitch + pitch_offset_;
    io::Command command = {true, false, yaw, direction_ * pitch};

    return command;
  }
}

AimPoint Shooter::choose_aim_point(const auto_aim::Target & target)
{
  Eigen::VectorXd ekf_x = target.ekf_x();
  std::vector<Eigen::Vector4d> armor_xyza_list = target.armor_xyza_list();
  auto armor_num = armor_xyza_list.size();

  // 如果装甲板未发生过跳变，则只有当前装甲板的位置已知
  if (!target.jumped) return {true, armor_xyza_list[0]};

  // 整车旋转中心的球坐标yaw
  auto center_yaw = std::atan2(ekf_x[2], ekf_x[0]);

  // 如果delta_angle为0，则该装甲板中心和整车中心的连线在世界坐标系的xy平面过原点
  std::vector<double> delta_angle_list;
  for (int i = 0; i < 3; i++) {
    auto delta_angle = tools::limit_rad(armor_xyza_list[i][3] - center_yaw);
    delta_angle_list.emplace_back(delta_angle);
  }

  // 选择在可射击范围内的装甲板
  std::vector<int> id_list;
  // tools::logger()->debug("-------------------------------------------");
  for (int i = 0; i < armor_num; i++) {
    if (std::abs(delta_angle_list[i]) > 60 / 57.3) {
      // tools::logger()->debug(std::to_string(std::abs(delta_angle_list[i] * 57.3)));
      continue;  // 以60度为射击范围
    }
    id_list.push_back(i);
  }
  // tools::logger()->debug("-------------------------------------------");

  // 绝无可能
  if (id_list.empty()) {
    tools::logger()->warn("Empty id list in Shooter!");
    return {false, armor_xyza_list[0]};
  }

  // 锁定模式：防止在两个都呈45度的装甲板之间来回切换
  if (id_list.size() > 1) {
    int id0 = id_list[0], id1 = id_list[1];

    // 未处于锁定模式时，选择delta_angle绝对值较小（更“正对”机器人）的装甲板，进入锁定模式
    if (lock_id_ != id0 && lock_id_ != id1)
      lock_id_ = (std::abs(delta_angle_list[id0]) < std::abs(delta_angle_list[id1])) ? id0 : id1;

    return {true, armor_xyza_list[lock_id_]};
  }

  // 只有一个装甲板在可射击范围内时，退出锁定模式
  lock_id_ = -1;
  return {true, armor_xyza_list[id_list[0]]};
}

Shooter::~Shooter()
{
  exit_ = true;
  if (when_to_fire_.joinable()) {
    when_to_fire_.join();
  }
  tools::logger()->info("Shooter destructed");
}

}  // namespace auto_outpost
