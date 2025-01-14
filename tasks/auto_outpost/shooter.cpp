#include "shooter.hpp"

#include <yaml-cpp/yaml.h>

#include <nlohmann/json.hpp>

#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"
#include "tools/trajectory.hpp"
namespace auto_outpost
{
double get_delta_angle(double d0, double r, double d1)
{
  // 确保输入的三边满足三角形不等式
  if (d0 <= 0 || r <= 0 || d1 <= 0 || d0 + r <= d1 || d0 + d1 <= r || r + d1 <= d0) {
    // tools::logger()->debug("[Shooter.cpp/13] Invalid Geometry!");
    return M_PI;  // 这样肯定不会打
  }

  // 余弦定理
  // cos(theta) = (d0^2 + r^2 - d1^2) / (2 * d0 * r)
  double cos_theta = (std::pow(d0, 2) + std::pow(r, 2) - std::pow(d1, 2)) / (2 * d0 * r);

  // 防止浮点数误差导致 cos_theta 超过 [-1, 1]
  cos_theta = std::max(-1.0, std::min(1.0, cos_theta));

  return std::acos(cos_theta);  // 返回弧度制夹角
}

// 得到哪一個裝甲板是在逐漸傳過來接紫蛋。还有问题，不能用！！！！
// TODO： 改用装甲板的alpha计算
int Shooter::get_next_armor(const auto_aim::Target & target, double flytime, std::chrono::steady_clock::time_point timestamp)
{
  auto t1 = target;
  auto t2 = target;
  auto dt = tools::delta_time(std::chrono::steady_clock::now(), timestamp) + 0.01;
  t2.predict(tools::add_time(timestamp, dt));  //一小段時間之後的target

  auto armor_xyza_list1 = t1.armor_xyza_list();
  auto armor_xyza_list2 = t2.armor_xyza_list();
  int armor_num = armor_xyza_list1.size();

  auto ekf_x1 = t1.ekf_x();
  double d_center1 = std::sqrt(ekf_x1[0] * ekf_x1[0] + ekf_x1[2] * ekf_x1[2]);
  double r1 = ekf_x1[8];

  auto ekf_x2 = t2.ekf_x();
  double d_center2 = std::sqrt(ekf_x2[0] * ekf_x2[0] + ekf_x2[2] * ekf_x2[2]);
  double r2 = ekf_x2[8];

  double min_angle = INFINITY,
         id = -1;  //angle是水平方向上旋轉半徑，車-旋轉中心，車-裝甲板中中心這個三角形中，車-裝甲板中中心邊所對角
  for (int aim_id = 0; aim_id < armor_num; aim_id++) {
    double d1 = std::sqrt(
      armor_xyza_list1[aim_id][0] * armor_xyza_list1[aim_id][0] + armor_xyza_list1[aim_id][1] * armor_xyza_list1[aim_id][1]);
    double d2 = std::sqrt(
      armor_xyza_list2[aim_id][0] * armor_xyza_list2[aim_id][0] + armor_xyza_list2[aim_id][1] * armor_xyza_list2[aim_id][1]);
    if (d1 > d2) {  // 說明在接近
      auto angle = get_delta_angle(d_center1, r1, d1);

      if (std::abs((flytime + ctrl_to_fire_) * ekf_x1[7]) - std::abs(angle) > 0.005) {  //這塊裝甲板還沒錯過發射時機
        if (std::abs(angle) < min_angle) {
          min_angle = std::abs(angle);
          id = aim_id;
        }
      }
    }
  }
  return id;
}

Shooter::Shooter(const std::string & config_path, io::CBoard & cboard)
: cboard_{cboard}, exit_{false}, queue_0_(500), queue_1_(500), queue_2_(500)
{
  auto yaml = YAML::LoadFile(config_path);
  ctrl_to_fire_ = yaml["shoot_delay"].as<double>();
  yaw_offset_ = yaml["yaw_offset"].as<double>() / 57.3;      // degree to rad
  pitch_offset_ = yaml["pitch_offset"].as<double>() / 57.3;  // degree to rad
  // tools::logger()->info("Shooter construct 1");
  // when_to_fire_ = std::thread([this]() {
  //   std::vector<shooter_info> infos(3);
  //   std::vector<std::chrono::steady_clock::time_point> send_times(3);
  //   queue_0_.pop(infos[0]);
  //   queue_1_.pop(infos[1]);
  //   queue_2_.pop(infos[2]);
  //   while (!exit_) {
  //     queue_0_.pop_no_wait(infos[0]);
  //     queue_1_.pop_no_wait(infos[1]);
  //     queue_2_.pop_no_wait(infos[2]);
  //     for (int i = 0; i < infos.size(); i++) {
  //       send_times[i] = infos[i].shoot_time - std::chrono::microseconds(int(ctrl_to_fire_ * 1e6));
  //     }
  //     auto id = choose_armor(send_times, std::chrono::steady_clock::now());
  //     // tools::logger()->info(
  //     //   "#### " + std::to_string(last_hit_id_) + " " + std::to_string(info.aim_id));
  //     // tools::logger()->info(
  //     //   "^^^^ " +
  //     //   std::to_string(tools::delta_time(send_times[id], std::chrono::steady_clock::now())));
  //     if (
  //       std::abs(tools::delta_time(send_times[id], std::chrono::steady_clock::now())) < 0.005 &&
  //       last_hit_id_ != infos[id].aim_id) {
  //       last_hit_id_ = infos[id].aim_id;
  //       cboard_.send({true, true, infos[id].yaw, infos[id].pitch});
  //       tools::logger()->info("########### fire ##########");
  //     } else
  //       cboard_.send({true, false, infos[id].yaw, infos[id].pitch});
  //     std::this_thread::sleep_for(1ms);
  //   }
  // });
}

// 对于一个target，我们要打的位置AimPoint
Eigen::Vector3d Shooter::get_front(const auto_aim::Target & target_origin)
{
  auto ekf_x = target_origin.ekf_x();

  Eigen::Vector3d center = {ekf_x[0], ekf_x[2], ekf_x[4]};
  auto center_yaw = std::atan2(ekf_x[2], ekf_x[0]);  // 整车旋转中心的球坐标yaw

  Eigen::Vector2d center_xy = center.head(2);
  double horizonal_distance = center_xy.norm() - ekf_x[8];
  Eigen::Vector2d hit_point_xy = (horizonal_distance / center_xy.norm()) * center_xy;
  return {hit_point_xy[0], hit_point_xy[1], ekf_x[4]};
}

void Shooter::shoot(
  std::list<auto_aim::Target> targets, std::chrono::steady_clock::time_point timestamp, double bullet_speed, bool to_now)
{
  int mode = 1;  // debug使用，0表示不考虑空气阻力，1表示考虑空气阻力
  if (targets.empty()) {
    cboard_.send({false, false, 0, 0});
    return;
  }
  auto target = targets.front();
  auto target_predicted = target;
  auto t_img = timestamp;
  auto t_fire = timestamp;
  if (to_now) {
    auto dt = tools::delta_time(std::chrono::steady_clock::now(), timestamp) + ctrl_to_fire_;
    t_fire = tools::add_time(t_img, dt);
    t_img + std::chrono::microseconds(int(dt * 1e6));  // ?
    target_predicted.predict(t_fire);
  }

  if (bullet_speed < 15.2 || bullet_speed > 15.8) bullet_speed = 15.4;
  // bullet_speed=15.8;

  auto ekf_x = target_predicted.ekf_x();

  // tools::logger()->debug("omega = {:.2f}",ekf_x[7]);
  if (std::abs(ekf_x[7]) > 1) {  // w 大于1就认为在旋转
    tools::logger()->info("top mode");
    auto target_rotate = target;
    // 程序运行到这一句的时刻
    auto t_decide = tools::add_time(t_img, tools::delta_time(std::chrono::steady_clock::now(), t_img));
    target_rotate.predict(t_decide);  // 补上延迟，主要是神经网络
    ekf_x = target_rotate.ekf_x();

    auto xyz0 = get_front(target_rotate);  // 瞄准点

    double yaw = std::atan2(xyz0[1], xyz0[0]) + yaw_offset_;  // yaw直接瞄准旋转中心

    double d0 = std::sqrt(xyz0[0] * xyz0[0] + xyz0[1] * xyz0[1]);
    tools::Trajectory trajectory0(bullet_speed, d0, xyz0[2], mode);
    if (trajectory0.unsolvable) {
      tools::logger()->debug("[Aimer] Unsolvable trajectory0: {:.2f} {:.2f} {:.2f}", bullet_speed, d0, xyz0[2]);
      return;
    }
    auto pitch = trajectory0.pitch + pitch_offset_;  // pitch瞄准装甲板正对时的位置

    debug_aim_point_ = {true, {xyz0[0], xyz0[1], xyz0[2], 0}};

    auto t_hit = tools::add_time(t_decide, (ctrl_to_fire_ + trajectory0.fly_time));
    target_rotate.predict(t_hit);  // 预测如果这时候发射，紫蛋到达时的情况
    auto armors_hit = target_rotate.armor_xyza_list();
    int armor_num = armors_hit.size();
    int sig = ekf_x[7] < 0 ? -1 : +1;
    auto center_yaw = std::atan2(ekf_x[2], ekf_x[0]);  // - sig * 0.015;

    for (int aim_id = 0; aim_id < armor_num; aim_id++) {
      if (GET_STATE(armor_state, aim_id) == ALLOW) {
        if (
          ((sig * (-armors_hit[aim_id][3] + center_yaw)) <= 0.08) &&
          (sig * (-armors_hit[aim_id][3] + center_yaw)) >= 0) {  // 在击打窗口内
          tools::logger()->info("########## fire ##########");
          cboard_.send({true, true, yaw, -pitch});
          armor_state = 0;
          SET_STATE(armor_state, aim_id, SHOOTED);
          SET_STATE(armor_state, (aim_id - sig + armor_num) % armor_num, SKIP);
          return;
        }
      } else if (GET_STATE(armor_state, aim_id) == SKIP) {  // 上一块刚打过
        tools::logger()->info("---------- wait ----------");
        armor_state = 0;
        SET_STATE(armor_state, aim_id, SHOOTED);
      }
    }
    cboard_.send({true, false, yaw, -pitch});
    nlohmann::json data;
    tools::Plotter plotter;
    data["command_yaw"] = yaw * 57.3;
    data["command_pitch"] = pitch * 57.3;
    data["center_yaw"] = center_yaw * 57.3;
    plotter.plot(data);
    return;
  }

  auto aim_point0 = choose_aim_point(target_predicted);
  debug_aim_point_ = aim_point0;
  if (!aim_point0.valid) {
    // tools::logger()->debug("Invalid aim_point0.");
    cboard_.send({false, false, 0, 0});
    return;
  }

  Eigen::Vector3d xyz0 = aim_point0.xyza.head(3);
  auto d0 = std::sqrt(xyz0[0] * xyz0[0] + xyz0[1] * xyz0[1]);
  tools::Trajectory trajectory0(bullet_speed, d0, xyz0[2], mode);
  if (trajectory0.unsolvable) {
    tools::logger()->debug("[Aimer] Unsolvable trajectory0: {:.2f} {:.2f} {:.2f}", bullet_speed, d0, xyz0[2]);
    debug_aim_point_.valid = false;
    cboard_.send({false, false, 0, 0});
    return;
  }

  auto t_hit = tools::add_time(t_fire, trajectory0.fly_time);
  target_predicted.predict(t_hit);

  auto aim_point1 = choose_aim_point(target_predicted);
  debug_aim_point_ = aim_point1;
  if (!aim_point1.valid) {
    // tools::logger()->debug("Invalid aim_point1.");
    cboard_.send({false, false, 0, 0});
    return;
  }

  Eigen::Vector3d xyz1 = aim_point1.xyza.head(3);
  auto d1 = std::sqrt(xyz1[0] * xyz1[0] + xyz1[1] * xyz1[1]);
  tools::Trajectory trajectory1(bullet_speed, d1, xyz1[2], mode);
  if (trajectory1.unsolvable) {
    tools::logger()->debug("[Aimer] Unsolvable trajectory1: {:.2f} {:.2f} {:.2f}", bullet_speed, d1, xyz1[2]);
    debug_aim_point_.valid = false;
    cboard_.send({false, false, 0, 0});
    return;
  }

  auto time_error = trajectory1.fly_time - trajectory0.fly_time;
  if (std::abs(time_error) > 0.1) {
    tools::logger()->debug("[Aimer] Large time error: {:.3f}", time_error);
    debug_aim_point_.valid = false;
    cboard_.send({false, false, 0, 0});
    return;
  }

  auto yaw = std::atan2(xyz1[1], xyz1[0]) + yaw_offset_;
  auto pitch = trajectory1.pitch + pitch_offset_;
  cboard_.send({true, false, yaw, -pitch});
  nlohmann::json data;
  tools::Plotter plotter;
  data["command_yaw"] = yaw * 57.3;
  data["command_pitch"] = pitch * 57.3;
  plotter.plot(data);

  // tools::logger()->info("command # yaw {:.2f} {:.2f}",yaw*57.3,pitch*57.3);
  return;
}
// void Shooter::schedule(const shooter_info & info)
// {
//   // tools::logger()->info("y {} p {} ", info.yaw, info.pitch);
//   if (!info.valid || info.aim_id == -1) {
//     cboard_.send({0, 0, 0, 0});
//     return;
//   }
//   if (info.aim_id == 0) {
//     queue_0_.push(info);
//   } else if (info.aim_id == 1) {
//     queue_1_.push(info);
//   } else if (info.aim_id == 2) {
//     queue_2_.push(info);
//   }
// }

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
