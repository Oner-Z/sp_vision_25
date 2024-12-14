#include "oldbuff_target.hpp"

namespace auto_buff
{
///voter

Voter::Voter() : clockwise_(0) {}

void Voter::vote(const double angle_last, const double angle_now)
{
  if (std::abs(clockwise_) > 50) return;
  if (angle_last > angle_now)
    clockwise_--;
  else
    clockwise_++;
}

int Voter::clockwise() { return clockwise_ > 0 ? 1 : -1; }

/// Target

Target::Target() : first_in_(true), unsolvable_(true){};

Eigen::Vector3d Target::point_buff2world(Eigen::Vector3d point_in_buff)
{
  if (unsolvable_) return Eigen::Vector3d(0, 0, 0);
  Eigen::Matrix3d R_buff2world = rotation_matrix(Eigen::Vector3d(ekf_.x[4], ekf_.x[5], ekf_.x[6]));
  auto R_yaw = ekf_.x[0];
  auto R_pitch = ekf_.x[2];
  auto R_dis = ekf_.x[3];
  Eigen::Vector3d point_in_world =
    R_buff2world * point_in_buff + Eigen::Vector3d(
                                     R_dis * std::cos(R_pitch) * std::cos(R_yaw),
                                     R_dis * std::cos(R_pitch) * std::sin(R_yaw),
                                     R_dis * std::sin(R_pitch));
  return point_in_world;
}

bool Target::is_unsolve() const { return unsolvable_; }

Eigen::VectorXd Target::ekf_x() const { return ekf_.x; }

Eigen::Matrix3d Target::rotation_matrix(Eigen::Vector3d ypr)
{
  double roll = ypr[2];
  double pitch = ypr[1];
  double yaw = ypr[0];
  double cos_yaw = cos(yaw);
  double sin_yaw = sin(yaw);
  double cos_pitch = cos(pitch);
  double sin_pitch = sin(pitch);
  double cos_roll = cos(roll);
  double sin_roll = sin(roll);
  // clang-format off
    Eigen::Matrix3d R{
      {cos_yaw * cos_pitch, cos_yaw * sin_pitch * sin_roll - sin_yaw * cos_roll, cos_yaw * sin_pitch * cos_roll + sin_yaw * sin_roll},
      {sin_yaw * cos_pitch, sin_yaw * sin_pitch * sin_roll + cos_yaw * cos_roll, sin_yaw * sin_pitch * cos_roll - cos_yaw * sin_roll},
      {         -sin_pitch,                                cos_pitch * sin_roll,                                cos_pitch * cos_roll}
    };// sin_roll = 0,cos_roll=1
  // clang-format on
  return R;
}

/// SmallTarget

SmallTarget::SmallTarget() : Target() {}

void SmallTarget::get_target(
  const std::optional<PowerRune> & p, std::chrono::steady_clock::time_point & timestamp)
{
  // 如果没有识别，退出函数
  static int lost_cn = 0;
  if (!p.has_value()) {
    unsolvable_ = true;
    lost_cn++;
    return;
  }

  static std::chrono::steady_clock::time_point start_timestamp = timestamp;
  auto time_gap = tools::delta_time(timestamp, start_timestamp);

  // init
  if (first_in_) {
    init(time_gap, p.value());
    first_in_ = false;
  }

  // 处理识别时间间隔过大
  if (lost_cn > 6) {
    tools::logger()->debug("[Target] 丢失buff");
    lost_cn = 0;
    first_in_ = true;
    return;
  }

  // kalman update
  update(time_gap, p.value());

  // 处理发散
  if (std::abs(ekf_.x[7]) > SMALL_W * 1.5 || std::abs(ekf_.x[7]) < SMALL_W / 1.5) {
    tools::logger()->debug("[Target] 小符角度发散spd: {:.2f}", ekf_.x[7]);
    first_in_ = true;
    // update(time_gap, p.value());
    return;
  }

  //
}

void SmallTarget::predict(double dt)
{
  // 预测下一个状态
  // clang-format off
  A_ << 1.0,  dt, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0,  dt,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0;
  // 过程噪声协方差矩阵                            //// 调整
  auto v1 = 0.9;  // 角加速度方差
  auto a = dt * dt * dt * dt / 4;
  auto b = dt * dt * dt / 2;
  auto c = dt * dt;
  Q_ << a * v1, b * v1, 0.0, 0.0, 0.0, 0.0,  0.0, 0.0,
        b * v1, c * v1, 0.0, 0.0, 0.0, 0.0,  0.0, 0.0,
           0.0,    0.0, 0.0, 0.0, 0.0, 0.0,  0.0, 0.0,
           0.0,    0.0, 0.0, 0.0, 0.0, 0.0,  0.0, 0.0,
           0.0,    0.0, 0.0, 0.0, 0.0, 0.0,  0.0, 0.0,
           0.0,    0.0, 0.0, 0.0, 0.0, 0.0,  0.0, 0.0,
           0.0,    0.0, 0.0, 0.0, 0.0, 0.0, 0.05, 0.0,
           0.0,    0.0, 0.0, 0.0, 0.0, 0.0,  0.0, 0.0;
  // clang-format on 
  auto f = [&](const Eigen::VectorXd & x) -> Eigen::VectorXd {
    Eigen::VectorXd x_prior = A_ * x;
    x_prior[0] = tools::limit_rad(x_prior[0]);
    x_prior[2] = tools::limit_rad(x_prior[2]);
    x_prior[4] = tools::limit_rad(x_prior[4]);
    x_prior[5] = tools::limit_rad(x_prior[5]);
    x_prior[6] = tools::limit_rad(x_prior[6]);
    return x_prior;
  };
  ekf_.predict(A_, Q_, f);
}

void SmallTarget::init(double nowtime, const PowerRune & p)
{
  // 初始化内部变量
  lasttime_ = nowtime;
  unsolvable_ = true;

  // 初始状态协方差矩阵
  x0_.resize(8);
  P0_.resize(8, 8);
  A_.resize(8, 8);
  Q_.resize(8, 8);
  H_.resize(6, 8);
  R_.resize(6, 6);
  // [R_yaw]
  // [v_R_yaw]
  // [R_pitch]
  // [R_dis]
  // [row]
  // [pitch]
  // [angle/row]
  // [w    ]   w=CV_PI/6

  // clang-format off
    // 初始状态
    x0_ << p.ypd_in_world[0], 0.0, p.ypd_in_world[1], p.ypd_in_world[2],
           p.ypr_in_world[0], p.ypr_in_world[1], p.ypr_in_world[2], 
           SMALL_W * voter.clockwise();
    // 初始状态协方差矩阵
    P0_ << 10.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
            0.0, 10.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
            0.0,  0.0, 10.0,  0.0,  0.0,  0.0,  0.0,  0.0,
            0.0,  0.0,  0.0, 10.0,  0.0,  0.0,  0.0,  0.0,
            0.0,  0.0,  0.0,  0.0, 10.0,  0.0,  0.0,  0.0,
            0.0,  0.0,  0.0,  0.0,  0.0, 10.0,  0.0,  0.0,
            0.0,  0.0,  0.0,  0.0,  0.0,  0.0, 10.0,  0.0,
            0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0, 10.0;
    // 状态转移矩阵
    // A_ 
    // 过程噪声协方差矩阵                            //// 调整
    // Q_ 
    // 测量方程矩阵
    H_ << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
          0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
          0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
          0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
          0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
          0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0;
    // 测量噪声协方差矩阵                            //// 调整
    R_ << 0.5, 0.0, 0.0, 0.0, 0.0, 0.0,
          0.0, 0.5, 0.0, 0.0, 0.0, 0.0,
          0.0, 0.0, 0.5, 0.0, 0.0, 0.0,
          0.0, 0.0, 0.0, 0.05, 0.0, 0.0,
          0.0, 0.0, 0.0, 0.0, 0.05, 0.0,
          0.0, 0.0, 0.0, 0.0, 0.0, 1.0;
  // clang-format on

  // 防止夹角求和出现异常值
  auto x_add = [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) -> Eigen::VectorXd {
    Eigen::VectorXd c = a + b;
    c[0] = tools::limit_rad(c[0]);
    c[2] = tools::limit_rad(c[2]);
    c[4] = tools::limit_rad(c[4]);
    c[5] = tools::limit_rad(c[5]);
    c[6] = tools::limit_rad(c[6]);
    return c;
  };
  // 创建扩展卡尔曼滤波器对象
  ekf_ = tools::ExtendedKalmanFilter(x0_, P0_, x_add);
}

void SmallTarget::update(double nowtime, const PowerRune & p)
{
  // [x]
  // [v_x]
  // [y]
  // [v_y]
  // [z]
  // [v_z]
  // [row]
  // [pitch]
  // [angle/row]
  // [w    ]   w=CV_PI/6
  const Eigen::VectorXd & ypd = p.ypd_in_world;
  const Eigen::VectorXd & ypr = p.ypr_in_world;

  // 处理扇叶跳变
  if (abs(ypr[2] - ekf_.x[6]) > CV_PI / 12) {
    for (int i = -5; i <= 5; i++) {
      double angle_c = ekf_.x[6] + i * 2 * CV_PI / 5;
      if (std::fabs(angle_c - ypr[2]) < CV_PI / 5) {
        ekf_.x[6] += i * 2 * CV_PI / 5;
        break;
      }
    }
  }

  // vote判断是顺时针还是逆时针旋转
  voter.vote(ekf_.x[6], ypr[2]);
  if (voter.clockwise() * ekf_.x[7] < 0) ekf_.x[7] *= -1;

  // 预测下一个状态
  predict(nowtime - lasttime_);

  auto z_subtract = [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) -> Eigen::VectorXd {
    Eigen::VectorXd c = a - b;  //6 1
    c[0] = tools::limit_rad(c[0]);
    c[1] = tools::limit_rad(c[1]);
    c[3] = tools::limit_rad(c[3]);
    c[4] = tools::limit_rad(c[4]);
    c[5] = tools::limit_rad(c[5]);
    return c;
  };

  Eigen::VectorXd z{{ypd[0], ypd[1], ypd[2], ypr[0], ypr[1], ypr[2]}};
  ekf_.update(z, H_, R_, z_subtract);

  // 更新lasttime
  lasttime_ = nowtime;
  unsolvable_ = false;
  return;
}

/// BigTarget

BigTarget::BigTarget() : Target() {}

void BigTarget::get_target(
  const std::optional<PowerRune> & p, std::chrono::steady_clock::time_point & timestamp)
{
  // 如果没有识别，退出函数
  static int lost_cn = 0;
  if (!p.has_value()) {
    unsolvable_ = true;
    lost_cn++;
    return;
  }

  static std::chrono::steady_clock::time_point start_timestamp = timestamp;
  auto time_gap = tools::delta_time(timestamp, start_timestamp);

  // init
  if (first_in_) {
    init(time_gap, p.value());
    first_in_ = false;
  }

  // 处理识别时间间隔过大
  if (lost_cn > 6) {
    tools::logger()->debug("[Target] 丢失buff");
    lost_cn = 0;
    first_in_ = true;
    return;
  }

  // kalman update
  update(time_gap, p.value());

  // 处理发散
  // if (
  //   ekf_.x[7] > 1.045 * 1.5 || ekf_.x[7] < 0.78 / 1.5 || ekf_.x[8] > 2.0 * 1.5 ||
  //   ekf_.x[8] < 1.884 / 1.5) {
  //   tools::logger()->debug("[Target] 大符角度发散a: {:.2f}b:{:.2f}", ekf_.x[7], ekf_.x[8]);
  //   first_in_ = true;
  //   return;
  // }
}

void BigTarget::predict(double dt)
{
  // 预测下一个状态
  double spd = ekf_.x[7];
  double a = ekf_.x[8];
  double w = ekf_.x[9];
  double theta = ekf_.x[10];
  // clang-format off
  A_ << 1.0,  dt, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,//R_yaw
        0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,//v_R_yaw
        0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,//R_pitch
        0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,//R_dis
        0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,//yaw
        0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,//pitch
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 
        voter.clockwise() * (-1 / w * cos(theta + w * dt) + 1 / w * cos(theta) - dt),
        voter.clockwise() * (a / (w * w) * cos(theta + w * dt) + a /w*sin(theta + w * dt) * dt - a / (w * w) * cos(theta)),
        voter.clockwise() * (a / w * sin(theta + w * dt) - a / w * sin(theta)),//row
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, sin(theta + w * dt) - 1, dt * a * cos(theta + w * dt), a * cos(theta + w * dt),//spd
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,//a
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0,//w
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  dt, 1.0;//theta
  // 过程噪声协方差矩阵                            //// 调整
  auto v1 = 0.9;  // 角加速度方差
  auto a = dt * dt * dt * dt / 4;
  auto b = dt * dt * dt / 2;
  auto c = dt * dt;
  Q_ << a * v1, b * v1, 0.0, 0.0, 0.0, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
        b * v1, c * v1, 0.0, 0.0, 0.0, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
           0.0,    0.0, 0.0, 0.0, 0.0, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
           0.0,    0.0, 0.0, 0.0, 0.0, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
           0.0,    0.0, 0.0, 0.0, 0.0, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
           0.0,    0.0, 0.0, 0.0, 0.0, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
           0.0,    0.0, 0.0, 0.0, 0.0, 0.0, 0.05,  0.0,  0.0,  0.0,  0.0,
           0.0,    0.0, 0.0, 0.0, 0.0, 0.0,  0.0, 0.05,  0.0,  0.0,  0.0,
           0.0,    0.0, 0.0, 0.0, 0.0, 0.0,  0.0,  0.0, 0.03,  0.0,  0.0,
           0.0,    0.0, 0.0, 0.0, 0.0, 0.0,  0.0,  0.0,  0.0, 0.03,  0.0,
           0.0,    0.0, 0.0, 0.0, 0.0, 0.0,  0.0,  0.0,  0.0,  0.0, 0.05;
  auto f = [&](const Eigen::VectorXd & x) -> Eigen::VectorXd {
    Eigen::VectorXd x_prior = A_ * x;
    x_prior[0] = tools::limit_rad(x_prior[0]);
    x_prior[2] = tools::limit_rad(x_prior[2]);
    x_prior[4] = tools::limit_rad(x_prior[4]); // yaw
    x_prior[5] = tools::limit_rad(x_prior[5]); // pitch
    x_prior[6] = tools::limit_rad(x_prior[6]); // roll
    // x_prior[6] = tools::limit_rad(x[6] + voter.clockwise() * (-a / w * cos(theta + w * dt) + a / w * cos(theta) + (2.09 - a) * dt));
    // x_prior[7] = a * sin(theta + w * dt) + 2.09 - a; // spd
    // x_prior[10] = tools::limit_rad(theta + w * dt); // theta
    return x_prior;
  };
  // clang-format on
  ekf_.predict(A_, Q_, f);
}

void BigTarget::init(double nowtime, const PowerRune & p)
{
  // 初始化内部变量
  lasttime_ = nowtime;
  unsolvable_ = true;

  // 初始状态协方差矩阵
  x0_.resize(11);
  P0_.resize(11, 11);
  A_.resize(11, 11);
  Q_.resize(11, 11);
  H_.resize(6, 11);
  R_.resize(6, 6);

  // [R_yaw]
  // [v_R_yaw]
  // [R_pitch]
  // [R_dis]
  // [yaw]
  // [pitch]
  // [angle/row] 角度
  // [spd]       角速度 a*sin(wt) + 2.09 - a
  // [a]         0.78-1.045
  // [w]         1.884-2.000
  // [theta]

  // clang-format off
  // 初始状态
  x0_ << p.ypd_in_world[0], 0.0, p.ypd_in_world[1], p.ypd_in_world[2],
         p.ypr_in_world[0], p.ypr_in_world[1], p.ypr_in_world[2], 
         1.1775, 0.9125, 1.942, 0.0;//spd a w theta
  // 初始状态协方差矩阵
  P0_ << 10.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
          0.0, 10.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
          0.0,  0.0, 10.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
          0.0,  0.0,  0.0, 10.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
          0.0,  0.0,  0.0,  0.0, 10.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
          0.0,  0.0,  0.0,  0.0,  0.0, 10.0,  0.0,  0.0,  0.0,  0.0,  0.0,
          0.0,  0.0,  0.0,  0.0,  0.0,  0.0, 10.0,  0.0,  0.0,  0.0,  0.0,
          0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0, 10.0,  0.0,  0.0,  0.0,
          0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0, 10.0,  0.0,  0.0,
          0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0, 10.0,  0.0,
          0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0, 10.0;
  // 状态转移矩阵
  // A_ << 
  // 过程噪声协方差矩阵                            //// 调整
  // Q_ << 
  // 测量方程矩阵
  H_ << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
        0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
        0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
        0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
        0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0;
       
  // 测量噪声协方差矩阵                            //// 调整
  R_ << 0.5, 0.0, 0.0, 0.0,  0.0,  0.0,
        0.0, 0.5, 0.0, 0.0,  0.0,  0.0, 
        0.0, 0.0, 0.5, 0.0,  0.0,  0.0, 
        0.0, 0.0, 0.0, 0.05, 0.0,  0.0,
        0.0, 0.0, 0.0, 0.0,  0.05, 0.0,
        0.0, 0.0, 0.0, 0.0,  0.0,  1.0;
  // clang-format on

  // 防止夹角求和出现异常值
  auto x_add = [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) -> Eigen::VectorXd {
    Eigen::VectorXd c = a + b;
    c[0] = tools::limit_rad(c[0]);
    c[2] = tools::limit_rad(c[2]);
    c[4] = tools::limit_rad(c[4]);
    c[5] = tools::limit_rad(c[5]);
    c[6] = tools::limit_rad(c[6]);
    return c;
  };
  // 创建扩展卡尔曼滤波器对象
  ekf_ = tools::ExtendedKalmanFilter(x0_, P0_, x_add);
}

void BigTarget::update(double nowtime, const PowerRune & p)
{
  // [R_yaw]
  // [v_R_yaw]
  // [R_pitch]
  // [R_dis]
  // [yaw]
  // [pitch]
  // [angle/row] 角度
  // [spd]       角速度 a*sin(wt) + 2.09 - a
  // [a]         0.78-1.045
  // [w]         1.884-2.000
  // [sita]
  const Eigen::VectorXd & ypd = p.ypd_in_world;
  const Eigen::VectorXd & ypr = p.ypr_in_world;

  // 处理扇叶跳变
  if (abs(ypr[2] - ekf_.x[6]) > CV_PI / 12) {
    for (int i = -5; i <= 5; i++) {
      double angle_c = ekf_.x[6] + i * 2 * CV_PI / 5;
      if (std::fabs(angle_c - ypr[2]) < CV_PI / 5) {
        ekf_.x[6] += i * 2 * CV_PI / 5;
        break;
      }
    }
  }

  // vote判断是顺时针还是逆时针旋转
  voter.vote(ekf_.x[6], ypr[2]);

  // 预测下一个状态
  predict(nowtime - lasttime_);

  auto z_subtract = [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) -> Eigen::VectorXd {
    Eigen::VectorXd c = a - b;  //6 1
    c[0] = tools::limit_rad(c[0]);
    c[1] = tools::limit_rad(c[1]);
    c[3] = tools::limit_rad(c[3]);
    c[4] = tools::limit_rad(c[4]);
    c[5] = tools::limit_rad(c[5]);
    return c;
  };

  Eigen::VectorXd z{{ypd[0], ypd[1], ypd[2], ypr[0], ypr[1], ypr[2]}};
  ekf_.update(z, H_, R_, z_subtract);

  // 更新lasttime
  lasttime_ = nowtime;
  unsolvable_ = false;
  return;
}

}  // namespace auto_buff