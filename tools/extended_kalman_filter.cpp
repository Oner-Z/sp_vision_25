#include "extended_kalman_filter.hpp"

#include "tools/plotter.hpp"

namespace tools
{
ExtendedKalmanFilter::ExtendedKalmanFilter(
  const Eigen::VectorXd & x0, const Eigen::MatrixXd & P0,
  std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> x_add)
: x(x0), P(P0), I(Eigen::MatrixXd::Identity(x0.rows(), x0.rows())), x_add(x_add)
{
  Eigen::VectorXd R_dig{{4e-3, 4e-3, 1, 2e-1}};  // 观测噪声 yaw pitch distance angle, 前三个是球坐标系坐标，最后一个是朝向
  R_adaptive = R_dig.asDiagonal();
}

Eigen::VectorXd ExtendedKalmanFilter::predict(const Eigen::MatrixXd & F, const Eigen::MatrixXd & Q)
{
  return predict(F, Q, [&](const Eigen::VectorXd & x) { return F * x; });
}

Eigen::VectorXd ExtendedKalmanFilter::predict(
  const Eigen::MatrixXd & F, const Eigen::MatrixXd & Q, std::function<Eigen::VectorXd(const Eigen::VectorXd &)> f)
{
  P = F * P * F.transpose() + Q;
  x = f(x);
  return x;
}

Eigen::VectorXd ExtendedKalmanFilter::update(
  const Eigen::VectorXd & z, const Eigen::MatrixXd & H, const Eigen::MatrixXd & R,
  std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> z_subtract)
{
  return update(z, H, R, [&](const Eigen::VectorXd & x) { return H * x; }, z_subtract);
}

Eigen::VectorXd ExtendedKalmanFilter::update(
  const Eigen::VectorXd & z, const Eigen::MatrixXd & H, const Eigen::MatrixXd & R,
  std::function<Eigen::VectorXd(const Eigen::VectorXd &)> h,
  std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> z_subtract)
{
  Eigen::MatrixXd R_used = use_adaptive_R ? R_adaptive : R;

  tools::Plotter plotter;
  nlohmann::json data;
  auto temp = R_used.diagonal();

  data["R_yaw"] = temp(0);
  data["R_pitch"] = temp(1);
  data["R_distance"] = temp(2);
  data["R_angle"] = temp(3);

  Eigen::MatrixXd K = P * H.transpose() * (H * P * H.transpose() + R).inverse();

  // Stable Compution of the Posterior Covariance
  // https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python/blob/master/07-Kalman-Filter-Math.ipynb
  P = (I - K * H) * P * (I - K * H).transpose() + K * R * K.transpose();
  Eigen::VectorXd residual = z_subtract(z, h(x));
  x = x_add(x, K * residual);

  data["residual_yaw"] = residual[0];
  data["residual_pitch"] = residual[1];
  data["residual_distance"] = residual[2];
  data["residual_angle"] = residual[3];

  if (use_adaptive_R) {
    Eigen::MatrixXd innovation_cov = residual * residual.transpose();
    R_adaptive = smooth * R_adaptive + (1.0 - smooth) * innovation_cov;
  }

  plotter.plot(data);
  return x;
}

}  // namespace tools