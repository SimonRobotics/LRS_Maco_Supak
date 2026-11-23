// Eight-state Kalman filter for A2 §2 -- no ROS in here, so it can be unit tested.
//
// State, ENU in frame `map`:
//   x = [px, py, pz, vx, vy, vz, yaw, b]^T
//   p    position [m]
//   v    velocity [m/s], ENU (not body frame)
//   yaw  heading [rad], ENU (0 = East, counter-clockwise), kept in [-pi, pi)
//   b    gyro z bias [rad/s]
//
// Prediction over dt with the measured gyro rate w (an input, not a state):
//   p   <- p + v dt                      constant velocity
//   v   <- v                             (acceleration = white noise, in Q)
//   yaw <- wrap(yaw + (w - b) dt)        gyro minus estimated bias
//   b   <- b                             (random walk, in Q)
//   P   <- F P F^T + Q(dt)
// The model is linear in x, so F is exact: identity plus dt in (p, v) and -dt in (yaw, b).
//
// Measurement update, z = h(x) + noise with covariance R:
//   r = z - h(x)            innovation, angle rows wrapped
//   S = H P H^T + R         its expected covariance
//   K = P H^T S^-1          computed by solving with S, never by inverting it
//   x <- x + K r
//   P <- (I - K H) P (I - K H)^T + K R K^T     Joseph form: stays symmetric and positive
// NIS = r^T S^-1 r is returned with every update (consistency check, later the outlier gate).
#pragma once

#include <Eigen/Dense>

#include <array>
#include <cmath>
#include <limits>

namespace lrs_localization
{

constexpr int kNx = 8;
using StateVec = Eigen::Matrix<double, kNx, 1>;
using StateCov = Eigen::Matrix<double, kNx, kNx>;

enum Index : int {kPx = 0, kPy, kPz, kVx, kVy, kVz, kYaw, kBias};

// Wrap an angle into [-pi, pi).
inline double wrapAngle(double a)
{
  if (a >= -M_PI && a < M_PI) {return a;}  // already wrapped: no round-off
  constexpr double kTwoPi = 2.0 * M_PI;
  a = std::fmod(a + M_PI, kTwoPi);
  if (a < 0.0) {a += kTwoPi;}
  return a - M_PI;
}

// Continuous-time noise densities (power spectral densities) of the process model.
struct ProcessNoise
{
  double acc_xy{0.5};   // horizontal acceleration [m^2/s^3]
  double acc_z{0.2};    // vertical acceleration [m^2/s^3]
  double gyro{1e-6};    // gyro rate noise driving yaw [rad^2/s]
  double bias{1e-8};    // gyro bias random walk [rad^2/s^3]
};

template<int M>
struct UpdateResult
{
  bool applied{false};
  Eigen::Matrix<double, M, 1> innovation{Eigen::Matrix<double, M, 1>::Zero()};
  Eigen::Matrix<double, M, M> S{Eigen::Matrix<double, M, M>::Zero()};
  double nis{0.0};  // r^T S^-1 r
};

class Ekf8
{
public:
  void initialize(const StateVec & x0, const StateCov & P0, double t0)
  {
    x_ = x0;
    x_(kYaw) = wrapAngle(x_(kYaw));
    P_ = 0.5 * (P0 + P0.transpose());
    t_ = t0;
    initialized_ = true;
  }

  bool initialized() const {return initialized_;}
  double time() const {return t_;}
  const StateVec & x() const {return x_;}
  const StateCov & P() const {return P_;}
  void setProcessNoise(const ProcessNoise & q) {q_ = q;}
  const ProcessNoise & processNoise() const {return q_;}

  // Propagate to time t [s] with gyro rate w [rad/s]. Only forward: returns false and changes
  // nothing if t is not after the current filter time.
  bool predictTo(double t, double w)
  {
    const double dt = t - t_;
    if (!initialized_ || !(dt > 0.0)) {return false;}

    StateCov F = StateCov::Identity();
    F.block<3, 3>(kPx, kVx) = Eigen::Matrix3d::Identity() * dt;
    F(kYaw, kBias) = -dt;

    x_.segment<3>(kPx) += x_.segment<3>(kVx) * dt;
    x_(kYaw) = wrapAngle(x_(kYaw) + (w - x_(kBias)) * dt);

    P_ = F * P_ * F.transpose() + processNoiseMatrix(dt);
    P_ = 0.5 * (P_ + P_.transpose());
    t_ = t;
    return true;
  }

  // Discrete Q(dt) from the continuous densities.
  // Per axis, white acceleration with density q:  [q dt^3/3  q dt^2/2; q dt^2/2  q dt] on (p, v).
  // (yaw, b) with yaw' = -b + n_gyro, b' = n_bias:
  //   [qg dt + qb dt^3/3   -qb dt^2/2;  -qb dt^2/2   qb dt]
  StateCov processNoiseMatrix(double dt) const
  {
    StateCov Q = StateCov::Zero();
    const double dt2 = dt * dt, dt3 = dt2 * dt;
    for (int i = 0; i < 3; ++i) {
      const double q = (i < 2) ? q_.acc_xy : q_.acc_z;
      Q(kPx + i, kPx + i) = q * dt3 / 3.0;
      Q(kPx + i, kVx + i) = Q(kVx + i, kPx + i) = q * dt2 / 2.0;
      Q(kVx + i, kVx + i) = q * dt;
    }
    Q(kYaw, kYaw) = q_.gyro * dt + q_.bias * dt3 / 3.0;
    Q(kYaw, kBias) = Q(kBias, kYaw) = -q_.bias * dt2 / 2.0;
    Q(kBias, kBias) = q_.bias * dt;
    return Q;
  }

  // General update for an M-dimensional measurement z with prediction hx = h(x), Jacobian H
  // and noise R. Rows flagged in angle_rows are wrapped. The result is committed only if the
  // new state and covariance are finite.
  template<int M>
  UpdateResult<M> update(
    const Eigen::Matrix<double, M, 1> & z, const Eigen::Matrix<double, M, 1> & hx,
    const Eigen::Matrix<double, M, kNx> & H, const Eigen::Matrix<double, M, M> & R,
    const std::array<bool, M> & angle_rows = {})
  {
    UpdateResult<M> res;
    if (!initialized_) {return res;}

    Eigen::Matrix<double, M, 1> r = z - hx;
    for (int i = 0; i < M; ++i) {
      if (angle_rows[i]) {r(i) = wrapAngle(r(i));}
    }
    const Eigen::Matrix<double, M, M> S = H * P_ * H.transpose() + R;
    const Eigen::LDLT<Eigen::Matrix<double, M, M>> S_ldlt(S);
    // K^T = S^-1 H P  (S and P are symmetric), so K = (S \ (H P))^T
    const Eigen::Matrix<double, M, kNx> Kt = S_ldlt.solve(H * P_);
    const Eigen::Matrix<double, kNx, M> K = Kt.transpose();
    const Eigen::Matrix<double, M, 1> S_inv_r = S_ldlt.solve(r);

    res.innovation = r;
    res.S = S;
    res.nis = r.dot(S_inv_r);

    StateVec x_new = x_ + K * r;
    x_new(kYaw) = wrapAngle(x_new(kYaw));
    const StateCov IKH = StateCov::Identity() - K * H;
    StateCov P_new = IKH * P_ * IKH.transpose() + K * R * K.transpose();
    P_new = 0.5 * (P_new + P_new.transpose());

    if (!x_new.allFinite() || !P_new.allFinite() || !std::isfinite(res.nis)) {return res;}
    x_ = x_new;
    P_ = P_new;
    res.applied = true;
    return res;
  }

  // GPS: z = [px, py, pz], H = [I3 0], linear.
  UpdateResult<3> updatePosition(const Eigen::Vector3d & z, const Eigen::Matrix3d & R)
  {
    Eigen::Matrix<double, 3, kNx> H = Eigen::Matrix<double, 3, kNx>::Zero();
    H.block<3, 3>(0, kPx) = Eigen::Matrix3d::Identity();
    return update<3>(z, x_.segment<3>(kPx), H, R);
  }

private:
  bool initialized_{false};
  double t_{0.0};
  StateVec x_{StateVec::Zero()};
  StateCov P_{StateCov::Identity()};
  ProcessNoise q_;
};

}  // namespace lrs_localization
