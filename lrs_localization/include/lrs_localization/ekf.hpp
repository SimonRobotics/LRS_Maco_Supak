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
// Measurement update, z = h(x) + noise with covariance R (H = dh/dx; for the linear GPS, yaw
// and bias updates H is constant, for the body-velocity update it is the EKF Jacobian):
//   r = z - h(x)            innovation, angle rows wrapped
//   S = H P H^T + R         its expected covariance
//   K = P H^T S^-1          computed by solving with S, never by inverting it
//   x <- x + K r
//   P <- (I - K H) P (I - K H)^T + K R K^T     Joseph form: stays symmetric and positive
// NIS = r^T S^-1 r is returned with every update (consistency check and the outlier gate).
//
// Innovation gate (A2 §3). If the measurement is right and the filter is consistent, r is
// zero-mean Gaussian with covariance S, so NIS = r^T S^-1 r is chi-square distributed with M
// degrees of freedom. NIS is computed before the gain; if NIS > gate.nis the measurement is
// so unlikely under that hypothesis that it is rejected and the filter is left unchanged
// (gate 16.27 for M = 3: a correct measurement exceeds it with probability 0.1 %). Because S
// contains H P H^T, the gate widens automatically while P grows (e.g. during a GPS outage) --
// unlike a fixed "distance from the estimate" rule.
// With gate.inflate the measurement is applied anyway, but with R scaled by alpha >= 1 chosen
// so that r^T (H P H^T + alpha R)^-1 r = gate.nis: the filter moves towards it only as far as
// a borderline-acceptable measurement would move it. That is the recovery path when the
// filter itself is wrong and keeps rejecting good data.
#pragma once

#include <Eigen/Dense>

#include <algorithm>
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
  double yaw{1e-5};     // yaw random walk [rad^2/s]: gyro noise with margin
  double bias{1e-8};    // gyro bias random walk [rad^2/s^3]
};

// Innovation gate for one update; the default never rejects.
struct Gate
{
  double nis{std::numeric_limits<double>::infinity()};  // reject if NIS > nis
  bool inflate{false};  // instead of rejecting: apply with R inflated so that NIS = nis
};

// Probability that a chi-square variable with 1, 2 or 3 degrees of freedom exceeds x, i.e. how
// likely a correct measurement is to have an NIS this large (closed forms, no table needed).
inline double chi2Survival(double x, int dof)
{
  if (!(x > 0.0)) {return 1.0;}
  switch (dof) {
    case 1: return std::erfc(std::sqrt(0.5 * x));
    case 2: return std::exp(-0.5 * x);
    case 3: return std::erfc(std::sqrt(0.5 * x)) + std::sqrt(2.0 * x / M_PI) * std::exp(-0.5 * x);
    default: return std::numeric_limits<double>::quiet_NaN();
  }
}

template<int M>
struct UpdateResult
{
  bool applied{false};
  bool rejected{false};  // NIS above the gate, nothing changed
  double r_scale{1.0};   // alpha > 1: applied with R * alpha (gate.inflate)
  Eigen::Matrix<double, M, 1> innovation{Eigen::Matrix<double, M, 1>::Zero()};
  Eigen::Matrix<double, M, M> S{Eigen::Matrix<double, M, M>::Zero()};  // with the original R
  double nis{0.0};  // r^T S^-1 r with the original R, before the update
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
  // extra_q_yaw [rad^2/s] is added to the yaw noise density for this step: without a gyro
  // (IMU timeout) the node predicts with w = b, i.e. "not turning", and this is how unsure
  // that guess is.
  bool predictTo(double t, double w, double extra_q_yaw = 0.0)
  {
    const double dt = t - t_;
    if (!initialized_ || !(dt > 0.0)) {return false;}

    StateCov F = StateCov::Identity();
    F.block<3, 3>(kPx, kVx) = Eigen::Matrix3d::Identity() * dt;
    F(kYaw, kBias) = -dt;

    x_.segment<3>(kPx) += x_.segment<3>(kVx) * dt;
    x_(kYaw) = wrapAngle(x_(kYaw) + (w - x_(kBias)) * dt);

    P_ = F * P_ * F.transpose() + processNoiseMatrix(dt);
    P_(kYaw, kYaw) += std::max(0.0, extra_q_yaw) * dt;
    P_ = 0.5 * (P_ + P_.transpose());
    t_ = t;
    return true;
  }

  // Discrete Q(dt) from the continuous densities.
  // Per axis, white acceleration with density q:  [q dt^3/3  q dt^2/2; q dt^2/2  q dt] on (p, v).
  // (yaw, b) with yaw' = -b + n_yaw, b' = n_bias:
  //   [qy dt + qb dt^3/3   -qb dt^2/2;  -qb dt^2/2   qb dt]
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
    Q(kYaw, kYaw) = q_.yaw * dt + q_.bias * dt3 / 3.0;
    Q(kYaw, kBias) = Q(kBias, kYaw) = -q_.bias * dt2 / 2.0;
    Q(kBias, kBias) = q_.bias * dt;
    return Q;
  }

  // General update for an M-dimensional measurement z with prediction hx = h(x), Jacobian H
  // and noise R. Rows flagged in angle_rows are wrapped. The innovation is gated first (see
  // Gate); the result is committed only if the new state and covariance are finite.
  template<int M>
  UpdateResult<M> update(
    const Eigen::Matrix<double, M, 1> & z, const Eigen::Matrix<double, M, 1> & hx,
    const Eigen::Matrix<double, M, kNx> & H, const Eigen::Matrix<double, M, M> & R_meas,
    const std::array<bool, M> & angle_rows = {}, const Gate & gate = {})
  {
    using MatM = Eigen::Matrix<double, M, M>;
    UpdateResult<M> res;
    if (!initialized_) {return res;}

    Eigen::Matrix<double, M, 1> r = z - hx;
    for (int i = 0; i < M; ++i) {
      if (angle_rows[i]) {r(i) = wrapAngle(r(i));}
    }
    const MatM HPHt = H * P_ * H.transpose();
    const auto nisWith = [&](const MatM & S) {return r.dot(Eigen::LDLT<MatM>(S).solve(r));};

    // Gate on the NIS with the measurement's own R -- before any gain is computed.
    res.innovation = r;
    res.S = HPHt + R_meas;
    res.nis = nisWith(res.S);
    if (!std::isfinite(res.nis)) {return res;}
    MatM R = R_meas;
    if (res.nis > gate.nis) {
      if (!gate.inflate) {
        res.rejected = true;
        return res;
      }
      // NIS(alpha) = r^T (HPH^T + alpha R)^-1 r falls monotonically with alpha; find
      // NIS(alpha) = gate by bisection on log(alpha). M <= 3, so this costs nothing.
      double lo = 1.0, hi = 2.0;
      while (nisWith(HPHt + hi * R_meas) > gate.nis && hi < 1e12) {lo = hi; hi *= 4.0;}
      for (int i = 0; i < 60; ++i) {
        const double mid = std::sqrt(lo * hi);
        (nisWith(HPHt + mid * R_meas) > gate.nis ? lo : hi) = mid;
      }
      res.r_scale = hi;
      R = hi * R_meas;
    }

    const MatM S = HPHt + R;
    const Eigen::LDLT<MatM> S_ldlt(S);
    // K^T = S^-1 H P  (S and P are symmetric), so K = (S \ (H P))^T
    const Eigen::Matrix<double, M, kNx> HP = H * P_;
    const Eigen::Matrix<double, kNx, M> K = S_ldlt.solve(HP).transpose();

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
  UpdateResult<3> updatePosition(
    const Eigen::Vector3d & z, const Eigen::Matrix3d & R, const Gate & gate = {})
  {
    Eigen::Matrix<double, 3, kNx> H = Eigen::Matrix<double, 3, kNx>::Zero();
    H.block<3, 3>(0, kPx) = Eigen::Matrix3d::Identity();
    return update<3>(z, x_.segment<3>(kPx), H, R, {}, gate);
  }

  // VO linear twist = velocity in the body frame: z = Rz(-yaw) v, nonlinear in (v, yaw).
  //   h(x) = [ c vx + s vy,  -s vx + c vy,  vz ]        c = cos(yaw), s = sin(yaw)
  //   H = dh/dx:  row 1: d/dvx = c,  d/dvy = s,  d/dyaw = -s vx + c vy
  //               row 2: d/dvx = -s, d/dvy = c,  d/dyaw = -c vx - s vy
  //               row 3: d/dvz = 1
  // The yaw column is what makes the heading observable from motion: GPS fixes the velocity
  // in ENU, VO measures the same velocity in the body frame, and the rotation between the two
  // is the yaw. When the drone does not move, that column is zero -- no heading information.
  UpdateResult<3> updateBodyVelocity(
    const Eigen::Vector3d & z, const Eigen::Matrix3d & R, const Gate & gate = {})
  {
    return update<3>(z, bodyVelocity(x_), bodyVelocityJacobian(x_), R, {}, gate);
  }

  static Eigen::Vector3d bodyVelocity(const StateVec & x)
  {
    const double c = std::cos(x(kYaw)), s = std::sin(x(kYaw));
    return {c * x(kVx) + s * x(kVy), -s * x(kVx) + c * x(kVy), x(kVz)};
  }

  static Eigen::Matrix<double, 3, kNx> bodyVelocityJacobian(const StateVec & x)
  {
    const double c = std::cos(x(kYaw)), s = std::sin(x(kYaw));
    Eigen::Matrix<double, 3, kNx> H = Eigen::Matrix<double, 3, kNx>::Zero();
    H(0, kVx) = c;
    H(0, kVy) = s;
    H(0, kYaw) = -s * x(kVx) + c * x(kVy);
    H(1, kVx) = -s;
    H(1, kVy) = c;
    H(1, kYaw) = -c * x(kVx) - s * x(kVy);
    H(2, kVz) = 1.0;
    return H;
  }

  // ENU velocity directly: z = v, H = [0 I3 0 0], linear. Used as a zero-velocity update --
  // while the drone stands still its true velocity is 0, which also cancels the VO velocity bias.
  UpdateResult<3> updateVelocity(const Eigen::Vector3d & z, const Eigen::Matrix3d & R)
  {
    Eigen::Matrix<double, 3, kNx> H = Eigen::Matrix<double, 3, kNx>::Zero();
    H.block<3, 3>(0, kVx) = Eigen::Matrix3d::Identity();
    return update<3>(z, x_.segment<3>(kVx), H, R);
  }

  // Gyro bias directly: z = b, H = e_b. Used as a zero-rate update -- while the drone stands
  // still its true yaw rate is 0, so the mean gyro reading over a window IS the bias.
  UpdateResult<1> updateBias(double z, double var)
  {
    Eigen::Matrix<double, 1, kNx> H = Eigen::Matrix<double, 1, kNx>::Zero();
    H(0, kBias) = 1.0;
    return update<1>(
      Eigen::Matrix<double, 1, 1>(z), Eigen::Matrix<double, 1, 1>(x_(kBias)), H,
      Eigen::Matrix<double, 1, 1>(var));
  }

  // Heading (VO yaw): z = yaw, H = e_yaw, linear; the innovation is wrapped, so a measurement
  // of +179 deg against an estimate of -179 deg is a 2 deg correction, not 358 deg.
  // This is also what makes the gyro bias observable: the prediction integrates (w - b), so a
  // wrong b makes the predicted yaw walk away from the measured one, and the (yaw, b)
  // covariance built up by F(yaw, b) = -dt turns that innovation into a bias correction.
  UpdateResult<1> updateYaw(double z, double var)
  {
    Eigen::Matrix<double, 1, kNx> H = Eigen::Matrix<double, 1, kNx>::Zero();
    H(0, kYaw) = 1.0;
    return update<1>(
      Eigen::Matrix<double, 1, 1>(z), Eigen::Matrix<double, 1, 1>(x_(kYaw)), H,
      Eigen::Matrix<double, 1, 1>(var), {true});
  }

private:
  bool initialized_{false};
  double t_{0.0};
  StateVec x_{StateVec::Zero()};
  StateCov P_{StateCov::Identity()};
  ProcessNoise q_;
};

}  // namespace lrs_localization
