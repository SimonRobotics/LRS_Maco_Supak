#include <gtest/gtest.h>

#include <random>

#include "lrs_localization/ekf.hpp"

using namespace lrs_localization;

namespace
{

Ekf8 makeFilter(const StateVec & x0, double pos_var = 1.0, double t0 = 10.0)
{
  StateCov P0 = StateCov::Zero();
  P0.diagonal() << pos_var, pos_var, pos_var, 1.0, 1.0, 1.0, 0.01, 0.0025;
  Ekf8 f;
  f.initialize(x0, P0, t0);
  return f;
}

bool symmetricPositive(const StateCov & P)
{
  return (P - P.transpose()).cwiseAbs().maxCoeff() < 1e-12 &&
         Eigen::LLT<StateCov>(P).info() == Eigen::Success;
}

}  // namespace

TEST(Ekf8, WrapAngleIntoMinusPiPi)
{
  EXPECT_DOUBLE_EQ(wrapAngle(0.1), 0.1);
  EXPECT_DOUBLE_EQ(wrapAngle(M_PI), -M_PI);       // [-pi, pi): +pi maps to -pi
  EXPECT_DOUBLE_EQ(wrapAngle(-M_PI), -M_PI);
  EXPECT_NEAR(wrapAngle(1.5 * M_PI), -0.5 * M_PI, 1e-12);
  EXPECT_NEAR(wrapAngle(-7.0), -7.0 + 2 * M_PI, 1e-12);
}

TEST(Ekf8, PredictConstantVelocity)
{
  StateVec x0 = StateVec::Zero();
  x0 << 1, 2, 3, 0.5, -1.0, 0.2, 0, 0;
  Ekf8 f = makeFilter(x0);
  const StateCov P0 = f.P();
  ASSERT_TRUE(f.predictTo(12.0, 0.0));
  EXPECT_NEAR(f.x()(kPx), 2.0, 1e-12);
  EXPECT_NEAR(f.x()(kPy), 0.0, 1e-12);
  EXPECT_NEAR(f.x()(kPz), 3.4, 1e-12);
  EXPECT_NEAR(f.x()(kVx), 0.5, 1e-12);   // velocity unchanged
  EXPECT_GT(f.P()(kPx, kPx), P0(kPx, kPx));  // uncertainty grows without measurements
  EXPECT_GT(f.P()(kPx, kVx), 0.0);           // position and velocity become correlated
  EXPECT_TRUE(symmetricPositive(f.P()));
  EXPECT_DOUBLE_EQ(f.time(), 12.0);
}

TEST(Ekf8, PredictYawWithGyroMinusBiasAndWraps)
{
  StateVec x0 = StateVec::Zero();
  x0(kYaw) = 3.0;
  x0(kBias) = 0.1;
  Ekf8 f = makeFilter(x0);
  f.predictTo(11.0, 0.6);  // yaw += (0.6 - 0.1) * 1 s = 3.5 rad -> past +pi
  EXPECT_NEAR(f.x()(kYaw), 3.5 - 2 * M_PI, 1e-12);
  EXPECT_DOUBLE_EQ(f.x()(kBias), 0.1);
  EXPECT_LT(f.P()(kYaw, kBias), 0.0);  // yaw error now depends on the bias error
}

TEST(Ekf8, PredictOnlyForwardInTime)
{
  StateVec x0 = StateVec::Zero();
  x0(kVx) = 1.0;
  Ekf8 f = makeFilter(x0);
  EXPECT_FALSE(f.predictTo(10.0, 0.0));  // same time
  EXPECT_FALSE(f.predictTo(9.5, 0.0));   // older
  EXPECT_DOUBLE_EQ(f.x()(kPx), 0.0);
  EXPECT_DOUBLE_EQ(f.time(), 10.0);
}

TEST(Ekf8, FifteenSecondsWithoutMeasurementsStayFinite)
{
  Ekf8 f = makeFilter(StateVec::Zero());
  for (int k = 1; k <= 1500; ++k) {ASSERT_TRUE(f.predictTo(10.0 + 0.01 * k, 0.02));}
  EXPECT_TRUE(f.x().allFinite());
  EXPECT_TRUE(f.P().allFinite());
  EXPECT_TRUE(symmetricPositive(f.P()));
  EXPECT_GT(f.P()(kPx, kPx), 1.0);
}

TEST(Ekf8, GpsUpdateIsTheScalarKalmanGainPerAxis)
{
  Ekf8 f = makeFilter(StateVec::Zero(), 1.0);
  const Eigen::Matrix3d R = Eigen::Vector3d(0.36, 0.36, 0.81).asDiagonal();
  const auto res = f.updatePosition(Eigen::Vector3d(1.36, 0.0, 0.0), R);
  ASSERT_TRUE(res.applied);
  // K = P / (P + R) = 1 / 1.36 for x, so x moves to 1.0 and its variance drops to 0.36 / 1.36.
  EXPECT_NEAR(f.x()(kPx), 1.0, 1e-12);
  EXPECT_NEAR(f.P()(kPx, kPx), 0.36 / 1.36, 1e-12);
  EXPECT_NEAR(res.nis, 1.36 * 1.36 / 1.36, 1e-12);  // r^2 / S
  EXPECT_TRUE(symmetricPositive(f.P()));
}

TEST(Ekf8, NonFiniteMeasurementIsNotApplied)
{
  Ekf8 f = makeFilter(StateVec::Zero());
  const StateVec before = f.x();
  const auto res = f.updatePosition(Eigen::Vector3d(NAN, 0, 0), Eigen::Matrix3d::Identity());
  EXPECT_FALSE(res.applied);
  EXPECT_TRUE(f.x() == before);
}

TEST(Ekf8, YawInnovationIsWrapped)
{
  StateVec x0 = StateVec::Zero();
  x0(kYaw) = -3.1;
  Ekf8 f = makeFilter(x0);  // P(yaw) = 0.01
  // z - yaw = 6.1 rad, but the shortest way is 6.1 - 2pi = -0.18 rad: down through -pi.
  const auto res = f.updateYaw(3.0, 0.01);
  ASSERT_TRUE(res.applied);
  EXPECT_NEAR(res.innovation(0), 6.1 - 2 * M_PI, 1e-12);
  // K = 0.01 / (0.01 + 0.01) = 0.5: half way, across the seam, and wrapped back into [-pi, pi).
  EXPECT_NEAR(f.x()(kYaw), wrapAngle(-3.1 + 0.5 * (6.1 - 2 * M_PI)), 1e-12);
  EXPECT_GT(f.x()(kYaw), 3.0);
  EXPECT_LT(f.x()(kYaw), M_PI);
}

// Gyro with an unknown bias of 0.02 rad/s, a turning drone and yaw measured once per second:
// the bias is not measured directly, but it becomes observable through the yaw innovations.
TEST(Ekf8, GyroBiasBecomesObservableFromYawUpdates)
{
  std::mt19937 rng(2);
  std::normal_distribution<double> n(0.0, 1.0);
  const double b_true = 0.02, yaw_sigma = 0.05;

  StateVec x0 = StateVec::Zero();
  Ekf8 f = makeFilter(x0, 1.0, 0.0);  // bias starts at 0 with sigma 0.05
  // Here the yaw measurement has white noise only (no drift), so the physical gyro noise is
  // the right yaw process noise -- not the inflated value tuned for the drifting VO heading.
  ProcessNoise q;
  q.yaw = 0.004 * 0.004 * 0.01;
  f.setProcessNoise(q);
  double yaw_true = 0.0;
  for (int k = 1; k <= 6000; ++k) {  // 60 s at 100 Hz
    const double t = 0.01 * k;
    const double w_true = 0.3 * std::sin(0.2 * t);   // turning back and forth
    yaw_true = wrapAngle(yaw_true + w_true * 0.01);
    f.predictTo(t, w_true + b_true + 0.004 * n(rng));
    if (k % 100 == 0) {f.updateYaw(wrapAngle(yaw_true + yaw_sigma * n(rng)), yaw_sigma * yaw_sigma);}
  }
  const double sb = std::sqrt(f.P()(kBias, kBias));
  EXPECT_NEAR(f.x()(kBias), b_true, 3.0 * sb);
  EXPECT_LT(sb, 0.002);  // from 0.05 at the start
  EXPECT_LT(std::abs(wrapAngle(f.x()(kYaw) - yaw_true)), 3.0 * std::sqrt(f.P()(kYaw, kYaw)));
}

TEST(Ekf8, BodyVelocityJacobianMatchesNumericalDerivative)
{
  StateVec x = StateVec::Zero();
  x << 1, 2, 3, 0.4, -0.3, 0.1, 2.5, 0.01;
  const auto H = Ekf8::bodyVelocityJacobian(x);
  for (int j = 0; j < kNx; ++j) {
    StateVec xp = x, xm = x;
    xp(j) += 1e-6;
    xm(j) -= 1e-6;
    const Eigen::Vector3d num = (Ekf8::bodyVelocity(xp) - Ekf8::bodyVelocity(xm)) / 2e-6;
    for (int i = 0; i < 3; ++i) {EXPECT_NEAR(H(i, j), num(i), 1e-8) << "row " << i << " col " << j;}
  }
}

TEST(Ekf8, BodyVelocityIsRotatedIntoEnu)
{
  StateVec x0 = StateVec::Zero();
  x0(kYaw) = M_PI / 2;  // nose north
  StateCov P0 = StateCov::Identity();
  P0(kYaw, kYaw) = 1e-6;  // heading known, velocity unknown
  Ekf8 f;
  f.initialize(x0, P0, 0.0);
  for (int k = 0; k < 20; ++k) {f.updateBodyVelocity(Eigen::Vector3d(0.5, 0, 0), Eigen::Matrix3d::Identity() * 1e-4);}
  // forward 0.5 m/s with the nose north = 0.5 m/s north in ENU
  EXPECT_NEAR(f.x()(kVx), 0.0, 1e-3);
  EXPECT_NEAR(f.x()(kVy), 0.5, 1e-3);
}

// The drone flies a rectangle (forward 0.5 m/s, 90 deg turns in place) and the filter starts
// with a 20 deg heading error. Nothing measures yaw directly: GPS gives the track in ENU, VO the
// velocity in the body frame, and the EKF velocity Jacobian turns their disagreement into a
// heading correction while the drone moves.
TEST(Ekf8, HeadingIsObservableFromMotion)
{
  std::mt19937 rng(3);
  std::normal_distribution<double> n(0.0, 1.0);
  const double dt = 0.01, b_true = -0.017;

  StateVec x0 = StateVec::Zero();
  x0(kYaw) = M_PI / 2 + 20.0 * M_PI / 180.0;  // wrong by 20 deg
  x0(kBias) = b_true;                         // known, as after zero-rate updates at rest
  StateCov P0 = StateCov::Zero();
  P0.diagonal() << 0.36, 0.36, 0.81, 0.01, 0.01, 0.01, std::pow(0.4, 2), 1e-8;
  Ekf8 f;
  f.initialize(x0, P0, 0.0);
  ProcessNoise q;
  q.yaw = 1e-6;
  f.setProcessNoise(q);

  Eigen::Vector3d p(0, 0, 0);
  double yaw = M_PI / 2;
  int k = 0;
  for (int lap = 0; lap < 2; ++lap) {
    for (int leg = 0; leg < 4; ++leg) {
      for (int phase = 0; phase < 2; ++phase) {  // 8 s forward, then 2 s turning in place
        const int steps = phase == 0 ? 800 : 200;
        for (int i = 0; i < steps; ++i, ++k) {
          const double speed = phase == 0 ? 0.5 : 0.0, rate = phase == 0 ? 0.0 : (M_PI / 2) / 2.0;
          yaw = wrapAngle(yaw + rate * dt);
          p += Eigen::Vector3d(std::cos(yaw), std::sin(yaw), 0) * speed * dt;
          const double t = (k + 1) * dt;
          f.predictTo(t, rate + b_true + 0.004 * n(rng));
          if ((k + 1) % 3 == 0) {  // ~30 Hz VO body velocity
            f.updateBodyVelocity(
              Eigen::Vector3d(speed + 0.03 * n(rng), 0.03 * n(rng), 0.03 * n(rng)),
              Eigen::Matrix3d::Identity() * 0.0009);
          }
          if ((k + 1) % 20 == 0) {  // 5 Hz GPS
            f.updatePosition(
              p + Eigen::Vector3d(0.6 * n(rng), 0.6 * n(rng), 0.9 * n(rng)),
              Eigen::Vector3d(0.36, 0.36, 0.81).asDiagonal());
          }
        }
      }
    }
  }
  const double err = wrapAngle(f.x()(kYaw) - yaw);
  const double sy = std::sqrt(f.P()(kYaw, kYaw));
  EXPECT_LT(std::abs(err), 3.0 * sy);
  EXPECT_LT(std::abs(err), 3.0 * M_PI / 180.0);  // from 20 deg
  EXPECT_LT(sy, 0.4);                             // and the filter knows it learned something
}

TEST(Ekf8, ZeroVelocityUpdatePullsVelocityToZero)
{
  StateVec x0 = StateVec::Zero();
  x0.segment<3>(kVx) << 0.05, -0.03, 0.04;  // a creeping estimate, as from a biased VO velocity
  Ekf8 f = makeFilter(x0);                  // velocity sigma 1 m/s
  const auto res = f.updateVelocity(Eigen::Vector3d::Zero(), Eigen::Matrix3d::Identity() * 0.02 * 0.02);
  ASSERT_TRUE(res.applied);
  EXPECT_LT(f.x().segment<3>(kVx).norm(), 0.001);
  EXPECT_LT(std::sqrt(f.P()(kVx, kVx)), 0.021);
}

// Standing still, the true yaw rate is zero, so the mean gyro rate over a window measures the
// bias directly -- independent of any drifting heading reference.
TEST(Ekf8, ZeroRateUpdateEstimatesBias)
{
  std::mt19937 rng(4);
  std::normal_distribution<double> n(0.0, 1.0);
  const double b_true = 0.023;
  Ekf8 f = makeFilter(StateVec::Zero(), 1.0, 0.0);  // b = 0, sigma 0.05
  for (int s = 1; s <= 30; ++s) {
    double sum = 0.0;
    for (int i = 1; i <= 100; ++i) {
      const double w = b_true + 0.004 * n(rng);
      f.predictTo(s - 1 + 0.01 * i, w);
      sum += w;
    }
    f.updateBias(sum / 100.0, 0.004 * 0.004 / 100.0 + 0.001 * 0.001);
  }
  const double sb = std::sqrt(f.P()(kBias, kBias));
  EXPECT_NEAR(f.x()(kBias), b_true, 3.0 * sb);
  EXPECT_LT(sb, 0.001);
}

// Noisy 5 Hz GPS with the simulator's sigmas, 100 Hz prediction: the estimate must converge,
// follow a constant velocity within its own 3 sigma, and the mean NIS of a consistent filter
// is the measurement dimension (3). GPS alone determines velocity only loosely (sigma ~0.3 m/s
// here), which is why a fixed tolerance would be the wrong test.
TEST(Ekf8, TracksNoisyGpsAndIsConsistent)
{
  std::mt19937 rng(1);
  std::normal_distribution<double> n(0.0, 1.0);
  const Eigen::Vector3d sigma(0.6, 0.6, 0.9);
  const Eigen::Matrix3d R = sigma.cwiseAbs2().asDiagonal();

  StateVec x0 = StateVec::Zero();
  x0.segment<3>(kPx) << -7.0, 13.0, 0.0;
  Ekf8 f = makeFilter(x0, 0.81, 0.0);
  ProcessNoise q;
  q.acc_xy = 0.05;
  q.acc_z = 0.05;
  f.setProcessNoise(q);

  const Eigen::Vector3d v_true(0.5, 0.0, 0.0);
  double nis_sum = 0.0;
  int nis_n = 0;
  for (int k = 1; k <= 6000; ++k) {  // 60 s
    const double t = 0.01 * k;
    f.predictTo(t, 0.0);
    if (k % 20 == 0) {  // 5 Hz
      const Eigen::Vector3d p_true = x0.segment<3>(kPx) + v_true * t;
      const Eigen::Vector3d z = p_true + Eigen::Vector3d(sigma(0) * n(rng), sigma(1) * n(rng), sigma(2) * n(rng));
      const auto res = f.updatePosition(z, R);
      ASSERT_TRUE(res.applied);
      if (t > 10.0) {nis_sum += res.nis; ++nis_n;}
    }
  }
  const Eigen::Vector3d p_true = x0.segment<3>(kPx) + v_true * 60.0;
  for (int i = 0; i < 3; ++i) {
    EXPECT_LT(std::abs(f.x()(kPx + i) - p_true(i)), 3.0 * std::sqrt(f.P()(kPx + i, kPx + i))) << "p" << i;
    EXPECT_LT(std::abs(f.x()(kVx + i) - v_true(i)), 3.0 * std::sqrt(f.P()(kVx + i, kVx + i))) << "v" << i;
  }
  EXPECT_LT(std::sqrt(f.P()(kPx, kPx)), 0.6);  // better than one raw GPS fix
  EXPECT_NEAR(nis_sum / nis_n, 3.0, 0.6);
  EXPECT_TRUE(symmetricPositive(f.P()));
}
