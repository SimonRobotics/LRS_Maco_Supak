// Assignment 2 — drone localisation, estimator node.
//
// Inputs (the only ones the estimator may use, see assignments/assignment2/localization.md):
//   /localization/gps  PoseWithCovarianceStamped  5 Hz   position in `map` (ENU)
//   /localization/vo   Odometry                  30 Hz   pose in `vo_odom`, twist in `base_link`
//   /localization/imu  Imu                      100 Hz   only angular_velocity.z is valid
// Output:
//   /lrs_localization/odom  Odometry, frame `map`, child `base_link`: x, y, z, yaw with
//   covariance from P; twist = velocity in `base_link` (REP 105) and bias-corrected yaw rate.
//
// §1 Sensor acquisition: every message is checked (frame ids, header stamp) by its
// SensorChannel before it is used, and the node periodically prints the simulation-time rate
// of each sensor.
//
// §2 Filter (ekf.hpp): the IMU drives the prediction -- every gyro sample propagates the state
// to its own header stamp and the estimate is published. A GPS fix or a VO sample first
// predicts the state forward to its stamp, then corrects it. Nothing waits for the sensors to
// line up. Updates:
//   GPS position                      linear, R from the message
//   VO body velocity (10 Hz)          EKF, h = Rz(-yaw) v -- couples yaw, heading from motion
//   zero-rate / zero-velocity (1 Hz)  only while standing still: b = mean gyro, v = 0
//   VO yaw                            off by default (it drifts); vo_yaw_mode absolute to compare
//
// §3 Missing and bad measurements:
//   - GPS and VO velocity pass an innovation gate (NIS = r^T S^-1 r against a chi-square
//     threshold, ekf.hpp). A rejected GPS fix is logged with its innovation, NIS and the
//     probability that a correct fix lies that far out. After gps_max_rejections rejections in
//     a row the filter itself is presumed wrong (rejection_guard.hpp): fixes are then applied
//     with R inflated so that NIS = gate -- a bounded step, never a jump onto one raw
//     measurement -- until as many fixes in a row pass the gate again.
//   - Each sensor has a timeout in header-stamp time (GPS 1 s, VO 0.5 s, IMU 0.2 s). TIMEOUT
//     and RECOVERED transitions are logged once each; on GPS return the log shows the outage
//     length, the sigma before the fix, its innovation and whether it was accepted.
//   - Without the IMU the filter predicts with w = b (no turn) and a larger yaw noise, and the
//     estimate is published after GPS/VO messages instead -- the output does not stop.
//   Nothing is ever reset because of an outage; prediction simply carries the state.
//
// Threading: all callbacks share the filter state, so they must run one at a time. The node
// is spun by a SingleThreadedExecutor, which guarantees that without any locks.

#include <algorithm>
#include <cmath>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "lrs_localization/ekf.hpp"
#include "lrs_localization/rejection_guard.hpp"
#include "lrs_localization/sensor_channel.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

using lrs_localization::Ekf8;
using lrs_localization::SensorChannel;
using lrs_localization::StampCheck;
using lrs_localization::StateCov;
using lrs_localization::StateVec;
namespace idx = lrs_localization;

namespace
{

double toSec(const builtin_interfaces::msg::Time & t)
{
  return static_cast<double>(t.sec) + static_cast<double>(t.nanosec) * 1e-9;
}

builtin_interfaces::msg::Time toStamp(double t)
{
  builtin_interfaces::msg::Time s;
  double sec = std::floor(t);
  int64_t ns = std::llround((t - sec) * 1e9);
  if (ns >= 1000000000) {sec += 1.0; ns -= 1000000000;}
  s.sec = static_cast<int32_t>(sec);
  s.nanosec = static_cast<uint32_t>(ns);
  return s;
}

double quatYaw(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

const char * toString(StampCheck c)
{
  switch (c) {
    case StampCheck::kOk: return "ok";
    case StampCheck::kZero: return "zero stamp";
    case StampCheck::kDuplicate: return "duplicate stamp";
    case StampCheck::kOutOfOrder: return "out-of-order stamp";
    case StampCheck::kReset: return "time jumped back (simulation reset)";
  }
  return "?";
}

}  // namespace

class LocalizationNode : public rclcpp::Node
{
public:
  LocalizationNode()
  : Node("lrs_localization"),
    gps_("gps", declare_parameter("gps_frame", std::string("map"))),
    vo_("vo", declare_parameter("vo_frame", std::string("vo_odom")),
      declare_parameter("vo_child_frame", std::string("base_link"))),
    imu_("imu", declare_parameter("imu_frame", std::string("base_link")))
  {
    const double reset_jump = declare_parameter("reset_jump_s", 1.0);
    for (auto * ch : {&gps_, &vo_, &imu_}) {ch->setResetJump(reset_jump);}
    gps_.setTimeout(declare_parameter("gps_timeout_s", 1.0));
    vo_.setTimeout(declare_parameter("vo_timeout_s", 0.5));
    imu_.setTimeout(declare_parameter("imu_timeout_s", 0.2));
    const double report_period = declare_parameter("report_period_s", 5.0);

    // --- filter parameters
    lrs_localization::ProcessNoise q;
    q.acc_xy = declare_parameter("q_acc_xy", q.acc_xy);
    q.acc_z = declare_parameter("q_acc_z", q.acc_z);
    q.yaw = declare_parameter("q_yaw", q.yaw);
    q.bias = declare_parameter("q_bias", q.bias);
    q_ = q;
    ekf_.setProcessNoise(q_);
    init_vel_std_ = declare_parameter("init_vel_std", 1.0);
    init_yaw_source_ = declare_parameter("init_yaw_source", std::string("spawn"));  // spawn | vo
    if (init_yaw_source_ != "spawn" && init_yaw_source_ != "vo") {
      throw std::runtime_error("init_yaw_source must be spawn or vo");
    }
    initial_yaw_ = declare_parameter("initial_yaw_deg", 90.0) * M_PI / 180.0;
    init_yaw_std_ = declare_parameter("init_yaw_std", 0.05);
    init_bias_std_ = declare_parameter("init_bias_std", 0.05);
    gps_std_fallback_ = declare_parameter("gps_std_fallback", std::vector<double>{0.6, 0.6, 0.9});
    if (gps_std_fallback_.size() != 3) {throw std::runtime_error("gps_std_fallback needs 3 values");}
    max_lag_ = declare_parameter("max_meas_lag_s", 0.1);
    vo_vel_std_ = declare_parameter("vo_vel_std", 0.1);  // <= 0: covariance from the message
    vo_vel_period_ = declare_parameter("vo_vel_period_s", 0.1);
    vo_yaw_mode_ = declare_parameter("vo_yaw_mode", std::string("off"));  // off | absolute
    if (vo_yaw_mode_ != "off" && vo_yaw_mode_ != "absolute") {
      throw std::runtime_error("vo_yaw_mode must be off or absolute");
    }
    const double vo_yaw_std = declare_parameter("vo_yaw_std_deg", 3.0) * M_PI / 180.0;
    vo_yaw_var_ = vo_yaw_std * vo_yaw_std;
    vo_yaw_period_ = declare_parameter("vo_yaw_period_s", 1.0);
    zaru_enabled_ = declare_parameter("zaru_enabled", true);
    zaru_window_ = declare_parameter("zaru_window_s", 1.0);
    zaru_max_speed_ = declare_parameter("zaru_max_speed", 0.1);
    zaru_max_rate_ = declare_parameter("zaru_max_rate", 0.05);
    zaru_max_rate_std_ = declare_parameter("zaru_max_rate_std", 0.02);
    zaru_slack_ = declare_parameter("zaru_slack", 0.001);
    zupt_enabled_ = declare_parameter("zupt_enabled", true);
    zupt_std_ = declare_parameter("zupt_std", 0.02);
    // §3 gates and outage handling
    gps_gate_nis_ = declare_parameter("gps_gate_nis", 16.27);
    gps_guard_ = lrs_localization::RejectionGuard(declare_parameter("gps_max_rejections", 10));
    vo_gate_nis_ = declare_parameter("vo_gate_nis", 16.27);
    vo_guard_ = lrs_localization::RejectionGuard(declare_parameter("vo_max_rejections", 10));
    const double imu_lost_rate_std = declare_parameter("imu_lost_yaw_rate_std", 1.0);
    imu_lost_q_yaw_ = imu_lost_rate_std * imu_lost_rate_std;

    odom_frame_ = declare_parameter("odom_frame", std::string("map"));
    base_frame_ = declare_parameter("base_frame", std::string("base_link"));
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(
      declare_parameter("odom_topic", std::string("/lrs_localization/odom")), 10);
    // Debug/plotting only: [t, x(8), sigma(8)] -- the full state incl. the gyro bias, which
    // the Odometry message has no field for.
    state_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      declare_parameter("state_topic", std::string("/lrs_localization/state")), 10);

    // The sensor simulator publishes best effort; a reliable subscriber would get nothing.
    const auto qos = rclcpp::SensorDataQoS();
    gps_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      declare_parameter("gps_topic", std::string("/localization/gps")), qos,
      [this](geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr m) {onGps(*m);});
    vo_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      declare_parameter("vo_topic", std::string("/localization/vo")), qos,
      [this](nav_msgs::msg::Odometry::ConstSharedPtr m) {onVo(*m);});
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      declare_parameter("imu_topic", std::string("/localization/imu")), qos,
      [this](sensor_msgs::msg::Imu::ConstSharedPtr m) {onImu(*m);});

    // Wall timer on purpose: if /clock or a sensor stops, the report still appears and says so.
    report_timer_ = create_wall_timer(
      std::chrono::duration<double>(report_period), [this] {report();});

    RCLCPP_INFO(
      get_logger(), "waiting for %s, %s, %s (use_sim_time=%s); estimate on %s",
      gps_sub_->get_topic_name(), vo_sub_->get_topic_name(), imu_sub_->get_topic_name(),
      get_parameter("use_sim_time").as_bool() ? "true" : "false", odom_pub_->get_topic_name());
  }

private:
  // ------------------------------------------------------------ input callbacks

  // The gyro drives the prediction: propagate to this sample's stamp, then publish.
  void onImu(const sensor_msgs::msg::Imu & m)
  {
    const double t = toSec(m.header.stamp);
    if (!accept(imu_, t, m.header.frame_id)) {return;}
    if (const double gap = imu_.takeRecoveredGap(); gap > 0.0) {
      RCLCPP_INFO(
        get_logger(), "[imu] RECOVERED after %.2f s without gyro: yaw sigma %.1f deg -- gyro prediction resumes",
        gap, ekf_.initialized() ? std::sqrt(ekf_.P()(idx::kYaw, idx::kYaw)) * 180.0 / M_PI : 0.0);
      zaru_n_ = 0;  // the stillness window would span the outage
      zaru_sum_ = zaru_sumsq_ = 0.0;
      zaru_vo_sum_.setZero();
      zaru_vo_n_ = 0;
    }
    imu_wz_sum_ += m.angular_velocity.z;
    ++imu_wz_n_;
    last_wz_ = m.angular_velocity.z;
    gyro_var_ = m.angular_velocity_covariance[8] > 0.0 ? m.angular_velocity_covariance[8] : gyro_var_;
    if (!ekf_.initialized()) {return;}
    if (ekf_.predictTo(t, last_wz_)) {publish();}
    zeroRateWindow(t, last_wz_);
  }

  // Zero-rate update (ZARU). Over each window of zaru_window_s the node collects the gyro rate
  // and the VO body velocity. If the drone stood still for the whole window -- VO speed small,
  // gyro mean close to the bias estimate and no oscillation -- its true yaw rate was zero, so
  // the mean gyro rate is a direct measurement of the bias: z = mean(w), H = e_b,
  // R = gyro_var / N + zaru_slack^2. This estimates the bias from the gyro itself, not from the
  // drifting VO heading, and keeps the yaw from walking away while the drone waits.
  void zeroRateWindow(double t, double w)
  {
    if (zaru_n_ == 0) {zaru_t0_ = t;}
    zaru_sum_ += w;
    zaru_sumsq_ += w * w;
    ++zaru_n_;
    if (t - zaru_t0_ < zaru_window_) {return;}

    const double mean = zaru_sum_ / static_cast<double>(zaru_n_);
    const double std = std::sqrt(std::max(0.0, zaru_sumsq_ / static_cast<double>(zaru_n_) - mean * mean));
    const double speed = zaru_vo_n_ ? (zaru_vo_sum_ / static_cast<double>(zaru_vo_n_)).norm() : 1e9;
    const bool still = zaru_vo_n_ >= 5 && speed < zaru_max_speed_ &&
      std::abs(mean - ekf_.x()(idx::kBias)) < zaru_max_rate_ && std < zaru_max_rate_std_;
    if (still != stationary_) {
      stationary_ = still;
      RCLCPP_INFO(
        get_logger(), "[zaru] %s (VO speed %.3f m/s, gyro mean %+.4f std %.4f rad/s)",
        still ? "standing still: zero-rate bias updates ON" : "moving or turning: zero-rate updates OFF",
        speed, mean, std);
    }
    if (still && zaru_enabled_) {
      const auto res = ekf_.updateBias(mean, gyro_var_ / static_cast<double>(zaru_n_) + zaru_slack_ * zaru_slack_);
      if (res.applied) {zaru_nis_sum_ += res.nis; ++zaru_nis_n_;}
    }
    // Zero-velocity update (ZUPT), same detection: standing still means v = 0. Without it the
    // slowly wandering VO velocity bias makes the filter believe the drone creeps (z off by
    // 0.4 m after minutes on the ground, far outside its 2 sigma).
    if (still && zupt_enabled_) {
      const auto res = ekf_.updateVelocity(Eigen::Vector3d::Zero(), Eigen::Matrix3d::Identity() * zupt_std_ * zupt_std_);
      if (res.applied) {zupt_nis_sum_ += res.nis; ++zupt_nis_n_;}
    }
    zaru_n_ = 0;
    zaru_sum_ = zaru_sumsq_ = 0.0;
    zaru_vo_sum_.setZero();
    zaru_vo_n_ = 0;
  }

  void onGps(const geometry_msgs::msg::PoseWithCovarianceStamped & m)
  {
    const double t = toSec(m.header.stamp);
    if (!accept(gps_, t, m.header.frame_id)) {return;}
    const auto & p = m.pose.pose.position;
    const Eigen::Vector3d z(p.x, p.y, p.z);
    const Eigen::Matrix3d R = gpsCovariance(m);

    if (!ekf_.initialized()) {
      initialize(t, z, R);
      return;
    }
    const double outage = gps_.takeRecoveredGap();
    if (!bringTo(gps_, t)) {return;}
    const StateCov P_before = ekf_.P();

    lrs_localization::Gate gate;
    gate.nis = gps_gate_nis_;
    gate.inflate = gps_guard_.recovering();
    const auto res = ekf_.updatePosition(z, R, gate);
    const Eigen::Vector3d r = res.innovation;

    if (outage > 0.0) {
      RCLCPP_INFO(
        get_logger(),
        "[gps] RECOVERED after %.1f s outage: sigma before the fix xy %.2f m, z %.2f m | innovation "
        "(%+.2f, %+.2f, %+.2f) m, |r| %.2f m, NIS %.2f (gate %.2f) -> %s",
        outage, std::sqrt(0.5 * (P_before(idx::kPx, idx::kPx) + P_before(idx::kPy, idx::kPy))),
        std::sqrt(P_before(idx::kPz, idx::kPz)), r.x(), r.y(), r.z(), r.norm(), res.nis, gps_gate_nis_,
        res.rejected ? "REJECTED (see below)" : (res.applied ? "accepted" : "not applied (non-finite)"));
    }

    if (res.rejected) {
      const auto ev = gps_guard_.onRejected();
      ++gps_rejected_total_;
      const Eigen::Vector3d sig = res.S.diagonal().cwiseSqrt();
      RCLCPP_WARN(
        get_logger(),
        "[gps] REJECTED fix at t=%.2f: innovation (%+.2f, %+.2f, %+.2f) m, |r| %.2f m; expected "
        "innovation sigma (%.2f, %.2f, %.2f) m -> NIS %.1f > gate %.2f (chi-square, 3 dof). A correct "
        "fix is this far out with probability %.1e, so it is treated as an outlier; filter unchanged "
        "[%d in a row]",
        t, r.x(), r.y(), r.z(), r.norm(), sig.x(), sig.y(), sig.z(), res.nis, gps_gate_nis_,
        lrs_localization::chi2Survival(res.nis, 3), gps_guard_.rejectedInRow());
      if (ev == lrs_localization::RejectionGuard::Event::kEnterRecovery) {
        // The GPS agrees with itself but not with the filter, so the filter is the one that is
        // wrong: from now on fixes outside the gate are applied with R * alpha so NIS = gate,
        // and the estimate walks towards GPS over several fixes.
        RCLCPP_WARN(
          get_logger(),
          "[gps] %d fixes rejected in a row -- the filter is presumed wrong, not the GPS: RECOVERY, "
          "next fixes are applied with R inflated so that NIS = gate", gps_guard_.rejectedInRow());
      }
      return;
    }
    if (!res.applied) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "[gps] update produced non-finite values, skipped");
      return;
    }
    if (res.r_scale > 1.0) {
      if (gps_guard_.inflatedThisEpisode() == 0) {
        RCLCPP_WARN(
          get_logger(), "[gps] recovery: first inflated fix, R x %.1f (NIS was %.1f, |r| %.2f m)",
          res.r_scale, res.nis, r.norm());
      }
      ++gps_inflated_total_;
    }
    if (gps_guard_.onApplied(res.r_scale > 1.0) == lrs_localization::RejectionGuard::Event::kLeaveRecovery) {
      RCLCPP_INFO(
        get_logger(), "[gps] recovery done: fixes pass the gate on their own again, after %zu inflated updates",
        gps_guard_.inflatedThisEpisode());
    }
    gps_nis_sum_ += res.nis;
    ++gps_nis_n_;
    if (imu_.timedOut()) {publish();}
  }

  // VO: the linear twist (velocity in base_link) is fused with the EKF body-velocity update.
  // The VO pose is not used -- it drifts without bound (characterisation: 0.8 m/min at rest,
  // yaw 113 deg off after an hour) and its covariance (1 cm) does not say so.
  // vo_yaw_mode "absolute" re-enables the step-4 heading update from VO yaw, for comparison.
  void onVo(const nav_msgs::msg::Odometry & m)
  {
    const double t = toSec(m.header.stamp);
    if (!accept(vo_, t, m.header.frame_id, m.child_frame_id)) {return;}
    const double outage = vo_.takeRecoveredGap();
    vo_yaw_ = quatYaw(m.pose.pose.orientation);
    have_vo_yaw_ = true;
    const auto & l = m.twist.twist.linear;
    const Eigen::Vector3d vb(l.x, l.y, l.z);
    zaru_vo_sum_ += vb;
    ++zaru_vo_n_;
    if (!ekf_.initialized()) {return;}

    if ((outage > 0.0 || t - last_vo_vel_t_ >= vo_vel_period_) && bringTo(vo_, t)) {
      lrs_localization::Gate gate;
      gate.nis = vo_gate_nis_;
      gate.inflate = vo_guard_.recovering();
      const auto res = ekf_.updateBodyVelocity(vb, voVelocityCovariance(m), gate);
      if (outage > 0.0) {
        RCLCPP_INFO(
          get_logger(),
          "[vo] RECOVERED after %.1f s tracking loss: velocity innovation |r| %.3f m/s, NIS %.2f "
          "(gate %.2f) -> %s",
          outage, res.innovation.norm(), res.nis, vo_gate_nis_,
          res.rejected ? "rejected" : (res.applied ? "accepted" : "not applied"));
      }
      if (res.rejected) {
        // Up to 10 per second, so logged throttled; the count is in the [filter] report.
        const auto ev = vo_guard_.onRejected();
        ++vo_rejected_total_;
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "[vo] velocity REJECTED: innovation |r| %.3f m/s, NIS %.1f > gate %.2f (p = %.1e) [%d in a row]",
          res.innovation.norm(), res.nis, vo_gate_nis_, lrs_localization::chi2Survival(res.nis, 3),
          vo_guard_.rejectedInRow());
        if (ev == lrs_localization::RejectionGuard::Event::kEnterRecovery) {
          RCLCPP_WARN(
            get_logger(), "[vo] %d velocities rejected in a row -- RECOVERY: applied with R inflated so that NIS = gate",
            vo_guard_.rejectedInRow());
        }
        last_vo_vel_t_ = t;  // try again one period later, not on the next 30 Hz sample
      } else if (res.applied) {
        if (vo_guard_.onApplied(res.r_scale > 1.0) == lrs_localization::RejectionGuard::Event::kLeaveRecovery) {
          RCLCPP_INFO(
            get_logger(), "[vo] recovery done after %zu inflated updates", vo_guard_.inflatedThisEpisode());
        }
        last_vo_vel_t_ = t;
        vo_vel_nis_sum_ += res.nis;
        ++vo_vel_nis_n_;
      } else {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "[vo] velocity update produced non-finite values, skipped");
      }
    }

    if (vo_yaw_mode_ == "absolute" && t - last_vo_yaw_t_ >= vo_yaw_period_ && bringTo(vo_, t)) {
      const auto res = ekf_.updateYaw(vo_yaw_, vo_yaw_var_);
      if (res.applied) {
        last_vo_yaw_t_ = t;
        vo_yaw_nis_sum_ += res.nis;
        ++vo_yaw_nis_n_;
      }
    }
    // Without the gyro, VO (30 Hz) drives the output instead.
    if (imu_.timedOut()) {
      if (t > ekf_.time()) {bringTo(vo_, t);}
      publish();
    }
  }

  // VO velocity noise: vo_vel_std if set, otherwise the message's twist covariance.
  Eigen::Matrix3d voVelocityCovariance(const nav_msgs::msg::Odometry & m) const
  {
    if (vo_vel_std_ > 0.0) {return Eigen::Matrix3d::Identity() * vo_vel_std_ * vo_vel_std_;}
    Eigen::Matrix3d R;
    for (int r = 0; r < 3; ++r) {
      for (int c = 0; c < 3; ++c) {R(r, c) = m.twist.covariance[r * 6 + c];}
    }
    if (R.allFinite() && R.diagonal().minCoeff() > 0.0) {return 0.5 * (R + R.transpose());}
    return Eigen::Matrix3d::Identity() * 0.03 * 0.03;
  }

  // ------------------------------------------------------------ filter helpers

  // Position from the first GPS fix (its own covariance). Heading: the known spawn yaw
  // (init_yaw_source "spawn", the assignment's initial ENU yaw pi/2) -- or the current VO yaw
  // ("vo"), which is right only if VO started recently, since it drifts. Velocity and gyro
  // bias start at zero with a wide sigma.
  void initialize(double t, const Eigen::Vector3d & z, const Eigen::Matrix3d & R)
  {
    if (init_yaw_source_ == "vo" && !have_vo_yaw_) {
      RCLCPP_INFO_ONCE(get_logger(), "[filter] GPS ready, waiting for VO heading to initialise");
      return;
    }
    StateVec x0 = StateVec::Zero();
    x0.segment<3>(idx::kPx) = z;
    x0(idx::kYaw) = init_yaw_source_ == "vo" ? vo_yaw_ : initial_yaw_;
    StateCov P0 = StateCov::Zero();
    P0.block<3, 3>(idx::kPx, idx::kPx) = R;
    P0.block<3, 3>(idx::kVx, idx::kVx) = Eigen::Matrix3d::Identity() * init_vel_std_ * init_vel_std_;
    P0(idx::kYaw, idx::kYaw) = init_yaw_std_ * init_yaw_std_;
    P0(idx::kBias, idx::kBias) = init_bias_std_ * init_bias_std_;
    ekf_.initialize(x0, P0, t);
    RCLCPP_INFO(
      get_logger(), "[filter] initialised at t=%.3f s: p=(%.2f, %.2f, %.2f) yaw=%.1f deg (%s)",
      t, z.x(), z.y(), z.z(), x0(idx::kYaw) * 180.0 / M_PI, init_yaw_source_.c_str());
    publish();
  }

  // Bring the filter to a measurement's stamp t. Newer than the filter: predict forward with
  // the latest gyro rate. Slightly older (it arrived after a newer IMU sample): apply it at
  // the filter time -- the error is |v| * lag, a few mm. Older than max_lag: drop it.
  bool bringTo(const SensorChannel & ch, double t)
  {
    if (t > ekf_.time()) {
      ekf_.predictTo(t, gyroRate(), imu_.timedOut() ? imu_lost_q_yaw_ : 0.0);
      return true;
    }
    const double lag = ekf_.time() - t;
    if (lag <= max_lag_) {return true;}
    ++late_dropped_;
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000, "[%s] measurement %.3f s older than the filter, dropped",
      ch.name().c_str(), lag);
    return false;
  }

  // Gyro rate for the prediction. Without the IMU: w = b, i.e. assume the drone does not turn
  // (yaw stays, its uncertainty grows by imu_lost_yaw_rate_std^2 per second).
  double gyroRate() const {return imu_.timedOut() ? ekf_.x()(idx::kBias) : last_wz_;}

  // Called with every accepted stamp: the newest stamp of any sensor is "now" in simulation
  // time; a channel silent for longer than its timeout is reported once.
  void checkTimeouts(double t)
  {
    newest_stamp_ = std::max(newest_stamp_, t);
    for (auto * ch : {&gps_, &vo_, &imu_}) {
      if (!ch->checkTimeout(newest_stamp_)) {continue;}
      const char * consequence =
        ch == &gps_ ? "position carried by prediction + VO velocity, sigma grows" :
        ch == &vo_ ? "velocity carried by GPS + motion model" :
        "predicting with w = b (no turn), yaw sigma grows; output published on GPS/VO";
      RCLCPP_WARN(
        get_logger(), "[%s] TIMEOUT: no message since t=%.2f (%.2f s > %.2f s) -- %s",
        ch->name().c_str(), ch->lastStamp(), newest_stamp_ - ch->lastStamp(), ch->timeout(), consequence);
    }
  }

  // GPS covariance from the message if it is usable, otherwise the configured sigmas.
  Eigen::Matrix3d gpsCovariance(const geometry_msgs::msg::PoseWithCovarianceStamped & m) const
  {
    Eigen::Matrix3d R;
    for (int r = 0; r < 3; ++r) {
      for (int c = 0; c < 3; ++c) {R(r, c) = m.pose.covariance[r * 6 + c];}
    }
    if (R.allFinite() && R.diagonal().minCoeff() > 0.0) {return 0.5 * (R + R.transpose());}
    return Eigen::Vector3d(
      gps_std_fallback_[0] * gps_std_fallback_[0], gps_std_fallback_[1] * gps_std_fallback_[1],
      gps_std_fallback_[2] * gps_std_fallback_[2]).asDiagonal();
  }

  void publish()
  {
    const StateVec & x = ekf_.x();
    const StateCov & P = ekf_.P();
    nav_msgs::msg::Odometry o;
    o.header.stamp = toStamp(ekf_.time());
    o.header.frame_id = odom_frame_;
    o.child_frame_id = base_frame_;

    const double yaw = x(idx::kYaw);
    o.pose.pose.position.x = x(idx::kPx);
    o.pose.pose.position.y = x(idx::kPy);
    o.pose.pose.position.z = x(idx::kPz);
    o.pose.pose.orientation.z = std::sin(0.5 * yaw);
    o.pose.pose.orientation.w = std::cos(0.5 * yaw);
    // 6x6 row-major over (x, y, z, roll, pitch, yaw); roll and pitch are not estimated.
    const int pose_idx[6] = {idx::kPx, idx::kPy, idx::kPz, -1, -1, idx::kYaw};
    for (int r = 0; r < 6; ++r) {
      for (int c = 0; c < 6; ++c) {
        const bool est = pose_idx[r] >= 0 && pose_idx[c] >= 0;
        o.pose.covariance[r * 6 + c] = est ? P(pose_idx[r], pose_idx[c]) : (r == c ? 1e6 : 0.0);
      }
    }

    // Velocity in base_link: v_b = Rz(-yaw) v_enu, covariance rotated the same way.
    const double c = std::cos(yaw), s = std::sin(yaw);
    Eigen::Matrix3d Rbe;
    Rbe << c, s, 0, -s, c, 0, 0, 0, 1;
    const Eigen::Vector3d vb = Rbe * x.segment<3>(idx::kVx);
    const Eigen::Matrix3d Cb = Rbe * P.block<3, 3>(idx::kVx, idx::kVx) * Rbe.transpose();
    o.twist.twist.linear.x = vb.x();
    o.twist.twist.linear.y = vb.y();
    o.twist.twist.linear.z = vb.z();
    o.twist.twist.angular.z = gyroRate() - x(idx::kBias);
    for (int r = 0; r < 3; ++r) {
      for (int k = 0; k < 3; ++k) {o.twist.covariance[r * 6 + k] = Cb(r, k);}
    }
    o.twist.covariance[3 * 6 + 3] = o.twist.covariance[4 * 6 + 4] = 1e6;
    o.twist.covariance[5 * 6 + 5] = imu_.timedOut() ? imu_lost_q_yaw_ : gyro_var_ + P(idx::kBias, idx::kBias);
    odom_pub_->publish(o);

    std_msgs::msg::Float64MultiArray st;
    st.data.reserve(1 + 2 * lrs_localization::kNx);
    st.data.push_back(ekf_.time());
    for (int i = 0; i < lrs_localization::kNx; ++i) {st.data.push_back(x(i));}
    for (int i = 0; i < lrs_localization::kNx; ++i) {st.data.push_back(std::sqrt(P(i, i)));}
    state_pub_->publish(st);
  }

  // Frame and stamp check shared by all three inputs. Logs at most once per problem kind
  // (frames) or throttled (stamps) -- never at the sensor rate.
  bool accept(SensorChannel & ch, double t, const std::string & frame, const std::string & child = "")
  {
    if (!ch.frameOk(frame, child)) {
      ++frame_errors_;
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "[%s] wrong frame: got '%s'%s%s, expected '%s'%s%s -- message ignored",
        ch.name().c_str(), frame.c_str(), child.empty() ? "" : " -> ", child.c_str(),
        ch.expectedFrame().c_str(), ch.expectedChild().empty() ? "" : " -> ",
        ch.expectedChild().c_str());
      return false;
    }
    const bool first = !ch.hasData();
    const StampCheck c = ch.check(t);
    switch (c) {
      case StampCheck::kOk:
        checkTimeouts(t);
        if (first) {
          RCLCPP_INFO(
            get_logger(), "[%s] first message: frame '%s'%s%s, stamp %.3f s", ch.name().c_str(),
            frame.c_str(), child.empty() ? "" : " -> ", child.c_str(), t);
        }
        return true;
      case StampCheck::kReset:
        // Only a restarted simulation moves time back by seconds. The old state belongs to
        // another timeline, so start over from the next GPS fix (this is not used for bad
        // measurements -- those are dropped or, later, gated).
        RCLCPP_WARN(
          get_logger(), "[%s] %s, stamp %.3f s -- filter re-initialises", ch.name().c_str(),
          toString(c), t);
        ekf_ = Ekf8();
        ekf_.setProcessNoise(q_);
        have_vo_yaw_ = false;
        last_vo_yaw_t_ = last_vo_vel_t_ = -1e9;
        zaru_n_ = zaru_vo_n_ = 0;
        zaru_sum_ = zaru_sumsq_ = 0.0;
        zaru_vo_sum_.setZero();
        stationary_ = false;
        newest_stamp_ = t;
        gps_guard_.reset();
        vo_guard_.reset();
        return true;
      default:
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000, "[%s] %s %.3f s (last %.3f s) -- dropped",
          ch.name().c_str(), toString(c), t, ch.lastStamp());
        return false;
    }
  }

  // ------------------------------------------------------------ periodic report

  void report()
  {
    std::ostringstream s;
    s.setf(std::ios::fixed);
    for (auto * ch : {&gps_, &vo_, &imu_}) {
      const auto w = ch->takeWindow();
      s.precision(1);
      s << ch->name() << " ";
      if (w.count < 2) {
        s << "NO DATA";
      } else {
        s << w.rate_hz << " Hz";
        s.precision(2);
        s << " (gap " << w.max_gap_s << " s)";
      }
      s << " | ";
    }
    s.precision(4);
    if (imu_wz_n_ > 0) {s << "mean wz " << std::showpos << imu_wz_sum_ / imu_wz_n_ << std::noshowpos << " rad/s";}
    imu_wz_sum_ = 0.0;
    imu_wz_n_ = 0;

    const std::size_t dropped = gps_.duplicates() + gps_.outOfOrder() + gps_.zeros() +
      vo_.duplicates() + vo_.outOfOrder() + vo_.zeros() +
      imu_.duplicates() + imu_.outOfOrder() + imu_.zeros() + late_dropped_;
    s << " | dropped " << dropped << ", wrong frame " << frame_errors_;
    RCLCPP_INFO(get_logger(), "[sensors] %s", s.str().c_str());

    if (!ekf_.initialized()) {return;}
    const StateVec & x = ekf_.x();
    const StateCov & P = ekf_.P();
    RCLCPP_INFO(
      get_logger(),
      "[filter] t=%.1f p=(%.2f, %.2f, %.2f) v=(%.2f, %.2f, %.2f) yaw=%.1f deg b=%+.4f rad/s | "
      "sigma xy %.2f m, z %.2f m, yaw %.1f deg, b %.4f | %s | updates (mean NIS / expected): "
      "GPS %zu (%.2f/3), VO vel %zu (%.2f/3), ZARU %zu (%.2f/1), ZUPT %zu (%.2f/3), VO yaw %zu (%.2f/1) | "
      "gated so far: GPS %zu rejected, %zu inflated; VO %zu rejected%s%s%s%s",
      ekf_.time(), x(idx::kPx), x(idx::kPy), x(idx::kPz), x(idx::kVx), x(idx::kVy), x(idx::kVz),
      x(idx::kYaw) * 180.0 / M_PI, x(idx::kBias),
      std::sqrt(0.5 * (P(idx::kPx, idx::kPx) + P(idx::kPy, idx::kPy))), std::sqrt(P(idx::kPz, idx::kPz)),
      std::sqrt(P(idx::kYaw, idx::kYaw)) * 180.0 / M_PI, std::sqrt(P(idx::kBias, idx::kBias)),
      stationary_ ? "still" : "moving",
      gps_nis_n_, mean(gps_nis_sum_, gps_nis_n_), vo_vel_nis_n_, mean(vo_vel_nis_sum_, vo_vel_nis_n_),
      zaru_nis_n_, mean(zaru_nis_sum_, zaru_nis_n_), zupt_nis_n_, mean(zupt_nis_sum_, zupt_nis_n_),
      vo_yaw_nis_n_, mean(vo_yaw_nis_sum_, vo_yaw_nis_n_),
      gps_rejected_total_, gps_inflated_total_, vo_rejected_total_,
      gps_.timedOut() ? " | GPS TIMED OUT" : "", vo_.timedOut() ? " | VO TIMED OUT" : "",
      imu_.timedOut() ? " | IMU TIMED OUT" : "", gps_guard_.recovering() ? " | GPS RECOVERY" : "");
    gps_nis_sum_ = vo_vel_nis_sum_ = zaru_nis_sum_ = zupt_nis_sum_ = vo_yaw_nis_sum_ = 0.0;
    gps_nis_n_ = vo_vel_nis_n_ = zaru_nis_n_ = zupt_nis_n_ = vo_yaw_nis_n_ = 0;
  }

  static double mean(double sum, std::size_t n) {return n ? sum / static_cast<double>(n) : 0.0;}

  SensorChannel gps_, vo_, imu_;
  double imu_wz_sum_{0.0};
  std::size_t imu_wz_n_{0};
  std::size_t frame_errors_{0};

  Ekf8 ekf_;
  lrs_localization::ProcessNoise q_;
  double last_wz_{0.0};
  double gyro_var_{1.6e-5};
  double vo_yaw_{0.0};
  bool have_vo_yaw_{false};
  double init_vel_std_, init_yaw_std_, init_bias_std_, max_lag_, initial_yaw_;
  std::string init_yaw_source_;
  std::vector<double> gps_std_fallback_;
  std::size_t late_dropped_{0};
  double gps_nis_sum_{0.0};
  std::size_t gps_nis_n_{0};
  // VO velocity
  double vo_vel_std_{0.0}, vo_vel_period_{0.0}, last_vo_vel_t_{-1e9};
  double vo_vel_nis_sum_{0.0};
  std::size_t vo_vel_nis_n_{0};
  // VO yaw (only with vo_yaw_mode absolute)
  std::string vo_yaw_mode_;
  double vo_yaw_var_{0.0}, vo_yaw_period_{1.0}, last_vo_yaw_t_{-1e9};
  double vo_yaw_nis_sum_{0.0};
  std::size_t vo_yaw_nis_n_{0};
  // zero-rate update
  bool zaru_enabled_{true}, stationary_{false};
  double zaru_window_{1.0}, zaru_max_speed_{0.1}, zaru_max_rate_{0.05}, zaru_max_rate_std_{0.02}, zaru_slack_{0.001};
  double zaru_t0_{0.0}, zaru_sum_{0.0}, zaru_sumsq_{0.0};
  std::size_t zaru_n_{0};
  Eigen::Vector3d zaru_vo_sum_{Eigen::Vector3d::Zero()};
  std::size_t zaru_vo_n_{0};
  double zaru_nis_sum_{0.0};
  std::size_t zaru_nis_n_{0};
  bool zupt_enabled_{true};
  double zupt_std_{0.05};
  double zupt_nis_sum_{0.0};
  std::size_t zupt_nis_n_{0};
  std::string odom_frame_, base_frame_;
  // §3
  double newest_stamp_{0.0};
  double gps_gate_nis_{16.27}, vo_gate_nis_{16.27}, imu_lost_q_yaw_{1.0};
  lrs_localization::RejectionGuard gps_guard_, vo_guard_;
  std::size_t gps_rejected_total_{0}, gps_inflated_total_{0}, vo_rejected_total_{0};

  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr gps_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr vo_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr state_pub_;
  rclcpp::TimerBase::SharedPtr report_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LocalizationNode>());
  rclcpp::shutdown();
  return 0;
}
