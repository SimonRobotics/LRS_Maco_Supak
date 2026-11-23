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
// to its own header stamp and the estimate is published. A GPS fix first predicts the state
// forward to the fix's stamp, then corrects it. Nothing waits for the sensors to line up.
//
// Threading: all callbacks share the filter state, so they must run one at a time. The node
// is spun by a SingleThreadedExecutor, which guarantees that without any locks.

#include <cmath>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "lrs_localization/ekf.hpp"
#include "lrs_localization/sensor_channel.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

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
    const double report_period = declare_parameter("report_period_s", 5.0);

    // --- filter parameters
    lrs_localization::ProcessNoise q;
    q.acc_xy = declare_parameter("q_acc_xy", q.acc_xy);
    q.acc_z = declare_parameter("q_acc_z", q.acc_z);
    q.gyro = declare_parameter("q_gyro", q.gyro);
    q.bias = declare_parameter("q_bias", q.bias);
    q_ = q;
    ekf_.setProcessNoise(q_);
    init_vel_std_ = declare_parameter("init_vel_std", 1.0);
    init_yaw_std_ = declare_parameter("init_yaw_std", 0.1);
    init_bias_std_ = declare_parameter("init_bias_std", 0.05);
    gps_std_fallback_ = declare_parameter("gps_std_fallback", std::vector<double>{0.6, 0.6, 0.9});
    if (gps_std_fallback_.size() != 3) {throw std::runtime_error("gps_std_fallback needs 3 values");}
    max_lag_ = declare_parameter("max_meas_lag_s", 0.1);

    odom_frame_ = declare_parameter("odom_frame", std::string("map"));
    base_frame_ = declare_parameter("base_frame", std::string("base_link"));
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(
      declare_parameter("odom_topic", std::string("/lrs_localization/odom")), 10);

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
    imu_wz_sum_ += m.angular_velocity.z;
    ++imu_wz_n_;
    last_wz_ = m.angular_velocity.z;
    gyro_var_ = m.angular_velocity_covariance[8] > 0.0 ? m.angular_velocity_covariance[8] : gyro_var_;
    if (ekf_.initialized() && ekf_.predictTo(t, last_wz_)) {publish();}
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
    if (!bringTo(gps_, t)) {return;}
    const auto res = ekf_.updatePosition(z, R);
    if (!res.applied) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "[gps] update produced non-finite values, skipped");
      return;
    }
    gps_nis_sum_ += res.nis;
    ++gps_nis_n_;
  }

  // VO is not fused yet; its yaw is only used to initialise the filter's heading.
  void onVo(const nav_msgs::msg::Odometry & m)
  {
    if (!accept(vo_, toSec(m.header.stamp), m.header.frame_id, m.child_frame_id)) {return;}
    vo_yaw_ = quatYaw(m.pose.pose.orientation);
    have_vo_yaw_ = true;
  }

  // ------------------------------------------------------------ filter helpers

  // Position from the first GPS fix (its own covariance), heading from VO -- the VO frame
  // starts aligned with the true pose. Velocity and gyro bias start at zero with a wide sigma.
  void initialize(double t, const Eigen::Vector3d & z, const Eigen::Matrix3d & R)
  {
    if (!have_vo_yaw_) {
      RCLCPP_INFO_ONCE(get_logger(), "[filter] GPS ready, waiting for VO heading to initialise");
      return;
    }
    StateVec x0 = StateVec::Zero();
    x0.segment<3>(idx::kPx) = z;
    x0(idx::kYaw) = vo_yaw_;
    StateCov P0 = StateCov::Zero();
    P0.block<3, 3>(idx::kPx, idx::kPx) = R;
    P0.block<3, 3>(idx::kVx, idx::kVx) = Eigen::Matrix3d::Identity() * init_vel_std_ * init_vel_std_;
    P0(idx::kYaw, idx::kYaw) = init_yaw_std_ * init_yaw_std_;
    P0(idx::kBias, idx::kBias) = init_bias_std_ * init_bias_std_;
    ekf_.initialize(x0, P0, t);
    RCLCPP_INFO(
      get_logger(), "[filter] initialised at t=%.3f s: p=(%.2f, %.2f, %.2f) yaw=%.1f deg",
      t, z.x(), z.y(), z.z(), x0(idx::kYaw) * 180.0 / M_PI);
    publish();
  }

  // Bring the filter to a measurement's stamp t. Newer than the filter: predict forward with
  // the latest gyro rate. Slightly older (it arrived after a newer IMU sample): apply it at
  // the filter time -- the error is |v| * lag, a few mm. Older than max_lag: drop it.
  bool bringTo(const SensorChannel & ch, double t)
  {
    if (t > ekf_.time()) {
      ekf_.predictTo(t, last_wz_);
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
    o.twist.twist.angular.z = last_wz_ - x(idx::kBias);
    for (int r = 0; r < 3; ++r) {
      for (int k = 0; k < 3; ++k) {o.twist.covariance[r * 6 + k] = Cb(r, k);}
    }
    o.twist.covariance[3 * 6 + 3] = o.twist.covariance[4 * 6 + 4] = 1e6;
    o.twist.covariance[5 * 6 + 5] = gyro_var_ + P(idx::kBias, idx::kBias);
    odom_pub_->publish(o);
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
      "sigma xy %.2f m, z %.2f m, yaw %.1f deg | GPS updates %zu, mean NIS %.2f (expect 3)",
      ekf_.time(), x(idx::kPx), x(idx::kPy), x(idx::kPz), x(idx::kVx), x(idx::kVy), x(idx::kVz),
      x(idx::kYaw) * 180.0 / M_PI, x(idx::kBias),
      std::sqrt(0.5 * (P(idx::kPx, idx::kPx) + P(idx::kPy, idx::kPy))), std::sqrt(P(idx::kPz, idx::kPz)),
      std::sqrt(P(idx::kYaw, idx::kYaw)) * 180.0 / M_PI,
      gps_nis_n_, gps_nis_n_ ? gps_nis_sum_ / gps_nis_n_ : 0.0);
    gps_nis_sum_ = 0.0;
    gps_nis_n_ = 0;
  }

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
  double init_vel_std_, init_yaw_std_, init_bias_std_, max_lag_;
  std::vector<double> gps_std_fallback_;
  std::size_t late_dropped_{0};
  double gps_nis_sum_{0.0};
  std::size_t gps_nis_n_{0};
  std::string odom_frame_, base_frame_;

  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr gps_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr vo_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::TimerBase::SharedPtr report_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LocalizationNode>());
  rclcpp::shutdown();
  return 0;
}
