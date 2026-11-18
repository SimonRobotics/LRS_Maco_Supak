// Assignment 2 — drone localisation, estimator node.
//
// Inputs (the only ones the estimator may use, see assignments/assignment2/localization.md):
//   /localization/gps  PoseWithCovarianceStamped  5 Hz   position in `map` (ENU)
//   /localization/vo   Odometry                  30 Hz   pose in `vo_odom`, twist in `base_link`
//   /localization/imu  Imu                      100 Hz   only angular_velocity.z is valid
//
// §1 Sensor acquisition: every message is checked (frame ids, header stamp) by its
// SensorChannel before it is used, and the node periodically prints the simulation-time rate
// of each sensor. Rates are computed from header stamps, not from arrival times, so they read
// 5 / 30 / 100 Hz even when Gazebo runs slower than real time.
//
// Threading: all callbacks share the filter state, so they must run one at a time. The node
// is spun by a SingleThreadedExecutor, which guarantees that without any locks.

#include <cmath>
#include <memory>
#include <sstream>
#include <string>

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "lrs_localization/sensor_channel.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

using lrs_localization::SensorChannel;
using lrs_localization::StampCheck;

namespace
{

double toSec(const builtin_interfaces::msg::Time & t)
{
  return static_cast<double>(t.sec) + static_cast<double>(t.nanosec) * 1e-9;
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
      get_logger(), "waiting for %s, %s, %s (use_sim_time=%s)",
      gps_sub_->get_topic_name(), vo_sub_->get_topic_name(), imu_sub_->get_topic_name(),
      get_parameter("use_sim_time").as_bool() ? "true" : "false");
  }

private:
  // ------------------------------------------------------------ input callbacks

  void onGps(const geometry_msgs::msg::PoseWithCovarianceStamped & m)
  {
    if (!accept(gps_, toSec(m.header.stamp), m.header.frame_id)) {return;}
  }

  void onVo(const nav_msgs::msg::Odometry & m)
  {
    if (!accept(vo_, toSec(m.header.stamp), m.header.frame_id, m.child_frame_id)) {return;}
  }

  void onImu(const sensor_msgs::msg::Imu & m)
  {
    if (!accept(imu_, toSec(m.header.stamp), m.header.frame_id)) {return;}
    imu_wz_sum_ += m.angular_velocity.z;
    ++imu_wz_n_;
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
        RCLCPP_WARN(get_logger(), "[%s] %s, stamp %.3f s", ch.name().c_str(), toString(c), t);
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
      imu_.duplicates() + imu_.outOfOrder() + imu_.zeros();
    s << " | dropped " << dropped << ", wrong frame " << frame_errors_;
    RCLCPP_INFO(get_logger(), "[sensors] %s", s.str().c_str());
  }

  SensorChannel gps_, vo_, imu_;
  double imu_wz_sum_{0.0};
  std::size_t imu_wz_n_{0};
  std::size_t frame_errors_{0};

  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr gps_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr vo_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::TimerBase::SharedPtr report_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LocalizationNode>());
  rclcpp::shutdown();
  return 0;
}
