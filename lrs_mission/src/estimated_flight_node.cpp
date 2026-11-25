// Assignment 2 §4 -- flight on the estimated localisation.
//
// Feedback: ONLY /lrs_localization/odom (our Kalman filter, frame `map` = MAVROS local ENU).
// Also used: /mavros/state (connected, armed, mode), the MAVROS services set_mode, arming,
// takeoff, land, and the setpoint output /mavros/setpoint_raw/local. Nothing else is
// subscribed -- in particular no /mavros/local_position/*, /mavros/global_position/*,
// /mavros/odometry/*, /mavros/imu/* and no ground truth. This is a separate node (not a mode of
// the A1 lrs_mission_node, which always subscribes to the MAVROS pose), so `ros2 node info`
// shows the complete and short list of what drives the drone.
//
// Mission (localization.md §4): origin = first valid estimate; take off to `altitude`; fly
// `laps` laps through `route` (ENU offsets from the origin) at `speed`; return to the origin
// and land.
//
// Controller -- outer loop on our estimate, ArduPilot only tracks a velocity:
//   reference r(t) slides along the current segment A -> B with a trapezoidal speed profile:
//   up to `speed` at `ref_accel`, and down to 0 at B (sim time); v_ff = its velocity.
//   (A reference that stopped dead at a corner made the drone, ~1 s behind it, overshoot the
//   corner by 0.5 m; braking the reference itself removes that.)
//   v_cmd = v_ff + kp * (r - p_est)           horizontal, |v_cmd| <= max_speed
//   vz    = kp_z * (z_ref - z_est)            |vz| <= max_speed_z
//   v_cmd changes by at most max_accel * dt  (the estimate can step ~0.5 m per GPS fix while VO is
//                                             lost; this keeps such a step from becoming a jerk)
// sent as an ENU velocity + fixed yaw. With position setpoints ArduPilot would close the loop on
// its OWN position estimate; with velocity setpoints the position loop is ours.
// A waypoint counts as reached when r(t) = B and our estimate is within `reach_tolerance` of B.
//
// Safety: no estimate for `estimate_timeout_s` -> zero velocity (hover); for
// `estimate_lost_land_s` -> LAND. A mode change by someone else stops the setpoints.

#include <algorithm>
#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <type_traits>
#include <vector>

#include "mavros_msgs/msg/position_target.hpp"
#include "mavros_msgs/msg/state.hpp"
#include "mavros_msgs/srv/command_bool.hpp"
#include "mavros_msgs/srv/command_tol.hpp"
#include "mavros_msgs/srv/set_mode.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{

struct V3
{
  double x{0}, y{0}, z{0};
};
V3 operator+(V3 a, V3 b) {return {a.x + b.x, a.y + b.y, a.z + b.z};}
V3 operator-(V3 a, V3 b) {return {a.x - b.x, a.y - b.y, a.z - b.z};}
V3 operator*(V3 a, double s) {return {a.x * s, a.y * s, a.z * s};}
double hnorm(V3 a) {return std::hypot(a.x, a.y);}

// Scale the horizontal part of v down to at most `max` (keeps the direction).
V3 clampH(V3 v, double max)
{
  const double n = hnorm(v);
  if (n > max) {v.x *= max / n; v.y *= max / n;}
  return v;
}

double stampSec(const builtin_interfaces::msg::Time & t) {return t.sec + t.nanosec * 1e-9;}

}  // namespace

class EstimatedFlight : public rclcpp::Node
{
public:
  EstimatedFlight()
  : Node("lrs_estimated_flight")
  {
    altitude_ = declare_parameter("altitude", 2.0);
    speed_ = declare_parameter("speed", 0.5);
    ref_accel_ = declare_parameter("ref_accel", 0.5);
    laps_ = declare_parameter("laps", 2);
    const auto route = declare_parameter(
      "route", std::vector<double>{0.0, 0.0, 0.0, -4.0, 2.5, -4.0, 2.5, 0.0, 0.0, 0.0});
    if (route.size() < 4 || route.size() % 2) {throw std::runtime_error("route: pairs of ENU offsets [m]");}
    for (size_t i = 0; i + 1 < route.size(); i += 2) {route_.push_back({route[i], route[i + 1], 0.0});}
    kp_ = declare_parameter("kp", 0.8);
    kp_z_ = declare_parameter("kp_z", 0.8);
    max_speed_ = declare_parameter("max_speed", 1.0);
    max_speed_z_ = declare_parameter("max_speed_z", 0.5);
    max_accel_ = declare_parameter("max_accel", 1.0);
    reach_tol_ = declare_parameter("reach_tolerance", 0.25);
    land_tol_ = declare_parameter("land_tolerance", 0.15);
    land_speed_ = declare_parameter("land_max_speed", 0.1);
    land_settle_ = declare_parameter("land_settle_s", 1.0);
    origin_sigma_ = declare_parameter("origin_max_sigma", 0.3);
    origin_wait_ = declare_parameter("origin_settle_s", 3.0);
    est_timeout_ = declare_parameter("estimate_timeout_s", 0.5);
    est_lost_land_ = declare_parameter("estimate_lost_land_s", 3.0);
    climb_timeout_ = declare_parameter("climb_timeout_s", 15.0);
    const double rate = declare_parameter("rate_hz", 20.0);

    state_sub_ = create_subscription<mavros_msgs::msg::State>(
      "/mavros/state", 10, [this](mavros_msgs::msg::State::ConstSharedPtr m) {state_ = *m;});
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      declare_parameter("estimate_topic", std::string("/lrs_localization/odom")), 10,
      [this](nav_msgs::msg::Odometry::ConstSharedPtr m) {onEstimate(*m);});
    sp_pub_ = create_publisher<mavros_msgs::msg::PositionTarget>("/mavros/setpoint_raw/local", 10);
    // Debug/plotting only: where the controller wants the drone to be (frame map).
    ref_pub_ = create_publisher<geometry_msgs::msg::PointStamped>("~/reference", 10);
    mode_cli_ = create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");
    arm_cli_ = create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");
    takeoff_cli_ = create_client<mavros_msgs::srv::CommandTOL>("/mavros/cmd/takeoff");
    land_cli_ = create_client<mavros_msgs::srv::CommandTOL>("/mavros/cmd/land");

    // Wall timer: a steady setpoint stream; all decisions use now() = simulation time.
    timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / rate), [this] {tick();});
    RCLCPP_INFO(
      get_logger(), "feedback: %s only; %d laps of %zu points at %.2f m/s, %.1f m (use_sim_time=%s)",
      odom_sub_->get_topic_name(), laps_, route_.size(), speed_, altitude_,
      get_parameter("use_sim_time").as_bool() ? "true" : "false");
    state("WAIT_ESTIMATE");
  }

private:
  // ------------------------------------------------------------ estimate

  void onEstimate(const nav_msgs::msg::Odometry & m)
  {
    const auto & p = m.pose.pose.position;
    const auto & q = m.pose.pose.orientation;
    const auto & l = m.twist.twist.linear;  // base_link
    const double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    const double sxy = std::sqrt(0.5 * (m.pose.covariance[0] + m.pose.covariance[7]));
    if (!std::isfinite(p.x + p.y + p.z + yaw + sxy) || m.header.frame_id != "map") {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "invalid estimate ignored (frame '%s')",
        m.header.frame_id.c_str());
      return;
    }
    est_ = {p.x, p.y, p.z};
    est_yaw_ = yaw;
    // twist is in base_link: rotate into ENU for the speed check
    est_vel_ = {std::cos(yaw) * l.x - std::sin(yaw) * l.y, std::sin(yaw) * l.x + std::cos(yaw) * l.y, l.z};
    est_sigma_xy_ = sxy;
    est_t_ = stampSec(m.header.stamp);
    have_est_ = true;
  }

  // Age of the newest estimate in simulation time.
  double estimateAge() const {return have_est_ ? now().seconds() - est_t_ : 1e9;}

  // ------------------------------------------------------------ state machine

  void tick()
  {
    if (phase_ == "DONE" || phase_ == "ABORTED") {return;}
    if (phase_ != "WAIT_ESTIMATE" && !state_.connected) {return abort("MAVROS lost connection", false);}
    if (phase_ == "ROUTE" || phase_ == "RETURN" || phase_ == "HOLD") {
      if (state_.mode != "GUIDED") {return abort("mode changed to " + state_.mode + " by someone else", false);}
      if (!state_.armed) {return abort("disarmed in flight", false);}
    }

    if (phase_ == "WAIT_ESTIMATE") {
      // Mission origin = first valid estimate: fresh, finite, converged, held for origin_settle_s.
      const bool ok = state_.connected && estimateAge() < est_timeout_ && est_sigma_xy_ < origin_sigma_;
      if (!ok || !waiting_) {since_ = now(); waiting_ = ok; return;}
      if ((now() - since_).seconds() < origin_wait_) {return;}
      origin_ = est_;
      ground_z_ = est_.z;
      yaw_hold_ = est_yaw_;
      buildRoute();
      RCLCPP_INFO(
        get_logger(), "origin = first valid estimate (%.2f, %.2f, %.2f), sigma xy %.2f m, yaw %.1f deg",
        origin_.x, origin_.y, origin_.z, est_sigma_xy_, yaw_hold_ * 180.0 / M_PI);
      t_mission_ = now();
      return state("SET_MODE");
    }
    if (phase_ == "SET_MODE") {
      if (state_.mode == "GUIDED") {return state("ARM");}
      return call<mavros_msgs::srv::SetMode>(mode_cli_, [](auto & r) {r.custom_mode = "GUIDED";});
    }
    if (phase_ == "ARM") {
      if (state_.armed) {return state("TAKEOFF");}
      return call<mavros_msgs::srv::CommandBool>(arm_cli_, [](auto & r) {r.value = true;});
    }
    if (phase_ == "TAKEOFF") {
      // ArduPilot's takeoff controller climbs; no setpoints until it is done.
      return call<mavros_msgs::srv::CommandTOL>(
        takeoff_cli_, [this](auto & r) {r.altitude = static_cast<float>(altitude_);},
        [this](bool ok) {if (ok) {since_ = now(); state("CLIMB");}});
    }
    if (phase_ == "CLIMB") {
      if (!state_.armed) {return state("SET_MODE");}  // arming timed out before takeoff
      // Done when OUR estimate says so. ArduPilot climbs to `altitude` by its barometer; if our z
      // disagrees by more than 0.2 m we take over after climb_timeout_s and our z loop corrects it.
      const double h = est_.z - ground_z_;
      const bool timed_out = (now() - since_).seconds() > climb_timeout_ && h > 0.5 * altitude_;
      if (h > altitude_ - 0.2 || timed_out) {
        RCLCPP_INFO(get_logger(), "airborne: estimated height %.2f m above the origin%s", h,
          timed_out ? " (climb timeout, our z loop takes over)" : "");
        cmd_ = {0, 0, 0};
        seg_ = 0;
        startSegment(V3{est_.x, est_.y, ref_z()}, wps_[0]);
        return state("ROUTE");
      }
      return;
    }
    if (phase_ == "ROUTE") {return route();}
    if (phase_ == "RETURN") {return returnToOrigin();}
    if (phase_ == "LAND") {
      return call<mavros_msgs::srv::CommandTOL>(land_cli_, [](auto &) {}, [this](bool ok) {if (ok) {state("DESCEND");}});
    }
    if (phase_ == "DESCEND") {
      if (!state_.armed) {
        const V3 d = est_ - origin_;
        RCLCPP_INFO(
          get_logger(), "landed and disarmed at estimated offset (%+.2f, %+.2f) m from the origin; "
          "mission %.1f s (first estimate -> touchdown)", d.x, d.y, (now() - t_mission_).seconds());
        return state("DONE");
      }
    }
  }

  // Waypoints in the map frame: `laps` times the route, then back to the origin.
  void buildRoute()
  {
    wps_.clear();
    for (int lap = 0; lap < laps_; ++lap) {
      for (size_t i = 0; i < route_.size(); ++i) {
        if (lap > 0 && i == 0) {continue;}  // the lap ends where the next begins
        wps_.push_back(V3{origin_.x + route_[i].x, origin_.y + route_[i].y, ref_z()});
      }
    }
    if (hnorm(wps_.back() - V3{origin_.x, origin_.y, ref_z()}) > 1e-6) {
      wps_.push_back(V3{origin_.x, origin_.y, ref_z()});
    }
  }

  double ref_z() const {return ground_z_ + altitude_;}

  void startSegment(V3 from, V3 to)
  {
    seg_a_ = from;
    seg_b_ = to;
    t_seg_ = now();
  }

  // Sliding reference along A -> B and the velocity command around it.
  void route()
  {
    const V3 ab = seg_b_ - seg_a_;
    const double len = hnorm(ab);
    double s, v;
    profile(len, (now() - t_seg_).seconds(), s, v);
    const V3 dir = len > 1e-6 ? ab * (1.0 / len) : V3{};
    const V3 ref = seg_a_ + dir * s;
    const V3 ff = dir * v;
    control(ref, ff);

    if (s >= len && hnorm(seg_b_ - est_) < reach_tol_) {
      RCLCPP_INFO(
        get_logger(), "waypoint %zu/%zu (%+.1f, %+.1f) reached by the estimate: error %.2f m, sigma xy %.2f m, t=%.1f s",
        seg_ + 1, wps_.size(), seg_b_.x - origin_.x, seg_b_.y - origin_.y, hnorm(seg_b_ - est_), est_sigma_xy_,
        (now() - t_mission_).seconds());
      if (++seg_ >= wps_.size()) {
        since_ = now();
        return state("RETURN");
      }
      startSegment(seg_b_, wps_[seg_]);  // from the waypoint, not from the estimate: stays on the line
    }
  }

  // Trapezoidal profile over a segment of length L: accelerate at a to speed v, cruise, brake at
  // a to stop exactly at L (triangular if L < v^2/a). Returns position s and speed ds/dt at time t.
  void profile(double L, double t, double & s, double & v) const
  {
    const double a = ref_accel_;
    const double vp = std::min(speed_, std::sqrt(a * L));  // peak speed
    const double ta = vp / a, da = 0.5 * vp * ta;            // accel time and distance
    const double tc = vp > 0.0 ? (L - 2.0 * da) / vp : 0.0;  // cruise time
    const double T = 2.0 * ta + tc;
    if (t <= 0.0 || L <= 0.0) {s = 0.0; v = 0.0;}
    else if (t < ta) {s = 0.5 * a * t * t; v = a * t;}
    else if (t < ta + tc) {s = da + vp * (t - ta); v = vp;}
    else if (t < T) {s = L - 0.5 * a * (T - t) * (T - t); v = a * (T - t);}
    else {s = L; v = 0.0;}
  }

  // Hold over the origin until the estimate is there and slow, then land.
  void returnToOrigin()
  {
    const V3 target{origin_.x, origin_.y, ref_z()};
    control(target, {});
    const bool ok = hnorm(target - est_) < land_tol_ && hnorm(est_vel_) < land_speed_;
    if (!ok) {since_ = now(); return;}
    if ((now() - since_).seconds() >= land_settle_) {
      RCLCPP_INFO(get_logger(), "over the origin: error %.2f m -> LAND", hnorm(target - est_));
      state("LAND");
    }
  }

  void control(V3 ref, V3 ff)
  {
    const double age = estimateAge();
    V3 want{};
    if (age < est_timeout_) {
      const V3 e = ref - est_;
      want = clampH(ff + V3{e.x, e.y, 0.0} * kp_, max_speed_);
      want.z = std::clamp(kp_z_ * (ref.z - est_.z), -max_speed_z_, max_speed_z_);
    } else if (age < est_lost_land_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "no estimate for %.2f s: hovering", age);
    } else {
      return abort("no estimate for " + std::to_string(age) + " s", true);
    }
    // Acceleration limit on the command.
    const double dt = last_cmd_t_.nanoseconds() ? std::clamp((now() - last_cmd_t_).seconds(), 0.0, 0.2) : 0.05;
    last_cmd_t_ = now();
    const V3 d = want - cmd_;
    const double n = std::sqrt(d.x * d.x + d.y * d.y + d.z * d.z), lim = max_accel_ * dt;
    cmd_ = n > lim ? cmd_ + d * (lim / n) : want;
    sendVelocity(cmd_);

    geometry_msgs::msg::PointStamped r;
    r.header.stamp = now();
    r.header.frame_id = "map";
    r.point.x = ref.x;
    r.point.y = ref.y;
    r.point.z = ref.z;
    ref_pub_->publish(r);

    if ((now() - last_report_).seconds() >= 2.0) {
      last_report_ = now();
      RCLCPP_INFO(
        get_logger(), "[%s] est (%+.2f, %+.2f, %.2f) ref (%+.2f, %+.2f) err %.2f m | cmd %.2f m/s, vz %+.2f | sigma xy %.2f m",
        phase_.c_str(), est_.x - origin_.x, est_.y - origin_.y, est_.z - ground_z_, ref.x - origin_.x,
        ref.y - origin_.y, hnorm(ref - est_), hnorm(cmd_), cmd_.z, est_sigma_xy_);
    }
  }

  // ENU velocity + fixed yaw (MAVROS converts ENU -> NED for the FCU).
  void sendVelocity(V3 v)
  {
    using PT = mavros_msgs::msg::PositionTarget;
    PT sp;
    sp.header.stamp = now();
    sp.coordinate_frame = PT::FRAME_LOCAL_NED;
    sp.type_mask = PT::IGNORE_PX | PT::IGNORE_PY | PT::IGNORE_PZ | PT::IGNORE_AFX | PT::IGNORE_AFY |
      PT::IGNORE_AFZ | PT::IGNORE_YAW_RATE;
    sp.velocity.x = v.x;
    sp.velocity.y = v.y;
    sp.velocity.z = v.z;
    sp.yaw = static_cast<float>(yaw_hold_);
    sp_pub_->publish(sp);
  }

  void abort(const std::string & why, bool land)
  {
    RCLCPP_ERROR(get_logger(), "ABORTED: %s%s", why.c_str(), land ? " -> LAND" : "");
    phase_ = "ABORTED";
    if (land && mode_cli_->service_is_ready()) {
      auto req = std::make_shared<mavros_msgs::srv::SetMode::Request>();
      req->custom_mode = "LAND";
      mode_cli_->async_send_request(req);
    }
  }

  void state(const std::string & s)
  {
    phase_ = s;
    RCLCPP_INFO(get_logger(), "[state] %s", s.c_str());
  }

  // One request in flight at a time, at most one per second; `done(success)` on the reply.
  template<typename Srv, typename Client, typename Fill>
  void call(Client & cli, Fill fill, std::function<void(bool)> done = nullptr)
  {
    if (pending_ || (now() - last_call_).seconds() < 1.0) {
      if (pending_ && (now() - last_call_).seconds() > 5.0) {cli->prune_pending_requests(); pending_ = false;}
      return;
    }
    if (!cli->service_is_ready()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "waiting for %s", cli->get_service_name());
      return;
    }
    auto req = std::make_shared<typename Srv::Request>();
    fill(*req);
    pending_ = true;
    last_call_ = now();
    RCLCPP_INFO(get_logger(), "-> %s", cli->get_service_name());
    cli->async_send_request(req, [this, done](typename rclcpp::Client<Srv>::SharedFuture f) {
        pending_ = false;
        const auto & r = *f.get();
        bool ok = true;
        if constexpr (std::is_same_v<Srv, mavros_msgs::srv::SetMode>) {ok = r.mode_sent;} else {ok = r.success;}
        if (!ok) {RCLCPP_WARN(get_logger(), "request rejected, retrying");}
        if (done) {done(ok);}
      });
  }

  // parameters
  double altitude_, speed_, ref_accel_, kp_, kp_z_, max_speed_, max_speed_z_, max_accel_, reach_tol_, land_tol_,
    land_speed_, land_settle_, origin_sigma_, origin_wait_, est_timeout_, est_lost_land_, climb_timeout_;
  int laps_;
  std::vector<V3> route_;

  // inputs
  mavros_msgs::msg::State state_;
  V3 est_, est_vel_;
  double est_yaw_{0}, est_sigma_xy_{1e9}, est_t_{0};
  bool have_est_{false};

  // mission
  std::string phase_;
  V3 origin_, seg_a_, seg_b_, cmd_;
  double ground_z_{0}, yaw_hold_{0};
  std::vector<V3> wps_;
  size_t seg_{0};
  rclcpp::Time since_{0, 0, RCL_ROS_TIME}, t_seg_{0, 0, RCL_ROS_TIME}, t_mission_{0, 0, RCL_ROS_TIME},
    last_call_{0, 0, RCL_ROS_TIME}, last_cmd_t_{0, 0, RCL_ROS_TIME}, last_report_{0, 0, RCL_ROS_TIME};
  bool pending_{false}, waiting_{false};

  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<mavros_msgs::msg::PositionTarget>::SharedPtr sp_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr ref_pub_;
  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr mode_cli_;
  rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arm_cli_;
  rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr takeoff_cli_, land_cli_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EstimatedFlight>());  // single thread: callbacks never overlap
  rclcpp::shutdown();
  return 0;
}
