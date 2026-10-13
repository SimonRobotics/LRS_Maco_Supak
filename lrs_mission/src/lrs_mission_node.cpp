// Assignment 1 — one node for the whole mission.
//
//   A1.1  map.pcd -> pcl::VoxelGrid downsample -> dense 3D occupancy grid
//         -> racks filled as solid boxes -> spherical inflation
//         -> A* (26-connected) -> line-of-sight shortcutting
//   A1.2  indoor mission from a CSV file: takeoff / land / landtakeoff / yawN
//   A1.3  outdoor mission: GPS + compass, POI square, figure-8, return and land
//
// Frames
//   map    frame of map.pcd and of the mission CSV (Gazebo world frame of the hangar)
//   local  MAVROS local ENU (/mavros/local_position/pose)
//   local = Rz(map_to_local_yaw) * (map - map_offset)
// All yaw angles in this file are ENU radians (0 = East / map +X, counter-clockwise).

#include <pcl/common/common.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <fstream>
#include <functional>
#include <limits>
#include <mutex>
#include <queue>
#include <sstream>
#include <string>
#include <vector>

#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "mavros_msgs/msg/home_position.hpp"
#include "mavros_msgs/msg/position_target.hpp"
#include "mavros_msgs/msg/state.hpp"
#include "mavros_msgs/srv/command_bool.hpp"
#include "mavros_msgs/srv/command_tol.hpp"
#include "mavros_msgs/srv/set_mode.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/u_int32.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

using namespace std::chrono_literals;

// ---------------------------------------------------------------- helpers

struct V3
{
  double x{0}, y{0}, z{0};
};
V3 operator+(V3 a, V3 b) {return {a.x + b.x, a.y + b.y, a.z + b.z};}
V3 operator-(V3 a, V3 b) {return {a.x - b.x, a.y - b.y, a.z - b.z};}
V3 operator*(V3 a, double s) {return {a.x * s, a.y * s, a.z * s};}
double norm(V3 a) {return std::sqrt(a.x * a.x + a.y * a.y + a.z * a.z);}
double hnorm(V3 a) {return std::hypot(a.x, a.y);}

double wrap(double a) {return std::atan2(std::sin(a), std::cos(a));}  // -> (-pi, pi]
double deg(double rad) {return rad * 180.0 / M_PI;}
double rad(double deg) {return deg * M_PI / 180.0;}

// ---------------------------------------------------------------- A1.1 map

// Dense voxel grid. `raw` = voxels containing map points or rack boxes,
// `inflated` = raw grown by the safety radius. Outside the grid = occupied.
class VoxelMap
{
public:
  double res{0.1};
  V3 origin;  // min corner, aligned to multiples of res (same as pcl::VoxelGrid)
  int nx{0}, ny{0}, nz{0};
  std::vector<uint8_t> raw, inflated;

  void build(const pcl::PointCloud<pcl::PointXYZ> & cloud, double resolution)
  {
    res = resolution;
    pcl::PointXYZ lo, hi;
    pcl::getMinMax3D(cloud, lo, hi);
    origin = {std::floor(lo.x / res) * res, std::floor(lo.y / res) * res, std::floor(lo.z / res) * res};
    nx = static_cast<int>((hi.x - origin.x) / res) + 1;
    ny = static_cast<int>((hi.y - origin.y) / res) + 1;
    nz = static_cast<int>((hi.z - origin.z) / res) + 1;
    raw.assign(static_cast<size_t>(nx) * ny * nz, 0);
    for (const auto & p : cloud) {
      int i, j, k;
      if (toIndex({p.x, p.y, p.z}, i, j, k)) {raw[id(i, j, k)] = 1;}
    }
    inflated = raw;
  }

  // Every voxel whose centre lies inside the box becomes occupied.
  void fillBox(V3 lo, V3 hi)
  {
    for (int k = 0; k < nz; ++k) {
      for (int j = 0; j < ny; ++j) {
        for (int i = 0; i < nx; ++i) {
          V3 c = centre(i, j, k);
          if (c.x >= lo.x && c.x <= hi.x && c.y >= lo.y && c.y <= hi.y && c.z >= lo.z && c.z <= hi.z) {
            raw[id(i, j, k)] = 1;
          }
        }
      }
    }
  }

  // Stamp a sphere of `radius` around every raw voxel (same in x, y and z).
  void inflate(double radius)
  {
    const int r = static_cast<int>(radius / res + 1e-9);
    const double r2 = (radius / res + 1e-9) * (radius / res + 1e-9);
    std::vector<std::array<int, 3>> sphere;
    for (int dz = -r; dz <= r; ++dz) {
      for (int dy = -r; dy <= r; ++dy) {
        for (int dx = -r; dx <= r; ++dx) {
          if (dx * dx + dy * dy + dz * dz <= r2) {sphere.push_back({dx, dy, dz});}
        }
      }
    }
    inflated = raw;
    for (int k = 0; k < nz; ++k) {
      for (int j = 0; j < ny; ++j) {
        for (int i = 0; i < nx; ++i) {
          if (!raw[id(i, j, k)]) {continue;}
          for (const auto & o : sphere) {
            if (valid(i + o[0], j + o[1], k + o[2])) {inflated[id(i + o[0], j + o[1], k + o[2])] = 1;}
          }
        }
      }
    }
  }

  size_t id(int i, int j, int k) const {return (static_cast<size_t>(k) * ny + j) * nx + i;}
  bool valid(int i, int j, int k) const {return i >= 0 && j >= 0 && k >= 0 && i < nx && j < ny && k < nz;}
  V3 centre(int i, int j, int k) const {return origin + V3{i + 0.5, j + 0.5, k + 0.5} * res;}
  bool toIndex(V3 p, int & i, int & j, int & k) const
  {
    i = static_cast<int>(std::floor((p.x - origin.x) / res));
    j = static_cast<int>(std::floor((p.y - origin.y) / res));
    k = static_cast<int>(std::floor((p.z - origin.z) / res));
    return valid(i, j, k);
  }
  bool freeIdx(int i, int j, int k) const {return valid(i, j, k) && !inflated[id(i, j, k)];}
  bool free(V3 p) const
  {
    int i, j, k;
    return toIndex(p, i, j, k) && !inflated[id(i, j, k)];
  }
  bool rawOccupied(V3 p) const
  {
    int i, j, k;
    return !toIndex(p, i, j, k) || raw[id(i, j, k)];
  }
  // Straight segment checked every half voxel against the inflated grid.
  bool segmentFree(V3 a, V3 b) const
  {
    const int n = static_cast<int>(std::ceil(norm(b - a) / (0.5 * res))) + 1;
    for (int s = 0; s <= n; ++s) {
      if (!free(a + (b - a) * (static_cast<double>(s) / n))) {return false;}
    }
    return true;
  }
  size_t count(const std::vector<uint8_t> & g) const {return std::count(g.begin(), g.end(), 1);}
};

// Nearest free voxel to p within 1 m, used when a waypoint lies in the inflation zone.
bool nearestFree(const VoxelMap & m, V3 p, int & bi, int & bj, int & bk)
{
  int i, j, k;
  m.toIndex(p, i, j, k);
  if (m.freeIdx(i, j, k)) {bi = i; bj = j; bk = k; return true;}
  const int r = static_cast<int>(1.0 / m.res);
  double best = std::numeric_limits<double>::infinity();
  for (int dz = -r; dz <= r; ++dz) {
    for (int dy = -r; dy <= r; ++dy) {
      for (int dx = -r; dx <= r; ++dx) {
        const double d = dx * dx + dy * dy + dz * dz;
        if (d < best && m.freeIdx(i + dx, j + dy, k + dz)) {
          best = d; bi = i + dx; bj = j + dy; bk = k + dz;
        }
      }
    }
  }
  return std::isfinite(best);
}

// A* over the 26-connected voxel grid. Edge cost = Euclidean length of the move,
// heuristic = Euclidean distance to the goal: never overestimates -> admissible.
// Terminates when the open list is empty (no path) since the grid is finite.
bool astar(const VoxelMap & m, V3 start, V3 goal, std::vector<V3> & path)
{
  int si, sj, sk, gi, gj, gk;
  if (!nearestFree(m, start, si, sj, sk) || !nearestFree(m, goal, gi, gj, gk)) {return false;}
  const size_t n = m.inflated.size();
  std::vector<float> g(n, std::numeric_limits<float>::infinity());
  std::vector<int> parent(n, -1);
  std::vector<uint8_t> closed(n, 0);
  auto h = [&](int i, int j, int k) {
      return static_cast<float>(m.res * std::sqrt(double((i - gi) * (i - gi) + (j - gj) * (j - gj) + (k - gk) * (k - gk))));
    };
  using Entry = std::pair<float, int>;  // (f, voxel id)
  std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> open;
  const int s = static_cast<int>(m.id(si, sj, sk)), goal_id = static_cast<int>(m.id(gi, gj, gk));
  g[s] = 0;
  open.push({h(si, sj, sk), s});

  while (!open.empty()) {
    const int cur = open.top().second;
    open.pop();
    if (closed[cur]) {continue;}
    closed[cur] = 1;
    if (cur == goal_id) {
      path.clear();
      for (int c = cur; c != -1; c = parent[c]) {
        path.push_back(m.centre(c % m.nx, (c / m.nx) % m.ny, c / (m.nx * m.ny)));
      }
      std::reverse(path.begin(), path.end());
      return true;
    }
    const int ci = cur % m.nx, cj = (cur / m.nx) % m.ny, ck = cur / (m.nx * m.ny);
    for (int dz = -1; dz <= 1; ++dz) {
      for (int dy = -1; dy <= 1; ++dy) {
        for (int dx = -1; dx <= 1; ++dx) {
          const int i = ci + dx, j = cj + dy, k = ck + dz;
          if ((!dx && !dy && !dz) || !m.freeIdx(i, j, k)) {continue;}
          const int nb = static_cast<int>(m.id(i, j, k));
          const float cost = g[cur] + static_cast<float>(m.res * std::sqrt(double(dx * dx + dy * dy + dz * dz)));
          if (cost < g[nb]) {
            g[nb] = cost;
            parent[nb] = cur;
            open.push({cost + h(i, j, k), nb});
          }
        }
      }
    }
  }
  return false;
}

// Line-of-sight shortcutting: from each kept point jump to the farthest later
// point reachable in a straight collision-free line.
std::vector<V3> simplify(const VoxelMap & m, const std::vector<V3> & path)
{
  std::vector<V3> out{path.front()};
  size_t i = 0;
  while (i + 1 < path.size()) {
    size_t j = path.size() - 1;
    while (j > i + 1 && !m.segmentFree(path[i], path[j])) {--j;}
    out.push_back(path[j]);
    i = j;
  }
  return out;
}

// ---------------------------------------------------------------- A1.3 geodesy

// Local tangent plane (flat Earth) around a reference fix. Over the < 20 m of this
// mission the error vs. GeographicLib is far below 1 mm (it grows ~ d^2 / R).
struct Geo
{
  double lat0{0}, lon0{0};
  static constexpr double R = 6378137.0;  // WGS-84 equatorial radius
  V3 toEnu(double lat, double lon) const
  {
    return {rad(lon - lon0) * R * std::cos(rad(lat0)), rad(lat - lat0) * R, 0.0};
  }
  void toGeo(V3 enu, double & lat, double & lon) const
  {
    lat = lat0 + deg(enu.y / R);
    lon = lon0 + deg(enu.x / (R * std::cos(rad(lat0))));
  }
};

// ---------------------------------------------------------------- mission

enum class Type { TAKEOFF, GOTO, YAW, LAND, FIGURE8 };
enum class Tol { PASS, SOFT, HARD };

struct Action
{
  Type type;
  V3 pos;              // local ENU target (GOTO, YAW), takeoff altitude in pos.z
  double yaw{0};       // YAW target
  Tol tol{Tol::PASS};  // GOTO arrival condition
  bool face_poi{false};
  std::string label;
};

struct Row
{
  V3 p;
  std::string precision, task;
};

std::vector<Row> readMission(const std::string & file)
{
  std::ifstream in(file);
  if (!in) {throw std::runtime_error("cannot open mission file " + file);}
  std::vector<Row> rows;
  std::string line;
  while (std::getline(in, line)) {
    if (line.empty() || line[0] == '#' || line[0] == 'x') {continue;}  // comments, header
    std::replace(line.begin(), line.end(), ',', ' ');
    std::istringstream ss(line);
    Row r;
    if (ss >> r.p.x >> r.p.y >> r.p.z >> r.precision >> r.task) {rows.push_back(r);}
  }
  return rows;
}

class MissionNode : public rclcpp::Node
{
public:
  MissionNode()
  : Node("lrs_mission")
  {
    // Two MutuallyExclusive groups run concurrently on the MultiThreadedExecutor in main().
    // cb_control_ gets tick(), the MAVROS service replies and ~/abort, so everything that
    // mutates the state machine is serialised by the executor itself and needs no lock.
    // cb_sensors_ gets every subscription; mtx_ guards only what they write and the other
    // groups read -- the sensor snapshot and flown_.
    cb_control_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    cb_sensors_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    sensor_opts_.callback_group = cb_sensors_;

    mode_ = declare_parameter("mission", std::string("indoor"));  // indoor | outdoor
    rate_hz_ = declare_parameter("rate_hz", 20.0);
    hard_tol_ = declare_parameter("hard_tolerance", 0.15);
    soft_tol_ = declare_parameter("soft_tolerance", 0.40);
    pass_tol_ = declare_parameter("pass_tolerance", 0.50);
    hard_speed_ = declare_parameter("hard_max_speed", 0.15);
    settle_s_ = declare_parameter("hard_settle_time", 1.0);
    yaw_tol_ = rad(declare_parameter("yaw_tolerance_deg", 5.0));
    max_speed_ = declare_parameter("max_speed", 1.0);

    auto qos = rclcpp::SensorDataQoS();
    state_sub_ = create_subscription<mavros_msgs::msg::State>(
      "/mavros/state", 10, [this](mavros_msgs::msg::State::ConstSharedPtr m) {
        std::lock_guard<std::mutex> lk(mtx_);
        state_ = *m;
      }, sensor_opts_);
    pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/mavros/local_position/pose", qos, [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr m) {
        std::lock_guard<std::mutex> lk(mtx_);
        pos_ = {m->pose.position.x, m->pose.position.y, m->pose.position.z};
        const auto & q = m->pose.orientation;
        yaw_ = std::atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
        have_pose_ = true;
        recordFlown();
      }, sensor_opts_);
    vel_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
      "/mavros/local_position/velocity_local", qos, [this](geometry_msgs::msg::TwistStamped::ConstSharedPtr m) {
        std::lock_guard<std::mutex> lk(mtx_);
        vel_ = {m->twist.linear.x, m->twist.linear.y, m->twist.linear.z};
        speed_ = norm(vel_);
      }, sensor_opts_);
    sp_pub_ = create_publisher<mavros_msgs::msg::PositionTarget>("/mavros/setpoint_raw/local", 10);
    // RViz (fixed frame "map"), latched so RViz can be started at any time
    const auto latched = rclcpp::QoS(1).transient_local();
    path_pub_ = create_publisher<nav_msgs::msg::Path>("~/path", latched);
    flown_pub_ = create_publisher<nav_msgs::msg::Path>("~/flown_path", latched);
    wp_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("~/waypoints", latched);
    cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("~/map", latched);
    flown_.header.frame_id = "map";
    viz_timer_ = create_wall_timer(500ms, [this] {
        nav_msgs::msg::Path snapshot;
        {
          std::lock_guard<std::mutex> lk(mtx_);
          if (!flown_changed_) {return;}
          snapshot = flown_;
          flown_changed_ = false;
        }
        // Serialising a Path of >1000 poses must not happen under mtx_, or the 20 Hz tick()
        // would wait for it -- which is the stall the callback groups exist to avoid.
        flown_pub_->publish(snapshot);
      });
    // The replies land in cb_control_, so they cannot run while tick() is running and
    // may touch step_ / pending_ / phase_ without a lock.
    mode_cli_ = create_client<mavros_msgs::srv::SetMode>(
      "/mavros/set_mode", rclcpp::ServicesQoS(), cb_control_);
    arm_cli_ = create_client<mavros_msgs::srv::CommandBool>(
      "/mavros/cmd/arming", rclcpp::ServicesQoS(), cb_control_);
    takeoff_cli_ = create_client<mavros_msgs::srv::CommandTOL>(
      "/mavros/cmd/takeoff", rclcpp::ServicesQoS(), cb_control_);
    land_cli_ = create_client<mavros_msgs::srv::CommandTOL>(
      "/mavros/cmd/land", rclcpp::ServicesQoS(), cb_control_);
    abort_srv_ = create_service<std_srvs::srv::Trigger>(
      "~/abort", [this](std_srvs::srv::Trigger::Request::SharedPtr, std_srvs::srv::Trigger::Response::SharedPtr res) {
        // Lock needed: abort() writes phase_, which recordFlown() reads from cb_sensors_.
        // (The service *replies* above touch only step_/pending_, which cb_sensors_ never
        // reads, and cb_control_ already serialises them against tick().)
        std::lock_guard<std::mutex> lk(mtx_);
        abort("abort requested", true);
        res->success = true;
      }, rclcpp::ServicesQoS(), cb_control_);

    if (mode_ == "indoor") {
      if (!setupIndoor()) {return;}  // plan_only or planning failed
    } else if (mode_ == "outdoor") {
      setupOutdoor();
    } else {
      throw std::runtime_error("parameter 'mission' must be indoor or outdoor");
    }
    timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / rate_hz_), [this] {
        std::lock_guard<std::mutex> lk(mtx_);  // tick() reads the sensor snapshot
        tick();
      }, cb_control_);
    log("IDLE (waiting for FCU connection, pose and -- outdoors -- a GPS fix)");
  }

private:
  // ============================================================ A1.1 + A1.2 indoor

  bool setupIndoor()
  {
    const auto map_file = declare_parameter("map_file", std::string(""));
    const double res = declare_parameter("resolution", 0.1);
    const double radius = declare_parameter("inflation_radius", 0.5);
    const auto boxes = declare_parameter("rack_boxes", std::vector<double>{});
    const auto offset = declare_parameter("map_offset", std::vector<double>{0.0, 0.0, 0.0});
    map_yaw_ = rad(declare_parameter("map_to_local_yaw_deg", 90.0));
    map_offset_ = {offset.at(0), offset.at(1), offset.at(2)};
    const bool plan_only = declare_parameter("plan_only", false);
    const auto start = declare_parameter("start", std::vector<double>{});
    const auto goal = declare_parameter("goal", std::vector<double>{});

    // --- map loading + downsampling
    auto t = std::chrono::steady_clock::now();
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>), down(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile(map_file, *cloud) != 0) {throw std::runtime_error("cannot read " + map_file);}
    pcl::VoxelGrid<pcl::PointXYZ> filter;
    filter.setInputCloud(cloud);
    filter.setLeafSize(res, res, res);
    filter.filter(*down);
    map_.build(*down, res);
    pcl::PointXYZ lo, hi;
    pcl::getMinMax3D(*cloud, lo, hi);
    RCLCPP_INFO(get_logger(), "map %s: %zu points -> %zu after %.2f m voxel grid filter",
      map_file.c_str(), cloud->size(), down->size(), res);
    RCLCPP_INFO(get_logger(), "bounding box [%.2f %.2f %.2f] -> [%.2f %.2f %.2f] m, grid %d x %d x %d = %zu voxels",
      lo.x, lo.y, lo.z, hi.x, hi.y, hi.z, map_.nx, map_.ny, map_.nz, map_.raw.size());

    // --- racks: solid boxes, then inflation
    for (size_t i = 0; i + 5 < boxes.size(); i += 6) {
      map_.fillBox({boxes[i], boxes[i + 1], boxes[i + 2]}, {boxes[i + 3], boxes[i + 4], boxes[i + 5]});
    }
    map_.inflate(radius);
    RCLCPP_INFO(get_logger(), "occupied voxels: %zu raw (+%zu rack boxes), %zu after %.2f m inflation, %.0f ms",
      map_.count(map_.raw), boxes.size() / 6, map_.count(map_.inflated), radius, msSince(t));

    // Occupancy queries: RViz "Publish Point" or `ros2 topic pub /lrs_mission/query ...`
    auto on_query = [this](geometry_msgs::msg::PointStamped::ConstSharedPtr m) {
        V3 p{m->point.x, m->point.y, m->point.z};
        RCLCPP_INFO(get_logger(), "query (%.2f %.2f %.2f) map: %s", p.x, p.y, p.z,
          map_.rawOccupied(p) ? "OCCUPIED" : map_.free(p) ? "FREE" : "OCCUPIED (inflation)");
      };
    // No lock needed: on_query only reads map_, which is immutable once it is built below.
    query_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
      "~/query", 10, on_query, sensor_opts_);
    click_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
      "/clicked_point", 10, on_query, sensor_opts_);

    // --- plan between consecutive goals
    std::vector<Row> rows;
    if (start.size() == 3 && goal.size() == 3) {
      rows = {{{start[0], start[1], start[2]}, "hard", "-"}, {{goal[0], goal[1], goal[2]}, "hard", "-"}};
    } else {
      rows = readMission(declare_parameter("mission_file", std::string("")));
    }
    if (rows.size() < 2) {throw std::runtime_error("need at least two waypoints");}

    std::vector<std::vector<V3>> segments;
    std::vector<V3> planned;
    for (size_t i = 1; i < rows.size(); ++i) {
      t = std::chrono::steady_clock::now();
      std::vector<V3> raw_path;
      if (!astar(map_, rows[i - 1].p, rows[i].p, raw_path)) {
        RCLCPP_ERROR(get_logger(), "no path from waypoint %zu to %zu", i, i + 1);
        return false;
      }
      const double plan_ms = msSince(t);
      auto path = simplify(map_, raw_path);
      bool ok = true;
      for (size_t k = 1; k < path.size(); ++k) {ok = ok && map_.segmentFree(path[k - 1], path[k]);}
      RCLCPP_INFO(get_logger(), "path %zu -> %zu: A* %.0f ms, %zu points -> %zu after shortcutting, collision check %s",
        i, i + 1, plan_ms, raw_path.size(), path.size(), ok ? "OK" : "FAILED");
      if (!ok) {return false;}
      for (auto & p : path) {
        RCLCPP_INFO(get_logger(), "    (%.2f, %.2f, %.2f)", p.x, p.y, p.z);
        planned.push_back(p);
      }
      path.back() = rows[i].p;  // fly to the exact waypoint, not the voxel centre
      segments.push_back(path);
    }
    publishMap(declare_parameter("viz_map_max_z", 4.8));
    publishPath(planned);
    std::vector<Waypoint> wps;
    for (size_t i = 0; i < rows.size(); ++i) {
      wps.push_back({rows[i].p, std::to_string(i + 1) + " " + rows[i].precision + " " + rows[i].task,
          rows[i].precision == "hard"});
    }
    publishWaypoints(wps);
    if (plan_only || start.size() == 3) {return false;}

    // --- build the action list (local frame)
    const double map_yaw = map_yaw_;
    actions_.push_back({Type::TAKEOFF, toLocal(rows[0].p), 0, Tol::HARD, false, "takeoff wp 1"});
    for (size_t i = 0; i < rows.size(); ++i) {
      const auto & r = rows[i];
      const std::string wp = "wp " + std::to_string(i + 1);
      if (i > 0) {
        const auto & seg = segments[i - 1];
        for (size_t k = 1; k < seg.size(); ++k) {
          const bool last = k + 1 == seg.size();
          const Tol tol = !last ? Tol::PASS : r.precision == "hard" ? Tol::HARD : Tol::SOFT;
          actions_.push_back({Type::GOTO, toLocal(seg[k]), 0, tol, false, last ? wp : wp + " via " + std::to_string(k)});
        }
      }
      if (r.task.rfind("yaw", 0) == 0) {
        // yawN: N degrees in the map frame, counter-clockwise from map +X
        const double yaw = wrap(rad(std::stod(r.task.substr(3))) + map_yaw);
        actions_.push_back({Type::YAW, toLocal(r.p), yaw, Tol::HARD, false, wp + " " + r.task});
      } else if (r.task == "land") {
        actions_.push_back({Type::LAND, toLocal(r.p), 0, Tol::HARD, false, wp + " land"});
      } else if (r.task == "landtakeoff") {
        actions_.push_back({Type::LAND, toLocal(r.p), 0, Tol::HARD, false, wp + " land"});
        actions_.push_back({Type::TAKEOFF, toLocal(r.p), 0, Tol::HARD, false, wp + " takeoff"});
      }
    }
    if (actions_.back().type != Type::LAND) {
      actions_.push_back({Type::LAND, actions_.back().pos, 0, Tol::HARD, false, "final land"});
    }
    return true;
  }

  V3 toLocal(V3 p) const
  {
    const V3 d = p - map_offset_;
    return {std::cos(map_yaw_) * d.x - std::sin(map_yaw_) * d.y, std::sin(map_yaw_) * d.x + std::cos(map_yaw_) * d.y, d.z};
  }

  // Inverse of toLocal. Outdoors map_yaw_ = 0 and map_offset_ = 0, so "map" = MAVROS local ENU.
  V3 toMap(V3 p) const
  {
    const V3 d{std::cos(map_yaw_) * p.x + std::sin(map_yaw_) * p.y, -std::sin(map_yaw_) * p.x + std::cos(map_yaw_) * p.y, p.z};
    return d + map_offset_;
  }

  // ============================================================ RViz

  struct Waypoint
  {
    V3 p;  // map frame
    std::string label;
    bool hard;
  };

  static geometry_msgs::msg::PoseStamped toPose(V3 p)
  {
    geometry_msgs::msg::PoseStamped ps;
    ps.header.frame_id = "map";
    ps.pose.position.x = p.x;
    ps.pose.position.y = p.y;
    ps.pose.position.z = p.z;
    ps.pose.orientation.w = 1.0;
    return ps;
  }

  void publishPath(const std::vector<V3> & pts)
  {
    nav_msgs::msg::Path path;
    path.header.frame_id = "map";
    path.header.stamp = now();
    for (const auto & p : pts) {path.poses.push_back(toPose(p));}
    path_pub_->publish(path);
  }

  // Spheres (red = hard, green = soft) with a text label above each.
  void publishWaypoints(const std::vector<Waypoint> & wps)
  {
    visualization_msgs::msg::MarkerArray arr;
    visualization_msgs::msg::Marker clear;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    arr.markers.push_back(clear);
    for (size_t i = 0; i < wps.size(); ++i) {
      visualization_msgs::msg::Marker m;
      m.header.frame_id = "map";
      m.header.stamp = now();
      m.ns = "waypoints";
      m.id = static_cast<int>(i);
      m.type = visualization_msgs::msg::Marker::SPHERE;
      m.pose = toPose(wps[i].p).pose;
      m.scale.x = m.scale.y = m.scale.z = 0.3;
      m.color.r = wps[i].hard ? 0.9f : 0.1f;
      m.color.g = wps[i].hard ? 0.1f : 0.8f;
      m.color.b = 0.1f;
      m.color.a = 0.9f;
      arr.markers.push_back(m);

      m.ns = "labels";
      m.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      m.pose.position.z += 0.4;
      m.scale.x = m.scale.y = 0.0;
      m.scale.z = 0.35;  // text height
      m.color.r = m.color.g = m.color.b = 1.0f;
      m.text = wps[i].label;
      arr.markers.push_back(m);
    }
    wp_pub_->publish(arr);
  }

  // Actual track: a new pose every 5 cm while the mission runs, published at 2 Hz.
  void recordFlown()
  {
    if (phase_ != "RUN") {return;}
    const V3 p = toMap(pos_);
    if (!flown_.poses.empty()) {
      const auto & last = flown_.poses.back().pose.position;
      if (norm(p - V3{last.x, last.y, last.z}) < 0.05) {return;}
    }
    flown_.poses.push_back(toPose(p));
    flown_.header.stamp = now();
    flown_changed_ = true;
  }

  // Map voxels for RViz, cropped at max_z so the ceiling does not hide the inside.
  void publishMap(double max_z)
  {
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.frame_id = "map";
    sensor_msgs::PointCloud2Modifier mod(cloud);
    mod.setPointCloud2FieldsByString(1, "xyz");
    mod.resize(map_.count(map_.raw));  // upper bound, shrunk below
    sensor_msgs::PointCloud2Iterator<float> x(cloud, "x"), y(cloud, "y"), z(cloud, "z");
    size_t n = 0;
    for (int k = 0; k < map_.nz; ++k) {
      for (int j = 0; j < map_.ny; ++j) {
        for (int i = 0; i < map_.nx; ++i) {
          const V3 c = map_.centre(i, j, k);
          if (!map_.raw[map_.id(i, j, k)] || c.z > max_z) {continue;}
          *x = c.x; *y = c.y; *z = c.z;
          ++x; ++y; ++z;
          ++n;
        }
      }
    }
    mod.resize(n);
    cloud_pub_->publish(cloud);
  }

  // ============================================================ A1.3 outdoor

  void setupOutdoor()
  {
    alt_ = declare_parameter("altitude", 5.0);
    side_ = declare_parameter("square_side", 5.0);
    amp_ = declare_parameter("figure8_amplitude", 3.0);
    omega_ = declare_parameter("figure8_omega", 0.3);
    laps_ = declare_parameter("figure8_laps", 1.0);
    poi_offset_ = {declare_parameter("poi_east", 0.0), declare_parameter("poi_north", 5.0), 0.0};
    poi_lat_ = declare_parameter("poi_lat", 0.0);  // used instead of the offset when non-zero
    poi_lon_ = declare_parameter("poi_lon", 0.0);
    min_sats_ = declare_parameter("min_satellites", 6);

    auto qos = rclcpp::SensorDataQoS();
    fix_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
      "/mavros/global_position/global", qos, [this](sensor_msgs::msg::NavSatFix::ConstSharedPtr m) {
        std::lock_guard<std::mutex> lk(mtx_);
        fix_ = *m;
      }, sensor_opts_);
    sats_sub_ = create_subscription<std_msgs::msg::UInt32>(
      "/mavros/global_position/raw/satellites", qos, [this](std_msgs::msg::UInt32::ConstSharedPtr m) {
        std::lock_guard<std::mutex> lk(mtx_);
        sats_ = m->data;
      }, sensor_opts_);
    hdg_sub_ = create_subscription<std_msgs::msg::Float64>(
      "/mavros/global_position/compass_hdg", qos, [this](std_msgs::msg::Float64::ConstSharedPtr m) {
        std::lock_guard<std::mutex> lk(mtx_);
        hdg_ = m->data;
      }, sensor_opts_);
    relalt_sub_ = create_subscription<std_msgs::msg::Float64>(
      "/mavros/global_position/rel_alt", qos, [this](std_msgs::msg::Float64::ConstSharedPtr m) {
        std::lock_guard<std::mutex> lk(mtx_);
        rel_alt_ = m->data;
      }, sensor_opts_);
    home_sub_ = create_subscription<mavros_msgs::msg::HomePosition>(  // latched topic
      "/mavros/home_position/home", rclcpp::QoS(1).transient_local(), [this](mavros_msgs::msg::HomePosition::ConstSharedPtr m) {
        std::lock_guard<std::mutex> lk(mtx_);
        if (!home_) {  // one-shot, so logging under the lock is fine here
          home_ = true;
          geo_.lat0 = m->geo.latitude;
          geo_.lon0 = m->geo.longitude;
          home_local_ = {m->position.x, m->position.y, m->position.z};
          RCLCPP_INFO(get_logger(), "home: lat %.7f lon %.7f alt %.2f, local (%.2f %.2f %.2f)",
            geo_.lat0, geo_.lon0, m->geo.altitude, home_local_.x, home_local_.y, home_local_.z);
        }
      }, sensor_opts_);
    status_timer_ = create_wall_timer(1s, [this] {printStatus();});
  }

  bool gpsReady() const
  {
    return home_ && fix_.status.status >= sensor_msgs::msg::NavSatStatus::STATUS_FIX &&
           sats_ >= static_cast<unsigned>(min_sats_);
  }

  // Called once home is known: everything is defined relative to it.
  void buildOutdoorActions()
  {
    // POI and corners are defined geodetically, then converted to local metres.
    double plat = poi_lat_, plon = poi_lon_;
    if (plat == 0.0 && plon == 0.0) {geo_.toGeo(poi_offset_, plat, plon);}
    const V3 h{side_ / 2, side_ / 2, 0};
    const std::vector<V3> corner_offsets = {{-h.x, h.y, 0}, {h.x, h.y, 0}, {h.x, -h.y, 0}, {-h.x, -h.y, 0}};
    const V3 up{0, 0, alt_};
    poi_ = home_local_ + geo_.toEnu(plat, plon);  // POI on the ground
    RCLCPP_INFO(get_logger(), "POI lat %.7f lon %.7f -> local (%.2f %.2f)", plat, plon, poi_.x, poi_.y);

    actions_.push_back({Type::TAKEOFF, home_local_ + up, 0, Tol::HARD, false, "takeoff to 5 m"});
    for (size_t i = 0; i < corner_offsets.size(); ++i) {
      double lat, lon;
      geo_.toGeo(geo_.toEnu(plat, plon) + corner_offsets[i], lat, lon);
      const V3 local = home_local_ + geo_.toEnu(lat, lon) + up;
      RCLCPP_INFO(get_logger(), "corner W%zu lat %.7f lon %.7f -> local (%.2f %.2f %.2f)", i + 1, lat, lon,
        local.x, local.y, local.z);
      actions_.push_back({Type::GOTO, local, 0, Tol::SOFT, true, "corner W" + std::to_string(i + 1)});
    }
    actions_.push_back({Type::GOTO, poi_ + up, 0, Tol::HARD, false, "figure-8 centre"});
    actions_.push_back({Type::YAW, poi_ + up, M_PI / 4, Tol::HARD, false, "align with tangent at t=0"});
    actions_.push_back({Type::FIGURE8, poi_ + up, 0, Tol::PASS, false, "figure-8"});
    actions_.push_back({Type::GOTO, home_local_ + up, 0, Tol::HARD, false, "return to launch"});
    actions_.push_back({Type::LAND, home_local_, 0, Tol::HARD, false, "land"});

    // RViz: commanded trajectory (takeoff, square, centre, figure-8, return) and waypoints.
    std::vector<V3> traj{home_local_};
    std::vector<Waypoint> wps{{home_local_, "home", true}, {poi_, "POI", true}};
    for (const auto & a : actions_) {
      if (a.type == Type::GOTO || a.type == Type::TAKEOFF) {traj.push_back(a.pos);}
      if (a.type == Type::GOTO) {wps.push_back({a.pos, a.label, a.tol == Tol::HARD});}
      if (a.type == Type::FIGURE8) {
        for (int k = 0; k <= 200; ++k) {
          const double t = 2 * M_PI * k / 200;
          traj.push_back(a.pos + V3{amp_ * std::sin(t), amp_ * std::sin(t) * std::cos(t), 0});
        }
      }
    }
    traj.push_back(home_local_);
    publishPath(traj);
    publishWaypoints(wps);
  }

  void printStatus()
  {
    // Runs in the default group. Take a copy of the snapshot, then format and log without
    // the lock -- RCLCPP_INFO with this many arguments is far too slow to hold mtx_ across.
    sensor_msgs::msg::NavSatFix fix;
    Geo geo;
    V3 pos, home_local;
    double rel_alt, hdg, yaw;
    unsigned sats;
    {
      std::lock_guard<std::mutex> lk(mtx_);
      if (!home_ || !have_pose_) {return;}
      fix = fix_;
      geo = geo_;
      pos = pos_;
      home_local = home_local_;
      rel_alt = rel_alt_;
      hdg = hdg_;
      yaw = yaw_;
      sats = sats_;
    }
    const V3 gps = geo.toEnu(fix.latitude, fix.longitude);
    const V3 loc = pos - home_local;
    double lat, lon;
    geo.toGeo(loc, lat, lon);
    RCLCPP_INFO(get_logger(),
      "GPS lat %.7f lon %.7f amsl %.2f -> ENU (%.2f %.2f) | local ENU from home (%.2f %.2f %.2f) -> lat %.7f lon %.7f"
      " | rel_alt %.2f | compass %.1f deg (N, CW) -> ENU yaw %.1f deg, pose yaw %.1f deg (E, CCW) | sats %u",
      fix.latitude, fix.longitude, fix.altitude, gps.x, gps.y, loc.x, loc.y, loc.z, lat, lon, rel_alt, hdg,
      deg(wrap(rad(90.0 - hdg))), deg(yaw), sats);
  }

  // ============================================================ state machine

  void tick()
  {
    if (phase_ == "DONE" || phase_ == "ABORTED") {return;}
    if (phase_ != "IDLE" && !state_.connected) {return abort("MAVROS lost connection", false);}

    if (phase_ == "IDLE") {
      if (!state_.connected || !have_pose_) {return;}
      if (mode_ == "outdoor") {
        if (!gpsReady()) {
          RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000, "waiting for GPS fix / %d satellites / home (have %u)",
            min_sats_, sats_);
          return;
        }
        buildOutdoorActions();
      }
      return start(0);
    }

    Action & a = actions_[idx_];
    if (!modeAllowed(a)) {
      return abort("mode changed to " + state_.mode + " by someone else, stopping setpoints", false);
    }

    switch (a.type) {
      case Type::TAKEOFF: takeoff(a); break;
      case Type::LAND: land(); break;
      case Type::GOTO: go(a); break;
      case Type::YAW: {
          send(a.pos, a.yaw);
          const bool ok = std::abs(wrap(a.yaw - yaw_)) < yaw_tol_;
          if (ok && held(settle_s_)) {next();} else if (!ok) {since_ = now();}
          break;
        }
      case Type::FIGURE8: figure8(a); break;
    }
  }

  void start(size_t i)
  {
    idx_ = i;
    step_ = 0;
    since_ = now();
    t0_ = now();
    phase_ = "RUN";
    // Indexed by Type. TAKEOFF's first sub-state is SET_MODE, so that is what is logged.
    const char * names[] = {"SET_MODE", "NAVIGATE", "TASK_YAW", "LAND", "FIGURE8"};
    log(std::string(names[static_cast<int>(actions_[i].type)]) + " (" + actions_[i].label + ")");
    if (actions_[i].type == Type::GOTO) {
      // Heading = direction of travel from the previous setpoint to this target.
      seg_start_ = have_sp_ ? sp_pos_ : pos_;
      const V3 d = actions_[i].pos - seg_start_;
      travel_yaw_ = hnorm(d) > 0.3 ? std::atan2(d.y, d.x) : (have_sp_ ? sp_yaw_ : yaw_);
    }
  }

  void next()
  {
    if (idx_ + 1 >= actions_.size()) {
      phase_ = "DONE";
      return log("DONE");
    }
    start(idx_ + 1);
  }

  // true once the current condition has held for `s` seconds (caller resets since_)
  bool held(double s) {return (now() - since_).seconds() >= s;}

  // Sub-states of TAKEOFF and LAND. They are logged like the main states, so the log says
  // where a mission stopped instead of only "-> /mavros/set_mode".
  enum TakeoffStep { TK_SET_MODE, TK_ARM, TK_COMMAND, TK_CLIMB };
  enum LandStep { LD_COMMAND, LD_DESCEND, LD_GROUND };

  void setStep(int s, const char * name)
  {
    step_ = s;
    log(std::string(name) + " (" + actions_[idx_].label + ")");
  }

  // Which FCU modes are legitimate while the current action runs. Anything else means
  // someone else -- the safety pilot on the RC -- took over, so we stop commanding instead
  // of fighting them (A1.3 section 6).
  bool modeAllowed(const Action & a) const
  {
    switch (a.type) {
      case Type::TAKEOFF:
        // Below TK_ARM we are still trying to reach GUIDED, so any mode is expected.
        return step_ < TK_ARM || state_.mode == "GUIDED";
      case Type::LAND:
        // cmd/land switches the FCU to LAND, but /mavros/state keeps reporting the previous
        // GUIDED for a few cycles afterwards, so both are legitimate here.
        return state_.mode == "LAND" || state_.mode == "GUIDED";
      default:
        return state_.mode == "GUIDED";  // GOTO, YAW, FIGURE8
    }
  }

  void takeoff(const Action & a)
  {
    have_sp_ = false;  // no setpoints during takeoff, ArduPilot's takeoff controller climbs
    if (step_ >= TK_COMMAND && !state_.armed) {  // disarmed after arming (e.g. arming timeout)
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "disarmed during takeoff, restarting from SET_MODE");
      step_ = TK_SET_MODE;
    }
    switch (step_) {
      case TK_SET_MODE:
        if (state_.mode == "GUIDED") {setStep(TK_ARM, "ARM"); break;}
        call<mavros_msgs::srv::SetMode>(mode_cli_, [](auto & r) {r.custom_mode = "GUIDED";},
          [this](auto & r) {if (!r.mode_sent) {RCLCPP_WARN(get_logger(), "set GUIDED rejected");}});
        break;
      case TK_ARM:
        if (state_.armed) {setStep(TK_COMMAND, "TAKEOFF"); break;}
        call<mavros_msgs::srv::CommandBool>(arm_cli_, [](auto & r) {r.value = true;},
          [this](auto & r) {if (!r.success) {RCLCPP_WARN(get_logger(), "arming rejected, see SITL console");}});
        break;
      case TK_COMMAND:
        call<mavros_msgs::srv::CommandTOL>(takeoff_cli_,
          [this, &a](auto & r) {r.altitude = static_cast<float>(a.pos.z - (mode_ == "outdoor" ? home_local_.z : 0.0));},
          [this](auto & r) {
            if (r.success) {setStep(TK_CLIMB, "CLIMB");} else {RCLCPP_WARN(get_logger(), "takeoff rejected");}
          });
        break;
      case TK_CLIMB:
        if (pos_.z > a.pos.z - 0.15) {
          RCLCPP_INFO(get_logger(), "airborne at %.2f m", pos_.z);
          sp_pos_ = {pos_.x, pos_.y, a.pos.z};
          sp_yaw_ = yaw_;
          have_sp_ = true;
          next();
        }
        break;
    }
  }

  void land()
  {
    have_sp_ = false;
    switch (step_) {
      case LD_COMMAND:
        call<mavros_msgs::srv::CommandTOL>(land_cli_, [](auto &) {},
          [this](auto & r) {
            if (r.success) {setStep(LD_DESCEND, "DESCEND");} else {RCLCPP_WARN(get_logger(), "land rejected");}
          });
        break;
      case LD_DESCEND:  // ArduPilot's LAND mode disarms by itself after touchdown
        if (!state_.armed) {
          RCLCPP_INFO(get_logger(), "landed and disarmed");
          setStep(LD_GROUND, "ON_GROUND");
          since_ = now();
        }
        break;
      case LD_GROUND:
        if (held(2.0)) {next();}  // wait on the ground
        break;
    }
  }

  void go(const Action & a)
  {
    // Turn to the direction of travel before starting the segment (large turns only).
    if (step_ == 0) {
      if (!a.face_poi && std::abs(wrap(travel_yaw_ - yaw_)) > rad(30)) {
        send(seg_start_, travel_yaw_);
        if (std::abs(wrap(travel_yaw_ - yaw_)) > rad(10)) {return;}
      }
      step_ = 1;
      t0_ = now();
    }
    // Carrot: the setpoint slides from the segment start to the target at max_speed,
    // so the speed is bounded and the drone stays on the planned (collision-free) line.
    const V3 seg = a.pos - seg_start_;
    const double len = norm(seg);
    const double s = std::min(len, max_speed_ * (now() - t0_).seconds());
    const V3 carrot = len > 1e-3 ? seg_start_ + seg * (s / len) : a.pos;
    if (a.face_poi) {
      // Bearing to the POI, recomputed every tick, plus its rate of change as feed-forward:
      // d = poi - pos, d' = -vel  ->  bearing' = (d.x * d'.y - d.y * d'.x) / |d|^2
      const V3 d = poi_ - pos_;
      const double r2 = std::max(d.x * d.x + d.y * d.y, 0.25);
      send(carrot, std::atan2(d.y, d.x), (-d.x * vel_.y + d.y * vel_.x) / r2);
    } else {
      send(carrot, travel_yaw_);
    }

    const double err = norm(a.pos - pos_);
    if (a.tol == Tol::PASS && err < pass_tol_) {return next();}
    if (a.tol == Tol::SOFT && err < soft_tol_) {return next();}
    if (a.tol == Tol::HARD) {
      const bool ok = err < hard_tol_ && speed_ < hard_speed_;
      if (!ok) {since_ = now();} else if (held(settle_s_)) {
        RCLCPP_INFO(get_logger(), "settled at %s, error %.2f m", a.label.c_str(), err);
        next();
      }
    }
  }

  void figure8(const Action & a)
  {
    const double t = omega_ * (now() - t0_).seconds();
    if (t >= 2 * M_PI * laps_) {return next();}
    const V3 p = a.pos + V3{amp_ * std::sin(t), amp_ * std::sin(t) * std::cos(t), 0};
    // Tangent heading: (x', y') = (cos t, cos 2t) (times A), yaw = atan2(y', x').
    // Its derivative is sent as yaw-rate feed-forward: omega * (x' y'' - y' x'') / (x'^2 + y'^2).
    const double dx = std::cos(t), dy = std::cos(2 * t), ddx = -std::sin(t), ddy = -2 * std::sin(2 * t);
    send(p, std::atan2(dy, dx), omega_ * (dx * ddy - dy * ddx) / (dx * dx + dy * dy));
  }

  // Position + yaw setpoint in local ENU (MAVROS converts to NED).
  // yaw_rate [rad/s] is an optional feed-forward for ArduPilot's yaw controller.
  void send(V3 p, double yaw, double yaw_rate = std::numeric_limits<double>::quiet_NaN())
  {
    yaw = have_sp_ ? sp_yaw_ + wrap(yaw - sp_yaw_) : yaw;  // unwrap: never jump by 2 pi
    mavros_msgs::msg::PositionTarget sp;
    sp.header.stamp = now();
    sp.coordinate_frame = mavros_msgs::msg::PositionTarget::FRAME_LOCAL_NED;
    sp.type_mask = mavros_msgs::msg::PositionTarget::IGNORE_VX | mavros_msgs::msg::PositionTarget::IGNORE_VY |
      mavros_msgs::msg::PositionTarget::IGNORE_VZ | mavros_msgs::msg::PositionTarget::IGNORE_AFX |
      mavros_msgs::msg::PositionTarget::IGNORE_AFY | mavros_msgs::msg::PositionTarget::IGNORE_AFZ |
      (std::isnan(yaw_rate) ? mavros_msgs::msg::PositionTarget::IGNORE_YAW_RATE : 0);
    sp.position.x = p.x;
    sp.position.y = p.y;
    sp.position.z = p.z;
    sp.yaw = static_cast<float>(yaw);
    sp.yaw_rate = std::isnan(yaw_rate) ? 0.0f : static_cast<float>(yaw_rate);
    sp_pub_->publish(sp);
    sp_pos_ = p;
    sp_yaw_ = yaw;
    have_sp_ = true;
  }

  void abort(const std::string & why, bool command_land)
  {
    if (phase_ == "DONE" || phase_ == "ABORTED") {return;}
    phase_ = "ABORTED";
    RCLCPP_ERROR(get_logger(), "ABORTED: %s", why.c_str());
    if (command_land && mode_cli_->service_is_ready()) {
      auto req = std::make_shared<mavros_msgs::srv::SetMode::Request>();
      req->custom_mode = "LAND";
      mode_cli_->async_send_request(req);
    }
  }

  // One service call in flight at a time, retried at most every second.
  template<typename Srv, typename Client, typename Fill, typename Done>
  void call(Client & cli, Fill fill, Done done)
  {
    const double since = (now() - last_call_).seconds();
    if (pending_ && since > 5.0) {
      RCLCPP_WARN(get_logger(), "no response from %s, retrying", cli->get_service_name());
      cli->prune_pending_requests();
      pending_ = false;
    }
    if (pending_ || since < 1.0) {return;}
    if (!cli->service_is_ready()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "waiting for service %s", cli->get_service_name());
      return;
    }
    auto req = std::make_shared<typename Srv::Request>();
    fill(*req);
    pending_ = true;
    last_call_ = now();
    RCLCPP_INFO(get_logger(), "-> %s", cli->get_service_name());
    cli->async_send_request(req, [this, done](typename rclcpp::Client<Srv>::SharedFuture f) {
        pending_ = false;
        done(*f.get());
      });
  }

  void log(const std::string & s) {RCLCPP_INFO(get_logger(), "[state] %s", s.c_str());}
  static double msSince(std::chrono::steady_clock::time_point t)
  {
    return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t).count();
  }

  // parameters
  std::string mode_;
  double rate_hz_, hard_tol_, soft_tol_, pass_tol_, hard_speed_, settle_s_, yaw_tol_, max_speed_;
  double alt_{5}, side_{5}, amp_{3}, omega_{0.3}, laps_{1}, poi_lat_{0}, poi_lon_{0};
  int min_sats_{6};
  V3 poi_offset_, map_offset_;
  double map_yaw_{0};

  // telemetry
  mavros_msgs::msg::State state_;
  V3 pos_, vel_;
  double yaw_{0}, speed_{0}, hdg_{0}, rel_alt_{0};
  bool have_pose_{false};
  sensor_msgs::msg::NavSatFix fix_;
  unsigned sats_{0};
  bool home_{false};
  Geo geo_;
  V3 home_local_, poi_;

  // mission
  VoxelMap map_;
  std::vector<Action> actions_;
  std::string phase_{"IDLE"};
  size_t idx_{0};
  int step_{0};
  rclcpp::Time since_, t0_, last_call_{0, 0, RCL_ROS_TIME};
  bool pending_{false};
  V3 sp_pos_, seg_start_;
  double sp_yaw_{0}, travel_yaw_{0};
  bool have_sp_{false};

  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr vel_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr query_sub_, click_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr fix_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt32>::SharedPtr sats_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr hdg_sub_, relalt_sub_;
  rclcpp::Subscription<mavros_msgs::msg::HomePosition>::SharedPtr home_sub_;
  rclcpp::Publisher<mavros_msgs::msg::PositionTarget>::SharedPtr sp_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_, flown_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr wp_pub_;
  nav_msgs::msg::Path flown_;
  bool flown_changed_{false};
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr mode_cli_;
  rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arm_cli_;
  rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr takeoff_cli_, land_cli_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr abort_srv_;
  rclcpp::TimerBase::SharedPtr timer_, status_timer_, viz_timer_;

  // Threading (see the top of the constructor). mtx_ guards exactly the members the
  // cb_sensors_ callbacks write and the other groups read: state_, pos_, yaw_, have_pose_,
  // vel_, speed_, fix_, sats_, hdg_, rel_alt_, home_, geo_, home_local_, flown_ and
  // flown_changed_. The state machine's own members need no lock, because tick(), the
  // service replies and ~/abort all run in the MutuallyExclusive cb_control_ group.
  rclcpp::CallbackGroup::SharedPtr cb_control_, cb_sensors_;
  rclcpp::SubscriptionOptions sensor_opts_;
  std::mutex mtx_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MissionNode>();
  // Three groups can each run one callback at a time -- cb_control_, cb_sensors_ and the
  // default group (RViz/status timers, parameter services) -- so four threads is enough.
  rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 4);
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
