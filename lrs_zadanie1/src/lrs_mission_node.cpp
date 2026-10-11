
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
#include <queue>
#include <sstream>
#include <string>
#include <vector>

#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/u_int32.hpp"
#include "std_srvs/srv/trigger.hpp"

using namespace std::chrono_literals;

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


std::vector<Row> readMission(const std::string & file)
{
  std::ifstream in(file);
  if (!in) {throw std::runtime_error("cannot open mission file " + file);}
  std::vector<Row> rows;
  std::string line;
  while (std::getline(in, line)) {
    if (line.empty() || line[0] == '#' || line[0] == 'x') {continue;} 
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
    mode_ = declare_parameter("mission", std::string("indoor"));
    rate_hz_ = declare_parameter("rate_hz", 20.0);


    if (mode_ == "indoor") {
      if (!setupIndoor()) {return;}
    } else if (mode_ == "outdoor") {
      //TODO
    } else {
      throw std::runtime_error("parameter 'mission' must be indoor or outdoor");
    }
    timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / rate_hz_), [this] {tick();});
    log("WAIT_FOR_FCU");
  }

private:

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

    // --- racks: solid boxes, then inflation
    for (size_t i = 0; i + 5 < boxes.size(); i += 6) {
      map_.fillBox({boxes[i], boxes[i + 1], boxes[i + 2]}, {boxes[i + 3], boxes[i + 4], boxes[i + 5]});
    }
    map_.inflate(radius);

    publishViz(viz);
  }


  void publishViz(const nav_msgs::msg::Path & path)
  {
    auto latched = rclcpp::QoS(1).transient_local();
    path_pub_ = create_publisher<nav_msgs::msg::Path>("~/path", latched);
    cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("~/map", latched);
    path_pub_->publish(path);

    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.frame_id = "map";
    sensor_msgs::PointCloud2Modifier mod(cloud);
    mod.setPointCloud2FieldsByString(1, "xyz");
    mod.resize(map_.count(map_.raw));
    sensor_msgs::PointCloud2Iterator<float> x(cloud, "x"), y(cloud, "y"), z(cloud, "z");
    for (int k = 0; k < map_.nz; ++k) {
      for (int j = 0; j < map_.ny; ++j) {
        for (int i = 0; i < map_.nx; ++i) {
          if (!map_.raw[map_.id(i, j, k)]) {continue;}
          const V3 c = map_.centre(i, j, k);
          *x = c.x; *y = c.y; *z = c.z;
          ++x; ++y; ++z;
        }
      }
    }
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
      "/mavros/global_position/global", qos, [this](sensor_msgs::msg::NavSatFix::ConstSharedPtr m) {fix_ = *m;});
    sats_sub_ = create_subscription<std_msgs::msg::UInt32>(
      "/mavros/global_position/raw/satellites", qos, [this](std_msgs::msg::UInt32::ConstSharedPtr m) {sats_ = m->data;});
    hdg_sub_ = create_subscription<std_msgs::msg::Float64>(
      "/mavros/global_position/compass_hdg", qos, [this](std_msgs::msg::Float64::ConstSharedPtr m) {hdg_ = m->data;});
    relalt_sub_ = create_subscription<std_msgs::msg::Float64>(
      "/mavros/global_position/rel_alt", qos, [this](std_msgs::msg::Float64::ConstSharedPtr m) {rel_alt_ = m->data;});
    home_sub_ = create_subscription<mavros_msgs::msg::HomePosition>(  // latched topic
      "/mavros/home_position/home", rclcpp::QoS(1).transient_local(), [this](mavros_msgs::msg::HomePosition::ConstSharedPtr m) {
        if (!home_) {
          home_ = true;
          geo_.lat0 = m->geo.latitude;
          geo_.lon0 = m->geo.longitude;
          home_local_ = {m->position.x, m->position.y, m->position.z};
          RCLCPP_INFO(get_logger(), "home: lat %.7f lon %.7f alt %.2f, local (%.2f %.2f %.2f)",
            geo_.lat0, geo_.lon0, m->geo.altitude, home_local_.x, home_local_.y, home_local_.z);
        }
      });
    status_timer_ = create_wall_timer(1s, [this] {printStatus();});
  }

  VoxelMap map_;
  std::string phase_{"WAIT_FOR_FCU"};
  rclcpp::Time since_, t0_, last_call_{0, 0, RCL_ROS_TIME};

};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MissionNode>());
  rclcpp::shutdown();
  return 0;
}
