// Loads the hangar point cloud into a VoxelMap, fills the known rack volumes,
// inflates obstacles and serves occupancy queries.
//
// Queries:
//   - parameter `query_points` (x,y,z triples), answered once at startup
//   - topic `~/query` (geometry_msgs/PointStamped) and RViz "Publish Point"
//     (`/clicked_point`), answered in the log
// Visualisation: `~/occupied_raw` and `~/occupied_inflated` (PointCloud2, latched).
// `inflation_radius` can be changed at runtime with `ros2 param set`.

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/point_stamped.hpp"
#include "lrs_mission/voxel_map.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

using namespace std::chrono_literals;
using lrs_mission::Vec3;

class VoxelMapNode : public rclcpp::Node
{
public:
  VoxelMapNode()
  : Node("voxel_map")
  {
    const auto map_file = declare_parameter<std::string>("map_file", "");
    const auto resolution = declare_parameter<double>("resolution", 0.1);
    const auto padding = declare_parameter<double>("padding", 0.0);
    const auto radius = declare_parameter<double>("inflation_radius", 0.5);
    // Solid boxes, 6 numbers each: xmin ymin zmin xmax ymax zmax [m, map frame].
    const auto boxes = declare_parameter<std::vector<double>>("fill_boxes", std::vector<double>{});
    const auto queries = declare_parameter<std::vector<double>>("query_points", std::vector<double>{});
    frame_id_ = declare_parameter<std::string>("frame_id", "map");

    if (map_file.empty()) {
      throw std::runtime_error("parameter 'map_file' is not set");
    }
    if (boxes.size() % 6 != 0) {
      throw std::runtime_error("'fill_boxes' must contain 6 numbers per box");
    }
    if (queries.size() % 3 != 0) {
      throw std::runtime_error("'query_points' must contain 3 numbers per point");
    }

    auto t0 = std::chrono::steady_clock::now();
    map_.loadPcd(map_file, resolution, padding);
    for (std::size_t i = 0; i < boxes.size(); i += 6) {
      map_.fillBox({{boxes[i], boxes[i + 1], boxes[i + 2]}, {boxes[i + 3], boxes[i + 4], boxes[i + 5]}});
    }
    const double load_ms = msSince(t0);
    t0 = std::chrono::steady_clock::now();
    map_.inflate(radius);
    const double inflate_ms = msSince(t0);

    const auto & s = map_.size();
    const auto & cmin = map_.cloudMin();
    const auto & cmax = map_.cloudMax();
    const auto gmax = map_.gridMax();
    RCLCPP_INFO(get_logger(), "map: %s (%zu points)", map_file.c_str(), map_.pointCount());
    RCLCPP_INFO(
      get_logger(), "cloud bounding box: [%.2f, %.2f, %.2f] -> [%.2f, %.2f, %.2f] m",
      cmin.x, cmin.y, cmin.z, cmax.x, cmax.y, cmax.z);
    RCLCPP_INFO(
      get_logger(), "grid: %d x %d x %d = %zu voxels @ %.3f m, bounds [%.2f, %.2f, %.2f] -> [%.2f, %.2f, %.2f]",
      s[0], s[1], s[2], map_.voxelCount(), map_.resolution(),
      map_.origin().x, map_.origin().y, map_.origin().z, gmax.x, gmax.y, gmax.z);
    RCLCPP_INFO(
      get_logger(), "occupied: %zu raw (incl. %zu fill boxes), load %.0f ms",
      map_.occupiedRawCount(), boxes.size() / 6, load_ms);
    logInflation(inflate_ms);

    for (std::size_t i = 0; i < queries.size(); i += 3) {
      logQuery({queries[i], queries[i + 1], queries[i + 2]});
    }

    const auto latched = rclcpp::QoS(1).transient_local();
    raw_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("~/occupied_raw", latched);
    inflated_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("~/occupied_inflated", latched);
    publishClouds();

    auto on_query = [this](geometry_msgs::msg::PointStamped::ConstSharedPtr msg) {
        logQuery({msg->point.x, msg->point.y, msg->point.z});
      };
    query_sub_ = create_subscription<geometry_msgs::msg::PointStamped>("~/query", 10, on_query);
    clicked_sub_ = create_subscription<geometry_msgs::msg::PointStamped>("/clicked_point", 10, on_query);

    // Validate a new radius before it is accepted, apply it afterwards.
    param_check_ = add_on_set_parameters_callback(
      [](const std::vector<rclcpp::Parameter> & params) {
        rcl_interfaces::msg::SetParametersResult res;
        res.successful = true;
        for (const auto & p : params) {
          if (p.get_name() == "inflation_radius" && p.as_double() < 0.0) {
            res.successful = false;
            res.reason = "inflation_radius must be >= 0";
          }
        }
        return res;
      });
    param_apply_ = add_post_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & params) {
        for (const auto & p : params) {
          if (p.get_name() == "inflation_radius") {
            const auto t = std::chrono::steady_clock::now();
            map_.inflate(p.as_double());
            logInflation(msSince(t));
            publishClouds();
          }
        }
      });
  }

private:
  static double msSince(std::chrono::steady_clock::time_point t)
  {
    return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t).count();
  }

  void logInflation(double ms)
  {
    RCLCPP_INFO(
      get_logger(), "inflation radius %.3f m: %zu occupied voxels (%.1f %%), %.0f ms",
      map_.inflationRadius(), map_.occupiedInflatedCount(),
      100.0 * map_.occupiedInflatedCount() / map_.voxelCount(), ms);
  }

  void logQuery(const Vec3 & p)
  {
    if (!map_.inBounds(p)) {
      RCLCPP_INFO(get_logger(), "query (%.2f, %.2f, %.2f): OUT OF MAP (treated as occupied)", p.x, p.y, p.z);
      return;
    }
    const bool raw = map_.isOccupiedRaw(p);
    const bool inflated = map_.isOccupied(p);
    RCLCPP_INFO(
      get_logger(), "query (%.2f, %.2f, %.2f): %s", p.x, p.y, p.z,
      raw ? "OCCUPIED (obstacle)" : inflated ? "OCCUPIED (inflation zone)" : "FREE");
  }

  sensor_msgs::msg::PointCloud2 toCloud(const std::vector<Vec3> & pts) const
  {
    sensor_msgs::msg::PointCloud2 msg;
    msg.header.frame_id = frame_id_;
    msg.header.stamp = now();
    sensor_msgs::PointCloud2Modifier mod(msg);
    mod.setPointCloud2FieldsByString(1, "xyz");
    mod.resize(pts.size());
    sensor_msgs::PointCloud2Iterator<float> x(msg, "x"), y(msg, "y"), z(msg, "z");
    for (const auto & p : pts) {
      *x = static_cast<float>(p.x);
      *y = static_cast<float>(p.y);
      *z = static_cast<float>(p.z);
      ++x; ++y; ++z;
    }
    return msg;
  }

  void publishClouds()
  {
    raw_pub_->publish(toCloud(map_.occupiedCentres(false)));
    inflated_pub_->publish(toCloud(map_.occupiedCentres(true)));
  }

  lrs_mission::VoxelMap map_;
  std::string frame_id_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr raw_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr inflated_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr query_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr clicked_sub_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_check_;
  rclcpp::node_interfaces::PostSetParametersCallbackHandle::SharedPtr param_apply_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VoxelMapNode>());
  rclcpp::shutdown();
  return 0;
}
