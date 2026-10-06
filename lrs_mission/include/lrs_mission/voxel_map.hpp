#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace lrs_mission
{

struct Vec3
{
  double x{0.0}, y{0.0}, z{0.0};
};

// Axis-aligned box in map coordinates, used for obstacles known from the world
// file (the shelving racks) that the point cloud does not represent as solid.
struct Box
{
  Vec3 min;
  Vec3 max;
};

// Dense 3D occupancy grid built from a point cloud.
//
// Two layers are kept:
//   - raw:      voxels hit by at least one map point, plus filled boxes
//   - inflated: raw grown by a sphere of `inflation_radius`
// Keeping the raw layer lets the radius be changed without reloading the map.
// Everything outside the grid bounds counts as occupied, so a planner can never
// leave the map.
class VoxelMap
{
public:
  VoxelMap() = default;

  // Loads a PCD file and voxelises it with cubic voxels of size `resolution` [m].
  // `padding` [m] of free space is added around the cloud's bounding box.
  // Throws std::runtime_error on failure.
  void loadPcd(const std::string & path, double resolution, double padding = 0.0);

  // Builds the grid from raw points (used by loadPcd and by the tests).
  void build(const std::vector<Vec3> & points, double resolution, double padding = 0.0);

  // Marks every voxel whose centre lies inside the box as occupied (raw layer).
  // Call inflate() afterwards.
  void fillBox(const Box & box);

  // Recomputes the inflated layer from the raw layer. Radius in metres, >= 0.
  void inflate(double radius);

  // Occupancy queries in map coordinates [m].
  bool isOccupiedRaw(const Vec3 & p) const;
  bool isOccupied(const Vec3 & p) const;  // against the inflated layer
  bool inBounds(const Vec3 & p) const;

  // Index-space access for the planner.
  bool worldToIndex(const Vec3 & p, std::array<int, 3> & idx) const;
  Vec3 indexToWorld(const std::array<int, 3> & idx) const;  // voxel centre
  bool isOccupiedIndex(int ix, int iy, int iz) const;
  bool isOccupiedRawIndex(int ix, int iy, int iz) const;

  const std::array<int, 3> & size() const { return size_; }
  double resolution() const { return resolution_; }
  double inflationRadius() const { return inflation_radius_; }
  const Vec3 & origin() const { return origin_; }       // min corner of the grid
  Vec3 gridMax() const;                                  // max corner of the grid
  const Vec3 & cloudMin() const { return cloud_min_; }  // bounding box of the input cloud
  const Vec3 & cloudMax() const { return cloud_max_; }
  std::size_t pointCount() const { return point_count_; }
  std::size_t voxelCount() const { return grid_raw_.size(); }
  std::size_t occupiedRawCount() const { return occupied_raw_count_; }
  std::size_t occupiedInflatedCount() const { return occupied_inflated_count_; }

  // Centres of occupied voxels, for visualisation.
  std::vector<Vec3> occupiedCentres(bool inflated) const;

private:
  std::size_t linear(int ix, int iy, int iz) const
  {
    return (static_cast<std::size_t>(iz) * size_[1] + iy) * size_[0] + ix;
  }
  bool indexValid(int ix, int iy, int iz) const
  {
    return ix >= 0 && iy >= 0 && iz >= 0 && ix < size_[0] && iy < size_[1] && iz < size_[2];
  }

  double resolution_{0.1};
  double inflation_radius_{0.0};
  Vec3 origin_;
  Vec3 cloud_min_;
  Vec3 cloud_max_;
  std::array<int, 3> size_{0, 0, 0};
  std::size_t point_count_{0};
  std::size_t occupied_raw_count_{0};
  std::size_t occupied_inflated_count_{0};
  std::vector<std::uint8_t> grid_raw_;
  std::vector<std::uint8_t> grid_inflated_;
};

}  // namespace lrs_mission
