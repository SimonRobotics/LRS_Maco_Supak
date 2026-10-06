#include "lrs_mission/voxel_map.hpp"

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace lrs_mission
{

void VoxelMap::loadPcd(const std::string & path, double resolution, double padding)
{
  pcl::PointCloud<pcl::PointXYZ> cloud;
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(path, cloud) != 0) {
    throw std::runtime_error("cannot read PCD file: " + path);
  }

  std::vector<Vec3> points;
  points.reserve(cloud.size());
  for (const auto & pt : cloud) {
    if (std::isfinite(pt.x) && std::isfinite(pt.y) && std::isfinite(pt.z)) {
      points.push_back({pt.x, pt.y, pt.z});
    }
  }
  build(points, resolution, padding);
}

void VoxelMap::build(const std::vector<Vec3> & points, double resolution, double padding)
{
  if (resolution <= 0.0) {
    throw std::invalid_argument("resolution must be positive");
  }
  if (points.empty()) {
    throw std::runtime_error("point cloud is empty");
  }

  constexpr double inf = std::numeric_limits<double>::infinity();
  Vec3 lo{inf, inf, inf};
  Vec3 hi{-inf, -inf, -inf};
  for (const auto & p : points) {
    lo = {std::min(lo.x, p.x), std::min(lo.y, p.y), std::min(lo.z, p.z)};
    hi = {std::max(hi.x, p.x), std::max(hi.y, p.y), std::max(hi.z, p.z)};
  }

  resolution_ = resolution;
  cloud_min_ = lo;
  cloud_max_ = hi;
  point_count_ = points.size();
  origin_ = {lo.x - padding, lo.y - padding, lo.z - padding};
  // +1 so that points lying exactly on the max face still get a voxel.
  size_ = {
    static_cast<int>(std::floor((hi.x + padding - origin_.x) / resolution)) + 1,
    static_cast<int>(std::floor((hi.y + padding - origin_.y) / resolution)) + 1,
    static_cast<int>(std::floor((hi.z + padding - origin_.z) / resolution)) + 1};

  grid_raw_.assign(static_cast<std::size_t>(size_[0]) * size_[1] * size_[2], 0);
  occupied_raw_count_ = 0;
  std::array<int, 3> idx;
  for (const auto & p : points) {
    if (worldToIndex(p, idx)) {
      auto & cell = grid_raw_[linear(idx[0], idx[1], idx[2])];
      if (!cell) {
        cell = 1;
        ++occupied_raw_count_;
      }
    }
  }

  // Until inflate() is called, the inflated layer is the raw layer.
  grid_inflated_ = grid_raw_;
  occupied_inflated_count_ = occupied_raw_count_;
  inflation_radius_ = 0.0;
}

void VoxelMap::fillBox(const Box & box)
{
  // Voxel i covers [origin + i*res, origin + (i+1)*res); take every voxel whose
  // centre is inside the box.
  auto first = [this](double v, double o) {
      return static_cast<int>(std::ceil((v - o) / resolution_ - 0.5));
    };
  auto last = [this](double v, double o) {
      return static_cast<int>(std::floor((v - o) / resolution_ - 0.5));
    };
  const int x0 = std::max(0, first(box.min.x, origin_.x));
  const int y0 = std::max(0, first(box.min.y, origin_.y));
  const int z0 = std::max(0, first(box.min.z, origin_.z));
  const int x1 = std::min(size_[0] - 1, last(box.max.x, origin_.x));
  const int y1 = std::min(size_[1] - 1, last(box.max.y, origin_.y));
  const int z1 = std::min(size_[2] - 1, last(box.max.z, origin_.z));

  for (int iz = z0; iz <= z1; ++iz) {
    for (int iy = y0; iy <= y1; ++iy) {
      for (int ix = x0; ix <= x1; ++ix) {
        auto & cell = grid_raw_[linear(ix, iy, iz)];
        if (!cell) {
          cell = 1;
          ++occupied_raw_count_;
        }
      }
    }
  }
}

void VoxelMap::inflate(double radius)
{
  if (radius < 0.0) {
    throw std::invalid_argument("inflation radius must be >= 0");
  }
  inflation_radius_ = radius;
  grid_inflated_ = grid_raw_;

  // Spherical structuring element: every offset whose voxel centre is within
  // `radius` of the obstacle voxel centre. Same in all three axes -> true 3D.
  // Radius in voxels; the epsilon keeps e.g. 0.3 / 0.1 = 2.9999... from losing a voxel.
  const double rv = radius / resolution_ + 1e-9;
  const int r = static_cast<int>(std::floor(rv));
  const double r2 = rv * rv;
  std::vector<std::array<int, 3>> offsets;
  for (int dz = -r; dz <= r; ++dz) {
    for (int dy = -r; dy <= r; ++dy) {
      for (int dx = -r; dx <= r; ++dx) {
        if (dx * dx + dy * dy + dz * dz <= r2 && (dx || dy || dz)) {
          offsets.push_back({dx, dy, dz});
        }
      }
    }
  }

  // Only voxels on the obstacle surface need stamping: the sphere around an
  // interior voxel is covered by the spheres around the surface voxels.
  auto rawAt = [this](int ix, int iy, int iz) {
      return !indexValid(ix, iy, iz) || grid_raw_[linear(ix, iy, iz)];
    };
  for (int iz = 0; iz < size_[2]; ++iz) {
    for (int iy = 0; iy < size_[1]; ++iy) {
      for (int ix = 0; ix < size_[0]; ++ix) {
        if (!grid_raw_[linear(ix, iy, iz)]) {
          continue;
        }
        const bool surface =
          !rawAt(ix - 1, iy, iz) || !rawAt(ix + 1, iy, iz) ||
          !rawAt(ix, iy - 1, iz) || !rawAt(ix, iy + 1, iz) ||
          !rawAt(ix, iy, iz - 1) || !rawAt(ix, iy, iz + 1);
        if (!surface) {
          continue;
        }
        for (const auto & o : offsets) {
          const int jx = ix + o[0], jy = iy + o[1], jz = iz + o[2];
          if (indexValid(jx, jy, jz)) {
            grid_inflated_[linear(jx, jy, jz)] = 1;
          }
        }
      }
    }
  }

  occupied_inflated_count_ = static_cast<std::size_t>(
    std::count(grid_inflated_.begin(), grid_inflated_.end(), std::uint8_t{1}));
}

bool VoxelMap::worldToIndex(const Vec3 & p, std::array<int, 3> & idx) const
{
  idx = {
    static_cast<int>(std::floor((p.x - origin_.x) / resolution_)),
    static_cast<int>(std::floor((p.y - origin_.y) / resolution_)),
    static_cast<int>(std::floor((p.z - origin_.z) / resolution_))};
  return indexValid(idx[0], idx[1], idx[2]);
}

Vec3 VoxelMap::indexToWorld(const std::array<int, 3> & idx) const
{
  return {
    origin_.x + (idx[0] + 0.5) * resolution_,
    origin_.y + (idx[1] + 0.5) * resolution_,
    origin_.z + (idx[2] + 0.5) * resolution_};
}

Vec3 VoxelMap::gridMax() const
{
  return {
    origin_.x + size_[0] * resolution_,
    origin_.y + size_[1] * resolution_,
    origin_.z + size_[2] * resolution_};
}

bool VoxelMap::inBounds(const Vec3 & p) const
{
  std::array<int, 3> idx;
  return worldToIndex(p, idx);
}

bool VoxelMap::isOccupiedIndex(int ix, int iy, int iz) const
{
  return !indexValid(ix, iy, iz) || grid_inflated_[linear(ix, iy, iz)];
}

bool VoxelMap::isOccupiedRawIndex(int ix, int iy, int iz) const
{
  return !indexValid(ix, iy, iz) || grid_raw_[linear(ix, iy, iz)];
}

bool VoxelMap::isOccupied(const Vec3 & p) const
{
  std::array<int, 3> idx;
  return !worldToIndex(p, idx) || grid_inflated_[linear(idx[0], idx[1], idx[2])];
}

bool VoxelMap::isOccupiedRaw(const Vec3 & p) const
{
  std::array<int, 3> idx;
  return !worldToIndex(p, idx) || grid_raw_[linear(idx[0], idx[1], idx[2])];
}

std::vector<Vec3> VoxelMap::occupiedCentres(bool inflated) const
{
  const auto & grid = inflated ? grid_inflated_ : grid_raw_;
  std::vector<Vec3> out;
  out.reserve(inflated ? occupied_inflated_count_ : occupied_raw_count_);
  for (int iz = 0; iz < size_[2]; ++iz) {
    for (int iy = 0; iy < size_[1]; ++iy) {
      for (int ix = 0; ix < size_[0]; ++ix) {
        if (grid[linear(ix, iy, iz)]) {
          out.push_back(indexToWorld({ix, iy, iz}));
        }
      }
    }
  }
  return out;
}

}  // namespace lrs_mission
