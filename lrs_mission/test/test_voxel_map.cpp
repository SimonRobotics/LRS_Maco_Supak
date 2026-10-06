#include <gtest/gtest.h>

#include <vector>

#include "lrs_mission/voxel_map.hpp"

using lrs_mission::Box;
using lrs_mission::Vec3;
using lrs_mission::VoxelMap;

namespace
{

// 4 m cube with a single obstacle point in the middle.
VoxelMap singlePointMap()
{
  VoxelMap map;
  map.build({{0, 0, 0}, {4, 4, 4}, {2.05, 2.05, 2.05}}, 0.1);
  return map;
}

}  // namespace

TEST(VoxelMap, BoundsAndCounts)
{
  auto map = singlePointMap();
  EXPECT_EQ(map.size()[0], 41);
  EXPECT_EQ(map.voxelCount(), 41u * 41u * 41u);
  EXPECT_EQ(map.occupiedRawCount(), 3u);
  EXPECT_TRUE(map.isOccupied({2.05, 2.05, 2.05}));
  EXPECT_FALSE(map.isOccupied({1.0, 1.0, 1.0}));
}

TEST(VoxelMap, OutsideIsOccupied)
{
  auto map = singlePointMap();
  EXPECT_TRUE(map.isOccupied({-0.5, 1.0, 1.0}));
  EXPECT_TRUE(map.isOccupied({1.0, 1.0, 5.0}));
}

TEST(VoxelMap, InflationIsSphericalIn3D)
{
  auto map = singlePointMap();
  map.inflate(0.3);
  // Along every axis, including straight up and down.
  EXPECT_TRUE(map.isOccupied({2.35, 2.05, 2.05}));
  EXPECT_TRUE(map.isOccupied({2.05, 2.05, 1.75}));
  EXPECT_TRUE(map.isOccupied({2.05, 2.05, 2.35}));
  EXPECT_FALSE(map.isOccupied({2.45, 2.05, 2.05}));
  EXPECT_FALSE(map.isOccupied({2.05, 2.05, 2.45}));
  // Corner of the 0.3 m cube is 0.52 m away: outside the sphere.
  EXPECT_FALSE(map.isOccupied({2.35, 2.35, 2.35}));
  // Raw layer is untouched.
  EXPECT_FALSE(map.isOccupiedRaw({2.35, 2.05, 2.05}));
}

TEST(VoxelMap, RadiusCanBeChanged)
{
  auto map = singlePointMap();
  map.inflate(0.5);
  const auto big = map.occupiedInflatedCount();
  EXPECT_TRUE(map.isOccupied({2.55, 2.05, 2.05}));
  map.inflate(0.2);
  EXPECT_LT(map.occupiedInflatedCount(), big);
  EXPECT_FALSE(map.isOccupied({2.55, 2.05, 2.05}));
  map.inflate(0.0);
  EXPECT_EQ(map.occupiedInflatedCount(), map.occupiedRawCount());
}

TEST(VoxelMap, FillBoxIsSolidAndInflated)
{
  auto map = singlePointMap();
  map.fillBox({{0.5, 0.5, 0.0}, {1.5, 1.0, 3.0}});
  map.inflate(0.2);
  EXPECT_TRUE(map.isOccupiedRaw({1.0, 0.75, 1.5}));   // inside
  EXPECT_TRUE(map.isOccupied({1.0, 1.15, 1.5}));      // within radius
  EXPECT_TRUE(map.isOccupied({1.0, 0.75, 3.15}));     // above the top face
  EXPECT_FALSE(map.isOccupied({1.0, 1.35, 1.5}));
}

#ifdef MAP_FILE
// Same rack boxes and radius as config/voxel_map.yaml.
TEST(VoxelMap, HangarMap)
{
  VoxelMap map;
  map.loadPcd(MAP_FILE, 0.1);
  EXPECT_EQ(map.pointCount(), 620311u);
  map.fillBox({{1.07, 6.29, 0.0}, {7.57, 7.35, 4.10}});
  map.fillBox({{1.07, 10.83, 0.0}, {7.57, 11.89, 4.10}});
  map.inflate(0.5);

  // Gaps between the shelf levels are solid.
  EXPECT_TRUE(map.isOccupiedRaw({4.0, 6.8, 1.5}));
  EXPECT_TRUE(map.isOccupiedRaw({4.0, 11.4, 1.5}));
  // Floor is inflated upwards.
  EXPECT_TRUE(map.isOccupied({10.0, 4.0, 0.3}));

  // Takeoff point and the goal waypoints of both example missions are free.
  const std::vector<Vec3> waypoints = {
    {13.00, 7.00, 1.00}, {8.65, 2.02, 1.00}, {4.84, 5.37, 2.00}, {2.08, 9.74, 1.75},
    {8.84, 6.90, 2.00}, {2.81, 8.15, 1.50}, {13.60, 1.50, 2.00}, {13.60, 11.50, 1.50},
    {2.81, 8.15, 1.75}, {4.84, 9.74, 1.25}, {4.84, 5.37, 1.00}};
  for (const auto & w : waypoints) {
    EXPECT_FALSE(map.isOccupied(w)) << w.x << " " << w.y << " " << w.z;
  }
}
#endif
