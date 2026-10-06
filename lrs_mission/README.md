# lrs_mission

Assignment 1 package. Currently implements A1.1 §1 (map loading into a 3D voxel grid)
and §2 (obstacle inflation).

## Build and test

```bash
cd ~/ros2_ws/src && ln -s ~/LRS-URK/lrs_mission .   # or clone the repo into src/
cd ~/ros2_ws && colcon build --packages-select lrs_mission
colcon test --packages-select lrs_mission && colcon test-result --verbose
```

Dependencies: ROS 2 Jazzy, PCL (`sudo apt install libpcl-dev`).

## Run

```bash
ros2 launch lrs_mission voxel_map.launch.py                       # radius 0.5 m
ros2 launch lrs_mission voxel_map.launch.py inflation_radius:=0.3
```

The node logs the cloud bounding box, grid size, voxel counts, timings and the answers to
`query_points` from [config/voxel_map.yaml](config/voxel_map.yaml). The simulator does not need
to be running.

More queries and a different radius while it runs:

```bash
ros2 topic pub --once /voxel_map/query geometry_msgs/msg/PointStamped "{point: {x: 4.0, y: 6.8, z: 1.5}}"
ros2 param set /voxel_map inflation_radius 0.3
```

In RViz (fixed frame `map`), add PointCloud2 displays for `/voxel_map/occupied_raw` and
`/voxel_map/occupied_inflated` (Durability: Transient Local). The *Publish Point* tool also
sends queries (`/clicked_point`).

## Design

**Map representation** — [voxel_map.hpp](include/lrs_mission/voxel_map.hpp). The PCD is
loaded with PCL and stored as a dense 3D array of voxels (default 0.1 m, 183 × 144 × 60 ≈
1.6 M voxels for the hangar). A voxel is occupied when at least one map point falls into it.
A dense grid makes a query O(1) and is small enough here (1.6 MB per layer); an octree would
only pay off on a much larger map. Anything outside the grid is reported as occupied, so a
planner cannot leave the map.

**Racks** — the point cloud only samples rack surfaces, leaving free-looking gaps between
shelf levels. Each rack row is filled as a solid box (`fill_boxes`): the bounding box of
all collision elements in `models/fei_lrs_racks/model.sdf` for that row, from the floor to 4.1 m.

**Inflation** — two layers are kept: raw and inflated. Inflation stamps a spherical kernel
(every voxel whose centre is within `inflation_radius` of an obstacle voxel centre, the
same in x, y and z) around each obstacle *surface* voxel. Interior voxels are skipped since
the surface spheres already cover their neighbourhood. Because the raw layer is kept, the radius
can be changed at runtime without reloading the map (~50–200 ms).

**Radius** — 0.5 m = 0.25 m (half-diagonal of the drone's 0.347 × 0.347 m collision box)
+ 0.15 m (`hard` waypoint tolerance) + 0.10 m (overshoot / discretisation margin).

| Radius | Occupied voxels | Inflation time |
|---|---|---|
| 0.0 m (raw + racks) | 193 547 (12.2 %) | — |
| 0.3 m | 616 437 (39.0 %) | 56 ms |
| 0.5 m | 846 026 (53.5 %) | 191 ms |
