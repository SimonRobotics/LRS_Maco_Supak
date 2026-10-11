# lrs_mission

Assignment 1 in one ROS 2 node ([src/lrs_mission_node.cpp](src/lrs_mission_node.cpp)):

| Part | What |
|---|---|
| A1.1 | `map.pcd` → `pcl::VoxelGrid` downsample → dense 3D occupancy grid → racks filled → 3D inflation → A\* → line-of-sight shortcutting |
| A1.2 | indoor mission from a CSV: `takeoff`, `land`, `landtakeoff`, `yawN`, `hard` / `soft` precision |
| A1.3 | outdoor mission: GPS + compass, POI-locked square, figure-8 with tangent heading, return, land, abort |

## Build

```bash
cd ~/ros2_ws/src && ln -s ~/LRS-URK/lrs_mission .
cd ~/ros2_ws && colcon build --packages-select lrs_mission && source install/setup.bash
```

Dependencies: ROS 2 Jazzy, MAVROS, PCL (`sudo apt install libpcl-dev`).

## Run

Start the simulation in three terminals as in the [repository README](../README.md):
`scripts/run_gazebo.sh <world>`, `scripts/run_sitl.sh`, `scripts/run_mavros.sh`.

```bash
# Indoor (world fei_lrs_gazebo.world)
ros2 launch lrs_mission indoor.launch.py                                   # missions/exam_example.csv
ros2 launch lrs_mission indoor.launch.py mission_file:=/path/to/mission.csv

# Outdoor (world fei_lrs_outdoor.world)
ros2 launch lrs_mission outdoor.launch.py

# Abort: switch to LAND and stop sending setpoints
ros2 service call /lrs_mission/abort std_srvs/srv/Trigger
```

Planner only, no simulator needed:

```bash
ros2 launch lrs_mission indoor.launch.py plan_only:=true
ros2 run lrs_mission lrs_mission_node --ros-args --params-file install/lrs_mission/share/lrs_mission/config/indoor.yaml \
  -p map_file:=$PWD/install/lrs_mission/share/lrs_mission/maps/FEI_LRS_PCD/map.pcd \
  -p start:="[13.0, 7.0, 1.0]" -p goal:="[2.0, 9.7, 1.5]"
# occupancy query (or the "Publish Point" tool in RViz)
ros2 topic pub --once /lrs_mission/query geometry_msgs/msg/PointStamped "{point: {x: 4.0, y: 6.8, z: 1.5}}"
```

### RViz

Both launch files start RViz with [rviz/lrs_mission.rviz](rviz/lrs_mission.rviz). Pass `rviz:=false` to
skip it. Every topic is latched, so RViz can also be started later:

| Topic | Type | Shows |
|---|---|---|
| `/lrs_mission/map` | PointCloud2 | map voxels incl. rack boxes. The view is cropped at `viz_map_max_z` so the ceiling doesn't hide the inside |
| `/lrs_mission/path` | Path (blue) | indoor: planned path after shortcutting. Outdoor: commanded square + figure-8 |
| `/lrs_mission/flown_path` | Path (orange) | actual drone position, a pose every 5 cm, updated at 2 Hz |
| `/lrs_mission/waypoints` | MarkerArray | mission waypoints, red = `hard`, green = `soft`, labelled with number, precision and task |

Frame `map` is the frame of `map.pcd` indoors. The flown track is converted there from MAVROS
local ENU. Outdoors, `map` is the MAVROS local ENU frame.

Parameters: [config/indoor.yaml](config/indoor.yaml), [config/outdoor.yaml](config/outdoor.yaml).

## How it works

### Frames and angles

- **map**: the frame of `map.pcd` and of the mission CSV (Gazebo world of the hangar).
- **local**: MAVROS local ENU (`/mavros/local_position/pose`). Setpoints are sent here with
  `FRAME_LOCAL_NED`, and MAVROS converts them to NED.
- `local = Rz(90°) · map`. This was measured: the drone spawned at map (13, 7) reports local (−7, 13).
  The world uses `gazeboXYZToNED = roll 180°`, so Gazebo X = North and Y = West.
- Yaw in code: ENU radians (0 = East, counter-clockwise). The mission `yawN` is N degrees
  in the **map** frame, counter-clockwise from map +X, so the code adds 90°.
- Compass `compass_hdg`: 0 = North, clockwise; `yaw_ENU = 90° − heading`. The outdoor node
  prints both each second, and they agree.

### A1.1 Map and planning

- **Downsampling:** a 0.1 m `pcl::VoxelGrid` turns 620 311 points into 146 524. They go into a dense
  183 × 144 × 60 grid (1.58 M voxels, 1.6 MB). A query is O(1). Anything outside the grid counts as occupied.
- **Racks:** the point cloud only samples rack surfaces, so the gaps between shelf levels look
  free. Each rack row is filled as a solid box: the bounding box of its collision elements in
  `models/fei_lrs_racks/model.sdf`, from the floor up to 4.1 m (`rack_boxes`).
- **Inflation:** a sphere of `inflation_radius` is stamped around every occupied voxel, so it works
  the same in x, y and z. 0.5 m = 0.25 drone half-diagonal + 0.15 hard tolerance + 0.10 margin.
  At 0.5 m, 836 630 voxels are occupied; without inflation, 190 397.
- **A\*:** 26-connected. Edge cost is the Euclidean step length and the heuristic is the Euclidean
  distance to the goal. That heuristic never overestimates, so it is admissible and the path is
  optimal on the grid. When no path exists, the open list empties and the planner reports
  failure. A waypoint inside the inflation zone is moved to the nearest free voxel within 1 m.
- **Post-processing:** line-of-sight shortcutting. From each kept point, jump to the farthest later
  point reachable in a straight line that is collision-free (sampled every half voxel against the
  inflated grid). Every simplified segment is checked again.

`missions/exam_example.csv`, 0.1 m voxels, 0.5 m radius:

| Segment | A\* time | Points before → after |
|---|---|---|
| 1 → 2 | 9 ms | 50 → 2 |
| 2 → 3 | 6 ms | 45 → 4 |
| 3 → 4 (over the rack) | 20 ms | 69 → 9 |
| 4 → 5 | 9 ms | 69 → 3 |
| 5 → 6 | 3 ms | 61 → 4 |
| 6 → 7 | 12 ms | 114 → 4 |

### A1.2 Indoor mission

States are logged as `[state] ...`:
`WAIT_FOR_FCU → TAKEOFF → NAVIGATE → TASK_YAW → LAND → TAKEOFF → … → LAND → DONE`.
Every service response is checked, and failed calls are retried every second.

- **Mission:** all segments are planned at startup, then turned into a list of actions in the local frame.
- **Setpoints:** published at 20 Hz on `/mavros/setpoint_raw/local` (position + yaw).
  The setpoint is a *carrot*: it slides from the segment start to the target at `max_speed`
  (1 m/s). That bounds the speed, and the drone stays on the planned line instead of cutting
  corners. With a fixed far setpoint, ArduPilot flew at about 3 m/s and passed 0.23 m from a rack.
- **Precision:**
  - intermediate path points: pass-through at 0.5 m
  - `soft`: within 0.4 m, no stop
  - `hard`: within 0.15 m and below 0.15 m/s, held for 1 s
- **Heading:** along the segment. The node turns in place first if the turn is larger than 30°,
  and the yaw is unwrapped.
- **Takeoff:** GUIDED → arm → `cmd/takeoff` → wait until the altitude is reached.
  **Land:** `cmd/land` (LAND mode) → wait until ArduPilot disarms → wait 2 s.
  `landtakeoff` is land followed by takeoff, which sets GUIDED and re-arms.

### A1.3 Outdoor mission

- **Start checks:** before arming, the node waits for a GPS fix, `min_satellites` and the home position
  (`/mavros/home_position/home`, a latched topic).
- **Coordinates:** the POI is a parameter (offset from home, or lat/lon). The POI and the corners are
  converted to lat/lon and back to local metres with a flat-Earth (local tangent plane) approximation
  around home. Its error grows as d²/R, which is under 1 mm over this ~10 m mission.
- **Altitude:** commanded and checked as local z − home z (5 m above launch). `rel_alt` is printed for comparison.
- **Square:** the yaw is the bearing to the POI, recomputed every tick. Its derivative is sent as a
  yaw-rate feed-forward.
- **Figure-8:** `x = A sin t`, `y = A sin t cos t` with `t = ω·(now − t0)`, evaluated every tick.
  - Yaw is `atan2(cos 2t, cos t)`, with the analytic yaw rate as feed-forward.
  - The drone first flies to the centre and turns to the t = 0 tangent (45°).
  - ω = 0.3 rad/s gives a top speed of A·ω·√2 = 1.27 m/s and a 21 s lap.
- **Abort:** `~/abort` switches to LAND. If MAVROS disconnects or someone else changes the mode
  (e.g. the RC safety pilot), the node stops publishing setpoints and does not fight back.

### Results in SITL (headless gz + ArduPilot SITL + MAVROS)

| | |
|---|---|
| Indoor `exam_example.csv` | unattended, ~3 min. Hard waypoints settle within 0.05–0.12 m. Minimum clearance to walls/racks 0.32 m (crossing over the rack). Speed ≤ 1.2 m/s. Heading vs. travel direction: median 2.8° |
| Outdoor square | yaw vs. bearing to POI: median 1.4°, max 5.8° |
| Outdoor figure-8 | flown track within 0.05 m of the ideal curve (±3.02 × ±1.49 m). Yaw vs. commanded tangent: median 0.9° |
