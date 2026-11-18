# lrs_localization

Assignment 2, topic **drone localisation**
([assignment](../assignments/assignment2/localization.md)): an eight-state Kalman filter
that fuses noisy GPS, drifting visual odometry and a biased yaw gyro.

| Section | Status |
|---|---|
| §1 Sensor acquisition and characterisation | node: inputs, frames, stamps, rates |
| §2 Eight-state KF/EKF | – |
| §3 Missing and bad measurements | – |
| §4 Flight using estimated localisation | – |

## Build

From the repository root:

```bash
colcon build --packages-select localization_sensor_sim lrs_localization
source install/setup.bash
colcon test --packages-select lrs_localization && colcon test-result --verbose
```

Dependencies: ROS 2 Humble or Jazzy, Gazebo Harmonic, ArduPilot SITL, MAVROS
(see the [repository README](../README.md)).

## Run

Five terminals, all from the repository root:

```bash
# 1 Gazebo, dedicated localisation world, explicit seed
gz sim -r -v 4 --seed 42 "$PWD/worlds/fei_lrs_gazebo_localization.world"
# 2 ArduPilot SITL (MAVProxy lives in the ArduPilot venv)
source ~/venv-ardupilot/bin/activate && scripts/run_sitl.sh
# 3 MAVROS
scripts/run_mavros.sh
# 4 sensor simulator, same seed as Gazebo
source install/setup.bash && ros2 launch localization_sensor_sim localization_sensors.launch.py seed:=42
# 5 estimator
source install/setup.bash && ros2 launch lrs_localization localization.launch.py
```

Parameters: [config/localization.yaml](config/localization.yaml).

## §1 Sensor acquisition

The node [`localization_node`](src/localization_node.cpp) subscribes to exactly three topics.
Nothing else reaches the estimator, in particular no ground truth and no MAVROS pose.

| Topic | Type | Expected frame | Used fields |
|---|---|---|---|
| `/localization/gps` | `geometry_msgs/PoseWithCovarianceStamped` | `map` | position |
| `/localization/vo` | `nav_msgs/Odometry` | `vo_odom` → child `base_link` | pose, yaw, body-frame twist |
| `/localization/imu` | `sensor_msgs/Imu` | `base_link` | `angular_velocity.z` only |

- **QoS:** `rclcpp::SensorDataQoS()` (best effort), matching the sensor simulator. A reliable
  subscriber would be incompatible and would receive nothing.
- **Time:** every message is handled at its own `header.stamp` (simulation time,
  `use_sim_time: true`), never at its arrival time.
- **Checks** ([`sensor_channel.hpp`](include/lrs_localization/sensor_channel.hpp)), before a
  message may be used:
  - wrong `frame_id` / `child_frame_id` → ignored, error logged (throttled);
  - zero, duplicate or out-of-order stamp → dropped and counted;
  - stamp more than `reset_jump_s` in the past → simulation reset, reported once.
- **Report:** every `report_period_s` the node prints one line, never one per message. The
  rate is computed from header stamps, so it stays at 5 / 30 / 100 Hz even when Gazebo runs
  below real time and `ros2 topic hz` (wall time) shows less:

  ```text
  [sensors] gps 5.0 Hz (gap 0.20 s) | vo 30.0 Hz (gap 0.04 s) | imu 100.0 Hz (gap 0.01 s) | mean wz -0.0171 rad/s | dropped 0, wrong frame 0
  ```

  `gap` is the largest stamp difference since the previous report, so a sensor outage shows
  up as e.g. `gps 5.0 Hz (gap 7.20 s)`. `mean wz` is the average gyro rate; while the drone
  is not turning it is the gyro bias (about −0.017 rad/s with seed 42).

Unit tests for the checks: [`test/test_sensor_channel.cpp`](test/test_sensor_channel.cpp).
