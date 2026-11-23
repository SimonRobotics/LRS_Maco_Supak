# lrs_localization

Assignment 2, topic **drone localisation**
([assignment](../assignments/assignment2/localization.md)): an eight-state Kalman filter
that fuses noisy GPS, drifting visual odometry and a biased yaw gyro.

| Section | Status |
|---|---|
| §1 Sensor acquisition and characterisation | done — [characterisation](../docs/a2_charakterizacia_senzorov.md) |
| §2 Eight-state KF/EKF | prediction (constant velocity, gyro yaw) + GPS update; yaw/VO updates next |
| §3 Missing and bad measurements | – |
| §4 Flight using estimated localisation | – |

## Build

From the repository root:

```bash
colcon build --packages-select localization_sensor_sim lrs_localization
source install/setup.bash
colcon test --packages-select lrs_localization && colcon test-result --verbose
```

Dependencies: ROS 2 Humble or Jazzy, Eigen 3, Gazebo Harmonic, ArduPilot SITL, MAVROS
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

**Output: `/lrs_localization/odom`** (`nav_msgs/Odometry`), frame **`map`** (ENU, the GPS
frame), child `base_link`. Pose = x, y, z and yaw with their covariance from the filter's `P`
(roll/pitch are not estimated, variance 10⁶). Twist = velocity in `base_link` (REP 105) and the
bias-corrected yaw rate. Published on every gyro sample (100 Hz).

Parameters: [config/localization.yaml](config/localization.yaml).

### Without the simulator: replay a bag

The filter only uses header stamps, so a recorded sensor bag can be replayed into it — faster
than real time — and compared with the truth that is in the same bag:

```bash
lrs_localization/scripts/replay.sh bags/a2_flight_seed42 /tmp/replay_flight
python3 lrs_localization/scripts/evaluate.py /tmp/replay_flight --out /tmp/figures
```

[`evaluate.py`](scripts/evaluate.py) is an offline evaluation tool: it is the only place besides
[`characterise.py`](scripts/characterise.py) that reads `/evaluation/ground_truth`, and it sends
nothing back.

## §1 Sensor acquisition

The node [`localization_node`](src/localization_node.cpp) subscribes to exactly three topics.
Nothing else reaches the estimator, in particular no ground truth and no MAVROS pose.

| Topic | Type | Expected frame | Used fields |
|---|---|---|---|
| `/localization/gps` | `geometry_msgs/PoseWithCovarianceStamped` | `map` | position, its covariance |
| `/localization/vo` | `nav_msgs/Odometry` | `vo_odom` → child `base_link` | yaw (initialisation so far) |
| `/localization/imu` | `sensor_msgs/Imu` | `base_link` | `angular_velocity.z` only |

- **QoS:** `rclcpp::SensorDataQoS()` (best effort), matching the sensor simulator. A reliable
  subscriber would be incompatible and would receive nothing.
- **Time:** every message is handled at its own `header.stamp` (simulation time,
  `use_sim_time: true`), never at its arrival time.
- **Checks** ([`sensor_channel.hpp`](include/lrs_localization/sensor_channel.hpp)), before a
  message may be used:
  - wrong `frame_id` / `child_frame_id` → ignored, error logged (throttled);
  - zero, duplicate or out-of-order stamp → dropped and counted;
  - stamp more than `reset_jump_s` in the past → simulation reset: reported once and the
    filter starts over from the next GPS fix.
- **Report:** every `report_period_s` the node prints one line, never one per message. The
  rate is computed from header stamps, so it stays at 5 / 30 / 100 Hz even when Gazebo runs
  below real time and `ros2 topic hz` (wall time) shows less:

  ```text
  [sensors] gps 5.0 Hz (gap 0.20 s) | vo 30.0 Hz (gap 0.04 s) | imu 100.0 Hz (gap 0.01 s) | mean wz -0.0171 rad/s | dropped 0, wrong frame 0
  ```

  `gap` is the largest stamp difference since the previous report, so a sensor outage shows
  up as e.g. `gps 5.0 Hz (gap 7.20 s)`. `mean wz` is the average gyro rate; while the drone
  is not turning it is the gyro bias (about −0.017 rad/s with seed 42).

Recordings, figures and numbers: [docs/a2_charakterizacia_senzorov.md](../docs/a2_charakterizacia_senzorov.md).

## §2 Filter

Implementation: [`ekf.hpp`](include/lrs_localization/ekf.hpp) (no ROS, unit tested), used by the node.

**State** (ENU, frame `map`), in the mandatory order:

```text
x = [x, y, z, vx, vy, vz, yaw, b]^T      v in ENU, yaw in [-pi, pi), b = gyro z bias
```

**Prediction** to time `t` with the measured gyro rate `w`, `dt = t − t_filter` from header stamps:

```text
p   <- p + v dt                    F = I + dt in (p, v) and -dt in (yaw, b)
v   <- v                           (the model is linear in x, F is exact)
yaw <- wrap(yaw + (w - b) dt)
b   <- b
P   <- F P F^T + Q(dt)
```

`Q(dt)` is the exact discretisation of white noise with densities `q_acc_xy`, `q_acc_z` on the
acceleration (per axis `[q dt³/3, q dt²/2; q dt²/2, q dt]` on (p, v)), `q_gyro` on the yaw rate
and `q_bias` on the bias random walk (`[qg dt + qb dt³/3, −qb dt²/2; −qb dt²/2, qb dt]` on (yaw, b)).

**Asynchronous timing.** Every gyro sample predicts the filter to its own stamp. A GPS fix
first predicts forward to its stamp (with the latest gyro rate), then corrects. A fix that is
slightly older than the filter (it arrived after a newer gyro sample; up to `max_meas_lag_s`)
is applied at the filter time — the error is `|v|·lag`, a few millimetres; older ones are
dropped and counted. Nothing ever waits for the three sensors to line up.

**GPS update**, linear:

```text
z = [x, y, z]_gps     H = [I3 0]     R = covariance from the message (0.6², 0.6², 0.9² m²)
r = z - H x           S = H P H^T + R           K = P H^T S^-1   (solved with LDLT, no inverse)
x <- x + K r          P <- (I - K H) P (I - K H)^T + K R K^T   (Joseph form)
NIS = r^T S^-1 r      (mean 3 for a consistent filter; printed in the [filter] report)
```

An update whose result is not finite is not applied.

**Initialisation**, from the inputs only: position = first GPS fix with its own covariance,
yaw = current VO yaw (the VO frame starts aligned with the true pose), velocity and bias 0.
Nothing is hard-coded.

| Parameter | Value | Meaning |
|---|---|---|
| `q_acc_xy`, `q_acc_z` | 0.25, 0.1 m²/s³ | acceleration noise density (≈ 0.5 m/s² over ~1 s turns) |
| `q_gyro` | 1·10⁻⁶ rad²/s | gyro noise into yaw (measured 0.004² · 0.01 s = 1.6·10⁻⁷, inflated) |
| `q_bias` | 1·10⁻⁸ rad²/s³ | bias random walk (the bias is static) |
| `P0` position | GPS covariance | 0.36, 0.36, 0.81 m² |
| `init_vel_std` | 1.0 m/s | |
| `init_yaw_std` | 0.1 rad | |
| `init_bias_std` | 0.05 rad/s | bias magnitude ~0.02 rad/s, either sign |
| `R` GPS | message covariance | fallback `gps_std_fallback` 0.6, 0.6, 0.9 m |

Every ~5 s the node prints the state, its sigmas and the mean GPS NIS:

```text
[filter] t=98.4 p=(-6.85, 13.05, 2.08) v=(-0.65, 0.14, 0.20) yaw=88.8 deg b=+0.0000 rad/s | sigma xy 0.37 m, z 0.45 m, yaw 268.5 deg | GPS updates 50, mean NIS 2.81 (expect 3)
```

### Results so far: prediction + GPS only

Replay of `bags/a2_flight_seed42` (two laps of the §4 rectangle); a live run with the full
simulator gives the same numbers (horizontal 0.47 m, 96–98 % inside 2σ):

| | estimate | raw GPS |
|---|---:|---:|
| RMS error horizontal | 0.50 m | 0.87 m |
| RMS error z | 0.44 m | 0.93 m |
| inside the filter's own 2σ (x, y, z) | 97 %, 97 %, 96 % | |

![GPS-only errors](../docs/figures/a2/kf_gps_only_errors.png)

The position is better than raw GPS and consistent with its covariance. Two things are still
missing and are visible in the figure: velocity from GPS alone is noisy (≈ 0.3–0.5 m/s error),
and nothing measures yaw yet, so it drifts with the uncorrected bias (−1°/s) and its 2σ grows
without bound. The VO yaw and VO body-velocity updates come next.

## Tests

```bash
colcon test --packages-select lrs_localization && colcon test-result --verbose
```

- [`test_sensor_channel.cpp`](test/test_sensor_channel.cpp) — frame and stamp checks, rates, gaps;
- [`test_ekf.cpp`](test/test_ekf.cpp) — angle wrapping, constant-velocity and gyro prediction,
  forward-only time, 15 s without measurements stays finite, the GPS update equals the scalar
  Kalman gain, non-finite updates are rejected, and a simulated 60 s flight with noisy 5 Hz
  GPS is tracked within its own 3σ with mean NIS ≈ 3.
