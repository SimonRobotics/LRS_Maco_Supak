# lrs_localization

Assignment 2, topic **drone localisation**
([assignment](../assignments/assignment2/localization.md)): an eight-state Kalman filter
that fuses noisy GPS, drifting visual odometry and a biased yaw gyro.

| Section | Status |
|---|---|
| §1 Sensor acquisition and characterisation | done — [characterisation](../docs/a2_charakterizacia_senzorov.md) |
| §2 Eight-state KF/EKF | prediction, GPS update, VO yaw update (gyro bias estimated); VO velocity next |
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

# try a parameter without editing the config: everything after the rate goes to the node
lrs_localization/scripts/replay.sh bags/a2_flight_seed42 /tmp/replay_q 4 -p q_yaw:=1.0e-6
```

Besides `/lrs_localization/odom` the node publishes `/lrs_localization/state`
(`std_msgs/Float64MultiArray`, `[t, x(8), sigma(8)]`) for plotting only — it is the one place
the gyro bias can be seen.

### Live view

```bash
python3 lrs_localization/scripts/live_plot.py
```

A separate evaluation process (it reads the truth, publishes nothing): top view with truth,
estimate and its 2σ ellipse, the last GPS fixes and VO; on the right the x/y and yaw error of
the estimate against the filter's own 2σ and the gyro bias estimate. Times are header stamps.

[`evaluate.py`](scripts/evaluate.py) is an offline evaluation tool: it is the only place besides
[`characterise.py`](scripts/characterise.py) that reads `/evaluation/ground_truth`, and it sends
nothing back.

## §1 Sensor acquisition

The node [`localization_node`](src/localization_node.cpp) subscribes to exactly three topics.
Nothing else reaches the estimator, in particular no ground truth and no MAVROS pose.

| Topic | Type | Expected frame | Used fields |
|---|---|---|---|
| `/localization/gps` | `geometry_msgs/PoseWithCovarianceStamped` | `map` | position, its covariance |
| `/localization/vo` | `nav_msgs/Odometry` | `vo_odom` → child `base_link` | yaw (initialisation, heading update at 1 Hz) |
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
acceleration (per axis `[q dt³/3, q dt²/2; q dt²/2, q dt]` on (p, v)), `q_yaw` on the yaw rate
and `q_bias` on the bias random walk (`[qy dt + qb dt³/3, −qb dt²/2; −qb dt²/2, qb dt]` on (yaw, b)).

**Asynchronous timing.** Every gyro sample predicts the filter to its own stamp. A GPS or VO
measurement first predicts forward to its stamp (with the latest gyro rate), then corrects. One that is
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

**VO yaw update**, linear, the only heading measurement (GPS has no orientation):

```text
z = yaw_vo     H = e_yaw     R = vo_yaw_std²     r = wrap(z - yaw)   -> wrapped innovation
```

The innovation is wrapped to `[-pi, pi)`, so +179° measured against −179° estimated is a 2°
correction, not 358°; the corrected yaw is wrapped again.

**How the gyro bias becomes observable.** No sensor measures `b` directly. The prediction
integrates `w − b`; if the estimated `b` is wrong, the predicted yaw walks away from the VO yaw
at the rate `b_true − b_est`. The prediction itself builds the covariance `P(yaw, b)` (through
`F(yaw, b) = −dt`), so the Kalman gain of a yaw update has a non-zero `b` row and every yaw
innovation also corrects the bias. With seed 42 the bias goes from 0 to −0.016 rad/s in ~8 s
(true −0.017, figure below).

**Choice: VO yaw at 1 Hz with σ = 3°, `q_yaw` inflated.** VO yaw has small sample noise (0.3°)
but drifts ~3°/min (characterisation). That drift is a slowly changing error, strongly
correlated in time, and no other sensor observes it — the filter follows the VO heading, drift
included. Treating 30 samples per second as independent would make `P(yaw)` tiny while the real
error grows. So:
- only one VO yaw per second is fused (`vo_yaw_period_s`), with σ = 3° (`vo_yaw_std_deg`), the
  drift accumulated over a mission;
- `q_yaw` is not the gyro's white noise (0.004² · 0.01 s = 1.6·10⁻⁷) but 2·10⁻³: yaw process
  noise that also covers the wander of the heading reference.

Replay of the flight bag; the yaw error itself does not change — it is the VO drift — only
whether the filter admits it:

| `q_yaw` [rad²/s] | yaw RMS error | yaw inside own 2σ | estimated bias (true −0.0170) |
|---|---:|---:|---:|
| 1·10⁻⁶ (≈ gyro noise) | 3.38° | 31 % | −0.0162 |
| 1·10⁻⁴ | 3.38° | 50 % | −0.0161 |
| 1·10⁻³ | 3.41° | 82 % | −0.0160 |
| **2·10⁻³** | 3.39° | **94 %** | −0.0160 |
| 5·10⁻³ | 3.40° | 99 % | −0.0158 |

Fusing VO yaw at 30 Hz with σ 0.5° or at 1 Hz with σ 1° gives the same 3.4° but only 2 % and
12 % inside 2σ. Consequences of following the VO heading: the bias estimate absorbs the VO yaw
drift rate (−0.016 instead of −0.017), and NIS of the VO yaw updates is ≈ 0.01 instead of 1 —
the innovations against VO are tiny because filter and VO drift together; `R` is chosen for the
error against the truth, which the innovations cannot show.

**Initialisation**, from the inputs only: position = first GPS fix with its own covariance,
yaw = current VO yaw (the VO frame starts aligned with the true pose), velocity and bias 0.
Nothing is hard-coded.

| Parameter | Value | Meaning |
|---|---|---|
| `q_acc_xy`, `q_acc_z` | 0.25, 0.1 m²/s³ | acceleration noise density (≈ 0.5 m/s² over ~1 s turns) |
| `q_yaw` | 2·10⁻³ rad²/s | yaw random walk: gyro noise + unobserved VO heading drift (tuning above) |
| `q_bias` | 1·10⁻⁸ rad²/s³ | bias random walk (the bias is static) |
| `P0` position | GPS covariance | 0.36, 0.36, 0.81 m² |
| `init_vel_std` | 1.0 m/s | |
| `init_yaw_std` | 0.1 rad | |
| `init_bias_std` | 0.05 rad/s | bias magnitude ~0.02 rad/s, either sign |
| `R` GPS | message covariance | fallback `gps_std_fallback` 0.6, 0.6, 0.9 m |
| `R` VO yaw | (3°)², 1 Hz | `vo_yaw_std_deg`, `vo_yaw_period_s` |

Every ~5 s the node prints the state, its sigmas and the mean NIS per measurement type:

```text
[filter] t=101.0 p=(-7.56, 13.07, 1.86) v=(-0.51, 0.16, 0.09) yaw=174.8 deg b=-0.0161 rad/s | sigma xy 0.39 m, z 0.47 m, yaw 3.1 deg, b 0.0044 | updates (mean NIS, expect dim): GPS 74 (2.85/3), VO yaw 15 (0.01/1)
```

### Results so far: prediction + GPS + VO yaw

Replay of `bags/a2_flight_seed42` (two laps of the §4 rectangle) and `bags/a2_static_seed73`
(another seed, another bias):

| | flight, seed 42 | static, seed 73 | GPS only (previous step) |
|---|---:|---:|---:|
| RMS error horizontal | 0.50 m | 0.48 m | 0.50 m (raw GPS 0.87 m) |
| RMS error z | 0.44 m | 0.42 m | 0.44 m (raw GPS 0.93 m) |
| RMS error yaw | **3.4°** | 0.4° | 61° |
| inside own 2σ (x, y, z, yaw) | 97, 97, 96, 94 % | 99, 98, 98, 100 % | yaw: σ unbounded |
| gyro bias estimate (true) | −0.0160 ± 0.0044 (−0.0170) | −0.0238 ± 0.0059 (−0.0239) | not observable |

![VO yaw errors](../docs/figures/a2/kf_vo_yaw_errors.png)

Yaw is now bounded and the bias is estimated for either seed. Still missing: the velocity comes
from GPS alone (≈ 0.47 m/s RMS error, the noisy speed in the figure), which also limits the
position.

**Known limitation found in a long live run.** After ~60 min of simulation time (three flights)
VO yaw was 113° off the truth, VO position 36 m away — and the estimate followed the VO heading
(yaw error −113°, position still fine thanks to GPS). The 2-minute bags only show a few degrees
of this drift. Fusing the absolute VO yaw therefore ties the filter's heading to a reference
that drifts without bound; in a defence the simulation may run for minutes before the mission.
Next step: the absolute heading will come from motion — the VO body-velocity update couples
velocity and yaw, and GPS sees the same motion in ENU — while VO yaw is used only relatively
(its change) to keep the bias observable, and the initial heading comes from the known spawn
yaw instead of a possibly drifted VO.

## Tests

```bash
colcon test --packages-select lrs_localization && colcon test-result --verbose
```

- [`test_sensor_channel.cpp`](test/test_sensor_channel.cpp) — frame and stamp checks, rates, gaps;
- [`test_ekf.cpp`](test/test_ekf.cpp) — angle wrapping, constant-velocity and gyro prediction,
  forward-only time, 15 s without measurements stays finite, the GPS update equals the scalar
  Kalman gain, non-finite updates are rejected, a simulated 60 s flight with noisy 5 Hz
  GPS is tracked within its own 3σ with mean NIS ≈ 3, the yaw innovation is wrapped across
  ±π, and an unknown gyro bias becomes observable from yaw updates alone.
