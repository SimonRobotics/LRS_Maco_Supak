# lrs_localization

Assignment 2, topic **drone localisation**
([assignment](../assignments/assignment2/localization.md)): an eight-state Kalman filter
that fuses noisy GPS, drifting visual odometry and a biased yaw gyro.

| Section | Status |
|---|---|
| §1 Sensor acquisition and characterisation | done — [characterisation](../docs/a2_charakterizacia_senzorov.md) |
| §2 Eight-state KF/EKF | done — prediction, GPS, VO body velocity (EKF), zero-rate/zero-velocity updates |
| §3 Missing and bad measurements | done — innovation gate, timeouts, gyro-less prediction, recovery from permanent rejection |
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
[`characterise.py`](scripts/characterise.py) and [`live_plot.py`](scripts/live_plot.py) that reads
`/evaluation/ground_truth`, and it sends nothing back.

### Outage and outlier tests (§3)

```bash
# live, during a flight: the assignment's services, timed in simulation time (see §3)
python3 lrs_localization/scripts/teacher_tests.py

# offline, repeatable: the same faults cut into a recorded bag, then replayed
python3 lrs_localization/scripts/degrade_bag.py bags/a2_flight_seed42 /tmp/deg \
    --gps-outlier 20 --gps-off 40:55 --gps-outlier 45 --vo-off 70:73 --gps-outlier 85
lrs_localization/scripts/replay.sh /tmp/deg /tmp/deg_replay
python3 lrs_localization/scripts/evaluate.py /tmp/deg_replay --out /tmp/figures
```

[`degrade_bag.py`](scripts/degrade_bag.py) mimics the simulator: an outlier requested during a GPS
outage lands on the first fix after it. It can also cut the IMU (`--imu-off`) and shift GPS
permanently (`--gps-shift`), which the teacher services cannot. `evaluate.py` finds outages from
gaps in the recorded stamps and outliers from the truth, shades them in the plots and prints the
error during each outage and the estimate's largest step when the sensor returns.

## §1 Sensor acquisition

The node [`localization_node`](src/localization_node.cpp) subscribes to exactly three topics.
Nothing else reaches the estimator, in particular no ground truth and no MAVROS pose.

| Topic | Type | Expected frame | Used fields |
|---|---|---|---|
| `/localization/gps` | `geometry_msgs/PoseWithCovarianceStamped` | `map` | position, its covariance |
| `/localization/vo` | `nav_msgs/Odometry` | `vo_odom` → child `base_link` | linear twist (body velocity) only; pose not used |
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

**VO body-velocity update**, nonlinear → EKF. The VO linear twist is the velocity in
`base_link`, i.e. the ENU velocity rotated by −yaw:

```text
z = [vx, vy, vz]_vo (base_link)          h(x) = [ c vx + s vy,  -s vx + c vy,  vz ]     c = cos yaw, s = sin yaw

H = dh/dx = | 0 0 0   c   s  0   -s vx + c vy   0 |
            | 0 0 0  -s   c  0   -c vx - s vy   0 |
            | 0 0 0   0   0  1        0         0 |
```

We implement the genuine nonlinear measurement with its Jacobian, evaluated at the current
estimate, rather than rotating the VO velocity into ENU with the estimated yaw and fusing it as
a linear measurement. The difference is the yaw column: the linear approximation treats the
estimated yaw as exact and throws that column away. With it, a disagreement between the
velocity GPS implies (in ENU) and the velocity VO measures (in the body frame) corrects the
**yaw** — the heading becomes observable from motion. The Jacobian is checked against a
numerical derivative in the tests. When the drone does not move the yaw column is zero:
no heading information, as expected.

`R` is `vo_vel_std² I` with `vo_vel_std` = 0.1 m/s, not the 0.03 m/s in the message: that value
is only the white noise, but VO velocity also has a slowly wandering bias (VO drifts 0.8 m/min,
z 0.45 m/min at rest, characterisation). VO velocity is fused at 10 Hz (`vo_vel_period_s`).
The VO **pose** is not used at all — it drifts without bound and its covariance (1 cm) does not
say so.

**Zero-rate (ZARU) and zero-velocity (ZUPT) updates.** Over 1 s windows the node checks whether
the drone stands still: |mean VO velocity| < 0.1 m/s, |mean gyro − estimated bias| < 0.05 rad/s
and gyro std < 0.02 rad/s (no turning, no oscillation). Then the true yaw rate and velocity are
zero, so

```text
ZARU:  z = mean(w over the window)   H = e_b             R = gyro_var / N + 0.001²   -> measures the bias
ZUPT:  z = [0, 0, 0]                 H = [0 I3 0 0]      R = 0.02² I                 -> cancels VO velocity bias
```

Transitions are logged (`[zaru] standing still: ... ON` / `moving or turning: ... OFF`).

**Heading** — no absolute heading sensor is fused:
- initial yaw = the known spawn yaw (`init_yaw_source: spawn`, `initial_yaw_deg: 90` — the
  assignment's start pose, Gazebo yaw 0 → ENU π/2), σ 0.05 rad;
- standing still: gyro with the ZARU-estimated bias, so the heading does not walk away;
- moving: corrected by the VO-velocity/GPS coupling above.

VO yaw is not fused (`vo_yaw_mode: off`). Step 4 fused it as an absolute heading; a long live
run then showed VO yaw 113° off after an hour of simulation — the filter followed it. It can
still be switched on (`vo_yaw_mode: absolute`) for comparison; that path keeps the wrapped yaw
innovation: `r = wrap(yaw_vo − yaw)`, so +179° against −179° is a 2° correction.

**Angle wrapping.** Yaw is wrapped to `[-π, π)` after every prediction and every update; angle
innovations are wrapped before the gain is applied (tests: wrap function, a yaw update across
the ±π seam).

**How the gyro bias becomes observable.** No sensor measures `b` directly.
1. *Standing still* (ZARU): the true rate is zero, so the gyro reading itself is `b` + noise.
2. *Moving*: the prediction integrates `w − b`; a wrong `b` turns the predicted heading away from
   the heading implied by GPS + VO velocity. The prediction builds `P(yaw, b)` through
   `F(yaw, b) = −dt`, so every heading correction also corrects `b`.

Seed 42 bias −0.0171 ± 0.0004 (true −0.0170), seed 7 +0.0123 ± 0.0004 (true +0.0124), from
zero-rate updates in the first seconds on the ground.

**Initialisation**, from the inputs only: position = first GPS fix with its own covariance,
yaw = spawn yaw, velocity and bias 0. No position is hard-coded.

| Parameter | Value | Meaning |
|---|---|---|
| `q_acc_xy`, `q_acc_z` | 0.25, 0.1 m²/s³ | acceleration noise density (≈ 0.5 m/s² over ~1 s turns) |
| `q_yaw` | 1·10⁻⁵ rad²/s | yaw random walk: gyro noise (0.004² · 0.01 s = 1.6·10⁻⁷) with margin |
| `q_bias` | 1·10⁻⁸ rad²/s³ | bias random walk (the Gazebo bias is static) |
| `P0` position | GPS covariance | 0.36, 0.36, 0.81 m² |
| `init_vel_std` | 1.0 m/s | |
| `initial_yaw_deg`, `init_yaw_std` | 90°, 0.05 rad | spawn yaw |
| `init_bias_std` | 0.05 rad/s | bias magnitude ~0.02 rad/s, either sign |
| `R` GPS | message covariance | fallback `gps_std_fallback` 0.6, 0.6, 0.9 m |
| `R` VO velocity | (0.1 m/s)² I, 10 Hz | `vo_vel_std`, `vo_vel_period_s` |
| `R` ZARU | gyro_var/N + (0.001)² | `zaru_slack`; detection `zaru_max_speed/rate/rate_std` |
| `R` ZUPT | (0.02 m/s)² I | `zupt_std` |

Every ~5 s the node prints the state, its sigmas, still/moving and the mean NIS per update type:

```text
[filter] t=49.6 p=(-5.36, 8.99, 2.00) v=(0.46, 0.02, -0.03) yaw=0.7 deg b=-0.0171 rad/s | sigma xy 0.11 m, z 0.12 m, yaw 2.0 deg, b 0.0005 | moving | updates (mean NIS / expected): GPS 148 (3.08/3), VO vel 261 (0.34/3), ZARU 11 (0.10/1), ZUPT 11 (0.22/3), VO yaw 0 (0.00/1)
```

GPS NIS ≈ 3 as expected. VO velocity and ZARU NIS are well below their dimension on purpose:
their `R` covers slowly varying errors (VO velocity bias, residual motion) that the
sample-to-sample innovations do not show.

### Design choices, measured

Replays of `bags/a2_flight_after_8min_seed7` — seed 7, the drone **idles 8 minutes** on the
ground with all sensors running, then flies the two laps (as in a defence where the simulation
runs for a while before the mission):

| Variant | yaw RMS | why |
|---|---:|---|
| **final** (VO velocity, ZARU, spawn yaw) | **2.0°** | |
| + absolute VO yaw (step-4 style) | 13.2° | VO heading drifts, the filter follows |
| without ZARU | 100° | bias never learned at rest, yaw integrates it for 8 minutes |
| estimator started after 470 s, spawn yaw | 2.5° | |
| estimator started after 470 s, yaw from VO | 11.5° | VO had already drifted |

VO velocity noise and rate (both bags; position RMS and share inside the filter's own 2σ):

| VO velocity | ZUPT | flight seed 42: horiz / z / inside 2σ (x,y,z) | 8 min idle + flight: horiz / z / inside 2σ |
|---|---|---|---|
| 30 Hz, σ 0.03 (message) | – | 0.14 / 0.12 m / 88, 81, 72 % | – |
| 10 Hz, σ 0.03 | – | 0.13 / 0.11 m / 99, 89, 94 % | 0.17 / 0.35 m / 65, 93, **27 %** |
| 10 Hz, σ 0.03 | σ 0.05 | 0.13 / 0.11 m / 98, 89, 94 % | 0.17 / 0.34 m / 66, 94, 28 % |
| 10 Hz, σ 0.1 | – | 0.15 / 0.14 m / 99, 98, 90 % | 0.14 / 0.17 m / 96, 100, 84 % |
| **10 Hz, σ 0.1** | **σ 0.02** | 0.17 / 0.15 m / 96, 99, 89 % | **0.13 / 0.16 m / 97, 100, 87 %** |

With the message's σ 0.03 the filter trusts the VO velocity bias: after 8 minutes on the ground
z is 0.4 m off and only 27 % inside 2σ. σ 0.1 plus a zero-velocity update fixes that for a
slightly worse short flight.

### Results: prediction + GPS + VO velocity + ZARU/ZUPT

| | flight, seed 42 | 8 min idle + flight, seed 7 | static, seed 73 | step 4 (flight 42) | raw GPS |
|---|---:|---:|---:|---:|---:|
| RMS error horizontal | **0.17 m** | **0.13 m** | 0.10 m | 0.50 m | 0.87 m |
| RMS error z | 0.15 m | 0.16 m | 0.11 m | 0.44 m | 0.93 m |
| RMS error yaw | **0.8°** | **2.0°** | 0.3° | 3.4° | – |
| RMS error velocity | 0.07 m/s | 0.06 m/s | 0.05 m/s | 0.47 m/s | – |
| inside own 2σ (x, y, z, yaw) | 96, 99, 89, 100 % | 97, 100, 87, 98 % | 100 % | 97, 97, 96, 94 % | |
| gyro bias (true) | −0.0171 (−0.0170) | +0.0123 (+0.0124) | | −0.0160 | |

![Flight, seed 42](../docs/figures/a2/kf_vo_vel_errors.png)
![8 min idle then flight, seed 7](../docs/figures/a2/kf_long_idle_errors.png)

## §3 Missing and bad measurements

Nothing is ever reset because a sensor is missing or wrong. The prediction carries the state,
and the covariance `P` grows by exactly what the process model says it should.

### Innovation gate

Before any gain is computed, every GPS fix and every VO velocity is tested
([`ekf.hpp`](include/lrs_localization/ekf.hpp), `Ekf8::update`):

```text
r   = z - h(x)                       innovation
S   = H P H^T + R                    its covariance if the filter and the sensor are right
NIS = r^T S^-1 r                     (solved with LDLT, no inverse)
NIS > gate  ->  rejected, x and P unchanged
```

If the measurement is correct and the filter consistent, `r ~ N(0, S)` and NIS is chi-square
distributed with 3 degrees of freedom. `gps_gate_nis = 16.27` is its 99.9 % quantile: a correct
fix is rejected with probability 0.1 %, about one per 200 s at 5 Hz. Measured on the fault-free bags:
1 of ~550 fixes (seed 42) and 2–3 of ~2,900 (seed 7) were rejected, as predicted. A higher gate
lets in more of the outlier's tail, a lower one throws away good fixes.

Because `S` contains `H P H^T`, the gate is **not a fixed distance**: it widens while `P` grows.
The same 6 m innovation is rejected by a converged filter (NIS 97) and accepted after 15 s of pure
prediction (position σ 17 m, NIS 0.13) — test `GateIsStatisticalNotAFixedDistance`.

**The outlier after the outage.** The simulator applies an outlier requested during a GPS outage
to the *first* fix after GPS returns, exactly when `P` is largest. The gate still rejects it
because VO body velocity keeps fusing during the GPS outage: the velocity stays known, and the
position σ only grows from ~0.10 to ~0.2 m in 15 s. A 5 m outlier along z, the hardest direction,
is still `5 / sqrt(0.2² + 0.9²) ≈ 5.4 σ` out (test `OutlierRightAfterFifteenSecondGpsOutageIsRejected`).
With GPS **and** VO both lost for 15 s, σ would reach ~17 m and the outlier would pass the gate. The
assignment does not test that case, and nothing could distinguish such an outlier from a true fix then.

A rejected fix is logged with everything needed to explain it at the defence:

```text
[gps] REJECTED fix at t=40.09: innovation (-3.08, -5.83, +2.16) m, |r| 6.94 m; expected innovation sigma
(0.61, 0.61, 0.91) m -> NIS 122.9 > gate 16.27 (chi-square, 3 dof). A correct fix is this far out with
probability 1.8e-26, so it is treated as an outlier; filter unchanged [1 in a row]
```

The probability is the chi-square survival function (closed form for 3 dof:
`erfc(sqrt(x/2)) + sqrt(2x/π) e^(-x/2)`, `chi2Survival` in `ekf.hpp`).

VO velocity has the same gate (`vo_gate_nis`). In flight its NIS averages 0.3, because `R` is
deliberately wide (§2), so it never fired on the recorded flights. It is there for a VO glitch.
VO rejections are logged throttled (up to 10/s), and the count is in the `[filter]` report.

### Recovery from permanent rejection

The gate assumes the filter is right. If the filter is wrong (overconfident after a long outage,
or GPS has jumped for good), every fix fails and the filter would never recover.
[`rejection_guard.hpp`](include/lrs_localization/rejection_guard.hpp):

- `gps_max_rejections` = 10 rejected fixes **in a row** (2 s) → RECOVERY. The GPS agrees with
  itself but not with the filter, so the filter is presumed wrong.
- In recovery, a fix outside the gate is applied with `R` scaled by `α ≥ 1`, chosen so that
  `r^T (H P H^T + α R)^-1 r = gate` (NIS(α) falls monotonically in α, so α is found by bisection).
  The filter takes the step a borderline-acceptable fix would cause: towards GPS, bounded, never
  a jump onto one raw fix. Fixes inside the gate are applied normally.
- Recovery ends when 10 fixes in a row pass the gate on their own. Ending at the first passing
  fix would flip back and forth while the residual is still ~2 m.

A single outlier is one rejection, and the next good fix resets the count, so it can never
start recovery. Replay with GPS shifted permanently by (4, −3) m (`--gps-shift`): 10 rejections,
then 42 inflated fixes. The estimate moved onto the shifted GPS in ~15 s with no step larger
than 9.4 cm, and recovery ended on its own.

### Sensor timeouts

`SensorChannel::checkTimeout` measures silence in **header stamps**: "now" is the newest stamp of
any sensor, so a slow simulation does not cause false timeouts.

| Sensor | Timeout | Missed samples | While out |
|---|---:|---:|---|
| GPS | 1.0 s | 5 | position from prediction + VO velocity; σ grows |
| VO | 0.5 s | 15 | velocity from GPS + constant-velocity model |
| IMU | 0.2 s | 20 | predict with `w = b` (no turn), extra yaw noise; output published on GPS/VO messages |

Each transition is logged once (`TIMEOUT`, `RECOVERED`). The first GPS fix after an outage
reports the outage length, the σ before it, its innovation and the gate's decision:

```text
[gps] TIMEOUT: no message since t=44.89 (1.01 s > 1.00 s) -- position carried by prediction + VO velocity, sigma grows
[gps] RECOVERED after 15.2 s outage: sigma before the fix xy 0.19 m, z 0.18 m | innovation (+0.88, +3.70, +4.09) m,
|r| 5.58 m, NIS 56.11 (gate 16.27) -> REJECTED (see below)
[vo] RECOVERED after 3.0 s tracking loss: velocity innovation |r| 0.331 m/s, NIS 0.40 (gate 16.27) -> accepted
```

**IMU loss.** The gyro drives both the prediction and the publishing, so without it the output
would stop. During an IMU timeout the node predicts with `w = b` (true yaw rate 0) and adds
`imu_lost_yaw_rate_std²` = 1.0 rad²/s to the yaw noise density (`Ekf8::predictTo(t, w, extra_q_yaw)`).
It publishes after every GPS and VO message instead (30 Hz). 1.0 rad/s is the A1 mission's
largest yaw rate (1.07 rad/s measured). With 0.2 rad/s, a 2 s IMU loss in a 180° turn claimed a yaw σ of 15° while the true
error was 62°. With 1.0, the claimed σ was 57° and the true error 25°: the honest σ lets the
GPS/VO motion coupling (§2) correct the heading during the loss (yaw RMS of that run 11.3° → 4.7°).

### Results

Regression, no faults (replay; §2 numbers re-measured on the same commit before the gate):

| | horizontal | z | yaw | GPS fixes rejected |
|---|---:|---:|---:|---:|
| flight seed 42, before §3 → with §3 | 0.151 → 0.147 m | 0.144 → 0.142 m | 0.79 → 0.80° | 1 of ~550 |
| 8 min idle + flight seed 7 | 0.129 → 0.130 m | 0.155 → 0.154 m | 1.60 → 1.80° | 2–3 of ~2,900 |

The seed 7 yaw varies by ±0.2° between replays even with identical code, because the arrival order
of the messages differs; the gate does not change the accuracy. (These replays give
somewhat better numbers than the §2 table above, 0.15 m instead of 0.17 m on seed 42. The code is
the same, so the §2 table was measured on an earlier replay setup.)

The assignment's test: outlier in flight, GPS off for 15 s with an outlier requested during the
outage, VO off for 3 s. **Live** = flown by the A1 node (`missions/a2_rectangle.csv`,
`max_speed:=0.5`) with the services called by `teacher_tests.py`, recorded in
[`bags/a2_s3_outages_seed42`](../bags/a2_s3_outages_seed42) (sensors, truth, estimate, `/rosout`).
**Replay** = the same faults cut into the recorded flights with `degrade_bag.py`.

| | live, seed 42 | replay, seed 42 | replay, seed 7 (8 min idle) |
|---|---:|---:|---:|
| outlier in flight: \|r\|, NIS → | 6.9 m, 123 → rejected | 7.7 m, 156 → rejected | 6.9 m, 98 → rejected |
| outlier on the first fix after the 15 s outage | 5.6 m, 56 → rejected | 6.6 m, 102 → rejected | 7.9 m, 147 → rejected |
| σ xy just before that fix | 0.19 m | 0.19 m | 0.22 m |
| error at the end of the GPS outage (filter 2σ) | 0.13 m (0.55) | 0.11 m (0.54) | 0.14 m (0.62) |
| largest estimate step within 1 s of GPS return | 11 cm | 10 cm | 23 cm |
| largest error during the 3 s VO outage (2σ at return) | 0.53 m (1.08) | 0.61 m (1.15) | 0.74 m (1.15) |
| whole run: RMS horizontal / z / yaw | 0.16 m / 0.17 m / 1.0° | 0.17 m / 0.18 m / 0.6° | 0.14 m / 0.16 m / 1.9° |
| inside own 2σ (x, y, z, yaw) | 100, 93, 98, 100 % | 98, 98, 85, 100 % | 96, 100, 86, 96 % |

- **GPS outage**: VO velocity carries the position. The error stays below 0.2 m for 15 s, well
  inside the filter's 2σ, and the first good fix moves the estimate by about 10 cm.
- **VO outage**: the filter becomes GPS + constant velocity. Velocity is then known only from noisy GPS, so
  σ grows to ~0.55 m within 3 s and each fix moves the estimate by up to ~0.5 m (largest step
  at 100 Hz: 51 cm live). The truth stays inside 2σ, and after VO returns σ shrinks back within a
  few seconds. This is the Kalman gain doing what `q_acc_xy` = 0.25 (chosen in §2 for the turns)
  implies. §4 limits the commanded speed, so these steps cannot become a velocity spike.
- **Outliers**: no visible step. Within 0.5 s of an outlier the largest step is 2–10 cm, which is
  an ordinary fix. The 23 cm in the seed 7 column is the first *good* fix after the outage, 0.2 s later.

The A1 node must run with `use_sim_time:=true`: its setpoint carrot advances with `now()`, and in
wall time it moved at 0.5 m/s *wall*. In a first live run Gazebo ran at ~0.3× real time, so
the drone flew at 2 m/s. The estimator still tracked it (0.16 m), but that is not the specified
flight, so it was re-recorded.

![Live flight with outages, seed 42](../docs/figures/a2/kf_s3_live_errors.png)
![Live flight, top view](../docs/figures/a2/kf_s3_live_track.png)

Replays: [seed 42](../docs/figures/a2/kf_s3_replay_seed42_errors.png),
[seed 7](../docs/figures/a2/kf_s3_replay_seed7_errors.png).

| Parameter | Value | Why |
|---|---|---|
| `gps_gate_nis` | 16.27 | chi-square 3 dof, 99.9 %: 0.1 % of good fixes lost; the 5–8 m outliers measured NIS 56–170 |
| `vo_gate_nis` | 16.27 | same test for the 3-D VO velocity |
| `gps_max_rejections` | 10 (2 s) | 10 false rejections in a row: probability 10⁻³⁰; one outlier = 1 |
| `vo_max_rejections` | 10 (1 s) | |
| `gps_timeout_s`, `vo_timeout_s`, `imu_timeout_s` | 1.0, 0.5, 0.2 s | 5 / 15 / 20 missed samples, logs only (IMU: also switches the prediction) |
| `imu_lost_yaw_rate_std` | 1.0 rad/s | largest yaw rate of the mission |

## Tests

```bash
colcon test --packages-select lrs_localization && colcon test-result --verbose
```

- [`test_sensor_channel.cpp`](test/test_sensor_channel.cpp) — frame and stamp checks, rates, gaps;
- [`test_ekf.cpp`](test/test_ekf.cpp) — angle wrapping, constant-velocity and gyro prediction,
  forward-only time, 15 s without measurements stays finite, the GPS update equals the scalar
  Kalman gain, non-finite updates are rejected, a simulated 60 s flight with noisy 5 Hz GPS is
  tracked within its own 3σ with mean NIS ≈ 3, the yaw innovation is wrapped across ±π, a gyro
  bias is observable from yaw updates, the body-velocity Jacobian equals the numerical
  derivative, body velocity is rotated into ENU, **a 20° heading error is corrected from motion
  alone** (GPS + VO velocity, no yaw measurement), a zero-velocity update stops a creeping
  estimate, and zero-rate updates estimate the bias;
- §3 ([`test_ekf.cpp`](test/test_ekf.cpp), [`test_sensor_channel.cpp`](test/test_sensor_channel.cpp)):
  chi-square survival against tables, **a GPS outlier is rejected and x, P stay bit-identical**,
  **the gate is statistical** (the same 6 m innovation rejected with small P, accepted after 15 s of
  prediction), **the outlier on the first fix after a 15 s GPS outage with VO is still rejected**
  (all three axes) while a normal fix is accepted, the inflated update hits NIS = gate exactly and
  moves only part of the way, prediction without the gyro keeps the yaw and widens it, the
  rejection guard (one outlier never starts recovery; N in a row does; N passes end it), the
  15 s prediction stays finite, and timeouts/recoveries are measured in stamps and cleared by
  a simulation reset.
