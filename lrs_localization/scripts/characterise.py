#!/usr/bin/env python3
"""A2 §1 sensor characterisation from rosbags -- figures and numbers.

This is an offline plotting tool. It is the only A2 code that reads /evaluation/ground_truth,
and it sends nothing back to the estimator or the controller.

Record (all four topics, the recorder started before the sensor simulator):

  ros2 bag record -s mcap --storage-preset-profile zstd_fast -o bags/a2_static_seed42 \
    /localization/gps /localization/vo /localization/imu /evaluation/ground_truth

One bag -> rates/frames, GPS noise, VO drift, VO body velocity, gyro bias:

  python3 lrs_localization/scripts/characterise.py bags/a2_flight_seed42 --out docs/figures/a2

Several bags -> seed comparison (same seed must repeat, another seed must differ):

  python3 lrs_localization/scripts/characterise.py --compare \
    bags/a2_static_seed42 bags/a2_static_seed42_repeat bags/a2_static_seed73 --out docs/figures/a2
"""

import argparse
import os
from collections import defaultdict

import matplotlib
matplotlib.use('Agg')  # render to files, no display needed
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import rosbag2_py  # noqa: E402
from rclpy.serialization import deserialize_message  # noqa: E402
from rosidl_runtime_py.utilities import get_message  # noqa: E402

GPS, VO, IMU, TRUTH = '/localization/gps', '/localization/vo', '/localization/imu', '/evaluation/ground_truth'
# Public defaults from localization_sensor_sim/config/sensors.yaml, for comparison only.
GPS_SIGMA_CFG = np.array([0.6, 0.6, 0.9])
GYRO_SIGMA_CFG = np.sqrt(0.000016)
STILL_RATE = 0.02    # [rad/s] truth yaw rate below this = not turning
STILL_SPEED = 0.05   # [m/s]   truth speed below this = not moving


# ---------------------------------------------------------------- bag reading

def read_bag(path):
    """Return {topic: [msg, ...]} for the four A2 topics, in recording order."""
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=path, storage_id=''),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    data = defaultdict(list)
    while reader.has_next():
        topic, raw, _ = reader.read_next()  # bag receive time ignored: header stamps are used
        if topic in (GPS, VO, IMU, TRUTH):
            data[topic].append(deserialize_message(raw, get_message(types[topic])))
    return data


def stamp(m):
    return m.header.stamp.sec + m.header.stamp.nanosec * 1e-9


def wrap(a):
    return np.arctan2(np.sin(a), np.cos(a))


def quat_yaw(q):
    return np.arctan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Series:
    """Header-stamped arrays of one bag, sorted by stamp."""

    def __init__(self, data):
        self.frames = {}
        g, v, i, tr = data[GPS], data[VO], data[IMU], data[TRUTH]
        for topic, msgs in ((GPS, g), (VO, v), (IMU, i), (TRUTH, tr)):
            self.frames[topic] = sorted({(m.header.frame_id, getattr(m, 'child_frame_id', ''))
                                         for m in msgs})
        self.raw_t = {topic: np.array([stamp(m) for m in msgs])
                      for topic, msgs in ((GPS, g), (VO, v), (IMU, i), (TRUTH, tr))}

        self.gps_t = np.array([stamp(m) for m in g])
        self.gps = np.array([[m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z] for m in g])
        self.vo_t = np.array([stamp(m) for m in v])
        self.vo = np.array([[m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z] for m in v])
        self.vo_yaw = np.array([quat_yaw(m.pose.pose.orientation) for m in v])
        self.vo_vb = np.array([[m.twist.twist.linear.x, m.twist.twist.linear.y, m.twist.twist.linear.z] for m in v])
        self.imu_t = np.array([stamp(m) for m in i])
        self.wz = np.array([m.angular_velocity.z for m in i])
        self.tr_t = np.array([stamp(m) for m in tr])
        self.tr = np.array([[m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z] for m in tr])
        self.tr_yaw = np.array([quat_yaw(m.pose.pose.orientation) for m in tr])

        # Truth velocity and yaw rate by differentiating the truth pose -- independent of the
        # frame convention of the truth twist.
        yaw_u = np.unwrap(self.tr_yaw)
        self.tr_vel = np.gradient(self.tr, self.tr_t, axis=0)
        self.tr_rate = np.gradient(yaw_u, self.tr_t)
        self._yaw_u = yaw_u
        self.t0 = min(a[0] for a in (self.gps_t, self.vo_t, self.imu_t, self.tr_t) if len(a))

    def truth_at(self, t):
        """Truth position, yaw, world velocity and yaw rate interpolated at stamps t."""
        p = np.column_stack([np.interp(t, self.tr_t, self.tr[:, k]) for k in range(3)])
        yaw = wrap(np.interp(t, self.tr_t, self._yaw_u))
        vel = np.column_stack([np.interp(t, self.tr_t, self.tr_vel[:, k]) for k in range(3)])
        rate = np.interp(t, self.tr_t, self.tr_rate)
        return p, yaw, vel, rate


# ---------------------------------------------------------------- single bag

def rates_table(s):
    print('\nTopic                       n     rate[Hz]  median dt[s]  max dt[s]  non-monotonic  frames')
    for topic, t in s.raw_t.items():
        if len(t) < 2:
            print(f'{topic:26s} {len(t):5d}  NO DATA')
            continue
        dt = np.diff(t)
        rate = (len(t) - 1) / (t[-1] - t[0])
        print(f'{topic:26s} {len(t):5d}  {rate:8.2f}  {np.median(dt):12.4f}  {dt.max():9.3f}  '
              f'{int(np.sum(dt <= 0)):13d}  {s.frames[topic]}')


def analyse(s, out, name):
    rates_table(s)

    # GPS noise: GPS minus truth at the GPS stamp.
    p, _, _, _ = s.truth_at(s.gps_t)
    e_gps = s.gps - p
    print(f'\nGPS error mean [m]  {np.round(e_gps.mean(0), 3)}')
    print(f'GPS error std  [m]  {np.round(e_gps.std(0), 3)}   (config {GPS_SIGMA_CFG})')

    # VO drift: VO frame starts aligned with the truth, so VO minus truth is the drift.
    p_vo, yaw_vo, vel_vo, _ = s.truth_at(s.vo_t)
    e_vo = s.vo - p_vo
    e_vo_yaw = wrap(s.vo_yaw - yaw_vo)
    tv = s.vo_t - s.t0
    dur = tv[-1] - tv[0]
    print(f'\nVO drift after {dur:.0f} s: horizontal {np.hypot(*e_vo[-1, :2]):.3f} m, '
          f'z {e_vo[-1, 2]:+.3f} m, yaw {np.degrees(e_vo_yaw[-1]):+.2f} deg')

    # VO twist: compare with the truth velocity rotated into the body frame (and not rotated).
    c, sn = np.cos(yaw_vo), np.sin(yaw_vo)
    v_body = np.column_stack([c * vel_vo[:, 0] + sn * vel_vo[:, 1], -sn * vel_vo[:, 0] + c * vel_vo[:, 1]])
    moving = np.hypot(vel_vo[:, 0], vel_vo[:, 1]) > 0.2
    if moving.any():
        rms_body = np.sqrt(np.mean(np.sum((s.vo_vb[moving, :2] - v_body[moving]) ** 2, 1)))
        rms_enu = np.sqrt(np.mean(np.sum((s.vo_vb[moving, :2] - vel_vo[moving, :2]) ** 2, 1)))
        print(f'VO twist vs truth velocity while moving: RMS {rms_body:.3f} m/s in body frame, '
              f'{rms_enu:.3f} m/s if (wrongly) read as ENU')

    # Gyro: measured rate minus truth rate = bias + noise.
    _, _, vel_i, rate_i = s.truth_at(s.imu_t)
    still = (np.abs(rate_i) < STILL_RATE) & (np.hypot(vel_i[:, 0], vel_i[:, 1]) < STILL_SPEED)
    d = s.wz - rate_i
    bias_all = d.mean()
    print(f'\nGyro z: bias {bias_all:+.5f} rad/s (all samples), '
          f'{s.wz[still].mean():+.5f} rad/s (mean wz while not turning, {still.sum()} samples), '
          f'noise std {np.std(d - bias_all):.5f} rad/s (config {GYRO_SIGMA_CFG:.5f})')

    os.makedirs(out, exist_ok=True)

    # Figure 1: GPS
    fig, ax = plt.subplots(1, 2, figsize=(13, 5))
    ax[0].plot(s.tr[:, 0], s.tr[:, 1], 'g-', lw=2, label='truth (evaluation only)')
    ax[0].plot(s.gps[:, 0], s.gps[:, 1], 'b.', ms=3, label='GPS')
    ax[0].plot(s.vo[:, 0], s.vo[:, 1], color='tab:orange', lw=1, label='VO')
    ax[0].set(xlabel='x ENU [m]', ylabel='y ENU [m]', title='Top view, map frame')
    ax[0].axis('equal')
    ax[0].legend()
    tg = s.gps_t - s.t0
    for k, (lab, col) in enumerate(zip('xyz', ('tab:blue', 'tab:red', 'tab:green'))):
        ax[1].plot(tg, e_gps[:, k], '.', ms=3, color=col,
                   label=f'{lab}: std {e_gps[:, k].std():.2f} m (cfg {GPS_SIGMA_CFG[k]})')
    ax[1].set(xlabel='t [s]', ylabel='GPS - truth [m]', title='GPS noise')
    ax[1].legend()
    ax[1].grid(alpha=.3)
    fig.tight_layout()
    fig.savefig(os.path.join(out, f'{name}_gps.png'), dpi=110)

    # Figure 2: VO drift and VO body velocity
    fig, ax = plt.subplots(1, 2, figsize=(13, 5))
    ax[0].plot(tv, e_vo[:, 0], label='x')
    ax[0].plot(tv, e_vo[:, 1], label='y')
    ax[0].plot(tv, e_vo[:, 2], label='z')
    ax[0].plot(tv, np.hypot(e_vo[:, 0], e_vo[:, 1]), 'k', lw=2, label='horizontal')
    a2 = ax[0].twinx()
    a2.plot(tv, np.degrees(e_vo_yaw), 'm', lw=.8, alpha=.6, label='yaw (right axis)')
    a2.set_ylabel('VO yaw - truth [deg]', color='m')
    ax[0].set(xlabel='t [s]', ylabel='VO - truth [m]', title='VO drift')
    h1, l1 = ax[0].get_legend_handles_labels()
    h2, l2 = a2.get_legend_handles_labels()
    ax[0].legend(h1 + h2, l1 + l2, loc='lower left', fontsize=8, framealpha=.9)
    ax[0].grid(alpha=.3)
    ax[1].plot(tv, s.vo_vb[:, 0], 'b', lw=1, alpha=.7, label='VO twist x (body)')
    ax[1].plot(tv, v_body[:, 0], 'k--', lw=1.2, label='truth, rotated to body')
    ax[1].plot(tv, vel_vo[:, 0], 'c:', lw=1.5, label='truth ENU x (not body)')
    ax[1].set(xlabel='t [s]', ylabel='[m/s]', title='VO linear twist is body-frame velocity')
    ax[1].legend()
    ax[1].grid(alpha=.3)
    fig.tight_layout()
    fig.savefig(os.path.join(out, f'{name}_vo.png'), dpi=110)

    # Figure 3: gyro
    ti = s.imu_t - s.t0
    fig, ax = plt.subplots(figsize=(13, 4.5))
    ax.plot(ti, s.wz, '.', ms=1.5, color='tab:gray', label='IMU wz')
    ax.plot(ti, rate_i, 'g', lw=1.5, label='truth yaw rate')
    ax.axhline(bias_all, color='r', ls='--', label=f'bias {bias_all:+.4f} rad/s')
    ax.axhline(0, color='k', lw=.5)
    ax.set(xlabel='t [s]', ylabel='[rad/s]', title='Gyro z: rate minus truth = static bias + noise')
    ax.legend()
    ax.grid(alpha=.3)
    if not moving.any():
        ax.set_ylim(-0.06, 0.06)
    fig.tight_layout()
    fig.savefig(os.path.join(out, f'{name}_gyro.png'), dpi=110)
    print(f'\nfigures: {out}/{name}_{{gps,vo,gyro}}.png')


# ---------------------------------------------------------------- seed comparison

def best_shift(a, b, max_shift=10, n=150):
    """Smallest mean |a[k] - b[k+s]| over shifts s: the recorder may miss a first sample."""
    best = (np.inf, 0)
    for sh in range(-max_shift, max_shift + 1):
        i0, j0 = max(0, -sh), max(0, sh)
        m = min(n, len(a) - i0, len(b) - j0)
        if m < 20:
            continue
        best = min(best, (np.mean(np.abs(a[i0:i0 + m] - b[j0:j0 + m])), sh))
    return best


def compare(paths, out):
    names = [os.path.basename(os.path.normpath(p)) for p in paths]
    series = [Series(read_bag(p)) for p in paths]
    errs, biases = [], []
    for s in series:
        p, _, _, _ = s.truth_at(s.gps_t)
        errs.append(s.gps - p)
        _, _, _, rate = s.truth_at(s.imu_t)
        biases.append((s.wz - rate).mean())

    print('\nGyro bias per bag (static recordings, mean of wz - truth rate):')
    for n_, b in zip(names, biases):
        print(f'  {n_:28s} {b:+.5f} rad/s')
    print('\nGPS error sequences, best alignment against the first bag (mean |difference|):')
    for n_, e in zip(names[1:], errs[1:]):
        d, sh = best_shift(errs[0][:, 0], e[:, 0])
        print(f'  {names[0]} vs {n_:28s} {d:.4f} m (shift {sh})')
    common = np.intersect1d(series[0].imu_t, series[1].imu_t)
    if len(common):
        a = series[0].wz[np.searchsorted(series[0].imu_t, common)]
        b = series[1].wz[np.searchsorted(series[1].imu_t, common)]
        print(f'IMU wz at {len(common)} common stamps, {names[0]} vs {names[1]}: '
              f'max |difference| {np.max(np.abs(a - b)):.2e} rad/s')

    os.makedirs(out, exist_ok=True)
    fig, ax = plt.subplots(1, 2, figsize=(13, 4.8))
    for n_, e, ls in zip(names, errs, ('-', '--', '-')):
        d, sh = best_shift(errs[0][:, 0], e[:, 0])
        k = np.arange(len(e)) - sh
        ax[0].plot(k[:60], e[:60, 0], ls, marker='.', ms=4, label=n_)
    ax[0].set(xlabel='GPS sample index', ylabel='GPS x - truth [m]',
              title='GPS noise sequence: same seed repeats, other seed differs')
    ax[0].legend()
    ax[0].grid(alpha=.3)
    ax[1].bar(names, biases, color=['tab:blue', 'tab:cyan', 'tab:red'][:len(names)])
    for k, b in enumerate(biases):
        ax[1].text(k, b, f'{b:+.4f}', ha='center', va='bottom' if b > 0 else 'top')
    ax[1].axhline(0, color='k', lw=.5)
    ax[1].set(ylabel='gyro z bias [rad/s]', title='Hidden gyro bias per seed')
    fig.tight_layout()
    fig.savefig(os.path.join(out, 'seeds.png'), dpi=110)
    print(f'\nfigure: {out}/seeds.png')


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('bags', nargs='+')
    ap.add_argument('--compare', action='store_true', help='seed comparison of several static bags')
    ap.add_argument('--out', default='figures')
    a = ap.parse_args()
    if a.compare:
        compare(a.bags, a.out)
    else:
        for b in a.bags:
            name = os.path.basename(os.path.normpath(b))
            print(f'==================== {name}')
            analyse(Series(read_bag(b)), a.out, name)


if __name__ == '__main__':
    main()
