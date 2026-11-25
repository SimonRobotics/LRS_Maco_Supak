#!/usr/bin/env python3
"""Compare the estimator output with the truth -- offline plotting/evaluation only.

Reads a bag with /lrs_localization/odom, /evaluation/ground_truth and (optionally)
/localization/gps, recorded live or by replay.sh. Nothing is sent back to the estimator.

  python3 lrs_localization/scripts/evaluate.py /tmp/replay_flight --out docs/figures/a2 --name step3

Prints RMS errors of the estimate and of raw GPS, the share of samples inside the filter's own
2 sigma (about 95 % for a consistent filter), and saves <name>_track.png and <name>_errors.png.

§3: sensor outages are found from gaps in the recorded header stamps (GPS > 1 s, VO > 0.5 s,
IMU > 0.2 s) and GPS outliers as fixes more than 4 m from the truth. Both are marked in the
plots, and for each one the script prints the error during the outage, the estimate's step
when the sensor returns (continuity) and whether an outlier moved the estimate.
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

EST, TRUTH, GPS = '/lrs_localization/odom', '/evaluation/ground_truth', '/localization/gps'
STATE, IMU = '/lrs_localization/state', '/localization/imu'  # optional: bias plot
VO = '/localization/vo'  # optional: VO outages
TIMEOUT = {GPS: 1.0, VO: 0.5, IMU: 0.2}  # the estimator's timeouts [s]
OUTLIER_M = 4.0  # a GPS fix this far from the truth is an injected outlier (sigma 0.6 / 0.9 m)


def read_bag(path, topics):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=path, storage_id=''),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    data = defaultdict(list)
    while reader.has_next():
        topic, raw, _ = reader.read_next()
        if topic in topics:
            data[topic].append(deserialize_message(raw, get_message(types[topic])))
    return data


def stamp(m):
    return m.header.stamp.sec + m.header.stamp.nanosec * 1e-9


def wrap(a):
    return np.arctan2(np.sin(a), np.cos(a))


def yaw_of(q):
    return np.arctan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('bag')
    ap.add_argument('--out', default='figures')
    ap.add_argument('--name', default=None)
    a = ap.parse_args()
    name = a.name or os.path.basename(os.path.normpath(a.bag))
    d = read_bag(a.bag, (EST, TRUTH, GPS, STATE, IMU, VO))
    if not d[EST] or not d[TRUTH]:
        raise SystemExit(f'{a.bag}: needs {EST} and {TRUTH}')

    # Estimate, sorted by stamp (the filter publishes in stamp order, the recorder may not).
    est = sorted(d[EST], key=stamp)
    te = np.array([stamp(m) for m in est])
    pe = np.array([[m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z] for m in est])
    ye = np.array([yaw_of(m.pose.pose.orientation) for m in est])
    cov = np.array([np.array(m.pose.covariance).reshape(6, 6) for m in est])
    sp = np.sqrt(np.stack([cov[:, 0, 0], cov[:, 1, 1], cov[:, 2, 2]], 1))
    sy = np.sqrt(cov[:, 5, 5])
    vb = np.array([[m.twist.twist.linear.x, m.twist.twist.linear.y, m.twist.twist.linear.z] for m in est])
    ve = np.column_stack([np.cos(ye) * vb[:, 0] - np.sin(ye) * vb[:, 1],
                          np.sin(ye) * vb[:, 0] + np.cos(ye) * vb[:, 1], vb[:, 2]])  # back to ENU

    tru = sorted(d[TRUTH], key=stamp)
    tt = np.array([stamp(m) for m in tru])
    pt_all = np.array([[m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z] for m in tru])
    yt_u = np.unwrap([yaw_of(m.pose.pose.orientation) for m in tru])
    vt_all = np.gradient(pt_all, tt, axis=0)

    keep = (te >= tt[0]) & (te <= tt[-1])
    te, pe, ye, sp, sy, ve = te[keep], pe[keep], ye[keep], sp[keep], sy[keep], ve[keep]
    pt = np.column_stack([np.interp(te, tt, pt_all[:, k]) for k in range(3)])
    vt = np.column_stack([np.interp(te, tt, vt_all[:, k]) for k in range(3)])
    yt = wrap(np.interp(te, tt, yt_u))
    ep = pe - pt
    ey = wrap(ye - yt)
    t0 = te[0]

    print(f'==================== {name}  ({te[-1] - t0:.0f} s, {len(te)} estimates)')
    rms_h = np.sqrt(np.mean(ep[:, 0] ** 2 + ep[:, 1] ** 2))
    rms_z = np.sqrt(np.mean(ep[:, 2] ** 2))
    print(f'estimate RMS error: horizontal {rms_h:.3f} m, z {rms_z:.3f} m, '
          f'yaw {np.degrees(np.sqrt(np.mean(ey ** 2))):.2f} deg, '
          f'velocity {np.sqrt(np.mean(np.sum((ve - vt) ** 2, 1))):.3f} m/s')
    if d[GPS]:
        tg = np.array([stamp(m) for m in d[GPS]])
        pg = np.array([[m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z] for m in d[GPS]])
        ok = (tg >= tt[0]) & (tg <= tt[-1])
        tg, pg = tg[ok], pg[ok]
        eg = pg - np.column_stack([np.interp(tg, tt, pt_all[:, k]) for k in range(3)])
        print(f'raw GPS  RMS error: horizontal {np.sqrt(np.mean(eg[:, 0] ** 2 + eg[:, 1] ** 2)):.3f} m, '
              f'z {np.sqrt(np.mean(eg[:, 2] ** 2)):.3f} m')
    inside = [np.mean(np.abs(ep[:, k]) <= 2 * sp[:, k]) * 100 for k in range(3)]
    print(f'inside own 2 sigma: x {inside[0]:.0f} %, y {inside[1]:.0f} %, z {inside[2]:.0f} %, '
          f'yaw {np.mean(np.abs(ey) <= 2 * sy) * 100:.0f} %   (consistent filter: ~95 %)')
    print(f'final 1 sigma: xy {np.mean(sp[-1, :2]):.2f} m, z {sp[-1, 2]:.2f} m, yaw {np.degrees(sy[-1]):.1f} deg')

    # Gyro bias: estimate from the state topic [t, x(8), sigma(8)]; reference = mean of
    # (IMU wz - truth yaw rate), the same definition as in characterise.py.
    tb = b_est = b_sig = None
    b_true = None
    if d[STATE]:
        st = np.array([m.data for m in d[STATE]])
        st = st[np.argsort(st[:, 0])]
        tb, b_est, b_sig = st[:, 0], st[:, 1 + 7], st[:, 1 + 8 + 7]
        if d[IMU]:
            ti = np.array([stamp(m) for m in d[IMU]])
            wz = np.array([m.angular_velocity.z for m in d[IMU]])
            ok = (ti >= tt[0]) & (ti <= tt[-1])
            b_true = np.mean(wz[ok] - np.interp(ti[ok], tt, np.gradient(yt_u, tt)))
        print(f'gyro bias: estimate {b_est[-1]:+.4f} ± {b_sig[-1]:.4f} rad/s (1σ)'
              + (f', true {b_true:+.4f} rad/s' if b_true is not None else ''))

    # §3: outages from gaps in the sensor stamps, outliers from the truth.
    outages = {}
    for topic in (GPS, VO, IMU):
        ts = np.sort([stamp(m) for m in d[topic]])
        gaps = np.flatnonzero(np.diff(ts) > TIMEOUT[topic]) if len(ts) > 1 else []
        outages[topic] = [(ts[i], ts[i + 1]) for i in gaps]
    outliers = []
    if d[GPS]:
        outliers = [tg[i] for i in np.flatnonzero(np.linalg.norm(eg, axis=1) > OUTLIER_M)]
    step = np.r_[0.0, np.linalg.norm(np.diff(pe, axis=0), axis=1)]  # estimate step per sample
    yaw_step = np.r_[0.0, np.abs(wrap(np.diff(ye)))]
    eh = np.hypot(ep[:, 0], ep[:, 1])
    sh = np.hypot(sp[:, 0], sp[:, 1])

    def near(t, before=0.0, after=1.0):
        return (te >= t - before) & (te <= t + after)

    if any(outages.values()) or outliers:
        quiet = te > te[0] + 5.0  # not the initial convergence from the first GPS fixes
        for lst in outages.values():
            for lo, hi in lst:
                quiet &= ~near(hi, 0.0, 1.0)
        print(f'largest estimate step outside outage returns: {step[quiet].max() * 100:.1f} cm '
              f'(99th percentile {np.percentile(step[quiet], 99) * 100:.1f} cm)')
    for topic, lst in outages.items():
        for lo, hi in lst:
            inside = (te > lo) & (te < hi)
            ret = near(hi, 0.0, 1.0)
            k_end = max(np.searchsorted(te, hi) - 1, 0)
            msg = (f'{topic.split("/")[-1].upper()} outage {lo - t0:6.1f}-{hi - t0:6.1f} s ({hi - lo:4.1f} s): '
                   f'max horiz error {eh[inside].max() if inside.any() else 0:.2f} m, '
                   f'at return {eh[k_end]:.2f} m vs filter 2 sigma {2 * sh[k_end]:.2f} m, '
                   f'max yaw error {np.degrees(np.abs(ey[inside]).max()) if inside.any() else 0:.1f} deg; '
                   f'largest step in the 1 s after return {step[ret].max() * 100:.1f} cm, '
                   f'yaw {np.degrees(yaw_step[ret].max()):.2f} deg')
            print(msg)
    for to in outliers:
        k = np.searchsorted(te, to)
        win = near(to, 0.0, 0.5)
        print(f'GPS outlier at {to - t0:6.1f} s: fix {np.linalg.norm(eg[np.searchsorted(tg, to)]):.1f} m off; '
              f'estimate error before {eh[max(k - 1, 0)]:.2f} m, max in the next 0.5 s {eh[win].max():.2f} m, '
              f'largest step {step[win].max() * 100:.1f} cm')

    def mark(axis):
        colors = {GPS: 'tab:red', VO: 'tab:purple', IMU: 'tab:brown'}
        for topic, lst in outages.items():
            for lo, hi in lst:
                axis.axvspan(lo - t0, hi - t0, color=colors[topic], alpha=.12, lw=0)
        for to in outliers:
            axis.axvline(to - t0, color='tab:red', ls=':', lw=1.2)

    os.makedirs(a.out, exist_ok=True)
    fig, ax = plt.subplots(figsize=(7, 7))
    if d[GPS]:
        ax.plot(pg[:, 0], pg[:, 1], '.', color='tab:blue', ms=3, alpha=.5, label='raw GPS')
    ax.plot(pt_all[:, 0], pt_all[:, 1], 'g-', lw=2.5, label='truth (evaluation only)')
    ax.plot(pe[:, 0], pe[:, 1], color='tab:orange', lw=1.2, label='estimate')
    if outliers:
        po = pg[[np.searchsorted(tg, to) for to in outliers]]
        ax.plot(po[:, 0], po[:, 1], 'rx', ms=10, mew=2, label='GPS outlier (rejected?)')
    for lo, hi in outages[GPS]:
        seg = (te > lo) & (te < hi)
        ax.plot(pe[seg, 0], pe[seg, 1], color='tab:red', lw=2.5,
                label='estimate during GPS outage' if lo == outages[GPS][0][0] else None)
    ax.set(xlabel='x ENU [m]', ylabel='y ENU [m]', title=f'{name}: top view')
    ax.axis('equal')
    ax.legend()
    ax.grid(alpha=.3)
    fig.tight_layout()
    fig.savefig(os.path.join(a.out, f'{name}_track.png'), dpi=110)

    rows = 6 if tb is not None else 5
    fig, ax = plt.subplots(rows, 1, figsize=(12, 2.4 * rows), sharex=True)
    tr = te - t0
    for k, lab in enumerate(('x', 'y', 'z')):
        ax[k].fill_between(tr, -2 * sp[:, k], 2 * sp[:, k], color='tab:orange', alpha=.25, label='±2σ (filter)')
        ax[k].plot(tr, ep[:, k], 'k', lw=.8, label='estimate - truth')
        ax[k].set_ylabel(f'{lab} error [m]')
        ax[k].grid(alpha=.3)
    ax[0].legend(loc='upper right')
    ax[3].fill_between(tr, -2 * np.degrees(sy), 2 * np.degrees(sy), color='tab:orange', alpha=.25)
    ax[3].plot(tr, np.degrees(ey), 'k', lw=.8)
    ax[3].set_ylabel('yaw error [deg]')
    ax[3].grid(alpha=.3)
    ax[4].plot(tr, np.hypot(vt[:, 0], vt[:, 1]), 'g', lw=1.5, label='truth')
    ax[4].plot(tr, np.hypot(ve[:, 0], ve[:, 1]), color='tab:orange', lw=1, label='estimate')
    ax[4].set(ylabel='horizontal speed [m/s]', xlabel='t [s]')
    ax[4].legend(loc='upper right')
    ax[4].grid(alpha=.3)
    if tb is not None:
        tbr = tb - t0
        ax[5].fill_between(tbr, b_est - 2 * b_sig, b_est + 2 * b_sig, color='tab:orange', alpha=.25, label='±2σ')
        ax[5].plot(tbr, b_est, color='tab:orange', lw=1.2, label='estimated gyro bias')
        if b_true is not None:
            ax[5].axhline(b_true, color='g', ls='--', label=f'true {b_true:+.4f} (IMU - truth rate)')
        ax[5].set(ylabel='bias [rad/s]', xlabel='t [s]')
        ax[5].set_ylim(min(-0.05, (b_true or 0) - 0.02), max(0.05, (b_true or 0) + 0.02))
        ax[5].legend(loc='upper right')
        ax[5].grid(alpha=.3)
    for axis in ax:
        mark(axis)
    if any(outages.values()) or outliers:
        from matplotlib.patches import Patch
        from matplotlib.lines import Line2D
        handles = [Patch(color=c, alpha=.25, label=f'{n} outage') for n, c, tp in
                   (('GPS', 'tab:red', GPS), ('VO', 'tab:purple', VO), ('IMU', 'tab:brown', IMU)) if outages[tp]]
        if outliers:
            handles.append(Line2D([], [], color='tab:red', ls=':', label='GPS outlier'))
        ax[3].legend(handles=handles, loc='upper right')
    fig.suptitle(f'{name}: estimate minus truth with the filter\'s own 2σ')
    fig.tight_layout()
    fig.savefig(os.path.join(a.out, f'{name}_errors.png'), dpi=110)
    print(f'figures: {a.out}/{name}_{{track,errors}}.png')


if __name__ == '__main__':
    main()
