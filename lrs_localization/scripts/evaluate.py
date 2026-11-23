#!/usr/bin/env python3
"""Compare the estimator output with the truth -- offline plotting/evaluation only.

Reads a bag with /lrs_localization/odom, /evaluation/ground_truth and (optionally)
/localization/gps, recorded live or by replay.sh. Nothing is sent back to the estimator.

  python3 lrs_localization/scripts/evaluate.py /tmp/replay_flight --out docs/figures/a2 --name step3

Prints RMS errors of the estimate and of raw GPS, the share of samples inside the filter's own
2 sigma (about 95 % for a consistent filter), and saves <name>_track.png and <name>_errors.png.
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
    d = read_bag(a.bag, (EST, TRUTH, GPS))
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

    os.makedirs(a.out, exist_ok=True)
    fig, ax = plt.subplots(figsize=(7, 7))
    if d[GPS]:
        ax.plot(pg[:, 0], pg[:, 1], '.', color='tab:blue', ms=3, alpha=.5, label='raw GPS')
    ax.plot(pt_all[:, 0], pt_all[:, 1], 'g-', lw=2.5, label='truth (evaluation only)')
    ax.plot(pe[:, 0], pe[:, 1], color='tab:orange', lw=1.2, label='estimate')
    ax.set(xlabel='x ENU [m]', ylabel='y ENU [m]', title=f'{name}: top view')
    ax.axis('equal')
    ax.legend()
    ax.grid(alpha=.3)
    fig.tight_layout()
    fig.savefig(os.path.join(a.out, f'{name}_track.png'), dpi=110)

    fig, ax = plt.subplots(5, 1, figsize=(12, 12), sharex=True)
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
    fig.suptitle(f'{name}: estimate minus truth with the filter\'s own 2σ')
    fig.tight_layout()
    fig.savefig(os.path.join(a.out, f'{name}_errors.png'), dpi=110)
    print(f'figures: {a.out}/{name}_{{track,errors}}.png')


if __name__ == '__main__':
    main()
