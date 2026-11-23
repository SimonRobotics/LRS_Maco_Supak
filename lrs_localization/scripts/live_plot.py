#!/usr/bin/env python3
"""Live view of the estimator for development and the defence -- evaluation only.

A separate process: it subscribes to the estimate, the raw sensors AND the ground truth, and
only draws. It publishes nothing, so no truth can reach the estimator or the controller.

  python3 lrs_localization/scripts/live_plot.py

Left:  top view -- truth (green), estimate (orange) with its current 2 sigma ellipse, the last
       raw GPS fixes (blue), VO (purple, frame vo_odom drawn as map: shows its drift).
Right: x/y error of the estimate with the filter's own 2 sigma (and raw GPS x error for
       comparison), yaw error with 2 sigma, gyro bias estimate with 2 sigma.
All times are header stamps (simulation time).
"""

import math
import threading
from collections import deque

import matplotlib
matplotlib.use('QtAgg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib.animation import FuncAnimation  # noqa: E402
from matplotlib.patches import Ellipse  # noqa: E402

import rclpy  # noqa: E402
from geometry_msgs.msg import PoseWithCovarianceStamped  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import qos_profile_sensor_data  # noqa: E402
from std_msgs.msg import Float64MultiArray  # noqa: E402

N = 30000  # samples kept per series (at the decimated rates below: > 10 minutes)


def stamp(m):
    return m.header.stamp.sec + m.header.stamp.nanosec * 1e-9


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Collector(Node):
    """Stores decimated samples; the plot thread reads copies under the lock."""

    def __init__(self):
        super().__init__('lrs_live_plot')
        self.lock = threading.Lock()
        self.truth = deque(maxlen=N)  # t, x, y, yaw
        self.est = deque(maxlen=N)    # t, x, y, yaw, var_x, cov_xy, var_y, var_yaw
        self.gps = deque(maxlen=N)    # t, x, y
        self.vo = deque(maxlen=N)     # t, x, y
        self.bias = deque(maxlen=N)   # t, b, sigma_b
        self.count = {'truth': 0, 'est': 0, 'vo': 0, 'state': 0}
        q = qos_profile_sensor_data  # best effort: matches every publisher here
        self.create_subscription(Odometry, '/evaluation/ground_truth', self.on_truth, q)
        self.create_subscription(Odometry, '/lrs_localization/odom', self.on_est, q)
        self.create_subscription(Float64MultiArray, '/lrs_localization/state', self.on_state, q)
        self.create_subscription(PoseWithCovarianceStamped, '/localization/gps', self.on_gps, q)
        self.create_subscription(Odometry, '/localization/vo', self.on_vo, q)

    def _every(self, key, n):
        self.count[key] += 1
        return self.count[key] % n == 0

    def _reset_if_time_went_back(self, series, t):
        if series and t < series[-1][0] - 1.0:  # simulation restarted
            for s in (self.truth, self.est, self.gps, self.vo, self.bias):
                s.clear()

    def on_truth(self, m):
        if not self._every('truth', 5):  # 100 -> 20 Hz
            return
        t = stamp(m)
        p = m.pose.pose.position
        with self.lock:
            self._reset_if_time_went_back(self.truth, t)
            self.truth.append((t, p.x, p.y, yaw_of(m.pose.pose.orientation)))

    def on_est(self, m):
        if not self._every('est', 5):  # 100 -> 20 Hz
            return
        t = stamp(m)
        p = m.pose.pose.position
        c = m.pose.covariance
        with self.lock:
            self.est.append((t, p.x, p.y, yaw_of(m.pose.pose.orientation), c[0], c[1], c[7], c[35]))

    def on_state(self, m):
        if not self._every('state', 10):  # 100 -> 10 Hz
            return
        d = m.data
        with self.lock:
            self.bias.append((d[0], d[1 + 7], d[1 + 8 + 7]))

    def on_gps(self, m):
        p = m.pose.pose.position
        with self.lock:
            self.gps.append((stamp(m), p.x, p.y))

    def on_vo(self, m):
        if not self._every('vo', 3):  # 30 -> 10 Hz
            return
        p = m.pose.pose.position
        with self.lock:
            self.vo.append((stamp(m), p.x, p.y))

    def snapshot(self):
        with self.lock:
            return tuple(np.array(s) for s in (self.truth, self.est, self.gps, self.vo, self.bias))


def main():
    rclpy.init()
    node = Collector()
    ex = SingleThreadedExecutor()
    ex.add_node(node)
    threading.Thread(target=ex.spin, daemon=True).start()

    fig = plt.figure(figsize=(15, 8.5))
    fig.canvas.manager.set_window_title('lrs_localization - estimate vs truth (evaluation only)')
    gs = fig.add_gridspec(3, 2, width_ratios=[1.15, 1])
    ax_top = fig.add_subplot(gs[:, 0])
    ax_xy = fig.add_subplot(gs[0, 1])
    ax_yaw = fig.add_subplot(gs[1, 1], sharex=ax_xy)
    ax_b = fig.add_subplot(gs[2, 1], sharex=ax_xy)

    l_truth, = ax_top.plot([], [], 'g-', lw=2.5, label='truth (evaluation only)')
    l_vo, = ax_top.plot([], [], '-', color='tab:purple', lw=1, alpha=.8, label='VO (drifts)')
    l_est, = ax_top.plot([], [], '-', color='tab:orange', lw=1.3, label='estimate')
    l_gps, = ax_top.plot([], [], '.', color='tab:blue', ms=5, alpha=.6, label='GPS, last 50 fixes')
    l_now, = ax_top.plot([], [], 'o', color='tab:orange', mec='k', ms=8)
    ell = Ellipse((0, 0), 0, 0, fc='tab:orange', ec='tab:orange', alpha=.25, label='estimate 2σ')
    ax_top.add_patch(ell)
    ax_top.set(xlabel='x ENU [m]', ylabel='y ENU [m]', title='Top view, frame map')
    ax_top.set_aspect('equal', adjustable='datalim')
    ax_top.grid(alpha=.3)
    ax_top.legend(loc='upper right', fontsize=8)

    for ax, lab in ((ax_xy, 'position error [m]'), (ax_yaw, 'yaw error [deg]'), (ax_b, 'gyro bias [rad/s]')):
        ax.set_ylabel(lab)
        ax.grid(alpha=.3)
    ax_b.set_xlabel('t [s] (header stamps)')
    status = fig.text(0.01, 0.005, '', fontsize=9, family='monospace')

    def update(_):
        tr, est, gps, vo, bias = node.snapshot()
        if len(tr) < 2 or len(est) < 2:
            status.set_text('waiting for /evaluation/ground_truth and /lrs_localization/odom ...')
            return
        t0 = min(tr[0, 0], est[0, 0])
        l_truth.set_data(tr[:, 1], tr[:, 2])
        l_est.set_data(est[:, 1], est[:, 2])
        if len(vo):
            l_vo.set_data(vo[:, 1], vo[:, 2])
        if len(gps):
            l_gps.set_data(gps[-50:, 1], gps[-50:, 2])
        x, y = est[-1, 1], est[-1, 2]
        l_now.set_data([x], [y])
        C = np.array([[est[-1, 4], est[-1, 5]], [est[-1, 5], est[-1, 6]]])
        w, V = np.linalg.eigh(C)
        ell.set_center((x, y))
        ell.set_width(2 * 2 * math.sqrt(max(w[1], 0)))   # full width = 2 * (2 sigma)
        ell.set_height(2 * 2 * math.sqrt(max(w[0], 0)))
        ell.set_angle(math.degrees(math.atan2(V[1, 1], V[0, 1])))
        pts = np.vstack([tr[:, 1:3], est[:, 1:3]])
        lo, hi = pts.min(0) - 1.0, pts.max(0) + 1.0
        ax_top.set_xlim(lo[0], hi[0])
        ax_top.set_ylim(lo[1], hi[1])

        # errors at the estimate stamps (truth interpolated)
        te = est[:, 0]
        ok = (te >= tr[0, 0]) & (te <= tr[-1, 0])
        te, e = te[ok], est[ok]
        tr_yaw = np.unwrap(tr[:, 3])
        ex = e[:, 1] - np.interp(te, tr[:, 0], tr[:, 1])
        ey = e[:, 2] - np.interp(te, tr[:, 0], tr[:, 2])
        eyaw = np.degrees(np.arctan2(np.sin(e[:, 3] - np.interp(te, tr[:, 0], tr_yaw)),
                                     np.cos(e[:, 3] - np.interp(te, tr[:, 0], tr_yaw))))
        s_xy = np.sqrt(0.5 * (e[:, 4] + e[:, 6]))
        s_yaw = np.degrees(np.sqrt(e[:, 7]))
        tt = te - t0

        ax_xy.clear()
        ax_xy.fill_between(tt, -2 * s_xy, 2 * s_xy, color='tab:orange', alpha=.2, label='±2σ (filter)')
        if len(gps):
            tg = gps[:, 0]
            okg = (tg >= tr[0, 0]) & (tg <= tr[-1, 0])
            ax_xy.plot(tg[okg] - t0, gps[okg, 1] - np.interp(tg[okg], tr[:, 0], tr[:, 1]),
                       '.', color='tab:blue', ms=2, alpha=.4, label='raw GPS x error')
        ax_xy.plot(tt, ex, color='tab:orange', lw=1, label='estimate x error')
        ax_xy.plot(tt, ey, color='saddlebrown', lw=1, label='estimate y error')
        ax_xy.set_ylim(-2.5, 2.5)
        ax_xy.set_ylabel('position error [m]')
        ax_xy.legend(loc='upper left', fontsize=7, ncol=2)
        ax_xy.grid(alpha=.3)

        ax_yaw.clear()
        ax_yaw.fill_between(tt, -2 * s_yaw, 2 * s_yaw, color='tab:orange', alpha=.2)
        ax_yaw.plot(tt, eyaw, 'k', lw=1)
        ax_yaw.set_ylim(-15, 15)
        ax_yaw.set_ylabel('yaw error [deg]')
        ax_yaw.grid(alpha=.3)

        ax_b.clear()
        if len(bias):
            tb = bias[:, 0] - t0
            ax_b.fill_between(tb, bias[:, 1] - 2 * bias[:, 2], bias[:, 1] + 2 * bias[:, 2], color='tab:orange', alpha=.2)
            ax_b.plot(tb, bias[:, 1], color='tab:orange', lw=1.3)
            ax_b.set_ylim(-0.06, 0.06)
        ax_b.set_ylabel('gyro bias [rad/s]')
        ax_b.set_xlabel('t [s] (header stamps)')
        ax_b.grid(alpha=.3)

        h = math.hypot(ex[-1], ey[-1]) if len(ex) else float('nan')
        b_txt = f'  b {bias[-1, 1]:+.4f} ± {bias[-1, 2]:.4f} rad/s' if len(bias) else ''
        status.set_text(f't {te[-1] - t0:6.1f} s   horizontal error {h:.2f} m (2σ {2 * s_xy[-1]:.2f})'
                        f'   yaw error {eyaw[-1]:+.1f}° (2σ {2 * s_yaw[-1]:.1f}°){b_txt}'
                        f'   GPS fixes {len(gps)}')

    _anim = FuncAnimation(fig, update, interval=300, cache_frame_data=False)  # noqa: F841
    fig.tight_layout(rect=(0, 0.02, 1, 1))
    plt.show()
    ex.shutdown()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
