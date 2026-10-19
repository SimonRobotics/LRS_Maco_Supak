#!/usr/bin/env python3
"""Figures and numbers for the A1.3/A1.4 documentation, from a recorded rosbag.

Record a mission with (flown_path/path/waypoints are latched, so rosbag2 picks them
up even when recording starts after the node):

  ros2 bag record -o indoor_run1 \
    /mavros/local_position/pose /mavros/local_position/velocity_local \
    /mavros/setpoint_raw/local /lrs_mission/path /lrs_mission/flown_path \
    /lrs_mission/waypoints /rosout

Then:

  ros2 run lrs_mission plot_bag.py indoor_run1 --mission indoor --out figures/
"""

import argparse
import math
import os
from collections import defaultdict

import matplotlib
matplotlib.use('Agg')  # render to files, no display needed
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import rosbag2_py  # noqa: E402
from rclpy.serialization import deserialize_message  # noqa: E402
from rosidl_runtime_py.utilities import get_message  # noqa: E402

TRAVEL_SPEED_MIN = 0.3  # [m/s] below this the direction of travel is meaningless


# ---------------------------------------------------------------- bag reading

def read_bag(path):
    """Return {topic: [(t_sec, msg), ...]} for every topic in the bag."""
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=path, storage_id=''),  # '' = autodetect
        rosbag2_py.ConverterOptions('cdr', 'cdr'))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    data = defaultdict(list)
    while reader.has_next():
        topic, raw, t_ns = reader.read_next()
        data[topic].append(
            (t_ns * 1e-9, deserialize_message(raw, get_message(types[topic]))))
    return data


# ---------------------------------------------------------------- conversions

def wrap(a):
    """Wrap an angle (or array) into (-pi, pi]."""
    return np.arctan2(np.sin(a), np.cos(a))


def quat_yaw(q):
    """Yaw from an x,y,z,w quaternion -- same formula as lrs_mission_node.cpp."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def local_to_map(xyz, yaw_deg):
    """Inverse of the node's toLocal(): rotate MAVROS local ENU into the map frame."""
    a = -math.radians(yaw_deg)
    c, s = math.cos(a), math.sin(a)
    out = np.array(xyz, dtype=float, copy=True)
    out[:, 0] = c * xyz[:, 0] - s * xyz[:, 1]
    out[:, 1] = s * xyz[:, 0] + c * xyz[:, 1]
    return out


def pose_arrays(msgs):
    """(t, xyz Nx3, yaw N) from PoseStamped messages."""
    t = np.array([s for s, _ in msgs])
    xyz = np.array([[m.pose.position.x, m.pose.position.y, m.pose.position.z]
                    for _, m in msgs])
    yaw = np.array([quat_yaw(m.pose.orientation) for _, m in msgs])
    return t, xyz, yaw


def setpoint_arrays(msgs):
    """(t, xyz Nx3, yaw N) from mavros_msgs/PositionTarget messages."""
    t = np.array([s for s, _ in msgs])
    xyz = np.array([[m.position.x, m.position.y, m.position.z] for _, m in msgs])
    yaw = np.array([m.yaw for _, m in msgs])
    return t, xyz, yaw


def speed_arrays(msgs):
    """(t, |v| N) from TwistStamped messages."""
    t = np.array([s for s, _ in msgs])
    v = np.array([[m.twist.linear.x, m.twist.linear.y, m.twist.linear.z]
                  for _, m in msgs])
    return t, np.linalg.norm(v, axis=1), v


def path_xyz(msgs):
    """Nx3 of the last nav_msgs/Path on the topic, or None."""
    if not msgs:
        return None
    p = msgs[-1][1]
    if not p.poses:
        return None
    return np.array([[q.pose.position.x, q.pose.position.y, q.pose.position.z]
                     for q in p.poses])


def waypoints_from_markers(msgs):
    """[(xyz, label, hard)] from the last MarkerArray; skips the DELETEALL marker."""
    if not msgs:
        return []
    arr = msgs[-1][1]
    spheres, labels = {}, {}
    for m in arr.markers:
        if m.ns == 'waypoints':
            # hard waypoints are published red (r 0.9), soft green (r 0.1)
            spheres[m.id] = ([m.pose.position.x, m.pose.position.y,
                              m.pose.position.z], m.color.r > 0.5)
        elif m.ns == 'labels':
            labels[m.id] = m.text
    return [(np.array(p), labels.get(i, ''), hard)
            for i, (p, hard) in sorted(spheres.items())]


def state_timeline(msgs):
    """[(t, state_text)] from the node's own '[state] ...' lines on /rosout."""
    out = []
    for t, m in msgs:
        if getattr(m, 'name', '') == 'lrs_mission' and m.msg.startswith('[state] '):
            out.append((t, m.msg[len('[state] '):]))
    return out


def state_window(timeline, needle):
    """(t_start, t_end) of the first state whose text contains `needle`."""
    for i, (t, s) in enumerate(timeline):
        if needle in s:
            end = timeline[i + 1][0] if i + 1 < len(timeline) else math.inf
            return t, end
    return None


# ---------------------------------------------------------------- geometry

def match_nearest(t_a, t_b, max_dt):
    """For every t_a the index of the nearest t_b, plus a validity mask.

    The mask is what keeps this honest: the node publishes no setpoints during
    TAKEOFF and LAND (have_sp_ = false), so the bag has gaps -- during a
    landtakeoff even in the middle of the mission. Without the mask, searchsorted
    silently pairs a pose with a setpoint from before the gap and the resulting
    "tracking error" is nonsense.
    """
    if len(t_b) == 0:
        return np.zeros(len(t_a), dtype=int), np.zeros(len(t_a), dtype=bool)
    idx = np.searchsorted(t_b, t_a).clip(0, len(t_b) - 1)
    prev = (idx - 1).clip(0, len(t_b) - 1)
    take_prev = np.abs(t_b[prev] - t_a) < np.abs(t_b[idx] - t_a)
    idx = np.where(take_prev, prev, idx)
    return idx, np.abs(t_b[idx] - t_a) <= max_dt


def dist_to_polyline(pts, line):
    """Horizontal distance from every point in `pts` to the polyline `line`."""
    p = np.asarray(pts)[:, :2]
    a = np.asarray(line)[:-1, :2]
    b = np.asarray(line)[1:, :2]
    ab = b - a
    denom = np.einsum('ij,ij->i', ab, ab)
    denom[denom == 0] = 1e-12
    ap = p[:, None, :] - a[None, :, :]
    s = np.clip(np.einsum('ijk,jk->ij', ap, ab) / denom, 0.0, 1.0)
    closest = a[None, :, :] + s[:, :, None] * ab[None, :, :]
    return np.linalg.norm(p[:, None, :] - closest, axis=2).min(axis=1)


def figure8_segment(path):
    """The 201 densely-sampled figure-8 points inside the outdoor commanded path.

    buildOutdoorActions() appends the curve as 201 consecutive points, so it is
    the longest run of closely-spaced vertices. Detecting it that way needs no
    knowledge of the amplitude and compares the flight against what the node
    actually commanded, not against a re-derivation of the equations.
    """
    if path is None or len(path) < 3:
        return None
    step = np.linalg.norm(np.diff(path[:, :2], axis=0), axis=1)
    dense = step < 0.5
    best_len, best_start, run_start = 0, None, None
    for i, d in enumerate(dense):
        if d and run_start is None:
            run_start = i
        elif not d and run_start is not None:
            if i - run_start > best_len:
                best_len, best_start = i - run_start, run_start
            run_start = None
    if run_start is not None and len(dense) - run_start > best_len:
        best_len, best_start = len(dense) - run_start, run_start
    if best_start is None or best_len < 50:
        return None
    return path[best_start:best_start + best_len + 1]


# ---------------------------------------------------------------- metrics

def ang_stats(e):
    """median / p95 / max of an angular error array, in degrees.

    The max is dominated by the step response at a yaw task: the setpoint flips to the
    new heading in one cycle and the vehicle needs a second to turn, so the instantaneous
    error touches the size of the commanded turn. p95 is the number worth quoting.
    """
    return (f'median {np.median(e):.1f} deg, p95 {np.percentile(e, 95):.1f} deg, '
            f'max {e.max():.1f} deg')


def report(data, mission, map_yaw_deg):
    """Print the numbers that go into the A1.4 documentation."""
    lines = []
    t_p, xyz_l, yaw = pose_arrays(data['/mavros/local_position/pose'])
    xyz_m = local_to_map(xyz_l, map_yaw_deg)

    # Self-test: the node already publishes the flown track in the map frame, so
    # it is the reference for our own rotation. A sign error here shows up as
    # metres, not millimetres.
    flown = path_xyz(data.get('/lrs_mission/flown_path', []))
    if flown is not None:
        err = dist_to_polyline(flown, xyz_m)
        lines.append(f'frame self-test:      max {err.max():.3f} m from '
                     f'/lrs_mission/flown_path (expect < 0.05)')
    else:
        lines.append('frame self-test:      SKIPPED, record /lrs_mission/flown_path')

    sp = data.get('/mavros/setpoint_raw/local', [])
    if sp:
        t_s, sp_xyz, sp_yaw = setpoint_arrays(sp)
        idx, ok = match_nearest(t_p, t_s, max_dt=0.10)
        e = np.degrees(np.abs(wrap(yaw - sp_yaw[idx])))[ok]
        lines.append(f'yaw vs setpoint:      {ang_stats(e)}  '
                     f'({ok.sum()}/{len(ok)} paired; the rest is takeoff/land)')

    vel = data.get('/mavros/local_position/velocity_local', [])
    if vel:
        t_v, spd, v = speed_arrays(vel)
        lines.append(f'max speed:            {spd.max():.2f} m/s')
        # A1.2 rule 4: the nose points along the direction of travel.
        iv, okv = match_nearest(t_p, t_v, max_dt=0.10)
        vh = np.linalg.norm(v[iv][:, :2], axis=1)
        m = okv & (vh > TRAVEL_SPEED_MIN)
        if mission == 'outdoor':
            # Outdoors the nose is deliberately NOT along the direction of travel during the
            # square -- it is locked on the POI and the drone flies sideways and backwards
            # (A1.3 section 3). Tangent heading is only required on the figure-8 (section 5),
            # so that is the only window where this metric means anything.
            win = state_window(state_timeline(data.get('/rosout', [])), 'FIGURE8')
            if win:
                m = m & (t_p >= win[0]) & (t_p <= win[1])
            name, note = 'nose vs actual travel:', '  (figure-8 only, see setpoint lag)'
        else:
            name, note = 'nose vs travel dir:  ', f'  (|v_xy| > {TRAVEL_SPEED_MIN} m/s)'
        if m.any():
            travel = np.arctan2(v[iv][m, 1], v[iv][m, 0])
            e = np.degrees(np.abs(wrap(yaw[m] - travel)))
            lines.append(f'{name} {ang_stats(e)}{note}')

    if sp:
        # How far the vehicle trails the commanded setpoint. This is the position
        # controller's tracking error, and it is what explains the heading numbers: the
        # commanded yaw is the tangent at the *setpoint*, so a lag of L metres along a
        # curve whose tangent turns at w deg/s shows up as roughly L/speed * w degrees
        # between the nose and the actual direction of travel.
        isp, oksp = match_nearest(t_p, t_s, max_dt=0.10)
        lag = np.linalg.norm((xyz_l - sp_xyz[isp])[oksp][:, :2], axis=1)
        lines.append(f'setpoint lag:         median {np.median(lag):.3f} m, '
                     f'max {lag.max():.3f} m')

    planned = path_xyz(data.get('/lrs_mission/path', []))
    if planned is not None and mission == 'indoor':
        d = dist_to_polyline(xyz_m, planned)
        lines.append(f'path deviation:       median {np.median(d):.3f} m, '
                     f'max {d.max():.3f} m')

    if mission == 'outdoor':
        tl = state_timeline(data.get('/rosout', []))
        wps = waypoints_from_markers(data.get('/lrs_mission/waypoints', []))
        poi = next((p for p, lbl, _ in wps if lbl == 'POI'), None)
        # A1.3 section 3 wants this verifiable from the rosbag.
        if poi is not None and tl:
            w1, w4 = state_window(tl, 'corner W1'), state_window(tl, 'corner W4')
            if w1 and w4:
                m = (t_p >= w1[0]) & (t_p <= w4[1])
                if m.any():
                    bearing = np.arctan2(poi[1] - xyz_m[m, 1], poi[0] - xyz_m[m, 0])
                    e = np.degrees(np.abs(wrap(yaw[m] - bearing)))
                    lines.append(f'nose vs POI bearing:  {ang_stats(e)}  (square, W1..W4)')
        curve = figure8_segment(planned)
        if curve is not None and tl:
            w = state_window(tl, 'FIGURE8')
            m = (t_p >= w[0]) & (t_p <= w[1]) if w else np.ones(len(t_p), bool)
            if m.any():
                d = dist_to_polyline(xyz_m[m], curve)
                lines.append(f'figure-8 deviation:   median {np.median(d):.3f} m, '
                             f'max {d.max():.3f} m  ({len(curve)} commanded points)')

    print('\n'.join(lines))
    return lines


# ---------------------------------------------------------------- plots

def _finish(ax, out, name, equal=True):
    if equal:
        ax.set_aspect('equal')
    ax.grid(alpha=0.3)
    ax.legend(loc='best', fontsize=8)
    path = os.path.join(out, name)
    ax.figure.savefig(path, dpi=150, bbox_inches='tight')
    plt.close(ax.figure)
    print(f'wrote {path}')


def plot_top(data, mission, map_yaw_deg, out, racks=None):
    """Top-down view: commanded trajectory vs the flown track."""
    _, xyz_l, _ = pose_arrays(data['/mavros/local_position/pose'])
    xyz_m = local_to_map(xyz_l, map_yaw_deg)
    fig, ax = plt.subplots(figsize=(8, 7))

    for box in racks or []:
        ax.add_patch(plt.Rectangle((box[0], box[1]), box[3] - box[0], box[4] - box[1],
                                   facecolor='0.85', edgecolor='0.5', zorder=0,
                                   label='_rack'))
    if racks:
        ax.plot([], [], color='0.5', lw=6, alpha=0.5, label='racks (solid)')

    planned = path_xyz(data.get('/lrs_mission/path', []))
    if planned is not None:
        # dashed, so it stays visible where the flown track lies on top of it
        ax.plot(planned[:, 0], planned[:, 1], '--', color='tab:blue', lw=2.0,
                label='commanded / planned')
    ax.plot(xyz_m[:, 0], xyz_m[:, 1], '-', color='tab:orange', lw=1.2,
            label='flown')

    placed = []  # stagger labels of waypoints that sit on top of each other (home / RTL)
    for p, lbl, hard in waypoints_from_markers(data.get('/lrs_mission/waypoints', [])):
        if lbl == 'POI':
            ax.plot(p[0], p[1], '*', color='magenta', ms=16, label='POI')
            continue
        ax.plot(p[0], p[1], 'o', color='tab:red' if hard else 'tab:green', ms=7)
        dy = 5 - 11 * sum(1 for q in placed if np.hypot(*(q - p[:2])) < 0.4)
        placed.append(p[:2])
        ax.annotate(lbl, (p[0], p[1]), fontsize=7,
                    textcoords='offset points', xytext=(6, dy))
    ax.plot([], [], 'o', color='tab:red', label='hard waypoint')
    ax.plot([], [], 'o', color='tab:green', label='soft waypoint')

    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')
    ax.set_title(f'{mission}: commanded vs flown (map frame)')
    _finish(ax, out, f'{mission}_top.png')


def plot_altitude(data, out, racks):
    """z over time: the visual proof that the planner routes *over* the racks.

    A top-down figure cannot show this, and A1.1 section 3 requires the path to be able to
    change altitude rather than stay on one horizontal plane.
    """
    t_p, xyz_l, _ = pose_arrays(data['/mavros/local_position/pose'])
    sp = data.get('/mavros/setpoint_raw/local', [])
    t0 = t_p[0]
    fig, ax = plt.subplots(figsize=(10, 4))
    if sp:
        t_s, sp_xyz, _ = setpoint_arrays(sp)
        idx, ok = match_nearest(t_p, t_s, max_dt=0.10)
        ax.plot(t_p - t0, np.where(ok, sp_xyz[idx][:, 2], np.nan), '--',
                color='tab:blue', lw=1.5, label='commanded z')
    ax.plot(t_p - t0, xyz_l[:, 2], color='tab:orange', lw=1.2, label='actual z')
    top = max((b[5] for b in racks or []), default=None)
    if top is not None:
        ax.axhline(top, color='0.5', ls=':', lw=1.5,
                   label=f'rack top, {top:.2f} m (solid below)')
    ax.set_xlabel('t [s]')
    ax.set_ylabel('z [m]')
    ax.set_title('altitude profile (gaps = takeoff/land, no setpoints published)')
    _finish(ax, out, 'altitude.png', equal=False)


def plot_yaw(data, out):
    """Commanded vs actual yaw, and the error -- with the setpoint gaps left blank."""
    t_p, _, yaw = pose_arrays(data['/mavros/local_position/pose'])
    sp = data.get('/mavros/setpoint_raw/local', [])
    if not sp:
        return
    t_s, _, sp_yaw = setpoint_arrays(sp)
    idx, ok = match_nearest(t_p, t_s, max_dt=0.10)
    t0 = t_p[0]
    err = np.where(ok, np.degrees(wrap(yaw - sp_yaw[idx])), np.nan)

    fig, (a1, a2) = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
    # Unwrap the actual yaw for drawing, then derive the commanded one from it as
    # actual - error. Unwrapping the commanded series directly would accumulate
    # phase across the setpoint gaps and shift the curve by multiples of 360 deg;
    # this way both lines and the error subplot stay consistent by construction.
    yaw_un = np.degrees(np.unwrap(yaw))
    a1.plot(t_p - t0, yaw_un, color='tab:orange', lw=1, label='actual')
    a1.plot(t_p - t0, yaw_un - err, color='tab:blue', lw=1, label='commanded')
    a1.set_ylabel('yaw [deg]')
    a1.grid(alpha=0.3)
    a1.legend(loc='best', fontsize=8)
    a1.set_title('yaw tracking (gaps = takeoff/land, no setpoints published)')
    a2.plot(t_p - t0, err, color='tab:red', lw=1, label='error')
    a2.set_xlabel('t [s]')
    a2.set_ylabel('error [deg]')
    _finish(a2, out, 'yaw.png', equal=False)


# ---------------------------------------------------------------- main

def rack_boxes_from_config():
    """rack_boxes out of indoor.yaml, so the figure and the planner share one source.

    Tries the installed share directory first, then the source tree next to this script,
    so the figure is complete whether or not the workspace has been sourced.
    """
    import yaml
    candidates = []
    try:
        from ament_index_python.packages import get_package_share_directory
        candidates.append(os.path.join(
            get_package_share_directory('lrs_mission'), 'config', 'indoor.yaml'))
    except Exception:
        pass
    candidates.append(os.path.join(
        os.path.dirname(os.path.abspath(__file__)), '..', 'config', 'indoor.yaml'))
    for c in candidates:
        try:
            with open(c) as f:
                flat = yaml.safe_load(f)['lrs_mission']['ros__parameters']['rack_boxes']
            return [flat[i:i + 6] for i in range(0, len(flat), 6)]
        except Exception:
            continue
    print('warning: indoor.yaml not found, the racks will be missing from the figure')
    return []


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('bag')
    ap.add_argument('--mission', choices=['indoor', 'outdoor'], required=True)
    ap.add_argument('--map-yaw-deg', type=float, default=None,
                    help='map->local rotation; default 90 indoor, 0 outdoor '
                         '(matches map_to_local_yaw_deg)')
    ap.add_argument('--out', default='figures')
    args = ap.parse_args()

    yaw_deg = args.map_yaw_deg
    if yaw_deg is None:
        yaw_deg = 90.0 if args.mission == 'indoor' else 0.0
    os.makedirs(args.out, exist_ok=True)

    data = read_bag(args.bag)
    print('topics in bag: ' + ', '.join(
        f'{k} x{len(v)}' for k, v in sorted(data.items())))
    if not data.get('/mavros/local_position/pose'):
        raise SystemExit('no /mavros/local_position/pose in the bag -- nothing to plot')
    print()

    report(data, args.mission, yaw_deg)
    print()
    racks = rack_boxes_from_config() if args.mission == 'indoor' else []
    plot_top(data, args.mission, yaw_deg, args.out, racks)
    plot_altitude(data, args.out, racks)
    plot_yaw(data, args.out)


if __name__ == '__main__':
    main()
