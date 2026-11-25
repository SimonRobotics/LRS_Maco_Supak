#!/usr/bin/env python3
"""Make a copy of a sensor bag with outages and a GPS outlier -- offline §3 tests, repeatable.

Mimics the teacher's services of localization_sensor_sim on a recorded bag:
  --gps-off A:B   drop /localization/gps with stamps in [A, B) s after the bag's first stamp
                  (= set_gps_enabled false at A, true at B); repeatable
  --vo-off A:B    the same for /localization/vo
  --imu-off A:B   the same for /localization/imu (not a teacher service; tests the w = b path)
  --gps-outlier T inject_gps_outlier at T: the FIRST GPS sample kept at or after T gets an
                  offset of 5-8 m in a random direction, exactly like the simulator (a call
                  during a GPS outage therefore lands on the first fix after GPS returns)
  --gps-shift A:B:dx:dy:dz
                  add a constant offset to every GPS fix in [A, B) -- a jump the gate keeps
                  rejecting; tests the recovery after gps_max_rejections (not a teacher service)

  python3 lrs_localization/scripts/degrade_bag.py bags/a2_flight_seed42 /tmp/deg \\
      --gps-off 40:55 --gps-outlier 45 --vo-off 70:73 --gps-outlier 85
  lrs_localization/scripts/replay.sh /tmp/deg /tmp/deg_replay
  python3 lrs_localization/scripts/evaluate.py /tmp/deg_replay --out /tmp/fig

Truth is copied unchanged (evaluation only). Offsets are printed so they can be checked.
"""

import argparse
import shutil

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message, serialize_message
from rosidl_runtime_py.utilities import get_message

GPS, VO, IMU = '/localization/gps', '/localization/vo', '/localization/imu'


def window(s):
    a, b = s.split(':')
    return float(a), float(b)


def stamp(m):
    return m.header.stamp.sec + m.header.stamp.nanosec * 1e-9


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('inp')
    ap.add_argument('out')
    ap.add_argument('--gps-off', type=window, action='append', default=[])
    ap.add_argument('--vo-off', type=window, action='append', default=[])
    ap.add_argument('--imu-off', type=window, action='append', default=[])
    ap.add_argument('--gps-outlier', type=float, action='append', default=[])
    ap.add_argument('--gps-shift', type=lambda v: [float(x) for x in v.split(':')], action='append', default=[])
    ap.add_argument('--outlier-min', type=float, default=5.0)
    ap.add_argument('--outlier-max', type=float, default=8.0)
    ap.add_argument('--seed', type=int, default=1)
    a = ap.parse_args()
    rng = np.random.default_rng(a.seed)

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=a.inp, storage_id=''),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    topics = reader.get_all_topics_and_types()
    types = {t.name: t.type for t in topics}
    shutil.rmtree(a.out, ignore_errors=True)
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=a.out, storage_id='mcap'),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    for t in topics:
        writer.create_topic(t)

    off = {GPS: a.gps_off, VO: a.vo_off, IMU: a.imu_off}
    pending = sorted(a.gps_outlier)  # outlier requests not yet applied
    t0 = None
    dropped = {GPS: 0, VO: 0, IMU: 0}
    while reader.has_next():
        topic, raw, t_rec = reader.read_next()
        if topic in off:
            m = deserialize_message(raw, get_message(types[topic]))
            ts = stamp(m)
            if t0 is None:
                t0 = ts
            rel = ts - t0
            if any(lo <= rel < hi for lo, hi in off[topic]):
                dropped[topic] += 1
                continue
            if topic == GPS:
                for lo, hi, *dv in a.gps_shift:
                    if lo <= rel < hi:
                        p = m.pose.pose.position
                        p.x, p.y, p.z = p.x + dv[0], p.y + dv[1], p.z + dv[2]
                        raw = serialize_message(m)
            if topic == GPS and pending and rel >= pending[0]:
                req = pending.pop(0)
                d = rng.normal(size=3)
                d /= np.linalg.norm(d)
                off_v = rng.uniform(a.outlier_min, a.outlier_max) * d
                p = m.pose.pose.position
                p.x, p.y, p.z = p.x + off_v[0], p.y + off_v[1], p.z + off_v[2]
                raw = serialize_message(m)
                print(f'outlier requested at {req:.1f} s -> applied to the GPS fix at {rel:.2f} s '
                      f'(stamp {ts:.2f}): offset ({off_v[0]:+.2f}, {off_v[1]:+.2f}, {off_v[2]:+.2f}) m, '
                      f'|{np.linalg.norm(off_v):.2f}| m')
        writer.write(topic, raw, t_rec)
    del writer
    print(f'dropped: GPS {dropped[GPS]}, VO {dropped[VO]}, IMU {dropped[IMU]}  (first sensor stamp {t0:.2f})')
    if pending:
        print(f'WARNING: outlier requests never applied (no GPS after them): {pending}')


if __name__ == '__main__':
    main()
