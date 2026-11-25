#!/usr/bin/env python3
"""Run the assignment's §3 service tests during a flight, timed in simulation time.

Calls only the teacher services of localization_sensor_sim -- it never talks to the estimator
or the controller. It waits until the drone moves (speed of /lrs_localization/odom, which is
the estimate, not truth) and then runs the schedule; times are seconds after that moment:

  python3 lrs_localization/scripts/teacher_tests.py                 # default schedule below
  python3 lrs_localization/scripts/teacher_tests.py --schedule "5 gps_off; 20 gps_on"

Default schedule:
   3 s  inject_gps_outlier                 one-shot outlier in normal flight
   8 s  GPS off                            15 s outage ...
  13 s  inject_gps_outlier                 ... requested during it: lands on the first fix after
  23 s  GPS on
  33 s  VO off                             3 s tracking loss
  36 s  VO on
"""

import argparse
import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_srvs.srv import SetBool, Trigger

DEFAULT = '3 outlier; 8 gps_off; 13 outlier; 23 gps_on; 33 vo_off; 36 vo_on'


class TeacherTests(Node):
    def __init__(self, schedule, start_speed):
        super().__init__('a2_teacher_tests', parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.gps = self.create_client(SetBool, '/localization/set_gps_enabled')
        self.vo = self.create_client(SetBool, '/localization/set_vo_enabled')
        self.outlier = self.create_client(Trigger, '/localization/inject_gps_outlier')
        self.schedule = schedule
        self.start_speed = start_speed
        self.t_start = None
        self.create_subscription(Odometry, '/lrs_localization/odom', self.on_odom, 10)
        self.create_timer(0.05, self.tick)

    def on_odom(self, m):
        v = m.twist.twist.linear
        if self.t_start is None and math.hypot(v.x, v.y) > self.start_speed:
            self.t_start = self.get_clock().now().nanoseconds * 1e-9
            self.get_logger().info(f'drone moving (estimate {math.hypot(v.x, v.y):.2f} m/s): schedule starts')

    def tick(self):
        if self.t_start is None:
            return
        t = self.get_clock().now().nanoseconds * 1e-9 - self.t_start
        while self.schedule and t >= self.schedule[0][0]:
            at, what = self.schedule.pop(0)
            if what == 'outlier':
                self.outlier.call_async(Trigger.Request())
            elif what in ('gps_off', 'gps_on', 'vo_off', 'vo_on'):
                client = self.gps if what.startswith('gps') else self.vo
                client.call_async(SetBool.Request(data=what.endswith('on')))
            else:
                self.get_logger().error(f'unknown action {what}')
                continue
            self.get_logger().info(f'+{t:5.1f} s (planned {at:g} s): {what}')
        if not self.schedule:
            self.get_logger().info('schedule done')
            raise SystemExit


def parse(s):
    out = []
    for item in filter(None, (x.strip() for x in s.split(';'))):
        at, what = item.split()
        out.append((float(at), what))
    return sorted(out)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--schedule', default=DEFAULT, help=f'"time action; ..." (default "{DEFAULT}")')
    ap.add_argument('--start-speed', type=float, default=0.3, help='[m/s] horizontal speed that starts the clock')
    a = ap.parse_args()
    rclpy.init()
    node = TeacherTests(parse(a.schedule), a.start_speed)
    for c in (node.gps, node.vo, node.outlier):
        if not c.wait_for_service(timeout_sec=10.0):
            raise SystemExit(f'{c.srv_name} not available -- is localization_sensor_sim running?')
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
