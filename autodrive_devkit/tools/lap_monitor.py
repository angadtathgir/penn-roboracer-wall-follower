#!/usr/bin/env python3
"""Reset the car, then print every completed lap (time + wall contacts) for N seconds.
Run it next to wall_follow_node to check the real node:  python3 lap_monitor.py 45
"""
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float32, Int32

NS = '/autodrive/roboracer_1'


def main():
    dur = float(sys.argv[1]) if len(sys.argv) > 1 else 45.0
    rclpy.init()
    n = rclpy.create_node('lap_monitor')
    s = {}
    n.create_subscription(Int32, NS + '/lap_count', lambda m: s.update(l=m.data), 1)
    n.create_subscription(Int32, NS + '/collision_count', lambda m: s.update(c=m.data), 1)
    n.create_subscription(Float32, NS + '/last_lap_time', lambda m: s.update(t=m.data), 1)
    n.create_subscription(Float32, NS + '/best_lap_time', lambda m: s.update(b=m.data), 1)
    n.create_subscription(Odometry, NS + '/odom', lambda m: s.update(
        x=m.pose.pose.position.x, y=m.pose.pose.position.y), 1)
    pr = n.create_publisher(Bool, '/autodrive/reset_command', 1)
    for flag, sec in ((True, 0.4), (False, 0.6)):
        end = time.time() + sec
        while time.time() < end:
            pr.publish(Bool(data=flag))
            rclpy.spin_once(n, timeout_sec=0.01)
    while 'l' not in s or 'c' not in s:
        rclpy.spin_once(n, timeout_sec=0.05)
    l0, c0, last, clast = s['l'], s['c'], s['l'], s['c']
    t0 = time.time()
    while time.time() - t0 < dur:
        rclpy.spin_once(n, timeout_sec=0.05)
        if s['c'] != clast:
            clast = s['c']
            print(f"wall contact at x={s.get('x', 0):.2f} y={s.get('y', 0):.2f} "
                  f"after {time.time() - t0:.1f} s", flush=True)
        if s['l'] != last:
            last = s['l']
            end = time.time() + 0.1
            while time.time() < end:
                rclpy.spin_once(n, timeout_sec=0.01)
            print(f"lap {s['l'] - l0}: {s.get('t', 0):.3f} s   wall contacts so far: "
                  f"{s['c'] - c0}", flush=True)
    print(f"done: {s['l'] - l0} laps, {s['c'] - c0} wall contacts", flush=True)


if __name__ == '__main__':
    main()
