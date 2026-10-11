#!/usr/bin/env python3
"""Straight-line launch test: compare full throttle with slip-limited throttle.

    python3 accel_test.py <slip>     # slip 0 = full throttle; 0.15 = wheel speed 15% above v
Prints the time and distance to reach 3, 5 and 7 m/s.
"""
import math
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import Bool, Float32

NS = '/autodrive/roboracer_1'
V_FULL = 24.9


def main():
    slip, offset = float(sys.argv[1]), float(sys.argv[2]) if len(sys.argv) > 2 else 0.5
    rclpy.init()
    n = rclpy.create_node('accel_test')
    qos = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE,
                     history=QoSHistoryPolicy.KEEP_LAST, depth=1)
    st = {'v': 0.0, 'x': 0.0, 'y': 0.0}

    def odom(m):
        t = m.twist.twist.linear
        st['v'] = math.sqrt(t.x ** 2 + t.y ** 2 + t.z ** 2)
        st['x'], st['y'] = m.pose.pose.position.x, m.pose.pose.position.y

    n.create_subscription(Odometry, NS + '/odom', odom, qos)
    pt = n.create_publisher(Float32, NS + '/throttle_command', qos)
    ps = n.create_publisher(Float32, NS + '/steering_command', qos)
    pr = n.create_publisher(Bool, '/autodrive/reset_command', qos)
    for flag, sec in ((True, 0.4), (False, 1.0)):
        end = time.time() + sec
        while time.time() < end:
            pr.publish(Bool(data=flag))
            pt.publish(Float32(data=0.0))
            ps.publish(Float32(data=0.0))
            rclpy.spin_once(n, timeout_sec=0.01)
    x0, y0, t0 = st['x'], st['y'], time.time()
    marks = [3.0, 5.0, 7.0]
    while marks and time.time() - t0 < 3.0:
        thr = 1.0 if slip == 0 else min(1.0, (st['v'] * (1 + slip) + offset) / V_FULL)
        pt.publish(Float32(data=thr))
        ps.publish(Float32(data=0.0))
        rclpy.spin_once(n, timeout_sec=0.005)
        if st['v'] >= marks[0]:
            d = math.hypot(st['x'] - x0, st['y'] - y0)
            print(f'slip={slip} reached {marks.pop(0)} m/s in {time.time() - t0:.2f} s, {d:.2f} m')
    for _ in range(100):
        pt.publish(Float32(data=0.0))
        rclpy.spin_once(n, timeout_sec=0.01)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
