#!/usr/bin/env python3
"""System identification: drive open-loop throttle/steering steps and log the response.

Logs wall time, sim lap clock, speed, pose, commands and front range to a CSV so we can
measure top speed per throttle, speed time constant, braking, and sim-vs-wall time.
    python3 sysid.py <out.csv> "<throttle>,<steer>,<seconds>;..."
"""
import csv
import math
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32, Int32

NS = '/autodrive/roboracer_1'


def main():
    out, plan = sys.argv[1], sys.argv[2]
    steps = [tuple(float(x) for x in s.split(',')) for s in plan.split(';') if s]
    rclpy.init()
    n = rclpy.create_node('sysid')
    qos = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE,
                     history=QoSHistoryPolicy.KEEP_LAST, depth=1)
    st = {'v': 0.0, 'x': 0.0, 'y': 0.0, 'lap_t': 0.0, 'coll': 0, 'front': 0.0, 'scans': 0}

    def odom(m):
        t = m.twist.twist.linear
        st['v'] = math.sqrt(t.x ** 2 + t.y ** 2 + t.z ** 2)
        st['x'], st['y'] = m.pose.pose.position.x, m.pose.pose.position.y

    def scan(m):
        st['front'] = m.ranges[len(m.ranges) // 2]
        st['scans'] += 1

    n.create_subscription(Odometry, NS + '/odom', odom, qos)
    n.create_subscription(LaserScan, NS + '/lidar', scan, qos)
    n.create_subscription(Float32, NS + '/lap_time', lambda m: st.update(lap_t=m.data), qos)
    n.create_subscription(Int32, NS + '/collision_count', lambda m: st.update(coll=m.data), qos)
    pt = n.create_publisher(Float32, NS + '/throttle_command', qos)
    ps = n.create_publisher(Float32, NS + '/steering_command', qos)
    pr = n.create_publisher(Bool, '/autodrive/reset_command', qos)

    def spin_for(sec, thr, steer, w=None):
        end = time.time() + sec
        while time.time() < end:
            pt.publish(Float32(data=thr))
            ps.publish(Float32(data=steer))
            rclpy.spin_once(n, timeout_sec=0.005)
            if w:
                w.writerow([f'{time.time():.4f}', f"{st['lap_t']:.4f}", f"{st['v']:.4f}",
                            f"{st['x']:.4f}", f"{st['y']:.4f}", thr, steer,
                            f"{st['front']:.3f}", st['coll'], st['scans']])

    # reset: True for a moment, then False (the sim keeps resetting while it is True)
    for flag in (True, False):
        end = time.time() + 0.5
        while time.time() < end:
            pr.publish(Bool(data=flag))
            pt.publish(Float32(data=0.0))
            ps.publish(Float32(data=0.0))
            rclpy.spin_once(n, timeout_sec=0.01)
    spin_for(1.0, 0.0, 0.0)

    with open(out, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['wall', 'lap_t', 'v', 'x', 'y', 'thr', 'steer', 'front', 'coll', 'scans'])
        for thr, steer, sec in steps:
            spin_for(sec, float(thr), float(steer), w)
    spin_for(0.3, 0.0, 0.0)
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
