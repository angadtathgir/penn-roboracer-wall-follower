#!/usr/bin/env python3
"""Run the real controller in the AutoDRIVE sim for a batch of parameter sets.

For each config: reset the car, drive with wall_follow_core.Controller until N laps are
done, a wall is touched, or time runs out. Prints one JSON line per config with the lap
times, and optionally logs the trajectory to CSV for plotting.
    python3 run_laps.py --laps 3 --configs '[{"mode":"gap"},{"mode":"wall","v_max":7}]'
"""
import argparse
import csv
import json
import math
import os
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32, Int32

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from autodrive_roboracer.wall_follow_core import Controller  # noqa: E402

NS = '/autodrive/roboracer_1'


class Runner:
    def __init__(self):
        rclpy.init()
        self.n = rclpy.create_node('run_laps')
        qos = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE,
                         history=QoSHistoryPolicy.KEEP_LAST, depth=1)
        self.s = {'v': 0.0, 'x': 0.0, 'y': 0.0, 'laps': 0, 'last': 0.0, 'coll': 0,
                  'lap_t': 0.0}
        self.scan = None
        n = self.n
        n.create_subscription(Odometry, NS + '/odom', self.on_odom, qos)
        n.create_subscription(LaserScan, NS + '/lidar', self.on_scan, qos)
        n.create_subscription(Int32, NS + '/lap_count', lambda m: self.s.update(laps=m.data), qos)
        n.create_subscription(Float32, NS + '/last_lap_time',
                              lambda m: self.s.update(last=m.data), qos)
        n.create_subscription(Float32, NS + '/lap_time', lambda m: self.s.update(lap_t=m.data), qos)
        n.create_subscription(Int32, NS + '/collision_count',
                              lambda m: self.s.update(coll=m.data), qos)
        self.pt = n.create_publisher(Float32, NS + '/throttle_command', qos)
        self.ps = n.create_publisher(Float32, NS + '/steering_command', qos)
        self.pr = n.create_publisher(Bool, '/autodrive/reset_command', qos)

    def on_odom(self, m):
        t = m.twist.twist.linear
        self.s['v'] = math.sqrt(t.x ** 2 + t.y ** 2 + t.z ** 2)
        self.s['x'], self.s['y'] = m.pose.pose.position.x, m.pose.pose.position.y

    def on_scan(self, m):
        self.scan = m

    def cmd(self, thr, steer):
        self.pt.publish(Float32(data=float(thr)))
        self.ps.publish(Float32(data=float(steer)))

    def reset(self):
        for flag, sec in ((True, 0.4), (False, 0.8)):
            end = time.time() + sec
            while time.time() < end:
                self.pr.publish(Bool(data=flag))
                self.cmd(0.0, 0.0)
                rclpy.spin_once(self.n, timeout_sec=0.01)

    def run(self, params, laps, timeout, log=None):
        ctrl = Controller()
        ctrl.params.update(params)
        self.reset()
        laps0, coll0 = self.s['laps'], self.s['coll']
        times, last_seen = [], self.s['laps']
        t0 = time.time()
        prev_scan_t = None
        w = None
        if log:
            f = open(log, 'w', newline='')
            w = csv.writer(f)
            w.writerow(['t', 'x', 'y', 'v', 'v_target', 'thr', 'steer', 'free', 'L', 'R'])
        result = 'timeout'
        while time.time() - t0 < timeout:
            self.scan = None
            rclpy.spin_once(self.n, timeout_sec=0.05)
            if self.scan is None:
                continue
            now = time.time()
            dt = 0.01 if prev_scan_t is None else now - prev_scan_t
            prev_scan_t = now
            m = self.scan
            thr, steer = ctrl.step(m.ranges, m.angle_min, m.angle_increment, self.s['v'], dt)
            self.cmd(thr, steer)
            if w:
                L = ctrl.last
                w.writerow([f'{now - t0:.3f}', f"{self.s['x']:.3f}", f"{self.s['y']:.3f}",
                            f"{self.s['v']:.2f}", f"{L['v_target']:.2f}", f'{thr:.3f}',
                            f'{steer:.3f}', f"{L['d_free']:.2f}", f"{L['left']:.2f}",
                            f"{L['right']:.2f}"])
            if self.s['coll'] > coll0:
                result = 'collision'
                break
            if self.s['laps'] > last_seen:
                last_seen = self.s['laps']
                # let last_lap_time update before reading it
                end = time.time() + 0.05
                while time.time() < end:
                    rclpy.spin_once(self.n, timeout_sec=0.01)
                times.append(round(self.s['last'], 3))
                if self.s['laps'] - laps0 >= laps:
                    result = 'ok'
                    break
        self.cmd(0.0, 0.0)
        return {'result': result, 'laps': times,
                'best': min(times) if times else None,
                'at': [round(self.s['x'], 2), round(self.s['y'], 2)],
                'params': params}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--laps', type=int, default=3)
    ap.add_argument('--timeout', type=float, default=90.0)
    ap.add_argument('--configs', required=True, help='JSON list of param dicts, or @file')
    ap.add_argument('--log', default=None, help='CSV path prefix for trajectory logs')
    a = ap.parse_args()
    cfg = a.configs
    configs = json.load(open(cfg[1:])) if cfg.startswith('@') else json.loads(cfg)
    r = Runner()
    for i, p in enumerate(configs):
        log = f'{a.log}_{i}.csv' if a.log else None
        out = r.run(p, a.laps, a.timeout, log)
        print(json.dumps(out), flush=True)
    r.reset()


if __name__ == '__main__':
    main()
