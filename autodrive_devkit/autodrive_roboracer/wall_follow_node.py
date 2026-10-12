#!/usr/bin/env python3
"""ROS 2 wall-follow node for the AutoDRIVE RoboRacer.

Two versions share this file and the controller in wall_follow_core:
    ros2 run autodrive_roboracer wall_follow_node         # LiDAR steering + odometry speed
    ros2 run autodrive_roboracer wall_follow_node_lidar   # LiDAR only (speed from scan matching)
Each runs the controller once per scan and publishes throttle and steering in [-1, 1].
Every controller parameter is a ROS parameter, e.g.
    ros2 run autodrive_roboracer wall_follow_node --ros-args -p mode:=wall -p v_max:=8.0
"""
import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32

from autodrive_roboracer.wall_follow_core import Controller

NS = '/autodrive/roboracer_1'


class WallFollowNode(Node):
    def __init__(self, lidar_only=False):
        super().__init__('wall_follow_node_lidar' if lidar_only else 'wall_follow_node')
        self.ctrl = Controller()
        if lidar_only:
            self.ctrl.params['speed_source'] = 'lidar'
        for name, default in self.ctrl.params.items():
            self.ctrl.params[name] = self.declare_parameter(name, default).value
        self.add_on_set_parameters_callback(self.on_params)

        # match the bridge: reliable, keep only the newest message
        qos = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE,
                         history=QoSHistoryPolicy.KEEP_LAST, depth=1)
        self.pub_throttle = self.create_publisher(Float32, NS + '/throttle_command', qos)
        self.pub_steering = self.create_publisher(Float32, NS + '/steering_command', qos)
        self.create_subscription(LaserScan, NS + '/lidar', self.on_scan, qos)
        # the LiDAR-only version never touches odometry; its speed comes from scan matching
        self.lidar_only = self.ctrl.params['speed_source'] == 'lidar'
        if not self.lidar_only:
            self.create_subscription(Odometry, NS + '/odom', self.on_odom, qos)

        self.speed = 0.0
        self.last_stamp = None
        self.n = 0
        self.get_logger().info(f"{self.get_name()} up, mode={self.ctrl.params['mode']}, "
                               f"speed from {'lidar' if self.lidar_only else 'odom'}")

    def on_params(self, params):
        for p in params:
            if p.name in self.ctrl.params:
                self.ctrl.params[p.name] = p.value
        return SetParametersResult(successful=True)

    def on_odom(self, msg):
        # the bridge forwards the sim's velocity vector as-is (frame not guaranteed to be
        # the car's), so use its magnitude; the car never needs to reverse
        t = msg.twist.twist.linear
        self.speed = (t.x * t.x + t.y * t.y + t.z * t.z) ** 0.5

    def on_scan(self, msg):
        now = self.get_clock().now().nanoseconds * 1e-9
        dt = 0.05 if self.last_stamp is None else min(0.2, max(1e-3, now - self.last_stamp))
        self.last_stamp = now

        throttle, steering = self.ctrl.step(msg.ranges, msg.angle_min, msg.angle_increment,
                                            None if self.lidar_only else self.speed, dt)
        self.speed = self.ctrl.last['v_used']
        self.publish(throttle, steering)

        self.n += 1
        if self.n % 20 == 0:
            s = self.ctrl.last
            self.get_logger().info(
                f"v={self.speed:4.2f} v_t={s['v_target']:4.2f} thr={throttle:+.2f} "
                f"steer={steering:+.2f} free={s['d_free']:4.2f} "
                f"L={s['left']:.2f} R={s['right']:.2f} dt={dt * 1000:.0f}ms")

    def publish(self, throttle, steering):
        self.pub_throttle.publish(Float32(data=float(throttle)))
        self.pub_steering.publish(Float32(data=float(steering)))


def main(args=None, lidar_only=False):
    rclpy.init(args=args)
    node = WallFollowNode(lidar_only)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # leave the car stopped and straight (Ctrl+C may already have shut the context)
        if rclpy.ok():
            node.publish(0.0, 0.0)
        node.destroy_node()
        rclpy.try_shutdown()


def main_lidar_only(args=None):
    main(args, lidar_only=True)


if __name__ == '__main__':
    main()
