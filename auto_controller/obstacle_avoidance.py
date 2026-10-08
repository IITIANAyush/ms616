#!/usr/bin/env python3
"""
obstacle_avoidance.py

Simple reactive obstacle-avoidance node. Reads /scan directly and
publishes /cmd_vel — no map, no planner. Splits the lidar scan into
three sectors (left / front / right), and:

  - if front is clear: drive forward
  - if front is blocked: stop, turn toward whichever side has more
    clearance
  - if nothing is in range at all: drive forward (open space)

This is meant as a simple standalone wander/avoid behaviour, and as
a safety fallback you can run alongside Nav2 (Nav2 already avoids
obstacles using the costmap, so normally you would NOT run both
publishing to /cmd_vel at once — see controller.launch.py notes).

Topics:
  Subscribes: /scan (sensor_msgs/LaserScan)
  Publishes:  /cmd_vel (geometry_msgs/Twist)

Usage (standalone, after building):
  ros2 run ms616 obstacle_avoidance
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class ObstacleAvoidance(Node):

    def __init__(self):
        super().__init__("obstacle_avoidance")

        self.declare_parameter("linear_speed", 0.15)     # m/s forward speed
        self.declare_parameter("angular_speed", 0.6)      # rad/s turning speed
        self.declare_parameter("stop_distance", 0.35)     # m — front sector trigger
        self.declare_parameter("front_half_angle_deg", 25.0)  # +/- degrees counted as "front"
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")

        self.linear_speed = self.get_parameter("linear_speed").value
        self.angular_speed = self.get_parameter("angular_speed").value
        self.stop_distance = self.get_parameter("stop_distance").value
        self.front_half_angle = math.radians(self.get_parameter("front_half_angle_deg").value)
        scan_topic = self.get_parameter("scan_topic").value
        cmd_vel_topic = self.get_parameter("cmd_vel_topic").value

        scan_qos = QoSProfile(depth=5)
        scan_qos.reliability = QoSReliabilityPolicy.BEST_EFFORT

        self.cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.scan_sub = self.create_subscription(
            LaserScan, scan_topic, self.scan_callback, scan_qos
        )

        self.get_logger().info(
            f"obstacle_avoidance started. stop_distance={self.stop_distance}m, "
            f"linear_speed={self.linear_speed}m/s"
        )

    def scan_callback(self, msg: LaserScan):
        n = len(msg.ranges)
        if n == 0:
            return

        def angle_at(i):
            return msg.angle_min + i * msg.angle_increment

        def valid(r):
            return not math.isnan(r) and not math.isinf(r) and msg.range_min <= r <= msg.range_max

        front_ranges = []
        left_ranges = []
        right_ranges = []

        for i, r in enumerate(msg.ranges):
            if not valid(r):
                continue
            a = angle_at(i)
            if -self.front_half_angle <= a <= self.front_half_angle:
                front_ranges.append(r)
            elif a > self.front_half_angle:
                left_ranges.append(r)
            else:
                right_ranges.append(r)

        front_min = min(front_ranges) if front_ranges else float("inf")
        left_min = min(left_ranges) if left_ranges else float("inf")
        right_min = min(right_ranges) if right_ranges else float("inf")

        cmd = Twist()

        if front_min < self.stop_distance:
            # Blocked ahead — stop forward motion, turn toward the
            # side with more clearance.
            cmd.linear.x = 0.0
            if left_min > right_min:
                cmd.angular.z = self.angular_speed    # turn left
            else:
                cmd.angular.z = -self.angular_speed   # turn right
        else:
            cmd.linear.x = self.linear_speed
            cmd.angular.z = 0.0

        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidance()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Publish a stop command on shutdown so the robot doesn't
        # keep coasting/turning after Ctrl+C.
        stop = Twist()
        node.cmd_pub.publish(stop)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()