#!/usr/bin/env python3
"""
object_detector.py

Lightweight lidar-based object/obstacle detector.

NOTE: ms616.urdf only defines a 2D lidar (no camera), so this does
clustering on /scan rather than vision-based detection. It groups
consecutive lidar points into clusters (a simple distance-jump
segmentation), estimates each cluster's centroid and approximate
width, and publishes them as a PoseArray in the lidar's own frame
(lidar_scan_frame) so you can visualize them in RViz or consume
them in another node (e.g. to avoid or approach specific objects,
distinct from the general obstacle_avoidance.py safety layer).

If you add a camera later, this node's clustering can be extended
or replaced with a vision-based detector fairly easily — the
publish interface (PoseArray of detected object centroids) would
stay the same.

Topics:
  Subscribes: /scan (sensor_msgs/LaserScan)
  Publishes:  /detected_objects (geometry_msgs/PoseArray)

Usage (standalone, after building):
  ros2 run ms616 object_detector
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseArray, Pose


class ObjectDetector(Node):

    def __init__(self):
        super().__init__("object_detector")

        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("output_topic", "/detected_objects")
        self.declare_parameter("cluster_jump_threshold", 0.15)  # m — range gap that splits a cluster
        self.declare_parameter("min_cluster_points", 3)
        self.declare_parameter("max_cluster_width", 1.0)  # m — reject clusters wider than this (likely walls)

        scan_topic = self.get_parameter("scan_topic").value
        output_topic = self.get_parameter("output_topic").value
        self.jump_threshold = self.get_parameter("cluster_jump_threshold").value
        self.min_points = self.get_parameter("min_cluster_points").value
        self.max_width = self.get_parameter("max_cluster_width").value

        scan_qos = QoSProfile(depth=5)
        scan_qos.reliability = QoSReliabilityPolicy.BEST_EFFORT

        self.pub = self.create_publisher(PoseArray, output_topic, 10)
        self.scan_sub = self.create_subscription(
            LaserScan, scan_topic, self.scan_callback, scan_qos
        )

        self.get_logger().info(
            f"object_detector started. clustering /scan -> {output_topic}"
        )

    def scan_callback(self, msg: LaserScan):
        points = []  # list of (x, y) in the scan frame
        for i, r in enumerate(msg.ranges):
            if math.isnan(r) or math.isinf(r) or not (msg.range_min <= r <= msg.range_max):
                points.append(None)
                continue
            angle = msg.angle_min + i * msg.angle_increment
            x = r * math.cos(angle)
            y = r * math.sin(angle)
            points.append((x, y, r))

        clusters = []
        current = []
        prev_r = None

        for p in points:
            if p is None:
                if current:
                    clusters.append(current)
                    current = []
                prev_r = None
                continue

            x, y, r = p
            if prev_r is not None and abs(r - prev_r) > self.jump_threshold:
                if current:
                    clusters.append(current)
                current = []
            current.append((x, y))
            prev_r = r

        if current:
            clusters.append(current)

        pose_array = PoseArray()
        pose_array.header.stamp = msg.header.stamp
        pose_array.header.frame_id = msg.header.frame_id  # lidar_scan_frame

        detected_count = 0
        for cluster in clusters:
            if len(cluster) < self.min_points:
                continue

            xs = [c[0] for c in cluster]
            ys = [c[1] for c in cluster]
            cx = sum(xs) / len(xs)
            cy = sum(ys) / len(ys)

            width = math.hypot(xs[0] - xs[-1], ys[0] - ys[-1])
            if width > self.max_width:
                # too wide to be a discrete object — probably a wall segment
                continue

            pose = Pose()
            pose.position.x = cx
            pose.position.y = cy
            pose.position.z = 0.0
            pose.orientation.w = 1.0
            pose_array.poses.append(pose)
            detected_count += 1

        self.pub.publish(pose_array)
        if detected_count:
            self.get_logger().debug(f"Detected {detected_count} object(s) this scan.")


def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()