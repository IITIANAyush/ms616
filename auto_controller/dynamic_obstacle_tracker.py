#!/usr/bin/env python3
"""
LiDAR-based dynamic obstacle tracker / predictor.

Algorithm inspired by the linked Nav2 dynamic-obstacle project:
  LaserScan -> local costmap -> DWB local controller.

The added piece here is explicit short-horizon prediction:
  1. Segment compact LiDAR clusters.
  2. Transform cluster centroids into the odom frame.
  3. Associate clusters frame-to-frame.
  4. Estimate obstacle velocity with an exponential filter.
  5. For moving tracks, project the obstacle 1-1.2 s ahead.
  6. Publish the predicted occupancy as PointCloud2.

Nav2's local costmap marks those predicted points, and DWB evaluates them
when scoring its candidate velocity trajectories. Static obstacles continue
to come directly from /scan.
"""

from dataclasses import dataclass
import math
from typing import List, Optional

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
import tf2_ros


@dataclass
class Track:
    track_id: int
    x: float
    y: float
    vx: float = 0.0
    vy: float = 0.0
    hits: int = 1
    last_seen: float = 0.0


class DynamicObstacleTracker(Node):
    def __init__(self) -> None:
        super().__init__("dynamic_obstacle_tracker")

        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("prediction_topic", "/dynamic_obstacles/predicted_points")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("cluster_distance", 0.18)
        self.declare_parameter("min_cluster_points", 3)
        self.declare_parameter("max_cluster_diameter", 0.70)
        self.declare_parameter("max_tracking_range", 4.0)
        self.declare_parameter("association_distance", 0.50)
        self.declare_parameter("track_timeout", 0.60)
        self.declare_parameter("velocity_alpha", 0.55)
        self.declare_parameter("min_dynamic_speed", 0.10)
        self.declare_parameter("min_dynamic_hits", 4)
        self.declare_parameter("prediction_horizon", 1.10)
        self.declare_parameter("prediction_step", 0.20)
        self.declare_parameter("prediction_radius", 0.12)

        self.scan_topic = str(self.get_parameter("scan_topic").value)
        self.prediction_topic = str(self.get_parameter("prediction_topic").value)
        self.odom_frame = str(self.get_parameter("odom_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.cluster_distance = float(self.get_parameter("cluster_distance").value)
        self.min_cluster_points = int(self.get_parameter("min_cluster_points").value)
        self.max_cluster_diameter = float(self.get_parameter("max_cluster_diameter").value)
        self.max_tracking_range = float(self.get_parameter("max_tracking_range").value)
        self.association_distance = float(self.get_parameter("association_distance").value)
        self.track_timeout = float(self.get_parameter("track_timeout").value)
        self.velocity_alpha = float(self.get_parameter("velocity_alpha").value)
        self.min_dynamic_speed = float(self.get_parameter("min_dynamic_speed").value)
        self.min_dynamic_hits = int(self.get_parameter("min_dynamic_hits").value)
        self.prediction_horizon = float(self.get_parameter("prediction_horizon").value)
        self.prediction_step = float(self.get_parameter("prediction_step").value)
        self.prediction_radius = float(self.get_parameter("prediction_radius").value)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        scan_qos = QoSProfile(depth=5)
        scan_qos.reliability = QoSReliabilityPolicy.BEST_EFFORT

        self.scan_sub = self.create_subscription(
            LaserScan, self.scan_topic, self.scan_callback, scan_qos
        )
        self.pred_pub = self.create_publisher(PointCloud2, self.prediction_topic, 10)

        self.tracks: List[Track] = []
        self.next_track_id = 0
        self.last_scan_time: Optional[float] = None
        self.last_dynamic_count = -1

        self.get_logger().info(
            "Dynamic obstacle predictor ready: "
            f"horizon={self.prediction_horizon:.2f}s, "
            f"min_dynamic_speed={self.min_dynamic_speed:.2f}m/s"
        )

    @staticmethod
    def _finite_range(value: float, msg: LaserScan) -> bool:
        return math.isfinite(value) and msg.range_min <= value <= msg.range_max

    def extract_clusters(self, msg: LaserScan):
        points = []
        for i, rng in enumerate(msg.ranges):
            if not self._finite_range(rng, msg) or rng > self.max_tracking_range:
                points.append(None)
                continue
            angle = msg.angle_min + i * msg.angle_increment
            points.append((rng * math.cos(angle), rng * math.sin(angle)))

        clusters = []
        current = []

        for point in points:
            if point is None:
                if current:
                    clusters.append(current)
                    current = []
                continue

            if current:
                px, py = current[-1]
                if math.hypot(point[0] - px, point[1] - py) > self.cluster_distance:
                    clusters.append(current)
                    current = []
            current.append(point)

        if current:
            clusters.append(current)

        # The scan wraps at +/- pi. Merge first/last clusters when the
        # Euclidean gap is small enough.
        if len(clusters) > 1:
            first = clusters[0]
            last = clusters[-1]
            if math.hypot(first[0][0] - last[-1][0], first[0][1] - last[-1][1]) <= self.cluster_distance:
                clusters[0] = last + first
                clusters.pop()

        centroids = []
        for cluster in clusters:
            if len(cluster) < self.min_cluster_points:
                continue
            cx = sum(p[0] for p in cluster) / len(cluster)
            cy = sum(p[1] for p in cluster) / len(cluster)
            diameter = max(math.hypot(px - cx, py - cy) for px, py in cluster) * 2.0
            if diameter > self.max_cluster_diameter:
                continue
            centroids.append((cx, cy))

        return centroids

    @staticmethod
    def transform_point(x: float, y: float, tf: TransformStamped):
        q = tf.transform.rotation
        # planar quaternion -> yaw
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        tx = tf.transform.translation.x
        ty = tf.transform.translation.y
        co = math.cos(yaw)
        si = math.sin(yaw)
        return tx + co * x - si * y, ty + si * x + co * y

    def associate(self, observations, now_sec: float) -> None:
        matched = set()

        for ox, oy in observations:
            best = None
            best_dist = self.association_distance
            for track in self.tracks:
                if track.track_id in matched:
                    continue
                dt = max(0.0, now_sec - track.last_seen)
                px = track.x + track.vx * min(dt, 0.5)
                py = track.y + track.vy * min(dt, 0.5)
                dist = math.hypot(ox - px, oy - py)
                if dist < best_dist:
                    best = track
                    best_dist = dist

            if best is None:
                best = Track(
                    track_id=self.next_track_id,
                    x=ox,
                    y=oy,
                    last_seen=now_sec,
                )
                self.next_track_id += 1
                self.tracks.append(best)
                matched.add(best.track_id)
                continue

            dt = now_sec - best.last_seen
            if dt > 1e-3:
                raw_vx = (ox - best.x) / dt
                raw_vy = (oy - best.y) / dt
                a = self.velocity_alpha
                best.vx = (1.0 - a) * best.vx + a * raw_vx
                best.vy = (1.0 - a) * best.vy + a * raw_vy

            best.x = ox
            best.y = oy
            best.hits += 1
            best.last_seen = now_sec
            matched.add(best.track_id)

        self.tracks = [t for t in self.tracks if now_sec - t.last_seen <= self.track_timeout]

    def dynamic_tracks(self, now_sec: float):
        result = []
        for track in self.tracks:
            speed = math.hypot(track.vx, track.vy)
            age = now_sec - track.last_seen
            if age <= 0.20 and track.hits >= self.min_dynamic_hits and speed >= self.min_dynamic_speed:
                result.append(track)
        return result

    def publish_predictions(self, tracks, stamp) -> None:
        points = []
        steps = max(1, int(math.ceil(self.prediction_horizon / self.prediction_step)))

        for track in tracks:
            speed = math.hypot(track.vx, track.vy)
            if speed < self.min_dynamic_speed:
                continue

            nx = -track.vy / speed
            ny = track.vx / speed
            for k in range(steps + 1):
                tau = min(self.prediction_horizon, k * self.prediction_step)
                cx = track.x + track.vx * tau
                cy = track.y + track.vy * tau

                # A small lateral tube makes the prediction robust to the
                # cluster-centroid noise and gives the costmap a footprint-like
                # occupied region rather than a single point.
                points.append((cx, cy, 0.10))
                points.append((cx + nx * self.prediction_radius, cy + ny * self.prediction_radius, 0.10))
                points.append((cx - nx * self.prediction_radius, cy - ny * self.prediction_radius, 0.10))

        header = Header()
        header.stamp = stamp
        header.frame_id = self.odom_frame
        cloud = point_cloud2.create_cloud_xyz32(header, points)
        self.pred_pub.publish(cloud)

    def scan_callback(self, msg: LaserScan) -> None:
        try:
            tf = self.tf_buffer.lookup_transform(
                self.odom_frame,
                self.base_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.05),
            )
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            return

        base_clusters = self.extract_clusters(msg)
        observations = [self.transform_point(x, y, tf) for x, y in base_clusters]

        now_sec = self.get_clock().now().nanoseconds * 1e-9
        self.associate(observations, now_sec)
        dynamics = self.dynamic_tracks(now_sec)
        self.publish_predictions(dynamics, self.get_clock().now().to_msg())

        if len(dynamics) != self.last_dynamic_count:
            self.last_dynamic_count = len(dynamics)
            self.get_logger().info(
                f"Tracking {len(dynamics)} dynamic obstacle(s)",
                throttle_duration_sec=0.5,
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DynamicObstacleTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
