#!/usr/bin/env python3
"""
frontier_explorer.py

Minimal autonomous frontier-exploration node for ROS 2.

Strategy:
  1. Subscribe to /map (nav_msgs/OccupancyGrid) from slam_toolbox.
  2. Find "frontier" cells: free cells adjacent to at least one
     unknown (-1) cell. Cluster them with a simple flood fill.
  3. Pick the centroid of the nearest sufficiently-large frontier
     cluster (by robot pose from TF), send it as a Nav2 NavigateToPose
     goal via the action client.
  4. On goal success/failure/timeout, re-scan the latest map and
     pick the next frontier. Stop when no frontier clusters remain
     (environment considered fully mapped) or after max_no_frontier
     consecutive empty scans.

This intentionally does NOT reimplement Nav2 path planning/costmaps —
it only decides *where* to go next and hands that off to Nav2, which
must already be running (nav2_bringup) alongside this node.
"""

import math
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose

from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException


FREE_THRESH = 50       # occupancy value below this is "free" (0-100 scale)
UNKNOWN_VALUE = -1
MIN_FRONTIER_SIZE = 6  # minimum cells in a cluster to consider it worth visiting
MAX_CONSECUTIVE_EMPTY = 3


class FrontierExplorer(Node):
    def __init__(self):
        super().__init__("frontier_explorer")

        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("goal_timeout_sec", 60.0)
        self.declare_parameter("replan_period_sec", 3.0)

        self.map_topic = self.get_parameter("map_topic").value
        self.base_frame = self.get_parameter("base_frame").value
        self.map_frame = self.get_parameter("map_frame").value
        self.goal_timeout_sec = self.get_parameter("goal_timeout_sec").value
        self.replan_period_sec = self.get_parameter("replan_period_sec").value

        map_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.map_sub = self.create_subscription(
            OccupancyGrid, self.map_topic, self.map_callback, map_qos
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")

        self.latest_map = None
        self.goal_in_progress = False
        self.consecutive_empty = 0
        self.visited_frontiers = []  # (x, y) world coords we've already tried

        self.timer = self.create_timer(self.replan_period_sec, self.explore_step)
        self.get_logger().info("Frontier explorer started, waiting for map + Nav2 action server...")

    def map_callback(self, msg: OccupancyGrid):
        self.latest_map = msg

    def get_robot_pose(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, rclpy.time.Time()
            )
            return tf.transform.translation.x, tf.transform.translation.y
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(f"TF lookup failed: {e}")
            return None

    def find_frontiers(self, grid: OccupancyGrid):
        """Return list of frontier cluster centroids in world coordinates."""
        w = grid.info.width
        h = grid.info.height
        res = grid.info.resolution
        ox = grid.info.origin.position.x
        oy = grid.info.origin.position.y
        data = grid.data

        def idx(x, y):
            return y * w + x

        def is_free(x, y):
            return 0 <= x < w and 0 <= y < h and 0 <= data[idx(x, y)] < FREE_THRESH

        def is_unknown(x, y):
            return 0 <= x < w and 0 <= y < h and data[idx(x, y)] == UNKNOWN_VALUE

        # 1. Mark frontier cells: free cells with at least one unknown neighbor
        frontier_cells = set()
        neighbors4 = [(-1, 0), (1, 0), (0, -1), (0, 1)]
        for y in range(h):
            for x in range(w):
                if not is_free(x, y):
                    continue
                for dx, dy in neighbors4:
                    if is_unknown(x + dx, y + dy):
                        frontier_cells.add((x, y))
                        break

        # 2. Cluster frontier cells via BFS flood fill (8-connectivity)
        visited = set()
        clusters = []
        neighbors8 = [(-1, -1), (-1, 0), (-1, 1), (0, -1),
                      (0, 1), (1, -1), (1, 0), (1, 1)]

        for cell in frontier_cells:
            if cell in visited:
                continue
            cluster = []
            q = deque([cell])
            visited.add(cell)
            while q:
                cx, cy = q.popleft()
                cluster.append((cx, cy))
                for dx, dy in neighbors8:
                    n = (cx + dx, cy + dy)
                    if n in frontier_cells and n not in visited:
                        visited.add(n)
                        q.append(n)
            if len(cluster) >= MIN_FRONTIER_SIZE:
                clusters.append(cluster)

        # 3. Centroid of each cluster, in world coordinates
        centroids = []
        for cluster in clusters:
            cx = sum(c[0] for c in cluster) / len(cluster)
            cy = sum(c[1] for c in cluster) / len(cluster)
            wx = ox + (cx + 0.5) * res
            wy = oy + (cy + 0.5) * res
            centroids.append((wx, wy, len(cluster)))

        return centroids

    def explore_step(self):
        if self.goal_in_progress:
            return
        if self.latest_map is None:
            self.get_logger().info("No map received yet.", throttle_duration_sec=5.0)
            return
        if not self.nav_client.wait_for_server(timeout_sec=0.5):
            self.get_logger().info("Waiting for Nav2 action server...", throttle_duration_sec=5.0)
            return

        pose = self.get_robot_pose()
        if pose is None:
            return
        rx, ry = pose

        frontiers = self.find_frontiers(self.latest_map)
        # Filter out frontiers close to ones we've already visited/failed
        frontiers = [
            f for f in frontiers
            if all(math.hypot(f[0] - vx, f[1] - vy) > 0.5 for vx, vy in self.visited_frontiers)
        ]

        if not frontiers:
            self.consecutive_empty += 1
            self.get_logger().info(
                f"No frontier clusters found ({self.consecutive_empty}/{MAX_CONSECUTIVE_EMPTY})."
            )
            if self.consecutive_empty >= MAX_CONSECUTIVE_EMPTY:
                self.get_logger().info("Exploration complete — no reachable frontiers remain.")
            return

        self.consecutive_empty = 0

        # Pick nearest frontier (largest also possible — nearest is safer/faster)
        frontiers.sort(key=lambda f: math.hypot(f[0] - rx, f[1] - ry))
        target_x, target_y, size = frontiers[0]

        self.get_logger().info(
            f"Selected frontier at ({target_x:.2f}, {target_y:.2f}), cluster size={size}"
        )
        self.send_goal(target_x, target_y)

    def send_goal(self, x, y):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = self.map_frame
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.w = 1.0  # heading doesn't matter for exploration

        self.goal_in_progress = True
        send_future = self.nav_client.send_goal_async(goal_msg)
        send_future.add_done_callback(lambda f: self._on_goal_response(f, x, y))

    def _on_goal_response(self, future, x, y):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Goal rejected by Nav2.")
            self.visited_frontiers.append((x, y))
            self.goal_in_progress = False
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda f: self._on_goal_result(f, x, y))

    def _on_goal_result(self, future, x, y):
        self.visited_frontiers.append((x, y))
        self.goal_in_progress = False
        try:
            status = future.result().status
            self.get_logger().info(f"Goal to ({x:.2f}, {y:.2f}) finished with status {status}")
        except Exception as e:
            self.get_logger().warn(f"Goal result error: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
