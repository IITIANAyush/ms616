#!/usr/bin/env python3
"""
frontier_explorer.py  —  ms616 autonomous mapping node
=======================================================

Algorithm: Wavefront Frontier Detection (WFD)
  - Runs BFS from the robot's current cell outward on the occupancy grid
  - A "frontier" cell = free cell (0) adjacent to at least one unknown cell (-1)
  - Groups adjacent frontier cells into clusters via a second BFS
  - Scores each cluster and sends the best one as a Nav2 NavigateToPose goal

Why WFD over naive "find nearest unknown":
  - Nearest-unknown picks cells AT the boundary of unknown space, which land
    inside or right on the inflation zone → Nav2 rejects or bot gets stuck
  - WFD starts from free space and expands outward, so frontiers are always
    reachable, free cells that border unknown space — costmap-safe by construction
  - Clustering prevents sending many goals to the same frontier wall segment

Blacklisting:
  - Any goal the robot fails to reach (Nav2 returns FAILED/CANCELED) is
    blacklisted with a 1.5m radius for 120 s before being reconsidered
  - Prevents infinite retry loops at the same stuck corner

Costmap safety check:
  - Before sending any goal, we query the global costmap
  - Goals with costmap cost > COSTMAP_SAFE_THRESHOLD are skipped entirely

Recovery:
  - If no frontier can be found for STUCK_TIMEOUT seconds, the node
    publishes a single cmd_vel to back the robot up, then retries
"""

import math
import time
from collections import deque
from dataclasses import dataclass, field

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus


# ── tunables ────────────────────────────────────────────────────────────────
MIN_FRONTIER_SIZE       = 5      # ignore frontier clusters smaller than this (cells)
COSTMAP_SAFE_THRESHOLD  = 60     # 0=free 254=lethal; skip goals above this
GOAL_TIMEOUT            = 30.0   # seconds before we cancel a Nav2 goal
STUCK_TIMEOUT           = 20.0   # seconds without a frontier before recovery backup
BLACKLIST_RADIUS        = 1.5    # metres — blacklist sphere around failed goals
BLACKLIST_DURATION      = 120.0  # seconds a blacklisted goal stays blacklisted
REPLAN_INTERVAL         = 3.0    # seconds between frontier re-evaluations
BACKUP_DURATION         = 2.0    # seconds of backup cmd_vel on stuck recovery
BACKUP_SPEED            = -0.15  # m/s backward
SCORE_DISTANCE_WEIGHT   = 0.3    # penalise far frontiers
SCORE_SIZE_WEIGHT       = 0.7    # reward large frontier clusters
# ────────────────────────────────────────────────────────────────────────────


@dataclass
class BlacklistEntry:
    x: float
    y: float
    expires: float  # time.time() value


@dataclass
class FrontierCluster:
    cells: list = field(default_factory=list)   # list of (row, col)
    centroid_x: float = 0.0
    centroid_y: float = 0.0
    size: int = 0


class FrontierExplorer(Node):

    def __init__(self):
        super().__init__("frontier_explorer")

        self.declare_parameter("use_sim_time", True)

        # ── state ───────────────────────────────────────────────────────────
        self.map_data: OccupancyGrid | None = None
        self.robot_x: float = 0.0
        self.robot_y: float = 0.0
        self.robot_frame_ok: bool = False

        self.blacklist: list[BlacklistEntry] = []
        self.current_goal_handle = None
        self.goal_active: bool = False
        self.last_frontier_time: float = time.time()
        self.last_replan_time: float = 0.0

        # ── Nav2 action client ───────────────────────────────────────────────
        self._nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")

        # ── subscriptions ───────────────────────────────────────────────────
        map_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(OccupancyGrid, "/map", self._map_cb, map_qos)

        # ── cmd_vel for backup recovery ──────────────────────────────────────
        self._cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        # ── TF for robot pose ────────────────────────────────────────────────
        from tf2_ros import Buffer, TransformListener
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # ── main loop timer ──────────────────────────────────────────────────
        self.create_timer(1.0, self._loop)

        self.get_logger().info("FrontierExplorer ready — WFD algorithm active")

    # ════════════════════════════════════════════════════════════════════════
    # Callbacks
    # ════════════════════════════════════════════════════════════════════════

    def _map_cb(self, msg: OccupancyGrid):
        self.map_data = msg

    def _update_robot_pose(self):
        """Pull robot pose from TF (map → base_link)."""
        try:
            tf = self._tf_buffer.lookup_transform(
                "map", "base_link", rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.1)
            )
            self.robot_x = tf.transform.translation.x
            self.robot_y = tf.transform.translation.y
            self.robot_frame_ok = True
        except Exception:
            self.robot_frame_ok = False

    # ════════════════════════════════════════════════════════════════════════
    # Main loop
    # ════════════════════════════════════════════════════════════════════════

    def _loop(self):
        self._update_robot_pose()

        if self.map_data is None:
            self.get_logger().info("Waiting for /map …", throttle_duration_sec=5.0)
            return
        if not self.robot_frame_ok:
            self.get_logger().info("Waiting for TF map→base_link …", throttle_duration_sec=5.0)
            return

        # Prune expired blacklist entries
        now = time.time()
        self.blacklist = [b for b in self.blacklist if b.expires > now]

        # Don't replan while a goal is in flight (unless timeout exceeded)
        if self.goal_active:
            if now - self.last_replan_time > GOAL_TIMEOUT:
                self.get_logger().warn("Goal timed out — cancelling and replanning")
                self._cancel_goal()
            else:
                return

        # Throttle replan rate
        if now - self.last_replan_time < REPLAN_INTERVAL:
            return

        self.last_replan_time = now

        # ── WFD ─────────────────────────────────────────────────────────────
        clusters = self._wavefront_frontier_detection()

        if not clusters:
            self.get_logger().info("No frontiers found", throttle_duration_sec=5.0)
            if now - self.last_frontier_time > STUCK_TIMEOUT:
                self.get_logger().warn("Stuck! Running backup recovery …")
                self._do_backup()
                self.last_frontier_time = now
            return

        self.last_frontier_time = now

        # Score and pick best frontier
        best = self._score_and_pick(clusters)
        if best is None:
            self.get_logger().info("All frontiers blacklisted or unsafe")
            return

        self.get_logger().info(
            f"Sending goal → ({best.centroid_x:.2f}, {best.centroid_y:.2f})  "
            f"cluster size={best.size}"
        )
        self._send_goal(best.centroid_x, best.centroid_y)

    # ════════════════════════════════════════════════════════════════════════
    # Wavefront Frontier Detection
    # ════════════════════════════════════════════════════════════════════════

    def _wavefront_frontier_detection(self) -> list[FrontierCluster]:
        """
        WFD in two passes:

        Pass 1 — outer BFS from robot cell through FREE cells:
          Mark any free cell adjacent to an unknown cell as a FRONTIER cell.

        Pass 2 — inner BFS on FRONTIER cells:
          Group connected frontier cells into clusters.

        This guarantees every frontier centroid is reachable free space.
        """
        grid = self.map_data
        w     = grid.info.width
        h     = grid.info.height
        res   = grid.info.resolution
        ox    = grid.info.origin.position.x
        oy    = grid.info.origin.position.y
        data  = grid.data   # flat list, row-major; -1=unknown, 0=free, 100=occupied

        def idx(r, c):
            return r * w + c

        def in_bounds(r, c):
            return 0 <= r < h and 0 <= c < w

        def cell_val(r, c):
            return data[idx(r, c)]

        def is_free(r, c):
            return cell_val(r, c) == 0

        def is_unknown(r, c):
            return cell_val(r, c) == -1

        def is_occupied(r, c):
            # treat anything > 50 as occupied (avoids inflation zone centroids)
            return cell_val(r, c) > 50

        # Robot cell
        robot_col = int((self.robot_x - ox) / res)
        robot_row = int((self.robot_y - oy) / res)

        if not in_bounds(robot_row, robot_col):
            self.get_logger().warn("Robot cell out of map bounds")
            return []

        NEIGHBORS_4 = [(-1, 0), (1, 0), (0, -1), (0, 1)]
        NEIGHBORS_8 = [(-1,-1),(-1,0),(-1,1),(0,-1),(0,1),(1,-1),(1,0),(1,1)]

        # ── Pass 1: BFS outward from robot through free cells ────────────────
        frontier_cells = set()
        visited_outer  = set()
        queue = deque()

        start = (robot_row, robot_col)
        queue.append(start)
        visited_outer.add(start)

        while queue:
            r, c = queue.popleft()

            is_frontier = False
            for dr, dc in NEIGHBORS_4:
                nr, nc = r + dr, c + dc
                if not in_bounds(nr, nc):
                    continue
                if is_unknown(nr, nc):
                    is_frontier = True  # this free cell borders unknown space
                elif is_free(nr, nc) and (nr, nc) not in visited_outer:
                    visited_outer.add((nr, nc))
                    queue.append((nr, nc))

            if is_frontier:
                frontier_cells.add((r, c))

        if not frontier_cells:
            return []

        # ── Pass 2: cluster frontier cells via BFS on frontier_cells ─────────
        clusters: list[FrontierCluster] = []
        visited_inner = set()

        for seed in frontier_cells:
            if seed in visited_inner:
                continue

            cluster_queue = deque([seed])
            visited_inner.add(seed)
            cluster_members = []

            while cluster_queue:
                cr, cc = cluster_queue.popleft()
                cluster_members.append((cr, cc))
                for dr, dc in NEIGHBORS_8:  # 8-connected for tighter clusters
                    nb = (cr + dr, cc + dc)
                    if nb in frontier_cells and nb not in visited_inner:
                        visited_inner.add(nb)
                        cluster_queue.append(nb)

            if len(cluster_members) < MIN_FRONTIER_SIZE:
                continue  # skip tiny noise clusters

            # Centroid in world coords
            mean_row = sum(r for r, _ in cluster_members) / len(cluster_members)
            mean_col = sum(c for _, c in cluster_members) / len(cluster_members)
            cx = ox + (mean_col + 0.5) * res
            cy = oy + (mean_row + 0.5) * res

            fc = FrontierCluster()
            fc.cells = cluster_members
            fc.centroid_x = cx
            fc.centroid_y = cy
            fc.size = len(cluster_members)
            clusters.append(fc)

        return clusters

    # ════════════════════════════════════════════════════════════════════════
    # Scoring
    # ════════════════════════════════════════════════════════════════════════

    def _score_and_pick(self, clusters: list[FrontierCluster]) -> FrontierCluster | None:
        """
        Score = SIZE_WEIGHT * normalised_size
              - DISTANCE_WEIGHT * normalised_distance

        Bigger clusters → more new map revealed per navigation attempt.
        Closer clusters → less travel, fewer failure opportunities.
        Both are normalised 0-1 before weighting so they're comparable.
        """
        # Filter blacklisted and costmap-unsafe
        safe = []
        for fc in clusters:
            if self._is_blacklisted(fc.centroid_x, fc.centroid_y):
                continue
            if not self._is_costmap_safe(fc.centroid_x, fc.centroid_y):
                continue
            safe.append(fc)

        if not safe:
            return None

        sizes = [fc.size for fc in safe]
        dists = [
            math.hypot(fc.centroid_x - self.robot_x, fc.centroid_y - self.robot_y)
            for fc in safe
        ]

        max_size = max(sizes) or 1
        max_dist = max(dists) or 1

        best_fc    = None
        best_score = -1e9

        for fc, sz, dist in zip(safe, sizes, dists):
            norm_size = sz / max_size
            norm_dist = dist / max_dist
            score = (SCORE_SIZE_WEIGHT * norm_size
                     - SCORE_DISTANCE_WEIGHT * norm_dist)
            if score > best_score:
                best_score = score
                best_fc    = fc

        return best_fc

    # ════════════════════════════════════════════════════════════════════════
    # Safety checks
    # ════════════════════════════════════════════════════════════════════════

    def _is_blacklisted(self, x: float, y: float) -> bool:
        for b in self.blacklist:
            if math.hypot(x - b.x, y - b.y) < BLACKLIST_RADIUS:
                return True
        return False

    def _is_costmap_safe(self, x: float, y: float) -> bool:
        """Check the global costmap at (x, y). Returns True if safe."""
        if self.map_data is None:
            return True  # can't check, assume safe

        grid = self.map_data
        res  = grid.info.resolution
        ox   = grid.info.origin.position.x
        oy   = grid.info.origin.position.y
        w    = grid.info.width
        h    = grid.info.height

        col = int((x - ox) / res)
        row = int((y - oy) / res)

        if not (0 <= row < h and 0 <= col < w):
            return False

        cost = grid.data[row * w + col]
        # -1 = unknown (we WANT to go to unknown-adjacent cells, not unknown itself)
        # frontier centroids are free cells so cost should be 0
        # but we also reject anything already in the inflation zone
        if cost < 0:
            return False   # unknown cell — centroid shouldn't be here; skip
        return cost < COSTMAP_SAFE_THRESHOLD

    # ════════════════════════════════════════════════════════════════════════
    # Nav2 goal sending
    # ════════════════════════════════════════════════════════════════════════

    def _send_goal(self, x: float, y: float):
        if not self._nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn("NavigateToPose server not available")
            return

        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.w = 1.0  # heading doesn't matter for exploration

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        send_future = self._nav_client.send_goal_async(
            goal_msg, feedback_callback=self._feedback_cb
        )
        send_future.add_done_callback(lambda f: self._goal_accepted_cb(f, x, y))
        self.goal_active = True
        self.last_replan_time = time.time()

    def _goal_accepted_cb(self, future, x: float, y: float):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn("Goal rejected by Nav2")
            self.goal_active = False
            self._blacklist(x, y)
            return
        self.current_goal_handle = handle
        handle.get_result_async().add_done_callback(
            lambda f: self._result_cb(f, x, y)
        )

    def _result_cb(self, future, x: float, y: float):
        self.goal_active = False
        self.current_goal_handle = None
        result = future.result()
        status = result.status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f"Reached ({x:.2f}, {y:.2f})")
        else:
            self.get_logger().warn(
                f"Goal ({x:.2f}, {y:.2f}) failed (status={status}) — blacklisting"
            )
            self._blacklist(x, y)

    def _feedback_cb(self, feedback):
        pass  # could log distance remaining here if wanted

    def _cancel_goal(self):
        if self.current_goal_handle is not None:
            self.current_goal_handle.cancel_goal_async()
        self.goal_active = False
        self.current_goal_handle = None

    def _blacklist(self, x: float, y: float):
        self.blacklist.append(
            BlacklistEntry(x=x, y=y, expires=time.time() + BLACKLIST_DURATION)
        )
        self.get_logger().info(f"Blacklisted ({x:.2f}, {y:.2f}) for {BLACKLIST_DURATION}s")

    # ════════════════════════════════════════════════════════════════════════
    # Recovery
    # ════════════════════════════════════════════════════════════════════════

    def _do_backup(self):
        """Publish a short backward cmd_vel burst to escape local minima."""
        twist = Twist()
        twist.linear.x = BACKUP_SPEED
        deadline = time.time() + BACKUP_DURATION
        rate = self.create_rate(10)
        while time.time() < deadline:
            self._cmd_pub.publish(twist)
            rate.sleep()
        # Stop
        self._cmd_pub.publish(Twist())


# ═══════════════════════════════════════════════════════════════════════════
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