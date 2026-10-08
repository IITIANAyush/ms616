#!/usr/bin/env python3
"""
autonomous_mapper.py

Basic frontier-exploration node for autonomous SLAM mapping.

Subscribes to the occupancy grid slam_toolbox is building (/map),
finds "frontier" cells (free cells adjacent to unknown cells),
picks the nearest reachable frontier, and sends it to Nav2 as a
NavigateToPose action goal. When no frontiers remain, exploration
is considered complete and the node stops sending goals.

This is intentionally simple (nearest-frontier, not information-
gain based) — good enough to get autonomous exploration working
end to end. Swap select_frontier() for something smarter later if
you want.

Requires: slam_toolbox running (publishes /map), Nav2 bringup
running (provides the navigate_to_pose action server), and TF
map -> odom -> base_link all connected.

Topics:
  Subscribes: /map (nav_msgs/OccupancyGrid)
  Action client: navigate_to_pose (nav2_msgs/action/NavigateToPose)

Usage (standalone, after building):
  ros2 run ms616 autonomous_mapper

Usage (launch):
  ros2 launch ms616 slam_explore.launch.py
"""

import math

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy

from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import NavigateToPose

import tf2_ros


UNKNOWN = -1
FREE_THRESHOLD = 50      # occupancy value below this counts as "free"


class AutonomousMapper(Node):

    def __init__(self):
        super().__init__("autonomous_mapper")

        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("min_frontier_size", 4)      # min contiguous frontier cells
        self.declare_parameter("replan_period_sec", 2.0)    # how often to look for a new frontier
        self.declare_parameter("robot_base_frame", "base_link")
        self.declare_parameter("global_frame", "map")

        self.map_topic = self.get_parameter("map_topic").value
        self.min_frontier_size = self.get_parameter("min_frontier_size").value
        self.replan_period_sec = self.get_parameter("replan_period_sec").value
        self.robot_base_frame = self.get_parameter("robot_base_frame").value
        self.global_frame = self.get_parameter("global_frame").value

        map_qos = QoSProfile(depth=1)
        map_qos.reliability = QoSReliabilityPolicy.RELIABLE
        map_qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL

        self.declare_parameter("blacklist_radius", 0.4)     # m — how close a
                                                              # new candidate must
                                                              # be to a past
                                                              # failure to be
                                                              # rejected
        self.declare_parameter("connectivity_check_radius", 0.15)
        self.declare_parameter("min_obstacle_clearance", 0.40)  # minimum goal-center clearance from occupied cells  # m — how
                                                              # far to walk
                                                              # back along the
                                                              # straight line to
                                                              # the frontier
                                                              # checking for a
                                                              # continuous free
                                                              # path (crude BFS
                                                              # connectivity
                                                              # substitute)

        self.blacklist_radius = self.get_parameter("blacklist_radius").value
        self.connectivity_check_radius = self.get_parameter(
            "connectivity_check_radius"
        ).value
        self.min_obstacle_clearance = self.get_parameter("min_obstacle_clearance").value

        self.latest_map = None
        self.goal_active = False
        self.exploration_done = False
        # Frontiers Nav2 has already failed to reach — list of (x, y)
        # world coords. A new candidate within blacklist_radius of any
        # of these is skipped. This directly fixes the bug where the
        # same unreachable frontier (an isolated pocket of "free" cells
        # surrounded by unknown space, not actually connected to the
        # robot by a real corridor) kept getting re-selected forever,
        # since nothing previously remembered that it had already failed.
        self.failed_goals = []

        self.map_sub = self.create_subscription(
            OccupancyGrid, self.map_topic, self.map_callback, map_qos
        )

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")

        self.timer = self.create_timer(self.replan_period_sec, self.explore_step)

        self.get_logger().info(
            f"autonomous_mapper started. map_topic={self.map_topic}, "
            f"replanning every {self.replan_period_sec}s"
        )

    # ------------------------------------------------------------------
    def map_callback(self, msg: OccupancyGrid):
        self.latest_map = msg

    def get_robot_pose(self):
        """Return (x, y) of the robot in the global (map) frame, or None."""
        try:
            tf = self.tf_buffer.lookup_transform(
                self.global_frame, self.robot_base_frame, rclpy.time.Time()
            )
            return tf.transform.translation.x, tf.transform.translation.y
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(f"TF lookup failed: {e}", throttle_duration_sec=5.0)
            return None

    # ------------------------------------------------------------------
    def find_frontiers(self, grid: OccupancyGrid):
        """
        Return a list of (x, y) world-coordinates of frontier cell
        clusters (free cell touching an unknown cell), one point per
        cluster centroid, filtered by min_frontier_size.
        """
        width = grid.info.width
        height = grid.info.height
        res = grid.info.resolution
        ox = grid.info.origin.position.x
        oy = grid.info.origin.position.y
        data = grid.data

        def idx(x, y):
            return y * width + x

        def is_free(x, y):
            return 0 <= x < width and 0 <= y < height and 0 <= data[idx(x, y)] < FREE_THRESHOLD

        def is_unknown(x, y):
            return 0 <= x < width and 0 <= y < height and data[idx(x, y)] == UNKNOWN

        frontier_cells = set()
        for y in range(height):
            for x in range(width):
                if not is_free(x, y):
                    continue
                neighbours = [
                    (x + 1, y), (x - 1, y), (x, y + 1), (x, y - 1),
                ]
                if any(is_unknown(nx, ny) for nx, ny in neighbours):
                    frontier_cells.add((x, y))

        if not frontier_cells:
            return []

        # cluster via simple BFS flood-fill over 8-connected frontier cells
        visited = set()
        clusters = []
        for cell in frontier_cells:
            if cell in visited:
                continue
            stack = [cell]
            cluster = []
            visited.add(cell)
            while stack:
                cx, cy = stack.pop()
                cluster.append((cx, cy))
                for dx in (-1, 0, 1):
                    for dy in (-1, 0, 1):
                        if dx == 0 and dy == 0:
                            continue
                        n = (cx + dx, cy + dy)
                        if n in frontier_cells and n not in visited:
                            visited.add(n)
                            stack.append(n)
            clusters.append(cluster)

        centroids = []
        for cluster in clusters:
            if len(cluster) < self.min_frontier_size:
                continue
            cx = sum(c[0] for c in cluster) / len(cluster)
            cy = sum(c[1] for c in cluster) / len(cluster)
            world_x = ox + (cx + 0.5) * res
            world_y = oy + (cy + 0.5) * res
            centroids.append((world_x, world_y))

        return centroids

    def is_blacklisted(self, candidate):
        """True if candidate is within blacklist_radius of a frontier
        Nav2 has already failed to reach. This is what stops the
        exact same unreachable pocket from being re-selected forever."""
        cx, cy = candidate
        for fx, fy in self.failed_goals:
            if math.hypot(cx - fx, cy - fy) < self.blacklist_radius:
                return True
        return False

    def is_connected_by_free_space(self, grid: OccupancyGrid, robot_pos, candidate):
        """
        Real reachability check via BFS over free cells, from the
        robot's current grid cell to the candidate frontier's grid
        cell. This is the actual fix for "path generation does not
        account for bot's validity to reach the point" — the old
        code only checked straight-line distance, which happily
        targets isolated free-cell islands that are surrounded by
        unknown/unexplored space and have no real corridor connecting
        them to the robot at all (exactly the case in the screenshot).

        Returns True only if there exists a continuous chain of
        FREE cells (4-connected) from the robot's cell to the
        candidate's cell. This mirrors what the actual path planner
        needs — a connected free-space route — far more closely than
        raw Euclidean distance ever could.
        """
        width = grid.info.width
        height = grid.info.height
        res = grid.info.resolution
        ox = grid.info.origin.position.x
        oy = grid.info.origin.position.y
        data = grid.data

        def world_to_grid(wx, wy):
            gx = int((wx - ox) / res)
            gy = int((wy - oy) / res)
            return gx, gy

        def idx(x, y):
            return y * width + x

        def is_free(x, y):
            return (0 <= x < width and 0 <= y < height
                    and 0 <= data[idx(x, y)] < FREE_THRESHOLD)

        start = world_to_grid(*robot_pos)
        goal = world_to_grid(*candidate)

        if not is_free(*start) or not is_free(*goal):
            return False

        # BFS — capped iteration count so this stays cheap even on
        # larger maps; if we haven't reached the goal within this
        # many expansions, treat it as unreachable (a real reachable
        # frontier in a bounded maze will connect well within this).
        max_expansions = 20000
        visited = {start}
        queue = [start]
        expansions = 0

        while queue and expansions < max_expansions:
            cx, cy = queue.pop(0)
            if (cx, cy) == goal:
                return True
            for nx, ny in ((cx+1, cy), (cx-1, cy), (cx, cy+1), (cx, cy-1)):
                if (nx, ny) not in visited and is_free(nx, ny):
                    visited.add((nx, ny))
                    queue.append((nx, ny))
            expansions += 1

        return False

    def has_obstacle_clearance(self, grid: OccupancyGrid, candidate):
        """Reject frontier goals too close to mapped obstacles.

        The frontier itself is often a free cell immediately beside an
        occupied wall. Sending that cell as a NavigateToPose goal can make
        the robot approach the obstacle unnecessarily closely even though
        Nav2 has a valid path to it.
        """
        width = grid.info.width
        height = grid.info.height
        res = grid.info.resolution
        ox = grid.info.origin.position.x
        oy = grid.info.origin.position.y
        data = grid.data

        gx = int((candidate[0] - ox) / res)
        gy = int((candidate[1] - oy) / res)
        radius_cells = max(1, int(math.ceil(self.min_obstacle_clearance / res)))

        # Check a square neighborhood. Conservative is intentional here:
        # a goal is only useful if the robot can stand there with clearance.
        for dy in range(-radius_cells, radius_cells + 1):
            for dx in range(-radius_cells, radius_cells + 1):
                if dx * dx + dy * dy > radius_cells * radius_cells:
                    continue
                x, y = gx + dx, gy + dy
                if not (0 <= x < width and 0 <= y < height):
                    continue
                value = data[y * width + x]
                if value >= 65:
                    return False
        return True

    def select_frontier(self, frontiers, robot_pos, grid: OccupancyGrid):
        """
        Pick the nearest frontier that is BOTH:
          1. not blacklisted (Nav2 hasn't already failed to reach it), and
          2. actually connected to the robot via a continuous chain of
             free cells (not just close in raw x,y distance).

        Frontiers are checked nearest-first so we still prefer close
        goals when they're genuinely reachable. If nothing passes both
        checks, returns None so explore_step can decide what to do
        next instead of sending a doomed goal.
        """
        rx, ry = robot_pos
        candidates = sorted(
            frontiers, key=lambda f: math.hypot(f[0] - rx, f[1] - ry)
        )

        for candidate in candidates:
            if self.is_blacklisted(candidate):
                continue
            if not self.has_obstacle_clearance(grid, candidate):
                continue
            if not self.is_connected_by_free_space(grid, robot_pos, candidate):
                continue
            return candidate

        return None

    # ------------------------------------------------------------------
    def explore_step(self):
        if self.exploration_done or self.goal_active:
            return

        if self.latest_map is None:
            self.get_logger().info("Waiting for /map...", throttle_duration_sec=5.0)
            return

        robot_pos = self.get_robot_pose()
        if robot_pos is None:
            return

        frontiers = self.find_frontiers(self.latest_map)
        if not frontiers:
            self.get_logger().info("No frontiers left — exploration complete.")
            self.exploration_done = True
            return

        target = self.select_frontier(frontiers, robot_pos, self.latest_map)
        if target is None:
            # Every candidate frontier was either blacklisted (already
            # failed) or not actually connected to the robot by free
            # space (an unreachable pocket, like the isolated region
            # in the screenshot). Don't send a doomed goal — wait for
            # the map to grow on the next scan and try again next tick.
            self.get_logger().info(
                "No reachable, non-blacklisted frontier available this "
                "tick — waiting for more map data.",
                throttle_duration_sec=5.0,
            )
            return

        self.current_goal = target
        self.send_goal(target)

    def send_goal(self, target_xy):
        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn("navigate_to_pose action server not available yet.")
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = self.global_frame
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = target_xy[0]
        goal_msg.pose.pose.position.y = target_xy[1]
        # Face approximately toward the frontier instead of forcing yaw=0.
        # This avoids an unnecessary in-place rotation at every exploration goal.
        robot_xy = self.get_robot_pose()
        if robot_xy is not None:
            yaw = math.atan2(target_xy[1] - robot_xy[1],
                             target_xy[0] - robot_xy[0])
            goal_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
            goal_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        else:
            goal_msg.pose.pose.orientation.w = 1.0

        self.get_logger().info(f"Sending exploration goal: ({target_xy[0]:.2f}, {target_xy[1]:.2f})")
        self.goal_active = True

        future = self.nav_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Exploration goal was rejected.")
            self.goal_active = False
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future):
        result = future.result()
        # status codes from action_msgs/msg/GoalStatus:
        # 4 = SUCCEEDED, 5 = CANCELED, 6 = ABORTED
        status = result.status if result is not None else None

        if status is not None and status != 4 and self.current_goal is not None:
            # The goal did NOT succeed (aborted, e.g. "Failed to make
            # progress", planner failure, etc.) — blacklist it so we
            # never re-select this same unreachable spot again. This
            # is the fix for the frontier-repeat bug: previously this
            # callback threw away all failure information and the
            # exact same doomed frontier could be immediately
            # re-selected on the very next tick.
            self.failed_goals.append(self.current_goal)
            self.get_logger().warn(
                f"Goal {self.current_goal} did not succeed (status={status}) "
                f"— blacklisting it. Total blacklisted: {len(self.failed_goals)}"
            )

        self.current_goal = None
        self.goal_active = False


def main(args=None):
    rclpy.init(args=args)
    node = AutonomousMapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()