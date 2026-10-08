#!/usr/bin/env python3
"""
Drive simple kinematic obstacle models through Gazebo Sim.

The obstacles are real SDF models with collision + visual geometry. Their
poses are updated through Gazebo Sim's SetEntityPose service, so they appear
in /scan as moving objects without adding any sensor-side information to the
navigation stack.

This is deliberately kept separate from the robot controller: it is a
simulation-only world manager.
"""

import math
from dataclasses import dataclass

import rclpy
from geometry_msgs.msg import Pose
from rclpy.node import Node
from ros_gz_interfaces.srv import SetEntityPose


@dataclass
class MovingObstacle:
    name: str
    x: float
    y: float
    z: float
    vx: float
    vy: float
    min_x: float | None = None
    max_x: float | None = None
    min_y: float | None = None
    max_y: float | None = None

    def step(self, dt: float) -> float:
        """Advance obstacle and return its yaw in direction of travel."""
        self.x += self.vx * dt
        self.y += self.vy * dt

        if self.min_x is not None and self.x < self.min_x:
            self.x = self.min_x
            self.vx = abs(self.vx)
        elif self.max_x is not None and self.x > self.max_x:
            self.x = self.max_x
            self.vx = -abs(self.vx)

        if self.min_y is not None and self.y < self.min_y:
            self.y = self.min_y
            self.vy = abs(self.vy)
        elif self.max_y is not None and self.y > self.max_y:
            self.y = self.max_y
            self.vy = -abs(self.vy)

        return math.atan2(self.vy, self.vx) if math.hypot(self.vx, self.vy) > 1e-6 else 0.0


class DynamicObstacleManager(Node):
    def __init__(self) -> None:
        super().__init__("dynamic_obstacle_manager")

        self.declare_parameter("update_rate", 10.0)
        self.declare_parameter("world_name", "ms616_default")

        update_rate = float(self.get_parameter("update_rate").value)
        world_name = str(self.get_parameter("world_name").value)
        service_name = f"/world/{world_name}/set_pose"

        # Two obstacles are deliberately placed in open parts of the current
        # maze so that they repeatedly cross likely Nav2 paths.
        self.obstacles = [
            MovingObstacle(
                name="dynamic_crossing_box",
                x=-1.7,
                y=0.55,
                z=0.125,
                vx=0.32,
                vy=0.0,
                min_x=-1.75,
                max_x=1.75,
            ),
            MovingObstacle(
                name="dynamic_vertical_box",
                x=0.85,
                y=-1.85,
                z=0.125,
                vx=0.0,
                vy=0.24,
                min_y=-1.85,
                max_y=1.35,
            ),
        ]

        self.pose_clients = {
            obstacle.name: self.create_client(SetEntityPose, service_name)
            for obstacle in self.obstacles
        }
        self.in_flight = {obstacle.name: False for obstacle in self.obstacles}

        self.last_time = self.get_clock().now()
        self.have_started = False
        self.timer = self.create_timer(max(0.02, 1.0 / update_rate), self.update)

        self.get_logger().info(
            f"Dynamic obstacle manager started: {len(self.obstacles)} obstacles, "
            f"Gazebo service={service_name}"
        )

    def make_pose(self, obstacle: MovingObstacle, yaw: float) -> Pose:
        pose = Pose()
        pose.position.x = obstacle.x
        pose.position.y = obstacle.y
        pose.position.z = obstacle.z
        pose.orientation.z = math.sin(yaw / 2.0)
        pose.orientation.w = math.cos(yaw / 2.0)
        return pose

    def send_pose(self, obstacle: MovingObstacle, yaw: float) -> None:
        client = self.pose_clients[obstacle.name]
        if not client.service_is_ready() or self.in_flight[obstacle.name]:
            return

        request = SetEntityPose.Request()
        request.entity.name = obstacle.name
        request.entity.type = 2  # MODEL
        request.pose = self.make_pose(obstacle, yaw)

        self.in_flight[obstacle.name] = True
        future = client.call_async(request)
        future.add_done_callback(lambda f, name=obstacle.name: self.pose_done(name, f))

    def pose_done(self, name: str, future) -> None:
        self.in_flight[name] = False
        try:
            result = future.result()
            if result is not None and not result.success:
                self.get_logger().warn(
                    f"Gazebo rejected pose update for {name}",
                    throttle_duration_sec=3.0,
                )
        except Exception as exc:  # pragma: no cover - runtime/service failure path
            self.get_logger().warn(
                f"Pose update failed for {name}: {exc}",
                throttle_duration_sec=3.0,
            )

    def update(self) -> None:
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now

        # Ignore the first tick and any backwards jump caused by a Gazebo reset.
        if not self.have_started:
            self.have_started = True
            return
        if dt <= 0.0 or dt > 1.0:
            return

        for obstacle in self.obstacles:
            yaw = obstacle.step(dt)
            self.send_pose(obstacle, yaw)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DynamicObstacleManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
