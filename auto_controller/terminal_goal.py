#!/usr/bin/env python3
"""Send one map-frame Nav2 NavigateToPose goal from launch parameters.

Use `ros2 launch ms616 goal_from_terminal.launch.py x:=1.0 y:=2.0 yaw_deg:=90`.
This node is deliberately only a goal sender; Nav2 owns path planning/control.
"""

import math

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose


class TerminalGoal(Node):
    def __init__(self):
        super().__init__("terminal_goal")
        self.declare_parameter("x", 0.0)
        self.declare_parameter("y", 0.0)
        self.declare_parameter("yaw_deg", 0.0)
        self.declare_parameter("frame_id", "map")
        self._client = ActionClient(self, NavigateToPose, "/navigate_to_pose")

    def send_goal(self):
        x = float(self.get_parameter("x").value)
        y = float(self.get_parameter("y").value)
        yaw = math.radians(float(self.get_parameter("yaw_deg").value))
        frame = str(self.get_parameter("frame_id").value)

        self.get_logger().info("Waiting for Nav2 action server /navigate_to_pose ...")
        while rclpy.ok() and not self._client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info("Nav2 is not ready yet; waiting ...")
        if not rclpy.ok():
            return

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.position.z = 0.0
        goal.pose.pose.orientation.x = 0.0
        goal.pose.pose.orientation.y = 0.0
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)

        self.get_logger().info(
            f"Sending goal x={x:.2f} m, y={y:.2f} m, yaw={math.degrees(yaw):.1f} deg in {frame}"
        )
        send_future = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        handle = send_future.result()
        if handle is None or not handle.accepted:
            self.get_logger().error("Nav2 rejected the goal.")
            return
        self.get_logger().info("Goal accepted; Nav2 is planning and navigating.")
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result()
        if result is not None:
            self.get_logger().info(f"Navigation finished with status code {result.status}.")


def main(args=None):
    rclpy.init(args=args)
    node = TerminalGoal()
    try:
        node.send_goal()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
