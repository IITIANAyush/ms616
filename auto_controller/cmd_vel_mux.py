#!/usr/bin/env python3
"""Priority velocity mux for Nav2 autonomy and keyboard manual override.

Automatic input: /nav_cmd_vel
Manual input:    /manual_cmd_vel
Final output:    /diff_drive_controller/cmd_vel_unstamped

Enable override with:
  ros2 service call /cmd_vel_mux/set_manual_override std_srvs/srv/SetBool "{data: true}"
Disable with data:false. The mux publishes zero when the selected input times out.
This software override is not a certified hardware emergency stop.
"""
import copy
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from std_srvs.srv import SetBool
from action_msgs.srv import CancelGoal


class CmdVelMux(Node):
    def __init__(self):
        super().__init__("cmd_vel_mux")
        self.declare_parameter("nav_topic", "/nav_cmd_vel")
        self.declare_parameter("manual_topic", "/manual_cmd_vel")
        self.declare_parameter("output_topic", "/diff_drive_controller/cmd_vel_unstamped")
        self.declare_parameter("command_timeout", 0.40)
        self.declare_parameter("publish_rate", 30.0)
        self.declare_parameter("cancel_nav_goal_on_override", True)

        self.nav_topic = str(self.get_parameter("nav_topic").value)
        self.manual_topic = str(self.get_parameter("manual_topic").value)
        self.output_topic = str(self.get_parameter("output_topic").value)
        self.command_timeout = float(self.get_parameter("command_timeout").value)
        self.publish_rate = float(self.get_parameter("publish_rate").value)
        self.cancel_nav_goal_on_override = bool(self.get_parameter("cancel_nav_goal_on_override").value)

        self.manual_override = False
        self.last_nav_cmd = None
        self.last_nav_time = 0.0
        self.last_manual_cmd = None
        self.last_manual_time = 0.0

        self.cmd_pub = self.create_publisher(Twist, self.output_topic, 10)
        # Transient-local status makes the current override state available to
        # autonomous_mapper even if it starts slightly after this node.
        mode_qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.mode_pub = self.create_publisher(Bool, "/cmd_vel_mux/manual_override_active", mode_qos)
        self.create_subscription(Twist, self.nav_topic, self._nav_cb, 10)
        self.create_subscription(Twist, self.manual_topic, self._manual_cb, 10)
        self.create_service(SetBool, "/cmd_vel_mux/set_manual_override", self._set_override)
        self.cancel_client = self.create_client(
            CancelGoal, "/navigate_to_pose/_action/cancel_goal"
        )
        self.timer = self.create_timer(1.0 / max(1.0, self.publish_rate), self._publish_selected)
        self._publish_mode()
        self.get_logger().info(
            f"Velocity mux ready. AUTO={self.nav_topic}, MANUAL={self.manual_topic}, "
            f"OUTPUT={self.output_topic}. Manual override initially disabled."
        )

    def _nav_cb(self, msg: Twist):
        self.last_nav_cmd = copy.deepcopy(msg)
        self.last_nav_time = time.monotonic()

    def _manual_cb(self, msg: Twist):
        self.last_manual_cmd = copy.deepcopy(msg)
        self.last_manual_time = time.monotonic()

    @staticmethod
    def _zero_twist():
        return Twist()

    def _publish_mode(self):
        msg = Bool()
        msg.data = self.manual_override
        self.mode_pub.publish(msg)

    def _cancel_all_nav_goals(self):
        if not self.cancel_nav_goal_on_override:
            return
        if not self.cancel_client.service_is_ready():
            self.get_logger().warn(
                "Nav2 cancel service is not ready; velocity output is still locked to manual mode. "
                "Cancel any active goal in RViz before returning to AUTO."
            )
            return
        request = CancelGoal.Request()
        # Zero UUID + zero timestamp is the ROS 2 action protocol's cancel-all request.
        request.goal_info.goal_id.uuid = [0] * 16
        request.goal_info.stamp.sec = 0
        request.goal_info.stamp.nanosec = 0
        future = self.cancel_client.call_async(request)
        future.add_done_callback(self._cancel_response_cb)

    def _cancel_response_cb(self, future):
        try:
            response = future.result()
            self.get_logger().info(
                f"Requested cancellation of active NavigateToPose goals; code={response.return_code}."
            )
        except Exception as exc:  # keep the safety mux active even if cancellation fails
            self.get_logger().error(f"Nav2 goal-cancel request failed: {exc}")

    def _set_override(self, request: SetBool.Request, response: SetBool.Response):
        requested = bool(request.data)
        changed = requested != self.manual_override
        self.manual_override = requested
        if changed:
            if requested:
                # Never replay a stale command when switching control sources.
                self.last_manual_cmd = None
                self.last_manual_time = 0.0
                self.last_nav_cmd = None
                self.last_nav_time = 0.0
                self._publish_mode()
                self._cancel_all_nav_goals()
                self.get_logger().warn(
                    "MANUAL OVERRIDE ACTIVE: Nav2 commands are blocked. "
                    "No fresh keyboard command means zero velocity."
                )
            else:
                # Require a fresh autonomy command; the mux will stay stopped until one arrives.
                self.last_nav_cmd = None
                self.last_nav_time = 0.0
                self.last_manual_cmd = None
                self.last_manual_time = 0.0
                self._publish_mode()
                self.get_logger().info(
                    "Manual override disabled. Waiting for a fresh Nav2 command; send a new goal if needed."
                )
        response.success = True
        response.message = "Manual override enabled." if requested else "Manual override disabled."
        return response

    def _publish_selected(self):
        now = time.monotonic()
        if self.manual_override:
            if self.last_manual_cmd is not None and now - self.last_manual_time <= self.command_timeout:
                self.cmd_pub.publish(self.last_manual_cmd)
            else:
                self.cmd_pub.publish(self._zero_twist())
            return
        if self.last_nav_cmd is not None and now - self.last_nav_time <= self.command_timeout:
            self.cmd_pub.publish(self.last_nav_cmd)
        else:
            self.cmd_pub.publish(self._zero_twist())


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelMux()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop the robot if the mux itself is being shut down cleanly.
        try:
            node.cmd_pub.publish(Twist())
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
