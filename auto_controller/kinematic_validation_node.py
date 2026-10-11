#!/usr/bin/env python3
"""Automated differential-drive model validation against Gazebo ground truth.

Runs straight, constant-curvature and in-place rotation trials. Logs:
  * requested body command integrated through ideal differential-drive kinematics
  * measured left/right wheel rates integrated through the same kinematics
  * wheel odometry from diff_drive_controller
  * Gazebo world-pose ground truth (PoseArray bridged from Ignition)

The difference between wheel-rate-model and Gazebo ground truth helps reveal
slip / wheel geometry / contact errors. Command-model versus truth additionally
includes controller response and actuator lag. This node directly publishes to
the diff_drive_controller command topic, so do NOT run Nav2/teleop at the same
time. Start Gazebo alone with dynamic obstacles disabled first.
"""
import csv
import json
import math
import os
from datetime import datetime

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseArray

from kinematic_model import Pose2D, integrate_unicycle, relative_pose, wheel_rates_to_body, wrap_angle


TESTS = [
    {"name": "straight_line", "v": 0.16, "w": 0.0,
     "ramp": 0.60, "hold": 2.8, "settle": 1.2},
    {"name": "constant_curvature_left", "v": 0.13, "w": 0.40,
     "ramp": 0.60, "hold": 3.8, "settle": 1.2},
    {"name": "in_place_rotation", "v": 0.0, "w": 0.55,
     "ramp": 0.50, "hold": 3.8, "settle": 1.5},
]
PRE_SETTLE = 0.8
TIMER_PERIOD = 0.02  # 50 Hz; wheel-state and controller update at ~50 Hz


def quaternion_yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def rmse(values):
    if not values:
        return float("nan")
    return math.sqrt(sum(v * v for v in values) / len(values))


class KinematicValidationNode(Node):
    def __init__(self):
        super().__init__("kinematic_validation_node")
        self.declare_parameter("use_sim_time", True)
        self.declare_parameter("command_topic", "/diff_drive_controller/cmd_vel_unstamped")
        self.declare_parameter("odom_topic", "/diff_drive_controller/odom")
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("ground_truth_topic", "/world/ms616_default/dynamic_pose/info")
        self.declare_parameter("left_joint_name", "left_wheel_joint")
        self.declare_parameter("right_joint_name", "right_wheel_joint")
        # MUST match URDF and diff_drive_controller.yaml for the current Gazebo model.
        self.declare_parameter("wheel_radius", 0.035)
        self.declare_parameter("wheel_separation", 0.21)
        # gazebo.launch.py spawns the robot at this world pose. Used to identify its
        # pose in PoseArray, which does not retain Gazebo entity names.
        self.declare_parameter("spawn_x", -0.2)
        self.declare_parameter("spawn_y", 0.2)
        self.declare_parameter("max_ground_truth_jump_m", 0.35)
        self.declare_parameter("output_dir", os.path.expanduser("~/ros2_ws/logs"))

        gp = lambda name: self.get_parameter(name).value
        self.command_topic = gp("command_topic")
        self.odom_topic = gp("odom_topic")
        self.joint_states_topic = gp("joint_states_topic")
        self.ground_truth_topic = gp("ground_truth_topic")
        self.left_joint_name = gp("left_joint_name")
        self.right_joint_name = gp("right_joint_name")
        self.wheel_radius = float(gp("wheel_radius"))
        self.wheel_separation = float(gp("wheel_separation"))
        self.spawn_x, self.spawn_y = float(gp("spawn_x")), float(gp("spawn_y"))
        self.max_gt_jump = float(gp("max_ground_truth_jump_m"))
        self.output_dir = os.path.expanduser(str(gp("output_dir")))
        os.makedirs(self.output_dir, exist_ok=True)

        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.csv_path = os.path.join(self.output_dir, f"kinematic_validation_{stamp}.csv")
        self.summary_path = os.path.join(self.output_dir, f"kinematic_validation_{stamp}_summary.json")
        self.csv_file = open(self.csv_path, "w", newline="", encoding="utf-8")
        self.csv_writer = csv.DictWriter(self.csv_file, fieldnames=[
            "test", "sim_time", "case_time", "phase", "motion_active",
            "cmd_v_mps", "cmd_w_radps", "odom_v_mps", "odom_w_radps",
            "left_wheel_radps", "right_wheel_radps",
            "wheel_kinematic_v_mps", "wheel_kinematic_w_radps",
            "gt_x_m", "gt_y_m", "gt_yaw_rad",
            "odom_x_m", "odom_y_m", "odom_yaw_rad",
            "cmd_model_x_m", "cmd_model_y_m", "cmd_model_yaw_rad",
            "wheel_model_x_m", "wheel_model_y_m", "wheel_model_yaw_rad",
            "cmd_pos_error_m", "cmd_yaw_error_rad",
            "wheel_model_pos_error_m", "wheel_model_yaw_error_rad",
            "odom_pos_error_m", "odom_yaw_error_rad",
            "gt_candidate_count", "gt_candidate_index",
        ])
        self.csv_writer.writeheader()

        self.cmd_pub = self.create_publisher(Twist, self.command_topic, 10)
        self.odom_sub = self.create_subscription(Odometry, self.odom_topic, self._odom_cb, 20)
        self.joint_sub = self.create_subscription(JointState, self.joint_states_topic, self._joint_cb, 20)
        self.gt_sub = self.create_subscription(PoseArray, self.ground_truth_topic, self._gt_cb, 10)

        self.odom = None
        self.odom_yaw_unwrapped = None
        self.odom_yaw_last = None
        self.joint_velocities = {}
        self.gt_pose = None
        self.gt_yaw_unwrapped = None
        self.gt_yaw_last = None
        self.gt_track_x = None
        self.gt_track_y = None
        self.gt_candidate_count = 0
        self.gt_candidate_index = -1
        self._warned_gt = False

        self.test_index = 0
        self.test_start_time = None
        self.test_origin = None
        self.cmd_model = Pose2D()
        self.wheel_model = Pose2D()
        self.last_tick_time = None
        self.last_gt_log_xy = None
        self.case_rows = []
        self.all_summaries = []
        self.finished = False
        self.latest_cmd_v = 0.0
        self.latest_cmd_w = 0.0

        self.timer = self.create_timer(TIMER_PERIOD, self._tick)
        self.get_logger().info("Kinematic validation initialized. Waiting for odom, joint states, and Gazebo ground truth.")
        self.get_logger().info(f"CSV: {self.csv_path}")
        self.get_logger().warn("Safety: run Gazebo only (no Nav2, frontier explorer, VFH controller, or teleop). This node directly commands the drive controller.")
        self.get_logger().info(f"Kinematic model uses r={self.wheel_radius:.3f} m, separation={self.wheel_separation:.3f} m (from the current Gazebo controller config).")

    def _odom_cb(self, msg):
        p = msg.pose.pose.position
        yaw = quaternion_yaw(msg.pose.pose.orientation)
        if self.odom_yaw_last is None:
            self.odom_yaw_unwrapped = yaw
        else:
            self.odom_yaw_unwrapped += wrap_angle(yaw - self.odom_yaw_last)
        self.odom_yaw_last = yaw
        self.odom = {
            "x": p.x, "y": p.y, "yaw": yaw,
            "yaw_unwrapped": self.odom_yaw_unwrapped,
            "v": msg.twist.twist.linear.x,
            "w": msg.twist.twist.angular.z,
        }

    def _joint_cb(self, msg):
        for i, name in enumerate(msg.name):
            if i < len(msg.velocity):
                self.joint_velocities[name] = float(msg.velocity[i])

    def _gt_cb(self, msg):
        if not msg.poses:
            return
        self.gt_candidate_count = len(msg.poses)
        candidates = []
        for i, p in enumerate(msg.poses):
            candidates.append((i, p.position.x, p.position.y, quaternion_yaw(p.orientation)))

        if self.gt_track_x is None:
            # The robot is spawned at (-0.2, 0.2) in gazebo.launch.py.
            idx, x, y, yaw = min(candidates,
                                 key=lambda c: (c[1] - self.spawn_x) ** 2 + (c[2] - self.spawn_y) ** 2)
            dist = math.hypot(x - self.spawn_x, y - self.spawn_y)
            if dist > 0.8 and not self._warned_gt:
                self.get_logger().warn(
                    f"Closest Gazebo dynamic pose is {dist:.2f} m from configured spawn. "
                    "Check /world/ms616_default/dynamic_pose/info or set spawn_x/spawn_y."
                )
                self._warned_gt = True
            self.gt_yaw_unwrapped = yaw
        else:
            # Follow the spatially continuous pose. Use static mapping world with
            # dynamic_obstacles:=false so another moving model cannot steal tracking.
            idx, x, y, yaw = min(candidates,
                                 key=lambda c: (c[1] - self.gt_track_x) ** 2 + (c[2] - self.gt_track_y) ** 2)
            jump = math.hypot(x - self.gt_track_x, y - self.gt_track_y)
            if jump > self.max_gt_jump:
                if not self._warned_gt:
                    self.get_logger().warn(
                        f"Rejecting a {jump:.2f} m jump in the selected Gazebo pose. "
                        "The bridge publishes unnamed PoseArray entries; check the selected candidate/topic. "
                        "Increase max_ground_truth_jump_m only if the simulator is genuinely publishing much slower."
                    )
                    self._warned_gt = True
                return
            if self.gt_yaw_last is not None:
                self.gt_yaw_unwrapped += wrap_angle(yaw - self.gt_yaw_last)
            else:
                self.gt_yaw_unwrapped = yaw
        self.gt_track_x, self.gt_track_y = x, y
        self.gt_yaw_last = yaw
        self.gt_candidate_index = idx
        self.gt_pose = {"x": x, "y": y, "yaw": yaw, "yaw_unwrapped": self.gt_yaw_unwrapped}

    def _ready(self):
        wheels_ready = self.left_joint_name in self.joint_velocities and self.right_joint_name in self.joint_velocities
        return self.odom is not None and wheels_ready and self.gt_pose is not None

    def _now_sec(self):
        return self.get_clock().now().nanoseconds / 1e9

    @staticmethod
    def _phase_command(case, elapsed):
        if elapsed < PRE_SETTLE:
            return 0.0, 0.0, "pre_settle", False
        t = elapsed - PRE_SETTLE
        ramp, hold = case["ramp"], case["hold"]
        v_target, w_target = case["v"], case["w"]
        if t < ramp:
            f = t / ramp if ramp > 0 else 1.0
            return v_target * f, w_target * f, "ramp_up", True
        t -= ramp
        if t < hold:
            return v_target, w_target, "hold", True
        t -= hold
        if t < ramp:
            f = max(0.0, 1.0 - t / ramp) if ramp > 0 else 0.0
            return v_target * f, w_target * f, "ramp_down", True
        return 0.0, 0.0, "post_settle", False

    def _begin_test(self, index, now):
        self.test_index = index
        self.test_start_time = now
        self.test_origin = {
            "gt_x": self.gt_pose["x"], "gt_y": self.gt_pose["y"],
            "gt_yaw": self.gt_pose["yaw_unwrapped"],
            "odom_x": self.odom["x"], "odom_y": self.odom["y"],
            "odom_yaw": self.odom["yaw_unwrapped"],
        }
        self.cmd_model = Pose2D()
        self.wheel_model = Pose2D()
        self.case_rows = []
        self.last_gt_log_xy = None
        self.get_logger().info(
            f"Starting test {index + 1}/{len(TESTS)}: {TESTS[index]['name']} "
            f"(target v={TESTS[index]['v']:.2f} m/s, w={TESTS[index]['w']:.2f} rad/s)"
        )

    def _metrics(self, case, rows):
        active = [r for r in rows if r["motion_active"]]
        def vals(key): return [float(r[key]) for r in active if r[key] != ""]
        final = rows[-1] if rows else {}
        gt_path = 0.0
        last = None
        for r in rows:
            xy = (float(r["gt_x_m"]), float(r["gt_y_m"]))
            if last is not None:
                gt_path += math.hypot(xy[0] - last[0], xy[1] - last[1])
            last = xy
        result = {
            "test": case["name"], "samples_total": len(rows), "samples_motion_active": len(active),
            "gazebo_ground_truth_path_length_m": gt_path,
            "command_model_position_rmse_m": rmse(vals("cmd_pos_error_m")),
            "wheel_rate_model_position_rmse_m": rmse(vals("wheel_model_pos_error_m")),
            "wheel_odometry_position_rmse_m": rmse(vals("odom_pos_error_m")),
            "command_model_yaw_rmse_deg": math.degrees(rmse(vals("cmd_yaw_error_rad"))),
            "wheel_rate_model_yaw_rmse_deg": math.degrees(rmse(vals("wheel_model_yaw_error_rad"))),
            "wheel_odometry_yaw_rmse_deg": math.degrees(rmse(vals("odom_yaw_error_rad"))),
            "final_command_model_position_error_m": float(final.get("cmd_pos_error_m", "nan")),
            "final_wheel_rate_model_position_error_m": float(final.get("wheel_model_pos_error_m", "nan")),
            "final_wheel_odometry_position_error_m": float(final.get("odom_pos_error_m", "nan")),
            "final_command_model_yaw_error_deg": math.degrees(float(final.get("cmd_yaw_error_rad", "nan"))),
            "final_wheel_rate_model_yaw_error_deg": math.degrees(float(final.get("wheel_model_yaw_error_rad", "nan"))),
            "final_wheel_odometry_yaw_error_deg": math.degrees(float(final.get("odom_yaw_error_rad", "nan"))),
        }
        return result

    def _finish_test(self, now):
        case = TESTS[self.test_index]
        result = self._metrics(case, self.case_rows)
        self.all_summaries.append(result)
        self.get_logger().info(
            f"RESULT {case['name']}: pos RMSE (cmd/wheel/odom vs Gazebo) = "
            f"{result['command_model_position_rmse_m']:.3f} / "
            f"{result['wheel_rate_model_position_rmse_m']:.3f} / "
            f"{result['wheel_odometry_position_rmse_m']:.3f} m; "
            f"yaw RMSE = {result['command_model_yaw_rmse_deg']:.1f} / "
            f"{result['wheel_rate_model_yaw_rmse_deg']:.1f} / "
            f"{result['wheel_odometry_yaw_rmse_deg']:.1f} deg"
        )
        if self.test_index + 1 < len(TESTS):
            self._begin_test(self.test_index + 1, now)
        else:
            self.finished = True
            self._publish_stop()
            self.csv_file.flush()
            with open(self.summary_path, "w", encoding="utf-8") as f:
                json.dump({
                    "wheel_radius_m": self.wheel_radius,
                    "wheel_separation_m": self.wheel_separation,
                    "ground_truth_topic": self.ground_truth_topic,
                    "odom_topic": self.odom_topic,
                    "joint_states_topic": self.joint_states_topic,
                    "tests": self.all_summaries,
                    "interpretation": {
                        "command_model_vs_ground_truth": "includes controller response, acceleration limits, and wheel/contact effects",
                        "wheel_rate_model_vs_ground_truth": "best indicator of kinematic geometry or physical slip, using measured joint rates",
                        "wheel_odometry_vs_ground_truth": "shows how far encoder-based odometry diverges from Gazebo world pose",
                    },
                }, f, indent=2)
            self.csv_file.close()
            self.get_logger().info(f"All trials completed. CSV: {self.csv_path}")
            self.get_logger().info(f"Summary JSON: {self.summary_path}")
            self.get_logger().info("Robot commands are held at zero. Press Ctrl+C to close the validation node.")

    def _publish_stop(self):
        self.cmd_pub.publish(Twist())
        self.latest_cmd_v = 0.0
        self.latest_cmd_w = 0.0

    def _tick(self):
        now = self._now_sec()
        if self.finished:
            self._publish_stop()
            return
        if not self._ready():
            self._publish_stop()
            self.last_tick_time = now
            return
        if self.test_start_time is None:
            self._begin_test(0, now)
            self.last_tick_time = now
            self._publish_stop()
            return

        dt = 0.0 if self.last_tick_time is None else max(0.0, min(0.10, now - self.last_tick_time))
        self.last_tick_time = now
        case = TESTS[self.test_index]
        elapsed = now - self.test_start_time
        v_cmd, w_cmd, phase, motion_active = self._phase_command(case, elapsed)
        msg = Twist()
        msg.linear.x = float(v_cmd)
        msg.angular.z = float(w_cmd)
        self.cmd_pub.publish(msg)
        self.latest_cmd_v, self.latest_cmd_w = v_cmd, w_cmd

        # Relative Gazebo ground-truth pose in this test's start frame.
        gt_x, gt_y, gt_yaw = relative_pose(
            self.gt_pose["x"], self.gt_pose["y"], self.gt_pose["yaw_unwrapped"],
            self.test_origin["gt_x"], self.test_origin["gt_y"], self.test_origin["gt_yaw"])
        odom_x, odom_y, odom_yaw = relative_pose(
            self.odom["x"], self.odom["y"], self.odom["yaw_unwrapped"],
            self.test_origin["odom_x"], self.test_origin["odom_y"], self.test_origin["odom_yaw"])

        wl = self.joint_velocities[self.left_joint_name]
        wr = self.joint_velocities[self.right_joint_name]
        v_wheel, w_wheel = wheel_rates_to_body(
            wl, wr, self.wheel_radius, self.wheel_separation)
        if dt > 0.0:
            integrate_unicycle(self.cmd_model, v_cmd, w_cmd, dt)
            integrate_unicycle(self.wheel_model, v_wheel, w_wheel, dt)

        cmd_pos_err = math.hypot(self.cmd_model.x - gt_x, self.cmd_model.y - gt_y)
        wheel_pos_err = math.hypot(self.wheel_model.x - gt_x, self.wheel_model.y - gt_y)
        odom_pos_err = math.hypot(odom_x - gt_x, odom_y - gt_y)
        row = {
            "test": case["name"], "sim_time": f"{now:.4f}", "case_time": f"{elapsed:.4f}",
            "phase": phase, "motion_active": int(motion_active),
            "cmd_v_mps": f"{v_cmd:.6f}", "cmd_w_radps": f"{w_cmd:.6f}",
            "odom_v_mps": f"{self.odom['v']:.6f}", "odom_w_radps": f"{self.odom['w']:.6f}",
            "left_wheel_radps": f"{wl:.6f}",
            "right_wheel_radps": f"{wr:.6f}",
            "wheel_kinematic_v_mps": f"{v_wheel:.6f}",
            "wheel_kinematic_w_radps": f"{w_wheel:.6f}",
            "gt_x_m": f"{gt_x:.6f}", "gt_y_m": f"{gt_y:.6f}", "gt_yaw_rad": f"{gt_yaw:.6f}",
            "odom_x_m": f"{odom_x:.6f}", "odom_y_m": f"{odom_y:.6f}", "odom_yaw_rad": f"{odom_yaw:.6f}",
            "cmd_model_x_m": f"{self.cmd_model.x:.6f}", "cmd_model_y_m": f"{self.cmd_model.y:.6f}", "cmd_model_yaw_rad": f"{self.cmd_model.yaw:.6f}",
            "wheel_model_x_m": f"{self.wheel_model.x:.6f}", "wheel_model_y_m": f"{self.wheel_model.y:.6f}", "wheel_model_yaw_rad": f"{self.wheel_model.yaw:.6f}",
            "cmd_pos_error_m": f"{cmd_pos_err:.6f}", "cmd_yaw_error_rad": f"{self.cmd_model.yaw - gt_yaw:.6f}",
            "wheel_model_pos_error_m": f"{wheel_pos_err:.6f}", "wheel_model_yaw_error_rad": f"{self.wheel_model.yaw - gt_yaw:.6f}",
            "odom_pos_error_m": f"{odom_pos_err:.6f}", "odom_yaw_error_rad": f"{odom_yaw - gt_yaw:.6f}",
            "gt_candidate_count": self.gt_candidate_count, "gt_candidate_index": self.gt_candidate_index,
        }
        self.csv_writer.writerow(row)
        self.case_rows.append(row)
        if elapsed >= PRE_SETTLE + case["ramp"] + case["hold"] + case["ramp"] + case["settle"]:
            self._finish_test(now)

    def destroy_node(self):
        try:
            self._publish_stop()
            if not self.csv_file.closed:
                self.csv_file.flush()
                self.csv_file.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = KinematicValidationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
