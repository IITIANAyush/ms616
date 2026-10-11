#!/usr/bin/env python3
"""Small, ROS-independent differential-drive kinematics helpers."""
from dataclasses import dataclass
import math


@dataclass
class Pose2D:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0  # deliberately unwrapped for accumulated rotation error


def wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def integrate_unicycle(pose: Pose2D, v: float, omega: float, dt: float) -> Pose2D:
    """Integrate body linear/angular velocity using exact constant-twist arc."""
    if dt <= 0.0:
        return pose
    yaw0 = pose.yaw
    if abs(omega) < 1e-8:
        pose.x += v * math.cos(yaw0) * dt
        pose.y += v * math.sin(yaw0) * dt
    else:
        yaw1 = yaw0 + omega * dt
        pose.x += (v / omega) * (math.sin(yaw1) - math.sin(yaw0))
        pose.y -= (v / omega) * (math.cos(yaw1) - math.cos(yaw0))
    pose.yaw += omega * dt
    return pose


def wheel_rates_to_body(left_rad_s: float, right_rad_s: float,
                        wheel_radius: float, wheel_separation: float):
    """Differential-drive forward kinematics (linear m/s, yaw rate rad/s)."""
    v = 0.5 * wheel_radius * (right_rad_s + left_rad_s)
    omega = (wheel_radius / wheel_separation) * (right_rad_s - left_rad_s)
    return v, omega


def relative_pose(x: float, y: float, yaw_unwrapped: float,
                  x0: float, y0: float, yaw0_unwrapped: float):
    """Return current pose expressed in the test's start frame."""
    dx, dy = x - x0, y - y0
    c, s = math.cos(yaw0_unwrapped), math.sin(yaw0_unwrapped)
    return c * dx + s * dy, -s * dx + c * dy, yaw_unwrapped - yaw0_unwrapped
