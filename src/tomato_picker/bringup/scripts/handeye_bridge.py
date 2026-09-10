#!/usr/bin/env python3

"""Minimal SerialArm hand-eye calibration bridge.

This node is intentionally independent from ros2_control/MoveIt. It owns the
arm through SerialArm-Core RobotSession, switches the arm to compliant drag,
and republishes the Core cached tool pose as geometry_msgs/PoseStamped for the
Handeye-Calibration-App.
"""

import math
from typing import Optional, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node

from serial_arm import JointImpedanceMode, RobotSession, load_robot_profile_core


def transform_matrix_to_pose(transform: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """Convert a 4x4 homogeneous transform to position and xyzw quaternion."""
    matrix = np.asarray(transform, dtype=float)
    if matrix.shape != (4, 4):
        raise ValueError(f"tool_pose must have shape (4, 4), got {matrix.shape}")
    if not np.all(np.isfinite(matrix)):
        raise ValueError("tool_pose contains NaN or Inf")

    rotation = matrix[:3, :3]
    trace = float(np.trace(rotation))

    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * scale
        qx = (rotation[2, 1] - rotation[1, 2]) / scale
        qy = (rotation[0, 2] - rotation[2, 0]) / scale
        qz = (rotation[1, 0] - rotation[0, 1]) / scale
    elif rotation[0, 0] > rotation[1, 1] and rotation[0, 0] > rotation[2, 2]:
        scale = math.sqrt(1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2]) * 2.0
        qw = (rotation[2, 1] - rotation[1, 2]) / scale
        qx = 0.25 * scale
        qy = (rotation[0, 1] + rotation[1, 0]) / scale
        qz = (rotation[0, 2] + rotation[2, 0]) / scale
    elif rotation[1, 1] > rotation[2, 2]:
        scale = math.sqrt(1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2]) * 2.0
        qw = (rotation[0, 2] - rotation[2, 0]) / scale
        qx = (rotation[0, 1] + rotation[1, 0]) / scale
        qy = 0.25 * scale
        qz = (rotation[1, 2] + rotation[2, 1]) / scale
    else:
        scale = math.sqrt(1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1]) * 2.0
        qw = (rotation[1, 0] - rotation[0, 1]) / scale
        qx = (rotation[0, 2] + rotation[2, 0]) / scale
        qy = (rotation[1, 2] + rotation[2, 1]) / scale
        qz = 0.25 * scale

    quaternion = np.asarray([qx, qy, qz, qw], dtype=float)
    norm = float(np.linalg.norm(quaternion))
    if not math.isfinite(norm) or norm <= 1.0e-12:
        raise ValueError("tool_pose rotation produced an invalid quaternion")
    quaternion /= norm

    return matrix[:3, 3].copy(), quaternion


def _optional_text(value: str) -> Optional[str]:
    value = value.strip()
    return value if value else None


def _optional_baudrate(value: str) -> Optional[int]:
    value = value.strip()
    if not value:
        return None
    baudrate = int(value)
    if baudrate <= 0:
        raise ValueError("baudrate must be a positive integer")
    return baudrate


class HandeyeBridge(Node):
    """Own the SerialArm session and publish base->tool pose for calibration."""

    def __init__(self) -> None:
        super().__init__("handeye_bridge")

        self.declare_parameter("robot_profile", "dm_arm_gray")
        self.declare_parameter("serial_port", "")
        self.declare_parameter("baudrate", "")
        self.declare_parameter("bus", "")
        self.declare_parameter("pose_topic", "/arm/pose")
        self.declare_parameter("publish_rate", 30.0)

        robot_profile = str(self.get_parameter("robot_profile").value)
        serial_port = _optional_text(str(self.get_parameter("serial_port").value))
        baudrate = _optional_baudrate(str(self.get_parameter("baudrate").value))
        bus = _optional_text(str(self.get_parameter("bus").value))
        pose_topic = str(self.get_parameter("pose_topic").value)
        publish_rate = float(self.get_parameter("publish_rate").value)

        if not robot_profile:
            raise ValueError("robot_profile must not be empty")
        if not pose_topic:
            raise ValueError("pose_topic must not be empty")
        if not math.isfinite(publish_rate) or publish_rate <= 0.0:
            raise ValueError("publish_rate must be > 0")

        profile = load_robot_profile_core(robot_profile)
        session = RobotSession(
            profile.core_config_path,
            profile.hardware_plugin,
            profile.hardware_config_path,
            serial_port=serial_port,
            baudrate=baudrate,
            bus=bus,
        )
        self._session = session
        self._session_started = False
        self._stopped = False
        self._waiting_for_snapshot_logged = False
        self._snapshot_error_logged = ""

        self._base_frame = session.config.dynamics.base_frame
        self._tool_frame = session.config.dynamics.tool_frame
        if not self._base_frame or not self._tool_frame:
            raise RuntimeError("SerialArm dynamics base_frame/tool_frame must not be empty")

        self._publisher = self.create_publisher(PoseStamped, pose_topic, 10)

        try:
            session.start()
            self._session_started = True
            session.set_impedance_mode(JointImpedanceMode.COMPLIANT_DRAG)
        except Exception:
            self.stop_session()
            raise

        self._timer = self.create_timer(1.0 / publish_rate, self._publish_pose)
        self.get_logger().info(
            f"Handeye bridge ready: profile={robot_profile}, mode=COMPLIANT_DRAG, "
            f"pose={pose_topic}, frame={self._base_frame}->{self._tool_frame}, rate={publish_rate:.1f} Hz"
        )

    def _publish_pose(self) -> None:
        snapshot = self._session.snapshot
        if snapshot.last_error:
            error_text = str(snapshot.last_error)
            if error_text != self._snapshot_error_logged:
                self.get_logger().error(f"SerialArm worker error: {error_text}")
                self._snapshot_error_logged = error_text
            return
        if not snapshot.valid:
            if not self._waiting_for_snapshot_logged:
                self.get_logger().warning("Waiting for the first valid SerialArm snapshot")
                self._waiting_for_snapshot_logged = True
            return

        try:
            position, quaternion = transform_matrix_to_pose(snapshot.dynamics.tool_pose)
        except ValueError as error:
            self.get_logger().error(f"Invalid SerialArm tool pose: {error}")
            return

        message = PoseStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self._base_frame
        message.pose.position.x = float(position[0])
        message.pose.position.y = float(position[1])
        message.pose.position.z = float(position[2])
        message.pose.orientation.x = float(quaternion[0])
        message.pose.orientation.y = float(quaternion[1])
        message.pose.orientation.z = float(quaternion[2])
        message.pose.orientation.w = float(quaternion[3])
        self._publisher.publish(message)

    def stop_session(self) -> None:
        """Stop SerialArm exactly once using the Core lifecycle."""
        if self._stopped:
            return
        self._stopped = True
        if not getattr(self, "_session_started", False):
            return
        try:
            self._session.stop()
        except Exception as error:
            self.get_logger().error(f"Failed to stop SerialArm session cleanly: {error}")
        finally:
            self._session_started = False

    def destroy_node(self):
        self.stop_session()
        return super().destroy_node()


def main() -> int:
    rclpy.init()
    node = None
    try:
        node = HandeyeBridge()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.stop_session()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
