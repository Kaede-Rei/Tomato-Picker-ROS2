import importlib.util
import math
from pathlib import Path
import sys
import types
import unittest

import numpy as np


BRIDGE = Path(__file__).resolve().parents[1] / "scripts" / "handeye_bridge.py"
LAUNCH = Path(__file__).resolve().parents[1] / "launch" / "handeye.launch.py"
CMAKE = Path(__file__).resolve().parents[1] / "CMakeLists.txt"
PACKAGE_XML = Path(__file__).resolve().parents[1] / "package.xml"


def _install_import_stubs():
    geometry_msgs = types.ModuleType("geometry_msgs")
    geometry_msgs_msg = types.ModuleType("geometry_msgs.msg")

    class PoseStamped:
        pass

    geometry_msgs_msg.PoseStamped = PoseStamped
    geometry_msgs.msg = geometry_msgs_msg
    sys.modules["geometry_msgs"] = geometry_msgs
    sys.modules["geometry_msgs.msg"] = geometry_msgs_msg

    rclpy = types.ModuleType("rclpy")
    rclpy_node = types.ModuleType("rclpy.node")

    class Node:
        pass

    rclpy_node.Node = Node
    rclpy.node = rclpy_node
    sys.modules["rclpy"] = rclpy
    sys.modules["rclpy.node"] = rclpy_node

    serial_arm = types.ModuleType("serial_arm")

    class JointImpedanceMode:
        COMPLIANT_DRAG = object()

    class RobotSession:
        pass

    serial_arm.JointImpedanceMode = JointImpedanceMode
    serial_arm.RobotSession = RobotSession
    serial_arm.load_robot_profile_core = lambda *_args, **_kwargs: None
    sys.modules["serial_arm"] = serial_arm


def _load_bridge_module():
    _install_import_stubs()
    spec = importlib.util.spec_from_file_location("handeye_bridge_under_test", BRIDGE)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


class HandeyeBridgeContractTest(unittest.TestCase):
    def test_bridge_file_exists(self):
        self.assertTrue(BRIDGE.is_file())

    def test_identity_transform_becomes_identity_pose(self):
        module = _load_bridge_module()
        position, quaternion = module.transform_matrix_to_pose(np.eye(4))
        np.testing.assert_allclose(position, [0.0, 0.0, 0.0], atol=1e-12)
        np.testing.assert_allclose(quaternion, [0.0, 0.0, 0.0, 1.0], atol=1e-12)

    def test_transform_conversion_keeps_translation_and_normalizes_quaternion(self):
        module = _load_bridge_module()
        angle = math.pi / 2.0
        transform = np.array(
            [
                [math.cos(angle), -math.sin(angle), 0.0, 0.12],
                [math.sin(angle), math.cos(angle), 0.0, -0.34],
                [0.0, 0.0, 1.0, 0.56],
                [0.0, 0.0, 0.0, 1.0],
            ],
            dtype=float,
        )
        position, quaternion = module.transform_matrix_to_pose(transform)
        np.testing.assert_allclose(position, [0.12, -0.34, 0.56], atol=1e-12)
        self.assertAlmostEqual(float(np.linalg.norm(quaternion)), 1.0, places=12)
        np.testing.assert_allclose(np.abs(quaternion), [0.0, 0.0, math.sqrt(0.5), math.sqrt(0.5)], atol=1e-12)

    def test_bridge_uses_core_session_drag_and_cached_fk(self):
        source = BRIDGE.read_text(encoding="utf-8")
        self.assertIn("RobotSession(", source)
        self.assertIn("JointImpedanceMode.COMPLIANT_DRAG", source)
        self.assertIn("snapshot.dynamics.tool_pose", source)
        self.assertIn("session.config.dynamics.base_frame", source)
        self.assertIn("PoseStamped", source)
        self.assertIn('"/arm/pose"', source)
        self.assertIn("session.stop()", source)
        self.assertNotIn("joint_trajectory_controller", source)
        self.assertNotIn("move_group", source)

    def test_handeye_launch_is_dedicated_and_minimal(self):
        self.assertTrue(LAUNCH.is_file())
        source = LAUNCH.read_text(encoding="utf-8")
        self.assertIn('default_value="dm_arm_gray"', source)
        self.assertIn('default_value="/arm/pose"', source)
        self.assertIn('default_value="30.0"', source)
        self.assertIn('executable="handeye_bridge"', source)
        for forbidden in (
            "moveit.launch.py",
            "arm_motion_node",
            "pick_task_node",
            "eef_controller",
            "cloud_preprocessor",
            "wrist_target_gui",
        ):
            self.assertNotIn(forbidden, source)

    def test_build_metadata_installs_bridge_and_declares_direct_dependencies(self):
        cmake = CMAKE.read_text(encoding="utf-8")
        package_xml = PACKAGE_XML.read_text(encoding="utf-8")
        self.assertIn("scripts/handeye_bridge.py", cmake)
        self.assertIn("RENAME handeye_bridge", cmake)
        self.assertIn("ament_add_pytest_test", cmake)
        self.assertIn("<exec_depend>geometry_msgs</exec_depend>", package_xml)
        self.assertIn("<exec_depend>serial_arm_core</exec_depend>", package_xml)
        self.assertIn("<exec_depend>python3-numpy</exec_depend>", package_xml)
        self.assertIn("<test_depend>ament_cmake_pytest</test_depend>", package_xml)


if __name__ == "__main__":
    unittest.main()
