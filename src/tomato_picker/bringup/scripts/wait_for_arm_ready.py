#!/usr/bin/env python3

import sys
import time

import rclpy
from controller_manager_msgs.srv import ListControllers
from moveit_msgs.action import MoveGroup
from rclpy.action import ActionClient
from rclpy.node import Node


class ArmReadyGate(Node):
    def __init__(self):
        super().__init__("arm_ready_gate")
        self.declare_parameter("controller_manager", "/controller_manager")
        self.declare_parameter("move_action", "/move_action")
        self.declare_parameter("timeout_sec", 30.0)
        self.declare_parameter("stable_sec", 1.0)

        manager = self.get_parameter("controller_manager").value.rstrip("/")
        move_action = self.get_parameter("move_action").value
        self.timeout_sec = float(self.get_parameter("timeout_sec").value)
        self.stable_sec = float(self.get_parameter("stable_sec").value)
        self.controllers = self.create_client(ListControllers, f"{manager}/list_controllers")
        self.move_group = ActionClient(self, MoveGroup, move_action)

    def ready(self):
        if not self.controllers.wait_for_service(timeout_sec=0.1):
            return False

        future = self.controllers.call_async(ListControllers.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=0.5)
        if not future.done() or future.result() is None:
            return False

        states = {item.name: item.state for item in future.result().controller}
        controllers_ready = (
            states.get("joint_state_broadcaster") == "active"
            and states.get("joint_trajectory_controller") == "active"
        )
        return controllers_ready and self.move_group.wait_for_server(timeout_sec=0.05)

    def wait(self):
        deadline = time.monotonic() + self.timeout_sec
        stable_since = None
        self.get_logger().info("Waiting for SerialArm controllers and MoveGroup readiness")

        while rclpy.ok() and time.monotonic() < deadline:
            if self.ready():
                if stable_since is None:
                    stable_since = time.monotonic()
                if time.monotonic() - stable_since >= self.stable_sec:
                    self.get_logger().info("SerialArm ready and stable")
                    return 0
            else:
                stable_since = None
            time.sleep(0.1)

        self.get_logger().error("Timed out waiting for SerialArm readiness")
        return 2


def main():
    rclpy.init()
    node = ArmReadyGate()
    code = node.wait()
    node.destroy_node()
    rclpy.shutdown()
    return code


if __name__ == "__main__":
    sys.exit(main())
