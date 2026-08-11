#!/usr/bin/env python3

import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from tomato_picker_interfaces.action import MoveArm


class MotionReadyGate(Node):
    def __init__(self):
        super().__init__("motion_ready_gate")
        self.declare_parameter("action_name", "/tomato_picker/motion/move_arm")
        self.declare_parameter("timeout_sec", 10.0)
        self.action_name = self.get_parameter("action_name").value
        self.timeout_sec = float(self.get_parameter("timeout_sec").value)
        self.client = ActionClient(self, MoveArm, self.action_name)

    def wait(self):
        deadline = time.monotonic() + self.timeout_sec
        self.get_logger().info(f"Waiting for Motion Action on {self.action_name}")
        while rclpy.ok() and time.monotonic() < deadline:
            if self.client.wait_for_server(timeout_sec=0.1):
                self.get_logger().info("Motion Action READY confirmed")
                return 0
        self.get_logger().error("Timed out waiting for Motion Action")
        return 2


def main():
    rclpy.init()
    node = MotionReadyGate()
    code = node.wait()
    node.destroy_node()
    rclpy.shutdown()
    return code


if __name__ == "__main__":
    sys.exit(main())
