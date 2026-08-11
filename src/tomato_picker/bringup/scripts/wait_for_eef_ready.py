#!/usr/bin/env python3

import sys
import time

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


class EefReadyGate(Node):
    def __init__(self):
        super().__init__("eef_ready_gate")
        self.declare_parameter("ready_service", "/tomato_picker/eef/ready")
        self.declare_parameter("timeout_sec", 15.0)
        self.service_name = self.get_parameter("ready_service").value
        self.timeout_sec = float(self.get_parameter("timeout_sec").value)
        self.client = self.create_client(Trigger, self.service_name)

    def wait(self):
        deadline = time.monotonic() + self.timeout_sec
        self.get_logger().info(f"Waiting for EEF READY on {self.service_name}")

        while rclpy.ok() and time.monotonic() < deadline:
            if not self.client.wait_for_service(timeout_sec=0.1):
                continue

            future = self.client.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(self, future, timeout_sec=0.5)
            if not future.done() or future.result() is None:
                continue

            response = future.result()
            if response.success:
                self.get_logger().info("EEF READY confirmed")
                return 0
            if response.message in {"ERROR", "SHUTDOWN"}:
                self.get_logger().error(f"EEF readiness failed: {response.message}")
                return 2
            time.sleep(0.1)

        self.get_logger().error("Timed out waiting for EEF READY")
        return 3


def main():
    rclpy.init()
    node = EefReadyGate()
    code = node.wait()
    node.destroy_node()
    rclpy.shutdown()
    return code


if __name__ == "__main__":
    sys.exit(main())
