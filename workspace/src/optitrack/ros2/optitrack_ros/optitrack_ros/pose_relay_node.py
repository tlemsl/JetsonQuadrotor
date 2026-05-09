from __future__ import annotations

from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node


class PoseRelayNode(Node):
    """Republish PoseStamped from VRPN (or any source) under a stable topic name."""

    def __init__(self) -> None:
        super().__init__("pose_relay_node")

        self.declare_parameter("input_topic", "")
        self.declare_parameter("output_topic", "/optitrack/vrpn/pose")

        input_topic = str(self.get_parameter("input_topic").value).strip()
        output_topic = str(self.get_parameter("output_topic").value)

        if not input_topic:
            raise RuntimeError("Parameter `input_topic` must be set (VRPN pose topic).")

        self._pub = self.create_publisher(PoseStamped, output_topic, 10)
        self.create_subscription(PoseStamped, input_topic, self._cb, 10)

        self.get_logger().info("Relay %s -> %s", input_topic, output_topic)

    def _cb(self, msg: PoseStamped) -> None:
        self._pub.publish(msg)


def main(args: Optional[list] = None) -> None:
    rclpy.init(args=args)
    node = PoseRelayNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
