from __future__ import annotations

import queue
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node

try:
    from optitrack_receiver.models import MoCapFrame, RigidBodySample
    from optitrack_receiver.natnet_receiver import NatNetReceiver
except ImportError as exc:  # pragma: no cover - runtime hint only
    NatNetReceiver = None  # type: ignore
    MoCapFrame = None  # type: ignore
    RigidBodySample = None  # type: ignore
    _IMPORT_ERROR = exc
else:
    _IMPORT_ERROR = None


class NatNetPoseNode(Node):
    """Subscribe-less publisher fed by a NatNet UDP receiver thread."""

    def __init__(self) -> None:
        super().__init__("natnet_pose_node")

        if NatNetReceiver is None:
            raise RuntimeError(
                "Python package `optitrack_receiver` not found. "
                "Install with: pip install -e <DroneXavier>/workspace/src/optitrack"
            ) from _IMPORT_ERROR

        self.declare_parameter("server_ip", "127.0.0.1")
        self.declare_parameter("local_ip", "0.0.0.0")
        self.declare_parameter("multicast_group", "239.255.42.99")
        self.declare_parameter("use_multicast", True)
        self.declare_parameter("command_port", 1510)
        self.declare_parameter("data_port", 1511)
        self.declare_parameter("rigid_body_id", -1)
        self.declare_parameter("frame_id", "world")
        self.declare_parameter("pose_topic", "/optitrack/natnet/pose")

        topic = str(self.get_parameter("pose_topic").value)
        self._pub = self.create_publisher(PoseStamped, topic, 10)

        self._frame_q = queue.Queue(maxsize=2)
        self._recv = NatNetReceiver(
            server_ip=str(self.get_parameter("server_ip").value),
            local_ip=str(self.get_parameter("local_ip").value),
            multicast_group=str(self.get_parameter("multicast_group").value),
            use_multicast=bool(self.get_parameter("use_multicast").value),
            command_port=int(self.get_parameter("command_port").value),
            data_port=int(self.get_parameter("data_port").value),
            frame_callback=self._enqueue_frame,
        )

        self._recv.start()
        period_s = 1.0 / 120.0
        self.create_timer(period_s, self._publish_latest)

        self.get_logger().info(
            "NatNet rigid body pose publisher (topic=%s). Params: server_ip, rigid_body_id, …",
            topic,
        )

    def destroy_node(self) -> bool:
        self._recv.stop()
        return super().destroy_node()

    def _enqueue_frame(self, frame: MoCapFrame) -> None:
        try:
            self._frame_q.put_nowait(frame)
        except queue.Full:
            try:
                self._frame_q.get_nowait()
            except queue.Empty:
                pass
            try:
                self._frame_q.put_nowait(frame)
            except queue.Full:
                pass

    def _pick_body(self, frame: MoCapFrame) -> Optional[RigidBodySample]:
        rid = int(self.get_parameter("rigid_body_id").value)
        if rid >= 0:
            for body in frame.rigid_bodies:
                if body.body_id == rid:
                    return body
            return None

        for body in frame.rigid_bodies:
            if body.tracking_valid:
                return body
        return frame.rigid_bodies[0] if frame.rigid_bodies else None

    def _publish_latest(self) -> None:
        frame: Optional[MoCapFrame] = None
        while True:
            try:
                frame = self._frame_q.get_nowait()
            except queue.Empty:
                break
        if frame is None:
            return

        body = self._pick_body(frame)
        if body is None:
            return

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = str(self.get_parameter("frame_id").value)
        msg.pose.position.x = float(body.position_m[0])
        msg.pose.position.y = float(body.position_m[1])
        msg.pose.position.z = float(body.position_m[2])
        q = body.quaternion_xyzw
        msg.pose.orientation.x = float(q[0])
        msg.pose.orientation.y = float(q[1])
        msg.pose.orientation.z = float(q[2])
        msg.pose.orientation.w = float(q[3])
        self._pub.publish(msg)


def main(args: Optional[list] = None) -> None:
    rclpy.init(args=args)
    node = NatNetPoseNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
