from __future__ import annotations

import queue
import time
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from optitrack_ros.enu_conversion import (
    VerticalAxisMode,
    enu_to_ned_px4,
    moticap_to_enu,
    quat_geometry_xyzw_to_px4_wxyz,
)

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

def _try_import_vehicle_odometry():  # pragma: no cover - env dependent
    try:
        from px4_msgs.msg import VehicleOdometry

        return VehicleOdometry
    except ImportError:
        return None


def _px4_qos() -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
    )


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

        self.declare_parameter("vertical_axis_mode", "z_up")
        self.declare_parameter("world_yaw_offset_deg", 0.0)
        self.declare_parameter("publish_visual_odometry", True)
        self.declare_parameter("visual_odometry_topic", "/fmu/in/vehicle_visual_odometry")
        self.declare_parameter(
            "px4_convert_enu_to_ned",
            True,
            "If true, publish POSE_FRAME_NED after internal ENU (px4_msgs has no POSE_FRAME_ENU). "
            "If false, send E/N/U in position with pose_frame=UNKNOWN (PX4 may or may not accept).",
        )
        self.declare_parameter("position_variance", [0.01, 0.01, 0.01])
        self.declare_parameter("orientation_variance", [0.001, 0.001, 0.001])

        topic = str(self.get_parameter("pose_topic").value)
        self._pub = self.create_publisher(PoseStamped, topic, 10)

        self._vo_pub = None
        vo_cls = _try_import_vehicle_odometry()
        self._vehicle_odometry_cls: Optional[type] = vo_cls
        if vo_cls is None:
            self.get_logger().warn(
                "px4_msgs not found; skipping /fmu/in/vehicle_visual_odometry. "
                "Build workspace with px4_msgs (e.g. source ~/px4_ros_dds_ws/install/setup.bash)."
            )
        elif bool(self.get_parameter("publish_visual_odometry").value):
            vo_topic = str(self.get_parameter("visual_odometry_topic").value)
            assert vo_cls is not None
            self._vo_pub = self.create_publisher(vo_cls, vo_topic, _px4_qos())
            self.get_logger().info(
                f"Publishing VehicleOdometry to {vo_topic} (px4_msgs; see px4_convert_enu_to_ned)."
            )

        self._t0_mono_ns = time.monotonic_ns()
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
            f"NatNet rigid body pose publisher (topic={topic}). "
            "Params: server_ip, rigid_body_id, vertical_axis_mode, …"
        )

    def destroy_node(self) -> bool:
        self._recv.stop()
        return super().destroy_node()

    def _px4_timestamp_us(self) -> int:
        return int((time.monotonic_ns() - self._t0_mono_ns) // 1000)

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

    def _axis_mode(self) -> VerticalAxisMode:
        raw = str(self.get_parameter("vertical_axis_mode").value).strip().lower()
        if raw not in ("z_up", "y_up"):
            self.get_logger().warn(f"vertical_axis_mode={raw!r} invalid; using z_up")
            return "z_up"
        return raw  # type: ignore[return-value]

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

        if self._vo_pub is None:
            return
        vo_mod = self._vehicle_odometry_cls
        assert vo_mod is not None

        mode = self._axis_mode()
        yaw_off = float(self.get_parameter("world_yaw_offset_deg").value)
        p_enu, q_xyzw_enu = moticap_to_enu(
            body.position_m,
            body.quaternion_xyzw,
            vertical_axis_mode=mode,
            world_yaw_offset_deg=yaw_off,
        )

        convert = bool(self.get_parameter("px4_convert_enu_to_ned").value)
        if convert:
            p_out, q_xyzw_out = enu_to_ned_px4(p_enu, q_xyzw_enu)
            pose_frame = vo_mod.POSE_FRAME_NED
            vel_frame = vo_mod.VELOCITY_FRAME_NED
        else:
            p_out = p_enu
            q_xyzw_out = q_xyzw_enu
            pose_frame = vo_mod.POSE_FRAME_UNKNOWN
            vel_frame = vo_mod.VELOCITY_FRAME_UNKNOWN

        qw, qx, qy, qz = quat_geometry_xyzw_to_px4_wxyz(q_xyzw_out)
        nan = float("nan")

        vo = vo_mod()
        if hasattr(vo, "message_version"):
            vo.message_version = int(getattr(vo_mod, "MESSAGE_VERSION", 0))
        ts = self._px4_timestamp_us()
        vo.timestamp = ts
        vo.timestamp_sample = ts
        vo.pose_frame = pose_frame
        vo.position = [float(p_out[0]), float(p_out[1]), float(p_out[2])]
        vo.q = [float(qw), float(qx), float(qy), float(qz)]
        vo.velocity_frame = vel_frame
        vo.velocity = [nan, nan, nan]
        vo.angular_velocity = [nan, nan, nan]

        pv = self.get_parameter("position_variance").value
        ov = self.get_parameter("orientation_variance").value
        vo.position_variance = [float(pv[0]), float(pv[1]), float(pv[2])]
        vo.orientation_variance = [float(ov[0]), float(ov[1]), float(ov[2])]
        vo.velocity_variance = [nan, nan, nan]
        vo.reset_counter = 0
        vo.quality = 0

        self._vo_pub.publish(vo)


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
