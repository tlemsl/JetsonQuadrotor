# SPDX-License-Identifier: Apache-2.0
#
# MoCap frame depacketization derived from OptiTrack NatNet Python samples
# (Apache-2.0), e.g. https://github.com/chobitsfan/PythonClient/blob/master/NatNetClient.py
# Adapted for DroneXavier: typed outputs, stop/start lifecycle, unicast option.

from __future__ import annotations

import logging
import socket
import struct
import threading
from typing import Callable, List, Optional, Tuple

from optitrack_receiver.models import MoCapFrame, RigidBodySample

_logger = logging.getLogger(__name__)

Vector3 = struct.Struct("<fff")
Quaternion = struct.Struct("<ffff")
FloatValue = struct.Struct("<f")
DoubleValue = struct.Struct("<d")

NatNetVersion = Tuple[int, int, int, int]


class NatNetDecoder:
    """Stateful decoder using NatNet stream version negotiated via ping responses."""

    NAT_FRAMEOFDATA = 7
    NAT_MODELDEF = 5
    NAT_PINGRESPONSE = 1
    NAT_RESPONSE = 3
    NAT_UNRECOGNIZED_REQUEST = 100
    NAT_MESSAGESTRING = 8

    def __init__(self, initial_version: NatNetVersion = (3, 0, 0, 0)) -> None:
        self.stream_version = initial_version

    def set_stream_version(self, v: NatNetVersion) -> None:
        self.stream_version = v
        _logger.info("NatNet stream version from server: %s.%s.%s.%s", *v)

    def unpack_rigid_body(
        self,
        data: bytes,
        offset: int,
        out: Optional[List[RigidBodySample]],
    ) -> int:
        major = self.stream_version[0]

        body_id = int.from_bytes(data[offset : offset + 4], "little")
        offset += 4

        pos = Vector3.unpack(data[offset : offset + 12])
        offset += 12

        quat = Quaternion.unpack(data[offset : offset + 16])
        offset += 16

        # Marker positions bundled in frame data for NatNet < 3 (not in RB description).
        if major < 3 and major != 0:
            marker_count = int.from_bytes(data[offset : offset + 4], "little")
            offset += 4
            offset += marker_count * 12
            if major >= 2:
                offset += marker_count * 4  # marker ids
                offset += marker_count * 4  # marker sizes

        marker_error = 0.0
        if major >= 2:
            marker_error = FloatValue.unpack(data[offset : offset + 4])[0]
            offset += 4

        tracking_valid = True
        minor = self.stream_version[1]
        if (major == 2 and minor >= 6) or major > 2 or major == 0:
            param = struct.unpack("<h", data[offset : offset + 2])[0]
            tracking_valid = (param & 0x01) != 0
            offset += 2

        if out is not None:
            out.append(
                RigidBodySample(
                    body_id=body_id,
                    position_m=(float(pos[0]), float(pos[1]), float(pos[2])),
                    quaternion_xyzw=(
                        float(quat[0]),
                        float(quat[1]),
                        float(quat[2]),
                        float(quat[3]),
                    ),
                    marker_error=float(marker_error),
                    tracking_valid=bool(tracking_valid),
                )
            )
        return offset

    def unpack_skeleton(self, data: bytes, offset: int) -> int:
        offset += 4  # skeleton id
        rigid_body_count = int.from_bytes(data[offset : offset + 4], "little")
        offset += 4
        for _ in range(rigid_body_count):
            offset = self.unpack_rigid_body(data, offset, None)
        return offset

    def unpack_mocap_data(self, payload: bytes) -> MoCapFrame:
        major = self.stream_version[0]
        minor = self.stream_version[1]

        data = memoryview(payload)
        offset = 0

        frame_number = int.from_bytes(data[offset : offset + 4], "little")
        offset += 4

        marker_set_count = int.from_bytes(data[offset : offset + 4], "little")
        offset += 4
        for _ in range(marker_set_count):
            raw = bytes(data[offset:])
            name, _, _ = raw.partition(b"\x00")
            offset += len(name) + 1
            marker_count = int.from_bytes(data[offset : offset + 4], "little")
            offset += 4
            offset += marker_count * 12

        unlabeled_markers_count = int.from_bytes(data[offset : offset + 4], "little")
        offset += 4
        offset += unlabeled_markers_count * 12

        rigid_body_count = int.from_bytes(data[offset : offset + 4], "little")
        offset += 4

        rigid_bodies: List[RigidBodySample] = []
        for _ in range(rigid_body_count):
            offset = self.unpack_rigid_body(bytes(data), offset, rigid_bodies)

        skeleton_count = 0
        if (major == 2 and minor > 0) or major > 2:
            skeleton_count = int.from_bytes(data[offset : offset + 4], "little")
            offset += 4
            for _ in range(skeleton_count):
                offset = self.unpack_skeleton(bytes(data), offset)

        labeled_marker_count = 0
        if (major == 2 and minor > 3) or major > 2:
            labeled_marker_count = int.from_bytes(data[offset : offset + 4], "little")
            offset += 4
            for _ in range(labeled_marker_count):
                offset += 4  # marker id
                offset += 12  # pos
                offset += 4  # size
                if (major == 2 and minor >= 6) or major > 2 or major == 0:
                    offset += 2  # params
                if major >= 3 or major == 0:
                    offset += 4  # residual

        if (major == 2 and minor >= 9) or major > 2:
            force_plate_count = int.from_bytes(data[offset : offset + 4], "little")
            offset += 4
            for _ in range(force_plate_count):
                offset += 4  # id
                force_plate_channel_count = int.from_bytes(data[offset : offset + 4], "little")
                offset += 4
                for _ in range(force_plate_channel_count):
                    force_plate_channel_frame_count = int.from_bytes(
                        data[offset : offset + 4], "little"
                    )
                    offset += 4
                    offset += force_plate_channel_frame_count * 4

        if (major == 2 and minor >= 11) or major > 2:
            device_count = int.from_bytes(data[offset : offset + 4], "little")
            offset += 4
            for _ in range(device_count):
                offset += 4  # device id
                device_channel_count = int.from_bytes(data[offset : offset + 4], "little")
                offset += 4
                for _ in range(device_channel_count):
                    device_channel_frame_count = int.from_bytes(data[offset : offset + 4], "little")
                    offset += 4
                    offset += device_channel_frame_count * 4

        offset += 4  # timecode
        offset += 4  # timecode_sub

        if (major == 2 and minor >= 7) or major > 2:
            offset += 8  # timestamp double
        else:
            offset += 4  # timestamp float

        if major >= 3 or major == 0:
            offset += 8  # stampCameraExposure
            offset += 8  # stampDataReceived
            offset += 8  # stampTransmit

        offset += 2  # isRecording / trackedModelsChanged

        _logger.debug(
            "Decoded frame=%s rb=%s skeleton=%s labeled=%s parse_end=%s payload=%s",
            frame_number,
            rigid_body_count,
            skeleton_count,
            labeled_marker_count,
            offset,
            len(payload),
        )

        return MoCapFrame(frame_number=int(frame_number), rigid_bodies=rigid_bodies)


def build_command_packet(command: int, command_str: str = "") -> bytes:
    """Pack a NatNet command datagram (matches OptiTrack sample layout)."""
    # NAT_REQUEST_MODELDEF = 4, NAT_REQUEST_FRAMEOFDATA = 6
    if command in (4, 6):
        packet_size = 0
        payload_str = ""
    elif command == 2:  # NAT_REQUEST
        packet_size = len(command_str) + 1
        payload_str = command_str
    elif command == 0:  # NAT_PING
        payload_str = "Ping"
        packet_size = len(payload_str) + 1
    else:
        payload_str = command_str
        packet_size = len(payload_str) + 1 if payload_str else 0

    data = command.to_bytes(2, "little")
    data += packet_size.to_bytes(2, "little")
    data += payload_str.encode("utf-8")
    data += b"\x00"
    return data


class NatNetReceiver:
    """Receive NatNet UDP frames and invoke an optional callback per decoded MoCap frame."""

    NAT_PING = 0
    NAT_REQUEST_MODELDEF = 4

    def __init__(
        self,
        *,
        server_ip: str,
        local_ip: str = "0.0.0.0",
        multicast_group: str = "239.255.42.99",
        use_multicast: bool = True,
        command_port: int = 1510,
        data_port: int = 1511,
        decoder: Optional[NatNetDecoder] = None,
        frame_callback: Optional[Callable[[MoCapFrame], None]] = None,
    ) -> None:
        self.server_ip = server_ip
        self.local_ip = local_ip
        self.multicast_group = multicast_group
        self.use_multicast = use_multicast
        self.command_port = command_port
        self.data_port = data_port
        self.decoder = decoder or NatNetDecoder()
        self.frame_callback = frame_callback

        self._stop = threading.Event()
        self._data_sock: Optional[socket.socket] = None
        self._cmd_sock: Optional[socket.socket] = None
        self._threads: List[threading.Thread] = []

    def _create_data_socket(self) -> socket.socket:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        bind_ip = self.local_ip if self.local_ip else "0.0.0.0"
        if self.use_multicast:
            mreq = socket.inet_aton(self.multicast_group) + socket.inet_aton(bind_ip)
            sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
        sock.bind((bind_ip, self.data_port))
        sock.settimeout(0.5)
        return sock

    def _create_command_socket(self) -> socket.socket:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind(("", 0))
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        sock.settimeout(0.5)
        return sock

    def _handle_packet(self, packet: bytes) -> None:
        if len(packet) < 4:
            return
        message_id = int.from_bytes(packet[0:2], "little")
        packet_size = int.from_bytes(packet[2:4], "little")
        payload = packet[4:]
        if packet_size > 0:
            payload = payload[:packet_size]

        if message_id == self.decoder.NAT_FRAMEOFDATA:
            frame = self.decoder.unpack_mocap_data(payload)
            if self.frame_callback is not None:
                self.frame_callback(frame)
        elif message_id == self.decoder.NAT_PINGRESPONSE:
            offset = 0
            offset += 256  # sending app name
            offset += 4  # sending app version / NatNet version blob start in legacy samples
            if len(payload) >= offset + 4:
                ver = struct.unpack("<BBBB", payload[offset : offset + 4])
                self.decoder.set_stream_version(ver)
        elif message_id == self.decoder.NAT_MODELDEF:
            _logger.debug("Ignored NAT_MODELDEF (%s bytes)", len(payload))

    def _recv_loop(self, sock: socket.socket) -> None:
        while not self._stop.is_set():
            try:
                data, _addr = sock.recvfrom(65536)
            except socket.timeout:
                continue
            except OSError:
                break
            if data:
                try:
                    self._handle_packet(data)
                except Exception:  # pragma: no cover - streaming robustness
                    _logger.exception("Failed to parse NatNet packet")

    def start(self, *, request_modeldef: bool = True, send_ping: bool = True) -> None:
        if self._data_sock is not None:
            raise RuntimeError("NatNetReceiver already started")

        self._stop.clear()
        self._data_sock = self._create_data_socket()
        self._cmd_sock = self._create_command_socket()

        self._threads = [
            threading.Thread(target=self._recv_loop, args=(self._data_sock,), daemon=True),
            threading.Thread(target=self._recv_loop, args=(self._cmd_sock,), daemon=True),
        ]
        for t in self._threads:
            t.start()

        cmd_sock = self._cmd_sock
        assert cmd_sock is not None
        server_addr = (self.server_ip, self.command_port)

        if send_ping:
            cmd_sock.sendto(build_command_packet(self.NAT_PING, ""), server_addr)
        if request_modeldef:
            cmd_sock.sendto(build_command_packet(self.NAT_REQUEST_MODELDEF, ""), server_addr)

    def stop(self) -> None:
        self._stop.set()
        for s in (self._data_sock, self._cmd_sock):
            if s is not None:
                try:
                    s.close()
                except OSError:
                    pass
        self._data_sock = None
        self._cmd_sock = None
        self._threads.clear()
