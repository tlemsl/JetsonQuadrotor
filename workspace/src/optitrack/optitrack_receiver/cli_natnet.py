from __future__ import annotations

import argparse
import logging
import sys
import time
from typing import List, Optional

from optitrack_receiver.models import MoCapFrame
from optitrack_receiver.natnet_receiver import NatNetReceiver


def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(description="Print OptiTrack NatNet rigid bodies (UDP).")
    parser.add_argument("--server", default="127.0.0.1", help="Motive / NatNet server IP")
    parser.add_argument("--local", default="0.0.0.0", help="Local bind / multicast interface IP")
    parser.add_argument("--multicast-group", default="239.255.42.99")
    parser.add_argument("--no-multicast", action="store_true", help="Unicast capture mode")
    parser.add_argument("--cmd-port", type=int, default=1510)
    parser.add_argument("--data-port", type=int, default=1511)
    parser.add_argument("--duration", type=float, default=0.0, help="Seconds to run (0 = until Ctrl+C)")
    parser.add_argument("-v", "--verbose", action="store_true")
    args = parser.parse_args(argv)

    logging.basicConfig(level=logging.DEBUG if args.verbose else logging.INFO)

    last_print = time.monotonic()

    def on_frame(frame: MoCapFrame) -> None:
        nonlocal last_print
        now = time.monotonic()
        if now - last_print < 0.25:
            return
        last_print = now
        bodies = ", ".join(
            f"id={b.body_id} pos=({b.position_m[0]:.3f},{b.position_m[1]:.3f},{b.position_m[2]:.3f})"
            f" valid={b.tracking_valid}"
            for b in frame.rigid_bodies
        )
        print(f"frame={frame.frame_number} rigid_bodies=[{bodies}]")

    recv = NatNetReceiver(
        server_ip=args.server,
        local_ip=args.local,
        multicast_group=args.multicast_group,
        use_multicast=not args.no_multicast,
        command_port=args.cmd_port,
        data_port=args.data_port,
        frame_callback=on_frame,
    )
    recv.start()
    try:
        if args.duration > 0:
            time.sleep(args.duration)
        else:
            while True:
                time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    finally:
        recv.stop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
