#!/usr/bin/env python3
"""Record can0 like `candump`, keep only ~1 minute before a fault.

juxie-ros2-control stops sending after a fault, so the bus goes quiet.
This script keeps a rolling 60s window and writes it out when CAN is idle
(or on Ctrl-C). Disk usage stays bounded.
"""

from __future__ import annotations

import argparse
import os
import select
import signal
import socket
import struct
import sys
import time
from collections import deque
from datetime import datetime
from typing import Deque, List, Optional, Tuple

# Linux CAN raw socket
AF_CAN = getattr(socket, "AF_CAN", 29)
SOCK_RAW = socket.SOCK_RAW
CAN_RAW = getattr(socket, "CAN_RAW", 1)
SOL_CAN_RAW = 101
CAN_RAW_FD_FRAMES = 5

CAN_EFF_FLAG = 0x80000000
CAN_RTR_FLAG = 0x40000000
CAN_ERR_FLAG = 0x20000000
CAN_SFF_MASK = 0x000007FF
CAN_EFF_MASK = 0x1FFFFFFF

# struct canfd_frame
CANFD_MTU = 72
CANFD_FRAME_FMT = "=IBBBB64s"


def format_candump_line(iface: str, can_id: int, data: bytes) -> str:
    is_eff = bool(can_id & CAN_EFF_FLAG)
    is_rtr = bool(can_id & CAN_RTR_FLAG)
    is_err = bool(can_id & CAN_ERR_FLAG)
    cid = can_id & (CAN_EFF_MASK if is_eff else CAN_SFF_MASK)

    if is_err:
        id_str = f"{cid:08X}"
    elif is_eff:
        id_str = f"{cid:08X}"
    else:
        id_str = f"{cid:3X}"

    extra = ""
    if is_rtr:
        extra = " remote request"
    payload = " ".join(f"{b:02X}" for b in data)
    if payload:
        return f"  {iface}  {id_str}  [{len(data):02}]  {payload}{extra}"
    return f"  {iface}  {id_str}  [{len(data):02}]{extra}"


def parse_frame(raw: bytes) -> Optional[Tuple[int, bytes]]:
    if len(raw) < 8:
        return None
    can_id, length, _flags, _r0, _r1, data = struct.unpack(CANFD_FRAME_FMT, raw[:CANFD_MTU].ljust(CANFD_MTU, b"\x00"))
    length = min(int(length), 64)
    return can_id, data[:length]


def open_can(iface: str) -> socket.socket:
    sock = socket.socket(AF_CAN, SOCK_RAW, CAN_RAW)
    try:
        sock.setsockopt(SOL_CAN_RAW, CAN_RAW_FD_FRAMES, 1)
    except OSError as exc:
        print(f"warning: CAN FD frames not enabled ({exc})", file=sys.stderr)
    sock.bind((iface,))
    sock.setblocking(False)
    return sock


def dump_window(window: Deque[Tuple[float, str]], out_path: str) -> int:
    os.makedirs(os.path.dirname(os.path.abspath(out_path)) or ".", exist_ok=True)
    with open(out_path, "w", encoding="utf-8") as f:
        for ts, line in window:
            f.write(f"({ts:.6f}){line}\n")
    return len(window)


def prune(window: Deque[Tuple[float, str]], keep_sec: float, now: float) -> None:
    cutoff = now - keep_sec
    while window and window[0][0] < cutoff:
        window.popleft()


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Circular CAN logger: keep the last N seconds and save when the bus goes idle."
    )
    parser.add_argument("-i", "--iface", default="can0", help="CAN interface (default: can0)")
    parser.add_argument(
        "-k",
        "--keep-sec",
        type=float,
        default=60.0,
        help="rolling window to keep before fault (default: 60)",
    )
    parser.add_argument(
        "--idle-sec",
        type=float,
        default=2.0,
        help="treat this much silence as a fault and save (default: 2)",
    )
    parser.add_argument(
        "-o",
        "--out-dir",
        default=".",
        help="directory for the saved log (default: current dir)",
    )
    args = parser.parse_args()

    stop = {"flag": False}

    def _on_stop(_signum, _frame) -> None:
        stop["flag"] = True

    signal.signal(signal.SIGINT, _on_stop)
    signal.signal(signal.SIGTERM, _on_stop)

    try:
        sock = open_can(args.iface)
    except PermissionError:
        print(
            f"no permission to open {args.iface}. try: sudo python3 {sys.argv[0]}",
            file=sys.stderr,
        )
        return 1
    except OSError as exc:
        print(f"failed to open {args.iface}: {exc}", file=sys.stderr)
        return 1

    window: Deque[Tuple[float, str]] = deque()
    last_frame_ts: Optional[float] = None
    frames = 0

    print(
        f"recording {args.iface}: keep last {args.keep_sec:.0f}s, "
        f"save on {args.idle_sec:.1f}s idle after traffic starts. Ctrl-C to save now.",
        file=sys.stderr,
    )

    reason = "stopped"
    try:
        while not stop["flag"]:
            now = time.time()
            if last_frame_ts is not None and (now - last_frame_ts) >= args.idle_sec:
                reason = "bus idle (likely fault / control stopped)"
                break

            ready, _, _ = select.select([sock], [], [], 0.2)
            if not ready:
                continue

            try:
                raw = sock.recv(CANFD_MTU)
            except BlockingIOError:
                continue

            ts = time.time()
            parsed = parse_frame(raw)
            if parsed is None:
                continue
            can_id, data = parsed
            line = format_candump_line(args.iface, can_id, data)
            window.append((ts, line))
            prune(window, args.keep_sec, ts)
            last_frame_ts = ts
            frames += 1
            if frames == 1:
                print("first frame received, window running.", file=sys.stderr)
    except KeyboardInterrupt:
        reason = "Ctrl-C"
    finally:
        sock.close()

    if not window:
        print("no frames captured, nothing to save.", file=sys.stderr)
        return 0

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    out_path = os.path.join(args.out_dir, f"can0_{stamp}.log")
    n = dump_window(window, out_path)
    span = window[-1][0] - window[0][0]
    print(
        f"saved {n} frames ({span:.1f}s) to {out_path}  [{reason}]",
        file=sys.stderr,
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
