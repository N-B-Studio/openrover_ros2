#!/usr/bin/env python3
"""
Standalone STM32 base serial test. No ROS dependency.

Examples:
  python3 serial_base_test.py --port /dev/ttyUSB0
  python3 serial_base_test.py --port /dev/ttyACM0 --drive-left 200 --drive-right 200
"""
from __future__ import annotations

import argparse
import signal
import sys
import time
from dataclasses import dataclass

import serial


@dataclass(frozen=True)
class OdomFrame:
    seq: int
    mcu_ms: int
    left_ticks: int
    right_ticks: int
    left_rpm_x100: int
    right_rpm_x100: int
    left_raw: int
    right_raw: int
    status: int


def parse_odom(line: str) -> OdomFrame | None:
    fields = line.strip().split(",")
    if len(fields) != 10 or fields[0] != "ODOM":
        return None
    try:
        values = [int(v, 0) for v in fields[1:]]
    except ValueError:
        return None
    return OdomFrame(*values)


def send_line(port: serial.Serial, command: str) -> None:
    port.write((command.rstrip("\r\n") + "\n").encode("ascii"))
    port.flush()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", required=True)
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--drive-left", type=int, default=0,
                        help="-1000..1000 normalized command")
    parser.add_argument("--drive-right", type=int, default=0,
                        help="-1000..1000 normalized command")
    parser.add_argument("--seconds", type=float, default=3.0)
    args = parser.parse_args()

    left = max(-1000, min(1000, args.drive_left))
    right = max(-1000, min(1000, args.drive_right))

    with serial.Serial(args.port, args.baud, timeout=0.2) as port:
        time.sleep(0.3)
        port.reset_input_buffer()

        def stop(*_: object) -> None:
            try:
                send_line(port, "S")
            finally:
                raise KeyboardInterrupt

        signal.signal(signal.SIGINT, stop)
        signal.signal(signal.SIGTERM, stop)

        send_line(port, "INFO")
        send_line(port, "Z")
        send_line(port, f"V,{left},{right}")

        deadline = time.monotonic() + args.seconds
        last_command = 0.0

        try:
            while time.monotonic() < deadline:
                # Refresh faster than the STM32 300 ms command watchdog.
                now = time.monotonic()
                if now - last_command >= 0.1:
                    send_line(port, f"V,{left},{right}")
                    last_command = now

                raw = port.readline()
                if not raw:
                    continue

                line = raw.decode("ascii", errors="replace").strip()
                frame = parse_odom(line)
                if frame is not None:
                    print(
                        f"seq={frame.seq:6d} ms={frame.mcu_ms:9d} "
                        f"ticks=({frame.left_ticks:8d},{frame.right_ticks:8d}) "
                        f"rpm=({frame.left_rpm_x100 / 100:7.2f},"
                        f"{frame.right_rpm_x100 / 100:7.2f}) "
                        f"raw=({frame.left_raw:8d},{frame.right_raw:8d}) "
                        f"status=0x{frame.status:02x}"
                    )
                elif line:
                    print(f"MCU: {line}")
        finally:
            send_line(port, "S")
            print("STOP sent")

    return 0


if __name__ == "__main__":
    sys.exit(main())