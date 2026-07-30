#!/usr/bin/env python3

import os
import select
import sys
import termios
import threading
import time
import tty

import serial


PORT = "/dev/ttyS4"
BAUD = 115200

TX_HZ = 20.0
IP_TX_PERIOD_S = 5.0

DRIVE_RPM = 50.0
TURN_RPM = 40.0


class BaseSerialTest:
    def __init__(self) -> None:
        self.ser = serial.Serial(
            port=PORT,
            baudrate=BAUD,
            timeout=0.05,
        )

        self.running = True

        self.left_rpm = 0.0
        self.right_rpm = 0.0

        self.rx_thread = threading.Thread(
            target=self._rx_loop,
            daemon=True,
        )

    @staticmethod
    def get_ipv4() -> str:
        output = os.popen("hostname -I").read().strip()

        for token in output.split():
            if (
                token.count(".") == 3
                and not token.startswith("127.")
            ):
                return token

        return "192.168.xxx.xxx"

    def send_ip(self) -> None:
        ip = self.get_ipv4()

        frame = f"IP,{ip}\n"

        self.ser.write(
            frame.encode("ascii")
        )

        print(f"\n[TX] {frame.strip()}")

    def set_velocity(
        self,
        left_rpm: float,
        right_rpm: float,
    ) -> None:
        self.left_rpm = left_rpm
        self.right_rpm = right_rpm

    def send_velocity(self) -> None:
        left_raw = round(
            self.left_rpm * 100.0
        )

        right_raw = round(
            self.right_rpm * 100.0
        )

        frame = (
            f"V,{left_raw},{right_raw}\n"
        )

        self.ser.write(
            frame.encode("ascii")
        )

    def _rx_loop(self) -> None:
        while self.running:
            try:
                line = (
                    self.ser
                    .readline()
                    .decode(
                        "ascii",
                        errors="replace",
                    )
                    .strip()
                )

                if not line:
                    continue

                if line.startswith("FB,"):
                    self._parse_feedback(line)

                elif line.startswith("DBG,"):
                    print(f"\n{line}")

                else:
                    print(f"\n[MCU] {line}")

            except serial.SerialException as exc:
                print(
                    f"\n[SERIAL ERROR] {exc}"
                )

                self.running = False

    @staticmethod
    def _parse_feedback(
        line: str,
    ) -> None:
        parts = line.split(",")

        if len(parts) != 7:
            print(f"\n[BAD FB] {line}")
            return

        try:
            mcu_ms = int(parts[1])

            left_pos_deg = (
                int(parts[2]) / 100.0
            )

            left_vel_rpm = (
                int(parts[3]) / 100.0
            )

            right_pos_deg = (
                int(parts[4]) / 100.0
            )

            right_vel_rpm = (
                int(parts[5]) / 100.0
            )

            status = int(parts[6])

        except ValueError:
            print(f"\n[BAD FB] {line}")
            return

        print(
            "\r"
            f"MCU={mcu_ms:8d} ms | "
            f"L={left_vel_rpm:+7.2f} rpm "
            f"pos={left_pos_deg:+9.2f} deg | "
            f"R={right_vel_rpm:+7.2f} rpm "
            f"pos={right_pos_deg:+9.2f} deg | "
            f"status=0x{status:02X}",
            end="",
            flush=True,
        )

    def run(self) -> None:
        self.rx_thread.start()

        self.send_ip()

        print(
            "\nControls:\n"
            "  W = forward\n"
            "  S = backward\n"
            "  A = left\n"
            "  D = right\n"
            "  X/Space = stop\n"
            "  Q = quit\n"
        )

        original_terminal = (
            termios.tcgetattr(sys.stdin)
        )

        tty.setcbreak(
            sys.stdin.fileno()
        )

        last_velocity_tx = 0.0
        last_ip_tx = 0.0

        try:
            while self.running:
                now = time.monotonic()

                if (
                    now - last_velocity_tx
                    >= 1.0 / TX_HZ
                ):
                    last_velocity_tx = now
                    self.send_velocity()

                if (
                    now - last_ip_tx
                    >= IP_TX_PERIOD_S
                ):
                    last_ip_tx = now
                    self.send_ip()

                readable, _, _ = select.select(
                    [sys.stdin],
                    [],
                    [],
                    0.01,
                )

                if not readable:
                    continue

                key = sys.stdin.read(1).lower()

                if key == "w":
                    self.set_velocity(
                        +DRIVE_RPM,
                        -DRIVE_RPM,
                    )

                elif key == "s":
                    self.set_velocity(
                        -DRIVE_RPM,
                        +DRIVE_RPM,
                    )

                elif key == "a":
                    self.set_velocity(
                        -TURN_RPM,
                        -TURN_RPM,
                    )

                elif key == "d":
                    self.set_velocity(
                        +TURN_RPM,
                        +TURN_RPM,
                    )

                elif key in ("x", " "):
                    self.set_velocity(
                        0.0,
                        0.0,
                    )

                elif key == "q":
                    break

        finally:
            self.set_velocity(
                0.0,
                0.0,
            )

            for _ in range(5):
                self.send_velocity()
                time.sleep(0.02)

            self.running = False

            termios.tcsetattr(
                sys.stdin,
                termios.TCSADRAIN,
                original_terminal,
            )

            self.ser.close()

            print("\nStopped.")


if __name__ == "__main__":
    BaseSerialTest().run()
