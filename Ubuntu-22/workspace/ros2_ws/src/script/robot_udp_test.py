#!/usr/bin/env python3

from __future__ import annotations

import math
import socket
import struct
import time
import zlib
from dataclasses import dataclass, field
from typing import Optional


# ============================================================
# Configuration
# ============================================================

ESP32_IP = "192.168.4.107"

PC_RX_PORT = 8888
ESP32_COMMAND_PORT = 8889

PROTOCOL_MAGIC = 0x524F424F
PROTOCOL_VERSION = 1

ROBOT_STATE_TYPE = 1
LIDAR_FRAME_TYPE = 2
ROBOT_COMMAND_TYPE = 3

COMMAND_RATE_HZ = 20.0
PRINT_PERIOD_SECONDS = 1.0

COMMAND_AMPLITUDE_RAD_S = 1.0
COMMAND_FREQUENCY_HZ = 0.05


# ============================================================
# Binary protocol
# ============================================================

# uint32 magic
# uint8  version
# uint8  type
# uint16 payload_length
# uint32 sequence
# uint32 device_time_us
# uint32 crc32

HEADER_FORMAT = "<IBBHIII"
HEADER_SIZE = struct.calcsize(HEADER_FORMAT)

STATE_FORMAT = "<iiffhhHHBBHII"
STATE_SIZE = struct.calcsize(STATE_FORMAT)

assert STATE_SIZE == 36

COMMAND_FORMAT = "<ffBBHI"
COMMAND_SIZE = struct.calcsize(COMMAND_FORMAT)

LIDAR_PREFIX_FORMAT = "<IB3x"
LIDAR_PREFIX_SIZE = struct.calcsize(LIDAR_PREFIX_FORMAT)

LIDAR_POINT_FORMAT = "<HHB"
LIDAR_POINT_SIZE = struct.calcsize(LIDAR_POINT_FORMAT)

LIDAR_POINTS_PER_PACKET = 12
LIDAR_PAYLOAD_SIZE = (
    LIDAR_PREFIX_SIZE
    + LIDAR_POINT_SIZE * LIDAR_POINTS_PER_PACKET
)


assert HEADER_SIZE == 20
assert STATE_SIZE == 36
assert COMMAND_SIZE == 16
assert LIDAR_POINT_SIZE == 5
assert LIDAR_PAYLOAD_SIZE == 68


# ============================================================
# Sequence tracking
# ============================================================

@dataclass
class SequenceTracker:
    name: str

    received: int = 0
    lost: int = 0
    duplicate: int = 0
    out_of_order: int = 0

    last_sequence: Optional[int] = None

    def update(self, sequence: int) -> None:
        self.received += 1

        if self.last_sequence is None:
            self.last_sequence = sequence
            return

        if sequence == self.last_sequence:
            self.duplicate += 1
            return

        if sequence > self.last_sequence:
            gap = sequence - self.last_sequence - 1

            if gap > 0:
                self.lost += gap

            self.last_sequence = sequence
            return

        self.out_of_order += 1

    @property
    def expected(self) -> int:
        return self.received + self.lost

    @property
    def loss_percent(self) -> float:
        if self.expected == 0:
            return 0.0

        return 100.0 * self.lost / self.expected


# ============================================================
# Timing statistics
# ============================================================

@dataclass
class TimingStatistics:
    samples_ms: list[float] = field(default_factory=list)

    def add(self, value_ms: float) -> None:
        self.samples_ms.append(value_ms)

        # Keep the most recent 10,000 samples.
        if len(self.samples_ms) > 10_000:
            del self.samples_ms[:1_000]

    def clear(self) -> None:
        self.samples_ms.clear()

    def summary(self) -> str:
        if not self.samples_ms:
            return "n/a"

        values = sorted(self.samples_ms)
        count = len(values)

        average = sum(values) / count
        minimum = values[0]
        maximum = values[-1]

        p50 = values[int(0.50 * (count - 1))]
        p95 = values[int(0.95 * (count - 1))]
        p99 = values[int(0.99 * (count - 1))]

        return (
            f"avg={average:.2f} ms "
            f"min={minimum:.2f} "
            f"p50={p50:.2f} "
            f"p95={p95:.2f} "
            f"p99={p99:.2f} "
            f"max={maximum:.2f}"
        )


# ============================================================
# Packet build and validation
# ============================================================

def calculate_crc(
    header_with_zero_crc: bytes,
    payload: bytes,
) -> int:
    crc = zlib.crc32(header_with_zero_crc)
    crc = zlib.crc32(payload, crc)

    return crc & 0xFFFFFFFF


def build_packet(
    packet_type: int,
    sequence: int,
    payload: bytes,
) -> bytes:
    host_time_us = (
        time.monotonic_ns() // 1_000
    ) & 0xFFFFFFFF

    header_with_zero_crc = struct.pack(
        HEADER_FORMAT,
        PROTOCOL_MAGIC,
        PROTOCOL_VERSION,
        packet_type,
        len(payload),
        sequence & 0xFFFFFFFF,
        host_time_us,
        0,
    )

    crc = calculate_crc(
        header_with_zero_crc,
        payload,
    )

    header = struct.pack(
        HEADER_FORMAT,
        PROTOCOL_MAGIC,
        PROTOCOL_VERSION,
        packet_type,
        len(payload),
        sequence & 0xFFFFFFFF,
        host_time_us,
        crc,
    )

    return header + payload


def parse_packet(
    packet: bytes,
) -> tuple[int, int, int, bytes]:
    if len(packet) < HEADER_SIZE:
        raise ValueError("packet shorter than header")

    (
        magic,
        version,
        packet_type,
        payload_length,
        sequence,
        device_time_us,
        received_crc,
    ) = struct.unpack_from(
        HEADER_FORMAT,
        packet,
        0,
    )

    if magic != PROTOCOL_MAGIC:
        raise ValueError(
            f"invalid magic 0x{magic:08X}"
        )

    if version != PROTOCOL_VERSION:
        raise ValueError(
            f"unsupported version {version}"
        )

    expected_size = HEADER_SIZE + payload_length

    if len(packet) != expected_size:
        raise ValueError(
            f"length mismatch: "
            f"packet={len(packet)}, "
            f"expected={expected_size}"
        )

    payload = packet[HEADER_SIZE:]

    header_with_zero_crc = struct.pack(
        HEADER_FORMAT,
        magic,
        version,
        packet_type,
        payload_length,
        sequence,
        device_time_us,
        0,
    )

    calculated_crc = calculate_crc(
        header_with_zero_crc,
        payload,
    )

    if calculated_crc != received_crc:
        raise ValueError(
            f"CRC mismatch: "
            f"received=0x{received_crc:08X}, "
            f"calculated=0x{calculated_crc:08X}"
        )

    return (
        packet_type,
        sequence,
        device_time_us,
        payload,
    )


# ============================================================
# Payload decoders
# ============================================================

@dataclass
class RobotState:
    left_position_ticks: int
    right_position_ticks: int

    left_velocity_rad_s: float
    right_velocity_rad_s: float

    left_load_raw: int
    right_load_raw: int

    battery_millivolts: int
    control_loop_hz: int

    left_temperature_c: int
    right_temperature_c: int

    reserved: int

    status_flags: int
    last_command_sequence: int


def decode_robot_state(
    payload: bytes,
) -> RobotState:
    if len(payload) != STATE_SIZE:
        raise ValueError(
            f"invalid state payload length {len(payload)}"
        )

    values = struct.unpack(
        STATE_FORMAT,
        payload,
    )

    return RobotState(*values)


@dataclass
class LidarPoint:
    angle_cdeg: int
    distance_mm: int
    intensity: int


@dataclass
class LidarFrame:
    scan_id: int
    points: list[LidarPoint]


def decode_lidar_frame(
    payload: bytes,
) -> LidarFrame:
    if len(payload) != LIDAR_PAYLOAD_SIZE:
        raise ValueError(
            f"invalid lidar payload length {len(payload)}"
        )

    scan_id, point_count = struct.unpack_from(
        LIDAR_PREFIX_FORMAT,
        payload,
        0,
    )

    if point_count > LIDAR_POINTS_PER_PACKET:
        raise ValueError(
            f"invalid lidar point count {point_count}"
        )

    points: list[LidarPoint] = []

    offset = LIDAR_PREFIX_SIZE

    for _ in range(point_count):
        angle_cdeg, distance_mm, intensity = (
            struct.unpack_from(
                LIDAR_POINT_FORMAT,
                payload,
                offset,
            )
        )

        points.append(
            LidarPoint(
                angle_cdeg=angle_cdeg,
                distance_mm=distance_mm,
                intensity=intensity,
            )
        )

        offset += LIDAR_POINT_SIZE

    return LidarFrame(
        scan_id=scan_id,
        points=points,
    )


# ============================================================
# Main tester
# ============================================================

def main() -> None:
    sock = socket.socket(
        socket.AF_INET,
        socket.SOCK_DGRAM,
    )

    sock.setsockopt(
        socket.SOL_SOCKET,
        socket.SO_RCVBUF,
        4 * 1024 * 1024,
    )

    sock.bind(
        ("0.0.0.0", PC_RX_PORT)
    )

    sock.setblocking(False)

    print(
        f"Listening on UDP 0.0.0.0:{PC_RX_PORT}"
    )

    print(
        f"Sending commands to "
        f"{ESP32_IP}:{ESP32_COMMAND_PORT}"
    )

    state_tracker = SequenceTracker(
        "robot_state"
    )

    lidar_tracker = SequenceTracker(
        "lidar"
    )

    rtt_statistics = TimingStatistics()

    command_sequence = 0

    # command sequence -> send monotonic timestamp
    command_send_times: dict[int, int] = {}

    start_time_ns = time.monotonic_ns()
    last_command_send_ns = start_time_ns
    last_print_ns = start_time_ns

    interval_packets = 0
    interval_bytes = 0
    interval_state_packets = 0
    interval_lidar_packets = 0
    invalid_packets = 0
    foreign_packets = 0

    latest_state: Optional[RobotState] = None
    latest_lidar: Optional[LidarFrame] = None

    command_period_ns = int(
        1_000_000_000 / COMMAND_RATE_HZ
    )

    while True:
        now_ns = time.monotonic_ns()

        # ----------------------------------------------------
        # Send motor command
        # ----------------------------------------------------

        if (
            now_ns - last_command_send_ns
            >= command_period_ns
        ):
            elapsed_s = (
                now_ns - start_time_ns
            ) / 1_000_000_000.0

            velocity = (
                COMMAND_AMPLITUDE_RAD_S
                * math.sin(
                    2.0
                    * math.pi
                    * COMMAND_FREQUENCY_HZ
                    * elapsed_s
                )
            )

            # Differential drive test:
            # both wheels receive the same velocity.
            left_velocity = velocity
            right_velocity = velocity

            payload = struct.pack(
                COMMAND_FORMAT,
                left_velocity,
                right_velocity,
                1,   # motor_enable
                0,   # emergency_stop
                0,   # reserved
                command_sequence,
            )

            packet = build_packet(
                ROBOT_COMMAND_TYPE,
                command_sequence,
                payload,
            )

            sock.sendto(
                packet,
                (
                    ESP32_IP,
                    ESP32_COMMAND_PORT,
                ),
            )

            command_send_times[
                command_sequence
            ] = now_ns

            # Keep dictionary bounded.
            if len(command_send_times) > 2_000:
                oldest_sequences = sorted(
                    command_send_times.keys()
                )[:1_000]

                for old_sequence in oldest_sequences:
                    command_send_times.pop(
                        old_sequence,
                        None,
                    )

            command_sequence = (
                command_sequence + 1
            ) & 0xFFFFFFFF

            last_command_send_ns += (
                command_period_ns
            )

        # ----------------------------------------------------
        # Receive all currently queued packets
        # ----------------------------------------------------

        while True:
            try:
                packet, address = sock.recvfrom(
                    65535
                )
            except BlockingIOError:
                break

            receive_time_ns = time.monotonic_ns()

            interval_packets += 1
            interval_bytes += len(packet)

            if address[0] != ESP32_IP:
                foreign_packets += 1
                continue

            try:
                (
                    packet_type,
                    sequence,
                    device_time_us,
                    payload,
                ) = parse_packet(packet)

                if packet_type == ROBOT_STATE_TYPE:
                    state_tracker.update(sequence)
                    interval_state_packets += 1

                    state = decode_robot_state(
                        payload
                    )

                    latest_state = state

                    echoed_command_sequence = (
                        state.last_command_sequence
                    )

                    send_time_ns = (
                        command_send_times.pop(
                            echoed_command_sequence,
                            None,
                        )
                    )

                    if send_time_ns is not None:
                        rtt_ms = (
                            receive_time_ns
                            - send_time_ns
                        ) / 1_000_000.0

                        rtt_statistics.add(rtt_ms)

                elif packet_type == LIDAR_FRAME_TYPE:
                    lidar_tracker.update(sequence)
                    interval_lidar_packets += 1

                    latest_lidar = (
                        decode_lidar_frame(payload)
                    )

                else:
                    invalid_packets += 1

            except (
                ValueError,
                struct.error,
            ) as error:
                invalid_packets += 1

                print(
                    f"Invalid packet from "
                    f"{address}: {error}"
                )

        # ----------------------------------------------------
        # Print statistics once per second
        # ----------------------------------------------------

        if (
            now_ns - last_print_ns
            >= int(
                PRINT_PERIOD_SECONDS
                * 1_000_000_000
            )
        ):
            elapsed_interval_s = (
                now_ns - last_print_ns
            ) / 1_000_000_000.0

            packet_rate = (
                interval_packets
                / elapsed_interval_s
            )

            state_rate = (
                interval_state_packets
                / elapsed_interval_s
            )

            lidar_rate = (
                interval_lidar_packets
                / elapsed_interval_s
            )

            bandwidth_kib_s = (
                interval_bytes
                / elapsed_interval_s
                / 1024.0
            )

            print()
            print(
                "================ UDP ROBOT TEST ================"
            )

            print(
                f"RX total rate : "
                f"{packet_rate:8.1f} packets/s"
            )

            print(
                f"State rate    : "
                f"{state_rate:8.1f} packets/s"
            )

            print(
                f"Lidar rate    : "
                f"{lidar_rate:8.1f} packets/s"
            )

            print(
                f"Bandwidth     : "
                f"{bandwidth_kib_s:8.2f} KiB/s"
            )

            print(
                f"State loss    : "
                f"{state_tracker.lost} lost, "
                f"{state_tracker.loss_percent:.6f}%"
            )

            print(
                f"Lidar loss    : "
                f"{lidar_tracker.lost} lost, "
                f"{lidar_tracker.loss_percent:.6f}%"
            )

            print(
                f"Duplicates    : "
                f"state={state_tracker.duplicate}, "
                f"lidar={lidar_tracker.duplicate}"
            )

            print(
                f"Out of order  : "
                f"state={state_tracker.out_of_order}, "
                f"lidar={lidar_tracker.out_of_order}"
            )

            print(
                f"Invalid/CRC   : "
                f"{invalid_packets}"
            )

            print(
                f"Foreign sender: "
                f"{foreign_packets}"
            )

            print(
                f"Command RTT   : "
                f"{rtt_statistics.summary()}"
            )

            if latest_state is not None:
                print(
                    "Robot state   : "
                    f"pos=("
                    f"{latest_state.left_position_ticks}, "
                    f"{latest_state.right_position_ticks}"
                    f") "
                    f"vel=("
                    f"{latest_state.left_velocity_rad_s:.3f}, "
                    f"{latest_state.right_velocity_rad_s:.3f}"
                    f") "
                    f"battery="
                    f"{latest_state.battery_millivolts / 1000.0:.2f} V "
                    f"status="
                    f"0x{latest_state.status_flags:08X}"
                )

            if (
                latest_lidar is not None
                and latest_lidar.points
            ):
                first_point = latest_lidar.points[0]

                print(
                    "Lidar sample  : "
                    f"scan={latest_lidar.scan_id} "
                    f"points={len(latest_lidar.points)} "
                    f"angle={first_point.angle_cdeg / 100.0:.2f} deg "
                    f"distance={first_point.distance_mm} mm "
                    f"intensity={first_point.intensity}"
                )

            print(
                "================================================"
            )

            interval_packets = 0
            interval_bytes = 0
            interval_state_packets = 0
            interval_lidar_packets = 0

            rtt_statistics.clear()

            last_print_ns = now_ns

        # Prevent a continuous 100% CPU busy loop.
        time.sleep(0.0005)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nStopped")