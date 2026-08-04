#!/usr/bin/env python3

import time
import serial


PORT = "/dev/ttyS6"
BAUD = 230400
TEST_SECONDS = 5.0


def main() -> None:
    print("LD06 RAW UART TEST")
    print(f"Port: {PORT}")
    print(f"Baud: {BAUD}")
    print()

    ser = serial.Serial(
        port=PORT,
        baudrate=BAUD,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=0.2,
    )

    ser.reset_input_buffer()

    start = time.monotonic()

    total_bytes = 0
    header_count = 0

    print("Receiving...")
    print()

    try:
        while (
            time.monotonic() - start
            < TEST_SECONDS
        ):
            data = ser.read(256)

            if not data:
                continue

            total_bytes += len(data)

            header_count += data.count(
                b"\x54"
            )

            hex_text = " ".join(
                f"{value:02X}"
                for value in data[:32]
            )

            print(
                f"RX {len(data):3d} bytes | "
                f"{hex_text}"
            )

    finally:
        ser.close()

    print()
    print("==============================")
    print(f"Total bytes : {total_bytes}")
    print(f"0x54 count  : {header_count}")
    print("==============================")

    if total_bytes == 0:
        print("RESULT: NO UART DATA")
    elif header_count == 0:
        print(
            "RESULT: DATA FOUND, "
            "but no LD06 0x54 header"
        )
    else:
        print("RESULT: LD06 DATA FOUND")


if __name__ == "__main__":
    main()
