#!/usr/bin/env python3

import time
import serial


PORT = "/dev/ttyS6"
BAUD = 230400

TEST_COUNT = 20
TIMEOUT_S = 0.5


def main() -> None:
    print("UART6 LOOPBACK TEST")
    print(f"Port: {PORT}")
    print(f"Baud: {BAUD}")
    print("Expected wiring:")
    print("  Pin 19 UART6_TX -> Pin 21 UART6_RX")
    print()

    ser = serial.Serial(
        port=PORT,
        baudrate=BAUD,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=TIMEOUT_S,
        write_timeout=TIMEOUT_S,
    )

    # Clear any stale bytes.
    ser.reset_input_buffer()
    ser.reset_output_buffer()

    passed = 0
    failed = 0

    try:
        for index in range(TEST_COUNT):
            message = (
                f"UART6_LOOPBACK_{index:03d}\r\n"
            ).encode("ascii")

            # Clear previous RX bytes.
            ser.reset_input_buffer()

            # Send through UART6 TX.
            ser.write(message)
            ser.flush()

            # Small time for physical loopback.
            time.sleep(0.01)

            # Read exactly what we sent.
            received = ser.read(
                len(message)
            )

            if received == message:
                passed += 1

                print(
                    f"[PASS {index:02d}] "
                    f"{received.decode('ascii').strip()}"
                )
            else:
                failed += 1

                print(
                    f"[FAIL {index:02d}]"
                )

                print(
                    f"  TX: {message!r}"
                )

                print(
                    f"  RX: {received!r}"
                )

            time.sleep(0.05)

    finally:
        ser.close()

    print()
    print("==============================")
    print(f"PASS: {passed}")
    print(f"FAIL: {failed}")
    print("==============================")

    if failed == 0:
        print("UART6 LOOPBACK: OK")
    else:
        print("UART6 LOOPBACK: FAILED")


if __name__ == "__main__":
    main()