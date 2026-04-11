#!/usr/bin/env python3

import argparse
import glob
import os
import random
import sys
import time

try:
    import serial
except ImportError as exc:
    raise SystemExit("pyserial is required: pip install pyserial") from exc


def autodetect_port() -> str:
    candidates = sorted(glob.glob("/dev/cu.usbmodem*"))
    if not candidates:
        raise SystemExit("No /dev/cu.usbmodem* device found")
    return candidates[-1]


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default=os.environ.get("SERIAL_PORT"))
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--timeout", type=float, default=3.0)
    args = parser.parse_args()

    port = args.port or autodetect_port()
    token = f"ping-{random.randrange(1_000_000):06d}\n".encode()

    with serial.Serial(port, args.baud, timeout=0.1) as ser:
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        ser.write(token)
        ser.flush()

        deadline = time.time() + args.timeout
        received = bytearray()
        while time.time() < deadline:
            chunk = ser.read(64)
            if chunk:
                received.extend(chunk)
                if token in received:
                    print(f"PASS {port}: echoed {token.decode().strip()}")
                    return 0
            else:
                time.sleep(0.01)

    print(f"FAIL {port}: did not receive expected echo {token!r}", file=sys.stderr)
    if received:
        print(f"received={received!r}", file=sys.stderr)
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
