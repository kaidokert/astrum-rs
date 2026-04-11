#!/usr/bin/env python3
"""Persistent SAME51 VCOM test fixture.

Usage:
  python3 scripts/vcom_fixture.py --port /dev/cu.usbmodem21302 --write A
  python3 scripts/vcom_fixture.py --port /dev/cu.usbmodem21302 --write-line ping
  python3 scripts/vcom_fixture.py --port /dev/cu.usbmodem21302 --interactive
"""

from __future__ import annotations

import argparse
import sys
import time

try:
    import serial
except ModuleNotFoundError as exc:  # pragma: no cover - depends on host env
    raise SystemExit(
        "pyserial is required for this fixture. Install it in your Python env with: "
        "python3 -m pip install pyserial"
    ) from exc


def hexdump(data: bytes) -> str:
    return " ".join(f"{byte:02x}" for byte in data)


def read_available(port: serial.Serial, duration: float) -> bytes:
    deadline = time.monotonic() + duration
    out = bytearray()
    while time.monotonic() < deadline:
        chunk = port.read(port.in_waiting or 1)
        if chunk:
            out.extend(chunk)
        else:
            time.sleep(0.01)
    return bytes(out)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", required=True)
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--settle", type=float, default=0.5)
    parser.add_argument("--read-time", type=float, default=0.5)
    parser.add_argument("--write")
    parser.add_argument("--write-line")
    parser.add_argument("--interactive", action="store_true")
    args = parser.parse_args()

    if args.write and args.write_line:
        parser.error("--write and --write-line are mutually exclusive")

    with serial.Serial(args.port, args.baud, timeout=0.1, write_timeout=1) as port:
        print(f"opened {args.port} @ {args.baud}")
        time.sleep(args.settle)

        boot = read_available(port, args.read_time)
        if boot:
            print(f"boot ascii: {boot.decode(errors='replace')!r}")
            print(f"boot hex:   {hexdump(boot)}")

        if args.write is not None:
            payload = args.write.encode("utf-8")
            port.write(payload)
            port.flush()
            time.sleep(0.05)
            reply = read_available(port, args.read_time)
            print(f"tx hex: {hexdump(payload)}")
            print(f"rx ascii: {reply.decode(errors='replace')!r}")
            print(f"rx hex:   {hexdump(reply)}")

        if args.write_line is not None:
            payload = (args.write_line + "\n").encode("utf-8")
            port.write(payload)
            port.flush()
            time.sleep(0.05)
            reply = read_available(port, args.read_time)
            print(f"tx hex: {hexdump(payload)}")
            print(f"rx ascii: {reply.decode(errors='replace')!r}")
            print(f"rx hex:   {hexdump(reply)}")

        if args.interactive:
            print("interactive mode; Ctrl-D to exit")
            while True:
                try:
                    line = input("> ")
                except EOFError:
                    break
                payload = (line + "\n").encode("utf-8")
                port.write(payload)
                port.flush()
                time.sleep(0.05)
                reply = read_available(port, args.read_time)
                print(f"rx ascii: {reply.decode(errors='replace')!r}")
                print(f"rx hex:   {hexdump(reply)}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
