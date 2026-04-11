#!/usr/bin/env python3
"""
Host script for SAME51 uart_dma_demo — drives the UART DMA receive demo.

MCU sends "READY\\n" when DMA is armed. We send 32-byte pattern (0x00..0x1F).
MCU verifies via kernel buffer pool and replies "OK\\n" or "ERR:...\\n".

Usage:
    python3 uart_dma_host.py --port /dev/ttyACM4 --cycles 50
"""

import sys
import time
import argparse
import serial

PATTERN = bytes(range(32))  # 0x00..0x1F


def run(port: str, baud: int, cycles: int, timeout: float) -> int:
    print(f"Opening {port} at {baud} bps...")
    ser = serial.Serial(port, baud, timeout=timeout)
    time.sleep(0.3)
    ser.reset_input_buffer()

    ok_count = 0
    err_count = 0
    cycle = 0
    t_start = time.monotonic()

    print(f"Sending {len(PATTERN)}-byte pattern per cycle. Ctrl-C to stop.")
    print(f"Pattern: {PATTERN.hex()}\n")

    try:
        while cycles == 0 or cycle < cycles:
            # Wait for "READY\n"
            line = ser.readline().decode("utf-8", errors="replace").strip()
            if not line:
                continue
            if line != "READY":
                if cycle == 0:
                    print(f"[{cycle:5}] unexpected: {line!r}")
                continue

            # Send pattern
            ser.write(PATTERN)
            ser.flush()

            # Read response
            resp = ser.readline().decode("utf-8", errors="replace").strip()
            if resp == "OK":
                ok_count += 1
            else:
                err_count += 1
                if err_count <= 5:
                    print(f"[{cycle:5}] ERR: {resp!r}")

            cycle += 1
            elapsed = time.monotonic() - t_start
            rate = cycle / elapsed if elapsed > 0 else 0

            if cycle % 20 == 0 or cycle == 1:
                print(f"[{cycle:5}] OK  (ok={ok_count} err={err_count} {rate:.1f} cyc/s)")

    except KeyboardInterrupt:
        print("\nInterrupted.")

    elapsed = time.monotonic() - t_start
    rate = cycle / elapsed if elapsed > 0 else 0
    print(f"\n--- Done ---")
    print(f"Cycles: {cycle}  OK: {ok_count}  ERR: {err_count}  "
          f"Elapsed: {elapsed:.1f}s  Rate: {rate:.1f} cyc/s")
    if err_count == 0 and cycle > 0:
        print("PASS: all cycles verified.")
    else:
        print(f"FAIL: {err_count} errors.")
    return err_count


def main():
    parser = argparse.ArgumentParser(description="SAME51 UART DMA demo host")
    parser.add_argument("--port", default="/dev/ttyACM4", help="Serial port")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate")
    parser.add_argument("--cycles", type=int, default=50, help="Cycles (0=forever)")
    parser.add_argument("--timeout", type=float, default=5.0, help="Per-op timeout")
    args = parser.parse_args()
    sys.exit(run(args.port, args.baud, args.cycles, args.timeout))


if __name__ == "__main__":
    main()
