#!/usr/bin/env python3
"""
Host script for uart_dma_demo — drives the STM32F429ZI UART DMA zero-copy demo.

The MCU sends "READY\\n" when its DMA stream is armed and waiting.
We reply with a 32-byte pattern (0x00..0x1F).
The MCU verifies via kernel sys_buf_read and replies "OK\\n" or "ERR:...\\n".

Usage:
    python3 uart_dma_host.py --port /dev/ttyACM0 --cycles 200
    python3 uart_dma_host.py --port /dev/ttyACM0 --cycles 0  # run forever
"""

import sys
import time
import argparse
import serial

PATTERN = bytes(range(32))   # 0x00..0x1F — matches EXPECTED in uart_dma_demo.rs
BUF_LEN = len(PATTERN)       # 32


def run(port: str, baud: int, cycles: int, timeout: float) -> int:
    """Returns number of errors encountered."""
    print(f"Opening {port} at {baud} bps...")
    ser = serial.Serial(port, baud, timeout=timeout)
    time.sleep(0.3)
    ser.reset_input_buffer()

    ok_count   = 0
    err_count  = 0
    cycle      = 0
    t_start    = time.monotonic()

    print(f"Sending {BUF_LEN}-byte pattern per cycle. Ctrl-C to stop.")
    print(f"Pattern: {PATTERN.hex()}\n")

    try:
        while cycles == 0 or cycle < cycles:
            # Wait for MCU to signal it's ready.
            line = ser.readline()
            if not line:
                print(f"[{cycle:5}] TIMEOUT waiting for READY — is firmware running?",
                      flush=True)
                err_count += 1
                if err_count > 5:
                    print("Too many timeouts — aborting.")
                    break
                continue

            line_s = line.decode("ascii", errors="replace").strip()

            if line_s != "READY":
                # Could be startup RTT noise forwarded through VCP, or an error
                # from a previous cycle. Print and keep waiting.
                print(f"[{cycle:5}] unexpected: {line_s!r}", flush=True)
                continue

            # MCU is armed — send the pattern immediately.
            ser.write(PATTERN)
            ser.flush()

            # Wait for ACK.
            ack = ser.readline()
            if not ack:
                print(f"[{cycle:5}] TIMEOUT waiting for ACK after sending pattern",
                      flush=True)
                err_count += 1
                cycle += 1
                continue

            ack_s = ack.decode("ascii", errors="replace").strip()

            elapsed = time.monotonic() - t_start
            if ack_s == "OK":
                ok_count += 1
                if cycle % 20 == 0:
                    rate = ok_count / elapsed if elapsed > 0 else 0
                    print(f"[{cycle:5}] OK  (ok={ok_count} err={err_count} "
                          f"{rate:.1f} cyc/s)", flush=True)
            else:
                err_count += 1
                print(f"[{cycle:5}] {ack_s}  (ok={ok_count} err={err_count})",
                      flush=True)

            cycle += 1

    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        ser.close()

    elapsed = time.monotonic() - t_start
    rate = ok_count / elapsed if elapsed > 0 else 0
    print(f"\n--- Done ---")
    print(f"Cycles: {cycle}  OK: {ok_count}  ERR: {err_count}  "
          f"Elapsed: {elapsed:.1f}s  Rate: {rate:.1f} cyc/s")
    if err_count == 0 and ok_count > 0:
        print("PASS: all cycles verified.")
    else:
        print("FAIL: errors detected." if err_count else "No cycles completed.")
    return err_count


def main():
    p = argparse.ArgumentParser(description="uart_dma_demo host driver")
    p.add_argument("--port", default="/dev/ttyACM0",
                   help="Serial port (default: /dev/ttyACM0)")
    p.add_argument("--baud", type=int, default=115200,
                   help="Baud rate (default: 115200)")
    p.add_argument("--cycles", type=int, default=100,
                   help="Number of cycles (0 = run forever, default: 100)")
    p.add_argument("--timeout", type=float, default=5.0,
                   help="Per-readline timeout seconds (default: 5.0)")
    args = p.parse_args()

    errs = run(args.port, args.baud, args.cycles, args.timeout)
    sys.exit(0 if errs == 0 else 1)


if __name__ == "__main__":
    main()
