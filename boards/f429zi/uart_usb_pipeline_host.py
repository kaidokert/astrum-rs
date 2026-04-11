#!/usr/bin/env python3
"""
UART→USB CDC Pipeline host driver — uart_usb_pipeline.rs (STM32F429ZI)

Exercises the two-partition pipeline:
  1. Wait for "READY\\n" on the UART port (ttyACM0) — P0 has armed DMA.
  2. Send 32-byte pattern (bytes 0x00..0x1F) to the UART port.
  3. Read 32 bytes from the USB CDC port (ttyACM1) — P1 forwarded the buffer.
  4. Verify the received bytes match the sent pattern.
  5. Report ok/fail per cycle and running totals.

Usage:
  python3 f429zi/uart_usb_pipeline_host.py [options]

Options:
  --uart PORT     UART port (default: /dev/ttyACM0)
  --usb  PORT     USB CDC port (default: /dev/ttyACM1)
  --baud RATE     UART baud rate (default: 115200)
  --cycles N      Number of cycles to run (default: 50)
  --timeout T     Per-operation timeout in seconds (default: 5.0)
"""

import argparse
import sys
import time
import serial


PATTERN = bytes(range(32))   # 0x00..0x1F — matches EXPECTED in the MCU firmware


def main() -> None:
    ap = argparse.ArgumentParser(description="uart_usb_pipeline_host.py")
    ap.add_argument("--uart",    default="/dev/ttyACM0", help="UART port (ST-LINK VCP)")
    ap.add_argument("--usb",     default="/dev/ttyACM1", help="USB CDC port")
    ap.add_argument("--baud",    type=int, default=115_200, help="UART baud rate")
    ap.add_argument("--cycles",  type=int, default=50, help="Cycles to run")
    ap.add_argument("--timeout", type=float, default=5.0, help="Per-op timeout (s)")
    args = ap.parse_args()

    print(f"=== UART→USB CDC Pipeline Host ===")
    print(f"UART:    {args.uart} @ {args.baud} bps")
    print(f"USB CDC: {args.usb}")
    print(f"Pattern: {PATTERN.hex()} ({len(PATTERN)} bytes)")
    print(f"Cycles:  {args.cycles}")
    print()

    uart = serial.Serial(args.uart, args.baud, timeout=args.timeout)
    usb  = serial.Serial(args.usb,  115_200,   timeout=args.timeout)

    # Flush any stale bytes from a previous run.
    uart.reset_input_buffer()
    usb.reset_input_buffer()

    ok = 0
    fail = 0
    t_start = time.monotonic()

    for cycle in range(1, args.cycles + 1):
        # ── Step 1: wait for "READY\n" on UART ──────────────────────────────
        deadline = time.monotonic() + args.timeout
        while True:
            if time.monotonic() > deadline:
                print(f"[{cycle:3d}] TIMEOUT waiting for READY")
                uart.reset_input_buffer()
                usb.reset_input_buffer()
                fail += 1
                break
            line = uart.readline()
            if not line:
                continue
            line_str = line.decode(errors="replace").strip()
            if "READY" in line_str:
                break
            # Unexpected line (e.g. RTT log on wrong port, stale data) — ignore.
            print(f"        [uart] unexpected: {line_str!r}")
        else:
            # READY received — fall through.
            pass

        if "READY" not in (line.decode(errors="replace") if line else ""):
            continue  # timed out above

        # ── Step 2: send pattern on UART ────────────────────────────────────
        uart.write(PATTERN)
        uart.flush()

        # ── Step 3: read 32 bytes from USB CDC ──────────────────────────────
        got = b""
        deadline = time.monotonic() + args.timeout
        while len(got) < len(PATTERN):
            if time.monotonic() > deadline:
                break
            chunk = usb.read(len(PATTERN) - len(got))
            if chunk:
                got += chunk

        # ── Step 4: verify ───────────────────────────────────────────────────
        if got == PATTERN:
            ok += 1
            rate = ok / (time.monotonic() - t_start)
            print(f"[{cycle:3d}] OK   got={got.hex()[:16]}...  ok={ok} fail={fail}  {rate:.1f} cyc/s")
        else:
            fail += 1
            if len(got) < len(PATTERN):
                print(f"[{cycle:3d}] FAIL short: got {len(got)}/{len(PATTERN)} bytes")
            else:
                first_bad = next(
                    (i for i, (g, e) in enumerate(zip(got, PATTERN)) if g != e),
                    None,
                )
                print(f"[{cycle:3d}] FAIL byte[{first_bad}] exp=0x{PATTERN[first_bad]:02x} "
                      f"got=0x{got[first_bad]:02x}  ok={ok} fail={fail}")

    elapsed = time.monotonic() - t_start
    print()
    print(f"=== Done: {args.cycles} cycles in {elapsed:.1f}s ===")
    print(f"OK={ok}  FAIL={fail}  rate={ok/elapsed:.1f} cyc/s")
    if fail == 0 and ok == args.cycles:
        print("PASS: all cycles verified.")
    else:
        print("FAIL: some cycles did not verify.")
        sys.exit(1)

    uart.close()
    usb.close()


if __name__ == "__main__":
    main()
