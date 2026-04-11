#!/usr/bin/env python3
"""
uart_dma_echo_host.py — host driver for uart_dma_echo STM32 demo.

Protocol per cycle:
  MCU sends  "READY\n"       (CPU TX)
  Host sends 0x00..0x1F      (32 bytes)
  MCU echoes 0x00..0x1F      (DMA TX — same kernel buffer, no CPU in data path)
  MCU sends  "OK\n"          (CPU TX, cycle complete)
  Host verifies echo == sent
"""

import argparse
import serial
import time

PATTERN = bytes(range(32))   # 0x00..0x1F

def run(port: str, baud: int, cycles: int, timeout: float) -> bool:
    print(f"=== UART Full-Duplex DMA Echo Host ===")
    print(f"Port: {port} @ {baud} bps")
    print(f"Pattern: {PATTERN.hex()} ({len(PATTERN)} bytes)")
    print(f"Cycles: {cycles}\n")

    ok = fail = 0
    t0 = time.monotonic()

    with serial.Serial(port, baud, timeout=timeout) as ser:
        ser.reset_input_buffer()
        ser.reset_output_buffer()

        for cyc in range(1, cycles + 1):
            # Drain-to-READY: the MCU may be sending TIMEOUT\n messages while
            # idle (no host), or may be mid-cycle. Keep reading until we get
            # a clean "READY" line.  Pattern byte 0x0a is a newline so we must
            # NOT use readline() to consume the echo — we use read(N) below.
            while True:
                raw = ser.readline()
                line = raw.decode(errors='replace').strip()
                if line == 'READY':
                    # Allow any bytes that are still in the UART pipeline to arrive
                    # before flushing (UART shift-register + tty kernel buffer latency).
                    # Without this sleep, a TIMEOUT message being transmitted concurrently
                    # with READY can slip through: the first few bytes are already in the
                    # tty buffer (flushed) but the tail bytes arrive after reset_input_buffer().
                    time.sleep(0.015)
                    # Flush anything that arrived since READY (stale TIMEOUT fragments,
                    # echo bytes from previous failed cycles, etc.).  Safe: MCU is in
                    # poll_tcif() and has not armed DMA TX yet, so no echo bytes can be
                    # in-flight between READY and our first PATTERN write.
                    ser.reset_input_buffer()
                    break
                if not raw:
                    # Timeout — MCU not responding
                    print(f"[{cyc:4}] timeout waiting for READY")
                    fail += 1
                    break
                # Skip TIMEOUT, partial lines, stale echo bytes, etc.
            else:
                continue  # inner while exhausted without break → shouldn't happen

            if line != 'READY':
                continue

            # Send 32-byte pattern.
            ser.write(PATTERN)

            # Read 32-byte echo from DMA TX.
            echo = ser.read(len(PATTERN))
            if len(echo) < len(PATTERN):
                print(f"[{cyc:4}] FAIL short echo: got {len(echo)}/{len(PATTERN)} bytes")
                ser.reset_input_buffer()
                fail += 1
                continue

            # Read "OK\n" from CPU TX.
            ack = ser.readline().decode(errors='replace').strip()
            if ack != 'OK':
                print(f"[{cyc:4}] FAIL bad ack: {ack!r}")
                fail += 1
                continue

            # Verify echo.
            if echo != PATTERN:
                first_bad = next(
                    (i for i, (g, e) in enumerate(zip(echo, PATTERN)) if g != e),
                    len(PATTERN)
                )
                print(f"[{cyc:4}] FAIL mismatch at byte {first_bad}: "
                      f"got 0x{echo[first_bad]:02x} expected 0x{PATTERN[first_bad]:02x}")
                fail += 1
                continue

            ok += 1
            elapsed = time.monotonic() - t0
            rate = ok / elapsed if elapsed > 0 else 0
            print(f"[{cyc:4}] OK   echo={echo[:8].hex()}...  ok={ok} fail={fail}  {rate:.1f} cyc/s")

    elapsed = time.monotonic() - t0
    rate = ok / elapsed if elapsed > 0 else 0
    print(f"\n=== Done: {cycles} cycles in {elapsed:.1f}s ===")
    print(f"OK={ok}  FAIL={fail}  rate={rate:.1f} cyc/s")
    if fail == 0:
        print("PASS: all cycles verified.")
        return True
    else:
        print("FAIL: some cycles did not verify.")
        return False

def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--port",    default="/dev/ttyACM0", help="Serial port")
    ap.add_argument("--baud",    type=int, default=115200, help="Baud rate")
    ap.add_argument("--cycles",  type=int, default=50,    help="Cycles to run")
    ap.add_argument("--timeout", type=float, default=5.0, help="Per-op timeout (s)")
    args = ap.parse_args()
    ok = run(args.port, args.baud, args.cycles, args.timeout)
    raise SystemExit(0 if ok else 1)

if __name__ == "__main__":
    main()
