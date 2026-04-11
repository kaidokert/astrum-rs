#!/usr/bin/env python3
"""Comprehensive virtual device UART echo test for STM32F429ZI.

Tests the kernel's Model C driver path: partition → SYS_DEV_READ/WRITE →
kernel dispatch → VirtualDevice backend → USART3 hardware.

Finds the ST-LINK VCP by VID:PID (0483:374b) automatically.

Usage:
    python3 vdev_uart_test.py                   # default 115200
    python3 vdev_uart_test.py --baud 921600     # high speed
    python3 vdev_uart_test.py --baud 460800 --bytes 2000
"""

import argparse
import serial
import serial.tools.list_ports
import sys
import time


def find_stlink_vcp():
    for p in serial.tools.list_ports.comports():
        if p.vid == 0x0483 and p.pid == 0x374B:
            return p.device
    return None


def test_single_bytes(s, count=256):
    """Send individual bytes and verify echo."""
    passed = failed = 0
    for i in range(count):
        s.write(bytes([i & 0xFF]))
        s.flush()
        time.sleep(0.003)
        echo = s.read(1)
        if len(echo) == 1 and echo[0] == (i & 0xFF):
            passed += 1
        else:
            failed += 1
            if failed <= 3:
                print(f"  FAIL byte {i}: got {echo.hex() if echo else 'nothing'}")
    return passed, count


def test_frames(s, max_size=32):
    """Send frames of varying sizes (1..max_size) and verify echo."""
    passed = failed = 0
    for size in range(1, max_size + 1):
        frame = bytes([(i * 7 + size) & 0xFF for i in range(size)])
        for b in frame:
            s.write(bytes([b]))
            s.flush()
            time.sleep(0.002)
        # Wait for kernel round-trip: ISR → ring → partition → sys_dev_write → TX
        time.sleep(0.1 + size * 0.005)
        echo = s.read(size)
        if echo == frame:
            passed += 1
        else:
            failed += 1
            if failed <= 3:
                print(f"  FAIL size={size}: sent {len(frame)}B, got {len(echo)}B")
    return passed, max_size


def test_sentences(s):
    """Send ASCII sentences of varying content and length."""
    sentences = [
        b"A",
        b"Hello",
        b"The quick brown fox",
        b"RTOS kernel 4KB on 20KB SRAM!",
        b"abcdefghijklmnopqrstuvwxyz",
        b"0123456789" * 3,
        b"\x00\x01\x02\x80\xfe\xff",
        b"Model C: partition never touches hardware registers",
    ]
    passed = 0
    for msg in sentences:
        for b in msg:
            s.write(bytes([b]))
            s.flush()
            time.sleep(0.002)
        time.sleep(0.1 + len(msg) * 0.005)
        echo = s.read(len(msg))
        if echo == msg:
            passed += 1
        else:
            print(f"  FAIL: sent {len(msg)}B, got {len(echo)}B")
    return passed, len(sentences)


def test_sustained(s, total_bytes=500, pace_ms=2.0):
    """Sustained stream at controlled pace."""
    sent = b""
    for i in range(total_bytes):
        b = bytes([(i * 13 + 7) & 0xFF])
        s.write(b)
        sent += b
        s.flush()
        time.sleep(pace_ms / 1000.0)

    # Read back with generous timeout
    echo = b""
    deadline = time.time() + 5
    while len(echo) < total_bytes and time.time() < deadline:
        chunk = s.read(total_bytes - len(echo))
        if chunk:
            echo += chunk
        else:
            time.sleep(0.1)

    match = sum(1 for a, b in zip(echo, sent) if a == b)
    return len(echo), total_bytes, match


def test_burst(s, frames=100, frame_size=8):
    """Rapid burst with no inter-frame delay."""
    expected = b""
    for i in range(frames):
        frame = bytes([(i + j) & 0xFF for j in range(frame_size)])
        s.write(frame)
        expected += frame
    s.flush()

    total = frames * frame_size
    echo = b""
    deadline = time.time() + 5
    while len(echo) < total and time.time() < deadline:
        chunk = s.read(total - len(echo))
        if chunk:
            echo += chunk
        else:
            time.sleep(0.1)

    match = sum(1 for a, b in zip(echo, expected) if a == b)
    return len(echo), total, match


def main():
    parser = argparse.ArgumentParser(description="vdev_uart echo test")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate")
    parser.add_argument("--bytes", type=int, default=500, help="Sustained test byte count")
    parser.add_argument("--pace-ms", type=float, default=2.0, help="Sustained test pace (ms/byte)")
    args = parser.parse_args()

    port = find_stlink_vcp()
    if not port:
        print("ERROR: ST-LINK VCP (0483:374b) not found")
        sys.exit(1)

    print(f"=== vdev_uart echo test ===")
    print(f"Port: {port}  Baud: {args.baud}\n")

    s = serial.Serial(port, args.baud, timeout=2)
    time.sleep(0.5)
    s.read(s.in_waiting)  # drain startup bytes

    # Test 1: Single bytes
    print("--- Test 1: Single bytes (0x00-0xFF) ---")
    p, t = test_single_bytes(s)
    print(f"  {p}/{t} matched")
    t1_ok = p == t

    # Test 2: Varying frame sizes
    print("\n--- Test 2: Varying frame sizes (1-32 bytes) ---")
    p, t = test_frames(s)
    print(f"  {p}/{t} matched")
    t2_ok = p == t

    # Test 3: ASCII sentences
    print("\n--- Test 3: ASCII sentences ---")
    p, t = test_sentences(s)
    print(f"  {p}/{t} matched")
    t3_ok = p == t

    # Test 4: Sustained stream (paced)
    print(f"\n--- Test 4: Sustained stream ({args.bytes} bytes @ {args.pace_ms}ms/byte) ---")
    got, total, match = test_sustained(s, args.bytes, args.pace_ms)
    print(f"  {got}/{total} bytes received, {match} content match")
    t4_ok = got == total and match == total

    # Test 5: Burst (no pacing — tests VCP bridge limits)
    print(f"\n--- Test 5: Burst (100 x 8B frames, no delay) ---")
    got, total, match = test_burst(s)
    print(f"  {got}/{total} bytes received, {match} content match")
    t5_info = f"{got}/{total}"

    s.close()

    # Summary
    tests = [
        ("Single bytes", t1_ok),
        ("Frame sizes", t2_ok),
        ("Sentences", t3_ok),
        ("Sustained", t4_ok),
    ]
    passed = sum(1 for _, ok in tests if ok)

    print(f"\n{'=' * 40}")
    for name, ok in tests:
        print(f"  {'PASS' if ok else 'FAIL'}  {name}")
    print(f"  INFO  Burst: {t5_info} (VCP bridge dependent)")
    print(f"\n  {passed}/{len(tests)} tests passed")
    print(f"{'=' * 40}")

    sys.exit(0 if passed == len(tests) else 1)


if __name__ == "__main__":
    main()
