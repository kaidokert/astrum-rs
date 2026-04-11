#!/usr/bin/env python3
"""
Echo server for uart_irq_partition_echo demo.

Receives bytes from the STM32F429ZI USART3 and echoes them back.
Handles the baud-rate sweep: when 0xFF is received, waits briefly
then switches to the next rate in the sweep table.

Wiring:
  STM32 PD8 (TX) → adapter RX
  STM32 PD9 (RX) → adapter TX
  GND             → adapter GND

Usage:
  python3 host_serial_echo.py --port /dev/ttyUSB0
  python3 host_serial_echo.py --port /dev/ttyUSB0 --stop-at 460800
"""

import sys
import time
import argparse
import serial
import serial.tools.list_ports

BAUD_TABLE = [9600, 115200, 230400, 460800, 921600, 1_000_000, 2_000_000]
RATE_SWITCH_SIGNAL = 0xFF


def find_adapter():
    """Try to find a USB-UART adapter automatically."""
    for p in serial.tools.list_ports.comports():
        desc = (p.description or "").lower()
        if any(k in desc for k in ("cp210", "ftdi", "ch340", "pl230", "uart", "usb serial")):
            return p.device
    return None


def echo_loop(port: str, stop_at):
    rate_idx = 0
    baud = BAUD_TABLE[rate_idx]

    ser = serial.Serial(port, baud, timeout=0.5)
    print(f"[host] opened {port} @ {baud} bps")
    time.sleep(0.3)
    ser.reset_input_buffer()

    rx_total = 0
    tx_total = 0
    err_total = 0
    rate_rx = 0

    try:
        while True:
            raw = ser.read(1)
            if not raw:
                continue

            b = raw[0]
            rx_total += 1
            rate_rx += 1

            if b == RATE_SWITCH_SIGNAL:
                # Device is about to switch baud rate.
                # Flush any bytes that arrived in the same burst,
                # wait for the device to complete its delay, then switch.
                next_idx = rate_idx + 1
                if next_idx >= len(BAUD_TABLE):
                    print(f"[host] sweep complete — final rx={rx_total} tx={tx_total} err={err_total}")
                    break

                next_baud = BAUD_TABLE[next_idx]
                if stop_at is not None and next_baud > stop_at:
                    print(f"[host] stopping before {next_baud} bps (--stop-at {stop_at})")
                    break

                print(f"[host] rate {baud} done (rx={rate_rx}) → switching to {next_baud}")
                time.sleep(0.08)   # give device time to change BRR
                ser.reset_input_buffer()
                ser.baudrate = next_baud
                rate_idx = next_idx
                baud = next_baud
                rate_rx = 0
                print(f"[host] now @ {baud} bps")
                continue

            # Regular byte — echo back.
            ser.write(bytes([b]))
            tx_total += 1

            if tx_total % 50 == 0:
                print(f"[host] baud={baud} rx={rx_total} tx={tx_total} err={err_total}")

    except KeyboardInterrupt:
        print(f"\n[host] interrupted — rx={rx_total} tx={tx_total} err={err_total}")
    finally:
        ser.close()


def main():
    parser = argparse.ArgumentParser(description="Echo server for uart_irq_partition_echo")
    parser.add_argument("--port", "-p", help="Serial port (e.g. /dev/ttyUSB0)", default=None)
    parser.add_argument(
        "--stop-at",
        type=int,
        default=None,
        metavar="BAUD",
        help="Stop sweep before this baud rate (e.g. 460800)",
    )
    args = parser.parse_args()

    port = args.port
    if not port:
        port = find_adapter()
        if not port:
            print("Error: could not find a USB-UART adapter automatically.")
            print("Available ports:")
            for p in serial.tools.list_ports.comports():
                print(f"  {p.device}: {p.description}")
            print("\nSpecify with --port /dev/ttyUSBx")
            sys.exit(1)
        print(f"Auto-detected adapter at {port}")

    echo_loop(port, args.stop_at)


if __name__ == "__main__":
    main()
