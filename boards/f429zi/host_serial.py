#!/usr/bin/env python3
"""
Host script to interact with STM32F429ZI USB command interface.

Usage:
    ./host_serial.py                    # Interactive mode
    ./host_serial.py "led on"           # Send single command
    ./host_serial.py -p /dev/ttyACM1    # Specify port
"""

import sys
import serial
import serial.tools.list_ports
import time
import argparse


def find_device():
    """Find the STM32F429 USB CDC device."""
    ports = serial.tools.list_ports.comports()
    for port in ports:
        # Look for our device by product name
        if port.product and "F429 Command Interface" in port.product:
            return port.device
        # Fallback: look for generic CDC ACM device
        if "USB" in port.device or "ACM" in port.device:
            # Could be our device, but not certain
            print(f"Found potential device: {port.device} ({port.description})")
    return None


def send_command(ser, cmd):
    """Send a command and receive response."""
    # Send command with newline
    ser.write((cmd + "\n").encode('utf-8'))
    ser.flush()

    # Read response (may be multiple lines)
    time.sleep(0.05)  # Short delay for device to process
    response = []
    while ser.in_waiting > 0:
        line = ser.readline().decode('utf-8', errors='ignore').strip()
        if line:
            response.append(line)

    return response


def interactive_mode(ser):
    """Interactive command prompt."""
    print("\n=== STM32F429ZI Command Interface ===")
    print("Type 'help' for available commands")
    print("Type 'quit' or Ctrl+C to exit\n")

    # Send initial help to show available commands
    response = send_command(ser, "help")
    for line in response:
        print(line)
    print()

    try:
        while True:
            try:
                cmd = input("stm32> ").strip()
            except EOFError:
                break

            if not cmd:
                continue

            if cmd.lower() in ('quit', 'exit', 'q'):
                break

            response = send_command(ser, cmd)
            for line in response:
                print(line)

            # If no response, indicate that
            if not response:
                print("(no response)")
            print()

    except KeyboardInterrupt:
        print("\nExiting...")


def single_command_mode(ser, cmd):
    """Send a single command and print response."""
    response = send_command(ser, cmd)
    for line in response:
        print(line)
    if not response:
        print("(no response)")


def main():
    parser = argparse.ArgumentParser(
        description="Interact with STM32F429ZI USB command interface"
    )
    parser.add_argument(
        '-p', '--port',
        help='Serial port (e.g., /dev/ttyACM0, COM3)',
        default=None
    )
    parser.add_argument(
        '-b', '--baudrate',
        type=int,
        default=115200,
        help='Baud rate (default: 115200)'
    )
    parser.add_argument(
        'command',
        nargs='?',
        help='Single command to send (interactive mode if omitted)'
    )

    args = parser.parse_args()

    # Find device if not specified
    port = args.port
    if not port:
        print("Searching for STM32F429 device...")
        port = find_device()
        if not port:
            print("\nError: Could not find device automatically.")
            print("Please specify port with -p option")
            print("\nAvailable ports:")
            for p in serial.tools.list_ports.comports():
                print(f"  {p.device}: {p.description}")
            sys.exit(1)
        print(f"Found device at: {port}\n")

    # Open serial port
    try:
        ser = serial.Serial(
            port,
            args.baudrate,
            timeout=0.5,
            write_timeout=1.0
        )
        time.sleep(0.5)  # Wait for device to be ready

        # Clear any pending data
        ser.reset_input_buffer()
        ser.reset_output_buffer()

    except serial.SerialException as e:
        print(f"Error opening port {port}: {e}")
        sys.exit(1)

    try:
        if args.command:
            # Single command mode
            single_command_mode(ser, args.command)
        else:
            # Interactive mode
            interactive_mode(ser)

    finally:
        ser.close()


if __name__ == '__main__':
    main()
