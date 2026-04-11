#!/usr/bin/env bash
# run_echo_server.sh — Start the UART echo server for uart_irq_partition_echo.
#
# Kills any existing instance, then starts a fresh one.
# Logs to f429zi/echo_server.log (next to this script).
# Usage:  ./run_echo_server.sh [--port /dev/ttyUSBx] [--stop-at BAUD]
#
# IMPORTANT: start this BEFORE resetting the MCU — the MCU sends its 0xA5
# handshake probe at ~900 ms after boot and won't retry if the host misses it.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOG="$SCRIPT_DIR/echo_server.log"
PY="$SCRIPT_DIR/host_serial_echo.py"

# Kill any running instance.
if pkill -f "host_serial_echo.py" 2>/dev/null; then
    echo "[run_echo_server] killed previous instance"
    sleep 0.3
fi

echo "[run_echo_server] starting — log: $LOG"
python3 -u "$PY" "$@" > "$LOG" 2>&1 &
PID=$!
echo "[run_echo_server] PID=$PID"

# Brief wait so the port opens before caller resets the MCU.
sleep 0.4
echo "[run_echo_server] port open — reset MCU now"
