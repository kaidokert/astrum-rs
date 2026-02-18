#!/usr/bin/env bash
# Hardware CI testing using probe-rs.
# Usage: ./scripts/probe-rs-ci.sh --elf <path> --chip <chip>
set -euo pipefail

usage() {
    cat <<EOF
Usage: $(basename "$0") --elf <path> --chip <chip>

Hardware CI testing using probe-rs.

Options:
  --elf <path>    Path to the ELF firmware binary
  --chip <chip>   Target chip (e.g., nRF52840_xxAA, STM32F401CCUx)
  --help          Show this help message

Example:
  $(basename "$0") --elf target/thumbv7em-none-eabihf/release/app --chip nRF52840_xxAA

Requirements:
  - probe-rs CLI must be installed (https://probe.rs)
  - A debug probe must be connected to the target hardware
EOF
}

ELF=""
CHIP=""

while [[ $# -gt 0 ]]; do
    case "$1" in
        --elf)
            if [[ $# -lt 2 ]]; then
                echo "Error: --elf requires a value" >&2
                usage >&2
                exit 1
            fi
            if [[ "$2" == -* ]]; then
                echo "Error: --elf value cannot start with '-': $2" >&2
                usage >&2
                exit 1
            fi
            ELF="$2"
            shift 2
            ;;
        --chip)
            if [[ $# -lt 2 ]]; then
                echo "Error: --chip requires a value" >&2
                usage >&2
                exit 1
            fi
            if [[ "$2" == -* ]]; then
                echo "Error: --chip value cannot start with '-': $2" >&2
                usage >&2
                exit 1
            fi
            CHIP="$2"
            shift 2
            ;;
        --help)
            usage
            exit 0
            ;;
        *)
            echo "Error: Unknown option: $1" >&2
            usage >&2
            exit 1
            ;;
    esac
done

if [[ -z "$ELF" ]]; then
    echo "Error: --elf argument is required" >&2
    usage >&2
    exit 1
fi

if [[ -z "$CHIP" ]]; then
    echo "Error: --chip argument is required" >&2
    usage >&2
    exit 1
fi

if ! command -v probe-rs &> /dev/null; then
    cat >&2 <<EOF
Error: probe-rs is not installed or not in PATH.

To install probe-rs:
  curl --proto '=https' --tlsv1.2 -LsSf https://github.com/probe-rs/probe-rs/releases/latest/download/probe-rs-tools-installer.sh | sh

Or via cargo:
  cargo install probe-rs-tools

See https://probe.rs for more installation options.
EOF
    echo "" >&2
    usage >&2
    exit 1
fi

if [[ ! -f "$ELF" ]]; then
    echo "Error: ELF file not found: $ELF" >&2
    exit 1
fi

echo "probe-rs CI test"
echo "  ELF:  $ELF"
echo "  Chip: $CHIP"
echo ""

# Timeout in seconds for probe-rs run
TIMEOUT_SECONDS=30
# Grace period before SIGKILL if SIGTERM is ignored (hardware debug tools can hang)
KILL_AFTER_SECONDS=5

# Run the firmware on hardware with timeout
echo "Running firmware with ${TIMEOUT_SECONDS}s timeout..."
echo "=== probe-rs output ==="

# Run probe-rs with timeout and real-time output streaming.
# - Use --kill-after to forcefully terminate hung debug tools
# - Stream output in real-time for hardware debugging visibility
# - Capture exit code without toggling global shell error state
timeout --kill-after="${KILL_AFTER_SECONDS}s" "$TIMEOUT_SECONDS" \
    probe-rs run --chip "$CHIP" "$ELF" 2>&1 \
    && EXIT_CODE=0 || EXIT_CODE=$?

echo "=== end probe-rs output ==="

# Check for timeout (exit code 124 from timeout command)
# Exit code 137 indicates SIGKILL was needed (128 + 9)
if [[ $EXIT_CODE -eq 124 ]]; then
    echo ""
    echo "FAIL: probe-rs run timed out after ${TIMEOUT_SECONDS}s (SIGTERM)" >&2
    exit 1
fi

if [[ $EXIT_CODE -eq 137 ]]; then
    echo ""
    echo "FAIL: probe-rs run timed out and required SIGKILL after ${KILL_AFTER_SECONDS}s grace period" >&2
    exit 1
fi

# Check exit code
if [[ $EXIT_CODE -ne 0 ]]; then
    echo ""
    echo "FAIL: probe-rs run exited with code $EXIT_CODE" >&2
    exit 1
fi

echo ""
echo "PASS: probe-rs run completed successfully"
exit 0
