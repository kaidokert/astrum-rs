#!/usr/bin/env bash
# Run all QEMU integration examples and check exit codes.
# Pass --dynamic-mpu to also run dynamic-mpu examples.
set -euo pipefail

TARGET="thumbv7m-none-eabi"
PASS=0
FAIL=0
FAILED=""
OUTDIR="${TMPDIR:-/tmp}/rtos-examples-$$"
mkdir -p "$OUTDIR"

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
source "$SCRIPT_DIR/examples.list"

# Run a single example, capturing full semihosting stdout to a file.
# Usage: capture_example_output <features> <example>
# Writes output to $OUTDIR/<example>.out and returns the cargo exit status.
capture_example_output() {
    local features="$1"
    local ex="$2"
    local outfile="$OUTDIR/${ex}.out"
    timeout 30 cargo run --target "$TARGET" --features "$features" --example "$ex" 2>&1 \
        | tee "$outfile"
    return "${PIPESTATUS[0]}"
}

run_examples() {
    local features="$1"
    shift
    for ex in "$@"; do
        printf "%-20s " "$ex"
        if capture_example_output "$features" "$ex" \
           | tail -1 | grep -qiE 'pass|done|success'; then
            echo "PASS"
            PASS=$((PASS + 1))
        else
            echo "FAIL"
            FAIL=$((FAIL + 1))
            FAILED="$FAILED $ex"
        fi
    done
}

echo "=== Static-mode examples ==="
run_examples "qemu,log-semihosting,ipc-blackboard" "${EXAMPLES[@]}"

echo ""
echo "=== Custom-IVT examples ==="
run_examples "qemu,log-semihosting,custom-ivt" "${CUSTOM_IVT_EXAMPLES[@]}"

if [[ "${1:-}" == "--dynamic-mpu" ]]; then
    echo ""
    echo "=== Dynamic-MPU examples ==="
    run_examples "qemu,log-semihosting,dynamic-mpu" "${DYNAMIC_MPU_EXAMPLES[@]}"
fi

echo ""
echo "Results: $PASS passed, $FAIL failed"
[ -n "$FAILED" ] && echo "Failed:$FAILED"
exit "$FAIL"
