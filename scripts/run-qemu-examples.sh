#!/usr/bin/env bash
# Run all QEMU integration examples and check exit codes.
# Pass --dynamic-mpu to also run dynamic-mpu examples.
set -euo pipefail

TARGET="thumbv7m-none-eabi"
PASS=0
FAIL=0
FAILED=""

EXAMPLES=(
    qemu_smoke
    mpu_region systick mpu_wx mpu_fault
    partition_switch context_switch mpu_partition
    svc_yield integration sampling_demo queuing_demo blackboard_demo
    callee_save_check  # committed in prior commits; see kernel/examples/callee_save_check.rs
)

DYNAMIC_MPU_EXAMPLES=(
    mpu_dynamic_test buffer_pool_test virtual_uart_demo
)

run_examples() {
    local features="$1"
    shift
    for ex in "$@"; do
        printf "%-20s " "$ex"
        if timeout 30 cargo run --target "$TARGET" --features "$features" --example "$ex" 2>&1 \
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
run_examples "qemu,log-semihosting" "${EXAMPLES[@]}"

if [[ "${1:-}" == "--dynamic-mpu" ]]; then
    echo ""
    echo "=== Dynamic-MPU examples ==="
    run_examples "qemu,log-semihosting,dynamic-mpu" "${DYNAMIC_MPU_EXAMPLES[@]}"
fi

echo ""
echo "Results: $PASS passed, $FAIL failed"
[ -n "$FAILED" ] && echo "Failed:$FAILED"
exit "$FAIL"
