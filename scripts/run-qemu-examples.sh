#!/usr/bin/env bash
# Run all QEMU integration examples and check exit codes.
set -euo pipefail

TARGET="thumbv7m-none-eabi"
FEATURES="qemu"
PASS=0
FAIL=0
FAILED=""

EXAMPLES=(
    mpu_region systick mpu_wx mpu_fault
    partition_switch context_switch mpu_partition
    svc_yield integration sampling_demo queuing_demo blackboard_demo
)

for ex in "${EXAMPLES[@]}"; do
    printf "%-20s " "$ex"
    if timeout 30 cargo run --target "$TARGET" --features "$FEATURES" --example "$ex" 2>&1 \
       | tail -1 | grep -qiE 'pass|done|success'; then
        echo "PASS"
        PASS=$((PASS + 1))
    else
        echo "FAIL"
        FAIL=$((FAIL + 1))
        FAILED="$FAILED $ex"
    fi
done

echo ""
echo "Results: $PASS passed, $FAIL failed"
[ -n "$FAILED" ] && echo "Failed:$FAILED"
exit "$FAIL"
