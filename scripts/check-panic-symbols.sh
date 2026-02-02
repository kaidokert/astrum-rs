#!/usr/bin/env bash
# check-panic-symbols.sh — Scan release binaries for panic/unwind symbols.
#
# The kernel *library* itself should be panic-free: it avoids unwrap(),
# panicking indexing, and other operations that pull in the panic machinery.
# However, the *examples* link against panic_semihosting (required by
# cortex-m-rt for a working panic handler), so they will always contain
# panic/unwind symbols. This script is therefore informational by default
# (exit 0) — it reports what it finds without failing CI.
#
# Usage:
#   scripts/check-panic-symbols.sh              # informational (exit 0)
#   scripts/check-panic-symbols.sh --strict     # exit 1 if any symbols found
#
# The --strict flag is reserved for future use once we have a mechanism to
# verify the kernel library in isolation (e.g. a no-panic example or a
# separate binary that does not link panic_semihosting).

set -euo pipefail

REPO_ROOT="$(git -C "$(dirname "$0")" rev-parse --show-toplevel)"
cd "$REPO_ROOT/kernel"

TARGET="thumbv7m-none-eabi"
STRICT=0

for arg in "$@"; do
    case "$arg" in
        --strict) STRICT=1 ;;
        -h|--help)
            echo "Usage: $0 [--strict]"
            echo "  --strict  Exit 1 if any panic/unwind symbols are found"
            exit 0
            ;;
        *)
            echo "Unknown argument: $arg" >&2
            exit 1
            ;;
    esac
done

# ── Examples to check ────────────────────────────────────────────────
# Static-mode examples (require 'qemu' feature)
STATIC_EXAMPLES=(
    hello panic_test mpu_info mpu_region systick mpu_wx mpu_fault
    partition_switch context_switch mpu_partition svc_yield
    scheduler_tick integration sampling_demo queuing_demo blackboard_demo
)

# Dynamic-MPU examples (require 'qemu,dynamic-mpu' features)
DYNAMIC_MPU_EXAMPLES=(
    mpu_dynamic mpu_dynamic_test buffer_pool_test
    virtual_uart_demo uart1_loopback
)

# ── Counters ─────────────────────────────────────────────────────────
CLEAN=0
TAINTED=0
TAINTED_LIST=""
ERRORS=0
ERROR_LIST=""

# ── Check the kernel library first ──────────────────────────────────
echo "=== Kernel library (release) ==="
if LIB_OUTPUT=$(cargo nm --target "$TARGET" --lib --release -- --demangle 2>&1); then
    LIB_SYMBOLS=$(echo "$LIB_OUTPUT" | grep -ciE 'panic|unwind' || true)
    if [ "$LIB_SYMBOLS" -gt 0 ]; then
        echo "  WARN: $LIB_SYMBOLS panic/unwind symbol(s) in kernel library!"
        echo "$LIB_OUTPUT" | grep -iE 'panic|unwind' | sed 's/^/    /'
        TAINTED=$((TAINTED + 1))
        TAINTED_LIST="$TAINTED_LIST lib"
    else
        echo "  CLEAN (no panic/unwind symbols)"
        CLEAN=$((CLEAN + 1))
    fi
else
    echo "  ERROR: cargo nm failed for library"
    ERRORS=$((ERRORS + 1))
    ERROR_LIST="$ERROR_LIST lib"
fi

# ── Helper: check one example ───────────────────────────────────────
check_example() {
    local name="$1"
    local features="$2"

    printf "  %-24s " "$name"

    local output
    if ! output=$(cargo nm --target "$TARGET" --example "$name" \
                  --features "$features" --release -- --demangle 2>&1); then
        echo "ERROR (build/nm failed)"
        ERRORS=$((ERRORS + 1))
        ERROR_LIST="$ERROR_LIST $name"
        return
    fi

    local count
    count=$(echo "$output" | grep -ciE 'panic|unwind' || true)

    if [ "$count" -gt 0 ]; then
        echo "FOUND ($count panic/unwind symbols)"
        TAINTED=$((TAINTED + 1))
        TAINTED_LIST="$TAINTED_LIST $name"
    else
        echo "CLEAN"
        CLEAN=$((CLEAN + 1))
    fi
}

# ── Check static-mode examples ──────────────────────────────────────
echo ""
echo "=== Static-mode examples (release, --features qemu) ==="
for ex in "${STATIC_EXAMPLES[@]}"; do
    check_example "$ex" "qemu"
done

# ── Check dynamic-MPU examples ──────────────────────────────────────
echo ""
echo "=== Dynamic-MPU examples (release, --features qemu,dynamic-mpu) ==="
for ex in "${DYNAMIC_MPU_EXAMPLES[@]}"; do
    check_example "$ex" "qemu,dynamic-mpu"
done

# ── Summary ──────────────────────────────────────────────────────────
echo ""
echo "=== Summary ==="
TOTAL=$((CLEAN + TAINTED + ERRORS))
echo "  Total checked:  $TOTAL"
echo "  Clean:          $CLEAN"
echo "  Panic symbols:  $TAINTED"
echo "  Errors:         $ERRORS"

if [ "$TAINTED" -gt 0 ]; then
    echo ""
    echo "  Binaries with panic/unwind symbols:$TAINTED_LIST"
    # NOTE: Examples are expected to have panic symbols because they use
    # panic_semihosting as their panic handler. The kernel library being
    # clean is the important invariant.
    echo ""
    echo "  (Expected: examples use panic_semihosting and will have panic symbols."
    echo "   The kernel library should remain panic-free.)"
fi

if [ "$ERRORS" -gt 0 ]; then
    echo ""
    echo "  Failed to check:$ERROR_LIST"
fi

# ── Exit code ────────────────────────────────────────────────────────
if [ "$STRICT" -eq 1 ] && [ "$TAINTED" -gt 0 ]; then
    echo ""
    echo "STRICT MODE: failing because panic/unwind symbols were found."
    exit 1
fi

exit 0
