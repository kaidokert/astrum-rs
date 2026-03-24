#!/usr/bin/env bash
# Run all QEMU integration examples and check exit codes.
# Pass --dynamic-mpu to also run dynamic-mpu examples.
# Pass --strict to fail examples that lack an expected-output file.
set -euo pipefail

TARGET="thumbv7m-none-eabi"

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="${REPO_ROOT:-$(cd "$SCRIPT_DIR/.." && pwd)}"

# Run a single example, capturing full semihosting stdout to a file.
# Usage: capture_example_output <features> <example>
# Writes output to $OUTDIR/<example>.out and returns the cargo exit status.
capture_example_output() {
    local features="$1"
    local ex="$2"
    local outfile="$OUTDIR/${ex}.out"
    timeout 30 cargo run --target "$TARGET" --features "$features" --example "$ex" \
        > "$outfile" 2>&1
}

# Check a single example's output against its expected file or last-line grep.
# Usage: check_example <example>
# Returns 0 on pass, 1 on fail.  On failure, prints a report to stderr.
check_example() {
    local ex="$1"
    local expected_file="$REPO_ROOT/kernel/expected/${ex}.expected"
    local outfile="$OUTDIR/${ex}.out"

    if [[ -f "$expected_file" ]]; then
        local report
        if report=$(python3 "$SCRIPT_DIR/compare-output.py" "$expected_file" "$outfile" 2>&1); then
            return 0
        fi
        if [[ -n "$report" ]]; then
            echo "$report" >&2
        fi
        return 1
    fi

    # No expected file — strict mode fails immediately.
    if [[ "$STRICT" -eq 1 ]]; then
        echo "  (no expected file; strict mode)" >&2
        return 1
    fi

    # Fallback: last-line grep check.
    if tail -1 "$outfile" | grep -qiE 'pass|done|success'; then
        return 0
    fi
    return 1
}

run_examples() {
    local features="$1"
    shift
    for ex in "$@"; do
        printf "%-20s " "$ex"
        if ! capture_example_output "$features" "$ex"; then
            echo "FAIL (build/runtime error)"
            FAIL=$((FAIL + 1))
            FAILED="$FAILED $ex"
            continue
        fi
        if check_example "$ex"; then
            echo "PASS"
            PASS=$((PASS + 1))
        else
            echo "FAIL"
            FAIL=$((FAIL + 1))
            FAILED="$FAILED $ex"
        fi
    done
}

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
    PASS=0
    FAIL=0
    FAILED=""
    STRICT=0
    OUTDIR="${TMPDIR:-/tmp}/rtos-examples-$$"
    mkdir -p "$OUTDIR"

    source "$SCRIPT_DIR/examples.list"

    # Parse flags (consume from positional args).
    DYNAMIC_MPU=0
    for arg in "$@"; do
        case "$arg" in
            --strict) STRICT=1 ;;
            --dynamic-mpu) DYNAMIC_MPU=1 ;;
        esac
    done

    echo "=== Static-mode examples ==="
    run_examples "qemu,log-semihosting,ipc-blackboard" "${EXAMPLES[@]}"

    echo ""
    echo "=== Custom-IVT examples ==="
    run_examples "qemu,log-semihosting,custom-ivt" "${CUSTOM_IVT_EXAMPLES[@]}"

    if [[ "$DYNAMIC_MPU" -eq 1 ]]; then
        echo ""
        echo "=== Dynamic-MPU examples ==="
        run_examples "qemu,log-semihosting,dynamic-mpu" "${DYNAMIC_MPU_EXAMPLES[@]}"
    fi

    echo ""
    echo "Results: $PASS passed, $FAIL failed"
    [ -n "$FAILED" ] && echo "Failed:$FAILED"
    exit "$FAIL"
fi
