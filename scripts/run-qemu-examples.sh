#!/usr/bin/env bash
# Run all QEMU integration examples and check exit codes.
# Pass --strict to fail examples that lack an expected-output file.
# Pass --record to save captured output as expected-output files.
# Pass --only <name> to run a single example instead of all.
set -euo pipefail

TARGET="thumbv7m-none-eabi"

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="${REPO_ROOT:-$(cd "$SCRIPT_DIR/.." && pwd)}"

# Prefer python3, fall back to python (Windows).
PYTHON="${PYTHON:-$(command -v python3 2>/dev/null || command -v python 2>/dev/null || echo python3)}"

# Run a single example, capturing full semihosting stdout to a file.
# Usage: capture_example_output <features> <example> [extra_cargo_flags...]
# Writes output to $OUTDIR/<example>.out and returns the cargo exit status.
capture_example_output() {
    local features="$1"
    local ex="$2"
    local outfile="$OUTDIR/${ex}.out"
    # shellcheck disable=SC2086
    timeout 30 cargo run -p kernel --target "$TARGET" --features "$features" --example "$ex" $CARGO_EXTRA_ARGS \
        > "$outfile" 2>"$OUTDIR/${ex}.stderr"
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
        if report=$("$PYTHON" "$SCRIPT_DIR/compare-output.py" "$expected_file" "$outfile" 2>&1); then
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
        local rc=0
        capture_example_output "$features" "$ex" || rc=$?
        if [[ "$rc" -ne 0 ]]; then
            # If an expected file matches the captured output, treat as pass.
            local expected_file="$REPO_ROOT/kernel/expected/${ex}.expected"
            if [[ -f "$expected_file" ]] && check_example "$ex"; then
                echo "PASS (expected failure)"
                PASS=$((PASS + 1))
                continue
            fi
            echo "FAIL (build/runtime error, rc=$rc)"
            if [[ -s "$OUTDIR/${ex}.out" ]]; then
                echo "  stdout: $(head -3 "$OUTDIR/${ex}.out")" >&2
            else
                echo "  stdout: (empty)" >&2
            fi
            if [[ -s "$OUTDIR/${ex}.stderr" ]]; then
                echo "  stderr: $(tail -5 "$OUTDIR/${ex}.stderr")" >&2
            fi
            FAIL=$((FAIL + 1))
            FAILED="$FAILED $ex"
            continue
        fi
        if check_example "$ex"; then
            echo "PASS"
            PASS=$((PASS + 1))
        else
            echo "FAIL (output mismatch)"
            if [[ -s "$OUTDIR/${ex}.out" ]]; then
                echo "  stdout: $(head -3 "$OUTDIR/${ex}.out")" >&2
            fi
            FAIL=$((FAIL + 1))
            FAILED="$FAILED $ex"
        fi
    done
}

# Record a single example's captured output as the expected-output file.
# Usage: record_example <example>
record_example() {
    local ex="$1"
    local expected_dir="$REPO_ROOT/kernel/expected"
    local expected_file="$expected_dir/${ex}.expected"
    local outfile="$OUTDIR/${ex}.out"

    mkdir -p "$expected_dir"

    if [[ -f "$expected_file" ]]; then
        echo "  WARNING: overwriting $expected_file" >&2
    fi
    cp "$outfile" "$expected_file"
}

# Run examples in record mode — capture and save expected output.
run_examples_record() {
    local features="$1"
    shift
    for ex in "$@"; do
        printf "%-20s " "$ex"
        local rc=0
        capture_example_output "$features" "$ex" || rc=$?
        if [[ "$rc" -ne 0 ]] && [[ ! -s "$OUTDIR/${ex}.out" ]]; then
            echo "FAIL (no output)"
            FAIL=$((FAIL + 1))
            FAILED="$FAILED $ex"
            continue
        fi
        record_example "$ex"
        if [[ "$rc" -ne 0 ]]; then
            echo "RECORDED (exit $rc)"
        else
            echo "RECORDED"
        fi
        PASS=$((PASS + 1))
    done
}

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
    echo "QEMU version: $(qemu-system-arm --version | head -1)"
    echo "Rust version: $(rustc --version)"
    echo ""
    PASS=0
    FAIL=0
    FAILED=""
    STRICT=0
    RECORD=0
    ONLY=""
    OUTDIR="${TMPDIR:-/tmp}/rtos-examples-$$"
    mkdir -p "$OUTDIR"
    CARGO_EXTRA_ARGS=""

    source "$SCRIPT_DIR/examples.list"

    # Parse flags (consume from positional args).
    QEMU_PERIPHERALS=0
    while [[ $# -gt 0 ]]; do
        case "$1" in
            --strict) STRICT=1 ;;
            --qemu-peripherals) QEMU_PERIPHERALS=1 ;;
            --record) RECORD=1 ;;
            --only) shift; ONLY="$1" ;;
        esac
        shift
    done

    # Select the run function based on mode.
    if [[ "$RECORD" -eq 1 ]]; then
        RUN_FN=run_examples_record
    else
        RUN_FN=run_examples
    fi

    # If --only is set, find which category the example belongs to and run it.
    if [[ -n "$ONLY" ]]; then
        # Map each category to its feature flags and example list.
        declare -A CATEGORY_FEATURES=(
            [EXAMPLES]="qemu,log-semihosting,ipc-blackboard"
            [RELEASE_EXAMPLES]="qemu,log-semihosting,ipc-blackboard"
            [CUSTOM_IVT_EXAMPLES]="qemu,log-semihosting,custom-ivt"
            [DYNAMIC_MPU_EXAMPLES]="qemu,log-semihosting"
            [QEMU_PERIPHERAL_EXAMPLES]="qemu,log-semihosting,qemu-peripherals"
            [INTRA_THREAD_EXAMPLES]="qemu,log-semihosting,intra-threads"
        )
        declare -A CATEGORY_EXTRA_ARGS=(
            [RELEASE_EXAMPLES]="--release"
        )
        found=0
        for category in EXAMPLES RELEASE_EXAMPLES CUSTOM_IVT_EXAMPLES DYNAMIC_MPU_EXAMPLES QEMU_PERIPHERAL_EXAMPLES INTRA_THREAD_EXAMPLES; do
            declare -n list="$category"
            for ex in "${list[@]}"; do
                if [[ "$ex" == "$ONLY" ]]; then
                    CARGO_EXTRA_ARGS="${CATEGORY_EXTRA_ARGS[$category]:-}"
                    $RUN_FN "${CATEGORY_FEATURES[$category]}" "$ONLY"
                    CARGO_EXTRA_ARGS=""
                    found=1; break 2
                fi
            done
        done
        if [[ "$found" -eq 0 ]]; then
            echo "ERROR: example '$ONLY' not found in any example list" >&2
            exit 1
        fi
    else
        echo "=== Static-mode examples ==="
        $RUN_FN "qemu,log-semihosting,ipc-blackboard" "${EXAMPLES[@]}"

        if [[ ${#RELEASE_EXAMPLES[@]} -gt 0 ]]; then
            echo ""
            echo "=== Release-only examples (--release) ==="
            CARGO_EXTRA_ARGS="--release"
            $RUN_FN "qemu,log-semihosting,ipc-blackboard" "${RELEASE_EXAMPLES[@]}"
            CARGO_EXTRA_ARGS=""
        fi

        echo ""
        echo "=== Dynamic-MPU examples ==="
        $RUN_FN "qemu,log-semihosting" "${DYNAMIC_MPU_EXAMPLES[@]}"

        echo ""
        echo "=== Custom-IVT examples ==="
        $RUN_FN "qemu,log-semihosting,custom-ivt" "${CUSTOM_IVT_EXAMPLES[@]}"

        echo ""
        echo "=== Intra-thread examples ==="
        $RUN_FN "qemu,log-semihosting,intra-threads" "${INTRA_THREAD_EXAMPLES[@]}"

        if [[ "$QEMU_PERIPHERALS" -eq 1 ]]; then
            echo ""
            echo "=== QEMU peripheral examples ==="
            $RUN_FN "qemu,log-semihosting,qemu-peripherals" "${QEMU_PERIPHERAL_EXAMPLES[@]}"
        fi
    fi

    echo ""
    echo "Results: $PASS passed, $FAIL failed"
    [ -n "$FAILED" ] && echo "Failed:$FAILED"
    exit "$FAIL"
fi
