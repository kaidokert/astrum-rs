#!/usr/bin/env bash
# Unit tests for probe-rs-ci.sh RTT output parsing logic.
set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TEST_DIR=$(mktemp -d)
trap 'rm -rf "$TEST_DIR"' EXIT
touch "$TEST_DIR/test.elf"

PASS=0 FAIL=0

mock_probe_rs() {
    cat > "$TEST_DIR/probe-rs" <<EOF
#!/bin/bash
echo "$1"; exit ${2:-0}
EOF
    chmod +x "$TEST_DIR/probe-rs"
}

run_test() {
    local name="$1" expected="$2" output="$3" exit_code="${4:-0}" expected_msg="${5:-}"
    mock_probe_rs "$output" "$exit_code"
    local script_output
    script_output=$(PATH="$TEST_DIR:$PATH" "$SCRIPT_DIR/probe-rs-ci.sh" \
        --elf "$TEST_DIR/test.elf" --chip TEST 2>&1) && rc=0 || rc=$?
    if [[ "$rc" -ne "$expected" ]]; then
        echo "FAIL: $name (expected exit $expected, got $rc)"; FAIL=$((FAIL + 1))
        return
    fi
    if [[ -n "$expected_msg" ]] && ! echo "$script_output" | grep -qF "$expected_msg"; then
        echo "FAIL: $name (expected message '$expected_msg' not found)"; FAIL=$((FAIL + 1))
        return
    fi
    echo "PASS: $name"; PASS=$((PASS + 1))
}

echo "=== probe-rs-ci.sh Tests ==="
run_test "TEST PASSED marker exits 0" 0 "TEST PASSED" 0 "PASS: Test marker"
run_test "TEST FAILED marker exits 1" 1 "TEST FAILED" 0 "FAIL: Test marker"
run_test "No marker exits 1" 1 "no markers here" 0 "No test marker"
run_test "FAILED takes precedence over PASSED" 1 $'TEST PASSED\nTEST FAILED' 0 "FAIL: Test marker"
run_test "Embedded marker matches" 0 "[INFO] TEST PASSED done" 0 "PASS:"
run_test "probe-rs error without marker exits 1" 1 "Flash error" 1 "exited with code 1"

echo ""; echo "Passed: $PASS / $((PASS + FAIL))"
if [[ $FAIL -eq 0 ]]; then exit 0; else exit 1; fi
