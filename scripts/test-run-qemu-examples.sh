#!/usr/bin/env bash
# Unit tests for run-qemu-examples.sh check_example logic.
set -euo pipefail

PASS_COUNT=0
FAIL_COUNT=0

check() {
    local name="$1" expected="$2" actual="$3"
    if [[ "$expected" == "$actual" ]]; then
        PASS_COUNT=$((PASS_COUNT + 1))
    else
        echo "FAIL: $name (expected=$expected, actual=$actual)"
        FAIL_COUNT=$((FAIL_COUNT + 1))
    fi
}

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

# Setup temp dirs
OUTDIR="$(mktemp -d)"
EXPECTED_DIR="$(mktemp -d)"
trap 'rm -rf "$OUTDIR" "$EXPECTED_DIR"' EXIT

# Override REPO_ROOT so check_example finds expected files under our temp dir.
# The function looks for $REPO_ROOT/kernel/expected/<ex>.expected.
REPO_ROOT="$EXPECTED_DIR"
mkdir -p "$EXPECTED_DIR/kernel/expected"

# Source shared functions from the runner (guarded by BASH_SOURCE check).
source "$SCRIPT_DIR/run-qemu-examples.sh"

# --- Test 1: Expected file matches output (exact mode) ---
STRICT=0
echo "hello world" > "$OUTDIR/t1.out"
echo "hello world" > "$EXPECTED_DIR/kernel/expected/t1.expected"
check_example "t1" && result=0 || result=1
check "expected-match-exact" "0" "$result"

# --- Test 2: Expected file does NOT match output ---
STRICT=0
echo "hello world" > "$OUTDIR/t2.out"
echo "goodbye world" > "$EXPECTED_DIR/kernel/expected/t2.expected"
check_example "t2" && result=0 || result=1
check "expected-mismatch" "1" "$result"

# --- Test 3: No expected file, fallback grep passes ---
STRICT=0
echo -e "some output\nALL TESTS PASSED" > "$OUTDIR/t3.out"
# No expected file for t3
check_example "t3" && result=0 || result=1
check "fallback-grep-pass" "0" "$result"

# --- Test 4: No expected file, fallback grep fails ---
STRICT=0
echo -e "some output\nerror occurred" > "$OUTDIR/t4.out"
check_example "t4" && result=0 || result=1
check "fallback-grep-fail" "1" "$result"

# --- Test 5: Strict mode, no expected file -> FAIL ---
STRICT=1
echo -e "some output\nALL TESTS PASSED" > "$OUTDIR/t5.out"
check_example "t5" && result=0 || result=1
check "strict-no-expected-fail" "1" "$result"

# --- Test 6: Strict mode, expected file exists and matches -> PASS ---
STRICT=1
echo "strict pass" > "$OUTDIR/t6.out"
echo "strict pass" > "$EXPECTED_DIR/kernel/expected/t6.expected"
check_example "t6" && result=0 || result=1
check "strict-with-expected-pass" "0" "$result"

# --- Test 7: Strict mode, expected file exists but mismatches -> FAIL ---
STRICT=1
echo "actual output" > "$OUTDIR/t7.out"
echo "expected output" > "$EXPECTED_DIR/kernel/expected/t7.expected"
check_example "t7" && result=0 || result=1
check "strict-with-expected-fail" "1" "$result"

# --- Test 8: Regex mode in expected file ---
STRICT=0
printf "# mode: regex\nhel+o\n" > "$EXPECTED_DIR/kernel/expected/t8.expected"
echo "hello" > "$OUTDIR/t8.out"
check_example "t8" && result=0 || result=1
check "regex-mode-match" "0" "$result"

# --- Test 9: Contains mode in expected file ---
STRICT=0
printf "# mode: contains\nworld\n" > "$EXPECTED_DIR/kernel/expected/t9.expected"
echo -e "hello world\ngoodbye" > "$OUTDIR/t9.out"
check_example "t9" && result=0 || result=1
check "contains-mode-match" "0" "$result"

# --- Test 10: Fallback grep matches 'done' (case insensitive) ---
STRICT=0
echo -e "output line\nDone" > "$OUTDIR/t10.out"
check_example "t10" && result=0 || result=1
check "fallback-grep-done" "0" "$result"

# --- Test 11: Fallback grep matches 'success' ---
STRICT=0
echo -e "output line\nSuccess!" > "$OUTDIR/t11.out"
check_example "t11" && result=0 || result=1
check "fallback-grep-success" "0" "$result"

# --- Run compare-output.py self-tests too ---
echo ""
echo "=== compare-output.py self-tests ==="
python3 "$SCRIPT_DIR/compare-output.py" --self-test
py_result=$?

echo ""
echo "=== Shell tests ==="
echo "$PASS_COUNT passed, $FAIL_COUNT failed"
if [[ "$FAIL_COUNT" -gt 0 || "$py_result" -ne 0 ]]; then
    exit 1
fi
exit 0
