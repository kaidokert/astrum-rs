#!/usr/bin/env bash
# Acceptance test for GDB MCP debugging.
# Part 1: prerequisites, build, MCP server startup, JSON-RPC helpers.
set -euo pipefail

REPO_ROOT="$(git -C "$(dirname "$0")" rev-parse --show-toplevel)"
KERNEL_DIR="$REPO_ROOT/kernel"
TARGET="thumbv7m-none-eabi"
ELF="$REPO_ROOT/target/$TARGET/debug/examples/svc_yield"
REQ_ID=0

# MCP server path: honour $GDB_MCP_SERVER, else look for a sibling checkout.
MCP_SERVER="${GDB_MCP_SERVER:-$REPO_ROOT/../gdb-mcp-server/dist/index.js}"

# Stderr log for the MCP server coprocess (aids debugging).
MCP_STDERR_LOG="$(mktemp "${TMPDIR:-/tmp}/mcp-stderr-XXXXXX.log")"

# ---------------------------------------------------------------------------
# Cleanup: kill MCP coprocess on exit
# ---------------------------------------------------------------------------
cleanup() {
    if [[ -n "${MCP_PID:-}" ]] && kill -0 "$MCP_PID" 2>/dev/null; then
        kill "$MCP_PID" 2>/dev/null || true
        wait "$MCP_PID" 2>/dev/null || true
    fi
    if [[ -s "$MCP_STDERR_LOG" ]]; then
        echo "--- MCP server stderr log ---" >&2
        cat "$MCP_STDERR_LOG" >&2
        echo "--- end MCP server stderr ---" >&2
    fi
    rm -f "$MCP_STDERR_LOG"
}
trap cleanup EXIT

# ---------------------------------------------------------------------------
# 1. Check prerequisites
# ---------------------------------------------------------------------------
check_prereqs() {
    local missing=()
    for cmd in node gdb-multiarch qemu-system-arm; do
        if ! command -v "$cmd" &>/dev/null; then
            missing+=("$cmd")
        fi
    done
    if [[ ${#missing[@]} -gt 0 ]]; then
        echo "ERROR: missing prerequisites: ${missing[*]}" >&2
        echo "Install them and re-run." >&2
        exit 1
    fi
    if [[ ! -f "$MCP_SERVER" ]]; then
        echo "ERROR: MCP server not found at $MCP_SERVER" >&2
        echo "Set \$GDB_MCP_SERVER or clone gdb-mcp-server beside this repo." >&2
        exit 1
    fi
    echo "Prerequisites OK: node, gdb-multiarch, qemu-system-arm, MCP server"
}

# ---------------------------------------------------------------------------
# 2. Build svc_yield example with debug symbols
# ---------------------------------------------------------------------------
build_elf() {
    echo "Building svc_yield example (debug)..."
    cargo build \
        --manifest-path "$KERNEL_DIR/Cargo.toml" \
        --target "$TARGET" \
        --example svc_yield \
        --features qemu 2>&1
    if [[ ! -f "$ELF" ]]; then
        echo "ERROR: ELF not found at $ELF" >&2
        exit 1
    fi
    echo "Build OK: $ELF"
}

# ---------------------------------------------------------------------------
# 3. JSON-RPC helpers
# ---------------------------------------------------------------------------

# Format a JSON-RPC request string. Args: method [params_json]
# Increments global REQ_ID and prints the JSON line.
format_request() {
    local method="$1"
    local params="${2:-\{\}}"
    local id=$REQ_ID
    REQ_ID=$((REQ_ID + 1))
    printf '{"jsonrpc":"2.0","id":%d,"method":"%s","params":%s}\n' \
        "$id" "$method" "$params"
}

# Format a JSON-RPC notification (no id, no response expected).
format_notification() {
    local method="$1"
    printf '{"jsonrpc":"2.0","method":"%s"}\n' "$method"
}

# Read a single JSON-RPC response line from the coprocess stdout.
# Times out after $1 seconds (default 15). Prints the raw JSON line.
read_response() {
    local timeout_sec="${1:-15}"
    local line=""
    if read -r -t "$timeout_sec" line <&"${COPROC[0]}"; then
        echo "$line"
    else
        echo "ERROR: timeout (${timeout_sec}s) waiting for MCP response" >&2
        return 1
    fi
}

# Extract one or more fields from a JSON string using python3.
# Accepts bracket-notation paths: '["result"]["protocolVersion"]'
# Multiple paths may be given; one output line per path.
# Usage: json_extract '{"result":{"protocolVersion":"1.0"}}' '["result"]["protocolVersion"]'
json_extract() {
    local json="$1"
    shift
    python3 -c "
import json, re, sys

obj = json.loads(sys.argv[1])
for path in sys.argv[2:]:
    node = obj
    # Walk bracket-notation keys: [\"key\"] or [0]
    for m in re.finditer(r'\[([^\]]+)\]', path):
        key = m.group(1)
        # Strip surrounding quotes for string keys
        if key.startswith('\"') and key.endswith('\"'):
            key = key[1:-1]
        elif key.isdigit():
            key = int(key)
        node = node[key]
    print(node)
" "$json" "$@"
}

# Check if a JSON response contains an error key.
# Returns 0 (true) if error present, 1 otherwise.
json_has_error() {
    local json="$1"
    echo "$json" | grep -q '"error"'
}

# Send a JSON-RPC request to the MCP coprocess and capture the response.
# Args: method [params_json] [timeout_sec]
# Prints the response JSON. Returns non-zero on timeout or error.
mcp_call() {
    local method="$1"
    local params="${2:-\{\}}"
    local timeout_sec="${3:-15}"

    local req
    req="$(format_request "$method" "$params")"
    echo "$req" >&"${COPROC[1]}"

    local resp
    resp="$(read_response "$timeout_sec")" || return 1

    if json_has_error "$resp"; then
        echo "ERROR: MCP error response: $resp" >&2
        return 1
    fi
    echo "$resp"
}

# Send a tools/call request for a named tool.
# Args: tool_name arguments_json [timeout_sec]
mcp_tool_call() {
    local tool="$1"
    local args="${2:-\{\}}"
    local timeout_sec="${3:-15}"
    local params
    params=$(printf '{"name":"%s","arguments":%s}' "$tool" "$args")
    mcp_call "tools/call" "$params" "$timeout_sec"
}

# Extract the text content from a tools/call response.
mcp_tool_text() {
    local resp="$1"
    json_extract "$resp" '["result"]["content"][0]["text"]'
}

# ---------------------------------------------------------------------------
# 4. Start MCP server coprocess and perform initialization handshake
# ---------------------------------------------------------------------------
start_mcp() {
    echo "Starting MCP server coprocess..."
    coproc node "$MCP_SERVER" 2>"$MCP_STDERR_LOG"

    MCP_PID="$COPROC_PID"
    echo "MCP server PID: $MCP_PID"

    # Send initialize request with retries (server may need time to start).
    local init_params
    init_params='{"protocolVersion":"2024-11-05","capabilities":{},"clientInfo":{"name":"validate-gdb","version":"0.1.0"}}'

    local max_attempts=5
    local attempt=1
    local resp=""

    echo "Sending initialize request..."
    while (( attempt <= max_attempts )); do
        if ! kill -0 "$MCP_PID" 2>/dev/null; then
            echo "ERROR: MCP server exited before initialization" >&2
            echo "Server stderr:" >&2
            cat "$MCP_STDERR_LOG" >&2
            exit 1
        fi

        local req
        req="$(format_request "initialize" "$init_params")"
        echo "$req" >&"${COPROC[1]}" 2>/dev/null || true

        if resp="$(read_response 3)" 2>/dev/null && [[ -n "$resp" ]]; then
            if ! json_has_error "$resp"; then
                break
            fi
        fi

        echo "  attempt $attempt/$max_attempts: no response, retrying..." >&2
        sleep 0.2
        attempt=$((attempt + 1))
    done

    if [[ -z "$resp" ]] || json_has_error "$resp"; then
        echo "ERROR: MCP initialize failed after $max_attempts attempts" >&2
        echo "Server stderr:" >&2
        cat "$MCP_STDERR_LOG" >&2
        exit 1
    fi

    local proto
    proto="$(json_extract "$resp" '["result"]["protocolVersion"]')"
    echo "MCP initialized: protocol $proto"

    # Send initialized notification (no response expected)
    local notif
    notif="$(format_notification "notifications/initialized")"
    echo "$notif" >&"${COPROC[1]}"
    echo "MCP handshake complete."
}

# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
echo "=== GDB MCP Validation ==="
check_prereqs
build_elf
start_mcp
echo ""
echo "Setup complete. MCP server ready (PID $MCP_PID)."
echo "ELF: $ELF"
