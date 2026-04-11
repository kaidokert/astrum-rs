#!/usr/bin/env bash
#
# hal-clone.sh — re-clone HAL forks for local debugging.
#
# Pulls the upstream HAL into a local working directory, adds the
# kaidokert fork as a second remote, checks out the matching tag, and
# prints the [patch.crates-io] snippet to drop into a workspace
# Cargo.toml so cargo uses the local checkout instead of the published
# crate or git dependency.
#
# Usage:
#   hal-clone.sh stm32f4xx-hal [DEST_DIR]
#   hal-clone.sh atsamd        [DEST_DIR]
#   hal-clone.sh nrf52-hal     [DEST_DIR]
#
# DEST_DIR defaults to ./ref/<hal>.
#
# After cloning, edit the workspace Cargo.toml (boards/Cargo.toml or the
# board's own Cargo.toml) and add the printed snippet. Don't commit it —
# the patch is for ad-hoc debugging only.

set -euo pipefail

HAL=${1:-}
DEST=${2:-}

if [ -z "$HAL" ]; then
    cat <<'USAGE' >&2
usage: hal-clone.sh <stm32f4xx-hal|atsamd|nrf52-hal> [DEST_DIR]

  stm32f4xx-hal  → clones stm32-rs/stm32f4xx-hal
                   adds kaidokert/stm32f4xx-hal as 'fork'
                   checks out tag astrum-mpu-v1
  atsamd         → clones atsamd-rs/atsamd
                   adds kaidokert/atsamd as 'fork'
                   checks out branch splitsamg55_1
  nrf52-hal      → clones nrf-rs/nrf-hal (no kaidokert fork needed)
USAGE
    exit 1
fi

case "$HAL" in
    stm32f4xx-hal)
        ORIGIN_URL=https://github.com/stm32-rs/stm32f4xx-hal.git
        FORK_URL=https://github.com/kaidokert/stm32f4xx-hal.git
        CHECKOUT=astrum-mpu-v1
        CRATE_NAME=stm32f4xx-hal
        SUB_PATH=
        ;;
    atsamd)
        ORIGIN_URL=https://github.com/atsamd-rs/atsamd.git
        FORK_URL=https://github.com/kaidokert/atsamd.git
        CHECKOUT=fork/splitsamg55_1
        CRATE_NAME=atsamd-hal
        SUB_PATH=hal
        ;;
    nrf52-hal|nrf-hal)
        ORIGIN_URL=https://github.com/nrf-rs/nrf-hal.git
        FORK_URL=
        CHECKOUT=
        CRATE_NAME=nrf52833-hal
        SUB_PATH=nrf52833-hal
        ;;
    *)
        echo "Unknown HAL: $HAL" >&2
        echo "Try: stm32f4xx-hal | atsamd | nrf52-hal" >&2
        exit 1
        ;;
esac

DEST=${DEST:-ref/$HAL}

if [ -e "$DEST" ]; then
    echo "Destination already exists: $DEST" >&2
    echo "Remove it or pick a different DEST_DIR." >&2
    exit 1
fi

mkdir -p "$(dirname "$DEST")"
echo "→ Cloning $ORIGIN_URL → $DEST"
git clone "$ORIGIN_URL" "$DEST"

cd "$DEST"

if [ -n "$FORK_URL" ]; then
    echo "→ Adding fork remote: $FORK_URL"
    git remote add fork "$FORK_URL"
    git fetch fork --tags
fi

if [ -n "$CHECKOUT" ]; then
    echo "→ Checking out $CHECKOUT"
    git checkout "$CHECKOUT"
fi

PATCH_PATH="$DEST"
[ -n "$SUB_PATH" ] && PATCH_PATH="$DEST/$SUB_PATH"

cat <<EOF

✓ $HAL ready at $DEST
  origin : $ORIGIN_URL
$([ -n "$FORK_URL" ] && printf '  fork   : %s\n' "$FORK_URL")
$([ -n "$CHECKOUT" ] && printf '  HEAD   : %s\n' "$CHECKOUT")

To redirect cargo at this checkout, add to the workspace Cargo.toml.

  # If the board uses a crates.io dep:
  [patch.crates-io]
  $CRATE_NAME = { path = "$PATCH_PATH" }

  # If the board uses a git dep (e.g. boards/f429zi pins kaidokert/stm32f4xx-hal):
  [patch."https://github.com/kaidokert/$HAL.git"]
  $CRATE_NAME = { path = "$PATCH_PATH" }

Run 'cargo update -p $CRATE_NAME' afterwards to refresh the lockfile.
Don't commit the patch — it's for local debugging only.
EOF
