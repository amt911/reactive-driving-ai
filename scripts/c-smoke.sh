#!/usr/bin/env bash
# Builds the static library and links the C smoke test against include/rdai.h.
set -euo pipefail
cd "$(dirname "$0")/.."
cargo build --release -q -p rdai-ffi
out="${TMPDIR:-/tmp}/rdai-smoke"
cc -std=c11 -Wall -Wextra -Werror -Iinclude examples/c/smoke.c target/release/librdai.a -lm -lpthread -ldl -o "$out"
"$out"
