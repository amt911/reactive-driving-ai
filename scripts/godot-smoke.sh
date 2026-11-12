#!/usr/bin/env bash
# usage: scripts/godot-smoke.sh -> builds the rdai GDExtension and drives one lap in a real, headless
# Godot 4 (examples/godot/smoke.gd). Exit 0 only on "ok: lap ...". No Godot binary is red, not a skip:
# a run that compared nothing proves nothing (docs/agents/lessons.md). GODOT=/path/to/godot to override.
set -euo pipefail
cd "$(dirname "$0")/.."
godot=${GODOT:-godot}
command -v "$godot" >/dev/null || { echo "FAIL: no Godot binary ($godot); install Godot 4.2+ or set GODOT" >&2; exit 1; }
scripts/heavy.sh cargo build --release -q -j 8 -p rdai-godot
# Godot loads extensions listed here. Written by hand: `godot --import` would also write it, but the
# editor dumps core on exit with an extension loaded (Godot 4.7.2, docs/FINDINGS.md).
mkdir -p examples/godot/.godot
echo "res://rdai.gdextension" > examples/godot/.godot/extension_list.cfg
out=$(timeout 300 "$godot" --headless --path examples/godot --script res://smoke.gd 2>&1) || { echo "$out" | tail -5 >&2; exit 1; }
line=$(grep '^ok: lap' <<<"$out") || { echo "$out" | tail -5 >&2; exit 1; }
echo "$line"
