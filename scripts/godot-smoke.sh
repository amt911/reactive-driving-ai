#!/usr/bin/env bash
# usage: scripts/godot-smoke.sh -> builds the rdai GDExtension and drives one lap in a real, headless
# Godot 4 twice: a 2D kinematic car (examples/godot/smoke.gd) and a real VehicleBody3D on Godot's
# physics (examples/godot/vehicle3d.gd). Exit 0 only on "ok: lap ...". No Godot binary is red, not a skip:
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
for script in smoke.gd vehicle3d.gd; do
  # --fixed-fps: every physics step is 1/60 s, run as fast as the CPU allows (deterministic, no wall clock).
  out=$(timeout 300 "$godot" --headless --fixed-fps 60 --path examples/godot --script "res://$script" 2>&1) ||
    { echo "$script: $(echo "$out" | grep -E 'FAIL|ERROR' | head -3)" >&2; exit 1; }
  line=$(grep -E '^ok: (3D )?lap' <<<"$out") || { echo "$out" | tail -5 >&2; exit 1; }
  echo "$script: $line"
done
