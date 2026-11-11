#!/usr/bin/env bash
# Coverage gate: 80% lines over rdai-core + rdai-sim (AGENTS.md, Tests). The viewer and the bench CLI
# glue are excluded (docs/agents/testing.md says why). Runs inside the memory ceiling.
set -euo pipefail
cd "$(dirname "$0")/.."
exec scripts/heavy.sh cargo llvm-cov --release -p rdai-core -p rdai-sim -j 8 \
  --ignore-filename-regex 'rdai-sim/src/bin/' --fail-under-lines 80 "$@"
