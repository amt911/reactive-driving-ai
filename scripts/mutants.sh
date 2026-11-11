#!/usr/bin/env bash
# Mutation gate over rdai-core (docs/agents/quality-beyond-coverage.md). Extra args go to cargo mutants,
# e.g. `scripts/mutants.sh --in-diff <(git diff main...HEAD -- crates/rdai-core)` for a PR.
set -euo pipefail
cd "$(dirname "$0")/.."
exec scripts/heavy.sh cargo mutants -p rdai-core -j 4 "$@"
