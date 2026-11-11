#!/usr/bin/env bash
# usage: scripts/heavy.sh COMMAND... -> runs COMMAND under the machine's memory ceiling (AGENTS.md, "Heavy jobs").
exec systemd-run --user --scope --quiet -p MemoryHigh=5G -p MemoryMax=6G -p MemorySwapMax=0 -- "$@"
