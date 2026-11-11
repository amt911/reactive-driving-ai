#!/usr/bin/env bash
# usage: scripts/eval.sh [BENCH_BINARY] -> one summary line over a matrix of bench runs.
# Single 8-car runs are chaotic: one incident changes every number after it. Comparing variants on
# a matrix (tracks x seeds x field sizes x scenarios, 81 runs) is what tells a real change from noise (docs/FINDINGS.md).
# Prints: runs, total walls, total contacts, contacts after the scenario, DNFs, mean best lap, and
# fails (exit 1) if any run printed no result table — a run that compared nothing is red.
set -euo pipefail
cd "$(dirname "$0")/.."
bench=${1:-target/release/bench}
laps=2
runs=0; walls=0; contacts=0; after=0; dnf=0; lapsum=0; lapn=0
for track in default proc:1 proc:2; do
for seed in 1 2 3; do
for cars in 6 8 10; do
  for scenario in race crash debris; do
    out=$("$bench" --cars "$cars" --laps "$laps" --scenario "$scenario" --seed "$seed" --track "$track")
    rows=$(grep -cE '^ +[0-9]+ ' <<<"$out" || true)
    if [ "$rows" -ne "$cars" ]; then
      echo "eval: $cars cars / $scenario printed $rows result rows" >&2
      exit 1
    fi
    read -r w c d l n < <(awk -v laps="$laps" '/^ +[0-9]+ /{w+=$6; c+=$7; if($5=="DNF")d++; else {l+=$4; n++}; if($3>laps+1) print "eval: lap count " $3 " > " laps > "/dev/stderr"} END{print w+0, c+0, d+0, l+0, n+0}' <<<"$out")
    a=$(sed -nE 's/.*contacts after scenario: ([0-9]+).*/\1/p' <<<"$out")
    runs=$((runs + 1)); walls=$((walls + w)); contacts=$((contacts + c)); after=$((after + a)); dnf=$((dnf + d))
    lapsum=$(awk -v a="$lapsum" -v b="$l" 'BEGIN{print a+b}'); lapn=$((lapn + n))
  done
done
done
done
mean=$(awk -v s="$lapsum" -v n="$lapn" 'BEGIN{printf "%.2f", (n ? s / n : 0)}')
echo "runs $runs walls $walls contacts $contacts after_scenario $after dnf $dnf mean_best_lap $mean"
