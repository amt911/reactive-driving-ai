#!/usr/bin/env bash
# usage: scripts/eval.sh [BENCH_BINARY] -> one summary line over a matrix of bench runs.
# Single races are chaotic: one incident changes every number after it. Comparing variants on a
# matrix (tracks x 9 seeds x field sizes x scenarios, 243 runs; SEEDS="..." to change) is what tells a real change from noise
# (docs/FINDINGS.md). Runs 4 at a time.
# Prints: runs, walls, contacts capped at 50 per run (two cars stuck together count thousands of
# contacts and would hide everything else), runs with a pile-up (> 50), contacts after the scenario
# (capped the same way), DNFs and the mean best lap. Exits 1 if any run printed no result table —
# a run that compared nothing is red.
set -euo pipefail
cd "$(dirname "$0")/.."
bench=$(realpath "${1:-target/release/bench}")
jobs=$(for track in default proc:1 proc:2; do for seed in ${SEEDS:-1 2 3 4 5 6 7 8 9}; do for cars in 6 8 10; do
  for scenario in race crash debris; do echo "$track $seed $cars $scenario"; done; done; done; done)
one() {
  read -r track seed cars scenario <<<"$1"
  out=$("$bench" --cars "$cars" --laps 2 --scenario "$scenario" --seed "$seed" --track "$track")
  rows=$(grep -cE '^ +[0-9]+ ' <<<"$out" || true)
  [ "$rows" -eq "$cars" ] || { echo "BAD $1: $rows result rows"; return; }
  a=$(sed -nE 's/.*contacts after scenario: ([0-9]+).*/\1/p' <<<"$out")
  awk -v a="$a" '/^ +[0-9]+ /{w+=$6; c+=$7; if($5=="DNF")d++; else {l+=$4; n++}}
    END{print "OK", w+0, c+0, a+0, d+0, l+0, n+0}' <<<"$out"
}
export -f one; export bench
results=$(xargs -P 4 -I{} bash -c 'one "$@"' _ {} <<<"$jobs")
if grep -q '^BAD' <<<"$results"; then grep '^BAD' <<<"$results" >&2; exit 1; fi
awk '{runs++; w+=$2; c+=($3>50?50:$3); pile+=($3>50); a+=($4>50?50:$4); d+=$5; l+=$6; n+=$7}
  END{printf "runs %d walls %d contacts %d pileups %d after_scenario %d dnf %d mean_best_lap %.2f\n", runs, w, c, pile, a, d, (n?l/n:0)}' <<<"$results"
