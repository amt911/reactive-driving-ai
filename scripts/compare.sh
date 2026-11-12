#!/usr/bin/env bash
# usage: scripts/compare.sh RUNS_A RUNS_B -> paired per-run comparison of two eval.sh RUNS_FILEs.
# Totals over the 243-run matrix move +-15 % on which race happens to deadlock (docs/FINDINGS.md), so
# variants are compared run by run: for each metric, the mean per-run difference B - A and its 95 %
# bootstrap confidence interval (2000 resamples, fixed seed; Efron & Tibshirani 1993; for noisy
# simulation results, Agarwal et al. 2021). A difference whose interval contains 0 is not shown to be
# real. Walls and contacts are capped at 50 per run, as eval.sh caps contacts: one stuck car must not
# carry the total. DNFs are counted, not capped.
set -euo pipefail
command -v gawk >/dev/null || { echo "compare.sh needs gawk (asort)" >&2; exit 2; }
[ $# -eq 2 ] || { echo "usage: $0 RUNS_A RUNS_B" >&2; exit 2; }
key() { awk '{print $8"_"$9"_"$10"_"$11, $2, $3, $5, ($7 ? $6/$7 : 0)}' "$1" | sort; }
join <(key "$1") <(key "$2") | gawk '
  function cap(x) { return x > 50 ? 50 : x }
  {
    n++
    dw[n] = cap($6) - cap($2); dc[n] = cap($7) - cap($3); dd[n] = $8 - $4
    # Mean best lap per run: only when both finished someone.
    if ($5 > 0 && $9 > 0) { m++; dl[m] = $9 - $5 }
  }
  function report(name, d, k,    i, j, s, mean, b, bs, lo, hi, means) {
    if (k == 0) { printf "%-22s no runs\n", name; return }
    s = 0; for (i = 1; i <= k; i++) s += d[i]; mean = s / k
    for (b = 1; b <= 2000; b++) { bs = 0; for (i = 1; i <= k; i++) { j = int(rand() * k) + 1; bs += d[j] } means[b] = bs / k }
    asort(means); lo = means[50]; hi = means[1951]
    printf "%-22s mean B-A %+8.3f  95%% CI [%+8.3f, %+8.3f]  %s\n", name, mean, lo, hi,
      (lo > 0 ? "B higher" : (hi < 0 ? "B lower" : "no difference shown"))
  }
  END {
    srand(1)
    printf "paired runs: %d\n", n
    report("walls/run (cap 50)", dw, n)
    report("contacts/run (cap 50)", dc, n)
    report("DNF/run", dd, n)
    report("mean best lap/run (s)", dl, m)
  }'
