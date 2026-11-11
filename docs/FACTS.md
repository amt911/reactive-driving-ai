# FACTS — verified working memory

Facts verified against this repo that a fresh agent would otherwise rediscover. Not FINDINGS (those
are gotchas not deducible from the code). Append when you finish; stale entries may be deleted.

- 2026-11-11 — `cargo test -p rdai-core -p rdai-sim` passes (15 + 2 tests) before the first commit.
- 2026-11-11 — `cargo build --release -p rdai-viewer` builds in 18 s (deps cached), no warnings; the
  window opens on this machine (Wayland + `DISPLAY=:0`).
- 2026-11-11 — `bench` flags: `--cars N --laps L --track default|oval|FILE.csv --scenario race|crash|debris
  --trace FILE.csv --dump-line FILE.csv` (`crates/rdai-sim/src/bin/bench.rs`).
