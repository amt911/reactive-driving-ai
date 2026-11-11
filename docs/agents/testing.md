# Tests — stack, per-folder table, conventions

> Part of `AGENTS.md`, kept in its own file so `AGENTS.md` fits the 32 KiB Codex reads by
> default. Its rules bind all the same: `AGENTS.md` lists the hard ones in *Rules by topic* and
> says when to read this file. Edit the rule here, not a copy of it.

## Tests — stack, per-crate table, conventions

- **Unit tests:** `#[cfg(test)] mod tests` at the bottom of each `rdai-core` / `rdai-sim` module —
  closed-form oracles (a circle's curvature, a straight line's speed limit, IDM's equilibrium gap).
- **Behaviour tests ("E2E" of this project):** `crates/rdai-sim/tests/*.rs` run the whole stack —
  track → line → planner → controller → physics — for whole laps and assert on what a spectator would
  see: laps finished, wall hits, contacts, lap time vs ideal. Each one was seen failing under a written
  sabotage (the sabotage goes in the test's doc comment).
- **C ABI:** `crates/rdai-ffi` round-trip test (Rust side) + `scripts/c-smoke.sh` (real C compiler
  against `include/rdai.h`).
- **Coverage** with `cargo llvm-cov`; **mutation** with `cargo mutants` over `rdai-core`.
- **Coverage exclusions** (with justification): `rdai-viewer` (macroquad window, checked by eye and
  by the sim it draws), `rdai-sim/src/bin/bench.rs` (CLI glue; its logic lives in `World`).

### Run before declaring done

| Change touches | Run before claiming success |
| --- | --- |
| `rdai-core` module | `cargo test -p rdai-core` + the behaviour tests (`cargo test -p rdai-sim --release`) |
| driving behaviour (planner, control, speed profile, vehicle) | the above **and** `bench` before/after numbers in the PR |
| `rdai-ffi` or `include/rdai.h` | `cargo test -p rdai-ffi` + `scripts/c-smoke.sh` |
| `rdai-viewer` | `cargo build --release -p rdai-viewer` and run it |
| anything large | `cargo fmt --all --check && cargo clippy --workspace --all-targets -- -D warnings && cargo test --workspace --release && scripts/c-smoke.sh` |

### What to test per crate

| Crate | What | Status |
| --- | --- | --- |
| `rdai-core` | math, Frenet projection, line curvature, speed profile envelope, planner decisions (follow/avoid/emergency), driver mapping | unit tests exist |
| `rdai-sim` | vehicle model limits, world contacts, laps; behaviour tests per scenario | 2 unit tests; behaviour suite pending (task 6) |
| `rdai-ffi` | handle lifetime, null safety, tick round trip | test written, not yet run |
| `rdai-viewer` | none (excluded, see above) | — |

### Operative conventions

- **Behaviour tests run in release** (`--release`): a full lap in debug costs ~10× and tempts
  shortening the run, which weakens the test.
- **Fixed seeds everywhere.** A behaviour test that fails one run in ten is a bug in determinism,
  not something to retry.
- **Numbers in asserts carry their origin** in a comment: the ideal lap from the speed profile, a
  car length, the tolerance's reason.
