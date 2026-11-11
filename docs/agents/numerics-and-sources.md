# Numerics, determinism and sources

> Part of `AGENTS.md`, kept in its own file so `AGENTS.md` fits the 32 KiB Codex reads by
> default. Its rules bind all the same: `AGENTS.md` lists the hard ones in *Rules by topic* and
> says when to read this file. Edit the rule here, not a copy of it.

For code that computes numbers — simulation, signal processing, finance, scoring — or models
something that exists outside the repo. Delete the parts that don't apply and say so in `AGENTS.md`.

## Deterministic core

- **The core is pure and deterministic.** No IO, no threads that change results, no global RNG: a
  seeded RNG is passed in. The same input gives the same bytes, so a test can pin an exact value and a
  regression is a diff, not a flake.
- **A flaky comparison is fixed in the harness, never retried until green** — fixed seeds, recorded
  input, no wall-clock dependence inside the compared window.
- **Fixed precision per layer.** `f32` throughout `rdai-core` (game engines hand us `f32`; lap time and
  track length are accumulated in `f64` where error grows), and say so; say which in the doc comment of the type.
- **Fast and light is a measurement.** When a change touches the hot loop, run the benchmark or the
  timed headless run before and after, and put both numbers in the PR.

## Units and data formats

- **One unit system internally** (SI: m, s, rad, m/s, m/s²; right-handed 2D, angles CCW from +X). Conversions happen only at the UI /
  config boundary, through one place (the engine wrapper, or the FFI comment in `include/rdai.h`). Field names carry the unit when it is ambiguous
  (`bore_m`, `rate_hz`).
- **Definitions are data, never code** (track CSV in TUMFTM format; driver profiles as plain structs). Every field has a default so
  old files keep loading; a breaking format change bumps `version` and adds a migration with a test.
- **No `unwrap()`-style panics outside tests and startup** (Rust: `unwrap`/`expect`; across the FFI no panic may unwind — return an error code): libraries
  return typed errors.

## Testing numbers

- **Tolerances are explicit and explained.** Every approximate comparison carries its tolerance and a
  comment saying where it comes from (the model's order of accuracy, a datasheet, a measured
  spread). Never widen one to make a test pass (see *Never weaken a gate* in
  [lessons](lessons.md)).
- **Oracles come from outside the code under test:** a closed form, a textbook value, a published
  measurement, an earlier trusted version. Not a recomputation by the same formula.
- **Property tests for invariants:** conservation (mass, energy, count), bounds (a position stays in
  its range), round trips (`parse(print(x)) == x`), and "no NaN / infinity for any valid input".
- **Silence, zero and the extremes are cases:** empty input, one element, the maximum, a step
  change.

## Sources and research

- **Cite the source of every model and constant** in its doc comment: paper or book plus equation or
  section number, in the same form the research notes use. A constant without a source is a guess,
  and is labelled one.
- **Licence hygiene.** Don't copy GPL (or otherwise incompatible) code into a differently-licensed
  repo: read it, then reimplement. Permissive code may be ported with attribution in `NOTICE`.
- **Papers and data you can't fetch go in a request list for the user**, not into guesses:
  `PAPERS.md`, newest block first, each entry dated and ordered by how much it would help, with the
  exact search terms (in the source language when it isn't English) and what you will do with it.
  Fetched files go under `papers/` (gitignored unless redistributable) with a note of what they settled.
