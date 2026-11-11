# FINDINGS — reactive-driving-ai

The repo's **non-obvious gotchas log** and its **open bugs**. Read it before debugging or touching
the build; add to it whenever something cost time and is not deducible from the code. One `###`
entry per finding, newest first: **Symptom / Cause / Fix / Why it isn't obvious**, then
`Found: <date> · <area>`. Open bugs say **Status: open** and move to *Closed* with the PR that
fixed them.

## Open bugs

### Planner `target_speed` falls faster than the car can brake in the s≈480–640 m braking zone

**Symptom.** Default circuit, 1 car (`bench --cars 1 --laps 2`): in the braking zone at
s≈480–640 m the planner's `target_speed` drops from 57 to 18 m/s in ~75 m — about 20 m/s², which no
car can do — while the racing line's own speed in that zone only goes from 62 to 51 m/s. The car
ends up ~10 m off the line. Baseline for the whole run: best lap **82.6 s vs 63.4 s ideal, 9 wall
hits, ~3.5 µs/tick**.
**Investigation 2026-11-11 (branch `fix/planner-braking`).**
- *Confirmed:* the chosen candidate's envelope (line κ + the return quintic's e'') falls to 17 m/s
  while the line allows 51–61; the minimum is at the first sample (σ ≈ 11 m). Then full braking
  mid-corner saturates the friction ellipse and the car drifts out: a feedback loop.
- *What fixed the driving* (bench, branch vs `main`): **temporal consistency** (the new plan starts
  from the previous plan's lateral state while within 1 m and above 5 m/s, Werling 2010) — before,
  every plan restarted from the measured state, so tracking errors were absorbed into the plan
  instead of corrected — and an **edge clearance cost** (paths within 1 m of the edge cost; before,
  end offsets reached 0.2 m from the wall at no cost). 1 car × 3 laps: 14 → 0 walls, best
  82.6 → 70.4 s; 8 cars: 150 → 3 walls; crash: 139 → 7 walls, contacts after the crash 8 → 1;
  debris: 144 → 5 walls, 5 → 1.
- *Consistency must stop at low speed:* stopped, σ = 0 and a stale slope (towards debris) was
  inherited for ever — two cars sat in Emergency behind the debris (DNF).
- *Tried and rejected* (same bench): speed-loss cost per candidate (101.7 s / 31 walls); + an
  infeasibility cost (78.2 s / 22 walls); aero drag outside the sim's tyre ellipse (no gain);
  infeasibility from σ > 0 (52 walls); three fixed transition times + feasibility at line speed
  (the return is postponed every tick: DNF); a v²·e'' lateral-acceleration cost (87 s); a
  "gentle" transition sized to 25 % of the grip (95 s, crash scenario collapses). A 15 % grip
  margin alone left 34 walls; a 1.5 m line margin did not help.
- *Evaluate on the matrix, not one race:* single 8-car runs are chaotic (one incident changes every
  number after it). `scripts/eval.sh [bench]` sums 15 runs (6–10 cars × race/crash/debris, 2
  laps). 2026-11-11: `main` 2480 walls / 308 contacts / 92 after the scenario / 6 DNF / mean best
  lap 87.5 s; this branch 56 / 28 / 11 / 0 / 76.0 s. A "gentle return" offered only on ticks
  without consistency and without obstacles fixed the FFI round trip and solo lap (69.9 s) but
  gave 61 / **118** / 15 / 0 / 75.7 s — rejected for the contacts.
- *Still open (latent):* a car that **starts** off the line, or is beyond 1 m of its previous
  plan (spawn, after avoiding, after a hit), still gets the short return quintic and brakes hard:
  `crates/rdai-sim/tests/planner_offset.rs` (2 m offset at s = 548 → 44 m/s target vs 58.8) and the
  `rdai-ffi` round trip (ellipse, 10 m/s → brakes) are red. Best lap 70.4 s vs the 69.7 s
  (ideal + 10 %) of `tests/solo_laps.rs`.

**Cause (original suspicion).** Step 4 of `Planner::plan` (`crates/rdai-core/src/planner.rs`)
evaluates `kappa = line.curvature[idx] + q.second(sigma)`. When the car is displaced laterally, the
chosen quintic's second derivative adds curvature that the speed profile never saw, the
`sqrt(a_lat / kappa)` cap collapses, and nothing makes that cap respect the braking envelope as the
horizon slides forward tick by tick.
**Fix.** Pending — investigate with `bench --trace` + `--dump-line` (systematic debugging), regression
test first.
**Why it isn't obvious.** On the line (`e = 0`) the term vanishes and everything looks right; it only
bites once the car is already off the line, which makes it look like a controller problem.

Found: 2026-11-11 · core/planner · **Status: open**

### Agent brakes in `avoid` mode at 10 m/s on an empty ellipse (FFI round-trip test fails)

**Symptom.** `cargo test -p rdai-ffi` fails: `assertion failed: c.throttle > 0.0 && c.steer.abs() <= 1.0`.
The car sits on the centreline of an ellipse (a = 150 m, b = 100 m, half-width 6 m) at 10 m/s with no
obstacles; the agent answers `throttle 0, brake 0.41, mode 2 (avoid)`. `scripts/c-smoke.sh` prints the
same (`steer 0.379 throttle 0.00 brake 0.41 mode 2`) but does not assert on it, so it passes. The
corner allows ~29 m/s (κ = a/b² = 0.015 1/m, 12.7 m/s² lateral), so braking is wrong.
**Cause.** Unknown. Likely the same family as the bug above: the car starts off the racing line
(e₀ ≠ 0, heading error vs the line), so the quintic's curvature enters the speed cap.
**Fix.** Pending. The test was written but never run before 2026-11-11.
**Why it isn't obvious.** The C smoke script is green, so the ABI looked verified while the
behaviour behind it was wrong — a check that never asserted on the result.

Found: 2026-11-11 · core/planner, ffi · **Status: open**

### FFI functions can unwind a panic into C

**Symptom.** None seen yet. `crates/rdai-ffi/src/lib.rs` has no `catch_unwind`: a panic inside
`Agent::tick` (e.g. an index out of range on a malformed track) would unwind across `extern "C"`,
which aborts the process (Rust ≥ 1.81) and takes the game engine down.
**Fix.** Pending: wrap every exported body in `std::panic::catch_unwind` and return an error code
(AGENTS.md, Working rules).

Found: 2026-11-11 · ffi · **Status: open**

### A faster car does not get past slower ones

**Symptom.** `crates/rdai-sim/tests/behaviour.rs::fast_car_overtakes_slower_ones`: a 430 kW / μ 1.3
car starting a row behind two 200 kW / μ 1.1 cars finishes last (laps 75–110 s, 2 contacts, 118
Emergency ticks). Also: a driver at 100 % grip laps no faster than one at 90 % (73–75 s both) —
pace is set by tracking quality, not by the grip the profile allows.
**Cause.** Not investigated yet. Note the test only became meaningful once each car class got its
own racing line: before, every car drove car 0's speed profile (fixed in the same branch).
**Fix.** Pending.

Found: 2026-11-11 · core/planner · **Status: open**

## Findings

### `bench` shows 20+ laps for a 2-lap race

**Symptom.** `bench --laps 2` (and `scripts/eval.sh`) shows 17–21 in the *laps* column for some
cars, with a finish time well before the end of the run.
**Cause.** The column is `lap_times.len()`; cars keep driving after they finish and every further
lap is appended, so a race that waits for a DNF counts dozens. Not an AI fault.
**Fix.** Pending: stop recording laps (or park the car) once `finished_at` is set.

Found: 2026-11-11 · sim

### crates.io downloads of large crates fail with `SSL routines::unexpected eof while reading`

**Symptom.** `cargo build -p rdai-viewer` fails downloading `image-0.24.9` (8.8 MB):
`[56] Failure when receiving data from the peer (OpenSSL SSL_read: OpenSSL/4.1.0: error:0A000126:SSL routines::unexpected eof while reading`.
Small crates download fine and the crates.io website opens, so the network looks healthy.
**Cause.** The connection on this machine drops long TLS streams mid-download (same error with plain
`curl`, with and without the agent sandbox). Not a crates.io outage.
**Fix / workaround.** Resume the download with curl and drop it in cargo's cache after checking the
checksum from `Cargo.lock`:
`curl -L -C - -o x.crate https://static.crates.io/crates/<name>/<ver>/download` (repeat until it
completes), `sha256sum x.crate` = the `checksum` in `Cargo.lock`, then copy to
`~/.cargo/registry/cache/index.crates.io-*/<name>-<ver>.crate`. `cargo fetch` then succeeds.
**Why it isn't obvious.** cargo reports it as a "spurious network error" and retries the whole file,
which fails the same way every time.

Found: 2026-11-11 · env
