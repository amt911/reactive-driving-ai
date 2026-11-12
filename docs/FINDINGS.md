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
**Cause (suspected, not confirmed).** Step 4 of `Planner::plan` (`crates/rdai-core/src/planner.rs`)
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
**Fix.** Pending for the driving part, tracked by `crates/rdai-sim/tests/planner_offset.rs` (a car
off the line gets a hard return; branch `fix/planner-braking`). The FFI test now checks only the ABI
contract (2026-11-12): return codes, `out` written (NaN sentinels), steer in [-1, 1], throttle and
brake in [0, 1]. Sabotage seen failing: `rdai_agent_tick` not writing `out` -> `steer NaN`.
The test was written but never run before 2026-11-11.
**Why it isn't obvious.** The C smoke script is green, so the ABI looked verified while the
behaviour behind it was wrong — a check that never asserted on the result.

Found: 2026-11-11 · core/planner, ffi · **Status: FFI test split (2026-11-12); planner part open**

### FFI functions can unwind a panic into C

**Symptom.** None seen yet. `crates/rdai-ffi/src/lib.rs` has no `catch_unwind`: a panic inside
`Agent::tick` (e.g. an index out of range on a malformed track) would unwind across `extern "C"`,
which aborts the process (Rust ≥ 1.81) and takes the game engine down.
**Fix.** Pending: wrap every exported body in `std::panic::catch_unwind` and return an error code
(AGENTS.md, Working rules).

Found: 2026-11-11 · ffi · **Status: open**

### Cars stopped against an obstacle never backed off

**Symptom.** In the viewer, cars that hit something sat still against it. Test
`crates/rdai-sim/tests/recovery.rs`: debris touching the nose → stopped in Emergency for 20 s.
**Cause.** Recovery required `target_speed > 3`, but Emergency sets the target to 0; and a 1.5 s
reverse (~3 m) left no room to steer round (a 3.5 m sidestep needs ~10 m at the 5.6 m minimum
turning radius), so the car looped. Reversing was also blind.
**Fix.** Stopped in Emergency counts as stuck; reverse 3 s; stop reversing when anything is within
1.5 m behind. Bench matrix (default seed, 45 runs) vs `main`: DNF 18 → 0, walls 7440 → 4842, but
contacts 924 → 1272 and after the scenario 276 → 453 — partly cars that used to sit out now
racing again; not yet separated.

Found: 2026-11-11 · core/agent

## Findings

### `godot --headless --import` dumps core on exit with a GDExtension loaded

**Symptom.** Godot 4.7.2 (Arch): `godot --headless --path examples/godot --import` does the import
(writes `.godot/extension_list.cfg`), then exits 134 (SIGABRT). Without `rdai.gdextension` in the
project it exits 0. The backtrace (`coredumpctl info`) has no frame in `librdai_godot.so`: a
segfault inside Godot during editor shutdown, turned into abort by Godot's handler.
**Workaround.** The extension does not need the editor: `scripts/godot-smoke.sh` writes
`.godot/extension_list.cfg` (one line, `res://rdai.gdextension`) and runs the script directly;
that run exits 0.
**Why it isn't obvious.** The import looks like the required first step, and the abort comes after
it has already done its work.

Found: 2026-11-12 · godot · **Status: worked around**

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
