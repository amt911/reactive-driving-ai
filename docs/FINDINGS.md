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
- *Seeds matter more than variants (2026-11-11):* `World::new_seeded` / `bench --seed N`;
  `scripts/eval.sh` now runs 3 seeds × 5 field sizes × 3 scenarios (45 runs). Same code, 8 cars:
  20 / 6 / 2 contacts depending on the seed. On 45 runs: `main` 6690 walls / 2140 contacts / 663
  after the scenario / 5 DNF / 85.5 s; this branch **674 / 2050 / 932 / 10 / 75.7 s**; the
  "gentle return" 711 / 2276 / 1050 / 11 / 75.2 s (not 4× worse: within noise). The default-seed
  numbers above flattered car-to-car behaviour: walls improved 10×, contacts did not.
- *Still open (latent):* a car that **starts** off the line, or is beyond 1 m of its previous
  plan (spawn, after avoiding, after a hit), still gets the short return quintic and brakes hard:
  `crates/rdai-sim/tests/planner_offset.rs` (2 m offset at s = 548 → 44 m/s target vs 58.8) and the
  `rdai-ffi` round trip (ellipse, 10 m/s → brakes) are red. Best lap 70.4 s vs the 69.7 s
  (ideal + 10 %) of `tests/solo_laps.rs`.
- *Where the solo lap really loses time (2026-11-12, probe of `last_plan` at s 2300–2700):* the
  car is on the line (|e| < 0.3 m) but every plan starts with slope `de0` ≈ +0.03–0.045 and bends
  back with e'' ≈ −0.004…−0.012, so the envelope caps 37–47 m/s against a line speed of 65–71. That
  slope is the **sideslip**: `heading_error` is body yaw minus track heading, and in the corner
  β = atan(vy/vx) ≈ −0.03…−0.04 cancels it exactly. The planner reads the slide as drifting
  outward. **But fixing it alone makes things worse:** planning from the course (estimated from
  the position delta between ticks; `course-attempt.patch` kept out of git) gave 80.8 s / 7 walls;
  plus course in pure pursuit 86.8 s / 24; pure pursuit from the CG with the course 83.5 s / 25;
  plus consistency tolerance 0.5 / 0.25 m: 98 s. With the course right the car drifts ~1 m outward
  through long corners (pure pursuit on body yaw under sideslip tracks with an offset ≈ β·ld), the
  plan diverges, consistency resets at 1 m and the reset plan brakes mid-corner (spin at s ≈ 650).
  The phantom slope was compensating the tracking drift.
- *Fixed (2026-11-12), three pieces in `Agent::tick`:* (1) the planner gets the **course**
  (direction of the last displacement) instead of the body yaw; (2) a leaky **integral** (gain
  0.03 rad/(m·s), τ 5 s) on the car's offset from its plan, integrated only in Cruise and reset
  otherwise; (3) **trail braking** in Cruise: brake ≤ √(1 − (a_lat/a_max)²), floor 0.3, so braking
  for the road never takes the grip the turn is using. (1)+(2) alone left a lap-1 spin at s ≈ 650:
  drifting off while braking and turning, the return curvature lowered the target, harder braking
  took lateral grip, the car drifted further — (3) breaks that loop. Solo best lap 70.4 → **65.4 s**
  (ideal 63.4), 0 walls, `solo_laps` green. On the 243 runs, against the branch before it:

  | variant | walls | contacts | pile-ups | after scenario | DNF | mean best lap |
  |---|---|---|---|---|---|---|
  | before | 3659 | 2934 | 21 | 1416 | 1 | 59.6 s |
  | (1)+(2) | 3650 | 2321 | 15 | 932 | 0 | 57.6 s |
  | (1)+(2)+(3) **kept** | **3132** | 3243 | 22 | 1294 | 1 | **54.9 s** |
  | (1)+(2)+(3) in every mode | 5328 | 6428 | 88 | 3178 | 2 | 55.3 s |

  (1)+(2) alone had fewer contacts but a wall on the solo lap and a contact in the crash journey;
  (3) is kept because every behaviour test is green with it. Trail braking in every mode doubled
  contacts: braking for a car ahead must stay whole.
  Rejected on the same matrix: the integral in every mode (gain 0.03: contacts 3931, pile-ups 38;
  0.05: walls 5563, contacts 4984 — it winds up during avoidance); gain 0.1 unstable solo (37
  walls); the gated integral without the course fix (contacts 3212, pile-ups 32); a per-candidate
  cost for the speed lost to the candidate's own curvature plus a "hold the current offset"
  candidate (fixes `planner_offset` and the FFI round trip, but walls 32884, contacts 8803:
  candidates flip between hold and return and the steering oscillates into the wall).
- *Still red after ff0e360: `planner_offset`, FFI round trip — the off-line return.* A 2 m return
  over the 0.9 s × 60 m/s = 54 m transition needs e'' ≈ 0.004, which caps ~43 m/s in that corner.
  Tried on ff0e360 (2026-11-12), each judged on the solo lap and an 8-car race (ff0e360: 0 walls
  solo, 1 wall / 0 contacts in 8 cars), all rejected: the curve-loss cost + hold-offset candidate
  again (13 walls solo, 103 / 22 in 8 cars); a second, 3× longer transition per end offset
  (Werling samples several end times; 103 s / 9 walls solo — the long plans lag the car); a
  GripMap-style cost on lateral acceleration beyond the grip at the current speed (11 walls solo,
  56 / 8 in 8 cars). Every candidate-cost fix so far trades the return for wall hits: the cost
  choice flips between ticks and the steering follows. What racing stacks do instead (to read
  before the next try, PAPERS.md): the local planner samples **raceline-relative** trajectories
  with a velocity profile of their own and a feasibility check (A2RL 2026, GripMap 2025), and
  tracks with a model-based controller (MAP, ForzaETH: ~4× pure pursuit's tracking at speed).
  Also tried (2026-11-12): stretching the chosen return after the choice (×1.25 up to the horizon)
  until it costs ≤ 3 % of the parallel path's speed, only with the road clear. Unbounded: 12 walls
  solo, 423 / 40 in 8 cars — **the racing line runs edge to edge, so a slow return leaves the
  track at the exits**; with a bounds check: 1 wall solo, 13 / 2 in 8 cars, first lap slower, and
  `planner_offset` still red at 6 m (a return over the whole horizon still caps the speed). The
  test's premise holds: at its four cases the parallel path stays ≥ 2.9 m inside the track over
  the horizon.

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
**Fix.** Pending for the driving part, tracked by `crates/rdai-sim/tests/planner_offset.rs` (a car
off the line gets a hard return; branch `fix/planner-braking`). The FFI test now checks only the ABI
contract (2026-11-12): return codes, `out` written (NaN sentinels), steer in [-1, 1], throttle and
brake in [0, 1]. Sabotage seen failing: `rdai_agent_tick` not writing `out` -> `steer NaN`.
The test was written but never run before 2026-11-11.
**Why it isn't obvious.** The C smoke script is green, so the ABI looked verified while the
behaviour behind it was wrong — a check that never asserted on the result.

Found: 2026-11-11 · core/planner, ffi · **Status: FFI test split (2026-11-12); planner part open**

### FFI functions can unwind a panic into C

**Symptom.** Seen 2026-11-12: an observation with a NaN position (what a physics explosion hands
the AI) panics in the core (`f32::clamp` with NaN bounds), the panic cannot unwind across
`extern "C"`, and the process aborts ("panic in a function that cannot unwind") — the game with it.
**Fix.** Non-finite observations, obstacles, `dt` and control points are refused at the boundary
(-1 / null), and every export runs inside `guard` (`catch_unwind` -> -2 / null / NaN). Null
handles are refused instead of dereferenced. Tests in `crates/rdai-ffi/src/lib.rs`.
**Why it isn't obvious.** Nothing in the sim ever sends NaN, so no behaviour test sees it; only a
host does.

Found: 2026-11-11 · ffi · **Status: fixed (2026-11-12)**

### A faster car did not get past slower ones (fixed on `fix/planner-braking`)

**Symptom.** `fast_car_overtakes_slower_ones`: the faster car finished last; the two cars tangled
for ~25 s at 7–20 m/s after a contact.
**Cause.** The car being passed treated *every* path as a collision while the other car was
alongside ("never steer into it" applied to the path that keeps its lane too), so with no free
path it braked in Emergency right in front of the overtaker.
**Fix.** Alongside, only paths that get closer than the current gap collide. Matrix (81 runs):
walls 2283 → 1261, contacts 943 → 722, after the scenario 560 → 247, DNF 20 → 3.

Found: 2026-11-11 · core/planner · **Status: fixed**
### A car touching the one in front kept accelerating into it

**Symptom.** `bench --cars 10 --seed 2`: 107 consecutive contacts between the same pair; the
follower stayed in Cruise, no brake, while the car in front braked.
**Cause.** With the bumpers already touching, `gap0 ≤ 0` classed the car in front as "alongside";
after the alongside rule stopped counting the lane-keeping path as a collision, nothing made the
follower brake. **Fix.** Centre ahead and overlapping sideways = in front (regression test
`crates/rdai-sim/tests/planner_contacts.rs`). Alone it did not help the matrix (it left cars
stopped in Emergency); with the stuck recovery (#4) the 243-run matrix went walls 5214 → 3659,
contacts 3066 → 2934, pile-ups 28 → 21, after the scenario 1673 → 1416, DNF 26 → 1.

Found: 2026-11-11 · core/planner

### Evaluation noise: one matrix of seeds is not enough

The same binary moved walls 1261 → 2531 and contacts 722 → 1218 between seed sets {1,2,3} and
{4,5,6}. `scripts/eval.sh` now sums 9 seeds (243 runs, ~6 min, 4 in parallel); differences under
~15 % on it are still noise. Earlier claims from one seed or one 81-run matrix were corrected:
the alongside rule is walls −22 %, contacts −8 %, after the scenario −17 %, DNF 45 → 15 on 243
runs (not the −45 % / −23 % first reported). HDM multi-anticipation (3 leaders, 1/j² weights) and
sorting leaders by distance were both worse on the 81-run matrix — not re-run on 243 runs.
Re-tested on 243 runs: the "gentle return" (extra candidate with a transition sized to 25 % of
the grip, only with nothing in range) fixes the FFI round trip but slows every lap (mean best
59.6 → 67.5 s) and raises walls 3659 → 5459 — rejected for good. The solo lap's remaining time
loss (4.6 s vs the profile, 3.3 s of it at s 2400–2800) is small lateral corrections at 60+ m/s
whose quintic curvature caps the speed; it needs a different speed/lateral coupling, not a
longer transition.

Found: 2026-11-11 · tooling

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

### `bench` shows 20+ laps for a 2-lap race

**Symptom.** `bench --laps 2` (and `scripts/eval.sh`) shows 17–21 in the *laps* column for some
cars, with a finish time well before the end of the run.
**Cause.** The column is `lap_times.len()`; cars keep driving after they finish and every further
lap is appended, so a race that waits for a DNF counts dozens. Not an AI fault.
**Fix.** Pending: stop recording laps (or park the car) once `finished_at` is set.

Found: 2026-11-11 · sim

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

## Godot: nodes added in `SceneTree._initialize` enter the tree a frame later

In a `--script` that extends `SceneTree`, `root.add_child(n)` inside `_initialize` is deferred: on
the first `_physics_process` `n.is_inside_tree()` is false and `global_position` reads (0, 0, 0)
(setting it fails). `examples/godot/vehicle3d.gd` waits until the car is in the tree (capped at 10
frames, else FAIL — a script error before `add_child` otherwise hangs the run until `timeout`).
Run headless physics with `--fixed-fps 60`: each step is 1/60 s and runs as fast as the CPU allows.
The `VehicleWheel3D` rest length is `wheel_rest_length` (there is no `suspension_rest_length`).

## Receding-horizon racing line: what made it work (2026-11-12)

The agent re-optimises a 200 m window of its own line every 0.1 s (`local_line.rs`). Measured on
the default circuit, solo, 3 laps (best lap; ideal 63.23 s):
- **Free-ended window** (end not tied to anything, end speed 0): myopic. It does not pay for the
  corner past its end, stays mid-track, reshapes as the corner comes into view, asks for a speed
  that falls faster than the car can brake: 9 walls, 69.9 s. Tie the end to a lap plan.
- **End tied to a gradient-descent plan:** the exact window QP finds a cheaper path than the
  not-quite-converged plan, so every window bends away and back, and the target oscillates 45-67
  m/s. The plan must be a fixed point of the window solver: `RacingLine::build` now polishes with
  the same QP (block Gauss-Seidel).
- **Objective:** the full vector `|Δ²p|^2`. The track-normal component alone zig-zags where the
  centreline has a sharp corner (peak curvature 0.47 on a polyline rectangle vs 0.13), even
  re-linearised on the path normal; it was 1 s faster on the smooth circuit. Weighting by v^2
  (lateral acceleration): ideal 64.99 s, worse.
- **Stitch tolerance:** at 0.5 m ordinary tracking errors re-pinned the window at the car and the
  line chased the car (68.0 s, 2 walls); 1.0-2.5 m: no re-pin solo, 65.12 s, 0 walls.
- **Off the line:** pure minimum curvature from 4 m wide cut across the corner (target 45.3 vs
  line 52.0 m/s); a term on bending relative to the plan (zero on it, so the plan stays a fixed
  point) keeps parallel paths free. Outside the margin, bounds that return at a bounded slope and
  start flat (a linear ramp cost a 0.012 1/m kink, 28-31 m/s).
- **Planner memory is relative to the line:** when the window re-pins at the car, the planner's
  previous choice (+3 m round debris) meant +6 m from the new line and the car steered into the
  debris. `Planner::rebase` keeps it in place. Gating re-pins by planner mode instead moved the
  failure around (crash journey, then 126 contacts in the 8-car race).
- **Sweep artefacts:** checking a window solver by sweeping it round a lap needs the sweep to
  close (overlap the start) and must not overwrite samples behind the window — both showed up as
  kinks that read as a 7 s slower line.
- **A spun car is as wide as it is long (2026-11-12):** the planner took every obstacle's footprint
  as aligned with the track (half width 1.0 m); a car spun across it reaches 2.25 m to each side, and
  followers in the crash journey brushed it at 3.2 m lateral distance. `frenet_extent` boxes the
  rotated rectangle. Matrix (243 runs): contacts after the scenario 960 -> 770, walls 1650 -> 1468.
- **Flying laps, proc:4 (2026-11-12):** two bugs behind one wall, every second lap. (1) Each window's
  two samples behind its start were extended straight *in offset*; the window's curvature is the
  plan's plus e'', so where the plan bends sideways that read as a false 0.0025 1/m kink under the
  car and the target fell 69 -> 58 m/s. They now continue the *deviation* from the plan. (2) With
  that fixed, ~1 m of tracking error at 66 m/s broke temporal consistency (1.0 m) and the planner's
  short return from the measured offset cut the target 67 -> 51 m/s: flat-out braking mid-corner,
  wide, wall. `consistency_tolerance` is now 1.6 m, above the local line's 1.5 m stitch tolerance:
  alone, the line re-pins at the car before the planner would return on its own.
- **Stuck by the wall (2026-11-12):** backing off mirrored the forward steering; wedged between
  debris and the wall, that put the tail into the wall, where the car pushed for the whole reverse
  and then drove back into the same gap (a DNF, 555 wall hits in one matrix run). Reverse now
  steers the nose back to the track direction. Tried and dropped (not needed for the fix, no
  evidence of their own): longer reverses when stuck again in the same place, and a cost for
  passing an obstacle on the other side from the last plan (the choice flipped every tick at
  walking pace). Also: `consistency_tolerance` 1.6 for every car cost 9 DNFs and +3.8 s of mean
  lap in traffic; it is 1.6 only when the car is alone.
- **Open (2026-11-12): walls in traffic after the window-start fix.** The false kink made cars
  slower into some braking zones; without it, a car that brakes flat out in `Avoid` (an obstacle
  past `RESTART_CLEARANCE`) slides wide, counts as alone, and the local line re-pins at it every
  tick: the line follows the car to the wall (default seed 2, 10 cars, debris: s 459-583). Trail
  braking in `Avoid` too was far worse (pile-ups 11 -> 53).
