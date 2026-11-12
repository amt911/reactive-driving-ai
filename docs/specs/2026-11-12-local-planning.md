# Spec — the AI finds its own line while driving (no precomputed racing line)

Status: draft for approval (2026-11-12). Supersedes the "raceline-relative candidates" fix of PR #2.

## Goal

The agent decides where to drive and how fast **from the road it can see**, every few ticks, starting
from its own state (position, offset, heading, speed). No global racing line is stored or followed.
Phase 2 adds lap-to-lap learning on top. Fixes `planner_offset` (PR #2) by construction: a car 2 m
off "the line" has no line to return to; it re-optimises from where it is.

## Phase 1 — receding-horizon local line (`rdai-core/src/local_line.rs`)

- **Window:** the next `H` m of track ahead of the car (default 300 m = 150 samples at 2 m). `H` is at
  least the braking distance from `max_speed` to 0 (80²/(2·13) ≈ 246 m), so a **terminal speed of 0**
  at the window end is safe without being slow: the car can always stop inside the road it has seen
  (the classic "drive within sight distance" condition, cited in the doc comment).
- **Path:** minimum-curvature offsets over the window (same objective and projected gradient descent as
  `racing_line.rs`, Braghin 2008 / Heilmeier 2019), with the first two samples pinned to the car's
  current offset and heading (course, not body yaw — PR #2's `course_estimate`), end free.
- **Speed:** the existing forward/backward pass (`speed_profile`), open path: forward from the current
  speed, backward from 0 at the window end.
- **Warm start:** each solve starts from the previous solution shifted by the distance travelled, so a
  re-plan every `T` s (default 0.1 s) needs few iterations. Iteration count is a parameter measured in
  the PR (µs/tick budget below).
- **Integration:** a small trait `Reference` (`offset_at`, `speed_at`, `min_speed_ahead`) implemented by
  `RacingLine` and `LocalLine` — two implementations, so the abstraction is allowed (design
  principles). The planner's lattice, IDM and emergency logic keep working against `Reference`
  unchanged; the agent owns a `LocalLine` and stops reading the global one.
- **The global `RacingLine` stays only as a yardstick:** `ideal_lap` in bench/FFI/Godot and the viewer
  overlay. The C ABI and Godot API do not change (no `RDAI_ABI_VERSION` bump).

## Phase 2 — learning lap to lap (deterministic, no neural network)

A per-track-sample memory owned by the agent (a `Vec<f32>` per quantity, sized once):
- **Grip scale** per sample: after a lap, lowered where the car slid/understeered (lateral error or
  wall contact), raised slowly where it used less than the planned lateral acceleration. Bounded,
  e.g. 0.85–1.05 × `max_lat_accel`. Iterative learning control (Arimoto 1984; Kapania & Gerdes 2015
  for racing) — cited when written.
- **Edge margin** per sample: widened where it touched a wall, narrowed slowly where it never came near.
- The local line reads both in its bounds and speed pass. Personality (`experience`) sets the
  learning rate; deterministic for the same seed and inputs.
- Separate PR after phase 1 is merged.

## Acceptance

- `planner_offset` green; `solo_laps` green (best lap within 3% of the global ideal, 0 walls); the
  crash/debris journeys and the 8-car race stay green.
- New behaviour test (phase 1): the agent laps a track **built without calling `RacingLine::build`**
  in its driving path (sabotage: pin the window to the centreline → slower lap / red).
- Phase 2 behaviour test: lap times over 5 laps on a car whose real grip is 10% below its declared
  limits: wall hits fall to 0 and lap time converges (sabotage: learning rate 0 → stays red).
- 243-run `scripts/eval.sh` matrix: walls, contacts, pile-ups not worse than main; numbers in the PR.
- **Budget:** ≤ 20 µs/tick for one car amortised (today 6.9), measured under `scripts/cpu-lock.sh`.
  No allocation per tick (buffers sized at `Agent::new`).

## Out of scope

Obstacles as corridor bounds inside the optimiser (the lattice keeps handling traffic for now);
unknown tracks (the window still reads the `Track`, i.e. a known map — sensing only the visible road is
a later step if wanted).
