# reactive-driving-ai — Agent Guide

A reactive, general-purpose driving AI (racing, traffic, obstacles) written in Rust, deterministic and
microseconds per agent and tick, reusable from any game engine (Godot, Unreal, Unity) through a C ABI.
It rewrites the driving AI of Andrés's degree project (an Unreal Engine 5.1 racing-management
simulator); a 2D reference simulation and viewer stand in for the engine.

## Start here

- **Run `/graphify` before each session.** The persistent graph at `graphify-out/graph.json`
  summarizes architecture, dependencies and cross-cutting concepts without re-reading the repo.
- **Read `docs/ALGORITHMS.md`** before touching `rdai-core`: what the original project did, what
  replaced it and why (decided: no neural network in the core).
- **Read `docs/FINDINGS.md` before debugging or touching the build** — non-obvious gotchas.
  **Convention:** when you discover something non-obvious that cost time and isn't deducible from the
  code, add a short entry to `docs/FINDINGS.md`.
- **`docs/FACTS.md` is the working-memory file, not a second FINDINGS** — verified facts about this
  repo that every fresh agent would otherwise rediscover (real selectors, which fakes exist, what a
  helper accepts). Read it when you start, append to it when you finish. See
  [Agent orchestration](docs/agents/agent-orchestration.md#agent-orchestration--parallel-where-its-free-batched-where-its-yours).
- **`include/rdai.h` is the authoritative C ABI.** It is written by hand: keep it in sync with
  `crates/rdai-ffi/src/lib.rs` in the same change, bump `RDAI_ABI_VERSION` on any breaking change, and
  run `scripts/c-smoke.sh`.

## ⚡ graphify — use every session

```text
/graphify            # first run (builds graph from scratch)
/graphify --update   # incremental update (only re-extracts changed files)
/graphify query "<question>"    # architecture questions instead of opening multiple files
/graphify explain "<name>"      # locate a concept or symbol
/graphify path "A" "B"          # dependency path between two modules
```

Outputs in `graphify-out/`: `graph.json` (source of truth), `GRAPH_REPORT.md` (god nodes,
communities, surprising connections), `graph.html` (interactive view).

Run `/graphify --update` at end of session if you touched docs or images (code changes rebuild via
hook if installed).

## ⚡ superpowers — use whenever applicable

Always prefer **superpowers** skills over ad-hoc approaches. If there's even a small chance a skill
applies to the task, invoke it via the `Skill` tool before acting (including before clarifying
questions).

- **Process skills first** — `brainstorming` before creative/feature work, `systematic-debugging`
  before fixing bugs, `test-driven-development` before writing implementation.
- **Then implementation skills** — domain-specific skills guide execution.
- **Verify before claiming done** — `verification-before-completion` / `requesting-code-review`
  before merging.

Flow: `brainstorming → spec (you approve) → writing-plans → plan (you approve) →
subagent-driven-development → finishing-a-development-branch`. **Nothing is implemented without an
approved spec.**

User instructions always take precedence over skills; skills override default behavior. **Skills
refine *how* the work is done; they never override the rules in this file. When a skill and this
`AGENTS.md` conflict, this file wins.**

### Mode switch

- **"lite mode"** — fully disables superpowers: no skill is invoked, not even the applicability
  check, until **"normal mode"** is said.
- **"normal mode"** (default) — standard superpowers behavior, plus: when delegating coding work,
  dispatch at most 1 agent at a time, and never use a model above Sonnet (no Opus). The cap counts
  **implementation** agents: a read-only review agent runs alongside one, and should — see
  [Agent orchestration](docs/agents/agent-orchestration.md#agent-orchestration--parallel-where-its-free-batched-where-its-yours).
- **"modo desatendido"** (unattended mode) — the user is away and delegates autonomy: work without
  waiting for confirmations and make reasonable decisions yourself instead of asking. In this mode you
  MAY **`git push` the feature branches you create** and **open PRs via `gh`** on your own, so the
  work is ready for review when the user returns. The hard limits still hold and are NOT lifted:
  **never merge anything** (no `git merge`, no fast-forward integration, no `gh pr merge`), **never
  push to `main`** or any protected/default branch directly, and **never** `git push --force` /
  `--force-with-lease`. Deliver everything as pushed branches + PRs for the user to merge. Reverts to
  defaults on **"normal mode"**.
  **Pace in this mode:** intermediate tasks run only the tests of what they touched
  (`vitest related <files>`, `jest --findRelatedTests <files>`,
  `./gradlew testDebugUnitTest --tests '<package>.*'`, `pytest <path>`, here `cargo test -p <crate> <filter>`) plus that task's E2E specs;
  commits pile up locally and the branch is pushed **once, at the end** — the push that runs the full
  pre-push hook, preceded by the final full E2E pass. Each intermediate push paid the whole hook
  (minutes) to report nothing the next one would not.
- **"modo pocos tokens"** (low-token mode) — the user's token quota is short (it resets Wednesdays 19:00): same quality and gates, fewer tokens, slower is fine. No subagents (the work and a separate self-review pass done inline, with the same skills), terse messages, no exploratory work beyond the priority the user set, batched tool calls, long output to files (read the tail), progress reports only at milestones. Ends on **"normal mode"**.

Confirm the switch briefly when it happens.

---

## Rules by topic — what always binds, and where the detail lives

This file fits in the 32 KiB Codex reads by default (`wc -c AGENTS.md` ≤ 32768; when it grows, move
detail to `docs/agents/`, never raise the limit). The detail of each topic lives in `docs/agents/`.
**The lines below bind even if you never open the document; open it before working on that topic.**
A rule is edited in its document, not here and there at once — except for its one-line summary in
this list.

- **Tests** → [docs/agents/testing.md](docs/agents/testing.md): stack, per-folder table, conventions.
  The binding rules (pyramid per feature, hard rules, TDD) are below, in *Tests — the rules that bind
  every branch*.
- **Quality beyond coverage** →
  [docs/agents/quality-beyond-coverage.md](docs/agents/quality-beyond-coverage.md). Mutation gate
  60% over the core-logic scope, a ratchet that only goes up; `SURVIVED` and `NO_COVERAGE` mean
  opposite things; expected values written by hand.
- **Real-environment verification** →
  [docs/agents/real-environment-verification.md](docs/agents/real-environment-verification.md). What
  no in-process test can prove (restarts, real server, disk, scheduler) gets a script against the real
  artifact; a check that has never failed is not proven.
- **Debugging** → [docs/agents/debugging.md](docs/agents/debugging.md), before chasing a bug. Measure
  before ablating; more than three reproductions → a shortcut script before the fourth; a review
  finding is not a reproduction; every assertion is seen failing once.
- **CI** → `.github/workflows/ci.yml`: `cargo fmt --check`, `clippy -D warnings`, `cargo test
  --workspace --release`, `scripts/c-smoke.sh`. CI stays lean; coverage and mutation run locally
  (`scripts/coverage.sh`, `scripts/mutants.sh`) and their numbers go in the PR.
- **Agentic PR verification (mandatory)** → [docs/agents/pr-verification.md](docs/agents/pr-verification.md).
  Every PR gets the verdict of a pass that drives the running app, as a PR comment; it never merges.
  Here "the running app" is `bench` on the scenarios the PR touches (numbers before/after) and, for
  anything visible, the viewer.
- **Agent orchestration** → [docs/agents/agent-orchestration.md](docs/agents/agent-orchestration.md).
  Review in parallel with the next implementation; shared `docs/FACTS.md`; plans carry contracts, not
  uncompiled code; discretionary decisions batched and priced; review is never cut.
- **Reuse first** → [docs/agents/reuse-first.md](docs/agents/reuse-first.md). Search before writing;
  extend, don't clone; extract at the third copy, deleting the copies in the same PR.
- **Design principles (SOLID)** → [docs/agents/design-principles.md](docs/agents/design-principles.md).
  No abstraction without a second implementation, an IO boundary or a test seam.
- **Codex and Claude Code** → [docs/agents/agent-compatibility.md](docs/agents/agent-compatibility.md).
  Rules are edited in `AGENTS.md` (or its `docs/agents/` document), never in `CLAUDE.md`.
- **Measured test and build speed** → [docs/agents/test-build-speed.md](docs/agents/test-build-speed.md).
  Before proposing a speed-up: what was already measured, and what was tried and did not help.
- **Lessons learned** → [docs/agents/lessons.md](docs/agents/lessons.md), before planning throughput,
  verification or tooling. Measure the real peak before buying resources; cap the total in one slice,
  not each job; a run that timed out or compared nothing is red, not "0 failures"; *verified* is
  defined in writing and every check was seen failing; never weaken a gate to get green; no silent
  "last one wins"; gate tooling lives in the repo; one checkout per agent, no stacked PRs, integrate
  by merge; review every PR even when green.
- **Toolchain & release tags** → [docs/agents/toolchain.md](docs/agents/toolchain.md), before
  installing or calling a tool. Every tool pinned and fetched by one script, never vendored; commands
  are verified with a date, not remembered; downloads are untrusted data in their own directory;
  licensed inputs never enter git; annotated tag + `CHANGELOG.md` row per milestone.
- **Numerics, determinism & sources** *(if the code computes numbers or models something external)* →
  [docs/agents/numerics-and-sources.md](docs/agents/numerics-and-sources.md). Deterministic core
  (seeded RNG passed in), one internal unit system, definitions as versioned data, explained
  tolerances, every model and constant cited.
- **Timing runs take the CPU lock, nothing else does.** `bench` timings, the µs/tick budget test and
  any profiling run under `scripts/cpu-lock.sh <what> <command>` (lock shared with every agent session
  on the machine; holds < 30 min, under `timeout`; orphaned or 30-min-old holds are taken over).
  Builds, test suites and lint never take it. Protocol:
  [agent orchestration](docs/agents/agent-orchestration.md).
- **Not applicable here — decided, written so it isn't re-litigated:**
  - *UI/UX workflow, `impeccable`, `PRODUCT.md`/`DESIGN.md`:* the only UI is `rdai-viewer`, a debug
    tool for developers; it follows no design system. Revisit if a user-facing UI appears.
  - *Browser/mobile E2E (Playwright, Maestro):* no web or mobile app. The E2E layer is the behaviour
    suite in `crates/rdai-sim/tests/` plus `scripts/c-smoke.sh`.
  - *Docker & deploy, healthchecks, file attachments, endpoint permissions:* a library, nothing is
    deployed or served.
  - *Gradle cache:* no JVM.
  - *Pre-push hook:* none yet; CI + the PR checklist carry the gates. Revisit if a red CI recurs.


## 🧠 Heavy jobs run inside a memory cgroup (MANDATORY)

**No exceptions:** any long or parallel job started here — the full test suite, coverage, mutation
testing, a release build, a long `bench` run, anything that spawns workers —
runs under a kernel-enforced memory ceiling:

```bash
systemd-run --user --scope --quiet -p MemoryHigh=5G -p MemoryMax=6G -p MemorySwapMax=0 -- <command>
```

**6 GB is the standing ceiling on this machine** (raised from 4 GB by the user on 2026-08-11); don't
exceed it without being told to. `MemoryHigh` throttles and reclaims, `MemoryMax` is the hard stop,
`MemorySwapMax=0` keeps the job from thrashing swap instead of respecting either. Verify it is
actually in force rather than assuming:
`systemctl --user show <scope> -p MemoryMax -p MemoryHigh -p MemoryCurrent`.

**Cap the tool too — but never *instead* of the cgroup.** Pass the tool's own concurrency limit
(`--concurrency`, `--maxWorkers`, `workers`, `--parallel`) so the job isn't throttled to a crawl by
the ceiling. A tool's default concurrency is not a budget, and an estimate of per-worker RSS is not a
ceiling. Only the cgroup is.

**Check that the ceiling reaches the process that does the work.** A job wrapped in the scope can
hand the real work to a **daemon or worker pool that lives outside it** — a build daemon reconnected
from a previous run, a container engine, a language server, a test runner attaching to workers that
were already up. The wrapper still reports the limit as applied, over a process that isn't doing
anything. The tell is `MemoryCurrent` sitting near zero while the machine swaps. Confirm against the
**worker's** cgroup, not the scope's: `cat /proc/<worker-pid>/cgroup`. If the worker is outside,
either kill the daemon so the job starts its own inside the scope, or configure the daemon's own
limit — a wrapper that reports success over an idle process is worse than no wrapper, because it
buys confidence and delivers nothing.

`scripts/heavy.sh <command>` wraps a command in this scope; cap cargo too (`-j 8`, `cargo mutants
-j 4`).

**Why this is a rule and not advice:** a mutation-testing run on this 24-core box sized its worker
pool from the core count and spawned **23 workers at ~2.3 GB each** — ~50 GB of demand on 31 GB of
RAM. It took the whole machine down hard enough that the user had to power-cycle it; `systemd-oomd`
did not save it. The run before that was wasted too: with the machine starving, **139 of the first
142 mutants "timed out"**, and a timeout is scored as *killed*, so the result came out inflated by
starvation and meant nothing. A job that OOMs the box doesn't merely fail — it also hands you
numbers you'd trust by mistake.

---

## Stack

| Area | Choice | Role |
| --- | --- | --- |
| Language | Rust (stable, edition 2021) | everything |
| Core | `rdai-core`, **no dependencies** | route model, racing line, speed profile, planner, control, driver |
| Reference physics | `rdai-sim` (depends only on `rdai-core`) | dynamic bicycle with friction ellipse, `World`, `bench` binary |
| Engine boundary | `rdai-ffi` → `librdai` (`cdylib` + `staticlib`) + hand-written `include/rdai.h` | C ABI for Godot (GDExtension), Unreal, Unity (P/Invoke) |
| Viewer | `macroquad` 0.4 | 2D debug viewer (`rdai-viewer`) |
| Godot | `godot` (gdext) 0.5 | `rdai-godot`: GDExtension (`RdaiTrack`, `RdaiDriver`), Godot 4.2+ |
| Tests | `cargo test`; `cargo llvm-cov`; `cargo mutants` | unit, behaviour, coverage, mutation |

Planned (add when the phase needs them, then update this table): `criterion` only if `bench`
timings stop being enough.

## Workspace structure

```text
/
├── crates/
│   ├── rdai-core/     # pure, deterministic, f32, no deps: math, track (Frenet s/d, TUMFTM CSV),
│   │                  #   racing_line (min curvature), min_curvature (exact window QP), local_line
│   │                  #   (agent's own line, re-planned ahead), learning (grip learned lap to lap), speed_profile (friction ellipse fwd/bwd),
│   │                  #   planner (Werling-style Frenet lattice + IDM + emergency), control (pure
│   │                  #   pursuit + speed loop), driver (experience/stamina/aggression), agent, rng
│   ├── rdai-sim/      # vehicle.rs, world.rs (N cars, walls, contacts, slipstream, laps), tracks.rs,
│   │                  #   bin/bench.rs; tests/ = behaviour suite
│   ├── rdai-ffi/      # C ABI (lib name `rdai`)
│   ├── rdai-viewer/   # macroquad viewer
│   └── rdai-godot/    # Godot 4 GDExtension (conv.rs = the only Godot<->core conversion)
├── include/rdai.h     # C header, kept in sync by hand with rdai-ffi
├── examples/c/        # smoke.c
├── examples/godot/    # headless Godot project for scripts/godot-smoke.sh
├── scripts/           # c-smoke.sh, godot-smoke.sh, heavy.sh, cpu-lock.sh, coverage.sh, mutants.sh
└── docs/              # ALGORITHMS, ENGINE-INTEGRATION, FINDINGS, FACTS, agents/
```

## Commands

```bash
cargo run --release -p rdai-viewer                       # the 2D viewer (keys in README.md)
cargo run --release -p rdai-sim --bin bench -- --cars 1 --laps 2          # baseline numbers
cargo run --release -p rdai-sim --bin bench -- --scenario crash|debris   # reactive scenarios
cargo run --release -p rdai-sim --bin bench -- --cars 1 --laps 2 --trace t.csv --dump-line l.csv
cargo test --workspace --release --no-fail-fast                       # all tests (behaviour suite needs release)
cargo clippy --workspace --all-targets -- -D warnings    # lint (gate)
cargo fmt --all --check                                  # format (gate)
scripts/c-smoke.sh                                       # C ABI from a real C compiler
scripts/godot-smoke.sh                                   # one lap in a real headless Godot 4 (local)
scripts/coverage.sh                                      # coverage gate (80% lines, rdai-core + rdai-sim)
scripts/mutants.sh                                       # mutation gate over rdai-core
```

---

## Tests — the rules that bind every branch

Stack, the per-crate table and conventions: [docs/agents/testing.md](docs/agents/testing.md).

- **Coverage gate: 80%** lines over `rdai-core` + `rdai-sim` (`scripts/coverage.sh`). Critical logic
  (planner, speed profile) ≥90%. Don't lower the gate — exclude infra with justification (the viewer
  and the `bench` CLI glue, see `docs/agents/testing.md`).
- **Mutation gate: 60% minimum** over `rdai-core`, a ratchet. Until it is first measured it is
  advisory, with the date and score written in `docs/agents/quality-beyond-coverage.md`. See
  [Mutation gate — the 60% floor](docs/agents/quality-beyond-coverage.md#mutation-gate--the-60-floor-and-it-only-goes-up).
- **The "E2E" of this project is the whole stack driving**: behaviour tests in
  `crates/rdai-sim/tests/` (whole laps, scenarios) and `scripts/c-smoke.sh` (the C ABI from C).

### The pyramid per feature — one behaviour test per journey, the rest one layer down

**A new behaviour gets one behaviour test per main journey** (e.g. "a car crashes ahead → the follower
never touches it"), everything else one layer down: unit tests of the module with closed-form
oracles. A driving bug still gets its failing test first, at the lowest layer that reproduces it.

**Why:** a behaviour test pays whole laps of simulation (seconds in release) before it asserts
anything; a unit test of the planner costs milliseconds.

- **When one more behaviour test is right:** what no unit test can answer — interaction between
  cars, the controller and the physics over time, a reaction that depends on the previous ticks. The
  test's doc comment says why it is not a unit test.
- **The journey test still proves the whole contract**: the agent sees the world only through the
  same observations an engine would give it, and the result shows in the world (laps, contacts).
- **Existing tests are not migrated for this rule.** It applies to new work and to what a change touches.

### Running the behaviour suite — the whole suite once at the end, only the reds in between

- **The full suite (`cargo test --workspace --release`) runs once, at the end of the branch, alone**,
  while you write the PR; push and PR only after it is green. While working: only the tests the task
  creates or touches (`cargo test -p rdai-sim --release --test <file> <filter>`).
- **Red pass → rerun only the reds**, then **one** full confirmation pass.
- **Three reds in a row on one test → stop** and read the trace (`bench --trace`) before the fourth try.
- **Fixed seeds, no wall clock** inside the compared window; the µs/tick budget test is the only
  timing test and runs under the CPU lock.

### Hard rules (no exceptions)

- **Never claim done without showing test output.** "It compiles" is not "it works".
- **New public function / type / FFI symbol → needs a test.** No exceptions.
- **A bug fix needs a failing regression test first**, then the fix (see `systematic-debugging`).
- **Never delete, `#[ignore]` or comment out a test to get green.** Fix the code or the test on purpose.
- **Every behaviour test was seen failing under sabotage**, and the sabotage is written in its doc
  comment ([lessons](docs/agents/lessons.md)).
- **Driving changes carry `bench` numbers before and after** in the PR (best lap vs ideal, wall hits,
  contacts, µs/tick) — measured under `scripts/cpu-lock.sh`.
- **Don't lower the 80% gate to ship** — exclude untestable modules in config with a written reason.
- **No driving feature is done without its behaviour test green in release.** A trace that "looks
  right" is not the proof.
- **Never stub the planner or the physics to make a behaviour test pass.** A stubbed run proves the
  stub works, not the driver.
- **Test over mock** — exercise real code; the only seam is the engine boundary (observations in,
  controls out).

### TDD — required for new logic

All new logic in `rdai-core`, `rdai-sim` and `rdai-ffi`:

1. **Red** — write a failing test that describes the behavior.
2. **Green** — implement the minimum to pass.
3. **Refactor** — clean up under green tests.

Exceptions (TDD not required): the viewer's drawing; spikes/exploration — but add tests before merging.

## Working rules

- **Heavy or parallel jobs run inside a memory cgroup** — never launch a suite, build or
  fan-out on a bare estimate; wrap it in
  `systemd-run --user --scope -p MemoryHigh=5G -p MemoryMax=6G -p MemorySwapMax=0 -- <command>`
  (`scripts/heavy.sh`) and cap the tool's own concurrency too. Check `free -h` first: other agents
  share this machine.
- **Use superpowers skills whenever they apply** — invoke via `Skill` before acting; process skills
  before implementation skills.
- **New dependencies: install, then say so** — no prior ask (user, 2026-10-05). Check
  `Cargo.toml` first so one job doesn't get two crates; **`rdai-core` stays dependency-free**; name every new package in the PR
  (which, why, what it replaces).
- **Fast and light by default** — at runtime and in deploys, pick the option with less memory,
  CPU and image size, and say in the PR what it saves. Measure when it matters.
- **The topic rules bind from *Rules by topic*** — reuse first, SOLID, debugging (instrument before
  you ablate, budget the lap, a review finding is not a reproduction), agent orchestration (review N
  alongside N+1, discretionary decisions batched and priced) and the UI workflow each have their
  one-line rules there and their detail in `docs/agents/`. Not repeated here, so they cannot drift.
- **TDD by default** for new logic. Don't merge logic without tests.
- **Every driving behaviour ships with a behaviour test** that runs the whole stack in `rdai-sim`.
  Unit tests green ≠ it drives — the recurring failure mode is a planner that passes its unit tests
  and still hits the wall at lap speed. One test per journey; edge cases go one layer down (see
  [The pyramid per feature](#the-pyramid-per-feature--one-behaviour-test-per-journey-the-rest-one-layer-down)).
- **Don't lower the coverage gate** — exclude with justification instead.
- **`rdai-core` is pure, deterministic and engine-agnostic** — no IO, no threads, no global RNG
  (seeded `rng` passed in), no allocation in the per-tick hot path, no engine types. Rules:
  [numerics and sources](docs/agents/numerics-and-sources.md).
- **SI units internally** (m, s, rad, right-handed 2D, CCW from +X). Engine conventions (Unreal cm,
  left-handed, Z-up) are converted only at the boundary — see `docs/ENGINE-INTEGRATION.md`.
- **No neural network in the core** (decided, `docs/ALGORITHMS.md`): reactions come from the
  planner in the same tick; learning, if ever, only tunes "personality" on top.
- **Cite the source** of every model and constant (`PAPERS.md` entry + equation/section) in its doc
  comment; a constant without a source is labelled a guess.
- **FFI never unwinds a panic into C** — every `extern "C"` function catches it and returns an error
  code; null pointers are checked. No `unwrap()`/`expect()` in libraries outside tests.
- **Measure before optimising** — `bench` µs/tick before and after, under the CPU lock.
- **Keep this file's Stack/Workspace section current** — when you ship something previously marked
  "planned", update the Stack tables and module list in the same change. A stale `AGENTS.md` misleads
  the next session.
- **Commits in English**, Conventional Commits. Scope = crate or folder (`fix(core): …`).
- **Docs in Spanish for people** (`README.md`, `docs/ALGORITHMS.md`, `docs/ENGINE-INTEGRATION.md`,
  `PAPERS.md`); agent docs, code and commits in English.
- **Progress tags** — after each merged milestone, an annotated tag and a `CHANGELOG.md` row
  ([toolchain](docs/agents/toolchain.md#progress-tags-and-the-changelog)).

## Git & GitHub

- **Commits and branches OK** — create commits and new branches whenever it makes sense, without asking first.
- **Never push** *(default)* — no `git push` under any circumstance, and absolutely never
  `git push --force` / `--force-with-lease`. Leave pushing to the user. **Exception:** when
  **"modo desatendido"** is active, you may push the feature branches you create (never `main`/protected
  branches, never force) so PRs are ready for review.
- **Never merge — no permission** — you do NOT have permission to merge anything into any branch, nor to
  merge any pull request. No `git merge`, no fast-forward integration, no `gh pr merge`. This holds in
  every mode, **including "modo desatendido"**. Leave every merge (branches and PRs alike) to the user.
- **GitHub via `gh`** — if the `gh` CLI is available, you may open pull requests, issues, and similar
  (comments, labels, etc.). These don't require pushing on your part beyond what `gh` itself does for an
  already-pushed branch.
- **Branches:** `feat/name`, `fix/description`, `chore/task`. Every change after the first commit
  goes on a branch, in its own worktree (`~/repos/worktrees/reactive-driving-ai-<topic>`), and a PR.
- **Project override (user, 2026-11-11): pushing feature branches, opening PRs and merging them is
  authorized** when the PR is good: CI green (or its only red already exists on `main`, named in
  the PR), the agent's own review pass done and the manual test plan written. Merge with
  `gh pr merge --merge --delete-branch` (merge commit, no squash/rebase). Still forbidden in every
  mode: pushing to `main` directly, force pushes, rewriting `main`'s history.
- **Every PR must include a manual test plan** — when opening a PR, add a **How to test manually**
  section describing the exact steps to exercise the change by hand. For a web page/UI, list the concrete
  routes/URLs to visit (e.g. `/dashboard/settings`), what to click or input, and the expected result.
  Include any setup (seed data, env vars, feature flags, login/role) and, where relevant, edge cases and
  error states to check. Here: the exact `bench` / viewer commands, the keys to press, and what you
  should see.
