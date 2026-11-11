# Lessons learned — throughput, verification, process

> Part of `AGENTS.md`, kept in its own file so `AGENTS.md` fits the 32 KiB Codex reads by
> default. Its rules bind all the same: `AGENTS.md` lists the hard ones in *Rules by topic* and
> says when to read this file. Edit the rule here, not a copy of it.

Distilled from long agent-driven projects (a decompilation with a differential gate, a numeric
simulator). The incidents are real; the rules are project-agnostic. Add your own at the bottom with
the incident and the date: `<YYYY-MM-DD> — <incident> → <rule>`.

## Throughput — what actually limits it

- **Measure before buying resources.** The memory a job *seems* to need is rarely the bottleneck.
  Read the real peak first (`systemd-run --user --wait …` prints it, or
  `systemctl --user show <unit> -p MemoryPeak`) before raising a limit. In one project a full build
  peaked at 0.9 GB under a 6 GB cap: more RAM would not have helped. The real limits were
  serialization (one global lock for every heavy job) and gates that run in real time.
- **Cap the total, not each job.** Put all heavy jobs in one systemd slice with the ceiling
  (`--slice=<name>.slice`, `MemoryMax` on the slice) and let several run at once. A per-job cap plus a
  global lock is safe but serializes everything; a shared slice gives the same safety in parallel.
- **A shared ceiling still needs the big jobs kept apart.** Below `MemoryMax` the slice doesn't kill,
  it throttles at `MemoryHigh` — and a throttled real-time test times out. Know each heavy step's
  peak (`<!-- pendiente de medir: release build, viewer build -->`) and give the biggest one the whole slice.
- **Files written to tmpfs (`/tmp`) count as the writer's cgroup memory** (shmem) and, with no swap,
  are never reclaimed: 3.5 GB of dumps under `/tmp` kept a 6 GB slice at its `MemoryHigh` and made
  another agent's gates time out. Write large artifacts to disk, and delete them when done.
- **The gate's fixed cost dominates small changes.** A run costs the same for 1 unit of work or 100:
  batch the work so each gate run carries enough.
- **Profile before optimising, and measure the fix.** An exact cache cut the hottest-looking function
  and saved only 5–10 %: the per-item arithmetic dominated. Drop changes that don't pay, and write
  down why in `docs/FINDINGS.md`.

## Verification — a gate is only as good as what it reaches

- **A run that timed out or printed nothing is a failure, not "0 failures".** A gate tool once summed
  every route of a run — all empty, all timed out — into "0 mismatched, rc=0". The tool must treat
  "compared nothing" as red.
- **Code the tests never reach cannot be verified.** Track "written but unverified" separately. The
  cheapest way to raise verified coverage is often **more test routes** (inputs that reach states the
  current ones skip), not more code.
- **Define "verified" precisely and write it down** — e.g. *reached (compared > 0) with 0 mismatches
  in any gate run of this change, and a sabotage of it detected*: *a behaviour is verified when a test in `crates/rdai-sim/tests/` asserts it over a full run, that test was seen failing under a written sabotage of the code it protects, and the run compared something (laps > 0, ticks > 0)*. A faster or
  uncapped run often reaches code the slow real-time run doesn't; requiring the slow run alone throws
  away verified work.
- **Every check must have been seen failing — sabotage it.** Make a change every comparison sees and
  require the gate to catch it. **A sabotage must survive callers:** bumping a callee-saved register
  is erased by any caller that saves and restores it; corrupt a compared value nobody overwrites
  (or do both).
- **The gate only proves the paths it runs.** A branch no test takes (a special-case constant, an
  error path) is compared with the reference by hand, and still gets an independent review: a
  reviewer once found `a == 2 → 2` where the original stores 0.
- **Nested calls inside a test case may not exercise the callee's new code** (the harness may run the
  originals for nested calls). Know your harness's rule, and don't count a child as verified because
  its parent passed; when a parent mismatches, log which members ran and credit a sabotage only where
  exactly one thing was sabotaged.
- **Never weaken a gate to get green** — no deleting or editing recorded cases, no widening the
  comparison or the tolerance, no excluding the side effects that fail. Fix the code; if a case is
  wrong, say why in the commit.
- **No silent "last one wins".** If two registrations / definitions of the same thing can coexist,
  make it a hard error at startup. An older copy once silently shadowed renamed functions, so their
  new code never ran while the gate stayed green.
- **Independent review catches what the author's gate can't** — wrong comments, invented names, weak
  evidence, mis-stated numbers. Review every PR before merging, even when all gates are green.

## Process hygiene

- **Tooling the workflow depends on lives in the repo**, tested — never only in a temp or scratch
  directory. A reboot wiped `/tmp` mid-PR and took every gate script with it.
- **One checkout per agent.** Never `git checkout`/switch branches in a working tree another agent is
  using: its uncommitted work moves with you and its branch changes under it. Make your own worktree.
- **Don't stack PRs on each other's branches.** Merging the upper one merges into the lower branch,
  not into the default branch; the work silently never lands. Branch from the default branch, and
  merge it into your branch to integrate.
- **Integrate by merge, never by rebase + force push**, when the project forbids force pushes — even
  on your own branches.
- **Counters must agree.** If the progress report and the gate disagree on how many items are done,
  one of them is wrong; check item by item.
- **Correct your own mistaken instructions to helpers explicitly** — what was wrong, the right rule,
  what to redo — and write the corrected rule into the project docs so it doesn't recur.
- **Names only with evidence, written next to the name.** A role-derived name is a guess: keep a
  neutral placeholder name and put the role in a comment until something in the program itself
  (strings, registration tables, asserts) confirms it.
