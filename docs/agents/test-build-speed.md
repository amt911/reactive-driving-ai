# Test and build speed — measured

> Part of `AGENTS.md`, kept in its own file so `AGENTS.md` fits the 32 KiB Codex reads by
> default. Its rules bind all the same: `AGENTS.md` lists the hard ones in *Rules by topic* and
> says when to read this file. Edit the rule here, not a copy of it.

What is written here was **measured, not estimated** — with the date, the branch and how. Read it
before proposing a speed-up, and add to it when you measure something, **including what did not
help**: an idea that was tried and failed, written down with its numbers, is the one the next agent
won't spend an afternoon retrying.

## How to measure so the numbers mean something

- **Interleave** the two variants (base, branch, base, branch) and run each at least twice; a single
  pair is noise.
- **Same conditions:** inside the memory cgroup, same worker cap, build daemon stopped before each
  cold run (or warm for both), and **nothing else heavy running** — an idle Android emulator alone
  was measured eating ~5 cores, which invalidates any build timing taken next to it.
- **Measure where the time goes before cutting it.** The E2E tool's own run history (Maestro keeps
  14 days in `~/.maestro/tests`; Playwright's JSON/HTML report) says how many runs, how long and how
  many red — on the project these rules come from, the dominant term was relaunching the whole suite,
  not any single slow flow.

## Where the time goes

| Date | What | Runs | Time | Red |
| --- | --- | --- | --- | --- |
| <YYYY-MM-DD> | <E2E suite / single specs / build> | <n> | <h> | <n> |

## Before / after

| Task | Base | Branch | Δ |
| --- | --- | --- | --- |
| <`clean build`, incremental build after touching one file, unit suite, E2E suite> | <s> | <s> | <%> |

## Tried and did NOT help — don't retry without new data

- <Change> — <numbers that show it did nothing, or made it worse> (<date>).
