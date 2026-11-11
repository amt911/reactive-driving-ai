# Toolchain, pins and release tags

> Part of `AGENTS.md`, kept in its own file so `AGENTS.md` fits the 32 KiB Codex reads by
> default. Its rules bind all the same: `AGENTS.md` lists the hard ones in *Rules by topic* and
> says when to read this file. Edit the rule here, not a copy of it.

## Rules

- **Every tool is pinned** (release version, package version or git commit) and fetched or built by
  one script — `scripts/setup-tools.sh` — into cargo's own bin directory (`cargo install --locked`). Nothing is vendored. Bumping a pin is its own commit, with the reason.
- **Verified, not remembered.** Before this file (or `AGENTS.md`) names a tool, flag or command, it
  was checked against the tool's README/source or run here, and the check date goes next to it. What
  could not be verified is marked *unverified* and carries no command.
- **Language tools run in the project environment** (`cargo`), never
  in the system interpreter or a global install.
- **Downloads are untrusted data.** Each fetched archive or repo goes in its own new, empty
  directory (`tools/src/<name>/`, gitignored); keep scripts in a different directory and pass the downloaded
  paths as arguments. Interpreters load code from the script's directory and the current directory:
  run Python that reads downloaded or third-party data with `-I`.
- **System-wide changes are asked first** (`sudo`, the system package manager, user-scope agent
  config); project-local tooling is installed, then named in the PR (what, why, what it replaces).
- **Licensed or proprietary inputs never enter git.** User-supplied data, vendor compilers and
  anything under a licence you cannot redistribute stay gitignored, with written provenance (where
  it came from, its hash). Third-party code kept in the repo keeps its licence file and an
  attribution entry in `NOTICE`.

## The table (checked 2026-11-11)

| Tool | Pin | Role | Source / note |
| --- | --- | --- | --- |
| Rust (rustc, cargo, clippy, rustfmt) | 1.99.1 (CI: same, `dtolnay/rust-toolchain`) | build, lint, format | Arch `rust` package locally; **Verified here** |
| C compiler (`cc`) | GCC 16.2.1 locally, `ubuntu-latest` gcc in CI | `scripts/c-smoke.sh` | **Verified here** |
| cargo-llvm-cov | 0.9.1 | coverage gate | `cargo install --locked cargo-llvm-cov`; **Verified here** (installed) |
| cargo-mutants | 27.1.0 | mutation gate over `rdai-core` | `cargo install --locked cargo-mutants`; **Verified here** (installed) |
| gh | 2.103.3 | PRs, repo | **Verified here** |
| macroquad | 0.4 (Cargo.lock pins the patch) | `rdai-viewer` window + 2D drawing | crates.io; *build verified when the viewer first compiles* |

## Progress tags and the changelog

- After each merged milestone, tag the default branch with an **annotated** tag
  `vMAJOR.MINOR.PATCH` (`git tag -a v0.2.0 -m "<what is usable now>"`) and push it **only when the
  user has asked for pushes** (the Git & GitHub rules in `AGENTS.md` win).
- **Every tag must build and run.** `CHANGELOG.md` lists each tag with its date and *what you can
  do with it now* — one row per tag, written for a person who checks it out
  (`git checkout <tag> && cargo run --release -p rdai-viewer`), not a commit log.
