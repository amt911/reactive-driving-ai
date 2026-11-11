# Changelog — progress tags

Each tag on `main` is a runnable snapshot. `git checkout <tag> && cargo run --release -p rdai-viewer`.

| Tag | Date | What you can do |
| --- | --- | --- |
| *(unreleased)* | 2026-11-11 | First commit: `rdai-core` (Frenet track, min-curvature line, speed profile, Frenet planner with IDM and emergency braking, pure pursuit, driver personality), `rdai-sim` + `bench` (race, crash, debris scenarios), C ABI + header + C smoke test, 2D viewer. Best lap 82.6 s vs 63.4 s ideal, 9 wall hits in 2 laps, ~3.5 µs/tick |
