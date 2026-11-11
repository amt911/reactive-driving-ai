//! Regression: a car displaced from the racing line must not be told to brake far below the line's
//! speed (docs/FINDINGS.md, "Planner `target_speed` falls faster than the car can brake").
//!
//! A car beside the line can drive a path parallel to it, whose curvature is almost the line's
//! (κ / (1 - κ·d) for an offset d). So with no obstacles the target speed stays close to the line's
//! own speed profile — the oracle here, computed offline by `speed_profile`, not by the planner.
//!
//! Sabotage seen failing: on `main` (before the fix) the planner always returns towards the line
//! along the shortest quintic, whose extra curvature collapsed the target to ~37 m/s at s = 590
//! with a 4 m offset, against a line speed of ~57 m/s.

use rdai_core::planner::{EgoFrenet, Planner};
use rdai_core::PlannerParams;
use rdai_sim::{default_field, tracks, World};

/// Target speed the planner gives a car `offset` metres beside the line, parallel to it, at `s`.
fn target_beside_line(w: &World, s: f32, offset: f32, speed: f32) -> (f32, f32) {
    let (track, line) = (&w.track, &w.line);
    let line_d = line.offset_at(track, s);
    let slope = (line.offset_at(track, s + 1.0) - line.offset_at(track, s - 1.0)) * 0.5;
    let ego = EgoFrenet { s, d: line_d + offset, heading_error: slope.atan(), speed };
    let pos = track.to_world(s, ego.d);
    let idx = track.index_at(s).0;
    let drv = w.cars[0].agent.driver.params();
    let mut planner = Planner::new(PlannerParams::default());
    let plan = planner.plan(track, line, &ego, pos, idx, &[], &drv);
    // The line's own requirement over the first 20 m, scaled by the grip this driver uses (the
    // planner scales line speeds by sqrt(grip_usage) — same convention).
    let oracle = line.min_speed_ahead(track, s, 20.0) * drv.grip_usage.sqrt();
    (plan.target_speed, oracle)
}

#[test]
fn offset_car_keeps_close_to_line_speed_in_a_braking_zone() {
    let w = World::new(&tracks::default_circuit(), &default_field(1), 1);
    // The zone of the bug (s ≈ 480–640 m), at the speeds and offsets the trace showed.
    for &(s, offset, speed) in &[(548.0, -2.0, 59.8), (575.0, -3.0, 54.0), (590.0, -4.0, 50.0), (610.0, -6.0, 44.0)] {
        let (target, oracle) = target_beside_line(&w, s, offset, speed);
        // 10 %: a parallel path at these offsets needs < 3 % more curvature (κ ≤ 0.0042 1/m,
        // |d| ≤ 6 m), and the 0.9 braking margin of the planner costs a little more.
        assert!(
            target >= 0.9 * oracle,
            "s {s}: offset {offset} m at {speed} m/s -> target {target:.1} m/s, line needs {oracle:.1} m/s"
        );
    }
}
