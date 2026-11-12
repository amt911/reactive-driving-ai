//! Regression: the planner must plan from the direction the car travels (its course), not the
//! direction its body points. In a corner a car slides a few degrees (sideslip β = atan(vy / vx));
//! read as a heading error, β made every plan start "heading outward" at 0.03–0.045 rad and bend
//! back hard, and the envelope capped the speed for that phantom curvature: the solo lap lost
//! ~3 s at s 2300–2700 (docs/FINDINGS.md).
//!
//! Not a behaviour test: the agent sees only observations (pose, speed), as from an engine, so the
//! case is set up by hand. Oracle: a car moving exactly along the racing line has zero lateral
//! slope relative to it, whatever its body yaw.
//!
//! Sabotage seen failing: the agent's planning heading back to `obs.heading` -> slope 0.04.

use rdai_core::Observation;
use rdai_sim::{default_field, tracks, World};

#[test]
fn sideslip_is_not_read_as_heading_away_from_the_line() {
    let w = World::new(&tracks::default_circuit(), &default_field(1), 1);
    let (track, line) = (&w.track, &w.lines[0]);
    let mut agent = w.cars[0].agent.clone();
    let (dt, v, beta) = (1.0 / 120.0, 4.0, 0.04);
    for k in 0..4 {
        let s = 2400.0 + v * dt * k as f32;
        let d = line.offset_at(track, s);
        let slope = (line.offset_at(track, s + 1.0) - line.offset_at(track, s - 1.0)) * 0.5;
        let course = track.heading_at(s) + slope.atan();
        // The first tick has no displacement to estimate the course from: there the car is aligned
        // with it (a car spawned or just measured), so the agent's line starts along the course.
        let yaw = if k == 0 { 0.0 } else { beta };
        let obs = Observation { position: track.to_world(s, d), heading: course + yaw, speed: v };
        agent.tick(track, line, &obs, &[], dt);
    }
    let slope = agent.last_plan().expect("planned").lateral.first(0.0);
    assert!(
        slope.abs() < 0.01,
        "plan starts with slope {slope:.4} relative to the line (body yaw {beta} rad off course)"
    );
}
