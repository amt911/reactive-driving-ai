//! Behaviour: the agent finds its own line. Handed only the centreline as "the line" (its limits
//! are the only thing the agent reads from it), it still laps close to the ideal of the optimal
//! line, because it works out its own lap plan from the track edges and re-optimises the road
//! ahead while it drives (`local_line.rs`).
//!
//! Not a unit test: whether a line the agent computed itself is drivable at speed only shows over
//! whole corners with the controllers and the physics.
//!
//! Oracle: the ideal lap of `RacingLine::build` (offline speed profile, Velenis & Tsiotras 2005),
//! computed before any agent drives; the centreline's own ideal lap shows the margin.
//!
//! Sabotage seen failing: the agent's lap plan set to the centreline (`LocalLine::new`, plan =
//! `RacingLine::from_offsets(track, vec![0.0; n], limits)`) -> best lap 68.93 s vs 63.23 ideal
//! (9.0 %): the 200 m window alone, joined to the centreline, does not find the line.

use rdai_core::RacingLine;
use rdai_sim::{default_field, tracks, World};

const LAPS: i32 = 2;

#[test]
fn agent_laps_on_its_own_line_when_handed_the_centreline() {
    let mut w = World::new(&tracks::default_circuit(), &default_field(1), LAPS);
    let ideal = w.line.lap_time;
    let centre = RacingLine::from_offsets(&w.track, vec![0.0; w.track.len()], &w.line.limits);
    let centre_lap = centre.lap_time;
    for l in w.lines.iter_mut() {
        *l = centre.clone();
    }
    w.line = centre;
    let dt = 1.0 / 120.0;
    while !w.finished() && w.time < 2.0 * ideal * LAPS as f32 + 30.0 {
        w.step(dt);
    }
    let car = &w.cars[0];
    assert_eq!(car.lap_times.len(), LAPS as usize, "laps completed: {:?}", car.lap_times);
    assert_eq!(car.wall_hits, 0, "wall hits; laps {:?}", car.lap_times);
    let best = car.lap_times.iter().copied().fold(f32::INFINITY, f32::min);
    // 5 %: ~2 % from the grip the driver uses (sqrt of ~96 %), the rest tracking. Measured 65.33 s
    // vs 63.23 ideal (3.3 %); with the sabotage below 68.93 s (9.0 %).
    assert!(best <= 1.05 * ideal, "best lap {best:.2} s vs ideal {ideal:.2} s (centreline ideal {centre_lap:.2} s)");
}
