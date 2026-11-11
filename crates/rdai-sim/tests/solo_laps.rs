//! Behaviour: one car alone completes laps cleanly and close to the ideal lap.
//!
//! Journey: the whole stack (line, speed profile, planner, controllers, vehicle physics) driving
//! the default circuit with nobody around. Not a unit test because tracking errors, braking and the
//! friction ellipse only interact over whole corners.
//!
//! Oracle: `line.lap_time`, the ideal lap of the offline speed profile (Velenis & Tsiotras 2005),
//! computed before any agent drives — not by the code under test.
//!
//! Sabotage seen failing: on `main` (2026-11-11) best lap 82.6 s vs 63.4 s ideal, 9 wall hits.

use rdai_sim::{default_field, tracks, World};

const LAPS: i32 = 3;

#[test]
fn solo_car_laps_without_walls_within_ten_percent_of_ideal() {
    let mut w = World::new(&tracks::default_circuit(), &default_field(1), LAPS);
    let dt = 1.0 / 120.0;
    // Cap: twice the ideal per lap, so a car that never finishes fails instead of hanging.
    while !w.finished() && w.time < 2.0 * w.line.lap_time * LAPS as f32 + 30.0 {
        w.step(dt);
    }
    let car = &w.cars[0];
    // A run that compared nothing is red, not green.
    assert_eq!(car.lap_times.len(), LAPS as usize, "laps completed: {:?}", car.lap_times);
    assert_eq!(car.wall_hits, 0, "wall hits; laps {:?}", car.lap_times);
    let best = car.lap_times.iter().copied().fold(f32::INFINITY, f32::min);
    // 10 %: the driver uses ~96 % of the grip (sqrt -> ~2 % slower), the rest is tracking margin.
    assert!(best <= 1.10 * w.line.lap_time, "best lap {best:.2} s vs ideal {:.2} s", w.line.lap_time);
}
