//! Behaviour: alone on the track, the car keeps lapping without touching a wall once it is on a
//! flying lap (faster than out of the grid). The bug this guards came back every second lap on
//! procedural circuit 4 (s ~ 430 m): ~1 m of tracking error in a fast sweep, the planner restarted
//! its plan from the measured offset, the short return's curvature dropped the target 67 -> 51 m/s,
//! the car braked flat out mid-corner, ran wide and hit the wall (docs/FINDINGS.md).
//!
//! Not a unit test: tracking error at 66 m/s comes from the controller and the physics over a
//! whole sweep, and it only appears on flying laps.
//!
//! Sabotage seen failing: `PlannerParams::default().consistency_tolerance` back to 1.0 -> proc:4
//! walls on laps 3 and 5.

use rdai_sim::{default_field, tracks, World};

const LAPS: i32 = 6;

#[test]
fn a_lone_car_laps_procedural_circuits_without_walls() {
    for seed in 1..=4 {
        let mut w = World::new(&tracks::procedural(seed), &default_field(1), LAPS);
        let cap = 2.0 * w.line.lap_time * LAPS as f32 + 30.0;
        while !w.finished() && w.time < cap {
            w.step(1.0 / 120.0);
        }
        let car = &w.cars[0];
        assert_eq!(car.lap_times.len(), LAPS as usize, "proc:{seed}: laps {:?}", car.lap_times);
        assert_eq!(car.wall_hits, 0, "proc:{seed}: wall hits; laps {:?}", car.lap_times);
    }
}
