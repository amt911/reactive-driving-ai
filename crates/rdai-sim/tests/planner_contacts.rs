//! Regression: a car touching the rear of another must treat it as in front, not alongside
//! (docs/FINDINGS.md: classed "alongside", the follower kept accelerating into the car it touched;
//! bench --cars 10 --seed 2 logged 107 consecutive contacts between the same pair).

use rdai_core::planner::{EgoFrenet, Planner};
use rdai_core::{Mode, Obstacle, PlannerParams};
use rdai_sim::{default_field, tracks, World};

/// Follower 4 m behind (centre to centre, cars 4.6 m long: already touching), same lateral
/// position, both at 16 m/s; the car in front brakes hard (-10 m/s^2). The follower must not
/// cruise on: it brakes (negative acceleration cap) or plans an emergency.
/// Sabotage seen failing: the "in front" condition removed (state before the fix) -> Cruise.
#[test]
fn touching_the_car_in_front_is_not_alongside() {
    let w = World::new(&tracks::default_circuit(), &default_field(1), 1);
    let (track, line) = (&w.track, &w.line);
    let s = 140.0;
    let d = -5.0;
    let ego = EgoFrenet { s, d, heading_error: 0.0, speed: 16.0 };
    let pos = track.to_world(s, d);
    let front = Obstacle {
        position: track.to_world(s + 4.0, d + 0.3),
        heading: track.heading_at(s + 4.0),
        speed: 16.0,
        accel: -10.0,
        length: 4.6,
        width: 1.9,
    };
    let drv = w.cars[0].agent.driver.params();
    let mut planner = Planner::new(PlannerParams::default());
    let plan = planner.plan(track, line, &ego, pos, track.index_at(s).0, &[front], &drv);
    assert!(
        plan.mode == Mode::Emergency || plan.accel_limit < 0.0,
        "mode {:?}, accel limit {:.1}: the follower does not brake for the car it is touching",
        plan.mode,
        plan.accel_limit
    );
}
