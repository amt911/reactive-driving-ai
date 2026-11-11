//! Behaviour suite: the whole stack (line, planner, controllers, vehicle physics, many cars)
//! driving the default circuit. Each test is a journey a spectator could judge; edge cases live in
//! the unit tests. Every test names the sabotage it was seen failing under (docs/agents/lessons.md).
//!
//! Scenarios mirror `bench --scenario crash|debris`: at t = 25 s the leader crashes, or debris is
//! dropped on the racing line 120 m ahead of it.

use rdai_core::DriverProfile;
use rdai_sim::{default_field, tracks, CarSpec, VehicleParams, World};

const DT: f32 = 1.0 / 120.0;
const INJECT_AT: f32 = 25.0;

enum Scenario {
    Race,
    Crash,
    Debris,
}

struct Run {
    world: World,
    /// Index of the car the scenario was injected at (the leader at `INJECT_AT`).
    victim: Option<usize>,
    /// Contacts after the injection: (car, other car or `usize::MAX` for debris).
    contacts_after: Vec<(usize, usize)>,
}

fn run(cars: usize, laps: i32, scenario: Scenario) -> Run {
    let mut w = World::new(&tracks::default_circuit(), &default_field(cars), laps);
    let mut victim = None;
    let mut contacts_after = Vec::new();
    // Cap: twice the ideal per lap plus slack, so a stuck car fails instead of hanging.
    let cap = 2.0 * w.line.lap_time * laps as f32 + 60.0;
    while !w.finished() && w.time < cap {
        w.step(DT);
        if victim.is_none() && w.time > INJECT_AT {
            let leader = w.positions()[0];
            match scenario {
                Scenario::Race => {}
                Scenario::Crash => w.crash_car(leader, 6.0),
                Scenario::Debris => {
                    let d = w.line.offset_at(&w.track, w.cars[leader].s + 120.0);
                    w.drop_debris(leader, 120.0, d, 1.2);
                }
            }
            victim = Some(leader);
        } else if victim.is_some() {
            contacts_after.extend(w.new_contacts.iter().map(|c| (c.a, c.b)));
        }
    }
    Run { world: w, victim, contacts_after }
}

fn assert_all_finished(w: &World) {
    for c in &w.cars {
        assert!(c.finished_at.is_some(), "{} did not finish (laps {:?}, t = {:.0} s)", c.name, c.lap_times, w.time);
    }
}

/// A car crashes in the lead (spins, AI off for 6 s, brakes locked): nobody behind may touch it.
/// Sabotage seen failing: `perception_range: 0.0` in `PlannerParams::default` (the planner sees no
/// obstacles) -> cars run into the spinning car.
#[test]
fn followers_never_touch_a_car_that_crashes_ahead() {
    let r = run(8, 2, Scenario::Crash);
    let v = r.victim.expect("scenario injected");
    let hits: Vec<_> = r.contacts_after.iter().filter(|&&(a, b)| a == v || b == v).collect();
    assert!(hits.is_empty(), "contacts with the crashed car {}: {hits:?}", r.world.cars[v].name);
    assert_all_finished(&r.world);
}

/// Debris (r = 1.2 m) dropped on the racing line ahead of the leader: nobody may hit it, and
/// nobody may stay stuck behind it.
/// Sabotage seen failing: `perception_range: 0.0` -> cars drive into the debris.
#[test]
fn nobody_hits_debris_on_the_racing_line() {
    let r = run(8, 2, Scenario::Debris);
    let hits: Vec<_> = r.contacts_after.iter().filter(|&&(_, b)| b == usize::MAX).collect();
    assert!(hits.is_empty(), "debris hits: {hits:?}");
    assert_all_finished(&r.world);
}

/// An 8-car, 3-lap race: everyone finishes and contacts are rare.
/// "Rare": at most one contact per car over the race. Measured 2026-11-11: 2 in total on this
/// branch, 16 on the code before the fix; a racing AI may touch, but a contact per lap is a pile-up.
/// Sabotage seen failing: `line_attachment: 50.0` in `DriverState::params` (nobody leaves the
/// line to pass) -> 16+ contacts as faster cars run into slower ones.
#[test]
fn eight_car_race_finishes_with_few_contacts() {
    let r = run(8, 3, Scenario::Race);
    let w = &r.world;
    assert_all_finished(w);
    let contacts: u32 = w.cars.iter().map(|c| c.contacts).sum();
    assert!(contacts <= w.cars.len() as u32, "{contacts} contacts in an 8-car race");
}

/// Same seeds, same inputs -> bit-identical race. The core takes its randomness from a seeded RNG
/// and the sim steps at a fixed dt, so any difference is a determinism bug, not noise.
/// Sabotage seen failing: seeding each agent's RNG from the wall clock in `World::new`.
#[test]
fn same_seed_gives_identical_race() {
    let snapshot = |w: &World| -> Vec<(u32, u32, u32, Vec<u32>, u32, u32)> {
        w.cars
            .iter()
            .map(|c| {
                (
                    c.state.position.x.to_bits(),
                    c.state.position.y.to_bits(),
                    c.state.vx.to_bits(),
                    c.lap_times.iter().map(|t| t.to_bits()).collect(),
                    c.wall_hits,
                    c.contacts,
                )
            })
            .collect()
    };
    let a = run(4, 1, Scenario::Crash);
    let b = run(4, 1, Scenario::Crash);
    assert_eq!(a.world.time.to_bits(), b.world.time.to_bits());
    assert_eq!(snapshot(&a.world), snapshot(&b.world));
}

/// Cost budget: microseconds per agent per tick, in release. 50 µs is ~4x what an 8-car race
/// measured under the CPU lock (13 µs, 2026-11-11), so machine noise does not trip it but an
/// order-of-magnitude regression does. Debug builds are 10x slower by design, so the assertion
/// only runs in release (`cargo test --release`, as CI does).
/// Sabotage seen failing: `samples: 400` in `PlannerParams::default`.
#[test]
fn ai_costs_microseconds_per_agent_tick() {
    let r = run(8, 1, Scenario::Race);
    let w = &r.world;
    assert!(w.agent_calls > 0, "no agent ticks measured");
    let us = w.agent_ns as f64 / w.agent_calls as f64 / 1e3;
    eprintln!("AI cost: {us:.2} µs per agent tick over {} ticks", w.agent_calls);
    if !cfg!(debug_assertions) {
        assert!(us < 50.0, "{us:.1} µs per agent tick");
    }
}

/// Overtaking: a car starts a row behind two clearly slower cars (200 kW and mu = 1.1 against
/// 430 kW and 1.3: ~10 % less corner speed, much less acceleration), same drivers, and must finish
/// ahead of both. `World::overtakes` counts every change of order (starts, contacts, slipstream),
/// so it is not evidence of a pass; finishing order against grid order is.
/// The slower cars are slower through the car, not the driver: on 2026-11-11 a 90 % vs 100 % grip
/// driver difference produced no lap-time difference (tracking dominates; docs/FINDINGS.md).
/// Status 2026-11-11: **red on purpose** — the fast car finishes last (laps 75–110 s, contacts,
/// Emergency ticks); see docs/FINDINGS.md. Its sabotage is pending until it is green.
#[test]
fn fast_car_overtakes_slower_ones() {
    let driver = DriverProfile { experience: 0.7, stamina: 1.0, aggression: 0.5 };
    let slow_car = VehicleParams { power: 200_000.0, mu: 1.1, ..VehicleParams::default() };
    let spec = |name: &str, params| CarSpec { name: name.into(), profile: driver, params };
    let specs = [spec("Slow1", slow_car), spec("Slow2", slow_car), spec("Fast", VehicleParams::default())];
    let mut w = World::new(&tracks::default_circuit(), &specs, 3);
    while !w.finished() && w.time < 2.0 * w.line.lap_time * 3.0 + 60.0 {
        w.step(DT);
    }
    assert_all_finished(&w);
    let order: Vec<&str> = w.positions().iter().map(|&i| w.cars[i].name.as_str()).collect();
    assert_eq!(order[0], "Fast", "finishing order {order:?}");
}
