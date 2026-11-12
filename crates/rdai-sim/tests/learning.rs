//! Behaviour: the agent learns, lap to lap, where its car has less grip than it declared.
//! The car's real tyre friction is 10 % below the limits the agent was given (its line, its speed
//! profile). It slides wide and hits walls at first; it must stop hitting them and settle on a lap
//! time.
//!
//! Not a unit test: the learning signal is the car running wide of its own line, which only the
//! controllers and the physics produce, over whole laps.
//!
//! Measured: walls per lap [1, 0, 0, 0, 0, 0], laps 77.8, 70.0, 67.9, 67.7, 67.7, 67.7 s.
//! Sabotage seen failing: `LearnParams::default().rate` 0 -> walls [1, 3, 1, 2, 4, 0], laps
//! 69-85 s.

use rdai_sim::{default_field, tracks, World};

const LAPS: usize = 6;

#[test]
fn agent_learns_a_car_with_less_grip_than_declared() {
    let mut w = World::new(&tracks::default_circuit(), &default_field(1), LAPS as i32);
    w.cars[0].params.mu *= 0.9;
    let dt = 1.0 / 120.0;
    let mut walls_at_lap = vec![0u32];
    while !w.finished() && w.time < 3.0 * w.line.lap_time * LAPS as f32 {
        w.step(dt);
        let car = &w.cars[0];
        if car.lap_times.len() == walls_at_lap.len() {
            walls_at_lap.push(car.wall_hits);
        }
    }
    let car = &w.cars[0];
    let walls: Vec<u32> = walls_at_lap.windows(2).map(|p| p[1] - p[0]).collect();
    eprintln!("laps {:?} walls per lap {walls:?}", car.lap_times);
    assert_eq!(car.lap_times.len(), LAPS, "laps completed: {:?}", car.lap_times);
    assert_eq!(walls[LAPS - 2..], [0, 0], "wall hits per lap {walls:?}");
    let (a, b) = (car.lap_times[LAPS - 2], car.lap_times[LAPS - 1]);
    assert!((a - b).abs() <= 0.01 * a, "lap time not settled: {:?}", car.lap_times);
}
