//! Procedural circuits: every seed gives a drivable closed track, the same seed the same track,
//! and the AI can lap them. They widen testing beyond the hand-made circuits.

use rdai_core::math::menger_curvature;
use rdai_core::Track;
use rdai_sim::{default_field, tracks, World};

const SEEDS: u64 = 100;

fn build(seed: u64) -> Track {
    Track::from_control_points(&tracks::procedural(seed), 2.0).expect("procedural track builds")
}

/// Geometry every seed must satisfy (oracle: plane geometry, not the generator):
/// - the inner edge never folds: centreline radius > the half-width on the inside + 1 m;
/// - stretches of track far apart along `s` never overlap: their centrelines are further apart
///   than both half-widths plus a 3 m wall gap.
///
/// Sabotage seen failing: `drivable` returning `true` (seed 1: 1372 m, too short), and its radius
/// check loosened by 100 m (seed 23: a 7 m corner). The overlap check has never fired on seeds
/// 0..100 — a radial polygon cannot fold back — so it is a guard, not verified behaviour.
#[test]
fn every_seed_gives_a_drivable_closed_track() {
    for seed in 0..SEEDS {
        let t = build(seed);
        let (n, ds) = (t.len(), t.spacing());
        assert!((1500.0..=6000.0).contains(&t.length()), "seed {seed}: length {:.0} m", t.length());
        for i in 0..n {
            let k = menger_curvature(t.point((i + n - 1) % n), t.point(i), t.point((i + 1) % n));
            let inner = if k > 0.0 { t.width_left(i) } else { t.width_right(i) };
            if k.abs() > 1e-6 {
                assert!(1.0 / k.abs() > inner + 1.0, "seed {seed}: radius {:.1} m at sample {i}", 1.0 / k.abs());
            }
        }
        // Pairs at least 60 m apart along the track, both ways round the loop.
        let skip = (60.0 / ds) as usize;
        for i in (0..n).step_by(4) {
            for j in (0..n).step_by(4) {
                let gap = (i as isize - j as isize).unsigned_abs();
                if gap.min(n - gap) < skip {
                    continue;
                }
                let need = t.width_left(i).max(t.width_right(i)) + t.width_left(j).max(t.width_right(j)) + 3.0;
                let dist = (t.point(i) - t.point(j)).length();
                assert!(dist > need, "seed {seed}: samples {i} and {j} are {dist:.1} m apart, need {need:.1}");
            }
        }
    }
}

/// Same seed, same track; different seeds, different tracks.
/// Sabotage seen failing: seeding the generator from the wall clock.
#[test]
fn procedural_tracks_are_deterministic() {
    assert_eq!(tracks::procedural(7), tracks::procedural(7));
    assert_ne!(tracks::procedural(7), tracks::procedural(8));
}

/// The AI laps unseen circuits: one car, 2 laps, on several seeds, finishes within 1.5x the
/// ideal lap (oracle: each track's own offline speed profile). Not 10 % like the default circuit:
/// these tracks are not tuned for, and this test guards "drivable", not "fast".
/// Sabotage seen failing: `max_steer: 0.05` in the sim's `VehicleParams::default` (the car cannot
/// turn) -> no laps.
#[test]
fn ai_laps_procedural_tracks() {
    for seed in [1u64, 2, 3, 4, 5] {
        let mut w = World::new(&tracks::procedural(seed), &default_field(1), 2);
        let cap = 3.0 * w.line.lap_time * 2.0 + 60.0;
        while !w.finished() && w.time < cap {
            w.step(1.0 / 120.0);
        }
        let c = &w.cars[0];
        assert_eq!(c.lap_times.len(), 2, "seed {seed}: laps {:?} in {:.0} s", c.lap_times, w.time);
        let best = c.lap_times.iter().copied().fold(f32::INFINITY, f32::min);
        assert!(best < 1.5 * w.line.lap_time, "seed {seed}: best {best:.1} s vs ideal {:.1} s", w.line.lap_time);
    }
}
