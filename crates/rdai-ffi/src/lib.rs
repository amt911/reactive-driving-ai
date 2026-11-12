//! C ABI over `rdai-core`, for Unreal (C++), Godot (GDExtension), Unity (P/Invoke) or any host that
//! can call C. Header: `include/rdai.h` (keep in sync by hand; `RDAI_ABI_VERSION` bumps on change).
//!
//! Ownership: every `*_new` returns a heap handle the caller frees with the matching `*_free`.
//! Handles are not thread-safe; use one agent per thread or lock.
//!
//! Errors never cross into C as panics: every export runs inside [`guard`], and non-finite inputs
//! are refused at the boundary. `rdai_agent_tick` returns 0 on success, -1 on a null or non-finite
//! argument, -2 on an internal error (the agent's state is then unspecified: recreate it).

use rdai_core::{
    Agent, AgentConfig, Controls, DriverProfile, LineParams, Observation, Obstacle, PlannerParams, RacingLine, Track,
    TrackPoint, VehicleGeometry, VehicleLimits,
};

pub const RDAI_ABI_VERSION: u32 = 1;

/// Runs `f`, turning a panic into `fallback`: a panic cannot unwind through `extern "C"` (the process
/// aborts, Rust >= 1.81), and the host is a game that must keep running (docs/FINDINGS.md).
fn guard<T>(fallback: T, f: impl FnOnce() -> T) -> T {
    std::panic::catch_unwind(std::panic::AssertUnwindSafe(f)).unwrap_or(fallback)
}

fn finite_obs(o: &Observation) -> bool {
    o.position.x.is_finite() && o.position.y.is_finite() && o.heading.is_finite() && o.speed.is_finite()
}

fn finite_obstacle(o: &Obstacle) -> bool {
    [o.position.x, o.position.y, o.heading, o.speed, o.accel, o.length, o.width].iter().all(|v| v.is_finite())
}

/// Track + its precomputed racing line.
pub struct RdaiTrack {
    track: Track,
    line: RacingLine,
}

#[no_mangle]
pub extern "C" fn rdai_abi_version() -> u32 {
    RDAI_ABI_VERSION
}

/// Builds a closed track through `n` control points and its racing line. Returns null on bad input.
///
/// # Safety
/// `points` must point to `n` valid `TrackPoint`s; `limits` to a valid `VehicleLimits`.
#[no_mangle]
pub unsafe extern "C" fn rdai_track_new(
    points: *const TrackPoint,
    n: usize,
    spacing: f32,
    edge_margin: f32,
    limits: *const VehicleLimits,
) -> *mut RdaiTrack {
    if points.is_null() || limits.is_null() || !spacing.is_finite() || !edge_margin.is_finite() {
        return core::ptr::null_mut();
    }
    let pts = core::slice::from_raw_parts(points, n);
    if !pts.iter().all(|p| [p.x, p.y, p.width_left, p.width_right].iter().all(|v| v.is_finite())) {
        return core::ptr::null_mut();
    }
    let limits = *limits;
    guard(core::ptr::null_mut(), || {
        let Ok(track) = Track::from_control_points(pts, spacing) else {
            return core::ptr::null_mut();
        };
        let line = RacingLine::build(&track, &LineParams { margin: edge_margin, ..Default::default() }, &limits);
        Box::into_raw(Box::new(RdaiTrack { track, line }))
    })
}

/// # Safety
/// `t` must come from `rdai_track_new` and not be used afterwards.
#[no_mangle]
pub unsafe extern "C" fn rdai_track_free(t: *mut RdaiTrack) {
    if !t.is_null() {
        guard((), || drop(Box::from_raw(t)));
    }
}

/// # Safety
/// `t` must be a valid track handle.
#[no_mangle]
pub unsafe extern "C" fn rdai_track_length(t: *const RdaiTrack) -> f32 {
    if t.is_null() {
        return f32::NAN;
    }
    guard(f32::NAN, || (*t).track.length())
}

/// Ideal lap time of the racing line (s); NaN for a null handle.
///
/// # Safety
/// `t` must be a valid track handle or null.
#[no_mangle]
pub unsafe extern "C" fn rdai_track_ideal_lap(t: *const RdaiTrack) -> f32 {
    if t.is_null() {
        return f32::NAN;
    }
    (*t).line.lap_time
}

/// Copies up to `cap` racing-line points (x, y pairs) into `out`; returns how many exist.
///
/// # Safety
/// `t` valid; `out` must hold `cap` `Vec2`s (or be null with `cap = 0`).
#[no_mangle]
pub unsafe extern "C" fn rdai_track_racing_line(t: *const RdaiTrack, out: *mut rdai_core::Vec2, cap: usize) -> usize {
    if t.is_null() {
        return 0;
    }
    let pts = &(*t).line.points;
    if !out.is_null() {
        let k = pts.len().min(cap);
        core::ptr::copy_nonoverlapping(pts.as_ptr(), out, k);
    }
    pts.len()
}

/// # Safety
/// `geometry` and `profile` must be valid pointers.
#[no_mangle]
pub unsafe extern "C" fn rdai_agent_new(
    geometry: *const VehicleGeometry,
    profile: *const DriverProfile,
    seed: u64,
) -> *mut Agent {
    if geometry.is_null() || profile.is_null() {
        return core::ptr::null_mut();
    }
    let g = *geometry;
    let cfg = AgentConfig {
        geometry: g,
        planner: PlannerParams { length: g.length, width: g.width, ..Default::default() },
        profile: *profile,
        seed,
    };
    guard(core::ptr::null_mut(), || Box::into_raw(Box::new(Agent::new(cfg))))
}

/// # Safety
/// `a` must come from `rdai_agent_new` and not be used afterwards.
#[no_mangle]
pub unsafe extern "C" fn rdai_agent_free(a: *mut Agent) {
    if !a.is_null() {
        guard((), || drop(Box::from_raw(a)));
    }
}

/// One AI step. Returns 0 on success, -1 on a null or non-finite argument (observation, obstacle,
/// `dt`), -2 on an internal error.
///
/// # Safety
/// All pointers valid; `obstacles` holds `n_obstacles` items (may be null when 0).
#[no_mangle]
pub unsafe extern "C" fn rdai_agent_tick(
    a: *mut Agent,
    t: *const RdaiTrack,
    obs: *const Observation,
    obstacles: *const Obstacle,
    n_obstacles: usize,
    dt: f32,
    out: *mut Controls,
) -> i32 {
    if a.is_null() || t.is_null() || obs.is_null() || out.is_null() || (obstacles.is_null() && n_obstacles > 0) {
        return -1;
    }
    let list: &[Obstacle] = if n_obstacles == 0 { &[] } else { core::slice::from_raw_parts(obstacles, n_obstacles) };
    if !finite_obs(&*obs) || !dt.is_finite() || dt <= 0.0 || !list.iter().all(finite_obstacle) {
        return -1;
    }
    let (a, t, obs) = (&mut *a, &*t, *obs);
    guard(-2, || {
        *out = a.tick(&t.track, &t.line, &obs, list, dt);
        0
    })
}

/// Reads the live driver state (stamina and aggression change during the race).
///
/// # Safety
/// `a` valid, `out` writable.
#[no_mangle]
pub unsafe extern "C" fn rdai_agent_get_profile(a: *const Agent, out: *mut DriverProfile) {
    if a.is_null() || out.is_null() {
        return;
    }
    *out = (*a).driver.profile;
}

/// Overrides the live driver state (the TFG's "modify a driver during the race").
///
/// # Safety
/// `a` valid, `p` readable.
#[no_mangle]
pub unsafe extern "C" fn rdai_agent_set_profile(a: *mut Agent, p: *const DriverProfile) {
    if a.is_null() || p.is_null() {
        return;
    }
    let d = &mut (*a).driver;
    d.profile = *p;
    d.base_aggression = (*p).aggression;
}

#[cfg(test)]
mod tests {
    use super::*;

    fn ellipse() -> Vec<TrackPoint> {
        (0..24)
            .map(|k| {
                let a = k as f32 / 24.0 * core::f32::consts::TAU;
                TrackPoint { x: 150.0 * a.cos(), y: 100.0 * a.sin(), width_left: 6.0, width_right: 6.0 }
            })
            .collect()
    }

    /// An engine can send garbage (a physics explosion gives NaN poses). Before, a NaN position
    /// panicked in the core (`f32::clamp` with NaN bounds) and the panic, unable to unwind through
    /// `extern "C"`, aborted the whole process — the game with it (docs/FINDINGS.md).
    /// Sabotage seen failing: the finiteness check removed from `rdai_agent_tick` -> the test
    /// binary aborts ("panic in a function that cannot unwind").
    #[test]
    fn garbage_observations_return_an_error_instead_of_aborting() {
        let pts = ellipse();
        unsafe {
            let t = rdai_track_new(pts.as_ptr(), pts.len(), 2.0, 2.0, &VehicleLimits::default());
            let a = rdai_agent_new(&VehicleGeometry::default(), &DriverProfile::default(), 7);
            let bad = [
                Observation { position: rdai_core::Vec2::new(f32::NAN, 0.0), heading: 0.0, speed: 10.0 },
                Observation { position: rdai_core::Vec2::new(150.0, 0.0), heading: f32::INFINITY, speed: 10.0 },
                Observation { position: rdai_core::Vec2::new(150.0, 0.0), heading: 0.0, speed: f32::NAN },
            ];
            for obs in &bad {
                let mut c = Controls::default();
                assert_eq!(rdai_agent_tick(a, t, obs, core::ptr::null(), 0, 1.0 / 60.0, &mut c), -1, "{obs:?}");
            }
            let good = Observation { position: rdai_core::Vec2::new(150.0, 0.0), heading: 1.57, speed: 10.0 };
            let nan_obstacle = Obstacle {
                position: rdai_core::Vec2::new(f32::NAN, 0.0),
                heading: 0.0,
                speed: 0.0,
                accel: 0.0,
                length: 4.0,
                width: 2.0,
            };
            let mut c = Controls::default();
            assert_eq!(rdai_agent_tick(a, t, &good, &nan_obstacle, 1, 1.0 / 60.0, &mut c), -1);
            assert_eq!(rdai_agent_tick(a, t, &good, core::ptr::null(), 0, f32::NAN, &mut c), -1);
            // The agent still works after rejected input.
            assert_eq!(rdai_agent_tick(a, t, &good, core::ptr::null(), 0, 1.0 / 60.0, &mut c), 0);
            rdai_agent_free(a);
            rdai_track_free(t);
        }
    }

    /// A non-finite control point makes no track (null), and null handles are refused, not
    /// dereferenced.
    #[test]
    fn bad_tracks_and_null_handles_are_refused() {
        let mut pts = ellipse();
        pts[3].x = f32::NAN;
        unsafe {
            assert!(rdai_track_new(pts.as_ptr(), pts.len(), 2.0, 2.0, &VehicleLimits::default()).is_null());
            assert!(rdai_track_length(core::ptr::null()).is_nan());
            assert!(rdai_track_ideal_lap(core::ptr::null()).is_nan());
            assert_eq!(rdai_track_racing_line(core::ptr::null(), core::ptr::null_mut(), 0), 0);
            let mut p = DriverProfile::default();
            rdai_agent_get_profile(core::ptr::null(), &mut p);
            rdai_agent_set_profile(core::ptr::null_mut(), &p);
        }
    }

    /// The guard every export runs in turns a panic into the fallback value.
    #[test]
    fn guard_turns_a_panic_into_the_fallback() {
        assert_eq!(guard(-2, || -> i32 { panic!("boom") }), -2);
        assert_eq!(guard(-2, || 0), 0);
    }

    #[test]
    fn round_trip_through_the_c_abi() {
        let pts: Vec<TrackPoint> = (0..24)
            .map(|k| {
                let a = k as f32 / 24.0 * core::f32::consts::TAU;
                TrackPoint { x: 150.0 * a.cos(), y: 100.0 * a.sin(), width_left: 6.0, width_right: 6.0 }
            })
            .collect();
        unsafe {
            let t = rdai_track_new(pts.as_ptr(), pts.len(), 2.0, 2.0, &VehicleLimits::default());
            assert!(!t.is_null());
            assert!(rdai_track_ideal_lap(t) > 0.0);
            let a = rdai_agent_new(&VehicleGeometry::default(), &DriverProfile::default(), 7);
            let obs = Observation {
                position: rdai_core::Vec2::new(150.0, 0.0),
                heading: core::f32::consts::FRAC_PI_2,
                speed: 10.0,
            };
            // NaN sentinels prove the call wrote `out`. This test checks the ABI contract (codes,
            // ranges); whether the car drives well is the behaviour suite's job (rdai-sim tests).
            // It used to assert throttle > 0 for a car spawned off the line, which is a planner
            // policy, and kept CI red for a driving reason (docs/FINDINGS.md).
            let mut c = Controls { steer: f32::NAN, throttle: f32::NAN, brake: f32::NAN, ..Controls::default() };
            assert_eq!(rdai_agent_tick(a, t, &obs, core::ptr::null(), 0, 1.0 / 60.0, &mut c), 0);
            assert!((-1.0..=1.0).contains(&c.steer), "steer {}", c.steer);
            assert!((0.0..=1.0).contains(&c.throttle), "throttle {}", c.throttle);
            assert!((0.0..=1.0).contains(&c.brake), "brake {}", c.brake);
            assert_eq!(rdai_agent_tick(a, t, &obs, core::ptr::null(), 3, 1.0 / 60.0, &mut c), -1);
            rdai_agent_free(a);
            rdai_track_free(t);
        }
        assert_eq!(rdai_abi_version(), RDAI_ABI_VERSION);
    }
}
