//! C ABI over `rdai-core`, for Unreal (C++), Godot (GDExtension), Unity (P/Invoke) or any host that
//! can call C. Header: `include/rdai.h` (keep in sync by hand; `RDAI_ABI_VERSION` bumps on change).
//!
//! Ownership: every `*_new` returns a heap handle the caller frees with the matching `*_free`.
//! Handles are not thread-safe; use one agent per thread or lock.

use rdai_core::{
    Agent, AgentConfig, Controls, DriverProfile, LineParams, Observation, Obstacle, PlannerParams, RacingLine, Track,
    TrackPoint, VehicleGeometry, VehicleLimits,
};

pub const RDAI_ABI_VERSION: u32 = 1;

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
    if points.is_null() || limits.is_null() {
        return core::ptr::null_mut();
    }
    let pts = core::slice::from_raw_parts(points, n);
    let Ok(track) = Track::from_control_points(pts, spacing) else {
        return core::ptr::null_mut();
    };
    let line = RacingLine::build(&track, &LineParams { margin: edge_margin, ..Default::default() }, &*limits);
    Box::into_raw(Box::new(RdaiTrack { track, line }))
}

/// # Safety
/// `t` must come from `rdai_track_new` and not be used afterwards.
#[no_mangle]
pub unsafe extern "C" fn rdai_track_free(t: *mut RdaiTrack) {
    if !t.is_null() {
        drop(Box::from_raw(t));
    }
}

/// # Safety
/// `t` must be a valid track handle.
#[no_mangle]
pub unsafe extern "C" fn rdai_track_length(t: *const RdaiTrack) -> f32 {
    (*t).track.length()
}

/// Ideal lap time of the racing line (s).
///
/// # Safety
/// `t` must be a valid track handle.
#[no_mangle]
pub unsafe extern "C" fn rdai_track_ideal_lap(t: *const RdaiTrack) -> f32 {
    (*t).line.lap_time
}

/// Copies up to `cap` racing-line points (x, y pairs) into `out`; returns how many exist.
///
/// # Safety
/// `t` valid; `out` must hold `cap` `Vec2`s (or be null with `cap = 0`).
#[no_mangle]
pub unsafe extern "C" fn rdai_track_racing_line(t: *const RdaiTrack, out: *mut rdai_core::Vec2, cap: usize) -> usize {
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
    Box::into_raw(Box::new(Agent::new(cfg)))
}

/// # Safety
/// `a` must come from `rdai_agent_new` and not be used afterwards.
#[no_mangle]
pub unsafe extern "C" fn rdai_agent_free(a: *mut Agent) {
    if !a.is_null() {
        drop(Box::from_raw(a));
    }
}

/// One AI step. Returns 0 on success, -1 on a null argument.
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
    let t = &*t;
    *out = (*a).tick(&t.track, &t.line, &*obs, list, dt);
    0
}

/// Reads the live driver state (stamina and aggression change during the race).
///
/// # Safety
/// `a` valid, `out` writable.
#[no_mangle]
pub unsafe extern "C" fn rdai_agent_get_profile(a: *const Agent, out: *mut DriverProfile) {
    *out = (*a).driver.profile;
}

/// Overrides the live driver state (the TFG's "modify a driver during the race").
///
/// # Safety
/// `a` valid, `p` readable.
#[no_mangle]
pub unsafe extern "C" fn rdai_agent_set_profile(a: *mut Agent, p: *const DriverProfile) {
    let d = &mut (*a).driver;
    d.profile = *p;
    d.base_aggression = (*p).aggression;
}

#[cfg(test)]
mod tests {
    use super::*;

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
            let mut c = Controls::default();
            assert_eq!(rdai_agent_tick(a, t, &obs, core::ptr::null(), 0, 1.0 / 60.0, &mut c), 0);
            assert!(c.throttle > 0.0 && c.steer.abs() <= 1.0);
            assert_eq!(rdai_agent_tick(a, t, &obs, core::ptr::null(), 3, 1.0 / 60.0, &mut c), -1);
            rdai_agent_free(a);
            rdai_track_free(t);
        }
        assert_eq!(rdai_abi_version(), RDAI_ABI_VERSION);
    }
}
