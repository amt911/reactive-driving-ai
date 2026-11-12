//! The agent's own racing line, found while driving. Two levels, as in racing stacks (Betz et al.
//! 2022 survey §4; the A2RL and ForzaETH stacks in PAPERS.md):
//!
//! - **Lap plan:** when the agent first sees the track it works out its own minimum-curvature lap
//!   and speed profile from the track edges and its car's limits (`RacingLine::build`); nobody hands
//!   it a line. Lap-to-lap learning (phase 2) rewrites this plan.
//! - **Window:** every `period` seconds it solves the minimum-curvature path (Braghin et al. 2008;
//!   Heilmeier et al. 2019) over the next `horizon` metres from where the car is, its end joined to
//!   the lap plan (offset, slope and speed), and the speed profile along it (Velenis & Tsiotras
//!   2005). A car pushed off its line plans its way back from where it is, at the speed the road
//!   allows.
//!
//! - **Why the end is joined to the lap plan:** measured, a free-ended 390 m window is myopic — it
//!   does not pay for the corner past its end, stays mid-track, then reshapes as the corner comes
//!   into view, the speed it asks for drops faster than the car can brake, and the car hits the
//!   wall (9 walls in 3 laps; docs/FINDINGS.md). The classic remedy is a terminal condition from a
//!   longer-horizon plan (receding-horizon control; Mayne et al. 2000).
//! - **Stitching** (Werling et al. 2010 §IV; Apollo's trajectory stitching): while the car is within
//!   `stitch_tol` of its line, the new window starts from the old line, so the controller keeps
//!   correcting tracking errors instead of the plan forgiving them; past it, the window starts at the
//!   car's offset and course.
//! - **Exact QP**: the curvature `c_j = p_{j-1} - 2 p_j + p_{j+1}` with `p_j = base_j + n_j a_j` is
//!   linear in the offsets `a`, so `sum |c_j|^2` is a quadratic with a pentadiagonal Hessian. It is
//!   solved with an active set over the track bounds and a banded Cholesky: O(window) per solve,
//!   no iteration count to tune (gradient descent needs thousands of steps on a long corner).
//!
//! The results are written into a track-indexed [`RacingLine`] the agent owns, so the planner reads
//! it exactly like the global line. Samples outside the window keep their last values.

use crate::math::menger_curvature;
use crate::min_curvature::{WindowQp, FREE};
use crate::racing_line::{LineParams, RacingLine};
use crate::speed_profile::{self, VehicleLimits};
use crate::track::Track;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LocalParams {
    /// Road looked at ahead (m); at least the planner's horizon (`horizon_time` × top speed).
    pub horizon: f32,
    /// Time between solves (s). A guess: 6 ticks at 60 Hz, fast against how the road changes.
    pub period: f32,
    /// Distance kept from each track edge (m), as `LineParams::margin`.
    pub margin: f32,
    /// Offset from the line (m) past which the window starts at the car instead of the line.
    /// Measured: at 0.5 m the window re-pinned on ordinary tracking errors and the line chased the
    /// car (68.0 s, 2 walls); from 1.0 to 2.5 m no re-pin in a solo lap (65.12 s, 0 walls).
    pub stitch_tol: f32,
    /// Weight of the bending relative to the lap plan against the absolute curvature
    /// (`min_curvature.rs`); 0 = pure minimum curvature from the car. Measured (planner_offset,
    /// solo lap): 0 cuts across a corner from 4 m wide (target 45.3 m/s, line 52.0); 1, 4 and 16
    /// keep near line speed; 4 kept, solo lap 65.22 s either way.
    pub relative_weight: f32,
}

impl Default for LocalParams {
    fn default() -> Self {
        LocalParams { horizon: 200.0, period: 0.1, margin: 1.5, stitch_tol: 1.5, relative_weight: 4.0 }
    }
}

/// Largest start slope (dd/ds) taken from the car's course: a car pointing across the road gets a
/// plan that turns it back over a few metres, not a vertical one.
const MAX_START_SLOPE: f32 = 0.5;

/// What [`LocalLine::update`] did.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Replan {
    /// Nothing: within the period and near the line.
    Kept,
    /// Solved ahead, starting from the line.
    Stitched,
    /// Solved ahead, starting from the car (it was far from the line).
    Restarted,
}

#[derive(Clone, Debug)]
pub struct LocalLine {
    pub params: LocalParams,
    /// The agent's own lap plan; the window's end is joined to it.
    pub plan: RacingLine,
    /// Track-indexed line the planner reads: the lap plan, with the window ahead rewritten every
    /// solve.
    pub line: RacingLine,
    timer: f32,
    qp: WindowQp,
    /// Active bounds per track sample (warm start of the next solve).
    bound: Vec<i8>,
    // Per-solve buffers, sized once (no allocation per tick).
    raw: Vec<f32>,
    curv: Vec<f32>,
    seg: Vec<f32>,
    vel: Vec<f32>,
}

impl LocalLine {
    /// Works out the lap plan (milliseconds, once); the first [`update`](Self::update) solves the
    /// window ahead.
    pub fn new(track: &Track, limits: &VehicleLimits, params: LocalParams) -> LocalLine {
        let n = track.len();
        let window = ((params.horizon / track.spacing()).ceil() as usize).clamp(8, n.saturating_sub(4).max(8));
        let f = || vec![0.0f32; window];
        let plan = RacingLine::build(track, &LineParams { margin: params.margin, ..Default::default() }, limits);
        LocalLine {
            params,
            line: plan.clone(),
            plan,
            timer: 0.0,
            qp: WindowQp::new(window),
            bound: vec![FREE; n],
            raw: f(),
            curv: f(),
            seg: f(),
            vel: f(),
        }
    }

    /// Samples in the window.
    pub fn window(&self) -> usize {
        self.qp.len()
    }

    /// Re-plans when the period is up (from the line: [`Replan::Stitched`]), or at once when the
    /// car is more than `stitch_tol` off the line (from the car: [`Replan::Restarted`]). `s`, `d`:
    /// the car's Frenet position; `course_error`: course minus track heading. Whoever keeps
    /// offsets relative to the line (the planner) must re-base them when it moves.
    pub fn update(&mut self, track: &Track, s: f32, d: f32, course_error: f32, dt: f32) -> Replan {
        self.timer -= dt;
        let far = (d - self.line.offset_at(track, s)).abs() > self.params.stitch_tol;
        if self.timer > 0.0 && !far {
            return Replan::Kept;
        }
        self.timer = self.params.period;
        let n = track.len();
        let i0 = (track.index_at(s).0 + 1) % n;
        let (a0, slope) = if far {
            let ahead = track.delta_s(s, i0 as f32 * track.spacing()).max(0.0);
            let slope = course_error.tan().clamp(-MAX_START_SLOPE, MAX_START_SLOPE);
            (d + slope * ahead, slope)
        } else {
            let a0 = self.line.offset[i0];
            (a0, (self.line.offset[(i0 + 1) % n] - a0) / track.spacing())
        };
        self.solve(track, i0, a0, slope);
        if far {
            Replan::Restarted
        } else {
            Replan::Stitched
        }
    }

    /// Solves the window starting at track sample `i0`, its first two offsets pinned to `a0` and
    /// `a0 + slope * spacing`, and writes offsets, points, curvature, segments and speeds into
    /// [`line`](Self::line).
    pub fn solve(&mut self, track: &Track, i0: usize, a0: f32, slope: f32) {
        let n = track.len();
        let w = self.qp.len();
        let end = (self.plan.offset[(i0 + w - 2) % n], self.plan.offset[(i0 + w - 1) % n]);
        let start = (a0, a0 + slope * track.spacing());
        let anchor = Some((self.plan.offset.as_slice(), self.params.relative_weight));
        self.qp.solve(track, i0, self.params.margin, start, end, &mut self.line.offset, &mut self.bound, anchor);
        self.write_back(track, i0);
    }

    fn write_back(&mut self, track: &Track, i0: usize) {
        let n = track.len();
        let w = self.qp.len();
        let idx = |j: usize| (i0 + j) % n;
        let line = &mut self.line;
        for j in 0..w {
            let i = idx(j);
            line.points[i] = track.point(i) + track.normal(i) * line.offset[i];
        }
        // The two samples behind the window continue the pinned start, so the curvature stencil at
        // the start does not see a kink against stale values (it read as a slow corner).
        let (x0, slope) = (line.offset[i0], line.offset[idx(1)] - line.offset[i0]);
        for k in 1..=2 {
            let i = (i0 + n - k) % n;
            line.offset[i] = x0 - slope * k as f32;
            line.points[i] = track.point(i) + track.normal(i) * line.offset[i];
        }
        // Curvature over a +-2 stencil and a light smoothing, as `RacingLine::from_offsets`.
        for j in 0..w {
            let i = idx(j);
            self.raw[j] = menger_curvature(line.points[(i + n - 2) % n], line.points[i], line.points[(i + 2) % n]);
        }
        for j in 0..w {
            let a = self.raw[j.saturating_sub(1)];
            let c = self.raw[(j + 1).min(w - 1)];
            self.curv[j] = 0.25 * a + 0.5 * self.raw[j] + 0.25 * c;
            let i = idx(j);
            self.seg[j] = (line.points[(i + 1) % n] - line.points[i]).length();
        }
        let v_end = self.plan.speed[idx(w - 1)];
        speed_profile::compute_open(&self.curv, &self.seg, &line.limits, v_end, &mut self.vel);
        for j in 0..w {
            let i = idx(j);
            line.curvature[i] = self.curv[j];
            line.segment[i] = self.seg[j];
            line.speed[i] = self.vel[j];
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::racing_line::LineParams;
    use crate::track::TrackPoint;

    /// A rounded rectangle: 200 m straights joined by 90-degree corners, 14 m wide.
    fn rect() -> Track {
        let pts = [
            (0.0, 0.0),
            (200.0, 0.0),
            (220.0, 20.0),
            (220.0, 120.0),
            (200.0, 140.0),
            (0.0, 140.0),
            (-20.0, 120.0),
            (-20.0, 20.0),
        ];
        let ctrl: Vec<TrackPoint> =
            pts.iter().map(|&(x, y)| TrackPoint { x, y, width_left: 7.0, width_right: 7.0 }).collect();
        Track::from_control_points(&ctrl, 2.0).unwrap()
    }

    /// Sum of squared curvature (second differences) over the window's interior: the objective.
    fn window_cost(t: &Track, offset: &[f32], i0: usize, w: usize) -> f32 {
        let n = t.len();
        let p = |k: usize| t.point(k % n) + t.normal(k % n) * offset[k % n];
        (i0 + 1..i0 + w - 1).map(|i| (p(i - 1) - p(i) * 2.0 + p(i + 1)).length_sq()).sum()
    }

    #[test]
    fn the_window_solution_is_optimal_against_the_global_line() {
        // Pinned to the global line, the global offsets are a feasible point of the window's QP, so
        // the exact local solution can only cost less or the same.
        let t = rect();
        let lim = VehicleLimits::default();
        let global = RacingLine::build(&t, &LineParams::default(), &lim);
        let mut l = LocalLine::new(&t, &lim, LocalParams::default());
        let i0 = 40;
        let slope = (global.offset[i0 + 1] - global.offset[i0]) / t.spacing();
        l.solve(&t, i0, global.offset[i0], slope);
        let (local, glob) =
            (window_cost(&t, &l.line.offset, i0, l.window()), window_cost(&t, &global.offset, i0, l.window()));
        assert!(local <= glob + 1e-4, "local {local} > global {glob}");
    }

    #[test]
    fn a_car_off_the_line_keeps_close_to_line_speed() {
        // The PR #2 bug at unit level: 2 m off the line before a braking zone, the plan from the
        // car must not cost more than 5% of the line's speed there.
        let t = rect();
        let lim = VehicleLimits::default();
        let global = RacingLine::build(&t, &LineParams::default(), &lim);
        let mut l = LocalLine::new(&t, &lim, LocalParams::default());
        let i0 = 80;
        l.solve(&t, i0, global.offset[i0] + 2.0, 0.0);
        for i in i0 + 2..i0 + 20 {
            assert!(
                l.line.speed[i] >= 0.95 * global.speed[i],
                "sample {i}: {} vs line {}",
                l.line.speed[i],
                global.speed[i]
            );
        }
    }

    #[test]
    fn the_window_stays_inside_the_track_and_ends_on_the_lap_plan() {
        let t = rect();
        let lim = VehicleLimits::default();
        let mut l = LocalLine::new(&t, &lim, LocalParams::default());
        let i0 = 80;
        l.solve(&t, i0, -5.5, 0.0);
        let n = t.len();
        for j in 2..l.window() {
            let i = (i0 + j) % n;
            let o = l.line.offset[i];
            assert!((-5.5 - 1e-3..=5.5 + 1e-3).contains(&o), "sample {i}: {o}");
        }
        let last = (i0 + l.window() - 1) % n;
        assert!(l.line.speed[last] <= l.plan.speed[last], "end speed above the plan");
        assert_eq!(l.line.offset[last], l.plan.offset[last]);
        assert!(l.line.speed[(i0 + 10) % n] > 20.0, "should not crawl near the start");
    }

    #[test]
    fn a_car_outside_the_margin_comes_back_without_a_corner() {
        // 7 m right of the centre on a 14 m track: outside the 1.5 m margin. The way back must
        // start gently (no jump to the margin) and never leave the track.
        let t = rect();
        let lim = VehicleLimits::default();
        let mut l = LocalLine::new(&t, &lim, LocalParams::default());
        let i0 = 20;
        l.solve(&t, i0, -6.8, 0.0);
        let n = t.len();
        for j in 0..l.window() {
            let i = (i0 + j) % n;
            assert!(l.line.offset[i] >= -7.0 - 1e-3, "sample {i} off the track: {}", l.line.offset[i]);
        }
        for i in i0..i0 + 10 {
            assert!(l.line.curvature[i].abs() < 0.003, "sample {i}: curvature {}", l.line.curvature[i]);
        }
    }

    #[test]
    fn update_stitches_near_the_line_and_restarts_from_the_car_when_far() {
        let t = rect();
        let mut l = LocalLine::new(&t, &VehicleLimits::default(), LocalParams::default());
        // The line starts as the lap plan: a car on it stitches.
        let on = l.line.offset_at(&t, 10.0);
        assert_eq!(l.update(&t, 10.0, on, 0.0, 0.016), Replan::Stitched, "first update solves");
        assert_eq!(l.update(&t, 10.5, on + 0.1, 0.0, 0.016), Replan::Kept, "within the period and near the line");
        // 3 m off the line: solve at once, from the car.
        let off = l.line.offset_at(&t, 11.0) + 3.0;
        assert_eq!(l.update(&t, 11.0, off, 0.0, 0.016), Replan::Restarted);
        let i0 = t.index_at(11.0).0 + 1;
        assert!((l.line.offset[i0] - off).abs() < 0.1, "{} vs {off}", l.line.offset[i0]);
    }
}
