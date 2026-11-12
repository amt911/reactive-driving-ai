//! The agent's own racing line, found while driving: every `period` seconds it solves the
//! minimum-curvature path (Braghin et al. 2008; Heilmeier et al. 2019, the same objective as
//! `racing_line.rs`) over the next `horizon` metres of road ahead, starting from the car, and the
//! speed profile along it (Velenis & Tsiotras 2005), ending at rest at the window's end. No global
//! line is followed: a car pushed off its line plans again from where it is.
//!
//! - **Ending at rest** keeps the car able to stop on the road it has looked at ("drive within the
//!   sight distance", the assured-clear-distance rule of road design; AASHTO Green Book 2018 §3.2).
//!   The horizon is at least 1.3 × the braking distance from top speed, so the end never slows the
//!   car on a straight.
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

use crate::math::{menger_curvature, Vec2};
use crate::racing_line::RacingLine;
use crate::speed_profile::{self, VehicleLimits};
use crate::track::Track;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LocalParams {
    /// Road looked at ahead (m); raised to 1.3 × the braking distance from top speed if shorter.
    pub horizon: f32,
    /// Time between solves (s). A guess: 6 ticks at 60 Hz, fast against how the road changes.
    pub period: f32,
    /// Distance kept from each track edge (m), as `LineParams::margin`.
    pub margin: f32,
    /// Offset from the line (m) past which the window starts at the car instead of the line.
    /// A guess: larger than tracking noise (~0.3 m in the solo lap), smaller than a pushed car.
    pub stitch_tol: f32,
}

impl Default for LocalParams {
    fn default() -> Self {
        LocalParams { horizon: 300.0, period: 0.1, margin: 1.5, stitch_tol: 0.5 }
    }
}

/// Largest start slope (dd/ds) taken from the car's course: a car pointing across the road gets a
/// plan that turns it back over a few metres, not a vertical one.
const MAX_START_SLOPE: f32 = 0.5;
/// Tikhonov term on the Hessian diagonal, so a straight window (all normals parallel) stays
/// positive definite in floating point; far below the curvature terms (~1).
const REGULARISATION: f32 = 1e-5;
/// Floor of the active-set iteration cap (the cap is 4 × window): a cold start adds about one
/// bound per iteration.
const MAX_ACTIVE_SET_ITERATIONS: usize = 12;

#[derive(Clone, Debug)]
pub struct LocalLine {
    pub params: LocalParams,
    /// Track-indexed line the planner reads; the window ahead is rewritten every solve.
    pub line: RacingLine,
    window: usize,
    timer: f32,
    // Per-solve buffers, sized once (no allocation per tick).
    base: Vec<Vec2>,
    nrm: Vec<Vec2>,
    lo: Vec<f32>,
    hi: Vec<f32>,
    h0: Vec<f32>,
    h1: Vec<f32>,
    h2: Vec<f32>,
    b: Vec<f32>,
    m0: Vec<f32>,
    m1: Vec<f32>,
    m2: Vec<f32>,
    rhs: Vec<f32>,
    x: Vec<f32>,
    xs: Vec<f32>,
    state: Vec<i8>,
    /// Active bound per track sample from the last solve (warm start).
    bound: Vec<i8>,
    raw: Vec<f32>,
    curv: Vec<f32>,
    seg: Vec<f32>,
    vel: Vec<f32>,
}

const FREE: i8 = 0;
const AT_LO: i8 = -1;
const AT_HI: i8 = 1;
const PINNED: i8 = 2;

impl LocalLine {
    /// Starts on the centreline; the first [`update`](Self::update) solves the window ahead.
    pub fn new(track: &Track, limits: &VehicleLimits, params: LocalParams) -> LocalLine {
        let n = track.len();
        let braking = limits.max_speed * limits.max_speed / (2.0 * limits.max_brake);
        let horizon = params.horizon.max(1.3 * braking);
        let window = ((horizon / track.spacing()).ceil() as usize).clamp(8, n.saturating_sub(4).max(8));
        let f = || vec![0.0f32; window];
        LocalLine {
            params: LocalParams { horizon, ..params },
            line: RacingLine::from_offsets(track, vec![0.0; n], limits),
            window,
            timer: 0.0,
            base: vec![Vec2::ZERO; window],
            nrm: vec![Vec2::ZERO; window],
            lo: f(),
            hi: f(),
            h0: f(),
            h1: f(),
            h2: f(),
            b: f(),
            m0: f(),
            m1: f(),
            m2: f(),
            rhs: f(),
            x: f(),
            xs: f(),
            state: vec![FREE; window],
            bound: vec![FREE; n],
            raw: f(),
            curv: f(),
            seg: f(),
            vel: f(),
        }
    }

    /// Samples in the window.
    pub fn window(&self) -> usize {
        self.window
    }

    /// Re-plans when the period is up, or at once when the car is more than `stitch_tol` off the
    /// line. `s`, `d`: the car's Frenet position; `course_error`: course minus track heading.
    /// Returns true when it solved.
    pub fn update(&mut self, track: &Track, s: f32, d: f32, course_error: f32, dt: f32) -> bool {
        self.timer -= dt;
        let far = (d - self.line.offset_at(track, s)).abs() > self.params.stitch_tol;
        if self.timer > 0.0 && !far {
            return false;
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
        true
    }

    /// Solves the window starting at track sample `i0`, its first two offsets pinned to `a0` and
    /// `a0 + slope * spacing`, and writes offsets, points, curvature, segments and speeds into
    /// [`line`](Self::line).
    pub fn solve(&mut self, track: &Track, i0: usize, a0: f32, slope: f32) {
        let n = track.len();
        let w = self.window;
        let idx = |j: usize| (i0 + j) % n;
        for j in 0..w {
            let i = idx(j);
            self.base[j] = track.point(i);
            self.nrm[j] = track.normal(i);
            self.lo[j] = -(track.width_right(i) - self.params.margin).max(0.0);
            self.hi[j] = (track.width_left(i) - self.params.margin).max(0.0);
            self.h0[j] = REGULARISATION;
            self.h1[j] = 0.0;
            self.h2[j] = 0.0;
            self.b[j] = 0.0;
            self.state[j] = FREE;
        }
        // Hessian band and linear term of sum_j |r_j + sum_k u_k a_k|^2 (times 2 for the gradient).
        for j in 1..w - 1 {
            let r = self.base[j - 1] - self.base[j] * 2.0 + self.base[j + 1];
            let u = [self.nrm[j - 1], self.nrm[j] * -2.0, self.nrm[j + 1]];
            for p in 0..3 {
                let k = j - 1 + p;
                self.b[k] += 2.0 * u[p].dot(r);
                self.h0[k] += 2.0 * u[p].dot(u[p]);
                for (q, uq) in u.iter().enumerate().take(p) {
                    let v = 2.0 * u[p].dot(*uq);
                    if p - q == 1 {
                        self.h1[k] += v;
                    } else {
                        self.h2[k] += v;
                    }
                }
            }
        }
        self.state[0] = PINNED;
        self.state[1] = PINNED;
        self.x[0] = a0;
        self.x[1] = a0 + slope * track.spacing();
        for j in 2..w {
            self.x[j] = self.line.offset[idx(j)].clamp(self.lo[j], self.hi[j]);
        }

        // Primal active set (Nocedal & Wright 2006, alg. 16.3) on the box constraints, from a
        // feasible point: solve with the current bounds held, step towards that solution until a
        // bound blocks (hold it), and once the step is whole release the one bound whose multiplier
        // has the wrong sign. Warm-started from the previous solve's active bounds, so a re-plan
        // usually takes a few iterations; capped, the result stays feasible.
        for j in 2..w {
            match self.bound[idx(j)] {
                AT_LO => {
                    self.state[j] = AT_LO;
                    self.x[j] = self.lo[j];
                }
                AT_HI => {
                    self.state[j] = AT_HI;
                    self.x[j] = self.hi[j];
                }
                _ => {}
            }
        }
        let cap = MAX_ACTIVE_SET_ITERATIONS.max(4 * w);
        for _ in 0..cap {
            self.solve_fixed();
            // Longest step in [0, 1] towards the solution that stays inside the bounds.
            let mut alpha = 1.0f32;
            let mut block = usize::MAX;
            for j in 2..w {
                if self.state[j] != FREE {
                    continue;
                }
                let (x, t) = (self.x[j], self.xs[j]);
                let a = if t < self.lo[j] {
                    (self.lo[j] - x) / (t - x)
                } else if t > self.hi[j] {
                    (self.hi[j] - x) / (t - x)
                } else {
                    continue;
                };
                if a < alpha {
                    alpha = a.max(0.0);
                    block = j;
                }
            }
            for j in 2..w {
                if self.state[j] == FREE {
                    self.x[j] += alpha * (self.xs[j] - self.x[j]);
                }
            }
            if block != usize::MAX {
                let j = block;
                if self.xs[j] < self.lo[j] {
                    self.state[j] = AT_LO;
                    self.x[j] = self.lo[j];
                } else {
                    self.state[j] = AT_HI;
                    self.x[j] = self.hi[j];
                }
                continue;
            }
            // Whole step: optimal for this active set. Release the worst wrong-signed bound.
            let mut worst = 0.0f32;
            let mut release = usize::MAX;
            for j in 2..w {
                let g = self.gradient(j);
                let wrong = match self.state[j] {
                    AT_LO => -g,
                    AT_HI => g,
                    _ => 0.0,
                };
                if wrong > worst {
                    worst = wrong;
                    release = j;
                }
            }
            if release == usize::MAX {
                break;
            }
            self.state[release] = FREE;
        }
        for j in 2..w {
            self.x[j] = self.x[j].clamp(self.lo[j], self.hi[j]);
            self.bound[idx(j)] = self.state[j];
        }
        self.write_back(track, i0);
    }

    /// `(H x + b)_j`, H symmetric with bands h0 (diagonal), h1 (j, j-1), h2 (j, j-2).
    fn gradient(&self, j: usize) -> f32 {
        let w = self.window;
        let x = &self.x;
        let mut g = self.h0[j] * x[j] + self.b[j];
        if j >= 1 {
            g += self.h1[j] * x[j - 1];
        }
        if j >= 2 {
            g += self.h2[j] * x[j - 2];
        }
        if j + 1 < w {
            g += self.h1[j + 1] * x[j + 1];
        }
        if j + 2 < w {
            g += self.h2[j + 2] * x[j + 2];
        }
        g
    }

    /// Solves into `xs` `H_FF x_F = -b_F - H_FX x_X` for the free entries, the fixed ones (pinned or at a
    /// bound) held at their values, with a banded Cholesky factorisation of the pentadiagonal system.
    fn solve_fixed(&mut self) {
        let w = self.window;
        for j in 0..w {
            self.m0[j] = self.h0[j];
            self.m1[j] = self.h1[j];
            self.m2[j] = self.h2[j];
            self.rhs[j] = -self.b[j];
        }
        for j in 0..w {
            if self.state[j] == FREE {
                continue;
            }
            let v = self.x[j];
            // Move the fixed column to the right-hand side of the free rows, then make row and
            // column j the identity.
            if j + 1 < w {
                self.rhs[j + 1] -= self.m1[j + 1] * v;
                self.m1[j + 1] = 0.0;
            }
            if j + 2 < w {
                self.rhs[j + 2] -= self.m2[j + 2] * v;
                self.m2[j + 2] = 0.0;
            }
            if j >= 1 {
                self.rhs[j - 1] -= self.m1[j] * v;
            }
            if j >= 2 {
                self.rhs[j - 2] -= self.m2[j] * v;
            }
            self.m0[j] = 1.0;
            self.m1[j] = 0.0;
            self.m2[j] = 0.0;
            self.rhs[j] = v;
        }
        // Fixed rows were set to the identity after their right-hand side was used; restore them.
        for j in 0..w {
            if self.state[j] != FREE {
                self.rhs[j] = self.x[j];
            }
        }
        // Cholesky in place: m0 <- L_jj, m1 <- L_{j,j-1}, m2 <- L_{j,j-2}.
        for j in 0..w {
            if j >= 2 {
                self.m2[j] /= self.m0[j - 2];
            }
            if j >= 1 {
                let t = if j >= 2 { self.m2[j] * self.m1[j - 1] } else { 0.0 };
                self.m1[j] = (self.m1[j] - t) / self.m0[j - 1];
            }
            let mut d = self.m0[j];
            if j >= 1 {
                d -= self.m1[j] * self.m1[j];
            }
            if j >= 2 {
                d -= self.m2[j] * self.m2[j];
            }
            self.m0[j] = d.max(1e-12).sqrt();
        }
        // L y = rhs, then L^T x = y.
        for j in 0..w {
            let mut y = self.rhs[j];
            if j >= 1 {
                y -= self.m1[j] * self.rhs[j - 1];
            }
            if j >= 2 {
                y -= self.m2[j] * self.rhs[j - 2];
            }
            self.rhs[j] = y / self.m0[j];
        }
        for j in (0..w).rev() {
            let mut x = self.rhs[j];
            if j + 1 < w {
                x -= self.m1[j + 1] * self.xs[j + 1];
            }
            if j + 2 < w {
                x -= self.m2[j + 2] * self.xs[j + 2];
            }
            self.xs[j] = x / self.m0[j];
        }
    }

    fn write_back(&mut self, track: &Track, i0: usize) {
        let n = track.len();
        let w = self.window;
        let idx = |j: usize| (i0 + j) % n;
        let line = &mut self.line;
        for j in 0..w {
            let i = idx(j);
            line.offset[i] = self.x[j];
            line.points[i] = track.point(i) + track.normal(i) * self.x[j];
        }
        // The two samples behind the window continue the pinned start, so the curvature stencil at
        // the start does not see a kink against stale values (it read as a slow corner).
        let slope = self.x[1] - self.x[0];
        for k in 1..=2 {
            let i = (i0 + n - k) % n;
            line.offset[i] = self.x[0] - slope * k as f32;
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
        speed_profile::compute_open(&self.curv, &self.seg, &line.limits, 0.0, &mut self.vel);
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
    fn the_window_stays_inside_the_track_and_ends_at_rest() {
        let t = rect();
        let lim = VehicleLimits::default();
        let mut l = LocalLine::new(&t, &lim, LocalParams::default());
        let i0 = 80;
        l.solve(&t, i0, -5.5, 0.0);
        let n = t.len();
        for j in 2..l.window() {
            let i = (i0 + j) % n;
            let o = l.line.offset[i];
            assert!(o >= -5.5 - 1e-3 && o <= 5.5 + 1e-3, "sample {i}: {o}");
        }
        let last = (i0 + l.window() - 1) % n;
        assert_eq!(l.line.speed[last], 0.0);
        assert!(l.line.speed[(i0 + 10) % n] > 20.0, "should not crawl near the start");
    }

    #[test]
    fn update_stitches_near_the_line_and_restarts_from_the_car_when_far() {
        let t = rect();
        let mut l = LocalLine::new(&t, &VehicleLimits::default(), LocalParams::default());
        assert!(l.update(&t, 10.0, 0.0, 0.0, 0.016), "first update solves");
        assert!(!l.update(&t, 10.5, 0.1, 0.0, 0.016), "within the period and near the line: no solve");
        // 3 m off the line: solve at once, from the car.
        assert!(l.update(&t, 11.0, 3.0, 0.0, 0.016));
        let i0 = t.index_at(11.0).0 + 1;
        assert!((l.line.offset[i0] - 3.0).abs() < 0.05, "{}", l.line.offset[i0]);
    }
}
