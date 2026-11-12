//! Exact minimum-curvature path over a window of track samples: the quadratic program of
//! Braghin et al. 2008 / Heilmeier et al. 2019 (sum of squared curvature, track bounds), solved
//! directly instead of by gradient descent.
//!
//! The second difference of the path points, `c_j = Δ²p_j` with `p_j = base_j + n_j a_j`, is linear
//! in the offsets `a`, so `sum |c_j|^2` is a quadratic with a pentadiagonal Hessian. The box bounds are handled by a primal active set (Nocedal & Wright 2006,
//! alg. 16.3) and each step by a banded Cholesky factorisation: O(window) per factorisation, no step
//! size or iteration count to tune. Both ends of the window are pinned (two samples each: offset
//! and slope), so a window cut out of an optimal closed line is solved back to that line.

use crate::math::Vec2;
use crate::track::Track;

/// Tikhonov term on the Hessian diagonal, so a straight window (all normals parallel) stays
/// positive definite in floating point; far below the curvature terms (~1).
const REGULARISATION: f32 = 1e-5;
/// Floor of the active-set iteration cap (the cap is 4 × window): a cold start adds about one
/// bound per iteration.
const MAX_ACTIVE_SET_ITERATIONS: usize = 12;

/// Slope (m per m) at which a start outside the margin is brought back inside it: 1 m per 20 m.
/// A guess, about the lateral slope of a lane change at speed.
const MARGIN_RETURN_SLOPE: f32 = 0.05;
/// Curvature (1/m) the return starts with: a 1 km radius, a tenth of the circuit's fast corners.
/// A guess.
const MARGIN_RETURN_CURVATURE: f32 = 0.001;

pub const FREE: i8 = 0;
pub const AT_LO: i8 = -1;
pub const AT_HI: i8 = 1;
const PINNED: i8 = 2;

/// Buffers for one window size, allocated once.
#[derive(Clone, Debug)]
pub struct WindowQp {
    w: usize,
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
}

impl WindowQp {
    /// A solver for windows of `w` samples (at least 6).
    pub fn new(w: usize) -> WindowQp {
        let w = w.max(6);
        let f = || vec![0.0f32; w];
        WindowQp {
            w,
            base: vec![Vec2::ZERO; w],
            nrm: vec![Vec2::ZERO; w],
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
            state: vec![FREE; w],
        }
    }

    pub fn len(&self) -> usize {
        self.w
    }

    pub fn is_empty(&self) -> bool {
        false
    }

    /// Solves the window of track samples `i0 .. i0 + len` (wrapping), keeping `margin` from each
    /// edge, with offsets pinned at its first two samples to `start` and its last two to `end`.
    /// `offsets` (track-indexed) gives the warm start and receives the solution; `bound`
    /// (track-indexed) carries the active bounds from one solve to the next.
    #[allow(clippy::too_many_arguments)]
    pub fn solve(
        &mut self,
        track: &Track,
        i0: usize,
        margin: f32,
        start: (f32, f32),
        end: (f32, f32),
        offsets: &mut [f32],
        bound: &mut [i8],
        anchor: Option<(&[f32], f32)>,
    ) {
        let n = track.len();
        let w = self.w;
        let idx = |j: usize| (i0 + j) % n;
        for j in 0..w {
            let i = idx(j);
            self.base[j] = track.point(i);
            self.nrm[j] = track.normal(i);
            self.lo[j] = -(track.width_right(i) - margin).max(0.0);
            self.hi[j] = (track.width_left(i) - margin).max(0.0);
            self.h0[j] = REGULARISATION;
            self.h1[j] = 0.0;
            self.h2[j] = 0.0;
            self.b[j] = 0.0;
            self.state[j] = FREE;
        }
        // Warm start: the current offsets inside the bounds, both ends pinned.
        for j in 2..w - 2 {
            self.x[j] = offsets[idx(j)].clamp(self.lo[j], self.hi[j]);
        }
        self.x[0] = start.0;
        self.x[1] = start.1;
        self.x[w - 2] = end.0;
        self.x[w - 1] = end.1;
        for j in [0, 1, w - 2, w - 1] {
            self.state[j] = PINNED;
        }
        // A start outside the margin (a car pushed wide) widens the bounds near the start, so the
        // path comes back inside the margin at a bounded slope instead of in one jump (measured: a
        // car 6 m off the line was told 28 m/s for the jump). Never past the track edge.
        let sp = track.spacing();
        for j in 2..w - 2 {
            // Parabolic, then straight: it starts flat and bends at MARGIN_RETURN_CURVATURE up to
            // the slope, so following it costs no corner at the start (a linear ramp did: 0.012).
            let x = sp * (j - 1) as f32;
            let x1 = MARGIN_RETURN_SLOPE / MARGIN_RETURN_CURVATURE;
            let ramp = if x < x1 {
                0.5 * MARGIN_RETURN_CURVATURE * x * x
            } else {
                0.5 * MARGIN_RETURN_CURVATURE * x1 * x1 + MARGIN_RETURN_SLOPE * (x - x1)
            };
            let i = idx(j);
            if start.1 < self.lo[j] {
                self.lo[j] = self.lo[j].min(start.1 + ramp).max(-track.width_right(i));
            }
            if start.1 > self.hi[j] {
                self.hi[j] = self.hi[j].max(start.1 - ramp).min(track.width_left(i));
            }
        }
        // Hessian band and linear term of sum_j |c_j|^2, c_j = Δ²p_j = r_j + sum_k u_k a_k with
        // r_j = Δ²base_j and u = (n_{j-1}, -2 n_j, n_{j+1}): the objective `racing_line.rs`
        // minimises by gradient descent. Measured alternatives: the component along the track
        // normal only zig-zags where the centreline has a sharp corner (peak curvature 0.47 vs
        // 0.13 on a polyline rectangle); along the path normal (Gauss-Newton) the same. Their
        // ideal lap on the default circuit was 1 s faster, robustness first.
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
        // Optional: lambda * sum_j (Δ²(a - anchor)_j)^2, the bending of the path relative to an
        // anchor line (scalar stencil 1, -2, 1 on the offsets). Zero on the anchor, so a window of an
        // anchor that is optimal for the curvature term solves back to it for any lambda; off it,
        // paths parallel to the anchor cost nothing and a merge spreads over the window instead of
        // cutting across (the raceline-relative planning of the A2RL stacks, PAPERS.md).
        if let Some((anchor, lambda)) = anchor {
            for j in 1..w - 1 {
                let r = -(anchor[idx(j - 1)] - 2.0 * anchor[idx(j)] + anchor[idx(j + 1)]);
                let u = [1.0f32, -2.0, 1.0];
                for p in 0..3 {
                    let k = j - 1 + p;
                    self.b[k] += 2.0 * lambda * u[p] * r;
                    self.h0[k] += 2.0 * lambda * u[p] * u[p];
                    for (q, uq) in u.iter().enumerate().take(p) {
                        let v = 2.0 * lambda * u[p] * uq;
                        if p - q == 1 {
                            self.h1[k] += v;
                        } else {
                            self.h2[k] += v;
                        }
                    }
                }
            }
        }

        // Primal active set (Nocedal & Wright 2006, alg. 16.3) on the box constraints, from a
        // feasible point: solve with the current bounds held, step towards that solution until a
        // bound blocks (hold it), and once the step is whole release the one bound whose multiplier
        // has the wrong sign. Warm-started from the previous solve's active bounds, so a re-plan
        // usually takes a few iterations; capped, the result stays feasible.
        for j in 2..w - 2 {
            match bound[idx(j)] {
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
        for j in 2..w - 2 {
            self.x[j] = self.x[j].clamp(self.lo[j], self.hi[j]);
            bound[idx(j)] = self.state[j];
        }
        for j in 0..w {
            offsets[idx(j)] = self.x[j];
        }
    }

    /// `(H x + b)_j`, H symmetric with bands h0 (diagonal), h1 (j, j-1), h2 (j, j-2).
    fn gradient(&self, j: usize) -> f32 {
        let w = self.w;
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
        let w = self.w;
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
}
