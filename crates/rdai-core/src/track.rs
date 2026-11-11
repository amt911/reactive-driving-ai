//! Closed circuit described by a centreline plus left/right half-widths.
//!
//! Everything the AI reasons about lives in the track's Frenet frame: `s` is the arc length along
//! the centreline (wrapping at [`Track::length`]) and `d` is the signed lateral offset, positive to
//! the left of the driving direction.

use crate::math::Vec2;

/// One input sample of a circuit. Same columns as the TUMFTM `racetrack-database` CSV format
/// (`x_m, y_m, w_tr_right_m, w_tr_left_m`), so public track data can be loaded directly.
#[derive(Clone, Copy, Debug, PartialEq)]
#[repr(C)]
pub struct TrackPoint {
    pub x: f32,
    pub y: f32,
    pub width_right: f32,
    pub width_left: f32,
}

#[derive(Clone, Debug, PartialEq)]
pub enum TrackError {
    TooFewPoints,
    BadSpacing,
    BadWidth,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Projection {
    /// Arc length along the centreline, in [0, length).
    pub s: f32,
    /// Signed lateral offset, positive to the left.
    pub d: f32,
    /// Index of the centreline sample at or before `s`.
    pub index: usize,
}

#[derive(Clone, Debug)]
pub struct Track {
    pts: Vec<Vec2>,
    tangents: Vec<Vec2>,
    width_left: Vec<f32>,
    width_right: Vec<f32>,
    spacing: f32,
    length: f32,
}

/// How many samples either side of the hint [`Track::project`] searches.
const PROJECT_WINDOW: usize = 24;

impl Track {
    /// Builds a closed track through `control` points (the last point connects to the first) using
    /// a centripetal-free uniform Catmull-Rom spline, resampled every `spacing` metres.
    pub fn from_control_points(control: &[TrackPoint], spacing: f32) -> Result<Track, TrackError> {
        if control.len() < 4 {
            return Err(TrackError::TooFewPoints);
        }
        if !(spacing > 0.05) {
            return Err(TrackError::BadSpacing);
        }
        if control.iter().any(|p| !(p.width_left > 0.0 && p.width_right > 0.0)) {
            return Err(TrackError::BadWidth);
        }
        // 1. Dense spline samples.
        let n = control.len();
        let sub = 64;
        let mut dense: Vec<(Vec2, f32, f32)> = Vec::with_capacity(n * sub);
        for i in 0..n {
            let p0 = control[(i + n - 1) % n];
            let p1 = control[i];
            let p2 = control[(i + 1) % n];
            let p3 = control[(i + 2) % n];
            for k in 0..sub {
                let t = k as f32 / sub as f32;
                let cr = |a: f32, b: f32, c: f32, d: f32| {
                    0.5 * ((2.0 * b)
                        + (-a + c) * t
                        + (2.0 * a - 5.0 * b + 4.0 * c - d) * t * t
                        + (-a + 3.0 * b - 3.0 * c + d) * t * t * t)
                };
                let pos = Vec2::new(cr(p0.x, p1.x, p2.x, p3.x), cr(p0.y, p1.y, p2.y, p3.y));
                let wl = p1.width_left + (p2.width_left - p1.width_left) * t;
                let wr = p1.width_right + (p2.width_right - p1.width_right) * t;
                dense.push((pos, wl, wr));
            }
        }
        // 2. Cumulative arc length of the dense polyline (closed).
        let m = dense.len();
        let mut cum = Vec::with_capacity(m + 1);
        cum.push(0.0f32);
        for i in 0..m {
            let a = dense[i].0;
            let b = dense[(i + 1) % m].0;
            cum.push(cum[i] + (b - a).length());
        }
        let total = cum[m];
        // 3. Uniform resampling.
        let count = (total / spacing).round().max(8.0) as usize;
        let step = total / count as f32;
        let mut pts = Vec::with_capacity(count);
        let mut wl = Vec::with_capacity(count);
        let mut wr = Vec::with_capacity(count);
        let mut j = 0;
        for k in 0..count {
            let target = k as f32 * step;
            while cum[j + 1] < target {
                j += 1;
            }
            let seg = cum[j + 1] - cum[j];
            let t = if seg > 1e-9 { (target - cum[j]) / seg } else { 0.0 };
            let a = dense[j];
            let b = dense[(j + 1) % m];
            pts.push(a.0.lerp(b.0, t));
            wl.push(a.1 + (b.1 - a.1) * t);
            wr.push(a.2 + (b.2 - a.2) * t);
        }
        Ok(Track::from_samples(pts, wl, wr))
    }

    /// Builds a track from already uniformly spaced centreline samples.
    pub fn from_samples(pts: Vec<Vec2>, width_left: Vec<f32>, width_right: Vec<f32>) -> Track {
        let n = pts.len();
        let mut length = 0.0;
        for i in 0..n {
            length += (pts[(i + 1) % n] - pts[i]).length();
        }
        let tangents = (0..n)
            .map(|i| (pts[(i + 1) % n] - pts[(i + n - 1) % n]).normalized())
            .collect();
        Track { spacing: length / n as f32, pts, tangents, width_left, width_right, length }
    }

    #[inline]
    pub fn len(&self) -> usize {
        self.pts.len()
    }
    #[inline]
    pub fn is_empty(&self) -> bool {
        self.pts.is_empty()
    }
    #[inline]
    pub fn length(&self) -> f32 {
        self.length
    }
    #[inline]
    pub fn spacing(&self) -> f32 {
        self.spacing
    }
    #[inline]
    pub fn point(&self, i: usize) -> Vec2 {
        self.pts[i % self.pts.len()]
    }
    #[inline]
    pub fn tangent(&self, i: usize) -> Vec2 {
        self.tangents[i % self.pts.len()]
    }
    /// Unit normal pointing left of the driving direction.
    #[inline]
    pub fn normal(&self, i: usize) -> Vec2 {
        self.tangent(i).perp()
    }
    #[inline]
    pub fn width_left(&self, i: usize) -> f32 {
        self.width_left[i % self.pts.len()]
    }
    #[inline]
    pub fn width_right(&self, i: usize) -> f32 {
        self.width_right[i % self.pts.len()]
    }
    pub fn points(&self) -> &[Vec2] {
        &self.pts
    }

    /// Wraps any arc length into [0, length).
    #[inline]
    pub fn wrap_s(&self, s: f32) -> f32 {
        let r = s % self.length;
        if r < 0.0 {
            r + self.length
        } else {
            r
        }
    }

    /// Signed shortest distance from `a` to `b` along the loop, in (-length/2, length/2].
    #[inline]
    pub fn delta_s(&self, a: f32, b: f32) -> f32 {
        let mut d = self.wrap_s(b - a);
        if d > 0.5 * self.length {
            d -= self.length;
        }
        d
    }

    #[inline]
    pub fn index_at(&self, s: f32) -> (usize, f32) {
        let u = self.wrap_s(s) / self.spacing;
        let i = u as usize % self.pts.len();
        (i, u - u.floor())
    }

    /// Track half-widths at `s` (left, right).
    pub fn widths_at(&self, s: f32) -> (f32, f32) {
        let (i, t) = self.index_at(s);
        let l = self.width_left(i) + (self.width_left(i + 1) - self.width_left(i)) * t;
        let r = self.width_right(i) + (self.width_right(i + 1) - self.width_right(i)) * t;
        (l, r)
    }

    /// Centreline heading (radians) at `s`.
    pub fn heading_at(&self, s: f32) -> f32 {
        let (i, t) = self.index_at(s);
        self.tangent(i).lerp(self.tangent(i + 1), t).angle()
    }

    /// Frenet (s, d) to world position.
    pub fn to_world(&self, s: f32, d: f32) -> Vec2 {
        let (i, t) = self.index_at(s);
        let p = self.point(i).lerp(self.point(i + 1), t);
        let n = self.normal(i).lerp(self.normal(i + 1), t).normalized();
        p + n * d
    }

    /// World position to Frenet. With `hint` (the previous projection's index) the search is a small
    /// window, O(1); without it the whole centreline is scanned once.
    pub fn project(&self, p: Vec2, hint: Option<usize>) -> Projection {
        match hint {
            Some(h) => self.project_window(p, h, PROJECT_WINDOW),
            None => self.project_window(p, 0, self.pts.len()),
        }
    }

    /// Like [`Track::project`], searching `window` samples either side of `hint`.
    pub fn project_window(&self, p: Vec2, hint: usize, window: usize) -> Projection {
        let n = self.pts.len();
        let best = match window < n / 2 {
            true => {
                let h = hint % n;
                let mut best = h;
                let mut best_d = f32::MAX;
                for k in 0..=2 * window {
                    let i = (h + n * (window / n + 1) + k - window) % n;
                    let d = (self.pts[i] - p).length_sq();
                    if d < best_d {
                        best_d = d;
                        best = i;
                    }
                }
                best
            }
            false => {
                let mut best = 0;
                let mut best_d = f32::MAX;
                for (i, q) in self.pts.iter().enumerate() {
                    let d = (*q - p).length_sq();
                    if d < best_d {
                        best_d = d;
                        best = i;
                    }
                }
                best
            }
        };
        // Refine on the segment before or after the closest sample.
        let a = self.pts[best];
        let b = self.pts[(best + 1) % n];
        let ab = b - a;
        let t = (p - a).dot(ab) / ab.length_sq().max(1e-9);
        let (i, t) = if t >= 0.0 {
            (best, t.min(1.0))
        } else {
            let prev = (best + n - 1) % n;
            let a2 = self.pts[prev];
            let ab2 = a - a2;
            (prev, ((p - a2).dot(ab2) / ab2.length_sq().max(1e-9)).clamp(0.0, 1.0))
        };
        let a = self.pts[i];
        let b = self.pts[(i + 1) % n];
        let foot = a.lerp(b, t);
        let dir = (b - a).normalized();
        let d = dir.cross(p - foot);
        Projection { s: self.wrap_s((i as f32 + t) * self.spacing), d, index: i }
    }
}

#[cfg(test)]
pub(crate) mod tests {
    use super::*;

    /// Circle of radius `r`, counter-clockwise, half-width `w`.
    pub fn circle_track(r: f32, w: f32) -> Track {
        let pts: Vec<TrackPoint> = (0..32)
            .map(|k| {
                let a = k as f32 / 32.0 * core::f32::consts::TAU;
                TrackPoint { x: r * a.cos(), y: r * a.sin(), width_left: w, width_right: w }
            })
            .collect();
        Track::from_control_points(&pts, 2.0).unwrap()
    }

    #[test]
    fn circle_length_and_projection() {
        let t = circle_track(100.0, 6.0);
        let expect = core::f32::consts::TAU * 100.0;
        assert!((t.length() - expect).abs() / expect < 0.005, "length {}", t.length());
        // A point 3 m inside a CCW circle is to the left.
        let p = Vec2::new(0.0, 97.0);
        let pr = t.project(p, None);
        assert!((pr.d - 3.0).abs() < 0.05, "d = {}", pr.d);
        assert!((pr.s - expect / 4.0).abs() < 1.0, "s = {}", pr.s);
        // Hinted projection agrees with the full scan.
        let pr2 = t.project(p, Some(pr.index + 5));
        assert!((pr2.s - pr.s).abs() < 1e-3 && (pr2.d - pr.d).abs() < 1e-3);
    }

    #[test]
    fn frenet_round_trip() {
        let t = circle_track(80.0, 6.0);
        for k in 0..50 {
            let s = k as f32 * 9.7;
            let d = (k as f32 * 0.37).sin() * 4.0;
            let w = t.to_world(s, d);
            let pr = t.project(w, None);
            assert!(t.delta_s(s, pr.s).abs() < 0.2, "s {s} vs {}", pr.s);
            assert!((pr.d - d).abs() < 0.1, "d {d} vs {}", pr.d);
        }
    }

    #[test]
    fn delta_s_wraps() {
        let t = circle_track(50.0, 5.0);
        let l = t.length();
        assert!((t.delta_s(l - 1.0, 1.0) - 2.0).abs() < 1e-3);
        assert!((t.delta_s(1.0, l - 1.0) + 2.0).abs() < 1e-3);
    }

    #[test]
    fn rejects_bad_input() {
        assert_eq!(Track::from_control_points(&[], 2.0).unwrap_err(), TrackError::TooFewPoints);
    }
}
