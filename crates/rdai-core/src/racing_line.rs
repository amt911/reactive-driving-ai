//! Offline racing line: minimum-curvature path inside the track (Braghin et al. 2008;
//! Heilmeier et al. 2019 solve the same objective as a QP). Here it is solved by projected gradient
//! descent on the lateral offsets, coarse-to-fine so long corners converge in milliseconds.
//!
//! Computed once per track and vehicle class; agents only read it.

use crate::math::{menger_curvature, Vec2};
use crate::speed_profile::{self, VehicleLimits};
use crate::track::Track;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LineParams {
    /// Distance kept from each track edge (m); at least half the car width.
    pub margin: f32,
    /// Gradient-descent iterations at the coarsest level; finer levels use fewer.
    pub iterations: usize,
}

impl Default for LineParams {
    fn default() -> Self {
        LineParams { margin: 1.5, iterations: 4000 }
    }
}

#[derive(Clone, Debug)]
pub struct RacingLine {
    /// Lateral offset from the centreline at each centreline sample (positive = left).
    pub offset: Vec<f32>,
    /// World positions of the line at each centreline sample.
    pub points: Vec<Vec2>,
    /// Signed curvature of the line at each sample (positive = left turn).
    pub curvature: Vec<f32>,
    /// Target speed at each sample (m/s).
    pub speed: Vec<f32>,
    /// Distance along the line from sample `i` to `i + 1`.
    pub segment: Vec<f32>,
    /// Ideal lap time of the line with the limits it was built for (s).
    pub lap_time: f32,
    /// Limits the speed profile was computed with.
    pub limits: VehicleLimits,
}

impl RacingLine {
    pub fn build(track: &Track, params: &LineParams, limits: &VehicleLimits) -> RacingLine {
        let n = track.len();
        let mut offset = vec![0.0f32; n];
        let lo: Vec<f32> = (0..n).map(|i| -(track.width_right(i) - params.margin).max(0.0)).collect();
        let hi: Vec<f32> = (0..n).map(|i| (track.width_left(i) - params.margin).max(0.0)).collect();

        // Coarse-to-fine: optimise every `stride`-th sample, then interpolate to the next level.
        let mut stride = 1usize;
        while n / (stride * 2) >= 32 && stride * 2 <= 16 {
            stride *= 2;
        }
        let mut iterations = params.iterations;
        loop {
            optimise_level(track, &mut offset, &lo, &hi, stride, iterations);
            if stride == 1 {
                break;
            }
            // Interpolate the samples between the optimised ones.
            let half = stride / 2;
            let mut i = half;
            while i < n {
                let a = offset[i - half];
                let b = offset[(i + half) % n];
                offset[i] = (0.5 * (a + b)).clamp(lo[i], hi[i]);
                i += stride;
            }
            stride = half;
            iterations = (iterations / 2).max(200);
        }

        Self::from_offsets(track, offset, limits)
    }

    /// Builds the line data (points, curvature, speed) from given offsets.
    pub fn from_offsets(track: &Track, offset: Vec<f32>, limits: &VehicleLimits) -> RacingLine {
        let n = track.len();
        let points: Vec<Vec2> = (0..n).map(|i| track.point(i) + track.normal(i) * offset[i]).collect();
        let segment: Vec<f32> = (0..n).map(|i| (points[(i + 1) % n] - points[i]).length()).collect();
        // Curvature over a +-2 sample stencil, then a light smoothing pass, to tame discretisation noise.
        let raw: Vec<f32> = (0..n)
            .map(|i| menger_curvature(points[(i + n - 2) % n], points[i], points[(i + 2) % n]))
            .collect();
        let curvature: Vec<f32> =
            (0..n).map(|i| 0.25 * raw[(i + n - 1) % n] + 0.5 * raw[i] + 0.25 * raw[(i + 1) % n]).collect();
        let speed = speed_profile::compute(&curvature, &segment, limits);
        let lap_time = speed_profile::lap_time(&speed, &segment);
        RacingLine { offset, points, curvature, speed, segment, lap_time, limits: *limits }
    }

    /// Line offset at centreline arc length `s` (interpolated).
    #[inline]
    pub fn offset_at(&self, track: &Track, s: f32) -> f32 {
        let (i, t) = track.index_at(s);
        let n = self.offset.len();
        self.offset[i] + (self.offset[(i + 1) % n] - self.offset[i]) * t
    }

    /// Target speed at centreline arc length `s` (interpolated).
    #[inline]
    pub fn speed_at(&self, track: &Track, s: f32) -> f32 {
        let (i, t) = track.index_at(s);
        let n = self.speed.len();
        self.speed[i] + (self.speed[(i + 1) % n] - self.speed[i]) * t
    }

    /// Lowest target speed in `[s, s + ahead]` — what a driver must be able to slow down to.
    pub fn min_speed_ahead(&self, track: &Track, s: f32, ahead: f32) -> f32 {
        let (i0, _) = track.index_at(s);
        let steps = (ahead / track.spacing()).ceil() as usize + 1;
        let n = self.speed.len();
        (0..steps).map(|k| self.speed[(i0 + k) % n]).fold(f32::MAX, f32::min)
    }
}

fn optimise_level(track: &Track, offset: &mut [f32], lo: &[f32], hi: &[f32], stride: usize, iterations: usize) {
    let n = track.len();
    let m = n / stride;
    if m < 5 {
        return;
    }
    let idx: Vec<usize> = (0..m).map(|j| j * stride).collect();
    let base: Vec<Vec2> = idx.iter().map(|&i| track.point(i)).collect();
    let normal: Vec<Vec2> = idx.iter().map(|&i| track.normal(i)).collect();
    let mut a: Vec<f32> = idx.iter().map(|&i| offset[i]).collect();
    let mut p: Vec<Vec2> = vec![Vec2::ZERO; m];
    let mut c: Vec<Vec2> = vec![Vec2::ZERO; m];
    // Step size: the biharmonic stencil's largest eigenvalue is 16, gradient factor 2 -> stable below 1/16.
    let eta = 0.04;
    for _ in 0..iterations {
        for j in 0..m {
            p[j] = base[j] + normal[j] * a[j];
        }
        for j in 0..m {
            c[j] = p[(j + m - 1) % m] - p[j] * 2.0 + p[(j + 1) % m];
        }
        for j in 0..m {
            let g = (c[(j + m - 1) % m] - c[j] * 2.0 + c[(j + 1) % m]) * 2.0;
            let i = idx[j];
            a[j] = (a[j] - eta * g.dot(normal[j])).clamp(lo[i], hi[i]);
        }
    }
    for (j, &i) in idx.iter().enumerate() {
        offset[i] = a[j];
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::track::{tests::circle_track, TrackPoint};

    #[test]
    fn circle_line_is_smooth_and_inside_bounds() {
        // On a constant-radius ring the objective is almost flat, so only bounds and smoothness are
        // well defined: no zig-zag between neighbouring samples.
        let t = circle_track(100.0, 6.0);
        let line = RacingLine::build(&t, &LineParams::default(), &VehicleLimits::default());
        for (i, &o) in line.offset.iter().enumerate() {
            assert!(o <= t.width_left(i) - 1.5 + 1e-4 && o >= -(t.width_right(i) - 1.5) - 1e-4);
        }
        let n = line.offset.len();
        let jump = (0..n).map(|i| (line.offset[(i + 1) % n] - line.offset[i]).abs()).fold(0.0, f32::max);
        assert!(jump < 0.05, "jump {jump}");
    }

    #[test]
    fn line_lowers_curvature_and_lap_time_vs_centreline() {
        // A rounded rectangle: the optimised line must cut corners and be faster than the centreline.
        let pts = [(0.0, 0.0), (200.0, 0.0), (220.0, 20.0), (220.0, 120.0), (200.0, 140.0), (0.0, 140.0), (-20.0, 120.0), (-20.0, 20.0)];
        let ctrl: Vec<TrackPoint> =
            pts.iter().map(|&(x, y)| TrackPoint { x, y, width_left: 7.0, width_right: 7.0 }).collect();
        let t = Track::from_control_points(&ctrl, 2.0).unwrap();
        let lim = VehicleLimits::default();
        let centre = RacingLine::from_offsets(&t, vec![0.0; t.len()], &lim);
        let line = RacingLine::build(&t, &LineParams::default(), &lim);
        let max_k = |l: &RacingLine| l.curvature.iter().map(|k| k.abs()).fold(0.0, f32::max);
        assert!(max_k(&line) < 0.8 * max_k(&centre), "{} vs {}", max_k(&line), max_k(&centre));
        assert!(line.lap_time < centre.lap_time * 0.97, "{} vs {}", line.lap_time, centre.lap_time);
    }
}
