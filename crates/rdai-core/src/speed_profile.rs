//! Minimum-time speed profile along a closed path: the classic forward/backward pass over a
//! friction ellipse (Velenis & Tsiotras 2005; Kapania, Subosits & Gerdes 2016).
//!
//! The result tells every agent where to brake and how fast to take each corner, replacing the
//! TFG's hand-placed "recommended speed per checkpoint" plus the MRUA braking formula.

/// Longitudinal and lateral performance envelope of a car, in SI units.
#[derive(Clone, Copy, Debug, PartialEq)]
#[repr(C)]
pub struct VehicleLimits {
    /// Peak lateral acceleration (m/s^2), roughly `mu * g`.
    pub max_lat_accel: f32,
    /// Traction-limited forward acceleration at low speed (m/s^2).
    pub max_accel: f32,
    /// Peak braking deceleration (m/s^2, positive).
    pub max_brake: f32,
    /// Engine power divided by mass (W/kg): above some speed acceleration is power-limited.
    pub power_to_mass: f32,
    /// Aerodynamic drag divided by mass (1/m): drag deceleration = `drag_per_mass * v^2`.
    pub drag_per_mass: f32,
    /// Top speed cap (m/s).
    pub max_speed: f32,
}

impl Default for VehicleLimits {
    /// A generic GT-class car.
    fn default() -> Self {
        VehicleLimits {
            max_lat_accel: 12.0,
            max_accel: 7.0,
            max_brake: 13.0,
            power_to_mass: 330.0,
            drag_per_mass: 0.00035,
            max_speed: 80.0,
        }
    }
}

impl VehicleLimits {
    /// Acceleration available along the path at speed `v` with `lat` lateral acceleration in use.
    #[inline]
    pub fn accel_available(&self, v: f32, lat: f32) -> f32 {
        let ellipse = (1.0 - (lat / self.max_lat_accel).powi(2)).max(0.0).sqrt();
        let engine = self.max_accel.min(self.power_to_mass / v.max(1.0));
        engine * ellipse - self.drag_per_mass * v * v
    }
    /// Deceleration available (positive) at speed `v`; drag helps braking.
    #[inline]
    pub fn brake_available(&self, v: f32, lat: f32) -> f32 {
        let ellipse = (1.0 - (lat / self.max_lat_accel).powi(2)).max(0.0).sqrt();
        self.max_brake * ellipse + self.drag_per_mass * v * v
    }
}

/// Speed at each point of a closed path. `curvature[i]` is the curvature at point `i`;
/// `seg[i]` is the distance from point `i` to point `i + 1` (wrapping).
pub fn compute(curvature: &[f32], seg: &[f32], lim: &VehicleLimits) -> Vec<f32> {
    let n = curvature.len();
    assert_eq!(n, seg.len());
    let v_corner: Vec<f32> = curvature
        .iter()
        .map(|k| {
            let k = k.abs();
            if k < 1e-6 {
                lim.max_speed
            } else {
                (lim.max_lat_accel / k).sqrt().min(lim.max_speed)
            }
        })
        .collect();
    let mut v = v_corner.clone();
    // Start both passes at the slowest corner so the wrap-around is consistent; two laps settle it.
    let start = (0..n).min_by(|&a, &b| v_corner[a].total_cmp(&v_corner[b])).unwrap_or(0);
    for k in 0..2 * n {
        let i = (start + k) % n;
        let j = (i + 1) % n;
        let lat = v[i] * v[i] * curvature[i].abs();
        let a = lim.accel_available(v[i], lat).max(0.0);
        let reach = (v[i] * v[i] + 2.0 * a * seg[i]).sqrt();
        if reach < v[j] {
            v[j] = reach;
        }
    }
    for k in 0..2 * n {
        let j = (start + 2 * n - k) % n;
        let i = (j + n - 1) % n;
        let lat = v[j] * v[j] * curvature[j].abs();
        let b = lim.brake_available(v[j], lat);
        let reach = (v[j] * v[j] + 2.0 * b * seg[i]).sqrt();
        if reach < v[i] {
            v[i] = reach;
        }
    }
    v
}

/// Lap time of a profile (trapezoidal in speed).
pub fn lap_time(speed: &[f32], seg: &[f32]) -> f32 {
    let n = speed.len();
    (0..n).map(|i| 2.0 * seg[i] / (speed[i] + speed[(i + 1) % n]).max(0.1)).sum()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn constant_radius_hits_lateral_limit() {
        let lim = VehicleLimits::default();
        let k = vec![1.0 / 100.0; 200];
        let seg = vec![2.0; 200];
        let v = compute(&k, &seg, &lim);
        let expect = (lim.max_lat_accel * 100.0).sqrt();
        for x in v {
            assert!((x - expect).abs() < 0.05, "{x} vs {expect}");
        }
    }

    #[test]
    fn brakes_before_a_corner_within_physics() {
        // Long straight then a tight corner: speed must fall to the corner speed before it,
        // never decelerating harder than max_brake + drag.
        let lim = VehicleLimits::default();
        let n = 600;
        let mut k = vec![0.0; n];
        for x in k.iter_mut().skip(500).take(30) {
            *x = 1.0 / 20.0;
        }
        let seg = vec![2.0; n];
        let v = compute(&k, &seg, &lim);
        let corner = (lim.max_lat_accel * 20.0).sqrt();
        assert!((v[510] - corner).abs() < 0.1);
        assert!(v[400] > corner * 2.0, "should be fast on the straight");
        for i in 0..n - 1 {
            let decel = (v[i] * v[i] - v[i + 1] * v[i + 1]) / (2.0 * seg[i]);
            assert!(decel <= lim.max_brake + lim.drag_per_mass * v[i] * v[i] + 1e-2, "i={i} decel={decel}");
        }
    }
}
