//! Low-level controllers: pure pursuit for steering (Coulter 1992) and a speed loop with an
//! acceleration cap. Pure pursuit steers towards a point ahead on the planned path, so it reacts
//! to heading and to where the path goes — the TFG's PID only saw the distance to the closest
//! route point, which is why it oscillated and reacted late.

use crate::math::{wrap_angle, Vec2};

#[derive(Clone, Copy, Debug, PartialEq)]
#[repr(C)]
pub struct VehicleGeometry {
    pub wheelbase: f32,
    /// Distance from the reported position (car centre) back to the rear axle (m).
    pub rear_axle_offset: f32,
    /// Steering angle that `steer = 1` means (rad).
    pub max_steer: f32,
    pub length: f32,
    pub width: f32,
    /// Understeer gradient (rad per m/s^2 of lateral acceleration); 0 for a neutral car. Lets the
    /// steering add the extra angle a real car needs at speed, as a feedforward term.
    pub understeer_gradient: f32,
}

impl Default for VehicleGeometry {
    fn default() -> Self {
        VehicleGeometry {
            wheelbase: 2.7,
            rear_axle_offset: 1.35,
            max_steer: 0.45,
            length: 4.6,
            width: 1.9,
            understeer_gradient: 0.0,
        }
    }
}

/// Lookahead distance for pure pursuit at speed `v`.
#[inline]
pub fn lookahead(v: f32) -> f32 {
    (3.5 + 0.28 * v).clamp(4.0, 30.0)
}

/// Steering angle (rad) to reach `target` from a car at `position` heading `heading`, moving at
/// `speed`: pure pursuit curvature plus understeer compensation `K_us * v^2 * kappa`.
pub fn pure_pursuit(geom: &VehicleGeometry, position: Vec2, heading: f32, speed: f32, target: Vec2) -> f32 {
    let rear = position - Vec2::from_angle(heading) * geom.rear_axle_offset;
    let to = target - rear;
    let ld = to.length().max(1.0);
    let alpha = wrap_angle(to.angle() - heading);
    let kappa = 2.0 * alpha.sin() / ld;
    let delta = (geom.wheelbase * kappa).atan() + geom.understeer_gradient * speed * speed * kappa;
    delta.clamp(-geom.max_steer, geom.max_steer)
}

/// Desired longitudinal acceleration: feedforward + proportional speed loop, capped by car-following.
#[inline]
pub fn speed_control(v: f32, target: f32, accel_ff: f32, accel_limit: f32) -> f32 {
    (accel_ff + 2.5 * (target - v)).min(accel_limit)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn steers_towards_the_side_of_the_target() {
        let g = VehicleGeometry::default();
        let left = pure_pursuit(&g, Vec2::ZERO, 0.0, 10.0, Vec2::new(10.0, 2.0));
        let right = pure_pursuit(&g, Vec2::ZERO, 0.0, 10.0, Vec2::new(10.0, -2.0));
        let ahead = pure_pursuit(&g, Vec2::ZERO, 0.0, 10.0, Vec2::new(10.0, 0.0));
        assert!(left > 0.0 && right < 0.0 && ahead.abs() < 1e-6);
    }

    #[test]
    fn pure_pursuit_matches_circle_curvature() {
        // Target on a circle of radius R through the rear axle: steer = atan(L / R).
        let g = VehicleGeometry { rear_axle_offset: 0.0, ..Default::default() };
        let r = 40.0f32;
        let a = 0.3f32;
        let target = Vec2::new(r * a.sin(), r * (1.0 - a.cos()));
        let steer = pure_pursuit(&g, Vec2::ZERO, 0.0, 10.0, target);
        assert!((steer - (g.wheelbase / r).atan()).abs() < 1e-3, "{steer}");
    }
}
