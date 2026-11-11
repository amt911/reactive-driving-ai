//! Reference vehicle model for the 2D demo and tests: a dynamic bicycle model with a friction
//! ellipse (Rajamani, *Vehicle Dynamics and Control*, ch. 2), falling back to the kinematic
//! bicycle at low speed where the slip-angle model is singular.
//!
//! Engines bring their own physics; this model only has to be plausible enough that the AI's
//! reactions (braking distances, grip limits, slides) are tested against something that pushes back.

use rdai_core::{Controls, Vec2, VehicleGeometry, VehicleLimits};

const G: f32 = 9.81;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct VehicleParams {
    pub mass: f32,
    pub yaw_inertia: f32,
    /// Centre of mass to front / rear axle (m).
    pub lf: f32,
    pub lr: f32,
    /// Cornering stiffness per axle (N/rad).
    pub cf: f32,
    pub cr: f32,
    pub mu: f32,
    pub max_steer: f32,
    /// Steering rate limit (rad/s).
    pub steer_rate: f32,
    pub power: f32,
    pub max_drive_force: f32,
    pub max_brake_force: f32,
    /// 0.5 * rho * Cd * A (N s^2/m^2).
    pub drag: f32,
    pub length: f32,
    pub width: f32,
}

impl Default for VehicleParams {
    /// A GT-class car: ~1300 kg, ~430 kW, 1.3 g of grip.
    fn default() -> Self {
        VehicleParams {
            mass: 1300.0,
            yaw_inertia: 2000.0,
            lf: 1.25,
            lr: 1.45,
            cf: 90_000.0,
            cr: 110_000.0,
            mu: 1.3,
            max_steer: 0.45,
            steer_rate: 2.0,
            power: 430_000.0,
            max_drive_force: 1300.0 * 7.0,
            max_brake_force: 1300.0 * 13.0,
            drag: 0.48,
            length: 4.6,
            width: 1.9,
        }
    }
}

impl VehicleParams {
    pub fn wheelbase(&self) -> f32 {
        self.lf + self.lr
    }
    /// What the AI is told about this car. Slightly conservative on purpose.
    pub fn limits(&self) -> VehicleLimits {
        let grip = self.mu * G;
        VehicleLimits {
            max_lat_accel: grip * 0.94,
            max_accel: (self.max_drive_force / self.mass).min(grip),
            max_brake: (self.max_brake_force / self.mass).min(grip) * 0.95,
            power_to_mass: self.power / self.mass,
            drag_per_mass: self.drag / self.mass,
            max_speed: 85.0,
        }
    }
    pub fn geometry(&self) -> VehicleGeometry {
        VehicleGeometry {
            wheelbase: self.wheelbase(),
            rear_axle_offset: self.lr,
            max_steer: self.max_steer,
            length: self.length,
            width: self.width,
            // K_us = m / L * (lr / Cf - lf / Cr)  (Rajamani eq. 3.15, per-axle stiffness).
            understeer_gradient: self.mass / self.wheelbase() * (self.lr / self.cf - self.lf / self.cr),
        }
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct VehicleState {
    pub position: Vec2,
    pub heading: f32,
    /// Body-frame velocity: forward and left (m/s).
    pub vx: f32,
    pub vy: f32,
    pub yaw_rate: f32,
    pub steer: f32,
}

impl VehicleState {
    pub fn velocity_world(&self) -> Vec2 {
        Vec2::new(self.vx, self.vy).rotated(self.heading)
    }
    pub fn set_velocity_world(&mut self, v: Vec2) {
        let b = v.rotated(-self.heading);
        self.vx = b.x;
        self.vy = b.y;
    }

    /// Advances `dt` seconds. `drag_factor` < 1 models slipstream.
    pub fn step(&mut self, p: &VehicleParams, c: &Controls, dt: f32, drag_factor: f32) {
        const SUB: usize = 4;
        let h = dt / SUB as f32;
        for _ in 0..SUB {
            self.substep(p, c, h, drag_factor);
        }
    }

    fn substep(&mut self, p: &VehicleParams, c: &Controls, dt: f32, drag_factor: f32) {
        let target = c.steer.clamp(-1.0, 1.0) * p.max_steer;
        let max_d = p.steer_rate * dt;
        self.steer += (target - self.steer).clamp(-max_d, max_d);
        let delta = self.steer;
        let l = p.wheelbase();
        let m = p.mass;

        // Longitudinal force.
        let throttle = c.throttle.clamp(0.0, 1.0);
        let brake = c.brake.clamp(0.0, 1.0);
        let mut fx = if c.reverse != 0 {
            if self.vx > -6.0 {
                -throttle * p.max_drive_force * 0.5
            } else {
                0.0
            }
        } else {
            throttle * p.max_drive_force.min(p.power / self.vx.max(1.0))
        };
        fx -= drag_factor * p.drag * self.vx * self.vx.abs();
        fx -= 0.015 * m * G * self.vx.signum() * (self.vx.abs() > 0.1) as i32 as f32;
        let fb = brake * p.max_brake_force;
        let grip_total = p.mu * m * G;
        // Brakes stop the wheels, they do not push the car backwards.
        if self.vx.abs() < fb / m * dt + 0.05 && (fx.abs() < fb) {
            self.vx = 0.0;
            fx = 0.0;
        } else {
            fx -= fb * self.vx.signum();
        }
        fx = fx.clamp(-grip_total, grip_total);
        let ellipse = (1.0 - (fx / grip_total).powi(2)).max(0.05).sqrt();

        if self.vx.abs() < 4.0 {
            // Kinematic bicycle.
            self.vx += fx / m * dt;
            let beta = (p.lr / l * delta.tan()).atan();
            self.vy = self.vx * beta.tan();
            self.yaw_rate = self.vx * beta.cos() * delta.tan() / l;
        } else {
            let vx = self.vx;
            let af = delta - (self.vy + p.lf * self.yaw_rate).atan2(vx.abs()) * vx.signum();
            let ar = -(self.vy - p.lr * self.yaw_rate).atan2(vx.abs()) * vx.signum();
            let fzf = m * G * p.lr / l;
            let fzr = m * G * p.lf / l;
            let fyf = (p.cf * af).clamp(-p.mu * fzf * ellipse, p.mu * fzf * ellipse);
            let fyr = (p.cr * ar).clamp(-p.mu * fzr * ellipse, p.mu * fzr * ellipse);
            let (sd, cd) = delta.sin_cos();
            let ax = (fx - fyf * sd) / m + self.vy * self.yaw_rate;
            let ay = (fyf * cd + fyr) / m - self.vx * self.yaw_rate;
            let rdot = (p.lf * fyf * cd - p.lr * fyr) / p.yaw_inertia;
            self.vx += ax * dt;
            self.vy += ay * dt;
            self.yaw_rate += rdot * dt;
        }
        self.heading = rdai_core::math::wrap_angle(self.heading + self.yaw_rate * dt);
        self.position += self.velocity_world() * dt;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn run(c: Controls, secs: f32) -> VehicleState {
        let p = VehicleParams::default();
        let mut s = VehicleState { vx: 30.0, ..Default::default() };
        let dt = 1.0 / 120.0;
        for _ in 0..(secs / dt) as usize {
            s.step(&p, &c, dt, 1.0);
        }
        s
    }

    #[test]
    fn full_brake_stops_within_physics() {
        // 30 m/s at ~12.7 m/s^2 -> ~35 m. Allow drag / ramp slack.
        let s = run(Controls { brake: 1.0, ..Default::default() }, 5.0);
        assert_eq!(s.vx, 0.0);
        assert!(s.position.x > 30.0 && s.position.x < 42.0, "{}", s.position.x);
    }

    #[test]
    fn steady_turn_does_not_exceed_grip() {
        let s0 = run(Controls { steer: 1.0, throttle: 0.4, ..Default::default() }, 6.0);
        let lat = (s0.vx * s0.yaw_rate).abs();
        assert!(lat < 1.3 * 9.81 * 1.05, "lateral accel {lat}");
    }
}
