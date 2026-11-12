//! One AI driver: owns its live driver state, planner buffers and recovery logic. The host engine
//! calls [`Agent::tick`] every physics step with the car's state and the obstacles around it, and
//! applies the returned [`Controls`] to its own vehicle model.

use crate::control::{lookahead, pure_pursuit, speed_control, VehicleGeometry};
use crate::driver::{DriverProfile, DriverState};
use crate::local_line::{CarFrenet, LocalLine, LocalParams, Replan};
use crate::math::{wrap_angle, Vec2};
use crate::planner::{plan_point, EgoFrenet, Mode, Obstacle, Plan, Planner, PlannerParams};
use crate::racing_line::RacingLine;
use crate::rng::Rng;
use crate::track::Track;

/// The controlled car's state, as the host engine sees it.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[repr(C)]
pub struct Observation {
    pub position: Vec2,
    pub heading: f32,
    /// Forward speed (m/s, negative when reversing).
    pub speed: f32,
}

/// Normalised driver inputs.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[repr(C)]
pub struct Controls {
    /// -1 (full right) .. 1 (full left), relative to `VehicleGeometry::max_steer`.
    pub steer: f32,
    pub throttle: f32,
    pub brake: f32,
    /// 1 = reverse gear (recovery manoeuvre).
    pub reverse: u8,
    /// [`Mode`] as a number, for HUDs and logs.
    pub mode: u8,
}

#[derive(Clone, Copy, Debug)]
pub struct AgentConfig {
    pub geometry: VehicleGeometry,
    pub planner: PlannerParams,
    pub profile: DriverProfile,
    pub seed: u64,
}

impl Default for AgentConfig {
    fn default() -> Self {
        let geometry = VehicleGeometry::default();
        AgentConfig {
            planner: PlannerParams { length: geometry.length, width: geometry.width, ..Default::default() },
            geometry,
            profile: DriverProfile::default(),
            seed: 1,
        }
    }
}

/// Gap (m) kept to anything behind while reversing out of a stuck position.
const REVERSE_CLEARANCE: f32 = 1.5;

#[derive(Clone, Debug)]
pub struct Agent {
    pub geometry: VehicleGeometry,
    pub driver: DriverState,
    pub planner: Planner,
    rng: Rng,
    hint: Option<usize>,
    last_plan: Option<Plan>,
    noise: f32,
    stuck_time: f32,
    reverse_time: f32,
    /// Position at the previous tick, to estimate the course (direction of travel).
    prev_position: Option<Vec2>,
    /// Leaky integral of the offset from the plan (m·s).
    track_int: f32,
    /// The agent's own line, re-optimised over the road ahead while it drives (`local_line.rs`);
    /// created on the first tick, when the track is known.
    local: Option<LocalLine>,
}

/// Largest gap between course and body heading trusted as sideslip (rad); a bigger one is a
/// teleport (engine respawn) or a spin, and the body heading is used instead. A guess: racing
/// sideslip stays under ~0.1 rad.
/// Integral gain on the offset from the plan (rad per m·s) and its leak time (s). Guesses sized so
/// a 1 m offset held 2 s adds 0.04 rad, the sideslip measured in long corners.
const TRACK_INT_GAIN: f32 = 0.03;
const TRACK_INT_TAU: f32 = 5.0;

/// Brake fraction always allowed, however hard the car corners: past the limit the car must still
/// slow down. A guess.
const TRAIL_BRAKE_FLOOR: f32 = 0.3;

const MAX_SIDESLIP: f32 = 0.3;

/// Distance (m) to the nearest obstacle beyond which the agent may re-plan its line from where it
/// is. A guess: about 1.5 s at racing speed, longer than an overtake's lateral move.
const RESTART_CLEARANCE: f32 = 60.0;

/// Direction of travel from the displacement since the previous tick; the body heading when there
/// is no usable displacement (first tick, slow, reversing, teleported).
fn course_estimate(obs: &Observation, prev: Option<Vec2>) -> f32 {
    let Some(prev) = prev else { return obs.heading };
    let disp = obs.position - prev;
    if obs.speed < 1.0 || disp.length_sq() < 1e-8 {
        return obs.heading;
    }
    let course = disp.angle();
    if wrap_angle(course - obs.heading).abs() > MAX_SIDESLIP {
        return obs.heading;
    }
    course
}

impl Agent {
    pub fn new(cfg: AgentConfig) -> Agent {
        Agent {
            geometry: cfg.geometry,
            driver: DriverState::new(cfg.profile),
            planner: Planner::new(cfg.planner),
            rng: Rng::new(cfg.seed),
            hint: None,
            last_plan: None,
            noise: 0.0,
            stuck_time: 0.0,
            reverse_time: 0.0,
            prev_position: None,
            track_int: 0.0,
            local: None,
        }
    }

    pub fn last_plan(&self) -> Option<&Plan> {
        self.last_plan.as_ref()
    }

    /// Keeps per-candidate costs for drawing (costs a small allocation per tick).
    pub fn set_debug(&mut self, on: bool) {
        self.planner.debug = if on { Some(Vec::with_capacity(48)) } else { None };
    }

    /// The line the agent drives on (its own, re-optimised ahead of it), once it has ticked.
    pub fn line(&self) -> Option<&RacingLine> {
        self.local.as_ref().map(|l| &l.line)
    }

    /// One step. `limits_from` only gives the car's limits: the agent finds its own line.
    pub fn tick(
        &mut self,
        track: &Track,
        limits_from: &RacingLine,
        obs: &Observation,
        obstacles: &[Obstacle],
        dt: f32,
    ) -> Controls {
        let pr = track.project(obs.position, self.hint);
        self.hint = Some(pr.index);
        let heading_error = wrap_angle(obs.heading - track.heading_at(pr.s));
        // The planner starts from the direction the car travels, not where its body points: in a
        // corner the car slides a few degrees (sideslip), and read as heading error that made
        // every plan bend back for a phantom drift and cap the speed for it (docs/FINDINGS.md).
        // The course is estimated from the last displacement, so engines need not send velocity.
        let course = course_estimate(obs, self.prev_position.replace(obs.position));
        let course_error = wrap_angle(course - track.heading_at(pr.s));
        let ego = EgoFrenet { s: pr.s, d: pr.d, heading_error: course_error, speed: obs.speed };
        let drv = self.driver.params();
        if self.local.as_ref().is_none_or(|l| l.line.offset.len() != track.len()) {
            self.local = Some(LocalLine::new(track, &limits_from.limits, LocalParams::default()));
        }
        let Some(local) = self.local.as_mut() else { return Controls::default() };
        // The planner's previous choice is an offset from the line: when the line moves, re-base it.
        let probe = self.planner.prev_end_s();
        let before = probe.map(|x| local.line.offset_at(track, x));
        // Re-plan from the car only when nobody is near: in traffic the car is off its line because
        // of the others (following, passing, avoiding), and moving the line every manoeuvre is
        // measured from put cars in each other's way (measured: 8-car race 2 -> 209 contacts).
        let alone =
            obstacles.iter().all(|o| (o.position - obs.position).length_sq() > RESTART_CLEARANCE * RESTART_CLEARANCE);
        let car = CarFrenet { s: pr.s, d: pr.d, course_error };
        let replan = local.update(track, &car, alone, dt);
        if let (Some(x), Some(b), true) = (probe, before, replan != Replan::Kept) {
            self.planner.rebase(local.line.offset_at(track, x) - b, replan == Replan::Restarted);
        }
        let line = &local.line;
        let mut plan = self.planner.plan(track, line, &ego, obs.position, pr.index, obstacles, &drv);
        self.driver.update(dt, plan.blocked);

        // Pointing the wrong way: turn around slowly.
        if heading_error.abs() > 1.6 {
            plan.target_speed = plan.target_speed.min(6.0);
        }

        // Steering: pure pursuit on the chosen path + human-like noise (low-pass random walk).
        let v = obs.speed.max(0.0);
        let target = plan_point(track, line, plan.s0, &plan.lateral, lookahead(v));
        let mut steer = pure_pursuit(&self.geometry, obs.position, obs.heading, v, target);
        // Integral action on the offset from the plan: pure pursuit on body yaw tracks with a
        // steady offset ≈ sideslip × lookahead in long corners (docs/FINDINGS.md). Leaky, clamped.
        let off_plan = pr.d - line.offset_at(track, pr.s) - plan.lateral.value(0.0);
        if v > 5.0 && plan.mode == Mode::Cruise {
            self.track_int = (self.track_int * (1.0 - dt / TRACK_INT_TAU) + off_plan * dt).clamp(-3.0, 3.0);
        } else {
            self.track_int = 0.0;
        }
        steer -= TRACK_INT_GAIN * self.track_int;
        self.noise = 0.97 * self.noise + 0.03 * self.rng.signed() * drv.steer_noise * 8.0;
        steer += self.noise;

        // Longitudinal.
        let a = speed_control(v, plan.target_speed, plan.accel_ff, plan.accel_limit);
        let lim = &line.limits;
        let (mut throttle, mut brake) = if a >= 0.0 {
            ((a / lim.accel_available(v, 0.0).max(0.5)).clamp(0.0, 1.0), 0.0)
        } else {
            (0.0, (-a / lim.max_brake).clamp(0.0, 1.0))
        };
        // Trail braking: brake only with the grip the turn leaves (friction circle, Kamm; Milliken
        // & Milliken 1995 ch. 2). Braking at full while cornering at the limit took the lateral
        // grip and the car slid wide, which lowered the target and braked it harder still
        // (docs/FINDINGS.md). The commanded curvature comes from the steady-state bicycle,
        // delta = (L + K_us v^2) kappa (Rajamani eq. 3.15).
        let g = &self.geometry;
        let kappa_cmd = steer / (g.wheelbase + g.understeer_gradient * v * v).max(0.1);
        let lat_use = (v * v * kappa_cmd.abs() / lim.max_lat_accel).min(1.0);
        // Only braking for the road: braking for a car ahead or an obstacle stays whole.
        if plan.mode == Mode::Cruise {
            brake = brake.min((1.0 - lat_use * lat_use).sqrt().max(TRAIL_BRAKE_FLOOR));
        }
        if plan.mode == Mode::Emergency {
            throttle = 0.0;
            brake = 1.0;
        }

        // Recovery: stuck -> reverse for a moment, steering the other way. Stuck means stopped
        // either with free road ahead, or in Emergency (every path ahead collides: a car or debris
        // against the nose). Before, Emergency set the target to 0, so a car stopped against an
        // obstacle never backed off (docs/FINDINGS.md).
        let mut reverse = 0u8;
        if self.reverse_time > 0.0 && self.clear_behind(obs, obstacles) {
            self.reverse_time -= dt;
            reverse = 1;
            throttle = 0.5;
            brake = 0.0;
            steer = -steer;
        } else if self.reverse_time > 0.0 {
            // Something behind: stop backing off (look in the mirror before reversing).
            self.reverse_time = 0.0;
            throttle = 0.0;
            brake = 1.0;
        } else if obs.speed.abs() < 0.5
            && ((plan.target_speed > 3.0 && plan.mode != Mode::Follow) || plan.mode == Mode::Emergency)
        {
            self.stuck_time += dt;
            if self.stuck_time > 2.5 {
                self.stuck_time = 0.0;
                self.reverse_time = 3.0;
            }
        } else {
            self.stuck_time = 0.0;
        }

        self.last_plan = Some(plan);
        Controls {
            steer: (steer / self.geometry.max_steer).clamp(-1.0, 1.0),
            throttle,
            brake,
            reverse,
            mode: plan.mode as u8,
        }
    }

    /// Samples the last chosen path in world space (for debug drawing).
    /// No obstacle within `REVERSE_CLEARANCE` behind the car, inside its width (car frame).
    fn clear_behind(&self, obs: &Observation, obstacles: &[Obstacle]) -> bool {
        let fwd = Vec2::from_angle(obs.heading);
        let half_len = 0.5 * self.geometry.length;
        let half_wid = 0.5 * self.geometry.width;
        obstacles.iter().all(|o| {
            let rel = o.position - obs.position;
            let x = rel.dot(fwd);
            let y = fwd.cross(rel);
            let behind = -x - half_len - 0.5 * o.length;
            !(x < 0.0 && behind < REVERSE_CLEARANCE && y.abs() < half_wid + 0.5 * o.width + 0.3)
        })
    }

    pub fn plan_polyline(&self, track: &Track, out: &mut Vec<Vec2>) {
        out.clear();
        if let (Some(p), Some(line)) = (&self.last_plan, self.line()) {
            for k in 0..=16 {
                out.push(plan_point(track, line, p.s0, &p.lateral, p.lateral.horizon * k as f32 / 16.0));
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn obs(x: f32, y: f32, heading: f32, speed: f32) -> Observation {
        Observation { position: Vec2::new(x, y), heading, speed }
    }

    #[test]
    fn course_is_the_direction_of_the_last_displacement() {
        // Moved 1 m along +X and 0.05 m along +Y: course atan(0.05) whatever the body yaw.
        let c = course_estimate(&obs(1.0, 0.05, 0.1, 30.0), Some(Vec2::new(0.0, 0.0)));
        assert!((c - 0.05f32.atan()).abs() < 1e-5, "{c}");
    }

    #[test]
    fn course_falls_back_to_the_body_heading() {
        let h = 0.3;
        // First tick, slow, standing still, and a jump the wrong way (teleport or reversing).
        assert_eq!(course_estimate(&obs(1.0, 0.0, h, 30.0), None), h);
        assert_eq!(course_estimate(&obs(1.0, 0.0, h, 0.5), Some(Vec2::new(0.0, 0.0))), h);
        assert_eq!(course_estimate(&obs(1.0, 0.0, h, 30.0), Some(Vec2::new(1.0, 0.0))), h);
        assert_eq!(course_estimate(&obs(-1.0, 0.0, h, 30.0), Some(Vec2::new(0.0, 0.0))), h);
    }
}
