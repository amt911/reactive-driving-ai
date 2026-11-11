//! One AI driver: owns its live driver state, planner buffers and recovery logic. The host engine
//! calls [`Agent::tick`] every physics step with the car's state and the obstacles around it, and
//! applies the returned [`Controls`] to its own vehicle model.

use crate::control::{lookahead, pure_pursuit, speed_control, VehicleGeometry};
use crate::driver::{DriverProfile, DriverState};
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
        }
    }

    pub fn last_plan(&self) -> Option<&Plan> {
        self.last_plan.as_ref()
    }

    /// Keeps per-candidate costs for drawing (costs a small allocation per tick).
    pub fn set_debug(&mut self, on: bool) {
        self.planner.debug = if on { Some(Vec::with_capacity(48)) } else { None };
    }

    pub fn tick(
        &mut self,
        track: &Track,
        line: &RacingLine,
        obs: &Observation,
        obstacles: &[Obstacle],
        dt: f32,
    ) -> Controls {
        let pr = track.project(obs.position, self.hint);
        self.hint = Some(pr.index);
        let heading_error = wrap_angle(obs.heading - track.heading_at(pr.s));
        let ego = EgoFrenet { s: pr.s, d: pr.d, heading_error, speed: obs.speed };
        let drv = self.driver.params();
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
        if plan.mode == Mode::Emergency {
            throttle = 0.0;
            brake = 1.0;
        }

        // Recovery: stuck with free road ahead -> reverse for a moment, steering the other way.
        let mut reverse = 0u8;
        if self.reverse_time > 0.0 {
            self.reverse_time -= dt;
            reverse = 1;
            throttle = 0.5;
            brake = 0.0;
            steer = -steer;
        } else if obs.speed.abs() < 0.5 && plan.target_speed > 3.0 && plan.mode != Mode::Follow {
            self.stuck_time += dt;
            if self.stuck_time > 2.5 {
                self.stuck_time = 0.0;
                self.reverse_time = 1.5;
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
    pub fn plan_polyline(&self, track: &Track, line: &RacingLine, out: &mut Vec<Vec2>) {
        out.clear();
        if let Some(p) = &self.last_plan {
            for k in 0..=16 {
                out.push(plan_point(track, line, p.s0, &p.lateral, p.lateral.horizon * k as f32 / 16.0));
            }
        }
    }
}
