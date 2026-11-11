//! Reactive local planner: Frenet-frame trajectory sampling (Werling et al. 2010), re-run every tick.
//!
//! Each tick it builds a fan of candidate paths that leave the car's current lateral state and
//! settle at different offsets from the reference line within a speed-dependent horizon. Every
//! candidate is checked against every nearby obstacle — other cars, a car that has just crashed
//! and is decelerating, debris standing still — using a constant-acceleration prediction. The
//! cheapest safe candidate wins. When no candidate is safe the planner picks the least severe one
//! and orders full braking (emergency mode): it reacts within one tick, not at the next checkpoint
//! as the TFG's A* did.
//!
//! Cost per tick: `candidates x samples x nearby obstacles` cheap float ops (~20 x 12 x k).

use crate::driver::DriverParams;
use crate::math::{wrap_angle, Vec2};
use crate::racing_line::RacingLine;
use crate::track::Track;

/// Anything the agent must not hit, in world coordinates. Cars, stopped cars, debris, pedestrians.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[repr(C)]
pub struct Obstacle {
    pub position: Vec2,
    pub heading: f32,
    /// Forward speed (m/s); 0 for static objects.
    pub speed: f32,
    /// Longitudinal acceleration (m/s^2); strongly negative for a car that is crashing or braking hard.
    pub accel: f32,
    pub length: f32,
    pub width: f32,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PlannerParams {
    /// Planning horizon in time (s); the distance horizon is `max(min_horizon, v * horizon_time)`.
    pub horizon_time: f32,
    pub min_horizon: f32,
    /// Time over which a candidate reaches its end offset (s); shorter = snappier lateral reactions.
    pub transition_time: f32,
    pub min_transition: f32,
    /// Lateral spacing between candidate end offsets (m).
    pub lateral_step: f32,
    /// Samples checked along each candidate.
    pub samples: usize,
    /// Ego footprint.
    pub length: f32,
    pub width: f32,
    /// Minimum standstill gap to the car ahead (m).
    pub standstill_gap: f32,
    /// Distance the car may use beyond the track edge in an emergency (m).
    pub runoff: f32,
    /// Obstacles further than this (m) are ignored.
    pub perception_range: f32,
}

impl Default for PlannerParams {
    fn default() -> Self {
        PlannerParams {
            horizon_time: 2.2,
            min_horizon: 18.0,
            transition_time: 0.9,
            min_transition: 10.0,
            lateral_step: 0.7,
            samples: 12,
            length: 4.6,
            width: 1.9,
            standstill_gap: 2.0,
            runoff: 1.5,
            perception_range: 160.0,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum Mode {
    /// Free road, following the reference line.
    Cruise = 0,
    /// Held behind an obstacle in the chosen path, matching its speed.
    Follow = 1,
    /// Off the line on purpose to pass or avoid something.
    Avoid = 2,
    /// Every path collides: full braking on the least severe one.
    Emergency = 3,
}

/// A lateral profile `e(sigma)` (offset from the reference line) as a quintic in arc length.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Quintic {
    pub c: [f32; 6],
    pub horizon: f32,
}

impl Quintic {
    /// From (e0, e0', 0) at sigma = 0 to (e1, 0, 0) at sigma = h.
    pub fn new(e0: f32, de0: f32, e1: f32, h: f32) -> Quintic {
        let a = e1 - e0 - de0 * h;
        let b = -de0;
        let (h2, h3) = (h * h, h * h * h);
        Quintic {
            c: [
                e0,
                de0,
                0.0,
                (10.0 * a - 4.0 * b * h) / h3,
                (-15.0 * a + 7.0 * b * h) / (h3 * h),
                (6.0 * a - 3.0 * b * h) / (h3 * h2),
            ],
            horizon: h,
        }
    }
    #[inline]
    pub fn value(&self, s: f32) -> f32 {
        let x = s.min(self.horizon);
        let c = &self.c;
        c[0] + x * (c[1] + x * (c[2] + x * (c[3] + x * (c[4] + x * c[5]))))
    }
    #[inline]
    pub fn second(&self, s: f32) -> f32 {
        if s >= self.horizon {
            return 0.0;
        }
        let c = &self.c;
        2.0 * c[2] + s * (6.0 * c[3] + s * (12.0 * c[4] + s * 20.0 * c[5]))
    }
}

/// Result of one planning tick.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Plan {
    pub mode: Mode,
    /// Track arc length where the plan starts.
    pub s0: f32,
    /// Chosen lateral profile relative to the reference line.
    pub lateral: Quintic,
    /// Speed the car should aim for now (m/s), already including braking for anything ahead.
    pub target_speed: f32,
    /// Feedforward acceleration (m/s^2): how fast the target speed changes along the path.
    pub accel_ff: f32,
    /// Upper bound on longitudinal acceleration from car-following (m/s^2); negative = brake.
    pub accel_limit: f32,
    /// Held up by a slower car on the reference line (feeds driver aggression).
    pub blocked: bool,
    pub cost: f32,
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct CandidateDebug {
    pub lateral: Quintic,
    pub cost: f32,
    pub collides: bool,
}

/// Ego state the planner needs, in Frenet coordinates.
#[derive(Clone, Copy, Debug)]
pub struct EgoFrenet {
    pub s: f32,
    pub d: f32,
    /// Heading relative to the track direction (rad).
    pub heading_error: f32,
    pub speed: f32,
}

#[derive(Clone, Copy, Debug)]
struct ObsFrenet {
    s: f32,
    d: f32,
    v_s: f32,
    v_d: f32,
    accel: f32,
    half_len: f32,
    half_wid: f32,
}

impl ObsFrenet {
    /// Predicted (s, d) at time t: constant acceleration until it stops, lateral drift for 1.5 s.
    #[inline]
    fn at(&self, t: f32) -> (f32, f32) {
        let ds = if self.accel < 0.0 && self.v_s > 0.0 {
            let t_stop = self.v_s / -self.accel;
            let tt = t.min(t_stop);
            self.v_s * tt + 0.5 * self.accel * tt * tt
        } else {
            self.v_s * t + 0.5 * self.accel.max(0.0) * t * t
        };
        (self.s + ds, self.d + self.v_d * t.min(1.5))
    }
    /// Distance it still travels before stopping (infinite if it is not braking).
    #[inline]
    fn stopping_distance(&self) -> f32 {
        if self.accel < -0.1 {
            self.v_s.max(0.0).powi(2) / (-2.0 * self.accel)
        } else if self.v_s <= 0.1 {
            0.0
        } else {
            f32::INFINITY
        }
    }
}

const COLLISION_COST: f32 = 1.0e4;
/// Cost per m/s of speed a candidate gives up (behind a leader, or through its own curvature).
const SPEED_LOSS_COST: f32 = 1.2;
/// Cost per m/s a candidate's speed envelope falls below what the car can brake to (infeasible).
const INFEASIBLE_COST: f32 = 100.0;

/// Reusable planner (keeps buffers and the previous choice for hysteresis).
#[derive(Clone, Debug)]
pub struct Planner {
    pub params: PlannerParams,
    prev_delta: f32,
    obstacles: Vec<ObsFrenet>,
    pub debug: Option<Vec<CandidateDebug>>,
}

impl Planner {
    pub fn new(params: PlannerParams) -> Self {
        Planner { params, prev_delta: 0.0, obstacles: Vec::with_capacity(32), debug: None }
    }

    /// Plans one tick. `obstacles` are in world coordinates; anything out of range is ignored.
    pub fn plan(
        &mut self,
        track: &Track,
        line: &RacingLine,
        ego: &EgoFrenet,
        ego_pos: Vec2,
        ego_index: usize,
        obstacles: &[Obstacle],
        drv: &DriverParams,
    ) -> Plan {
        let p = self.params;
        let v = ego.speed.max(0.0);
        let grip = drv.grip_usage;
        let b_eff = line.limits.max_brake * grip * 0.9;

        // 1. Obstacles to Frenet (only nearby ones; projection is a local search).
        self.obstacles.clear();
        for o in obstacles {
            let dist = (o.position - ego_pos).length();
            if dist > p.perception_range {
                continue;
            }
            let window = (dist / track.spacing()) as usize + 8;
            let pr = track.project_window(o.position, ego_index, window);
            let rel = wrap_angle(o.heading - track.heading_at(pr.s));
            self.obstacles.push(ObsFrenet {
                s: pr.s,
                d: pr.d,
                v_s: o.speed * rel.cos(),
                v_d: o.speed * rel.sin(),
                accel: o.accel * rel.cos(),
                half_len: 0.5 * o.length,
                half_wid: 0.5 * o.width,
            });
        }

        // 2. Lateral state relative to the reference line.
        let line_d = line.offset_at(track, ego.s);
        let line_slope = (line.offset_at(track, ego.s + 1.0) - line.offset_at(track, ego.s - 1.0)) * 0.5;
        let e0 = ego.d - line_d;
        let de0 = ego.heading_error.clamp(-1.2, 1.2).tan() - line_slope;
        let horizon = (v * p.horizon_time).max(p.min_horizon);
        let transition = (v * p.transition_time).max(p.min_transition).min(horizon);
        let s_end = ego.s + transition;
        let (wl, wr) = track.widths_at(s_end);
        let half_w = 0.5 * p.width;
        let d_line_end = line.offset_at(track, s_end);
        let lo = -wr + half_w + 0.2 - d_line_end;
        let hi = wl - half_w - 0.2 - d_line_end;

        // 3. Candidate end offsets: a regular fan plus "stay on the line" and "keep the previous choice".
        let mut deltas = [0.0f32; 48];
        let mut count = 0;
        let mut x = lo;
        while x <= hi + 1e-3 && count < deltas.len() - 2 {
            deltas[count] = x;
            count += 1;
            x += p.lateral_step;
        }
        deltas[count] = 0.0f32.clamp(lo, hi.max(lo));
        deltas[count + 1] = self.prev_delta.clamp(lo, hi.max(lo));
        count += 2;

        // Speed a candidate allows: line speed scaled by grip, capped by the candidate's own path
        // curvature, within the braking envelope over the horizon (from `from` metres ahead on).
        let a_lat = line.limits.max_lat_accel * grip;
        let envelope = |q: &Quintic, from: f32| {
            let mut best = f32::INFINITY;
            for k in 0..=p.samples {
                let sigma = from + (horizon - from) * k as f32 / p.samples as f32;
                let s = ego.s + sigma;
                let idx = track.index_at(s).0;
                let kappa = (line.curvature[idx] + q.second(sigma)).abs();
                let mut vk = line.speed_at(track, s) * grip.sqrt();
                if kappa > 1e-4 {
                    vk = vk.min((a_lat / kappa).sqrt());
                }
                best = best.min((vk * vk + 2.0 * b_eff * (sigma - from)).sqrt());
            }
            best
        };
        // What the line itself allows from here: the reference for each candidate's speed loss.
        // Werling et al. 2010 cost the lateral and longitudinal parts of a trajectory together;
        // costing only the lateral part picked "back to the line now" even when its curvature
        // demanded braking far beyond the line's own profile (docs/FINDINGS.md).
        let v_line = envelope(&Quintic::default(), 0.0);

        if let Some(dbg) = self.debug.as_mut() {
            dbg.clear();
        }
        let mut best: Option<(f32, Quintic, f32, Option<usize>, bool)> = None;
        let mut on_line_blocked = false;
        for (ci, &delta) in deltas[..count].iter().enumerate() {
            let q = Quintic::new(e0, de0, delta, transition);
            let mut cost = drv.line_attachment * 1.0 * delta * delta + 0.25 * (delta - self.prev_delta).powi(2);
            // Speed this path gives up against the line (same weight as speed lost behind a leader).
            let v_path = envelope(&q, 0.0);
            cost += SPEED_LOSS_COST * (v_line - v_path).max(0.0);
            // Below the current speed, following this path needs more than `b_eff` of braking: the
            // car cannot do it and would leave the path (Werling 2010 discards such candidates).
            // A steep cost instead of a discard keeps a least-bad choice when every path is too tight.
            // WIP (docs/FINDINGS.md): this also penalises the line itself whenever the car is above
            // the line's speed at sigma = 0, pushing it to flatter, outer paths (more wall hits).
            cost += INFEASIBLE_COST * (v.min(v_line) - v_path).max(0.0);
            let mut collides = false;
            let mut leader: Option<usize> = None;
            let mut v_cap = f32::INFINITY;
            for k in 1..=p.samples {
                let sigma = horizon * k as f32 / p.samples as f32;
                let s = ego.s + sigma;
                let d = line.offset_at(track, s) + q.value(sigma);
                // Smoothness.
                let e2 = q.second(sigma);
                cost += 40.0 * e2 * e2 * horizon / p.samples as f32;
                // Track bounds: soft inside the run-off, prohibitive beyond it.
                let (wl, wr) = track.widths_at(s);
                let over = (d + half_w - wl).max(-wr - d + half_w).max(0.0);
                if over > 0.0 {
                    cost += if over > p.runoff { COLLISION_COST } else { 200.0 * over * over };
                }
                // Obstacles.
                let t = sigma / v.max(3.0);
                for (oi, o) in self.obstacles.iter().enumerate() {
                    let (os, od) = o.at(t);
                    let ds = track.delta_s(s, os);
                    let reach_s = 0.5 * p.length + o.half_len + 0.5;
                    if ds.abs() > reach_s + 6.0 {
                        continue;
                    }
                    let clearance = (d - od).abs() - half_w - o.half_wid;
                    if ds.abs() < reach_s && clearance < drv.lateral_margin * 0.5 {
                        // Contact: classify by where the obstacle is now.
                        let gap0 = track.delta_s(ego.s, o.s) - 0.5 * p.length - o.half_len;
                        if gap0 > 0.0 {
                            // Ahead of us: survivable only if we can stop behind where it ends up.
                            let room = gap0 - p.standstill_gap + o.stopping_distance();
                            let need = v * v / (2.0 * b_eff);
                            if need <= room {
                                if leader.is_none() {
                                    leader = Some(oi);
                                    v_cap = v_cap.min(o.v_s.max(0.0));
                                }
                            } else {
                                collides = true;
                                let impact = v - (2.0 * b_eff * room.max(0.0)).sqrt();
                                cost += COLLISION_COST + 1.0e3 * impact.max(0.0);
                            }
                        } else if gap0 > -(p.length + 2.0 * o.half_len) {
                            // Alongside: never steer into it.
                            collides = true;
                            cost += COLLISION_COST;
                        }
                        // Behind us: its job to avoid us.
                    } else if clearance < drv.lateral_margin + 0.5 && ds.abs() < reach_s + 6.0 {
                        let m = drv.lateral_margin + 0.5 - clearance;
                        cost += 6.0 * m * m;
                    }
                }
            }
            if let Some(oi) = leader {
                // Speed lost by staying behind this obstacle.
                let v_line = line.speed_at(track, ego.s + horizon);
                let lost = (v_line - v_cap).max(0.0);
                cost += SPEED_LOSS_COST * lost;
                let _ = oi;
            }
            if ci == count - 2 && leader.is_some() {
                on_line_blocked = true;
            }
            if let Some(dbg) = self.debug.as_mut() {
                dbg.push(CandidateDebug { lateral: q, cost, collides });
            }
            if best.as_ref().map_or(true, |b| cost < b.0) {
                best = Some((cost, q, delta, leader, collides));
            }
        }
        let (cost, q, delta, leader, collides) = best.expect("at least two candidates");
        self.prev_delta = delta;

        // 4. Speed: line speed scaled by grip, path curvature, braking envelope over the horizon.
        //    Evaluated now and a short preview ahead; their difference is the feedforward
        //    deceleration, so braking starts on time instead of lagging a P loop.
        let preview = (v * 0.25).max(2.0);
        let mut target = envelope(&q, 0.0);
        let ahead = envelope(&q, preview);
        let accel_ff = ((ahead * ahead - target * target) / (2.0 * preview)).clamp(-line.limits.max_brake, line.limits.max_accel);

        // 5. Car following (IDM interaction term, Treiber et al. 2000) on the chosen path's leader.
        let mut accel_limit = f32::INFINITY;
        if let Some(oi) = leader {
            let o = self.obstacles[oi];
            let gap = (track.delta_s(ego.s, o.s) - 0.5 * p.length - o.half_len).max(0.1);
            let a_max = line.limits.max_accel;
            let dv = v - o.v_s;
            let s_star = p.standstill_gap + v * drv.follow_time_gap + v * dv / (2.0 * (a_max * b_eff).sqrt());
            let a_idm = a_max * (1.0 - (s_star.max(0.0) / gap).powi(2));
            accel_limit = a_idm.max(-line.limits.max_brake);
        }

        let mode = if collides {
            accel_limit = -line.limits.max_brake;
            target = 0.0;
            Mode::Emergency
        } else if delta.abs() > 0.8 {
            Mode::Avoid
        } else if leader.is_some() {
            Mode::Follow
        } else {
            Mode::Cruise
        };
        Plan { mode, s0: ego.s, lateral: q, target_speed: target, accel_ff, accel_limit, blocked: on_line_blocked, cost }
    }
}

/// World point of a plan at `sigma` metres ahead of its start (for steering and drawing).
pub fn plan_point(track: &Track, line: &RacingLine, s0: f32, q: &Quintic, sigma: f32) -> Vec2 {
    let s = s0 + sigma;
    track.to_world(s, line.offset_at(track, s) + q.value(sigma))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn quintic_meets_boundary_conditions() {
        let q = Quintic::new(1.5, 0.2, -2.0, 30.0);
        assert!((q.value(0.0) - 1.5).abs() < 1e-5);
        assert!((q.value(30.0) + 2.0).abs() < 1e-3);
        let h = 1e-2;
        let slope0 = (q.value(h) - q.value(0.0)) / h;
        assert!((slope0 - 0.2).abs() < 1e-2, "slope {slope0}");
        let slope1 = (q.value(30.0) - q.value(30.0 - h)) / h;
        assert!(slope1.abs() < 1e-2, "end slope {slope1}");
        assert!(q.second(29.999).abs() < 1e-2);
    }
}
