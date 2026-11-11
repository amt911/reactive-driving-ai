//! Race world for the demo and tests: N AI cars on one track, contacts, walls, slipstream,
//! laps and positions, plus scenario hooks (a car crashing ahead, debris on the road).

use rdai_core::{
    Agent, AgentConfig, Controls, DriverProfile, LineParams, Obstacle, Observation, PlannerParams, RacingLine, Track,
    TrackPoint, Vec2,
};

use crate::vehicle::{VehicleParams, VehicleState};

#[derive(Clone, Debug)]
pub struct CarSpec {
    pub name: String,
    pub profile: DriverProfile,
    pub params: VehicleParams,
}

#[derive(Clone, Debug)]
pub struct Car {
    pub name: String,
    pub params: VehicleParams,
    pub state: VehicleState,
    pub agent: Agent,
    /// Index into `World::lines`: the line of this car's class.
    pub line: usize,
    pub controls: Controls,
    pub accel: f32,
    prev_speed: f32,
    hint: Option<usize>,
    pub s: f32,
    pub lap: i32,
    pub progress: f32,
    lap_start: f32,
    pub lap_times: Vec<f32>,
    pub finished_at: Option<f32>,
    pub wall_hits: u32,
    on_wall: bool,
    pub contacts: u32,
    /// Seconds left with the AI switched off (spinning / crashed).
    pub disabled: f32,
}

#[derive(Clone, Copy, Debug)]
pub struct Debris {
    pub position: Vec2,
    pub radius: f32,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Contact {
    pub a: usize,
    /// Other car index, or `usize::MAX` for debris.
    pub b: usize,
}

pub struct World {
    pub track: Track,
    /// Racing line of car 0's class (for drawing and tests); every car drives `lines[car.line]`.
    pub line: RacingLine,
    /// One racing line and speed profile per distinct car class (`VehicleParams`) on the grid.
    pub lines: Vec<RacingLine>,
    pub cars: Vec<Car>,
    pub debris: Vec<Debris>,
    pub time: f32,
    pub laps: i32,
    /// New contacts in the last step (for tests and the HUD).
    pub new_contacts: Vec<Contact>,
    pub overtakes: u32,
    order: Vec<usize>,
    obstacles: Vec<Obstacle>,
    scratch: Vec<Obstacle>,
    /// Wall time spent inside `Agent::tick`, for benchmarks (ns) and the number of calls.
    pub agent_ns: u128,
    pub agent_calls: u64,
}

impl World {
    pub fn new(control: &[TrackPoint], specs: &[CarSpec], laps: i32) -> World {
        World::new_seeded(control, specs, laps, 0x5EED)
    }

    /// Like [`World::new`], with the base of the agents' RNG seeds (car `i` gets `seed + i`):
    /// the same race with different driving noise, to tell a real change from chaos.
    pub fn new_seeded(control: &[TrackPoint], specs: &[CarSpec], laps: i32, seed: u64) -> World {
        let track = Track::from_control_points(control, 2.0).expect("valid track");
        // One line per car class: a weaker car must not drive a stronger car's speed profile.
        let build = |p: &VehicleParams| {
            RacingLine::build(&track, &LineParams { margin: 0.5 * p.width + 1.0, ..Default::default() }, &p.limits())
        };
        let mut classes: Vec<VehicleParams> = Vec::new();
        let mut lines = Vec::new();
        let mut car_line = Vec::with_capacity(specs.len());
        for spec in specs {
            let k = match classes.iter().position(|c| *c == spec.params) {
                Some(k) => k,
                None => {
                    classes.push(spec.params);
                    lines.push(build(&spec.params));
                    classes.len() - 1
                }
            };
            car_line.push(k);
        }
        let line = lines.first().cloned().unwrap_or_else(|| build(&VehicleParams::default()));
        let mut w = World {
            cars: Vec::new(),
            debris: Vec::new(),
            time: 0.0,
            laps,
            new_contacts: Vec::new(),
            overtakes: 0,
            order: Vec::new(),
            obstacles: Vec::new(),
            scratch: Vec::new(),
            agent_ns: 0,
            agent_calls: 0,
            track,
            line,
            lines,
        };
        for (i, spec) in specs.iter().enumerate() {
            // Two-wide grid behind the start line, 9 m between rows.
            let s = w.track.length() - 10.0 - 9.0 * (i / 2) as f32;
            let d = if i % 2 == 0 { 2.5 } else { -2.5 };
            let pos = w.track.to_world(s, d);
            let heading = w.track.heading_at(s);
            let cfg = AgentConfig {
                geometry: spec.params.geometry(),
                planner: PlannerParams { length: spec.params.length, width: spec.params.width, ..Default::default() },
                profile: spec.profile,
                seed: seed.wrapping_add(i as u64),
            };
            w.cars.push(Car {
                name: spec.name.clone(),
                params: spec.params,
                state: VehicleState { position: pos, heading, ..Default::default() },
                agent: Agent::new(cfg),
                line: car_line[i],
                controls: Controls::default(),
                accel: 0.0,
                prev_speed: 0.0,
                hint: None,
                s,
                lap: -1,
                progress: s - w.track.length(),
                lap_start: 0.0,
                lap_times: Vec::new(),
                finished_at: None,
                wall_hits: 0,
                on_wall: false,
                contacts: 0,
                disabled: 0.0,
            });
        }
        w.order = w.positions();
        w
    }

    /// Car indices sorted by race position.
    pub fn positions(&self) -> Vec<usize> {
        let mut idx: Vec<usize> = (0..self.cars.len()).collect();
        idx.sort_by(|&a, &b| {
            let (ca, cb) = (&self.cars[a], &self.cars[b]);
            match (ca.finished_at, cb.finished_at) {
                (Some(x), Some(y)) => x.total_cmp(&y),
                (Some(_), None) => core::cmp::Ordering::Less,
                (None, Some(_)) => core::cmp::Ordering::Greater,
                _ => cb.progress.total_cmp(&ca.progress),
            }
        });
        idx
    }

    pub fn finished(&self) -> bool {
        self.cars.iter().all(|c| c.finished_at.is_some())
    }

    /// Makes car `i` lose control: AI off for `secs`, a yaw kick and locked brakes.
    pub fn crash_car(&mut self, i: usize, secs: f32) {
        let c = &mut self.cars[i];
        c.disabled = secs;
        c.state.yaw_rate += if c.state.vy >= 0.0 { 2.5 } else { -2.5 };
    }

    /// Drops debris on the road `ahead` metres in front of car `i` at lateral offset `d`.
    pub fn drop_debris(&mut self, i: usize, ahead: f32, d: f32, radius: f32) {
        let s = self.cars[i].s + ahead;
        self.debris.push(Debris { position: self.track.to_world(s, d), radius });
    }

    pub fn step(&mut self, dt: f32) {
        self.time += dt;
        self.new_contacts.clear();

        // Snapshot every car (and debris) as an obstacle.
        self.obstacles.clear();
        for c in &self.cars {
            self.obstacles.push(Obstacle {
                position: c.state.position,
                heading: c.state.heading,
                speed: c.state.vx,
                accel: c.accel,
                length: c.params.length,
                width: c.params.width,
            });
        }
        for d in &self.debris {
            self.obstacles.push(Obstacle {
                position: d.position,
                heading: 0.0,
                speed: 0.0,
                accel: 0.0,
                length: 2.0 * d.radius,
                width: 2.0 * d.radius,
            });
        }

        // AI + physics.
        let n = self.cars.len();
        for i in 0..n {
            self.scratch.clear();
            for (j, o) in self.obstacles.iter().enumerate() {
                if j != i {
                    self.scratch.push(*o);
                }
            }
            let car = &mut self.cars[i];
            let obs = Observation { position: car.state.position, heading: car.state.heading, speed: car.state.vx };
            let t0 = std::time::Instant::now();
            let ai = car.agent.tick(&self.track, &self.lines[car.line], &obs, &self.scratch, dt);
            self.agent_ns += t0.elapsed().as_nanos();
            self.agent_calls += 1;
            car.controls = if car.disabled > 0.0 {
                car.disabled -= dt;
                Controls { brake: 1.0, ..Default::default() }
            } else {
                ai
            };
            // Slipstream: a car 3..25 m ahead, within 1.6 m laterally, cuts drag by 40 %.
            let mut drag = 1.0;
            let fwd = Vec2::from_angle(car.state.heading);
            for (j, o) in self.obstacles.iter().enumerate().take(n) {
                if j == i {
                    continue;
                }
                let rel = o.position - car.state.position;
                let ahead = rel.dot(fwd);
                if (3.0..25.0).contains(&ahead) && rel.cross(fwd).abs() < 1.6 {
                    drag = 0.6;
                }
            }
            car.state.step(&car.params, &car.controls, dt, drag);
            car.accel = (car.state.vx - car.prev_speed) / dt;
            car.prev_speed = car.state.vx;
        }

        self.resolve_walls();
        self.resolve_contacts();
        self.update_progress();
    }

    fn resolve_walls(&mut self) {
        for car in &mut self.cars {
            let pr = self.track.project(car.state.position, car.hint);
            car.hint = Some(pr.index);
            let (wl, wr) = self.track.widths_at(pr.s);
            let half = 0.5 * car.params.width;
            let n = self.track.normal(pr.index);
            let (limit, out) = if pr.d > wl - half {
                (wl - half, n)
            } else if pr.d < -(wr - half) {
                (-(wr - half), -n)
            } else {
                car.on_wall = false;
                continue;
            };
            car.state.position = self.track.to_world(pr.s, limit);
            let v = car.state.velocity_world();
            let vn = v.dot(out);
            if vn > 0.0 {
                car.state.set_velocity_world((v - out * (1.3 * vn)) * 0.97);
                car.state.yaw_rate *= 0.5;
            }
            if !car.on_wall {
                car.wall_hits += 1;
                car.on_wall = true;
            }
        }
    }

    fn resolve_contacts(&mut self) {
        let n = self.cars.len();
        let circles = |c: &Car| {
            let f = Vec2::from_angle(c.state.heading) * (0.5 * c.params.length - 0.5 * c.params.width);
            [c.state.position + f, c.state.position - f]
        };
        for i in 0..n {
            for j in i + 1..n {
                let (ci, cj) = (circles(&self.cars[i]), circles(&self.cars[j]));
                let r = 0.5 * (self.cars[i].params.width + self.cars[j].params.width);
                let mut hit = false;
                for a in ci {
                    for b in cj {
                        let dvec = b - a;
                        let dist = dvec.length();
                        if dist < r && dist > 1e-4 {
                            hit = true;
                            let nrm = dvec * (1.0 / dist);
                            let push = nrm * (0.5 * (r - dist));
                            self.cars[i].state.position -= push;
                            self.cars[j].state.position += push;
                            let (vi, vj) = (self.cars[i].state.velocity_world(), self.cars[j].state.velocity_world());
                            let rel = (vj - vi).dot(nrm);
                            if rel < 0.0 {
                                let imp = nrm * (0.5 * 1.2 * rel);
                                self.cars[i].state.set_velocity_world(vi + imp);
                                self.cars[j].state.set_velocity_world(vj - imp);
                            }
                        }
                    }
                }
                if hit {
                    self.cars[i].contacts += 1;
                    self.cars[j].contacts += 1;
                    self.new_contacts.push(Contact { a: i, b: j });
                }
            }
            // Debris is immovable.
            for d in &self.debris {
                for a in circles(&self.cars[i]) {
                    let dvec = a - d.position;
                    let r = d.radius + 0.5 * self.cars[i].params.width;
                    let dist = dvec.length();
                    if dist < r && dist > 1e-4 {
                        let nrm = dvec * (1.0 / dist);
                        self.cars[i].state.position += nrm * (r - dist);
                        let v = self.cars[i].state.velocity_world();
                        let vn = v.dot(nrm);
                        if vn < 0.0 {
                            self.cars[i].state.set_velocity_world(v - nrm * (1.3 * vn));
                        }
                        self.new_contacts.push(Contact { a: i, b: usize::MAX });
                        self.cars[i].contacts += 1;
                    }
                }
            }
        }
    }

    fn update_progress(&mut self) {
        let len = self.track.length();
        for car in &mut self.cars {
            let pr = self.track.project(car.state.position, car.hint);
            let ds = self.track.delta_s(car.s, pr.s);
            car.s = pr.s;
            car.progress += ds;
            let lap_now = (car.progress / len).floor() as i32;
            if lap_now > car.lap {
                car.lap = lap_now;
                if lap_now >= 1 {
                    car.lap_times.push(self.time - car.lap_start);
                }
                car.lap_start = self.time;
                if lap_now >= self.laps && car.finished_at.is_none() {
                    car.finished_at = Some(self.time);
                }
            }
        }
        // Overtakes: adjacent position swaps among running cars.
        let order = self.positions();
        for (k, &c) in order.iter().enumerate() {
            if let Some(old) = self.order.iter().position(|&x| x == c) {
                if old > k && self.cars[c].finished_at.is_none() {
                    self.overtakes += (old - k) as u32;
                }
            }
        }
        self.order = order;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::tracks;

    #[test]
    fn each_car_class_drives_its_own_line() {
        // A weaker car (less power, less grip) must not inherit the speed profile of the first
        // car on the grid, and vice versa (docs/FINDINGS.md: every car used car 0's profile).
        let slow = VehicleParams { power: 200_000.0, mu: 1.1, ..VehicleParams::default() };
        let spec = |params| CarSpec { name: "x".into(), profile: DriverProfile::default(), params };
        let w = World::new(&tracks::default_circuit(), &[spec(slow), spec(VehicleParams::default())], 1);
        let (l0, l1) = (&w.lines[w.cars[0].line], &w.lines[w.cars[1].line]);
        // Oracle: the offline speed profile of each class; the faster class has the shorter lap.
        assert!(l1.lap_time < l0.lap_time - 1.0, "slow class {} s, fast class {} s", l0.lap_time, l1.lap_time);
        assert_eq!(w.lines.len(), 2);
    }
}
