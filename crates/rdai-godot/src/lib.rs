//! Godot 4 GDExtension over `rdai-core`: `RdaiTrack` (circuit + racing line, built once, shared)
//! and `RdaiDriver` (one per AI car, ticked from `_physics_process`).
//!
//! Coordinates: the classes take Godot **2D** conventions — Y down, `rotation` in radians growing
//! clockwise on screen, lengths in Godot units (`RdaiTrack.meters_per_unit` converts). A 3D game
//! passes `Vector2(pos.x, pos.z)` and `atan2(f.z, f.x)` with `f` the car's forward vector (for a
//! `VehicleBody3D`, `+basis.z`); see `docs/ENGINE-INTEGRATION.md`.
//! The core is SI, Y up, counter-clockwise; the conversion lives only in [`conv`] (AGENTS.md: engine
//! conventions are converted at the boundary). Physical sides do not change: `width_left` is the
//! driver's left, and `steer = +1` turns to the driver's left (counter-clockwise on screen).

use godot::prelude::*;
use rdai_core::{
    Agent, AgentConfig, DriverProfile, LineParams, Observation, PlannerParams, RacingLine, Track,
};

pub mod conv;

struct RdaiExtension;

#[gdextension]
unsafe impl ExtensionLibrary for RdaiExtension {}

/// A circuit and its racing line. Build once with `build`, share between drivers.
#[derive(GodotClass)]
#[class(base = RefCounted, init)]
pub struct RdaiTrack {
    /// Metres per Godot unit (1.0 for a 3D game in metres; e.g. 0.05 for 20 px per metre).
    #[var]
    #[init(val = 1.0)]
    meters_per_unit: f32,
    /// Car performance envelope used by `build` (SI; defaults: a generic GT car). Set these to
    /// what the engine's car can really do — grip `max_lat_accel` (m/s^2), `max_accel`,
    /// `max_brake` (m/s^2), `power_to_mass` (W/kg), `drag_per_mass` (1/m), `max_speed` (m/s) —
    /// or the AI will plan corners the car cannot take.
    #[var]
    #[init(val = 12.0)]
    max_lat_accel: f32,
    #[var]
    #[init(val = 7.0)]
    max_accel: f32,
    #[var]
    #[init(val = 13.0)]
    max_brake: f32,
    #[var]
    #[init(val = 330.0)]
    power_to_mass: f32,
    #[var]
    #[init(val = 0.00035)]
    drag_per_mass: f32,
    #[var]
    #[init(val = 80.0)]
    max_speed: f32,
    built: Option<(Track, RacingLine)>,
}

#[godot_api]
impl RdaiTrack {
    /// Builds the circuit from its closed centreline (Godot units) and the half-widths to the
    /// driver's left and right of it, with the car limits set on this object. Returns false on bad
    /// input (sizes differ, < 4 points, non-finite values, non-positive limits).
    #[func]
    fn build(
        &mut self,
        centre: PackedVector2Array,
        width_left: PackedFloat32Array,
        width_right: PackedFloat32Array,
    ) -> bool {
        let pts: Vec<(f32, f32)> = centre.as_slice().iter().map(|v| (v.x, v.y)).collect();
        let Some(control) =
            conv::track_points(&pts, width_left.as_slice(), width_right.as_slice(), self.meters_per_unit)
        else {
            return false;
        };
        let Some(limits) = conv::limits(
            self.max_lat_accel,
            self.max_accel,
            self.max_brake,
            self.power_to_mass,
            self.drag_per_mass,
            self.max_speed,
        ) else {
            return false;
        };
        let Ok(track) = Track::from_control_points(&control, 2.0) else {
            return false;
        };
        let line = RacingLine::build(&track, &LineParams::default(), &limits);
        self.built = Some((track, line));
        true
    }

    /// Ideal lap time of the racing line (s); -1 before `build`.
    #[func]
    fn ideal_lap(&self) -> f32 {
        self.built.as_ref().map_or(-1.0, |(_, l)| l.lap_time)
    }

    /// The racing line in Godot units, for drawing.
    #[func]
    fn racing_line(&self) -> PackedVector2Array {
        let Some((_, line)) = &self.built else { return PackedVector2Array::new() };
        line.points
            .iter()
            .map(|p| {
                let (x, y) = conv::to_godot(*p, self.meters_per_unit);
                Vector2::new(x, y)
            })
            .collect()
    }
}

/// One AI driver. `setup` once, then `tick` every physics frame.
#[derive(GodotClass)]
#[class(base = RefCounted, init)]
pub struct RdaiDriver {
    /// Car geometry used by `setup`, SI (metres, radians; not Godot units): `steer = 1` means
    /// `max_steer`. Defaults: a GT car (2.7 m wheelbase, 0.45 rad, 4.6 x 1.9 m).
    #[var]
    #[init(val = 2.7)]
    wheelbase: f32,
    #[var]
    #[init(val = 0.45)]
    max_steer: f32,
    #[var]
    #[init(val = 4.6)]
    length: f32,
    #[var]
    #[init(val = 1.9)]
    width: f32,
    agent: Option<Agent>,
    track: Option<Gd<RdaiTrack>>,
}

#[godot_api]
impl RdaiDriver {
    /// Binds the driver to a built track with a personality (each 0..1) and a seed (same seed and
    /// inputs -> same outputs), with the geometry set on this object. Returns false (and stays
    /// unset) on a bad geometry.
    #[func]
    fn setup(&mut self, track: Gd<RdaiTrack>, seed: i64, experience: f32, stamina: f32, aggression: f32) -> bool {
        let Some(g) = conv::geometry(self.wheelbase, self.max_steer, self.length, self.width) else {
            self.agent = None;
            return false;
        };
        self.agent = Some(Agent::new(AgentConfig {
            geometry: g,
            planner: PlannerParams { length: g.length, width: g.width, ..Default::default() },
            profile: DriverProfile { experience, stamina, aggression },
            seed: seed as u64,
        }));
        self.track = Some(track);
        true
    }

    /// One AI step. `position` and `rotation` in Godot 2D conventions, `speed` forward in units/s
    /// (negative reversing), `obstacles` flat: 7 floats each (x, y, rotation, speed, accel,
    /// length, width; Godot units). Returns {steer, throttle, brake, reverse, mode}; an empty
    /// dictionary when not set up or on non-finite input.
    #[func]
    fn tick(
        &mut self,
        position: Vector2,
        rotation: f32,
        speed: f32,
        obstacles: PackedFloat32Array,
        dt: f32,
    ) -> VarDictionary {
        let (Some(agent), Some(track)) = (self.agent.as_mut(), self.track.as_ref()) else {
            return VarDictionary::new();
        };
        let t = track.bind();
        let Some((tr, line)) = &t.built else { return VarDictionary::new() };
        let mpu = t.meters_per_unit;
        let Some(list) = conv::obstacles(obstacles.as_slice(), mpu) else { return VarDictionary::new() };
        let obs = Observation {
            position: conv::to_core((position.x, position.y), mpu),
            heading: conv::heading_to_core(rotation),
            speed: speed * mpu,
        };
        if !conv::finite_obs(&obs) || !dt.is_finite() || dt <= 0.0 {
            return VarDictionary::new();
        }
        let c = agent.tick(tr, line, &obs, &list, dt);
        vdict! {
            "steer" => c.steer,
            "throttle" => c.throttle,
            "brake" => c.brake,
            "reverse" => c.reverse != 0,
            "mode" => c.mode as i64,
        }
    }
}
