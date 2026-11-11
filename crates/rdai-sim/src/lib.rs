//! # rdai-sim
//!
//! Reference 2D physics and race world to exercise `rdai-core` headless: tests, benchmarks and
//! the viewer all run on this. Engines replace it with their own physics.

pub mod tracks;
pub mod vehicle;
pub mod world;

pub use vehicle::{VehicleParams, VehicleState};
pub use world::{CarSpec, Contact, Debris, World};

use rdai_core::DriverProfile;

/// A varied default field: names, experience, aggression.
pub fn default_field(n: usize) -> Vec<CarSpec> {
    let names = ["Alba", "Bruno", "Carla", "Dario", "Elena", "Fermin", "Gala", "Hugo", "Irene", "Jorge", "Kira", "Lucas"];
    (0..n)
        .map(|i| {
            let f = i as f32 / n.max(1) as f32;
            CarSpec {
                name: names[i % names.len()].to_string(),
                profile: DriverProfile {
                    experience: 0.35 + 0.6 * ((i * 7 % n.max(1)) as f32 / n.max(1) as f32),
                    stamina: 1.0,
                    aggression: 0.1 + 0.5 * f,
                },
                params: VehicleParams::default(),
            }
        })
        .collect()
}
