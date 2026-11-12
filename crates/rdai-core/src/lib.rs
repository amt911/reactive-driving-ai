//! # rdai-core
//!
//! Engine-agnostic reactive driving AI. No dependencies, no allocation in the per-tick hot path
//! (except optional debug output), deterministic for a given seed.
//!
//! Layers, slowest to fastest:
//! 1. [`track::Track`] — route geometry in a Frenet frame (`s` along, `d` across).
//! 2. [`racing_line::RacingLine`] — offline minimum-curvature line + [`speed_profile`].
//! 3. [`planner::Planner`] — every tick: Frenet candidate paths vs predicted obstacles,
//!    car-following (IDM) and emergency braking.
//! 4. [`control`] — pure pursuit steering + speed loop.
//! 5. [`driver`] — personality (experience / stamina / aggression) that tunes all of the above.
//!
//! [`agent::Agent`] ties them together behind one `tick` call.

pub mod agent;
pub mod control;
pub mod driver;
pub mod local_line;
pub mod math;
pub mod planner;
pub mod racing_line;
pub mod rng;
pub mod speed_profile;
pub mod track;

pub use agent::{Agent, AgentConfig, Controls, Observation};
pub use control::VehicleGeometry;
pub use driver::{DriverParams, DriverProfile, DriverState};
pub use local_line::{LocalLine, LocalParams};
pub use math::Vec2;
pub use planner::{Mode, Obstacle, Plan, PlannerParams};
pub use racing_line::{LineParams, RacingLine};
pub use speed_profile::VehicleLimits;
pub use track::{Track, TrackError, TrackPoint};
