//! Driver personality and live state, carried over from the TFG: experience, stamina and
//! aggression in [0, 1] change how hard the AI drives, how close it follows and how eagerly it
//! leaves the racing line to overtake. Only parameters change — the algorithms stay the same.

#[derive(Clone, Copy, Debug, PartialEq)]
#[repr(C)]
pub struct DriverProfile {
    /// Fixed for the race. Slows stamina loss and aggression build-up, reduces driving noise.
    pub experience: f32,
    /// Starts at 1, drains over the race. Low stamina = lower pace, more noise.
    pub stamina: f32,
    /// Builds while stuck behind someone, decays otherwise. High = later braking, tighter gaps.
    pub aggression: f32,
}

impl Default for DriverProfile {
    fn default() -> Self {
        DriverProfile { experience: 0.7, stamina: 1.0, aggression: 0.2 }
    }
}

/// Parameters the planner and controllers read, derived from the live state.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DriverParams {
    /// Fraction of the car's grip used (scales corner and braking limits).
    pub grip_usage: f32,
    /// Desired time gap when following (s).
    pub follow_time_gap: f32,
    /// Extra lateral clearance kept from other cars (m).
    pub lateral_margin: f32,
    /// Multiplier on the cost of leaving the racing line (lower = overtakes sooner).
    pub line_attachment: f32,
    /// Standard amplitude of steering noise (rad).
    pub steer_noise: f32,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DriverState {
    pub profile: DriverProfile,
    /// Aggression floor the live value decays back to (the value it was configured with).
    pub base_aggression: f32,
}

impl DriverState {
    pub fn new(profile: DriverProfile) -> Self {
        let p = DriverProfile {
            experience: profile.experience.clamp(0.0, 1.0),
            stamina: profile.stamina.clamp(0.0, 1.0),
            aggression: profile.aggression.clamp(0.0, 1.0),
        };
        DriverState { profile: p, base_aggression: p.aggression }
    }

    /// Advances the live state. `blocked` = currently held up by a slower car.
    pub fn update(&mut self, dt: f32, blocked: bool) {
        let p = &mut self.profile;
        // Full drain in 20 min (rookie) .. 60 min (veteran).
        p.stamina = (p.stamina - dt / (1200.0 + 2400.0 * p.experience)).clamp(0.0, 1.0);
        if blocked {
            p.aggression += dt * (0.03 + 0.05 * (1.0 - p.experience));
        } else if p.aggression > self.base_aggression {
            p.aggression -= dt * 0.02 * (0.5 + p.experience);
            p.aggression = p.aggression.max(self.base_aggression);
        }
        p.aggression = p.aggression.clamp(0.0, 1.0);
    }

    pub fn params(&self) -> DriverParams {
        let p = &self.profile;
        DriverParams {
            grip_usage: (0.90 + 0.06 * p.stamina + 0.04 * p.aggression).min(1.0),
            follow_time_gap: 0.9 - 0.6 * p.aggression,
            lateral_margin: 0.9 - 0.5 * p.aggression,
            line_attachment: 1.6 - 1.2 * p.aggression,
            steer_noise: 0.004 * (1.0 - p.experience) * (1.6 - p.stamina),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn aggression_builds_when_blocked_and_decays_to_base() {
        let mut d = DriverState::new(DriverProfile { experience: 0.5, stamina: 1.0, aggression: 0.1 });
        for _ in 0..600 {
            d.update(0.01, true);
        }
        assert!(d.profile.aggression > 0.3);
        for _ in 0..100_000 {
            d.update(0.01, false);
        }
        assert!((d.profile.aggression - 0.1).abs() < 1e-6);
    }

    #[test]
    fn tired_rookie_is_slower_and_noisier_than_fresh_veteran() {
        let rookie = DriverState::new(DriverProfile { experience: 0.0, stamina: 0.2, aggression: 0.0 }).params();
        let vet = DriverState::new(DriverProfile { experience: 1.0, stamina: 1.0, aggression: 0.0 }).params();
        assert!(rookie.grip_usage < vet.grip_usage);
        assert!(rookie.steer_noise > vet.steer_noise);
    }
}
