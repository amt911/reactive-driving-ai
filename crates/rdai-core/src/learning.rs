//! Lap-to-lap learning of where the car has less grip than it declared (spec
//! `docs/specs/2026-11-12-local-planning.md`, phase 2). Iterative learning control (Arimoto et al.
//! 1984; for racing, Kapania & Gerdes 2015, PAPERS.md): the same corner comes back every lap, so the
//! error seen there on one lap corrects the next one.
//!
//! Signal: how far the car ran **wide of its plan in a corner** (outward, metres), the only sign of
//! missing grip an engine's observations carry (position, heading, speed). Where it ran wide, the
//! grip scale of that sample and of the road before it (braking and turn-in, where the speed was
//! chosen) drops for the next lap; elsewhere it creeps back towards the declared grip. No
//! allocation after `new`; deterministic.

/// Tuning of [`LapMemory`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LearnParams {
    /// Running wide by less than this (m) is ordinary tracking error, not missing grip. Guess,
    /// from solo traces at full grip (tracking error < 0.4 m).
    pub tolerance: f32,
    /// Road before a slide whose grip is lowered with it (m): the braking zone and turn-in that
    /// set the speed. Guess, of the order of a braking zone at corner speeds.
    pub upstream: f32,
    /// Grip scale removed per metre wide beyond the tolerance, per lap. Measured on the 10 %-grip
    /// journey (`rdai-sim/tests/learning.rs`): 0.05 and 0.15 still hit walls on laps 4-5 (the
    /// controller keeps a sliding car within ~1 m of its plan, so the signal is small); 0.3 hits
    /// only on lap 1 and settles.
    pub rate: f32,
    /// Grip scale given back per lap where the car did not run wide.
    pub recover: f32,
    /// Lowest grip scale (spec: 0.85 of the declared grip).
    pub min_grip: f32,
}

impl Default for LearnParams {
    fn default() -> Self {
        LearnParams { tolerance: 0.5, upstream: 60.0, rate: 0.3, recover: 0.005, min_grip: 0.85 }
    }
}

/// Per-track-sample memory of the grip the car really has, as a scale of its declared grip.
#[derive(Clone, Debug)]
pub struct LapMemory {
    params: LearnParams,
    upstream: usize,
    grip: Vec<f32>,
    wide: Vec<f32>,
    corr: Vec<f32>,
    last: Option<usize>,
}

impl LapMemory {
    /// For a track of `samples` samples `spacing` m apart.
    pub fn new(samples: usize, spacing: f32, params: LearnParams) -> LapMemory {
        let n = samples.max(1);
        LapMemory {
            params,
            upstream: (params.upstream / spacing.max(1e-3)).round() as usize,
            grip: vec![1.0; n],
            wide: vec![0.0; n],
            corr: vec![0.0; n],
            last: None,
        }
    }

    /// Grip scale per track sample (1 = the declared grip).
    pub fn grip(&self) -> &[f32] {
        &self.grip
    }

    /// The car is at track sample `i`, `wide` m outside its plan in a corner (0 when not in a
    /// corner or not running wide). Returns true when this tick ended a lap (and learned from it).
    pub fn observe(&mut self, i: usize, wide: f32) -> bool {
        let n = self.grip.len();
        let i = i % n;
        let Some(last) = self.last.replace(i) else {
            self.wide[i] = self.wide[i].max(wide);
            return false;
        };
        let wrapped = last > i && last - i > n / 2;
        // Samples passed since the last tick (a fast car skips some); a step back marks nothing.
        let ahead = (i + n - last) % n;
        if ahead <= n / 2 {
            for k in 1..=ahead {
                let j = (last + k) % n;
                self.wide[j] = self.wide[j].max(wide);
            }
        }
        if wrapped {
            self.end_lap();
        }
        wrapped
    }

    /// Learns from the lap just driven and clears its record. The sample just crossed into
    /// belongs to the new lap and keeps its mark.
    fn end_lap(&mut self) {
        let n = self.grip.len();
        let p = self.params;
        self.corr.fill(0.0);
        for i in 0..n {
            let excess = self.wide[i] - p.tolerance;
            if excess <= 0.0 {
                continue;
            }
            for j in 0..=self.upstream.min(n - 1) {
                let k = (i + n - j) % n;
                self.corr[k] = self.corr[k].max(excess);
            }
        }
        for (g, &c) in self.grip.iter_mut().zip(&self.corr) {
            *g = if c > 0.0 { (*g - p.rate * c).max(p.min_grip) } else { (*g + p.recover).min(1.0) };
        }
        let keep = self.last.map(|i| self.wide[i]).unwrap_or(0.0);
        self.wide.fill(0.0);
        if let Some(i) = self.last {
            self.wide[i] = keep;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn lap(m: &mut LapMemory, n: usize, slide_at: Option<(usize, f32)>) {
        for i in 0..n {
            let wide = match slide_at {
                Some((k, w)) if k == i => w,
                _ => 0.0,
            };
            m.observe(i, wide);
        }
        assert!(m.observe(0, 0.0), "crossing the line ends the lap");
    }

    #[test]
    fn a_slide_lowers_the_grip_of_the_road_before_it() {
        // 100 samples, 2 m apart, upstream 60 m = 30 samples. Slide 0.75 m wide at sample 50:
        // 0.25 m over the tolerance -> `rate` x 0.25 off samples 20..=50, nothing elsewhere.
        let p = LearnParams::default();
        let mut m = LapMemory::new(100, 2.0, p);
        m.observe(0, 0.0);
        lap(&mut m, 100, Some((50, 0.75)));
        let g = m.grip();
        for (i, &gi) in g.iter().enumerate() {
            let want = if (20..=50).contains(&i) { 1.0 - p.rate * 0.25 } else { 1.0 };
            assert!((gi - want).abs() < 1e-6, "sample {i}: {gi} vs {want}");
        }
    }

    #[test]
    fn grip_comes_back_slowly_and_stays_within_bounds() {
        let p = LearnParams::default();
        let mut m = LapMemory::new(100, 2.0, p);
        m.observe(0, 0.0);
        for _ in 0..20 {
            lap(&mut m, 100, Some((50, 9.0)));
        }
        assert!((m.grip()[50] - p.min_grip).abs() < 1e-6, "floor: {}", m.grip()[50]);
        lap(&mut m, 100, None);
        assert!((m.grip()[50] - (p.min_grip + p.recover)).abs() < 1e-6, "recover: {}", m.grip()[50]);
        for _ in 0..100 {
            lap(&mut m, 100, None);
        }
        assert!(m.grip().iter().all(|&g| g == 1.0), "never above the declared grip");
    }

    #[test]
    fn a_lap_ends_only_when_the_car_crosses_the_line() {
        let mut m = LapMemory::new(100, 2.0, LearnParams::default());
        assert!(!m.observe(10, 0.0));
        assert!(!m.observe(9, 0.0), "a small step back is not a lap");
        assert!(!m.observe(99, 0.0));
        assert!(m.observe(1, 0.0));
        assert!(!m.observe(2, 0.0));
    }
}
