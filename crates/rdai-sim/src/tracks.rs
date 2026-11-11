//! Built-in circuits and a loader for TUMFTM-style CSV files (`x_m,y_m,w_tr_right_m,w_tr_left_m`).

use rdai_core::TrackPoint;

/// An original test circuit (~3 km): long straight, fast sweepers, a hairpin and a chicane.
pub fn default_circuit() -> Vec<TrackPoint> {
    let pts: [(f32, f32); 17] = [
        (0.0, 0.0),
        (400.0, 0.0),
        (600.0, 40.0),
        (700.0, 150.0),
        (680.0, 300.0),
        (560.0, 360.0),
        (450.0, 320.0),
        (380.0, 380.0),
        (400.0, 500.0),
        (520.0, 560.0),
        (560.0, 680.0),
        (450.0, 760.0),
        (200.0, 740.0),
        (60.0, 640.0),
        (-60.0, 500.0),
        (-120.0, 300.0),
        (-80.0, 120.0),
    ];
    pts.iter().map(|&(x, y)| TrackPoint { x, y, width_left: 7.0, width_right: 7.0 }).collect()
}

/// Rounded rectangle (~700 m), quick to run in tests.
pub fn test_oval() -> Vec<TrackPoint> {
    let pts: [(f32, f32); 8] = [
        (0.0, 0.0),
        (200.0, 0.0),
        (240.0, 40.0),
        (240.0, 100.0),
        (200.0, 140.0),
        (0.0, 140.0),
        (-40.0, 100.0),
        (-40.0, 40.0),
    ];
    pts.iter().map(|&(x, y)| TrackPoint { x, y, width_left: 7.0, width_right: 7.0 }).collect()
}

/// Parses `x_m,y_m,w_tr_right_m,w_tr_left_m` rows; `#` lines and a header are skipped.
pub fn parse_csv(text: &str) -> Result<Vec<TrackPoint>, String> {
    let mut out = Vec::new();
    for (n, line) in text.lines().enumerate() {
        let line = line.trim();
        if line.is_empty() || line.starts_with('#') || line.starts_with('x') {
            continue;
        }
        let v: Result<Vec<f32>, _> = line.split(',').map(|c| c.trim().parse::<f32>()).collect();
        match v {
            Ok(v) if v.len() >= 4 => out.push(TrackPoint { x: v[0], y: v[1], width_right: v[2], width_left: v[3] }),
            _ => return Err(format!("line {}: expected 4 numbers", n + 1)),
        }
    }
    Ok(out)
}

/// A procedural closed circuit from `seed`: control points around a radial polygon (so the loop
/// cannot cross itself), with jittered angles, smoothed random radii and widths. Candidates whose
/// geometry is not drivable are rejected and the next one is tried, so every seed returns a
/// track: no corner tighter than its inside half-width + 1 m, and no two stretches more than
/// 60 m apart along the track closer than both half-widths + 3 m. Deterministic per seed.
pub fn procedural(seed: u64) -> Vec<TrackPoint> {
    let mut rng = rdai_core::rng::Rng::new(seed ^ 0x7EAC_C0DE);
    loop {
        let pts = procedural_candidate(&mut rng);
        if let Ok(track) = rdai_core::Track::from_control_points(&pts, 2.0) {
            if drivable(&track) {
                return pts;
            }
        }
    }
}

fn procedural_candidate(rng: &mut rdai_core::rng::Rng) -> Vec<TrackPoint> {
    use core::f32::consts::TAU;
    let n = 10 + (rng.next_u64() % 9) as usize;
    let scale = 250.0 + 300.0 * rng.next_f32();
    let step = TAU / n as f32;
    let raw: Vec<(f32, f32, f32)> = (0..n)
        .map(|i| (i as f32 * step + 0.35 * step * rng.signed(), 0.45 + 0.55 * rng.next_f32(), 6.0 + 3.0 * rng.next_f32()))
        .collect();
    // Smooth radius and width with the neighbours: fewer spikes, still varied.
    (0..n)
        .map(|i| {
            let (p, c, q) = (raw[(i + n - 1) % n], raw[i], raw[(i + 1) % n]);
            let r = scale * (0.25 * p.1 + 0.5 * c.1 + 0.25 * q.1);
            let w = 0.25 * p.2 + 0.5 * c.2 + 0.25 * q.2;
            TrackPoint { x: r * c.0.cos(), y: r * c.0.sin(), width_left: w, width_right: w }
        })
        .collect()
}

fn drivable(t: &rdai_core::Track) -> bool {
    use rdai_core::math::menger_curvature;
    let (n, ds) = (t.len(), t.spacing());
    if !(1500.0..=6000.0).contains(&t.length()) {
        return false;
    }
    for i in 0..n {
        let k = menger_curvature(t.point((i + n - 1) % n), t.point(i), t.point((i + 1) % n));
        let inner = if k > 0.0 { t.width_left(i) } else { t.width_right(i) };
        if k.abs() > 1e-6 && 1.0 / k.abs() <= inner + 1.0 {
            return false;
        }
    }
    let skip = (60.0 / ds) as usize;
    for i in (0..n).step_by(4) {
        for j in (i..n).step_by(4) {
            let gap = j - i;
            if gap.min(n - gap) < skip {
                continue;
            }
            let need = t.width_left(i).max(t.width_right(i)) + t.width_left(j).max(t.width_right(j)) + 3.0;
            if (t.point(i) - t.point(j)).length() <= need {
                return false;
            }
        }
    }
    true
}
