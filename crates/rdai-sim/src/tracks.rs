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
