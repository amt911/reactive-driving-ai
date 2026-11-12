//! Godot 2D <-> core conversions, free of Godot types so they are unit-tested without the engine.
//! Godot 2D: Y down, angles clockwise on screen, Godot units. Core: SI metres, Y up, angles
//! counter-clockwise from +X (AGENTS.md, SI units). Flipping Y re-describes the same scene, it
//! does not mirror it: the driver's left stays the driver's left.

use rdai_core::{Observation, Obstacle, TrackPoint, Vec2, VehicleGeometry, VehicleLimits};

/// Floats per obstacle in the flat array `RdaiDriver.tick` takes.
pub const OBSTACLE_STRIDE: usize = 7;

pub fn to_core((x, y): (f32, f32), meters_per_unit: f32) -> Vec2 {
    Vec2::new(x * meters_per_unit, -y * meters_per_unit)
}

pub fn to_godot(p: Vec2, meters_per_unit: f32) -> (f32, f32) {
    (p.x / meters_per_unit, -p.y / meters_per_unit)
}

pub fn heading_to_core(rotation: f32) -> f32 {
    -rotation
}

pub fn finite_obs(o: &Observation) -> bool {
    o.position.x.is_finite() && o.position.y.is_finite() && o.heading.is_finite() && o.speed.is_finite()
}

/// Control points for `Track::from_control_points`; `None` when sizes differ, there are fewer
/// than 4 points, a value is not finite or the scale is not positive.
pub fn track_points(
    centre: &[(f32, f32)],
    width_left: &[f32],
    width_right: &[f32],
    meters_per_unit: f32,
) -> Option<Vec<TrackPoint>> {
    let ok = centre.len() >= 4
        && centre.len() == width_left.len()
        && centre.len() == width_right.len()
        && meters_per_unit.is_finite()
        && meters_per_unit > 0.0;
    if !ok {
        return None;
    }
    centre
        .iter()
        .zip(width_left.iter().zip(width_right))
        .map(|(&p, (&wl, &wr))| {
            let c = to_core(p, meters_per_unit);
            let pt = TrackPoint { x: c.x, y: c.y, width_left: wl * meters_per_unit, width_right: wr * meters_per_unit };
            [pt.x, pt.y, pt.width_left, pt.width_right].iter().all(|v| v.is_finite()).then_some(pt)
        })
        .collect()
}

/// Obstacles from the flat array (x, y, rotation, speed, accel, length, width per obstacle, Godot
/// units); `None` when the length is not a multiple of the stride or a value is not finite.
pub fn obstacles(flat: &[f32], meters_per_unit: f32) -> Option<Vec<Obstacle>> {
    if !flat.len().is_multiple_of(OBSTACLE_STRIDE) || !flat.iter().all(|v| v.is_finite()) {
        return None;
    }
    Some(
        flat.as_chunks::<OBSTACLE_STRIDE>()
            .0
            .iter()
            .map(|o| Obstacle {
                position: to_core((o[0], o[1]), meters_per_unit),
                heading: heading_to_core(o[2]),
                speed: o[3] * meters_per_unit,
                accel: o[4] * meters_per_unit,
                length: o[5] * meters_per_unit,
                width: o[6] * meters_per_unit,
            })
            .collect(),
    )
}

/// The car's performance envelope (SI) for the racing line and planner; `None` unless every value
/// is finite and positive (drag may be zero).
pub fn limits(
    max_lat_accel: f32,
    max_accel: f32,
    max_brake: f32,
    power_to_mass: f32,
    drag_per_mass: f32,
    max_speed: f32,
) -> Option<VehicleLimits> {
    let positive = [max_lat_accel, max_accel, max_brake, power_to_mass, max_speed];
    let ok = positive.iter().all(|v| v.is_finite() && *v > 0.0) && drag_per_mass.is_finite() && drag_per_mass >= 0.0;
    ok.then_some(VehicleLimits { max_lat_accel, max_accel, max_brake, power_to_mass, drag_per_mass, max_speed })
}

/// The car's geometry (SI metres, radians), with the reported position at the car's centre: the
/// rear axle half a wheelbase back. `None` unless all are finite and positive and
/// `max_steer < pi/2`.
pub fn geometry(wheelbase: f32, max_steer: f32, length: f32, width: f32) -> Option<VehicleGeometry> {
    let ok = [wheelbase, max_steer, length, width].iter().all(|v| v.is_finite() && *v > 0.0)
        && max_steer < core::f32::consts::FRAC_PI_2;
    ok.then_some(VehicleGeometry {
        wheelbase,
        rear_axle_offset: wheelbase / 2.0,
        max_steer,
        length,
        width,
        ..Default::default()
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn godot_down_is_core_minus_y_and_scale_is_metres_per_unit() {
        // 20 px per metre: (40, 60) px is (2, -3) m.
        let c = to_core((40.0, 60.0), 0.05);
        assert!((c.x - 2.0).abs() < 1e-6 && (c.y + 3.0).abs() < 1e-6, "{c:?}");
        let (x, y) = to_godot(c, 0.05);
        assert!((x - 40.0).abs() < 1e-4 && (y - 60.0).abs() < 1e-4);
    }

    #[test]
    fn a_car_facing_screen_down_faces_core_minus_y() {
        // Godot rotation +pi/2 points the car along +Y on screen (down), i.e. core heading -pi/2.
        let h = heading_to_core(core::f32::consts::FRAC_PI_2);
        let dir = Vec2::from_angle(h);
        assert!(dir.x.abs() < 1e-6 && (dir.y + 1.0).abs() < 1e-6, "{dir:?}");
    }

    #[test]
    fn track_points_reject_bad_input_and_scale_widths() {
        let sq = [(0.0, 0.0), (100.0, 0.0), (100.0, 100.0), (0.0, 100.0)];
        let w = [10.0; 4];
        let pts = track_points(&sq, &w, &w, 0.5).expect("valid");
        assert_eq!(pts[2].y, -50.0);
        assert_eq!(pts[2].width_left, 5.0);
        assert!(track_points(&sq[..3], &w[..3], &w[..3], 1.0).is_none(), "fewer than 4 points");
        assert!(track_points(&sq, &w[..3], &w, 1.0).is_none(), "sizes differ");
        assert!(track_points(&sq, &w, &w, 0.0).is_none(), "scale");
        let mut nan = sq;
        nan[1].0 = f32::NAN;
        assert!(track_points(&nan, &w, &w, 1.0).is_none(), "NaN");
    }

    #[test]
    fn obstacles_parse_the_flat_array() {
        let flat = [10.0, 20.0, 0.5, 30.0, -2.0, 80.0, 40.0];
        let o = obstacles(&flat, 0.05).expect("valid");
        assert_eq!(o.len(), 1);
        assert!((o[0].position.y + 1.0).abs() < 1e-6 && (o[0].heading + 0.5).abs() < 1e-6);
        assert!((o[0].length - 4.0).abs() < 1e-6 && (o[0].speed - 1.5).abs() < 1e-6);
        assert!(obstacles(&flat[..6], 1.0).is_none(), "stride");
        assert!(obstacles(&[f32::NAN; 7], 1.0).is_none(), "NaN");
        assert_eq!(obstacles(&[], 1.0).map(|v| v.len()), Some(0));
    }

    #[test]
    fn limits_take_positive_finite_values_and_allow_zero_drag() {
        let l = limits(9.0, 5.0, 10.0, 200.0, 0.0, 50.0).expect("valid");
        assert_eq!((l.max_lat_accel, l.max_accel, l.max_brake), (9.0, 5.0, 10.0));
        assert_eq!((l.power_to_mass, l.drag_per_mass, l.max_speed), (200.0, 0.0, 50.0));
        assert!(limits(0.0, 5.0, 10.0, 200.0, 0.0, 50.0).is_none(), "zero grip");
        assert!(limits(9.0, 5.0, -1.0, 200.0, 0.0, 50.0).is_none(), "negative brake");
        assert!(limits(9.0, 5.0, 10.0, 200.0, -0.1, 50.0).is_none(), "negative drag");
        assert!(limits(9.0, f32::NAN, 10.0, 200.0, 0.0, 50.0).is_none(), "NaN");
    }

    #[test]
    fn geometry_puts_the_rear_axle_half_a_wheelbase_back() {
        let g = geometry(2.0, 0.6, 4.0, 1.8).expect("valid");
        assert_eq!((g.wheelbase, g.rear_axle_offset, g.max_steer), (2.0, 1.0, 0.6));
        assert_eq!((g.length, g.width), (4.0, 1.8));
        assert!(geometry(0.0, 0.6, 4.0, 1.8).is_none(), "zero wheelbase");
        assert!(geometry(2.0, 1.6, 4.0, 1.8).is_none(), "steer past pi/2");
        assert!(geometry(2.0, 0.6, f32::INFINITY, 1.8).is_none(), "inf");
    }
}
