//! Recovery: a car stopped nose-to-nose with an obstacle backs off and drives round it, as a
//! driver would, instead of waiting for ever (seen in the viewer, 2026-11-11: after a hit, cars
//! sat still against each other).

use rdai_sim::{default_field, tracks, World};

/// The car starts at rest with debris (r = 1.2 m) touching its front bumper. Within 20 s it must
/// be 10 m past the debris along the track.
/// Oracle: geometry — the track is 14 m wide, so a way round exists once the car has room to turn.
#[test]
fn car_stuck_against_an_obstacle_backs_off_and_goes_round() {
    let mut w = World::new(&tracks::default_circuit(), &default_field(1), 1);
    let (s0, d0) = (w.cars[0].s, w.track.project(w.cars[0].state.position, None).d);
    // Front bumper at +2.3 m; debris centre 3.5 m ahead: 0 m gap, dead ahead.
    w.drop_debris(0, 3.5, d0, 1.2);
    let s_debris = s0 + 3.5;
    let mut passed_at = None;
    while w.time < 20.0 {
        w.step(1.0 / 120.0);
        if w.track.delta_s(s_debris, w.cars[0].s) > 10.0 {
            passed_at = Some(w.time);
            break;
        }
    }
    let c = &w.cars[0];
    assert!(
        passed_at.is_some(),
        "still behind the debris after 20 s: ds {:.1} m, v {:.1}, mode {}",
        w.track.delta_s(s_debris, c.s),
        c.state.vx,
        c.controls.mode
    );
}

/// Backing off looks behind first: with a car stopped 7 m behind (bumper to bumper ~2.4 m), the
/// stuck car may reverse only into the free space, and must never touch it.
/// Measured 2026-11-11: blind 3 s reverses doubled contacts on the bench matrix (924 -> 1986).
#[test]
fn reversing_never_backs_into_the_car_behind() {
    use rdai_core::DriverProfile;
    use rdai_sim::{CarSpec, VehicleParams};
    let spec =
        |name: &str| CarSpec { name: name.into(), profile: DriverProfile::default(), params: VehicleParams::default() };
    // Grid: cars 0 and 1 side by side, car 2 9 m behind car 0 (same lane).
    let mut w = World::new(&tracks::default_circuit(), &[spec("Stuck"), spec("Side"), spec("Behind")], 1);
    let d0 = w.track.project(w.cars[0].state.position, None).d;
    w.drop_debris(0, 3.5, d0, 1.2);
    // The car behind is parked: it never moves (disabled for the whole test).
    w.cars[2].disabled = 1e9;
    w.cars[1].disabled = 1e9;
    let mut touched = false;
    while w.time < 20.0 {
        w.step(1.0 / 120.0);
        touched |= w.new_contacts.iter().any(|c| (c.a == 0 && c.b == 2) || (c.a == 2 && c.b == 0));
    }
    assert!(!touched, "the stuck car backed into the parked car behind it");
}
