use rdai_sim::{default_field, tracks, World};
#[test]
fn dbg_speed() {
    let mut w = World::new(&tracks::default_circuit(), &default_field(1), 3);
    let dt = 1.0 / 120.0;
    let mut next = 70.0;
    while w.time < 140.0 {
        w.step(dt);
        if w.time >= next && next < 90.0 {
            next += 15.0;
            let own = w.cars[0].agent.line().unwrap();
            let pr = w.track.project(w.cars[0].state.position, None);
            let row: Vec<String> = (1040..1070)
                .step_by(3)
                .map(|i| {
                    format!(
                        "{}: v{:.0}/{:.0} k{:.4}/{:.4} d{:.2}/{:.2}",
                        i * 2,
                        own.speed[i],
                        w.line.speed[i],
                        own.curvature[i],
                        w.line.curvature[i],
                        own.offset[i],
                        w.line.offset[i]
                    )
                })
                .collect();
            eprintln!("t {:.0} car s {:.0}: {}", w.time, pr.s, row.join(" "));
        }
    }
}
