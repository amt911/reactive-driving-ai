//! Headless race / scenario runner. Prints results and the AI's cost per tick.
//!
//! cargo run --release -p rdai-sim --bin bench -- [--cars N] [--laps L] [--track default|oval|proc:SEED|FILE.csv]
//!     [--scenario race|crash|debris] [--seed N]

use rdai_sim::{default_field, tracks, World};

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let get = |k: &str| args.iter().position(|a| a == k).and_then(|i| args.get(i + 1)).cloned();
    let cars: usize = get("--cars").and_then(|v| v.parse().ok()).unwrap_or(8);
    let laps: i32 = get("--laps").and_then(|v| v.parse().ok()).unwrap_or(3);
    let scenario = get("--scenario").unwrap_or_else(|| "race".into());
    let track = match get("--track").as_deref() {
        None | Some("default") => tracks::default_circuit(),
        Some("oval") => tracks::test_oval(),
        Some(p) if p.starts_with("proc:") => tracks::procedural(p[5..].parse().expect("proc:SEED")),
        Some(path) => tracks::parse_csv(&std::fs::read_to_string(path).expect("read track")).expect("parse track"),
    };

    let t_build = std::time::Instant::now();
    let seed: u64 = get("--seed").and_then(|v| v.parse().ok()).unwrap_or(0x5EED);
    let mut w = World::new_seeded(&track, &default_field(cars), laps, seed);
    println!(
        "track {:.0} m, {} samples; racing line + speed profile built in {:.1} ms; ideal lap {:.2} s",
        w.track.length(),
        w.track.len(),
        t_build.elapsed().as_secs_f64() * 1e3,
        w.line.lap_time
    );

    // --dump-line FILE: CSV of the racing line (s, offset, curvature, speed) per centreline sample.
    if let Some(path) = get("--dump-line") {
        use std::io::Write;
        let mut f = std::io::BufWriter::new(std::fs::File::create(path).expect("line file"));
        for i in 0..w.track.len() {
            let _ = writeln!(
                f,
                "{:.1},{:.3},{:.5},{:.2}",
                i as f32 * w.track.spacing(),
                w.line.offset[i],
                w.line.curvature[i],
                w.line.speed[i]
            );
        }
    }

    let dt = 1.0 / 120.0;
    let mut injected = false;
    let mut contacts_after = 0usize;
    // --trace FILE: CSV of car 0 every 0.1 s (time, s, d, line offset, speed, target, mode, steer, throttle, brake).
    let mut trace = get("--trace").map(|p| std::io::BufWriter::new(std::fs::File::create(p).expect("trace file")));
    let mut steps = 0u64;
    while !w.finished() && w.time < 60.0 * 30.0 {
        w.step(dt);
        steps += 1;
        if let (Some(f), true) = (trace.as_mut(), steps.is_multiple_of(12)) {
            use std::io::Write;
            let c = &w.cars[0];
            let pr = w.track.project(c.state.position, None);
            let plan = c.agent.last_plan().copied();
            let _ = writeln!(
                f,
                "{:.2},{:.1},{:.2},{:.2},{:.2},{:.2},{},{:.3},{:.2},{:.2},{},{:.2},{:.2},{:.2}",
                w.time,
                pr.s,
                pr.d,
                w.line.offset_at(&w.track, pr.s),
                c.state.vx,
                plan.map(|p| p.target_speed).unwrap_or(0.0),
                c.controls.mode,
                c.controls.steer,
                c.controls.throttle,
                c.controls.brake,
                c.wall_hits,
                c.agent.line().map(|l| l.offset_at(&w.track, pr.s)).unwrap_or(0.0),
                c.agent.line().map(|l| l.offset_at(&w.track, pr.s + 60.0)).unwrap_or(0.0),
                w.line.offset_at(&w.track, pr.s + 60.0),
            );
        }
        if !injected && w.time > 25.0 && scenario != "race" {
            let leader = w.positions()[0];
            match scenario.as_str() {
                "crash" => w.crash_car(leader, 6.0),
                "debris" => {
                    let d = w.line.offset_at(&w.track, w.cars[leader].s + 120.0);
                    w.drop_debris(leader, 120.0, d, 1.2);
                }
                _ => {}
            }
            injected = true;
            println!("t={:.1}s: scenario '{scenario}' injected at car {}", w.time, w.cars[leader].name);
        }
        if injected {
            contacts_after += w.new_contacts.len();
        }
    }

    println!("\n pos  driver   laps  best lap  finish    walls contacts");
    for (p, &i) in w.positions().iter().enumerate() {
        let c = &w.cars[i];
        let best = c.lap_times.iter().cloned().fold(f32::INFINITY, f32::min);
        println!(
            " {:>3}  {:<8} {:>4}  {:>8.2}  {:>7}   {:>5} {:>8}",
            p + 1,
            c.name,
            c.lap_times.len(),
            best,
            c.finished_at.map(|t| format!("{t:.2}")).unwrap_or("DNF".into()),
            c.wall_hits,
            c.contacts
        );
    }
    println!(
        "\nposition changes: {}; contacts after scenario: {}; sim time {:.1} s",
        w.overtakes, contacts_after, w.time
    );
    println!(
        "AI cost: {:.2} us per agent tick ({} ticks)",
        w.agent_ns as f64 / w.agent_calls.max(1) as f64 / 1e3,
        w.agent_calls
    );
}
