//! 2D viewer for the reference simulation.
//!
//! Keys: Space pause · Up/Down sim speed · Tab next car · F follow camera · P show plans
//!       K crash the selected car · O drop debris ahead of it · A/Z aggression +/- · R restart
//!       N new procedural circuit (B back to the default one)
//! Mouse: wheel zoom, right-drag pan.

use macroquad::prelude::*;
use rdai_core::Vec2 as V;
use rdai_sim::{default_field, tracks, World};

/// `track_seed`: `None` = the default circuit, `Some(seed)` = `tracks::procedural(seed)`.
fn new_world(track_seed: Option<u64>) -> World {
    let pts = track_seed.map(tracks::procedural).unwrap_or_else(tracks::default_circuit);
    let mut w = World::new(&pts, &default_field(8), 5);
    for c in &mut w.cars {
        c.agent.set_debug(true);
    }
    w
}

fn window_conf() -> Conf {
    Conf {
        window_title: "reactive-driving-ai".into(),
        window_width: 1280,
        window_height: 800,
        high_dpi: true,
        ..Default::default()
    }
}

#[macroquad::main(window_conf)]
async fn main() {
    let mut track_seed: Option<u64> = None;
    let mut w = new_world(track_seed);
    let mut sim_speed = 1.0f32;
    let mut paused = false;
    let mut selected = 0usize;
    let mut follow = true;
    let mut show_plans = true;
    let mut zoom = 3.0f32; // pixels per metre
    let mut center = V::new(300.0, 380.0);
    let mut acc = 0.0f32;
    let dt = 1.0 / 120.0;
    let mut last_mouse = mouse_position();
    let mut path = Vec::new();

    loop {
        // Input.
        if is_key_pressed(KeyCode::Space) {
            paused = !paused;
        }
        if is_key_pressed(KeyCode::Up) {
            sim_speed = (sim_speed * 2.0).min(16.0);
        }
        if is_key_pressed(KeyCode::Down) {
            sim_speed = (sim_speed * 0.5).max(0.125);
        }
        if is_key_pressed(KeyCode::Tab) {
            selected = (selected + 1) % w.cars.len();
        }
        if is_key_pressed(KeyCode::F) {
            follow = !follow;
        }
        if is_key_pressed(KeyCode::P) {
            show_plans = !show_plans;
        }
        if is_key_pressed(KeyCode::K) {
            w.crash_car(selected, 6.0);
        }
        if is_key_pressed(KeyCode::O) {
            let s = w.cars[selected].s + 80.0;
            let d = w.line.offset_at(&w.track, s);
            w.drop_debris(selected, 80.0, d, 1.2);
        }
        if is_key_pressed(KeyCode::R) {
            w = new_world(track_seed);
        }
        if is_key_pressed(KeyCode::N) {
            track_seed = Some(track_seed.map_or(1, |s| s + 1));
            w = new_world(track_seed);
        }
        if is_key_pressed(KeyCode::B) {
            track_seed = None;
            w = new_world(track_seed);
        }
        let da = if is_key_pressed(KeyCode::A) {
            0.1
        } else if is_key_pressed(KeyCode::Z) {
            -0.1
        } else {
            0.0
        };
        if da != 0.0 {
            let d = &mut w.cars[selected].agent.driver;
            d.profile.aggression = (d.profile.aggression + da).clamp(0.0, 1.0);
            d.base_aggression = d.profile.aggression;
        }
        let (_, wheel) = mouse_wheel();
        if wheel != 0.0 {
            zoom = (zoom * if wheel > 0.0 { 1.15 } else { 1.0 / 1.15 }).clamp(0.3, 40.0);
        }
        let m = mouse_position();
        if is_mouse_button_down(MouseButton::Right) {
            follow = false;
            center.x -= (m.0 - last_mouse.0) / zoom;
            center.y += (m.1 - last_mouse.1) / zoom;
        }
        last_mouse = m;

        // Simulation at a fixed step.
        if !paused {
            acc += get_frame_time().min(0.1) * sim_speed;
            while acc >= dt {
                w.step(dt);
                acc -= dt;
            }
        }
        if follow {
            center = w.cars[selected].state.position;
        }

        // Drawing (world Y up -> screen Y down).
        clear_background(Color::from_rgba(28, 92, 40, 255));
        let (sw, sh) = (screen_width(), screen_height());
        let to_s = |p: V| vec2(sw * 0.5 + (p.x - center.x) * zoom, sh * 0.5 - (p.y - center.y) * zoom);
        let n = w.track.len();
        for i in 0..n {
            let j = (i + 1) % n;
            let (a1, a2) = (
                w.track.to_world(i as f32 * w.track.spacing(), w.track.width_left(i)),
                w.track.to_world(i as f32 * w.track.spacing(), -w.track.width_right(i)),
            );
            let (b1, b2) = (
                w.track.to_world(j as f32 * w.track.spacing(), w.track.width_left(j)),
                w.track.to_world(j as f32 * w.track.spacing(), -w.track.width_right(j)),
            );
            let asphalt = Color::from_rgba(70, 70, 76, 255);
            draw_triangle(to_s(a1), to_s(a2), to_s(b1), asphalt);
            draw_triangle(to_s(a2), to_s(b2), to_s(b1), asphalt);
            draw_line(to_s(a1).x, to_s(a1).y, to_s(b1).x, to_s(b1).y, 2.0, WHITE);
            draw_line(to_s(a2).x, to_s(a2).y, to_s(b2).x, to_s(b2).y, 2.0, WHITE);
            // Racing line coloured by target speed (red slow -> green fast).
            let v = (w.line.speed[i] / 80.0).clamp(0.0, 1.0);
            let (p, q) = (to_s(w.line.points[i]), to_s(w.line.points[j]));
            draw_line(p.x, p.y, q.x, q.y, 1.5, Color::new(1.0 - v, v, 0.2, 0.6));
        }
        let start = to_s(w.track.to_world(0.0, 0.0));
        draw_circle(start.x, start.y, 4.0, YELLOW);
        for d in &w.debris {
            let p = to_s(d.position);
            draw_circle(p.x, p.y, d.radius * zoom, ORANGE);
        }

        let palette = [RED, BLUE, GOLD, MAGENTA, SKYBLUE, LIME, PINK, VIOLET, BROWN, BEIGE, DARKBLUE, PURPLE];
        for (i, c) in w.cars.iter().enumerate() {
            if show_plans && (i == selected || zoom > 2.0) {
                c.agent.plan_polyline(&w.track, &mut path);
                let col = if i == selected { Color::new(0.2, 1.0, 0.4, 0.9) } else { Color::new(1.0, 1.0, 1.0, 0.25) };
                for s in path.windows(2) {
                    let (a, b) = (to_s(s[0]), to_s(s[1]));
                    draw_line(a.x, a.y, b.x, b.y, 2.0, col);
                }
            }
            let p = to_s(c.state.position);
            let len = (c.params.length * zoom).max(6.0);
            let wid = (c.params.width * zoom).max(3.0);
            draw_rectangle_ex(
                p.x,
                p.y,
                len,
                wid,
                DrawRectangleParams {
                    offset: vec2(0.5, 0.5),
                    rotation: -c.state.heading,
                    color: palette[i % palette.len()],
                },
            );
            if i == selected {
                draw_circle_lines(p.x, p.y, len * 0.8, 2.0, WHITE);
            }
            if c.disabled > 0.0 {
                draw_text("CRASH", p.x + 8.0, p.y - 8.0, 18.0, ORANGE);
            }
        }

        // HUD.
        let modes = ["cruise", "follow", "avoid", "EMERGENCY"];
        draw_rectangle(8.0, 8.0, 330.0, 40.0 + 20.0 * w.cars.len() as f32 + 120.0, Color::new(0.0, 0.0, 0.0, 0.6));
        draw_text(
            format!("t {:.1}s  x{}  {}", w.time, sim_speed, if paused { "PAUSED" } else { "" }),
            16.0,
            30.0,
            22.0,
            WHITE,
        );
        for (k, &i) in w.positions().iter().enumerate() {
            let c = &w.cars[i];
            let txt = format!(
                "{:>2}. {:<7} L{} {:>5.0} km/h {}",
                k + 1,
                c.name,
                c.lap.max(0),
                c.state.vx * 3.6,
                modes[c.controls.mode as usize % 4]
            );
            draw_text(&txt, 16.0, 54.0 + 20.0 * k as f32, 20.0, if i == selected { YELLOW } else { WHITE });
        }
        let c = &w.cars[selected];
        let p = c.agent.driver.profile;
        let y0 = 64.0 + 20.0 * w.cars.len() as f32;
        draw_text(
            format!("{}  exp {:.2}  stamina {:.2}  aggr {:.2}", c.name, p.experience, p.stamina, p.aggression),
            16.0,
            y0,
            20.0,
            YELLOW,
        );
        draw_text(
            format!("walls {}  contacts {}  overtakes {}", c.wall_hits, c.contacts, w.overtakes),
            16.0,
            y0 + 20.0,
            20.0,
            WHITE,
        );
        draw_text(
            format!("AI {:.1} us/tick", w.agent_ns as f64 / w.agent_calls.max(1) as f64 / 1e3),
            16.0,
            y0 + 40.0,
            20.0,
            WHITE,
        );
        draw_text("Space pause  Up/Down speed  Tab car  F follow  P plans", 16.0, y0 + 64.0, 16.0, LIGHTGRAY);
        draw_text(
            "K crash  O debris  A/Z aggression  R restart  N new track  B default",
            16.0,
            y0 + 82.0,
            16.0,
            LIGHTGRAY,
        );

        next_frame().await
    }
}
