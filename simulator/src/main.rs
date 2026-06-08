use macroquad::prelude::*;
use std::collections::VecDeque;
use std::io::{BufRead, BufReader, Write};
use std::net::{TcpListener, TcpStream};
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::{Duration, Instant};

const WHEEL_SPAN_MM: f32 = 235.0;
const MOVE_SPEED_CM_S: f64 = 20.0;
const TURN_RATE_DEG_S: f64 = 60.0;
const CONTINUOUS_MAX: Duration = Duration::from_secs(3);
const TRAIL_LEN: usize = 300;
const TCP_PORT: u16 = 9999;
const SCALE: f32 = 0.1; // 1 px = 10 mm → 800px window = 8 m arena
const ROOMBA_RADIUS_PX: f32 = 17.0; // 340 mm diameter at SCALE
const WIN: f32 = 800.0;

fn window_conf() -> Conf {
    Conf {
        window_title: "Roomba Simulator".to_owned(),
        window_width: WIN as i32,
        window_height: WIN as i32,
        ..Default::default()
    }
}

struct RobotState {
    x: f32,       // mm, +x right
    y: f32,       // mm, +y up (math coords)
    heading: f32, // radians, 0 = right, π/2 = up
    vel: f32,     // mm/s forward
    angular: f32, // rad/s, positive = CCW
    deadline: Option<Instant>,
    trail: VecDeque<(f32, f32)>,
}

impl RobotState {
    fn new() -> Self {
        Self {
            x: 0.0,
            y: 0.0,
            heading: 0.0,
            vel: 0.0,
            angular: 0.0,
            deadline: None,
            trail: VecDeque::new(),
        }
    }

    fn arm(&mut self, dur: Duration) {
        self.deadline = Some(Instant::now() + dur);
    }

    fn stop(&mut self) {
        self.vel = 0.0;
        self.angular = 0.0;
        self.deadline = None;
    }

    fn update(&mut self, dt: f32) {
        if let Some(d) = self.deadline {
            if Instant::now() >= d {
                self.stop();
            }
        }
        if self.vel.abs() > 0.5 || self.angular.abs() > 0.01 {
            self.trail.push_back((self.x, self.y));
            if self.trail.len() > TRAIL_LEN {
                self.trail.pop_front();
            }
        }
        self.x += self.vel * self.heading.cos() * dt;
        self.y += self.vel * self.heading.sin() * dt;
        self.heading += self.angular * dt;
    }
}

fn dispatch(line: &str, state: &Arc<Mutex<RobotState>>) -> String {
    let mut parts = line.split_whitespace();
    let cmd = match parts.next() {
        Some(c) => c,
        None => return "ERR empty".into(),
    };
    let rest: Vec<&str> = parts.collect();

    macro_rules! num {
        ($s:expr, $t:ty) => {
            match $s.parse::<$t>() {
                Ok(v) => v,
                Err(_) => return format!("ERR bad number '{}'", $s),
            }
        };
    }

    match cmd {
        "ping" => "OK pong".into(),
        "safe" => "OK safe".into(),
        "full" => "OK full".into(),

        "stop" => {
            state.lock().unwrap().stop();
            "OK stop".into()
        }

        "go" => {
            if rest.len() != 2 {
                return "ERR usage: go <cm_per_sec> <deg_per_sec>".into();
            }
            let cm: f64 = num!(rest[0], f64);
            let deg: f64 = num!(rest[1], f64);
            let mut s = state.lock().unwrap();
            s.vel = (cm * 10.0) as f32;
            s.angular = deg.to_radians() as f32;
            s.arm(CONTINUOUS_MAX);
            format!("OK go {cm} {deg}")
        }

        // pilot calls drive_direct(right, left) with rest[0]=left, rest[1]=right
        "drive" => {
            if rest.len() != 2 {
                return "ERR usage: drive <left_mm_s> <right_mm_s>".into();
            }
            let left: i16 = num!(rest[0], i16);
            let right: i16 = num!(rest[1], i16);
            let mut s = state.lock().unwrap();
            s.vel = (right as f32 + left as f32) / 2.0;
            s.angular = (right as f32 - left as f32) / WHEEL_SPAN_MM;
            s.arm(CONTINUOUS_MAX);
            format!("OK drive L{left} R{right}")
        }

        "forward" | "back" => {
            if rest.len() != 2 {
                return format!("ERR usage: {cmd} <cm_per_sec> <seconds>");
            }
            let mut speed: f64 = num!(rest[0], f64).abs();
            if cmd == "back" {
                speed = -speed;
            }
            let secs: f64 = num!(rest[1], f64);
            let mut s = state.lock().unwrap();
            s.vel = (speed * 10.0) as f32;
            s.angular = 0.0;
            s.arm(Duration::from_secs_f64(secs.max(0.0)));
            format!("OK {cmd} {speed}cm/s {secs}s")
        }

        "spin" => {
            if rest.len() != 2 {
                return "ERR usage: spin <deg_per_sec> <seconds>".into();
            }
            let rate: f64 = num!(rest[0], f64);
            let secs: f64 = num!(rest[1], f64);
            let mut s = state.lock().unwrap();
            s.vel = 0.0;
            s.angular = rate.to_radians() as f32;
            s.arm(Duration::from_secs_f64(secs.max(0.0)));
            format!("OK spin {rate}deg/s {secs}s")
        }

        "move" => {
            if rest.len() != 1 {
                return "ERR usage: move <cm>".into();
            }
            let dist: f64 = num!(rest[0], f64);
            let speed = if dist < 0.0 {
                -MOVE_SPEED_CM_S
            } else {
                MOVE_SPEED_CM_S
            };
            let secs = (dist / speed).abs();
            let mut s = state.lock().unwrap();
            s.vel = (speed * 10.0) as f32;
            s.angular = 0.0;
            s.arm(Duration::from_secs_f64(secs));
            format!("OK move {dist}cm (~{secs:.1}s)")
        }

        "turn" => {
            if rest.len() != 1 {
                return "ERR usage: turn <deg>".into();
            }
            let deg: f64 = num!(rest[0], f64);
            let rate = if deg < 0.0 {
                -TURN_RATE_DEG_S
            } else {
                TURN_RATE_DEG_S
            };
            let secs = (deg / rate).abs();
            let mut s = state.lock().unwrap();
            s.vel = 0.0;
            s.angular = rate.to_radians() as f32;
            s.arm(Duration::from_secs_f64(secs));
            format!("OK turn {deg}deg (~{secs:.1}s)")
        }

        "motors" => {
            if rest.len() != 3 {
                return "ERR usage: motors <side -1|0|1> <main -1|0|1> <vac 0|1>".into();
            }
            format!("OK motors side{} main{} vac{}", rest[0], rest[1], rest[2])
        }

        "led" => {
            if rest.len() != 4 {
                return "ERR usage: led <color 0-255> <intensity 0-255> <play 0|1> <advance 0|1>"
                    .into();
            }
            "OK led".into()
        }

        "dock" => {
            state.lock().unwrap().stop();
            "OK dock".into()
        }

        "sense" => {
            "OK mode=2 charging=0 volt=16400mV current=-400mA temp=25C charge=2600/2600mAh (100%) bumpL=0 bumpR=0".into()
        }

        "bumps" => "OK bumpL=0 bumpR=0 dropL=0 dropR=0 caster=0".into(),

        "shutdown" | "quit" => {
            state.lock().unwrap().stop();
            "OK shutdown".into()
        }

        other => format!("ERR unknown command '{other}'"),
    }
}

fn handle_conn(stream: TcpStream, state: Arc<Mutex<RobotState>>) {
    let mut writer = match stream.try_clone() {
        Ok(w) => w,
        Err(_) => return,
    };
    let reader = BufReader::new(stream);
    for line in reader.lines() {
        let line = match line {
            Ok(l) => l,
            Err(_) => break,
        };
        let resp = dispatch(line.trim(), &state);
        let shutdown = resp.starts_with("OK shutdown");
        if writer.write_all(resp.as_bytes()).is_err() || writer.write_all(b"\n").is_err() {
            break;
        }
        let _ = writer.flush();
        if shutdown {
            std::process::exit(0);
        }
    }
}

fn start_tcp_server(state: Arc<Mutex<RobotState>>) {
    let listener =
        TcpListener::bind(("127.0.0.1", TCP_PORT)).expect("failed to bind TCP port 9999");
    println!("simulator: listening on 127.0.0.1:{TCP_PORT}");
    thread::spawn(move || {
        for stream in listener.incoming() {
            match stream {
                Ok(s) => {
                    let state = Arc::clone(&state);
                    thread::spawn(move || handle_conn(s, state));
                }
                Err(e) => eprintln!("simulator: accept error: {e}"),
            }
        }
    });
}

fn to_screen(x: f32, y: f32) -> (f32, f32) {
    (WIN / 2.0 + x * SCALE, WIN / 2.0 - y * SCALE)
}

#[macroquad::main(window_conf)]
async fn main() {
    let state = Arc::new(Mutex::new(RobotState::new()));
    start_tcp_server(Arc::clone(&state));

    loop {
        let dt = get_frame_time().min(0.05);

        {
            let mut s = state.lock().unwrap();
            s.update(dt);
        }

        clear_background(Color::from_rgba(245, 245, 235, 255));

        // Grid lines every 500 mm (50 cm)
        let grid_px = 500.0 * SCALE; // 50 px
        let cx = WIN / 2.0;
        let cy = WIN / 2.0;
        let grid_color = Color::from_rgba(210, 210, 200, 255);
        let mut gx = cx % grid_px;
        while gx <= WIN {
            draw_line(gx, 0.0, gx, WIN, 1.0, grid_color);
            gx += grid_px;
        }
        let mut gy = cy % grid_px;
        while gy <= WIN {
            draw_line(0.0, gy, WIN, gy, 1.0, grid_color);
            gy += grid_px;
        }

        // Origin crosshair
        draw_line(cx - 8.0, cy, cx + 8.0, cy, 1.5, Color::from_rgba(160, 160, 150, 255));
        draw_line(cx, cy - 8.0, cx, cy + 8.0, 1.5, Color::from_rgba(160, 160, 150, 255));

        // Snapshot state for rendering
        let (rx, ry, heading, vel, angular, trail_snap, moving) = {
            let s = state.lock().unwrap();
            let trail: Vec<(f32, f32)> = s.trail.iter().cloned().collect();
            (s.x, s.y, s.heading, s.vel, s.angular, trail, s.deadline.is_some())
        };

        // Trail
        for i in 1..trail_snap.len() {
            let alpha = (i as f32 / trail_snap.len() as f32 * 200.0) as u8;
            let (x0, y0) = to_screen(trail_snap[i - 1].0, trail_snap[i - 1].1);
            let (x1, y1) = to_screen(trail_snap[i].0, trail_snap[i].1);
            draw_line(x0, y0, x1, y1, 2.0, Color::from_rgba(80, 100, 210, alpha));
        }

        // Roomba body — brighter blue when actively moving
        let (sx, sy) = to_screen(rx, ry);
        let body_color = if moving {
            Color::from_rgba(60, 120, 230, 240)
        } else {
            Color::from_rgba(80, 80, 160, 200)
        };
        draw_circle(sx, sy, ROOMBA_RADIUS_PX, body_color);
        draw_circle_lines(sx, sy, ROOMBA_RADIUS_PX, 2.0, Color::from_rgba(30, 30, 100, 255));

        // Heading arrow (screen y is flipped relative to math y)
        let hx = heading.cos();
        let hy = -heading.sin();
        let tip_x = sx + hx * ROOMBA_RADIUS_PX * 1.3;
        let tip_y = sy + hy * ROOMBA_RADIUS_PX * 1.3;
        draw_line(sx, sy, tip_x, tip_y, 3.0, Color::from_rgba(255, 220, 50, 255));

        // Status overlay
        let x_cm = rx / 10.0;
        let y_cm = ry / 10.0;
        let hdg_deg = heading.to_degrees().rem_euclid(360.0);
        let vel_cm = vel / 10.0;
        let ang_dps = angular.to_degrees();
        draw_text(
            &format!(
                "x={x_cm:.1}cm  y={y_cm:.1}cm  hdg={hdg_deg:.0}°  vel={vel_cm:.0}cm/s  ω={ang_dps:.0}°/s"
            ),
            10.0,
            22.0,
            20.0,
            BLACK,
        );
        draw_text(
            "TCP :9999  —  move / turn / go / drive / stop / forward / back / spin",
            10.0,
            42.0,
            15.0,
            Color::from_rgba(100, 100, 100, 200),
        );

        next_frame().await;
    }
}
