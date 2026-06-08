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
const TRAIL_LEN: usize = 400;
const TCP_PORT: u16 = 9999;

// --- Physical Dimensions ---
const ROOMBA_RADIUS_MM: f32 = 170.0; // 34 cm diameter
const PDF_TO_MM: f32 = 36.0;         // 1 PDF unit = 36 mm

// --- Screen Layout ---
const WIN_W: f32 = 1200.0;
const WIN_H: f32 = 850.0;
const MAP_WIDTH_MM: f32 = 1600.0 * PDF_TO_MM;  // 57600 mm
const _MAP_HEIGHT_MM: f32 = 750.0 * PDF_TO_MM;  // 27000 mm

fn window_conf() -> Conf {
    Conf {
        window_title: "Roomba 2D Turing Office Simulator".to_owned(),
        window_width: WIN_W as i32,
        window_height: WIN_H as i32,
        ..Default::default()
    }
}

// Convert PDF coordinates to millimeters
fn pdf_pt(x: f32, y: f32) -> Vec2 {
    Vec2::new(x * PDF_TO_MM, y * PDF_TO_MM)
}

// Map millimeter coordinates to screen pixels
fn to_screen(x: f32, y: f32) -> (f32, f32) {
    let scale = 1120.0 / MAP_WIDTH_MM;
    let offset_x = 40.0;
    let offset_y = 162.5; // (850.0 - MAP_HEIGHT_MM * scale) / 2.0
    (
        offset_x + x * scale,
        offset_y + (1000.0 * PDF_TO_MM - y) * scale,
    )
}

#[derive(Clone, Copy, Debug)]
struct Segment {
    p1: Vec2,
    p2: Vec2,
}

#[derive(Clone, Debug)]
struct Door {
    name: String,
    p1: Vec2,
    p2: Vec2,
    is_open: bool,
}

#[derive(Clone, Copy, Debug)]
struct Obstacle {
    center: Vec2,
    radius: f32,
}

struct RoomLabel {
    name: &'static str,
    pos: Vec2,
}

struct RobotState {
    x: f32,       // mm, +x right
    y: f32,       // mm, +y up (math coords)
    heading: f32, // radians, 0 = right, π/2 = up
    vel: f32,     // mm/s forward
    angular: f32, // rad/s, positive = CCW
    deadline: Option<Instant>,
    trail: VecDeque<(f32, f32)>,
    bump_left: bool,
    bump_right: bool,
    lidar_distances: [f32; 8], // mm
}

impl RobotState {
    fn new() -> Self {
        // Starts in the middle of Enigma room: X_pdf = 1400, Y_pdf = 900
        Self {
            x: 1400.0 * PDF_TO_MM,
            y: 900.0 * PDF_TO_MM,
            heading: 0.0,
            vel: 0.0,
            angular: 0.0,
            deadline: None,
            trail: VecDeque::new(),
            bump_left: false,
            bump_right: false,
            lidar_distances: [0.0; 8],
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

    fn update(&mut self, dt: f32, walls: &[Segment], doors: &[Door], obstacles: &[Obstacle]) {
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

        // Calculate candidate positions
        let mut new_x = self.x + self.vel * self.heading.cos() * dt;
        let mut new_y = self.y + self.vel * self.heading.sin() * dt;
        self.heading += self.angular * dt;

        let mut bump_l = false;
        let mut bump_r = false;

        // Perform multiple collision resolution passes (sliding along multiple walls)
        for _ in 0..4 {
            let mut collision_resolved = false;
            let current_pos = Vec2::new(new_x, new_y);

            // 1. Static walls
            for wall in walls {
                if let Some((normal, closest_pt)) = resolve_segment_collision(current_pos, ROOMBA_RADIUS_MM, wall.p1, wall.p2) {
                    let depth = ROOMBA_RADIUS_MM - (current_pos - closest_pt).length();
                    new_x += normal.x * depth;
                    new_y += normal.y * depth;
                    collision_resolved = true;

                    let (bl, br) = calculate_bumps(self.heading, normal);
                    bump_l |= bl;
                    bump_r |= br;
                }
            }

            // 2. Closed doors
            for door in doors {
                if !door.is_open {
                    if let Some((normal, closest_pt)) = resolve_segment_collision(current_pos, ROOMBA_RADIUS_MM, door.p1, door.p2) {
                        let depth = ROOMBA_RADIUS_MM - (current_pos - closest_pt).length();
                        new_x += normal.x * depth;
                        new_y += normal.y * depth;
                        collision_resolved = true;

                        let (bl, br) = calculate_bumps(self.heading, normal);
                        bump_l |= bl;
                        bump_r |= br;
                    }
                }
            }

            // 3. Obstacles
            for obs in obstacles {
                let to_roomba = current_pos - obs.center;
                let dist_sq = to_roomba.length_squared();
                let min_dist = ROOMBA_RADIUS_MM + obs.radius;
                if dist_sq < min_dist * min_dist {
                    let dist = dist_sq.sqrt();
                    let normal = if dist > 1e-4 { to_roomba / dist } else { Vec2::new(1.0, 0.0) };
                    let depth = min_dist - dist;
                    new_x += normal.x * depth;
                    new_y += normal.y * depth;
                    collision_resolved = true;

                    let (bl, br) = calculate_bumps(self.heading, normal);
                    bump_l |= bl;
                    bump_r |= br;
                }
            }

            if !collision_resolved {
                break;
            }
        }

        self.x = new_x;
        self.y = new_y;
        self.bump_left = bump_l;
        self.bump_right = bump_r;
    }
}

// Collide a circle against a line segment
fn resolve_segment_collision(center: Vec2, radius: f32, p1: Vec2, p2: Vec2) -> Option<(Vec2, Vec2)> {
    let segment = p2 - p1;
    let to_center = center - p1;
    let seg_len_sq = segment.length_squared();
    if seg_len_sq < 1e-6 {
        return None;
    }
    
    let t = (to_center.dot(segment) / seg_len_sq).clamp(0.0, 1.0);
    let closest_point = p1 + t * segment;
    let dist_vector = center - closest_point;
    let dist_sq = dist_vector.length_squared();
    
    if dist_sq < radius * radius {
        let dist = dist_sq.sqrt();
        let normal = if dist > 1e-4 {
            dist_vector / dist
        } else {
            // Perpendicular vector fallback
            Vec2::new(-segment.y, segment.x).normalize()
        };
        Some((normal, closest_point))
    } else {
        None
    }
}

// Decide bump sensor flags based on collision normal and robot heading
fn calculate_bumps(heading: f32, normal: Vec2) -> (bool, bool) {
    let h_vec = Vec2::new(heading.cos(), heading.sin());
    let impact_dir = -normal; // points into the Roomba
    let dot = h_vec.dot(impact_dir);
    let det = h_vec.x * impact_dir.y - h_vec.y * impact_dir.x;
    let angle = det.atan2(dot); // range [-PI, PI]

    let mut bump_left = false;
    let mut bump_right = false;
    if angle.abs() <= std::f32::consts::FRAC_PI_2 {
        if angle.abs() <= 0.2 { // direct front collision triggers both
            bump_left = true;
            bump_right = true;
        } else if angle > 0.0 {
            bump_left = true;
        } else {
            bump_right = true;
        }
    }
    (bump_left, bump_right)
}

// --- Ray Casting for simulated Lidar ---
fn cast_ray(
    origin: Vec2,
    dir: Vec2,
    walls: &[Segment],
    doors: &[Door],
    obstacles: &[Obstacle],
) -> f32 {
    let mut min_t = 100000.0; // 100 meters default max range

    for wall in walls {
        if let Some(t) = ray_intersect_segment(origin, dir, wall.p1, wall.p2) {
            if t < min_t {
                min_t = t;
            }
        }
    }

    for door in doors {
        if !door.is_open {
            if let Some(t) = ray_intersect_segment(origin, dir, door.p1, door.p2) {
                if t < min_t {
                    min_t = t;
                }
            }
        }
    }

    for obs in obstacles {
        if let Some(t) = ray_intersect_circle(origin, dir, obs.center, obs.radius) {
            if t < min_t {
                min_t = t;
            }
        }
    }

    min_t
}

fn ray_intersect_segment(origin: Vec2, dir: Vec2, p1: Vec2, p2: Vec2) -> Option<f32> {
    let v = p2 - p1;
    let denom = dir.x * v.y - dir.y * v.x;
    if denom.abs() < 1e-6 {
        return None;
    }
    let t = ((p1.x - origin.x) * v.y - (p1.y - origin.y) * v.x) / denom;
    let u = ((p1.x - origin.x) * dir.y - (p1.y - origin.y) * dir.x) / denom;
    if t >= 0.0 && (0.0..=1.0).contains(&u) {
        Some(t)
    } else {
        None
    }
}

fn ray_intersect_circle(origin: Vec2, dir: Vec2, center: Vec2, radius: f32) -> Option<f32> {
    let f = origin - center;
    let b = 2.0 * f.dot(dir);
    let c = f.length_squared() - radius * radius;
    let disc = b * b - 4.0 * c;
    if disc < 0.0 {
        None
    } else {
        let disc_sqrt = disc.sqrt();
        let t1 = (-b - disc_sqrt) / 2.0;
        let t2 = (-b + disc_sqrt) / 2.0;
        if t1 >= 0.0 {
            Some(t1)
        } else if t2 >= 0.0 {
            Some(t2)
        } else {
            None
        }
    }
}

// Generate random obstacles avoiding start area and doors
fn generate_obstacles(roomba_start: Vec2) -> Vec<Obstacle> {
    let mut obstacles: Vec<Obstacle> = Vec::new();
    let spawn_zones = [
        // Top rooms (except Enigma)
        (100.0, 1150.0, 820.0, 980.0),
        // Corridor
        (50.0, 1550.0, 730.0, 770.0),
        // Lower rooms
        (50.0, 1550.0, 300.0, 670.0),
    ];
    
    // Core door locations to keep clear (in PDF units)
    let doors = [
        Vec2::new(32.5, 800.0),
        Vec2::new(282.5, 800.0),
        Vec2::new(582.5, 800.0),
        Vec2::new(782.5, 800.0),
        Vec2::new(1032.5, 800.0),
        Vec2::new(1232.5, 800.0),
        Vec2::new(225.0, 700.0),
        Vec2::new(725.0, 700.0),
        Vec2::new(1375.0, 700.0),
    ];
    
    let mut attempts = 0;
    while obstacles.len() < 18 && attempts < 400 {
        attempts += 1;
        let zone_idx = macroquad::rand::rand() as usize % spawn_zones.len();
        let (x_min, x_max, y_min, y_max) = spawn_zones[zone_idx];
        
        let rx = x_min + macroquad::rand::gen_range(0.0, x_max - x_min);
        let ry = y_min + macroquad::rand::gen_range(0.0, y_max - y_min);
        let pos_mm = Vec2::new(rx * PDF_TO_MM, ry * PDF_TO_MM);
        
        // Don't spawn on top of starting position
        if (pos_mm - roomba_start).length() < 1600.0 {
            continue;
        }
        
        // Don't spawn blocking doorways
        let mut near_door = false;
        for &door_pos in &doors {
            let door_pos_mm = door_pos * PDF_TO_MM;
            if (pos_mm - door_pos_mm).length() < 1200.0 {
                near_door = true;
                break;
            }
        }
        if near_door {
            continue;
        }
        
        // Don't overlap too close to other obstacles
        let mut overlap = false;
        for obs in &obstacles {
            if (pos_mm - obs.center).length() < (obs.radius + 300.0) {
                overlap = true;
                break;
            }
        }
        if overlap {
            continue;
        }
        
        let radius = macroquad::rand::gen_range(160.0, 360.0); // 16 to 36 cm radius
        obstacles.push(Obstacle { center: pos_mm, radius });
    }
    obstacles
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
            let (b_l, b_r) = {
                let s = state.lock().unwrap();
                (s.bump_left as u8, s.bump_right as u8)
            };
            format!(
                "OK mode=2 charging=0 volt=16400mV current=-400mA temp=25C charge=2600/2600mAh (100%) bumpL={} bumpR={}",
                b_l, b_r
            )
        }

        "bumps" => {
            let (b_l, b_r) = {
                let s = state.lock().unwrap();
                (s.bump_left as u8, s.bump_right as u8)
            };
            format!("OK bumpL={} bumpR={} dropL=0 dropR=0 caster=0", b_l, b_r)
        }

        "lidar" => {
            let s = state.lock().unwrap();
            // Lidar distances returned in cm
            format!(
                "OK f={:.1} fl={:.1} l={:.1} bl={:.1} b={:.1} br={:.1} r={:.1} fr={:.1}",
                s.lidar_distances[0] / 10.0,
                s.lidar_distances[1] / 10.0,
                s.lidar_distances[2] / 10.0,
                s.lidar_distances[3] / 10.0,
                s.lidar_distances[4] / 10.0,
                s.lidar_distances[5] / 10.0,
                s.lidar_distances[6] / 10.0,
                s.lidar_distances[7] / 10.0,
            )
        }

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

#[macroquad::main(window_conf)]
async fn main() {
    // Initialize walls (static segments)
    let mut walls = Vec::new();
    
    // 1. Outer perimeter
    walls.push(Segment { p1: pdf_pt(0.0, 250.0), p2: pdf_pt(1600.0, 250.0) });
    walls.push(Segment { p1: pdf_pt(1600.0, 250.0), p2: pdf_pt(1600.0, 1000.0) });
    walls.push(Segment { p1: pdf_pt(1600.0, 1000.0), p2: pdf_pt(0.0, 1000.0) });
    walls.push(Segment { p1: pdf_pt(0.0, 1000.0), p2: pdf_pt(0.0, 250.0) });

    // 2. Top row rooms vertical dividers
    walls.push(Segment { p1: pdf_pt(250.0, 800.0), p2: pdf_pt(250.0, 1000.0) });
    walls.push(Segment { p1: pdf_pt(550.0, 800.0), p2: pdf_pt(550.0, 1000.0) });
    walls.push(Segment { p1: pdf_pt(750.0, 800.0), p2: pdf_pt(750.0, 1000.0) });
    walls.push(Segment { p1: pdf_pt(1000.0, 800.0), p2: pdf_pt(1000.0, 1000.0) });
    walls.push(Segment { p1: pdf_pt(1200.0, 800.0), p2: pdf_pt(1200.0, 1000.0) });

    // 3. Top corridor walls (Y = 800) with door frame segments
    // Roomba starts in Enigma (1200..1600). Door width 90cm = 25 units.
    // Jack Good (0..250). Door: 20..45
    walls.push(Segment { p1: pdf_pt(0.0, 800.0), p2: pdf_pt(20.0, 800.0) });
    walls.push(Segment { p1: pdf_pt(45.0, 800.0), p2: pdf_pt(250.0, 800.0) });
    // Rejewski (250..550). Door: 270..295
    walls.push(Segment { p1: pdf_pt(250.0, 800.0), p2: pdf_pt(270.0, 800.0) });
    walls.push(Segment { p1: pdf_pt(295.0, 800.0), p2: pdf_pt(550.0, 800.0) });
    // Ada (550..750). Door: 570..595
    walls.push(Segment { p1: pdf_pt(550.0, 800.0), p2: pdf_pt(570.0, 800.0) });
    walls.push(Segment { p1: pdf_pt(595.0, 800.0), p2: pdf_pt(750.0, 800.0) });
    // Lovelace (750..1000). Door: 770..795
    walls.push(Segment { p1: pdf_pt(750.0, 800.0), p2: pdf_pt(770.0, 800.0) });
    walls.push(Segment { p1: pdf_pt(795.0, 800.0), p2: pdf_pt(1000.0, 800.0) });
    // CEO Office (1000..1200). Door: 1020..1045
    walls.push(Segment { p1: pdf_pt(1000.0, 800.0), p2: pdf_pt(1020.0, 800.0) });
    walls.push(Segment { p1: pdf_pt(1045.0, 800.0), p2: pdf_pt(1200.0, 800.0) });
    // Enigma (1200..1600). Door: 1220..1245
    walls.push(Segment { p1: pdf_pt(1200.0, 800.0), p2: pdf_pt(1220.0, 800.0) });
    walls.push(Segment { p1: pdf_pt(1245.0, 800.0), p2: pdf_pt(1600.0, 800.0) });

    // 4. Middle hallway boundary at Y = 720
    // Mae Jemison door at X in [200, 245]
    walls.push(Segment { p1: pdf_pt(0.0, 720.0), p2: pdf_pt(200.0, 720.0) });
    walls.push(Segment { p1: pdf_pt(245.0, 720.0), p2: pdf_pt(600.0, 720.0) });
    // Project Space door at X in [700, 745]
    walls.push(Segment { p1: pdf_pt(600.0, 720.0), p2: pdf_pt(700.0, 720.0) });
    walls.push(Segment { p1: pdf_pt(745.0, 720.0), p2: pdf_pt(1300.0, 720.0) });
    // Kitchen door at X in [1350, 1395]
    walls.push(Segment { p1: pdf_pt(1300.0, 720.0), p2: pdf_pt(1350.0, 720.0) });
    walls.push(Segment { p1: pdf_pt(1395.0, 720.0), p2: pdf_pt(1600.0, 720.0) });

    // 5. Lower partitions
    walls.push(Segment { p1: pdf_pt(450.0, 250.0), p2: pdf_pt(450.0, 720.0) });
    walls.push(Segment { p1: pdf_pt(1200.0, 250.0), p2: pdf_pt(1200.0, 720.0) });
    walls.push(Segment { p1: pdf_pt(0.0, 480.0), p2: pdf_pt(450.0, 480.0) });
    walls.push(Segment { p1: pdf_pt(450.0, 480.0), p2: pdf_pt(1200.0, 480.0) });
    walls.push(Segment { p1: pdf_pt(1200.0, 480.0), p2: pdf_pt(1600.0, 480.0) });

    // Initialize doors
    let mut doors = vec![
        Door { name: "Jack Good".into(), p1: pdf_pt(20.0, 800.0), p2: pdf_pt(45.0, 800.0), is_open: false },
        Door { name: "Rejewski".into(), p1: pdf_pt(270.0, 800.0), p2: pdf_pt(295.0, 800.0), is_open: false },
        Door { name: "Ada".into(), p1: pdf_pt(570.0, 800.0), p2: pdf_pt(595.0, 800.0), is_open: false },
        Door { name: "Lovelace".into(), p1: pdf_pt(770.0, 800.0), p2: pdf_pt(795.0, 800.0), is_open: false },
        Door { name: "CEO Office".into(), p1: pdf_pt(1020.0, 800.0), p2: pdf_pt(1045.0, 800.0), is_open: false },
        Door { name: "Enigma".into(), p1: pdf_pt(1220.0, 800.0), p2: pdf_pt(1245.0, 800.0), is_open: false },
        
        Door { name: "Mae Jemison".into(), p1: pdf_pt(200.0, 720.0), p2: pdf_pt(245.0, 720.0), is_open: true },
        Door { name: "Project Space".into(), p1: pdf_pt(700.0, 720.0), p2: pdf_pt(745.0, 720.0), is_open: true },
        Door { name: "Kitchen".into(), p1: pdf_pt(1350.0, 720.0), p2: pdf_pt(1395.0, 720.0), is_open: true },
    ];

    // Room Label definitions
    let room_labels = vec![
        RoomLabel { name: "JACK GOOD", pos: pdf_pt(125.0, 920.0) },
        RoomLabel { name: "DAVID BLACKWELL", pos: pdf_pt(125.0, 970.0) },
        RoomLabel { name: "MARIAN REJEWSKI", pos: pdf_pt(400.0, 920.0) },
        RoomLabel { name: "JOAN CLARKE", pos: pdf_pt(400.0, 970.0) },
        RoomLabel { name: "ADA", pos: pdf_pt(650.0, 920.0) },
        RoomLabel { name: "LOVELACE", pos: pdf_pt(875.0, 920.0) },
        RoomLabel { name: "MARGARET HAMILTON", pos: pdf_pt(1100.0, 970.0) },
        RoomLabel { name: "CEO OFFICE", pos: pdf_pt(1100.0, 920.0) },
        RoomLabel { name: "ENIGMA 2.0", pos: pdf_pt(1400.0, 900.0) },
        
        RoomLabel { name: "CORRIDOR / HALLWAY", pos: pdf_pt(800.0, 750.0) },
        RoomLabel { name: "CIPHER", pos: pdf_pt(225.0, 600.0) },
        RoomLabel { name: "MAE JEMISON", pos: pdf_pt(225.0, 530.0) },
        RoomLabel { name: "FLORENCE NIGHTINGALE", pos: pdf_pt(225.0, 380.0) },
        RoomLabel { name: "PROJECT SPACE", pos: pdf_pt(825.0, 600.0) },
        RoomLabel { name: "TEA POINT", pos: pdf_pt(825.0, 530.0) },
        RoomLabel { name: "WELLBEING ROOM", pos: pdf_pt(825.0, 380.0) },
        RoomLabel { name: "MEDIA SUITE", pos: pdf_pt(900.0, 440.0) },
        RoomLabel { name: "MAIN KITCHEN", pos: pdf_pt(1400.0, 600.0) },
        RoomLabel { name: "RECEPTION DESK", pos: pdf_pt(1400.0, 530.0) },
        RoomLabel { name: "STAFF LIFT LOBBY", pos: pdf_pt(1400.0, 450.0) },
        RoomLabel { name: "URSULA FRANKLIN", pos: pdf_pt(1400.0, 380.0) },
    ];

    let roomba_start_pos = Vec2::new(1400.0 * PDF_TO_MM, 900.0 * PDF_TO_MM);
    let mut obstacles = generate_obstacles(roomba_start_pos);

    let state = Arc::new(Mutex::new(RobotState::new()));
    start_tcp_server(Arc::clone(&state));

    loop {
        let dt = get_frame_time().min(0.05);
        let total_time = get_time();

        // 1. Dynamic Doors Timer Cycle (18s: 12s closed, 6s open)
        let cycle = 18.0;
        let elapsed_in_cycle = total_time % cycle;
        let is_top_doors_open = elapsed_in_cycle >= 12.0;
        let doors_time_left = if is_top_doors_open {
            cycle - elapsed_in_cycle
        } else {
            12.0 - elapsed_in_cycle
        };

        for door in &mut doors {
            if door.name == "Enigma" || door.name == "CEO Office" || door.name == "Lovelace" || door.name == "Ada" || door.name == "Rejewski" || door.name == "Jack Good" {
                door.is_open = is_top_doors_open;
            }
        }

        // 2. Refresh Button and Obstacle Generation Interaction
        let mouse_pos = mouse_position();
        let btn_rect = Rect::new(40.0, 40.0, 180.0, 40.0);
        let btn_hover = btn_rect.contains(Vec2::new(mouse_pos.0, mouse_pos.1));
        let btn_clicked = btn_hover && is_mouse_button_pressed(MouseButton::Left);

        if btn_clicked || is_key_pressed(KeyCode::R) {
            let mut s = state.lock().unwrap();
            s.x = roomba_start_pos.x;
            s.y = roomba_start_pos.y;
            s.heading = 0.0;
            s.stop();
            obstacles = generate_obstacles(roomba_start_pos);
        }

        // 3. Physics & Sensor Update
        {
            let mut s = state.lock().unwrap();
            s.update(dt, &walls, &doors, &obstacles);

            // Cast Lidar rays (8 directions relative to heading)
            let origin = Vec2::new(s.x, s.y);
            let mut lidar_vals = [0.0; 8];
            let angles = [0.0, 45.0, 90.0, 135.0, 180.0, 225.0, 270.0, 315.0];
            for i in 0..8 {
                let angle_rad = s.heading + (angles[i] as f32).to_radians();
                let dir = Vec2::new(angle_rad.cos(), angle_rad.sin());
                lidar_vals[i] = cast_ray(origin, dir, &walls, &doors, &obstacles);
            }
            s.lidar_distances = lidar_vals;
        }

        // --- RENDER ---
        clear_background(Color::from_rgba(20, 22, 28, 255)); // sleek dark background

        // Draw Map Border Box
        let (x_min_s, y_max_s) = to_screen(0.0, 250.0 * PDF_TO_MM);
        let (x_max_s, y_min_s) = to_screen(1600.0 * PDF_TO_MM, 1000.0 * PDF_TO_MM);
        draw_rectangle(x_min_s, y_min_s, x_max_s - x_min_s, y_max_s - y_min_s, Color::from_rgba(30, 34, 45, 255));
        draw_rectangle_lines(x_min_s, y_min_s, x_max_s - x_min_s, y_max_s - y_min_s, 2.0, Color::from_rgba(65, 75, 95, 255));

        // Draw Map Grid Lines every 1.5 meters (1500 mm)
        let grid_gap = 1500.0;
        let scale = 1120.0 / MAP_WIDTH_MM;
        let grid_col = Color::from_rgba(45, 50, 65, 255);
        let mut gx = 0.0;
        while gx < MAP_WIDTH_MM {
            let (sx, _) = to_screen(gx, 0.0);
            if sx >= x_min_s && sx <= x_max_s {
                draw_line(sx, y_min_s, sx, y_max_s, 1.0, grid_col);
            }
            gx += grid_gap;
        }
        let mut gy = 250.0 * PDF_TO_MM;
        while gy < 1000.0 * PDF_TO_MM {
            let (_, sy) = to_screen(0.0, gy);
            if sy >= y_min_s && sy <= y_max_s {
                draw_line(x_min_s, sy, x_max_s, sy, 1.0, grid_col);
            }
            gy += grid_gap;
        }

        // Draw Room Zones & Labels
        for label in &room_labels {
            let (sx, sy) = to_screen(label.pos.x, label.pos.y);
            draw_text(label.name, sx - 20.0, sy, 13.0, Color::from_rgba(140, 150, 175, 200));
        }

        // Draw Static Walls
        for wall in &walls {
            let (sx1, sy1) = to_screen(wall.p1.x, wall.p1.y);
            let (sx2, sy2) = to_screen(wall.p2.x, wall.p2.y);
            draw_line(sx1, sy1, sx2, sy2, 2.0, Color::from_rgba(200, 205, 220, 255));
        }

        // Draw Dynamic Doors
        for door in &doors {
            let (sx1, sy1) = to_screen(door.p1.x, door.p1.y);
            let (sx2, sy2) = to_screen(door.p2.x, door.p2.y);
            if door.is_open {
                // Draw dashed green line for open
                let steps = 4;
                for i in 0..steps {
                    let t_start = i as f32 / steps as f32;
                    let t_end = (i as f32 + 0.5) / steps as f32;
                    let p_start = door.p1.lerp(door.p2, t_start);
                    let p_end = door.p1.lerp(door.p2, t_end);
                    let (dsx1, dsy1) = to_screen(p_start.x, p_start.y);
                    let (dsx2, dsy2) = to_screen(p_end.x, p_end.y);
                    draw_line(dsx1, dsy1, dsx2, dsy2, 2.5, Color::from_rgba(40, 220, 100, 120));
                }
            } else {
                // Draw solid red line for closed
                draw_line(sx1, sy1, sx2, sy2, 3.0, Color::from_rgba(255, 60, 80, 255));
            }
        }

        // Draw Obstacles (Orange Glow Circles)
        for obs in &obstacles {
            let (sx, sy) = to_screen(obs.center.x, obs.center.y);
            let s_rad = obs.radius * scale;
            draw_circle(sx, sy, s_rad, Color::from_rgba(255, 120, 40, 60)); // transparent body
            draw_circle_lines(sx, sy, s_rad, 1.5, Color::from_rgba(255, 120, 40, 220)); // glowing rim
        }

        // Retrieve robot state details for rendering
        let (rx, ry, heading, vel, angular, trail_snap, b_l, b_r, lidar_snap) = {
            let s = state.lock().unwrap();
            (
                s.x,
                s.y,
                s.heading,
                s.vel,
                s.angular,
                s.trail.iter().cloned().collect::<Vec<_>>(),
                s.bump_left,
                s.bump_right,
                s.lidar_distances,
            )
        };

        // Draw Roomba Trail
        for i in 1..trail_snap.len() {
            let alpha = (i as f32 / trail_snap.len() as f32 * 120.0) as u8;
            let (x0, y0) = to_screen(trail_snap[i - 1].0, trail_snap[i - 1].1);
            let (x1, y1) = to_screen(trail_snap[i].0, trail_snap[i].1);
            draw_line(x0, y0, x1, y1, 2.0, Color::from_rgba(0, 229, 255, alpha));
        }

        // Draw Lidar Active Ray Visualization
        let angles = [0.0, 45.0, 90.0, 135.0, 180.0, 225.0, 270.0, 315.0];
        for i in 0..8 {
            let dist_mm = lidar_snap[i];
            let angle_rad = heading + (angles[i] as f32).to_radians();
            let ray_end = Vec2::new(rx, ry) + Vec2::new(angle_rad.cos(), angle_rad.sin()) * dist_mm;
            let (sx0, sy0) = to_screen(rx, ry);
            let (sx1, sy1) = to_screen(ray_end.x, ray_end.y);
            draw_line(sx0, sy0, sx1, sy1, 1.0, Color::from_rgba(255, 235, 60, 45)); // thin yellow ray
            draw_circle(sx1, sy1, 2.0, Color::from_rgba(255, 235, 60, 180)); // ray hit point
        }

        // Draw Roomba Body (Turquoise neon)
        let (sx, sy) = to_screen(rx, ry);
        let visual_r = 10.0; // Draw slightly larger for visibility
        draw_circle(sx, sy, visual_r, Color::from_rgba(0, 210, 255, 200));
        draw_circle_lines(sx, sy, visual_r, 2.0, Color::from_rgba(0, 255, 255, 255));

        // Draw Bumps Glow Indicator
        if b_l {
            draw_circle_lines(sx, sy, visual_r + 3.0, 1.5, Color::from_rgba(255, 60, 80, 255));
        }
        if b_r {
            draw_circle_lines(sx, sy, visual_r + 3.0, 1.5, Color::from_rgba(255, 60, 80, 255));
        }

        // Draw Heading Arrow
        let hx = heading.cos();
        let hy = heading.sin(); // standard math Y
        let tip_x = sx + hx * visual_r * 1.5;
        let tip_y = sy - hy * visual_r * 1.5; // flipped screen Y
        draw_line(sx, sy, tip_x, tip_y, 2.5, Color::from_rgba(255, 235, 60, 255));

        // --- RENDER GUI & OVERLAYS ---

        // 1. "Refresh Map" Button
        let btn_color = if btn_hover {
            Color::from_rgba(0, 229, 255, 255)
        } else {
            Color::from_rgba(0, 150, 200, 220)
        };
        draw_rectangle(btn_rect.x, btn_rect.y, btn_rect.w, btn_rect.h, btn_color);
        draw_text("REFRESH MAP [R]", btn_rect.x + 20.0, btn_rect.y + 25.0, 16.0, WHITE);

        // 2. Door Cycle Status
        let door_status_text = if is_top_doors_open {
            format!("DOORS OPEN (closing in {:.1}s)", doors_time_left)
        } else {
            format!("DOORS LOCKED (opening in {:.1}s)", doors_time_left)
        };
        let door_status_color = if is_top_doors_open {
            Color::from_rgba(40, 220, 100, 255)
        } else {
            Color::from_rgba(255, 60, 80, 255)
        };
        draw_text(&door_status_text, 250.0, 65.0, 18.0, door_status_color);

        // 3. Status Information Text
        let x_cm = rx / 10.0;
        let y_cm = ry / 10.0;
        let hdg_deg = heading.to_degrees().rem_euclid(360.0);
        let vel_cm = vel / 10.0;
        let ang_dps = angular.to_degrees();
        draw_text(
            &format!("Position: X={x_cm:.1}cm  Y={y_cm:.1}cm  Heading={hdg_deg:.0}°"),
            600.0,
            48.0,
            16.0,
            Color::from_rgba(200, 205, 220, 255),
        );
        draw_text(
            &format!("Velocity: Forward={vel_cm:.1}cm/s  Angular={ang_dps:.0}°/s"),
            600.0,
            68.0,
            16.0,
            Color::from_rgba(200, 205, 220, 255),
        );

        // 4. Live Lidar Sensor Readings Overlay
        let panel_x = 40.0;
        let panel_y = WIN_H - 120.0;
        draw_rectangle(panel_x, panel_y, 1120.0, 90.0, Color::from_rgba(35, 38, 48, 200));
        draw_rectangle_lines(panel_x, panel_y, 1120.0, 90.0, 1.0, Color::from_rgba(65, 75, 95, 150));
        draw_text("LIVE LIDAR SENSOR READINGS (cm)", panel_x + 20.0, panel_y + 25.0, 14.0, Color::from_rgba(0, 255, 255, 255));
        
        let lidar_labels = ["Front", "Front-Left", "Left", "Back-Left", "Back", "Back-Right", "Right", "Front-Right"];
        for i in 0..8 {
            let lx = panel_x + 20.0 + (i as f32 * 135.0);
            let dist_val = lidar_snap[i] / 10.0; // convert mm to cm
            let display_str = if dist_val >= 9999.0 {
                "---".to_string()
            } else {
                format!("{:.1} cm", dist_val)
            };
            draw_text(lidar_labels[i], lx, panel_y + 50.0, 12.0, Color::from_rgba(150, 160, 185, 255));
            draw_text(&display_str, lx, panel_y + 72.0, 15.0, WHITE);
        }

        // Help info
        draw_text(
            "TCP port :9999  —  Commands: move, turn, go, drive, stop, sense, bumps, lidar",
            600.0,
            120.0,
            14.0,
            Color::from_rgba(140, 150, 175, 200),
        );

        next_frame().await;
    }
}
