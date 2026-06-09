use macroquad::prelude::*;
use std::collections::VecDeque;
use std::io::{BufRead, BufReader, Write};
use std::net::{TcpListener, TcpStream};
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::{Duration, Instant};

mod map_data;

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
#[allow(dead_code)]
struct Door {
    name: String,
    p1: Vec2,
    p2: Vec2,
    is_open: bool,
    is_top_door: bool,
    is_external: bool,
    is_target: bool,
}

const HUMAN_RADIUS_MM: f32 = 250.0; // 25 cm radius

#[derive(Clone, Debug)]
struct Human {
    pos: Vec2,
    target: Vec2,
    path: Vec<Vec2>,
    speed: f32, // mm/s
    color: Color,
    is_enigma: bool,
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
    speed: f32,                // simulation speed multiplier (1.0 = real-time, 10.0 = 10× faster)
    route: Vec<(f32, f32)>,    // complete route history in mm
    seen_grid: Vec<bool>,      // visibility coverage grid (160x75 cells)
    target_door_mid: Option<Vec2>,
    target_door_open: bool,
}

impl RobotState {
    fn new() -> Self {
        // Starts in the middle of Enigma room: X_pdf = 1520.0, Y_pdf = 775.0
        Self {
            x: 1520.0 * PDF_TO_MM,
            y: 775.0 * PDF_TO_MM,
            heading: 0.0,
            vel: 0.0,
            angular: 0.0,
            deadline: None,
            trail: VecDeque::new(),
            bump_left: false,
            bump_right: false,
            lidar_distances: [0.0; 8],
            speed: 1.0,
            route: Vec::new(),
            seen_grid: vec![false; 12000],
            target_door_mid: None,
            target_door_open: false,
        }
    }

    fn arm(&mut self, dur: Duration) {
        // Divide real-time duration by speed so the deadline expires proportionally sooner.
        let scaled = Duration::from_secs_f64(dur.as_secs_f64() / self.speed as f64);
        self.deadline = Some(Instant::now() + scaled);
    }

    fn stop(&mut self) {
        self.vel = 0.0;
        self.angular = 0.0;
        self.deadline = None;
    }

    fn update(&mut self, dt: f32, walls: &[Segment], doors: &[Door], obstacles: &[Obstacle], humans: &[Human]) {
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

            // 2. Closed doors (and external doors treated as permanent walls)
            for door in doors {
                if !door.is_open || door.is_external {
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

            // 4. Humans
            for human in humans {
                let to_human = current_pos - human.pos;
                let dist_sq = to_human.length_squared();
                let min_dist = ROOMBA_RADIUS_MM + HUMAN_RADIUS_MM;
                if dist_sq < min_dist * min_dist {
                    let dist = dist_sq.sqrt();
                    let normal = if dist > 1e-4 { to_human / dist } else { Vec2::new(1.0, 0.0) };
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

        // Track route history (only add if moved at least 100 mm from last position)
        let current_pos = (self.x, self.y);
        if self.route.is_empty() {
            self.route.push(current_pos);
        } else {
            let last = self.route.last().unwrap();
            let dist_sq = (current_pos.0 - last.0).powi(2) + (current_pos.1 - last.1).powi(2);
            if dist_sq > 10000.0 { // 100 mm (10 cm) threshold
                self.route.push(current_pos);
            }
        }
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

// Resolve sliding collision of a circle against walls and closed/external doors
fn resolve_pos_collisions_with_map(
    pos: Vec2,
    radius: f32,
    walls: &[Segment],
    doors: &[Door],
) -> Vec2 {
    let mut new_pos = pos;
    for _ in 0..4 {
        let mut collision_resolved = false;
        for wall in walls {
            if let Some((normal, closest_pt)) = resolve_segment_collision(new_pos, radius, wall.p1, wall.p2) {
                let depth = radius - (new_pos - closest_pt).length();
                new_pos += normal * depth;
                collision_resolved = true;
            }
        }
        for door in doors {
            if !door.is_open || door.is_external {
                if let Some((normal, closest_pt)) = resolve_segment_collision(new_pos, radius, door.p1, door.p2) {
                    let depth = radius - (new_pos - closest_pt).length();
                    new_pos += normal * depth;
                    collision_resolved = true;
                }
            }
        }
        if !collision_resolved {
            break;
        }
    }
    new_pos
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
    humans: &[Human],
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
        if !door.is_open || door.is_external {
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

    for human in humans {
        if let Some(t) = ray_intersect_circle(origin, dir, human.pos, HUMAN_RADIUS_MM) {
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

// Distance from a point to a line segment
fn dist_to_segment(p: Vec2, p1: Vec2, p2: Vec2) -> f32 {
    let segment = p2 - p1;
    let to_p = p - p1;
    let seg_len_sq = segment.length_squared();
    if seg_len_sq < 1e-6 {
        return to_p.length();
    }
    let t = (to_p.dot(segment) / seg_len_sq).clamp(0.0, 1.0);
    let closest_point = p1 + t * segment;
    (p - closest_point).length()
}

fn is_segment_intersecting_map(p1: Vec2, p2: Vec2, walls: &[Segment], doors: &[Door]) -> bool {
    let to_end = p2 - p1;
    let dist = to_end.length();
    if dist < 1.0 {
        return false;
    }
    let dir = to_end / dist;
    
    // Check walls
    for wall in walls {
        if let Some(t) = ray_intersect_segment(p1, dir, wall.p1, wall.p2) {
            if t <= dist {
                return true;
            }
        }
    }
    
    // Check doors
    for door in doors {
        if let Some(t) = ray_intersect_segment(p1, dir, door.p1, door.p2) {
            if t <= dist {
                return true;
            }
        }
    }
    
    false
}

fn pick_human_target(is_enigma: bool, room_labels: &[RoomLabel]) -> (Vec2, bool) {
    let roll = macroquad::rand::gen_range(0.0, 1.0);
    let target_is_enigma = if is_enigma {
        roll >= 0.40
    } else {
        roll < 0.40
    };

    if target_is_enigma {
        let rx = macroquad::rand::gen_range(1350.0, 1580.0);
        let ry = macroquad::rand::gen_range(755.0, 855.0);
        (Vec2::new(rx * PDF_TO_MM, ry * PDF_TO_MM), true)
    } else {
        let non_enigma_labels: Vec<&RoomLabel> = room_labels
            .iter()
            .filter(|rl| !rl.name.contains("ENIGMA"))
            .collect();
        if non_enigma_labels.is_empty() {
            (Vec2::new(1520.0 * PDF_TO_MM, 775.0 * PDF_TO_MM), true)
        } else {
            let idx = macroquad::rand::rand() as usize % non_enigma_labels.len();
            (non_enigma_labels[idx].pos, false)
        }
    }
}

fn get_waypoint_for_pos(pos: Vec2) -> Vec2 {
    let x = pos.x / PDF_TO_MM;
    let y = pos.y / PDF_TO_MM;

    if x >= 1277.65 {
        // Enigma
        pdf_pt(1260.0, 730.0)
    } else if x <= 225.0 && y >= 714.48 {
        // Jack Good / David Blackwell
        pdf_pt(240.0, 730.0)
    } else if x <= 483.84 && y >= 744.08 {
        // Marian Rejewski / Joan Clarke
        pdf_pt(240.0, 730.0)
    } else if x <= 731.77 && y >= 744.08 {
        // Ada
        pdf_pt(675.0, 730.0)
    } else if x <= 893.80 && y >= 744.08 {
        // Lovelace
        pdf_pt(800.0, 730.0)
    } else if x <= 1277.65 && y >= 744.08 {
        // Margaret Hamilton / CEO Office
        pdf_pt(1160.0, 730.0)
    } else if x <= 536.84 && y <= 624.57 {
        // Bottom left rooms (Cipher, Nightingale, etc.)
        pdf_pt(510.0, 640.0)
    } else if x <= 1003.14 && y <= 697.78 {
        // Tea Point / Kitchen
        pdf_pt(880.0, 640.0)
    } else if x <= 1108.83 && y <= 623.96 {
        // Ursula Franklin / Lobby
        pdf_pt(1050.0, 640.0)
    } else {
        // Default to corridor center
        pdf_pt(800.0, 730.0)
    }
}

fn plan_path(start: Vec2, end: Vec2) -> Vec<Vec2> {
    let wp_start = get_waypoint_for_pos(start);
    let wp_end = get_waypoint_for_pos(end);
    
    let mut path = Vec::new();
    if wp_start.distance(wp_end) > 10.0 {
        path.push(wp_start);
        
        if (wp_start.y / PDF_TO_MM - 730.0).abs() > 10.0 {
            path.push(pdf_pt(wp_start.x / PDF_TO_MM, 730.0));
        }
        
        if (wp_end.y / PDF_TO_MM - 730.0).abs() > 10.0 {
            path.push(pdf_pt(wp_end.x / PDF_TO_MM, 730.0));
        }
        
        path.push(wp_end);
    }
    
    path.push(end);
    path
}

fn generate_humans(room_labels: &[RoomLabel], walls: &[Segment], doors: &[Door]) -> Vec<Human> {
    let colors = [
        Color::from_rgba(255, 105, 180, 255), // Hot Pink
        Color::from_rgba(218, 112, 214, 255), // Orchid
        Color::from_rgba(186, 85, 211, 255),  // Medium Orchid
        Color::from_rgba(147, 112, 219, 255), // Medium Slate Blue
        Color::from_rgba(138, 43, 226, 255),  // Blue Violet
    ];
    let mut humans = Vec::new();

    // 1. Spawn 10 Enigma humans
    for i in 0..10 {
        let rx = macroquad::rand::gen_range(1350.0, 1580.0);
        let ry = macroquad::rand::gen_range(755.0, 855.0);
        let mut pos = Vec2::new(rx * PDF_TO_MM, ry * PDF_TO_MM);
        pos = resolve_pos_collisions_with_map(pos, HUMAN_RADIUS_MM, walls, doors);
        let (target, is_en) = pick_human_target(true, room_labels);
        let path = plan_path(pos, target);
        let speed = macroquad::rand::gen_range(120.0, 180.0);
        humans.push(Human {
            pos,
            target,
            path,
            speed,
            color: colors[i % colors.len()],
            is_enigma: is_en,
        });
    }

    // 2. Spawn 10 Non-Enigma humans
    let non_enigma_labels: Vec<&RoomLabel> = room_labels
        .iter()
        .filter(|rl| !rl.name.contains("ENIGMA"))
        .collect();

    let mut attempts = 0;
    while humans.len() < 20 && attempts < 2000 {
        attempts += 1;
        if non_enigma_labels.is_empty() {
            break;
        }
        let idx = macroquad::rand::rand() as usize % non_enigma_labels.len();
        let label_pos = non_enigma_labels[idx].pos;

        let angle = macroquad::rand::gen_range(0.0, 2.0 * std::f32::consts::PI);
        let dist = macroquad::rand::gen_range(0.0, 1000.0);
        let offset = Vec2::new(angle.cos() * dist, angle.sin() * dist);
        let pos_mm = label_pos + offset;

        if is_segment_intersecting_map(label_pos, pos_mm, walls, doors) {
            continue;
        }

        let mut pos = pos_mm;
        pos = resolve_pos_collisions_with_map(pos, HUMAN_RADIUS_MM, walls, doors);
        if (pos - pos_mm).length_squared() > 1.0 {
            continue;
        }

        let (target, is_en) = pick_human_target(false, room_labels);
        let path = plan_path(pos, target);
        let speed = macroquad::rand::gen_range(120.0, 180.0);
        let i_h = humans.len();
        humans.push(Human {
            pos,
            target,
            path,
            speed,
            color: colors[i_h % colors.len()],
            is_enigma: is_en,
        });
    }

    humans
}

// Generate random obstacles avoiding start area, doors, and walls
fn generate_obstacles(
    roomba_start: Vec2,
    walls: &[Segment],
    doors: &[Door],
    room_labels: &[RoomLabel],
) -> Vec<Obstacle> {
    let mut obstacles: Vec<Obstacle> = Vec::new();

    // 1. Generate 9 obstacles inside Enigma
    let mut enigma_attempts = 0;
    while obstacles.len() < 9 && enigma_attempts < 1000 {
        enigma_attempts += 1;
        let rx = macroquad::rand::gen_range(1350.0, 1580.0);
        let ry = macroquad::rand::gen_range(755.0, 855.0);
        let pos_mm = Vec2::new(rx * PDF_TO_MM, ry * PDF_TO_MM);
        let radius = macroquad::rand::gen_range(180.0, 250.0);

        // Don't spawn on top of starting position
        if (pos_mm - roomba_start).length() < 1600.0 {
            continue;
        }

        // Don't spawn blocking doorways
        let mut near_door = false;
        for door in doors {
            let door_mid = (door.p1 + door.p2) * 0.5;
            if (pos_mm - door_mid).length() < 1200.0 {
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
            if (pos_mm - obs.center).length() < (obs.radius + radius + 150.0) {
                overlap = true;
                break;
            }
        }
        if overlap {
            continue;
        }

        // Avoid spawning inside or too close to walls and doors
        let resolved = resolve_pos_collisions_with_map(pos_mm, radius + 100.0, walls, doors);
        if (resolved - pos_mm).length_squared() > 1.0 {
            continue;
        }

        obstacles.push(Obstacle { center: pos_mm, radius });
    }

    // 2. Generate 9 obstacles inside the other rooms (non-Enigma, non-corridor)
    let non_enigma_non_corr_labels: Vec<&RoomLabel> = room_labels
        .iter()
        .filter(|rl| !rl.name.contains("ENIGMA") && !rl.name.contains("CORRIDOR"))
        .collect();

    let mut other_attempts = 0;
    while obstacles.len() < 18 && other_attempts < 2000 {
        other_attempts += 1;
        if non_enigma_non_corr_labels.is_empty() {
            break;
        }
        let idx = macroquad::rand::rand() as usize % non_enigma_non_corr_labels.len();
        let label_pos = non_enigma_non_corr_labels[idx].pos;

        let angle = macroquad::rand::gen_range(0.0, 2.0 * std::f32::consts::PI);
        let dist = macroquad::rand::gen_range(0.0, 2000.0);
        let offset = Vec2::new(angle.cos() * dist, angle.sin() * dist);
        let pos_mm = label_pos + offset;
        let radius = macroquad::rand::gen_range(180.0, 250.0);

        if is_segment_intersecting_map(label_pos, pos_mm, walls, doors) {
            continue;
        }

        // Don't spawn blocking doorways
        let mut near_door = false;
        for door in doors {
            let door_mid = (door.p1 + door.p2) * 0.5;
            if (pos_mm - door_mid).length() < 1200.0 {
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
            if (pos_mm - obs.center).length() < (obs.radius + radius + 150.0) {
                overlap = true;
                break;
            }
        }
        if overlap {
            continue;
        }

        // Avoid spawning inside or too close to walls and doors
        let resolved = resolve_pos_collisions_with_map(pos_mm, radius + 100.0, walls, doors);
        if (resolved - pos_mm).length_squared() > 1.0 {
            continue;
        }

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

        "route" => {
            let s = state.lock().unwrap();
            let mut coords = Vec::with_capacity(s.route.len());
            for &(rx, ry) in &s.route {
                coords.push(format!("{:.1},{:.1}", rx / 10.0, ry / 10.0));
            }
            format!("OK route {}", coords.join(" "))
        }

        "speed" => {
            if rest.len() != 1 {
                return "ERR usage: speed <multiplier>".into();
            }
            let val: f32 = num!(rest[0], f32);
            state.lock().unwrap().speed = val;
            format!("OK speed {val}")
        }

        "target" => {
            let s = state.lock().unwrap();
            if let Some(mid) = s.target_door_mid {
                let dist = Vec2::new(s.x, s.y).distance(mid);
                let door_state = if s.target_door_open { "open" } else { "closed" };
                format!("OK dist={:.0} door={}", dist, door_state)
            } else {
                "OK dist=inf".into()
            }
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
    let walls = map_data::get_walls();
    let mut doors = map_data::get_doors();
    let room_labels = map_data::get_room_labels();

    let roomba_start_pos = Vec2::new(1520.0 * PDF_TO_MM, 775.0 * PDF_TO_MM);
    let mut obstacles = generate_obstacles(roomba_start_pos, &walls, &doors, &room_labels);
    let mut humans = generate_humans(&room_labels, &walls, &doors);
    let mut door_had_human_near = vec![false; doors.len()];

    let print_diagnostics = |obs: &[Obstacle], hums: &[Human]| {
        let enigma_obs_count = obs.iter().filter(|o| {
            let x = o.center.x / PDF_TO_MM;
            let y = o.center.y / PDF_TO_MM;
            x >= 1333.74 && x <= 1603.54 && y >= 744.08 && y <= 866.63
        }).count();
        let enigma_hums_count = hums.iter().filter(|h| h.is_enigma).count();
        println!("DIAGNOSTICS: Total obstacles = {}, Enigma obstacles = {}; Total humans = {}, Enigma humans = {}",
            obs.len(), enigma_obs_count, hums.len(), enigma_hums_count);
    };
    print_diagnostics(&obstacles, &humans);

    let state = Arc::new(Mutex::new(RobotState::new()));
    start_tcp_server(Arc::clone(&state));

    loop {
        let dt = get_frame_time().min(0.05);

        // Read speed multiplier once per frame (avoids holding the lock longer than needed)
        let speed_val = state.lock().unwrap().speed;

        // Speed toggle: S key switches between 1× and 10×
        if is_key_pressed(KeyCode::S) {
            let mut s = state.lock().unwrap();
            s.speed = if s.speed > 1.0 { 1.0 } else { 10.0 };
        }

        // 1. Human-Reactive Door Updates (doors open when a human is near, close immediately with 50% probability when they leave)
        for (idx, door) in doors.iter_mut().enumerate() {
            let mut any_human_near = false;
            for human in &humans {
                if dist_to_segment(human.pos, door.p1, door.p2) < 1200.0 {
                    any_human_near = true;
                    break;
                }
            }
            if any_human_near {
                door.is_open = true;
                door_had_human_near[idx] = true;
            } else if door_had_human_near[idx] {
                door_had_human_near[idx] = false;
                if macroquad::rand::gen_range(0.0, 1.0) < 0.5 {
                    door.is_open = false;
                }
            }
        }

        // Sync target door state into RobotState for TCP access
        if let Some(td) = doors.iter().find(|d| d.is_target) {
            let mid = (td.p1 + td.p2) * 0.5;
            let td_open = td.is_open;
            let mut s = state.lock().unwrap();
            s.target_door_mid = Some(mid);
            s.target_door_open = td_open;
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
            s.route.clear();
            s.seen_grid.fill(false);
            obstacles = generate_obstacles(roomba_start_pos, &walls, &doors, &room_labels);
            humans = generate_humans(&room_labels, &walls, &doors);
            door_had_human_near.fill(false);
            print_diagnostics(&obstacles, &humans);
        }

        {
            let mut s = state.lock().unwrap();
            let total_dt = dt * speed_val;
            let step_size = 0.005_f32; // 5ms steps for extremely high precision
            let mut elapsed = 0.0;
            while elapsed < total_dt {
                let step = step_size.min(total_dt - elapsed);
                // Update humans
                for human in &mut humans {
                    if human.path.is_empty() {
                        let (new_target, is_en) = pick_human_target(human.is_enigma, &room_labels);
                        human.target = new_target;
                        human.is_enigma = is_en;
                        human.path = plan_path(human.pos, new_target);
                    }
                    
                    let next_wp = human.path[0];
                    let to_wp = next_wp - human.pos;
                    let dist = to_wp.length();
                    if dist < 300.0 {
                        human.path.remove(0);
                    } else {
                        let dir = to_wp / dist;
                        let candidate_pos = human.pos + dir * human.speed * step;
                        let resolved_pos = resolve_pos_collisions_with_map(candidate_pos, HUMAN_RADIUS_MM, &walls, &doors);
                        if (resolved_pos - human.pos).length() < 10.0 * step {
                            // Blocked or stuck, choose a new target and replan
                            let (new_target, is_en) = pick_human_target(human.is_enigma, &room_labels);
                            human.target = new_target;
                            human.is_enigma = is_en;
                            human.path = plan_path(human.pos, new_target);
                        } else {
                            human.pos = resolved_pos;
                        }
                    }
                }
                
                s.update(step, &walls, &doors, &obstacles, &humans);
                elapsed += step;
            }

            // Cast Lidar rays (8 directions relative to heading)
            let origin = Vec2::new(s.x, s.y);
            let mut lidar_vals = [0.0; 8];
            let angles = [0.0, 45.0, 90.0, 135.0, 180.0, 225.0, 270.0, 315.0];
            for i in 0..8 {
                let angle_rad = s.heading + (angles[i] as f32).to_radians();
                let dir = Vec2::new(angle_rad.cos(), angle_rad.sin());
                lidar_vals[i] = cast_ray(origin, dir, &walls, &doors, &obstacles, &humans);
            }
            s.lidar_distances = lidar_vals;

            // Mark immediate area (radius 1.5m = 1500mm) as seen
            let rx = s.x;
            let ry = s.y;
            let cell_x = (rx / (10.0 * PDF_TO_MM)) as i32;
            let cell_y = ((ry - 250.0 * PDF_TO_MM) / (10.0 * PDF_TO_MM)) as i32;
            let reveal_radius = 4;
            for dx in -reveal_radius..=reveal_radius {
                for dy in -reveal_radius..=reveal_radius {
                    if dx*dx + dy*dy <= reveal_radius * reveal_radius {
                        let gx = cell_x + dx;
                        let gy = cell_y + dy;
                        if gx >= 0 && gx < 160 && gy >= 0 && gy < 75 {
                            s.seen_grid[(gx * 75 + gy) as usize] = true;
                        }
                    }
                }
            }

            // Reveal grid cells along lidar rays
            for i in 0..8 {
                let dist_mm = s.lidar_distances[i];
                let angle_rad = s.heading + (angles[i] as f32).to_radians();
                let dir = Vec2::new(angle_rad.cos(), angle_rad.sin());
                
                let steps = (dist_mm / 150.0) as i32;
                for step in 0..=steps {
                    let pt = origin + dir * (step as f32 * 150.0);
                    let gx = (pt.x / (10.0 * PDF_TO_MM)) as i32;
                    let gy = ((pt.y - 250.0 * PDF_TO_MM) / (10.0 * PDF_TO_MM)) as i32;
                    if gx >= 0 && gx < 160 && gy >= 0 && gy < 75 {
                        s.seen_grid[(gx * 75 + gy) as usize] = true;
                    }
                }
            }
        }

        // --- RENDER ---
        clear_background(Color::from_rgba(20, 22, 28, 255)); // sleek dark background

        // Retrieve robot state details for rendering
        let (rx, ry, heading, vel, angular, trail_snap, b_l, b_r, lidar_snap, seen_snap) = {
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
                s.seen_grid.clone(),
            )
        };

        // Draw Map Border Box
        let (x_min_s, y_max_s) = to_screen(0.0, 250.0 * PDF_TO_MM);
        let (x_max_s, y_min_s) = to_screen(1600.0 * PDF_TO_MM, 1000.0 * PDF_TO_MM);
        draw_rectangle(x_min_s, y_min_s, x_max_s - x_min_s, y_max_s - y_min_s, Color::from_rgba(30, 34, 45, 255));
        draw_rectangle_lines(x_min_s, y_min_s, x_max_s - x_min_s, y_max_s - y_min_s, 2.0, Color::from_rgba(65, 75, 95, 255));

        // Draw Seen Grid (Fog of War Reveal Highlight)
        let scale = 1120.0 / MAP_WIDTH_MM;
        let cell_w_s = 10.0 * PDF_TO_MM * scale;
        let cell_h_s = 10.0 * PDF_TO_MM * scale;
        for gx in 0..160 {
            for gy in 0..75 {
                let idx = gx * 75 + gy;
                if seen_snap[idx as usize] {
                    let px = gx as f32 * 10.0 * PDF_TO_MM;
                    let py = (250.0 + gy as f32 * 10.0) * PDF_TO_MM;
                    let (sx, sy) = to_screen(px, py);
                    draw_rectangle(
                        sx,
                        sy - cell_h_s,
                        cell_w_s,
                        cell_h_s,
                        Color::from_rgba(0, 255, 200, 20), // faint cyan glow
                    );
                }
            }
        }

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



        // Draw Static Walls
        for wall in &walls {
            let (sx1, sy1) = to_screen(wall.p1.x, wall.p1.y);
            let (sx2, sy2) = to_screen(wall.p2.x, wall.p2.y);
            draw_line(sx1, sy1, sx2, sy2, 2.0, Color::from_rgba(200, 205, 220, 255));
        }

        // Draw Dynamic Doors
        for door in &doors {
            if door.is_external {
                continue; // external doors are invisible (treated as solid walls)
            }
            let (sx1, sy1) = to_screen(door.p1.x, door.p1.y);
            let (sx2, sy2) = to_screen(door.p2.x, door.p2.y);
            if door.is_target {
                // Pulsing gold highlight for target exit door
                let pulse = (get_time() as f32 * 3.0).sin() * 0.5 + 0.5;
                let alpha = (150.0 + pulse * 105.0) as u8;
                draw_line(sx1, sy1, sx2, sy2, 8.0, Color::from_rgba(255, 200, 0, 40));
                draw_line(sx1, sy1, sx2, sy2, 4.0, Color::from_rgba(255, 200, 0, alpha));
            } else if door.is_open {
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

        // Draw Humans
        for human in &humans {
            let (hx, hy) = to_screen(human.pos.x, human.pos.y);
            let h_rad = HUMAN_RADIUS_MM * scale;
            draw_circle(hx, hy, h_rad, human.color);
            draw_circle_lines(hx, hy, h_rad, 1.0, Color::from_rgba(255, 255, 255, 180));
            
            // Draw a small direction vector
            let to_target = human.target - human.pos;
            if to_target.length_squared() > 1.0 {
                let dir = to_target.normalize();
                let tip_x = hx + dir.x * h_rad * 1.5;
                let tip_y = hy - dir.y * h_rad * 1.5; // flipped screen Y
                draw_line(hx, hy, tip_x, tip_y, 1.5, Color::from_rgba(255, 255, 255, 200));
            }
        }



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

        // Draw Roomba Body (Proportional to the room map, Turquoise neon)
        let (sx, sy) = to_screen(rx, ry);
        let visual_r = ROOMBA_RADIUS_MM * scale; // Proportional size
        
        // Draw a pulsing outer ring for high visibility
        let pulse = (get_time() as f32 * 4.0).sin() * 0.5 + 0.5;
        let pulse_r = visual_r + 4.0 + pulse * 6.0;
        let alpha = (120.0 + pulse * 100.0) as u8;
        draw_circle_lines(sx, sy, pulse_r, 1.0, Color::from_rgba(0, 255, 255, alpha));
        
        draw_circle(sx, sy, visual_r, Color::from_rgba(0, 210, 255, 200));
        draw_circle_lines(sx, sy, visual_r, 1.5, Color::from_rgba(0, 255, 255, 255));

        // Draw Bumps Glow Indicator
        if b_l {
            draw_circle_lines(sx, sy, visual_r + 3.0, 1.5, Color::from_rgba(255, 60, 80, 255));
        }
        if b_r {
            draw_circle_lines(sx, sy, visual_r + 3.0, 1.5, Color::from_rgba(255, 60, 80, 255));
        }

        // Draw Heading Arrow (extending slightly past the body)
        let hx = heading.cos();
        let hy = heading.sin(); // standard math Y
        let tip_x = sx + hx * visual_r * 2.0;
        let tip_y = sy - hy * visual_r * 2.0; // flipped screen Y
        draw_line(sx, sy, tip_x, tip_y, 2.0, Color::from_rgba(255, 235, 60, 255));

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
        let door_status_text = "DYNAMIC DOORS: ACTIVE (REACTIVE TO HUMANS)";
        let door_status_color = Color::from_rgba(0, 255, 255, 255);
        draw_text(door_status_text, 250.0, 65.0, 18.0, door_status_color);

        // Target door proximity indicator
        if let Some(td) = doors.iter().find(|d| d.is_target) {
            let mid = (td.p1 + td.p2) * 0.5;
            let dist = Vec2::new(rx, ry).distance(mid);
            if dist < 1500.0 {
                let pulse = (get_time() as f32 * 4.0).sin() * 0.5 + 0.5;
                let alpha = (180.0 + pulse * 75.0) as u8;
                draw_text(
                    &format!("★ TARGET DOOR NEARBY  ({:.0} mm)", dist),
                    250.0,
                    90.0,
                    18.0,
                    Color::from_rgba(255, 200, 0, alpha),
                );
            }
        }

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
            &format!("Velocity: Forward={vel_cm:.1}cm/s  Angular={ang_dps:.0}°/s  |  Speed: {speed_val:.0}× [S]"),
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
