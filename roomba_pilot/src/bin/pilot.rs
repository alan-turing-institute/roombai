//! Roomba pilot daemon + client.
//!
//!   pilot serve [SERIAL_PATH] [TCP_PORT]   run the control daemon
//!   pilot send  "<command>"  [TCP_PORT]    send one command, print the reply
//!
//! The daemon opens the serial port once, holds it open, enters SAFE mode, and
//! accepts newline-delimited text commands over a localhost TCP socket. A
//! watchdog thread auto-stops the robot when a motion deadline passes, so a
//! "go" that is never followed up cannot run away.

use std::io::{BufRead, BufReader, Write};
use std::net::{TcpListener, TcpStream};
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::{Duration, Instant};

use roomba_pilot::robot::Robot;
use roomba_pilot::sensors;

const DEFAULT_SERIAL: &str = "/dev/tty.usbserial-BG03LB1L";
const DEFAULT_TCP_PORT: u16 = 9999;
const BAUD: u32 = 115_200;

/// Safety ceiling for raw continuous motion commands (go/drive): if not
/// refreshed within this window the watchdog stops the robot.
const CONTINUOUS_MAX: Duration = Duration::from_secs(3);
/// Fixed rates for distance/angle based convenience commands.
const MOVE_SPEED_CM_S: f64 = 20.0;
const TURN_RATE_DEG_S: f64 = 60.0;

type Port = Box<dyn serialport::SerialPort>;

struct Shared {
    robot: Mutex<Robot<Port>>,
    /// When `Some(t)`, the robot is moving and must be stopped at time `t`.
    deadline: Mutex<Option<Instant>>,
}

impl Shared {
    fn arm(&self, dur: Duration) {
        *self.deadline.lock().unwrap() = Some(Instant::now() + dur);
    }
    fn disarm(&self) {
        *self.deadline.lock().unwrap() = None;
    }
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let mode = args.get(1).map(String::as_str).unwrap_or("serve");
    match mode {
        "serve" => {
            let serial = args.get(2).map(String::as_str).unwrap_or(DEFAULT_SERIAL);
            let tcp = args
                .get(3)
                .and_then(|s| s.parse().ok())
                .unwrap_or(DEFAULT_TCP_PORT);
            if let Err(e) = serve(serial, tcp) {
                eprintln!("pilot: fatal: {e}");
                std::process::exit(1);
            }
        }
        "send" => {
            let cmd = args.get(2).cloned().unwrap_or_default();
            let tcp = args
                .get(3)
                .and_then(|s| s.parse().ok())
                .unwrap_or(DEFAULT_TCP_PORT);
            if let Err(e) = client(&cmd, tcp) {
                eprintln!("pilot: {e}");
                std::process::exit(1);
            }
        }
        other => {
            eprintln!("unknown mode '{other}'. use 'serve' or 'send'.");
            std::process::exit(2);
        }
    }
}

fn client(cmd: &str, tcp_port: u16) -> std::io::Result<()> {
    let mut stream = TcpStream::connect(("127.0.0.1", tcp_port))?;
    stream.write_all(cmd.as_bytes())?;
    stream.write_all(b"\n")?;
    stream.flush()?;
    stream.set_read_timeout(Some(Duration::from_secs(5)))?;
    let mut buf = String::new();
    BufReader::new(stream).read_line(&mut buf)?;
    print!("{buf}");
    Ok(())
}

fn serve(serial_path: &str, tcp_port: u16) -> Result<(), Box<dyn std::error::Error>> {
    println!("pilot: opening serial {serial_path} @ {BAUD}");
    let port = serialport::new(serial_path, BAUD)
        .timeout(Duration::from_millis(2000))
        .open()?;

    let mut robot = Robot::new(port);
    // Wake + enter safe mode, mirroring create.py's startup delays.
    thread::sleep(Duration::from_millis(300));
    robot.start()?;
    thread::sleep(Duration::from_millis(300));
    robot.enter_safe()?;
    thread::sleep(Duration::from_millis(100));
    println!("pilot: robot in SAFE mode");

    let shared = Arc::new(Shared {
        robot: Mutex::new(robot),
        deadline: Mutex::new(None),
    });

    // Watchdog: stop the robot when a motion deadline passes.
    {
        let shared = Arc::clone(&shared);
        thread::spawn(move || loop {
            thread::sleep(Duration::from_millis(50));
            let expired = {
                let d = shared.deadline.lock().unwrap();
                matches!(*d, Some(t) if Instant::now() >= t)
            };
            if expired {
                let _ = shared.robot.lock().unwrap().stop();
                shared.disarm();
            }
        });
    }

    let listener = TcpListener::bind(("127.0.0.1", tcp_port))?;
    println!("pilot: listening on 127.0.0.1:{tcp_port}");
    for stream in listener.incoming() {
        match stream {
            Ok(s) => {
                let shared = Arc::clone(&shared);
                thread::spawn(move || handle_conn(s, shared));
            }
            Err(e) => eprintln!("pilot: accept error: {e}"),
        }
    }
    Ok(())
}

fn handle_conn(stream: TcpStream, shared: Arc<Shared>) {
    let peer = stream.peer_addr().ok();
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
        let resp = dispatch(line.trim(), &shared);
        if writer.write_all(resp.as_bytes()).is_err() || writer.write_all(b"\n").is_err() {
            break;
        }
        let _ = writer.flush();
        if resp.starts_with("OK shutdown") {
            println!("pilot: shutdown requested by {peer:?}");
            std::process::exit(0);
        }
    }
}

/// Execute one text command, returning a single-line response.
fn dispatch(line: &str, shared: &Shared) -> String {
    let mut parts = line.split_whitespace();
    let cmd = match parts.next() {
        Some(c) => c,
        None => return "ERR empty".into(),
    };
    let rest: Vec<&str> = parts.collect();

    macro_rules! robot {
        () => {
            shared.robot.lock().unwrap()
        };
    }
    macro_rules! num {
        ($s:expr, $t:ty) => {
            match $s.parse::<$t>() {
                Ok(v) => v,
                Err(_) => return format!("ERR bad number '{}'", $s),
            }
        };
    }
    macro_rules! io {
        ($e:expr) => {
            match $e {
                Ok(_) => {}
                Err(e) => return format!("ERR io {e}"),
            }
        };
    }

    match cmd {
        "ping" => "OK pong".into(),

        "safe" => {
            io!(robot!().enter_safe());
            "OK safe".into()
        }
        "full" => {
            io!(robot!().enter_full());
            "OK full".into()
        }

        "stop" => {
            shared.disarm();
            io!(robot!().stop());
            "OK stop".into()
        }

        // raw continuous translation+rotation, refreshes the 3s safety window
        "go" => {
            if rest.len() != 2 {
                return "ERR usage: go <cm_per_sec> <deg_per_sec>".into();
            }
            let cm = num!(rest[0], f64);
            let deg = num!(rest[1], f64);
            io!(robot!().go(cm, deg));
            shared.arm(CONTINUOUS_MAX);
            format!("OK go {cm} {deg}")
        }

        // raw continuous per-wheel velocities (mm/s), 3s safety window
        "drive" => {
            if rest.len() != 2 {
                return "ERR usage: drive <left_mm_s> <right_mm_s>".into();
            }
            let left = num!(rest[0], i16);
            let right = num!(rest[1], i16);
            io!(robot!().drive_direct(right, left));
            shared.arm(CONTINUOUS_MAX);
            format!("OK drive L{left} R{right}")
        }

        // timed forward/back: forward <cm_per_sec> <seconds>
        "forward" | "back" => {
            if rest.len() != 2 {
                return format!("ERR usage: {cmd} <cm_per_sec> <seconds>");
            }
            let mut speed = num!(rest[0], f64).abs();
            if cmd == "back" {
                speed = -speed;
            }
            let secs = num!(rest[1], f64);
            io!(robot!().go(speed, 0.0));
            shared.arm(Duration::from_secs_f64(secs.max(0.0)));
            format!("OK {cmd} {speed}cm/s {secs}s")
        }

        // timed spin in place: spin <deg_per_sec> <seconds>  (+ccw / -cw)
        "spin" => {
            if rest.len() != 2 {
                return "ERR usage: spin <deg_per_sec> <seconds>".into();
            }
            let rate = num!(rest[0], f64);
            let secs = num!(rest[1], f64);
            io!(robot!().go(0.0, rate));
            shared.arm(Duration::from_secs_f64(secs.max(0.0)));
            format!("OK spin {rate}deg/s {secs}s")
        }

        // distance move: move <cm>  (signed) at fixed speed, auto-stop
        "move" => {
            if rest.len() != 1 {
                return "ERR usage: move <cm>".into();
            }
            let dist = num!(rest[0], f64);
            let speed = if dist < 0.0 { -MOVE_SPEED_CM_S } else { MOVE_SPEED_CM_S };
            let secs = (dist / speed).abs();
            io!(robot!().go(speed, 0.0));
            shared.arm(Duration::from_secs_f64(secs));
            format!("OK move {dist}cm (~{secs:.1}s)")
        }

        // angle turn: turn <deg>  (+ccw / -cw) at fixed rate, auto-stop
        "turn" => {
            if rest.len() != 1 {
                return "ERR usage: turn <deg>".into();
            }
            let deg = num!(rest[0], f64);
            let rate = if deg < 0.0 { -TURN_RATE_DEG_S } else { TURN_RATE_DEG_S };
            let secs = (deg / rate).abs();
            io!(robot!().go(0.0, rate));
            shared.arm(Duration::from_secs_f64(secs));
            format!("OK turn {deg}deg (~{secs:.1}s)")
        }

        "motors" => {
            if rest.len() != 3 {
                return "ERR usage: motors <side -1|0|1> <main -1|0|1> <vac 0|1>".into();
            }
            let s = num!(rest[0], i8);
            let m = num!(rest[1], i8);
            let v = num!(rest[2], i8);
            io!(robot!().motors(s, m, v));
            format!("OK motors side{s} main{m} vac{v}")
        }

        "led" => {
            if rest.len() != 4 {
                return "ERR usage: led <color 0-255> <intensity 0-255> <play 0|1> <advance 0|1>"
                    .into();
            }
            let color = num!(rest[0], u8);
            let inten = num!(rest[1], u8);
            let play = num!(rest[2], u8) != 0;
            let adv = num!(rest[3], u8) != 0;
            io!(robot!().leds(color, inten, play, adv));
            "OK led".into()
        }

        "dock" => {
            shared.disarm();
            io!(robot!().seek_dock());
            "OK dock".into()
        }

        // battery + safety panel
        "sense" => {
            let ids = [
                sensors::OI_MODE,
                sensors::CHARGING_STATE,
                sensors::VOLTAGE,
                sensors::CURRENT,
                sensors::BATTERY_TEMP,
                sensors::BATTERY_CHARGE,
                sensors::BATTERY_CAPACITY,
                sensors::BUMPS_AND_WHEEL_DROPS,
            ];
            match robot!().query(&ids) {
                Ok(r) => {
                    let v = |id: u8| r.iter().find(|x| x.id == id).map(|x| x.value).unwrap_or(0);
                    let b = sensors::bumps(v(sensors::BUMPS_AND_WHEEL_DROPS));
                    let cap = v(sensors::BATTERY_CAPACITY).max(1);
                    let pct = 100 * v(sensors::BATTERY_CHARGE) / cap;
                    format!(
                        "OK mode={} charging={} volt={}mV current={}mA temp={}C charge={}/{}mAh ({}%) bumpL={} bumpR={}",
                        v(sensors::OI_MODE),
                        v(sensors::CHARGING_STATE),
                        v(sensors::VOLTAGE),
                        v(sensors::CURRENT),
                        v(sensors::BATTERY_TEMP),
                        v(sensors::BATTERY_CHARGE),
                        v(sensors::BATTERY_CAPACITY),
                        pct,
                        b.left_bump as u8,
                        b.right_bump as u8,
                    )
                }
                Err(e) => format!("ERR sense {e}"),
            }
        }

        "bumps" => {
            match robot!().query(&[sensors::BUMPS_AND_WHEEL_DROPS]) {
                Ok(r) => {
                    let b = sensors::bumps(r[0].value);
                    format!(
                        "OK bumpL={} bumpR={} dropL={} dropR={} caster={}",
                        b.left_bump as u8,
                        b.right_bump as u8,
                        b.left_wheel_drop as u8,
                        b.right_wheel_drop as u8,
                        b.caster_drop as u8,
                    )
                }
                Err(e) => format!("ERR bumps {e}"),
            }
        }

        "shutdown" | "quit" => {
            shared.disarm();
            let _ = robot!().stop();
            let _ = robot!().start(); // back to passive
            "OK shutdown".into()
        }

        other => format!("ERR unknown command '{other}'"),
    }
}
