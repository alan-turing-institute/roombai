//! 5-minute sensor/odometry/camera characterization session runner (spec §6, S1).
//!
//! Fully scripted: once started it narrates each phase over TTS and needs no
//! keyboard input (unless --step). Ctrl-C stops the wheels and wraps up
//! cleanly at the next tick.
//!
//!   characterize --out /tmp/s1 [--port /dev/ttyUSB0] [--dry-run] [--skip-camera] [--step] [--from B]

mod camera;
mod logger;
mod mock;
mod phases;
mod rig;
mod telemetry;
mod tts;

use std::io::{Read, Write};
use std::path::PathBuf;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

use rig::Rig;

const BAUD: u32 = 115_200;

struct Opts {
    port: String,
    out: PathBuf,
    dry_run: bool,
    camera: bool,
    step: bool,
    from: char, // first phase to run: A..E (resume aid if the runner is restarted mid-session)
}

fn usage() -> ! {
    eprintln!("usage: characterize --out DIR [--port /dev/ttyUSB0] [--dry-run] [--skip-camera] [--step] [--from A|B|C|D]");
    std::process::exit(2);
}

fn parse_args() -> Opts {
    let mut port = "/dev/ttyUSB0".to_string();
    let mut out = None;
    let (mut dry_run, mut camera, mut step, mut from) = (false, true, false, 'A');
    let mut it = std::env::args().skip(1);
    while let Some(a) = it.next() {
        match a.as_str() {
            "--port" => port = it.next().unwrap_or_else(|| usage()),
            "--out" => out = Some(PathBuf::from(it.next().unwrap_or_else(|| usage()))),
            "--dry-run" => dry_run = true,
            "--skip-camera" => camera = false,
            "--step" => step = true,
            "--from" => {
                let p = it.next().unwrap_or_else(|| usage());
                from = p.chars().next().unwrap_or('A').to_ascii_uppercase();
                if !('A'..='D').contains(&from) {
                    usage();
                }
            }
            _ => usage(),
        }
    }
    Opts { port, out: out.unwrap_or_else(|| usage()), dry_run, camera: camera && !dry_run, step, from }
}

fn main() {
    let opts = parse_args();
    std::fs::create_dir_all(&opts.out).expect("create out dir");
    let abort = Arc::new(AtomicBool::new(false));
    {
        let a = abort.clone();
        ctrlc::set_handler(move || a.store(true, Ordering::SeqCst)).expect("install ctrl-c handler");
    }
    if opts.dry_run {
        run(Rig::new(mock::MockRoomba::new()), opts, abort);
    } else {
        let port = serialport::new(&opts.port, BAUD)
            .timeout(Duration::from_millis(100))
            .open()
            .expect("open serial port (roomba powered on? pilot daemon stopped?)");
        run(Rig::new(port), opts, abort);
    }
}

fn run<T: Read + Write>(rig: Rig<T>, opts: Opts, abort: Arc<AtomicBool>) {
    let t0 = Instant::now();
    let log = logger::Logger::new(&opts.out.join("session.jsonl"), t0);
    std::fs::write(opts.out.join("notes_template.md"), phases::NOTES_TEMPLATE).ok();
    let mut cam = camera::Camera::new(opts.camera, opts.out.clone());

    let mut ctx = phases::Ctx {
        rig,
        log,
        odom: telemetry::Odom::default(),
        abort,
        t0,
        step: opts.step,
        results: Vec::new(),
    };

    tts::say("Characterization session starting.");

    // Phase A always runs: later phases need the capability table, and it is
    // cheap and stationary.
    let caps = phases::probe(&mut ctx);

    cam.start_video();
    ctx.log.event("session", "video started", serde_json::json!({"t_offset_s": t0.elapsed().as_secs_f64()}));

    // Sync pulse first thing on video: aligns the video clock to the encoder
    // log offline via motion-onset cross-correlation (no external sync prop).
    if !ctx.aborted() {
        phases::sync_pulse(&mut ctx, &caps);
    }

    let run_phase = |c: char| opts.from <= c;
    ctx.pause_if_step();
    if run_phase('B') && !ctx.aborted() {
        phases::wall_run(&mut ctx, &caps);
    }
    ctx.pause_if_step();
    if run_phase('C') && !ctx.aborted() {
        phases::rotation(&mut ctx, &caps);
    }
    ctx.pause_if_step();
    if run_phase('D') && !ctx.aborted() {
        phases::floor_traverse(&mut ctx, &caps);
    }
    cam.stop_video();

    // Wrap: wheels stopped, OI back to passive, summary on disk.
    let _ = ctx.rig.stop();
    let _ = ctx.rig.passive();
    let summary = serde_json::json!({
        "caps": caps.to_json(),
        "phases": std::mem::take(&mut ctx.results),
        "total_s": t0.elapsed().as_secs_f64(),
        "aborted": ctx.aborted(),
    });
    let pretty = serde_json::to_string_pretty(&summary).unwrap();
    std::fs::write(opts.out.join("summary.json"), &pretty).ok();
    tts::say(&format!("Session complete in {} seconds.", t0.elapsed().as_secs()));
    println!("{pretty}");
}
