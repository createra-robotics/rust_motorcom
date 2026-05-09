//! Slowly drive one or more Damiao actuators to their nearest "zero" — the
//! closest multiple of 2π to where they currently are. After a power cycle
//! the multi-turn counter resets, so a motor can report any starting angle;
//! rounding the goal to the nearest 2π keeps travel within ±π.
//!
//! Usage:
//! ```text
//! cargo run --example damiao_homing -- <master_id:can_id> [<master_id:can_id> ...]
//!     [--speed 0.2] [--kp 30] [--kd 1] [--tolerance 0.15] [--channel can0]
//!
//! cargo run --example damiao_homing -- 0:1
//! cargo run --example damiao_homing -- 0:1 0:2 0:3 --speed 0.1
//! ```
//!
//! Bring the CAN interface up first:
//! ```text
//! sudo ip link set can0 up type can bitrate 1000000
//! ```

use std::env;
use std::io::{self, Write};
use std::process;
use std::thread;
use std::time::Duration;

use motorcom::damiao::mit::{MitLimits, MitSetpoint};
use motorcom::damiao::{
    ControlMode, Damiao, DM10010, DM10010L, DM3507, DM4310, DM4310P, DM4340, DM4340P, DM6006,
    DM6248P, DM8006, DM8009, DMG6215, DMH3510, DMH6220, DMJH11,
};
use motorcom::transport::SocketCanTransport;

const VARIANTS: &[(&str, MitLimits)] = &[
    ("DM3507",   DM3507),
    ("DM4310",   DM4310),
    ("DM4310P",  DM4310P),
    ("DM4340",   DM4340),
    ("DM4340P",  DM4340P),
    ("DM6006",   DM6006),
    ("DM6248P",  DM6248P),
    ("DM8006",   DM8006),
    ("DM8009",   DM8009),
    ("DM10010",  DM10010),
    ("DM10010L", DM10010L),
    ("DMG6215",  DMG6215),
    ("DMH3510",  DMH3510),
    ("DMH6220",  DMH6220),
    ("DMJH11",   DMJH11),
];

const TWO_PI: f32 = 2.0 * std::f32::consts::PI;
const DT: Duration = Duration::from_millis(10);
const VEL_SETTLE: f32 = 0.3; // rad/s

struct CliArgs {
    motors: Vec<(u16, u16)>, // (master_id, motor_id)
    speed: f32,
    kp: f32,
    kd: f32,
    tolerance: f32,
    channel: String,
}

fn parse_args() -> CliArgs {
    let mut motors = Vec::new();
    let mut speed = 0.2_f32;
    let mut kp = 30.0_f32;
    let mut kd = 1.0_f32;
    let mut tolerance = 0.15_f32;
    let mut channel = "can0".to_string();

    let argv: Vec<String> = env::args().skip(1).collect();
    let mut i = 0;
    while i < argv.len() {
        let a = &argv[i];
        match a.as_str() {
            "-h" | "--help" => {
                eprintln!(
                    "Usage: damiao_homing <master_id:can_id> ... \
                     [--speed RAD_S] [--kp N] [--kd N] [--tolerance RAD] [--channel can0]"
                );
                process::exit(0);
            }
            "--speed"     => { speed     = next_arg(&argv, &mut i).parse().expect("--speed"); }
            "--kp"        => { kp        = next_arg(&argv, &mut i).parse().expect("--kp"); }
            "--kd"        => { kd        = next_arg(&argv, &mut i).parse().expect("--kd"); }
            "--tolerance" => { tolerance = next_arg(&argv, &mut i).parse().expect("--tolerance"); }
            "--channel"   => { channel   = next_arg(&argv, &mut i); }
            s => {
                let (m, c) = s.split_once(':').unwrap_or_else(|| {
                    eprintln!("Bad motor spec '{s}', expected master_id:can_id");
                    process::exit(1);
                });
                let master: u16 = m.parse().expect("master_id");
                let can: u16 = c.parse().expect("can_id");
                motors.push((master, can));
            }
        }
        i += 1;
    }
    if motors.is_empty() {
        eprintln!("Need at least one motor (master_id:can_id). Try --help.");
        process::exit(1);
    }
    CliArgs { motors, speed, kp, kd, tolerance, channel }
}

fn next_arg(argv: &[String], i: &mut usize) -> String {
    *i += 1;
    argv.get(*i)
        .cloned()
        .unwrap_or_else(|| {
            eprintln!("Missing value for {}", argv[*i - 1]);
            process::exit(1);
        })
}

fn select_variant() -> (&'static str, MitLimits) {
    println!("\nAvailable actuator types:");
    for (i, (name, _)) in VARIANTS.iter().enumerate() {
        println!("  {:>2}) {name}", i + 1);
    }
    loop {
        print!("\nSelect actuator type [1-{}]: ", VARIANTS.len());
        io::stdout().flush().ok();
        let mut buf = String::new();
        if io::stdin().read_line(&mut buf).is_err() {
            continue;
        }
        match buf.trim().parse::<usize>() {
            Ok(n) if (1..=VARIANTS.len()).contains(&n) => {
                let v = VARIANTS[n - 1];
                println!("Selected: {}", v.0);
                return v;
            }
            _ => println!("Please enter a number between 1 and {}.", VARIANTS.len()),
        }
    }
}

struct MotorCtx {
    label: String,
    dm: Damiao,
    target: f32,
    goal: f32,
    settled: bool,
}

fn run() -> Result<(), Box<dyn std::error::Error>> {
    let args = parse_args();
    let (variant_name, limits) = select_variant();

    let mut bus = SocketCanTransport::open(&args.channel)?;

    // Add and enable each motor; read its starting position.
    let mut motors: Vec<MotorCtx> = Vec::with_capacity(args.motors.len());
    for (master_id, motor_id) in &args.motors {
        let dm = Damiao::new(*motor_id, *master_id).with_limits(limits);
        let label = format!("{master_id}:{motor_id}");

        // Force MIT control mode (register 10 = 1). Some actuators ship or
        // end up in POS_VEL/VEL/FORCE_POS, in which case mit_control is
        // silently ignored — bus traffic looks fine, no torque is produced.
        if let Err(e) = dm.ensure_control_mode(&mut bus, ControlMode::Mit) {
            eprintln!("Warning: could not verify MIT mode for {label}: {e}");
        }
        let start_state = dm.enable(&mut bus)?;
        let start = start_state.position;
        let goal = (start / TWO_PI).round() * TWO_PI;
        let distance = start - goal;
        println!(
            "Actuator {label} ({variant_name}) at {start:+.3} rad, \
             homing to {goal:+.3} rad (distance {distance:+.3} rad)"
        );
        motors.push(MotorCtx { label, dm, target: start, goal, settled: false });
    }

    println!(
        "Homing {} actuator(s) to zero at {} rad/s...",
        motors.len(),
        args.speed
    );

    let step = args.speed * (DT.as_secs_f32());
    let max_distance = motors
        .iter()
        .map(|m| (m.target - m.goal).abs())
        .fold(0.0_f32, f32::max);
    let max_iters = (max_distance / step) as usize + 1000;

    let mut all_settled = false;
    for i in 0..max_iters {
        // Step un-settled motors toward their goals.
        for m in motors.iter_mut() {
            if m.settled {
                continue;
            }
            let diff = m.target - m.goal;
            if diff > step {
                m.target -= step;
            } else if diff < -step {
                m.target += step;
            } else {
                m.target = m.goal;
            }

            let sp = MitSetpoint {
                q: m.target,
                dq: 0.0,
                kp: args.kp,
                kd: args.kd,
                tau: 0.0,
            };
            let st = match m.dm.mit_control(&mut bus, &sp) {
                Ok(s) => s,
                Err(e) => {
                    eprintln!("[{}] {e}", m.label);
                    continue;
                }
            };

            if i % 100 == 0 {
                println!(
                    "  [{}] target={:+.3}  pos={:+.3} rad  vel={:+.3} rad/s  torque={:+.3} Nm",
                    m.label, m.target, st.position, st.velocity, st.torque
                );
            }

            if (m.target - m.goal).abs() < f32::EPSILON
                && (st.position - m.goal).abs() < args.tolerance
                && st.velocity.abs() < VEL_SETTLE
            {
                println!(
                    "  [{}] Reached zero position (pos={:+.4} rad)",
                    m.label, st.position
                );
                m.settled = true;
            }
        }

        // Hold settled motors at goal.
        for m in motors.iter().filter(|m| m.settled) {
            let sp = MitSetpoint {
                q: m.goal,
                dq: 0.0,
                kp: args.kp,
                kd: args.kd,
                tau: 0.0,
            };
            let _ = m.dm.mit_control(&mut bus, &sp);
        }

        if motors.iter().all(|m| m.settled) {
            all_settled = true;
            break;
        }
        thread::sleep(DT);
    }
    if !all_settled {
        eprintln!("Timed out before all actuators settled at zero.");
    }

    // Disable everything before returning.
    for m in &motors {
        if let Err(e) = m.dm.disable(&mut bus) {
            eprintln!("[{}] disable failed: {e}", m.label);
        }
    }
    println!("Disconnected.");
    Ok(())
}

fn main() {
    if let Err(e) = run() {
        eprintln!("Error: {e}");
        process::exit(1);
    }
}
