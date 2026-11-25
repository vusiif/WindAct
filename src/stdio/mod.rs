//! Persistent `--stdio` JSONL pipeline protocol with millisecond-level abort support.

use std::io::{self, BufRead, Write};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{mpsc, Arc};
use std::thread;
use std::time::Instant;

use serde::Deserialize;

use crate::core::{ActionResult, BoundingBox, MouseButton, Point, Result, ScrollAxis, WindActError};
use crate::engine::ActionEngine;
use crate::injector::InputInjector;
use crate::planner::{BezierSplinePlanner, TrajectoryPlanner, WindMousePlanner};
use crate::profile::Profile;

fn default_casual() -> String {
    "casual".to_string()
}

fn default_slider() -> String {
    "slider".to_string()
}

fn default_scroll_steps() -> u32 {
    3
}

/// Incoming JSONL command envelope.
#[derive(Debug, Deserialize)]
pub struct StdioCommand {
    pub id: u64,
    #[serde(flatten)]
    pub op: StdioOp,
}

/// Supported atomic operations over the `--stdio` JSONL stream.
#[derive(Debug, Deserialize)]
#[serde(tag = "op", rename_all = "snake_case")]
pub enum StdioOp {
    MoveClick {
        x: Option<f64>,
        y: Option<f64>,
        bbox: Option<[f64; 4]>,
        start: Option<[f64; 2]>,
        #[serde(default)]
        button: MouseButton,
        #[serde(default = "default_casual")]
        profile: String,
        #[serde(default)]
        planner: Option<String>,
        #[serde(default)]
        dry_run: bool,
    },
    Move {
        x: f64,
        y: f64,
        start: Option<[f64; 2]>,
        #[serde(default = "default_casual")]
        profile: String,
        #[serde(default)]
        planner: Option<String>,
        #[serde(default)]
        dry_run: bool,
    },
    Drag {
        from: Option<[f64; 2]>,
        to: Option<[f64; 2]>,
        path: Option<Vec<[f64; 2]>>,
        #[serde(default = "default_slider")]
        profile: String,
        #[serde(default)]
        planner: Option<String>,
        #[serde(default)]
        dry_run: bool,
    },
    Scroll {
        delta: i32,
        #[serde(default)]
        axis: ScrollAxis,
        #[serde(default = "default_scroll_steps")]
        steps: u32,
        #[serde(default)]
        dry_run: bool,
    },
    Abort,
}

fn select_planner<'a, P: TrajectoryPlanner>(
    name: Option<&str>,
    default_planner: &'a P,
    bezier: &'a BezierSplinePlanner,
) -> Result<&'a dyn TrajectoryPlanner> {
    match name.map(|s| s.to_lowercase()).as_deref() {
        None | Some("default") => Ok(default_planner),
        Some("windmouse") => Ok(default_planner),
        Some("bezier") => Ok(bezier),
        Some(other) => Err(WindActError::InvalidInput(format!(
            "Unknown planner '{other}'. Expected 'windmouse' or 'bezier'"
        ))),
    }
}

/// Dispatches a single parsed command (excluding `Abort`) against the `ActionEngine`.
pub fn dispatch_command<P: TrajectoryPlanner, I: InputInjector>(
    engine: &ActionEngine<P, I>,
    cmd: StdioCommand,
    cancel: Option<&AtomicBool>,
) -> ActionResult {
    let start_time = Instant::now();
    let id = cmd.id;
    let bezier = BezierSplinePlanner::new();

    let res: Result<ActionResult> = match cmd.op {
        StdioOp::MoveClick {
            x,
            y,
            bbox,
            start,
            button,
            profile,
            planner,
            dry_run,
        } => (|| {
            let profile_cfg = Profile::from_name(&profile)?;
            let selected = select_planner(planner.as_deref(), engine.planner(), &bezier)?;
            let target = match (bbox, x, y) {
                (Some(b), _, _) => WindMousePlanner::sample_gaussian_point(&BoundingBox::new(
                    b[0], b[1], b[2], b[3],
                ))?,
                (None, Some(px), Some(py)) => Point::new(px, py),
                _ => {
                    return Err(WindActError::InvalidInput(
                        "move_click requires either x/y or bbox".into(),
                    ));
                }
            };
            let start_pt = start.map(|s| Point::new(s[0], s[1]));
            engine.execute_click_with(selected, start_pt, target, button, &profile_cfg, dry_run, cancel)
        })(),

        StdioOp::Move {
            x,
            y,
            start,
            profile,
            planner,
            dry_run,
        } => (|| {
            let profile_cfg = Profile::from_name(&profile)?;
            let selected = select_planner(planner.as_deref(), engine.planner(), &bezier)?;
            let start_pt = start.map(|s| Point::new(s[0], s[1]));
            engine.execute_move_with(selected, start_pt, Point::new(x, y), &profile_cfg, dry_run, cancel)
        })(),

        StdioOp::Drag {
            from,
            to,
            path,
            profile,
            planner,
            dry_run,
        } => (|| {
            let profile_cfg = Profile::from_name(&profile)?;
            let waypoints: Vec<Point> = match (path, from, to) {
                (Some(pts), _, _) => pts.into_iter().map(|p| Point::new(p[0], p[1])).collect(),
                (None, Some(f), Some(t)) => vec![Point::new(f[0], f[1]), Point::new(t[0], t[1])],
                _ => {
                    return Err(WindActError::InvalidInput(
                        "drag requires either 'path' or both 'from' and 'to'".into(),
                    ));
                }
            };
            // Default to Bezier spline automatically for multi-waypoint curves unless overridden
            let selected: &dyn TrajectoryPlanner = if planner.is_none() && waypoints.len() > 2 {
                &bezier
            } else {
                select_planner(planner.as_deref(), engine.planner(), &bezier)?
            };
            engine.execute_drag_path_with(selected, &waypoints, &profile_cfg, dry_run, cancel)
        })(),

        StdioOp::Scroll {
            delta,
            axis,
            steps,
            dry_run,
        } => engine.execute_scroll(delta, axis, steps, dry_run, cancel),

        StdioOp::Abort => Ok(ActionResult {
            id: Some(id),
            status: "ok".to_string(),
            total_points: None,
            duration_ms: 0,
            points: None,
            error: None,
        }),
    };

    match res {
        Ok(mut action_res) => {
            action_res.id = Some(id);
            action_res
        }
        Err(WindActError::Aborted) => ActionResult {
            id: Some(id),
            status: "aborted".to_string(),
            total_points: None,
            duration_ms: start_time.elapsed().as_millis() as u64,
            points: None,
            error: Some("Action aborted by user".to_string()),
        },
        Err(e) => ActionResult {
            id: Some(id),
            status: "error".to_string(),
            total_points: None,
            duration_ms: start_time.elapsed().as_millis() as u64,
            points: None,
            error: Some(e.to_string()),
        },
    }
}

fn write_jsonl_line(out: &Arc<std::sync::Mutex<io::Stdout>>, res: &ActionResult) {
    if let Ok(line) = serde_json::to_string(res)
        && let Ok(mut guard) = out.lock()
    {
        let _ = writeln!(guard, "{line}");
        let _ = guard.flush();
    }
}

/// Runs the blocking `--stdio` JSONL loop using a reader/worker dual-thread model
/// so `abort` commands are processed immediately without waiting for active actions.
pub fn run_stdio_loop<P, I>(engine: ActionEngine<P, I>) -> io::Result<()>
where
    P: TrajectoryPlanner + 'static,
    I: InputInjector + 'static,
{
    let cancel_flag = Arc::new(AtomicBool::new(false));
    let stdout = Arc::new(std::sync::Mutex::new(io::stdout()));
    let (tx, rx) = mpsc::channel::<StdioCommand>();

    let worker_cancel = Arc::clone(&cancel_flag);
    let worker_out = Arc::clone(&stdout);

    let worker = thread::spawn(move || {
        while let Ok(cmd) = rx.recv() {
            // Reset cancel token before starting each new command
            worker_cancel.store(false, Ordering::SeqCst);
            let res = dispatch_command(&engine, cmd, Some(&worker_cancel));
            write_jsonl_line(&worker_out, &res);
        }
    });

    let stdin = io::stdin();
    for line_res in stdin.lock().lines() {
        let line = line_res?;
        let trimmed = line.trim();
        if trimmed.is_empty() {
            continue;
        }

        match serde_json::from_str::<StdioCommand>(trimmed) {
            Ok(cmd) => {
                if matches!(cmd.op, StdioOp::Abort) {
                    // Signal cancellation immediately on reader thread
                    cancel_flag.store(true, Ordering::SeqCst);
                    let ack = ActionResult {
                        id: Some(cmd.id),
                        status: "ok".to_string(),
                        total_points: None,
                        duration_ms: 0,
                        points: None,
                        error: None,
                    };
                    write_jsonl_line(&stdout, &ack);
                } else if tx.send(cmd).is_err() {
                    break;
                }
            }
            Err(e) => {
                let err_res = ActionResult {
                    id: None,
                    status: "error".to_string(),
                    total_points: None,
                    duration_ms: 0,
                    points: None,
                    error: Some(format!("Invalid JSONL command: {e}")),
                };
                write_jsonl_line(&stdout, &err_res);
            }
        }
    }

    // Signal worker to abort any in-flight action when stdin closes
    cancel_flag.store(true, Ordering::SeqCst);
    drop(tx);
    let _ = worker.join();
    Ok(())
}
