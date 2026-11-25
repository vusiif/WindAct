//! WindAct CLI and execution entry point.

use std::str::FromStr;
use clap::{Parser, Subcommand};

use windact::core::{BoundingBox, MouseButton, Point, ScrollAxis, TypingConfig, WindActError};
use windact::engine::ActionEngine;
use windact::injector::default_injector;
use windact::planner::{BezierSplinePlanner, TrajectoryPlanner, WindMousePlanner};
use windact::profile::Profile;
use windact::stdio::run_stdio_loop;

/// WindAct: Physics-based human-like mouse trajectory generator & input injector.
#[derive(Parser, Debug)]
#[command(name = "windact", version, about, long_about = None)]
struct Cli {
    #[command(subcommand)]
    command: Option<Commands>,

    /// Run in long-lived JSONL stdio pipeline mode.
    #[arg(long)]
    stdio: bool,
}

#[derive(Subcommand, Debug)]
enum Commands {
    /// Generate or execute a humanized click action.
    Click {
        /// Target X coordinate (required unless --bbox is provided).
        #[arg(long)]
        x: Option<f64>,

        /// Target Y coordinate (required unless --bbox is provided).
        #[arg(long)]
        y: Option<f64>,

        /// Target bounding box formatted as "x,y,width,height" for 2D Gaussian sampling.
        #[arg(long)]
        bbox: Option<String>,

        /// Optional start coordinate formatted as "x,y". Defaults to current cursor position.
        #[arg(long)]
        start: Option<String>,

        /// Mouse button to click (left, right, middle).
        #[arg(long, default_value = "left")]
        button: MouseButton,

        /// Movement profile (casual, precise, slider, fast).
        #[arg(long, default_value = "casual")]
        profile: String,

        /// Trajectory planner algorithm (windmouse, bezier).
        #[arg(long, default_value = "windmouse")]
        planner: String,

        /// Output calculated trajectory JSON without injecting OS physical input.
        #[arg(long, default_value_t = false)]
        dry_run: bool,
    },

    /// Generate or execute a humanized drag-and-drop action.
    Drag {
        /// Start coordinate formatted as "x,y" (required unless --path is provided).
        #[arg(long)]
        from: Option<String>,

        /// Destination coordinate formatted as "x,y" (required unless --path is provided).
        #[arg(long)]
        to: Option<String>,

        /// Multi-waypoint path formatted as "x1,y1;x2,y2;x3,y3" for continuous curved drag.
        #[arg(long)]
        path: Option<String>,

        /// Movement profile (casual, precise, slider, fast).
        #[arg(long, default_value = "slider")]
        profile: String,

        /// Trajectory planner algorithm (windmouse, bezier). Defaults to bezier when --path is used.
        #[arg(long)]
        planner: Option<String>,

        /// Output calculated trajectory JSON without injecting OS physical input.
        #[arg(long, default_value_t = false)]
        dry_run: bool,
    },

    /// Generate or execute a humanized move action.
    Move {
        /// Target X coordinate.
        #[arg(long)]
        x: f64,

        /// Target Y coordinate.
        #[arg(long)]
        y: f64,

        /// Optional start coordinate formatted as "x,y". Defaults to current cursor position.
        #[arg(long)]
        start: Option<String>,

        /// Movement profile (casual, precise, slider, fast).
        #[arg(long, default_value = "casual")]
        profile: String,

        /// Trajectory planner algorithm (windmouse, bezier).
        #[arg(long, default_value = "windmouse")]
        planner: String,

        /// Output calculated trajectory JSON without injecting OS physical input.
        #[arg(long, default_value_t = false)]
        dry_run: bool,
    },

    /// Execute a humanized mouse wheel scroll action.
    Scroll {
        /// Wheel notches to scroll (positive = up/right, negative = down/left).
        #[arg(long, allow_hyphen_values = true)]
        delta: i32,

        /// Scroll axis (vertical, horizontal).
        #[arg(long, default_value = "vertical")]
        axis: ScrollAxis,

        /// Number of humanized scroll sub-steps.
        #[arg(long, default_value_t = 3)]
        steps: u32,

        /// Simulate timing without injecting OS wheel events.
        #[arg(long, default_value_t = false)]
        dry_run: bool,
    },

    /// Type text with humanized key cadence and Unicode support.
    SendText {
        /// Text string to type.
        #[arg(long)]
        text: String,

        /// Typing rhythm profile (casual, fast, slow).
        #[arg(long, default_value = "casual")]
        profile: String,

        /// Simulate timing calculation without injecting OS keyboard events.
        #[arg(long, default_value_t = false)]
        dry_run: bool,
    },

    /// Press a single key or modifier combo (e.g., "enter", "ctrl+a", "ctrl+shift+s").
    KeyPress {
        /// Key or combination string.
        #[arg(long)]
        key: String,

        /// Optional fixed hold duration in milliseconds.
        #[arg(long)]
        hold_ms: Option<u32>,

        /// Simulate timing calculation without injecting OS keyboard events.
        #[arg(long, default_value_t = false)]
        dry_run: bool,
    },
}

fn parse_point(s: &str) -> Result<Point, WindActError> {
    let parts: Vec<&str> = s.split(',').map(|p| p.trim()).collect();
    if parts.len() != 2 {
        return Err(WindActError::InvalidInput(format!(
            "Invalid coordinate format '{s}', expected 'x,y'"
        )));
    }
    let x = f64::from_str(parts[0])
        .map_err(|e| WindActError::InvalidInput(format!("Invalid x coordinate: {e}")))?;
    let y = f64::from_str(parts[1])
        .map_err(|e| WindActError::InvalidInput(format!("Invalid y coordinate: {e}")))?;
    Ok(Point::new(x, y))
}

fn parse_path(s: &str) -> Result<Vec<Point>, WindActError> {
    s.split(';')
        .map(|seg| seg.trim())
        .filter(|seg| !seg.is_empty())
        .map(parse_point)
        .collect()
}

fn parse_bbox(s: &str) -> Result<BoundingBox, WindActError> {
    let parts: Vec<&str> = s.split(',').map(|p| p.trim()).collect();
    if parts.len() != 4 {
        return Err(WindActError::InvalidInput(format!(
            "Invalid bbox format '{s}', expected 'x,y,width,height'"
        )));
    }
    let vals: Result<Vec<f64>, _> = parts.iter().map(|p| f64::from_str(p)).collect();
    let vals = vals.map_err(|e| WindActError::InvalidInput(format!("Invalid bbox value: {e}")))?;
    Ok(BoundingBox::new(vals[0], vals[1], vals[2], vals[3]))
}

fn select_planner<'a>(
    name: &str,
    windmouse: &'a WindMousePlanner,
    bezier: &'a BezierSplinePlanner,
) -> Result<&'a dyn TrajectoryPlanner, WindActError> {
    match name.to_lowercase().as_str() {
        "windmouse" => Ok(windmouse),
        "bezier" => Ok(bezier),
        other => Err(WindActError::InvalidInput(format!(
            "Unknown planner '{other}'. Expected 'windmouse' or 'bezier'"
        ))),
    }
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let cli = Cli::parse();
    let windmouse = WindMousePlanner::new();
    let bezier = BezierSplinePlanner::new();
    let engine = ActionEngine::new(WindMousePlanner::new(), default_injector());

    if cli.stdio {
        run_stdio_loop(engine)?;
        return Ok(());
    }

    let Some(command) = cli.command else {
        eprintln!("No subcommand provided. Use 'windact --help' for usage.");
        return Ok(());
    };

    let result = match command {
        Commands::Click {
            x,
            y,
            bbox,
            start,
            button,
            profile,
            planner,
            dry_run,
        } => {
            let start_pt = start.as_deref().map(parse_point).transpose()?;
            let target_pt = match (bbox, x, y) {
                (Some(b), _, _) => WindMousePlanner::sample_gaussian_point(&parse_bbox(&b)?)?,
                (None, Some(px), Some(py)) => Point::new(px, py),
                _ => {
                    return Err(Box::new(WindActError::InvalidInput(
                        "Click requires either --x and --y, or --bbox".into(),
                    )));
                }
            };
            let profile_cfg = Profile::from_name(&profile)?;
            let selected = select_planner(&planner, &windmouse, &bezier)?;
            engine.execute_click_with(selected, start_pt, target_pt, button, &profile_cfg, dry_run, None)?
        }

        Commands::Drag {
            from,
            to,
            path,
            profile,
            planner,
            dry_run,
        } => {
            let waypoints = match (path, from, to) {
                (Some(p), _, _) => parse_path(&p)?,
                (None, Some(f), Some(t)) => vec![parse_point(&f)?, parse_point(&t)?],
                _ => {
                    return Err(Box::new(WindActError::InvalidInput(
                        "Drag requires either --path or both --from and --to".into(),
                    )));
                }
            };
            let profile_cfg = Profile::from_name(&profile)?;
            let planner_name = planner
                .as_deref()
                .unwrap_or(if waypoints.len() > 2 { "bezier" } else { "windmouse" });
            let selected = select_planner(planner_name, &windmouse, &bezier)?;
            engine.execute_drag_path_with(selected, &waypoints, &profile_cfg, dry_run, None)?
        }

        Commands::Move {
            x,
            y,
            start,
            profile,
            planner,
            dry_run,
        } => {
            let start_pt = start.as_deref().map(parse_point).transpose()?;
            let target_pt = Point::new(x, y);
            let profile_cfg = Profile::from_name(&profile)?;
            let selected = select_planner(&planner, &windmouse, &bezier)?;
            engine.execute_move_with(selected, start_pt, target_pt, &profile_cfg, dry_run, None)?
        }

        Commands::Scroll {
            delta,
            axis,
            steps,
            dry_run,
        } => engine.execute_scroll(delta, axis, steps, dry_run, None)?,

        Commands::SendText {
            text,
            profile,
            dry_run,
        } => {
            let cfg = TypingConfig::from_name(&profile)?;
            engine.execute_send_text(&text, &cfg, dry_run, None)?
        }

        Commands::KeyPress {
            key,
            hold_ms,
            dry_run,
        } => engine.execute_key_press(&key, hold_ms, dry_run, None)?,
    };

    println!("{}", serde_json::to_string_pretty(&result)?);
    Ok(())
}
