//! WindAct CLI and execution entry point.

use std::str::FromStr;
use std::time::Duration;
use clap::{Parser, Subcommand};
use rand::Rng;

use windact::core::{ActionResult, BoundingBox, MouseButton, Point, TrajectoryPoint, WindActError};
use windact::injector::{default_injector, hybrid_sleep, InputInjector};
use windact::planner::{TrajectoryPlanner, WindMousePlanner};
use windact::profile::Profile;

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

        /// Output calculated trajectory JSON without injecting OS physical input.
        #[arg(long, default_value_t = false)]
        dry_run: bool,
    },

    /// Generate or execute a humanized drag-and-drop action.
    Drag {
        /// Start coordinate formatted as "x,y".
        #[arg(long)]
        from: String,

        /// Destination coordinate formatted as "x,y".
        #[arg(long)]
        to: String,

        /// Movement profile (casual, precise, slider, fast).
        #[arg(long, default_value = "slider")]
        profile: String,

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

        /// Output calculated trajectory JSON without injecting OS physical input.
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

fn resolve_start_point(start: Option<String>, injector: &dyn InputInjector) -> Result<Point, WindActError> {
    match start {
        Some(s) => parse_point(&s),
        None => Ok(injector.current_cursor_pos().unwrap_or(Point::new(0.0, 0.0))),
    }
}

fn emit_result(
    trajectory: Vec<TrajectoryPoint>,
    extra_duration_ms: u64,
    dry_run: bool,
) -> Result<(), Box<dyn std::error::Error>> {
    let move_duration = trajectory.last().map(|p| p.elapsed_ms).unwrap_or(0);
    let result = ActionResult {
        id: None,
        status: "ok".to_string(),
        total_points: trajectory.len(),
        duration_ms: move_duration + extra_duration_ms,
        points: if dry_run { Some(trajectory) } else { None },
    };

    let json_output = serde_json::to_string_pretty(&result)?;
    println!("{json_output}");
    Ok(())
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let cli = Cli::parse();

    if cli.stdio {
        eprintln!("Stdio JSONL mode will be fully operational in Phase 3.");
        return Ok(());
    }

    let Some(command) = cli.command else {
        eprintln!("No subcommand provided. Use 'windact --help' for usage.");
        return Ok(());
    };

    let injector = default_injector();
    let planner = WindMousePlanner::new();
    let mut rng = rand::thread_rng();

    match command {
        Commands::Click {
            x,
            y,
            bbox,
            start,
            button,
            profile,
            dry_run,
        } => {
            let start_point = resolve_start_point(start, &injector)?;
            let target_point = match (bbox, x, y) {
                (Some(b), _, _) => WindMousePlanner::sample_gaussian_point(&parse_bbox(&b)?)?,
                (None, Some(px), Some(py)) => Point::new(px, py),
                _ => {
                    return Err(Box::new(WindActError::InvalidInput(
                        "Click requires either --x and --y, or --bbox".into(),
                    )));
                }
            };
            let profile_cfg = Profile::from_name(&profile)?;
            let trajectory = planner.generate(start_point, target_point, &profile_cfg)?;

            let hold_ms = rng.gen_range(profile_cfg.press_hold_min_ms..=profile_cfg.press_hold_max_ms);
            if !dry_run {
                injector.play_trajectory(&trajectory)?;
                injector.click_button(button, hold_ms)?;
            }

            emit_result(trajectory, hold_ms as u64, dry_run)?;
        }

        Commands::Drag {
            from,
            to,
            profile,
            dry_run,
        } => {
            let start_point = parse_point(&from)?;
            let target_point = parse_point(&to)?;
            let profile_cfg = Profile::from_name(&profile)?;

            // Plan approach trajectory from current cursor position to `from` to avoid teleportation
            let mut approach_trajectory = Vec::new();
            if !dry_run {
                let current_pos = injector.current_cursor_pos().unwrap_or(start_point);
                if current_pos.distance_to(&start_point) > 3.0 {
                    approach_trajectory = planner.generate(current_pos, start_point, &Profile::casual())?;
                }
            }

            let drag_trajectory = planner.generate(start_point, target_point, &profile_cfg)?;
            let pre_drag_hold = rng.gen_range(20..=50);
            let hover_ms = rng.gen_range(profile_cfg.hover_min_ms..=profile_cfg.hover_max_ms);

            if !dry_run {
                if !approach_trajectory.is_empty() {
                    injector.play_trajectory(&approach_trajectory)?;
                } else {
                    let (sx, sy) = start_point.to_i32();
                    injector.move_cursor(sx, sy)?;
                }
                hybrid_sleep(Duration::from_millis(30));

                injector.press_button(MouseButton::Left)?;
                hybrid_sleep(Duration::from_millis(pre_drag_hold));

                injector.play_trajectory(&drag_trajectory)?;

                // Mandatory dwell / hover at destination before releasing (anti-rebound for sliders)
                hybrid_sleep(Duration::from_millis(hover_ms as u64));
                injector.release_button(MouseButton::Left)?;
            }

            let approach_ms = approach_trajectory.last().map(|p| p.elapsed_ms).unwrap_or(0);
            let extra_ms = approach_ms + 30 + pre_drag_hold + hover_ms as u64;
            emit_result(drag_trajectory, extra_ms, dry_run)?;
        }

        Commands::Move {
            x,
            y,
            start,
            profile,
            dry_run,
        } => {
            let start_point = resolve_start_point(start, &injector)?;
            let target_point = Point::new(x, y);
            let profile_cfg = Profile::from_name(&profile)?;
            let trajectory = planner.generate(start_point, target_point, &profile_cfg)?;

            if !dry_run {
                injector.play_trajectory(&trajectory)?;
            }

            emit_result(trajectory, 0, dry_run)?;
        }
    }

    Ok(())
}
