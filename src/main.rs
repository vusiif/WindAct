//! WindAct CLI and execution entry point.

use std::str::FromStr;
use std::time::Duration;
use clap::{Parser, Subcommand};
use rand::Rng;

use windact::core::{ActionResult, MouseButton, Point, WindActError};
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
        /// Target X coordinate.
        #[arg(long)]
        x: f64,

        /// Target Y coordinate.
        #[arg(long)]
        y: f64,

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

/// Helper function to parse "x,y" string into Point.
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
            start,
            button,
            profile,
            dry_run,
        } => {
            let start_point = match start {
                Some(s) => parse_point(&s)?,
                None => injector.current_cursor_pos().unwrap_or(Point::new(0.0, 0.0)),
            };
            let target_point = Point::new(x, y);
            let profile_cfg = Profile::from_name(&profile);

            let trajectory = planner.generate(start_point, target_point, &profile_cfg)?;
            let duration_ms = trajectory.last().map(|p| p.elapsed_ms).unwrap_or(0);

            if !dry_run {
                injector.play_trajectory(&trajectory)?;
                let hold_ms = rng.gen_range(profile_cfg.press_hold_min_ms..=profile_cfg.press_hold_max_ms);
                injector.click_button(button, hold_ms)?;
            }

            let result = ActionResult {
                id: None,
                status: "ok".to_string(),
                total_points: trajectory.len(),
                duration_ms,
                points: if dry_run { Some(trajectory) } else { None },
            };

            let json_output = serde_json::to_string_pretty(&result)?;
            println!("{json_output}");
        }

        Commands::Drag {
            from,
            to,
            profile,
            dry_run,
        } => {
            let start_point = parse_point(&from)?;
            let target_point = parse_point(&to)?;
            let profile_cfg = Profile::from_name(&profile);

            let trajectory = planner.generate(start_point, target_point, &profile_cfg)?;
            let duration_ms = trajectory.last().map(|p| p.elapsed_ms).unwrap_or(0);

            if !dry_run {
                // Move cursor to start point
                injector.move_cursor(start_point.x as i32, start_point.y as i32)?;
                hybrid_sleep(Duration::from_millis(30));

                // Press left button
                injector.press_button(MouseButton::Left)?;
                let pre_drag_hold = rng.gen_range(20..=50);
                hybrid_sleep(Duration::from_millis(pre_drag_hold));

                // Play trajectory with button held
                injector.play_trajectory(&trajectory)?;

                // Mandatory dwell / hover at destination before releasing (anti-rebound for sliders)
                let hover_ms = rng.gen_range(profile_cfg.hover_min_ms..=profile_cfg.hover_max_ms);
                hybrid_sleep(Duration::from_millis(hover_ms as u64));

                // Release button
                injector.release_button(MouseButton::Left)?;
            }

            let result = ActionResult {
                id: None,
                status: "ok".to_string(),
                total_points: trajectory.len(),
                duration_ms,
                points: if dry_run { Some(trajectory) } else { None },
            };

            let json_output = serde_json::to_string_pretty(&result)?;
            println!("{json_output}");
        }

        Commands::Move {
            x,
            y,
            start,
            profile,
            dry_run,
        } => {
            let start_point = match start {
                Some(s) => parse_point(&s)?,
                None => injector.current_cursor_pos().unwrap_or(Point::new(0.0, 0.0)),
            };
            let target_point = Point::new(x, y);
            let profile_cfg = Profile::from_name(&profile);

            let trajectory = planner.generate(start_point, target_point, &profile_cfg)?;
            let duration_ms = trajectory.last().map(|p| p.elapsed_ms).unwrap_or(0);

            if !dry_run {
                injector.play_trajectory(&trajectory)?;
            }

            let result = ActionResult {
                id: None,
                status: "ok".to_string(),
                total_points: trajectory.len(),
                duration_ms,
                points: if dry_run { Some(trajectory) } else { None },
            };

            let json_output = serde_json::to_string_pretty(&result)?;
            println!("{json_output}");
        }
    }

    Ok(())
}
