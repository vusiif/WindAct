//! WindAct CLI and execution entry point.

use std::str::FromStr;
use clap::{Parser, Subcommand};
use windact::core::{ActionResult, MouseButton, Point, WindActError};
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

/// Retrieves the current system cursor position if available.
#[cfg(windows)]
fn get_current_cursor_pos() -> Option<Point> {
    use windows_sys::Win32::Foundation::POINT;
    use windows_sys::Win32::UI::WindowsAndMessaging::GetCursorPos;
    unsafe {
        let mut pt = POINT { x: 0, y: 0 };
        if GetCursorPos(&mut pt) != 0 {
            Some(Point::new(pt.x as f64, pt.y as f64))
        } else {
            None
        }
    }
}

#[cfg(not(windows))]
fn get_current_cursor_pos() -> Option<Point> {
    None
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

    let planner = WindMousePlanner::new();

    match command {
        Commands::Click {
            x,
            y,
            start,
            button: _,
            profile,
            dry_run,
        } => {
            let start_point = match start {
                Some(s) => parse_point(&s)?,
                None => get_current_cursor_pos().unwrap_or(Point::new(0.0, 0.0)),
            };
            let target_point = Point::new(x, y);
            let profile_cfg = Profile::from_name(&profile);

            let trajectory = planner.generate(start_point, target_point, &profile_cfg)?;
            let duration_ms = trajectory.last().map(|p| p.elapsed_ms).unwrap_or(0);

            let result = ActionResult {
                id: None,
                status: "ok".to_string(),
                total_points: trajectory.len(),
                duration_ms,
                points: if dry_run { Some(trajectory) } else { None },
            };

            let json_output = serde_json::to_string_pretty(&result)?;
            println!("{json_output}");

            if !dry_run {
                eprintln!("\n[Notice] OS SendInput injection will be enabled in Phase 2. Ran dry-run calculation.");
            }
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

            let result = ActionResult {
                id: None,
                status: "ok".to_string(),
                total_points: trajectory.len(),
                duration_ms,
                points: if dry_run { Some(trajectory) } else { None },
            };

            let json_output = serde_json::to_string_pretty(&result)?;
            println!("{json_output}");

            if !dry_run {
                eprintln!("\n[Notice] OS SendInput injection will be enabled in Phase 2. Ran dry-run calculation.");
            }
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
                None => get_current_cursor_pos().unwrap_or(Point::new(0.0, 0.0)),
            };
            let target_point = Point::new(x, y);
            let profile_cfg = Profile::from_name(&profile);

            let trajectory = planner.generate(start_point, target_point, &profile_cfg)?;
            let duration_ms = trajectory.last().map(|p| p.elapsed_ms).unwrap_or(0);

            let result = ActionResult {
                id: None,
                status: "ok".to_string(),
                total_points: trajectory.len(),
                duration_ms,
                points: if dry_run { Some(trajectory) } else { None },
            };

            let json_output = serde_json::to_string_pretty(&result)?;
            println!("{json_output}");

            if !dry_run {
                eprintln!("\n[Notice] OS SendInput injection will be enabled in Phase 2. Ran dry-run calculation.");
            }
        }
    }

    Ok(())
}
