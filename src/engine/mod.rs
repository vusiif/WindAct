//! Execution coordinator bridging trajectory planning and OS input injection.

use std::sync::atomic::AtomicBool;
use std::time::Duration;
use rand::Rng;

use crate::core::{
    ActionResult, MouseButton, Point, Result, ScrollAxis, TrajectoryPoint, WindActError,
};
use crate::injector::{hybrid_sleep_cancellable, InputInjector};
use crate::planner::{StrokeConfig, StrokePlanner, TrajectoryPlanner};
use crate::profile::Profile;

/// Coordinates trajectory generation and cancellable OS input execution.
pub struct ActionEngine<P: TrajectoryPlanner, I: InputInjector> {
    planner: P,
    injector: I,
}

impl<P: TrajectoryPlanner, I: InputInjector> ActionEngine<P, I> {
    pub fn new(planner: P, injector: I) -> Self {
        Self { planner, injector }
    }

    pub fn injector(&self) -> &I {
        &self.injector
    }

    pub fn planner(&self) -> &P {
        &self.planner
    }

    /// Resolves start coordinate, falling back to current cursor position or (0, 0).
    pub fn resolve_start(&self, explicit_start: Option<Point>) -> Point {
        explicit_start.unwrap_or_else(|| {
            self.injector
                .current_cursor_pos()
                .unwrap_or(Point::new(0.0, 0.0))
        })
    }

    /// Moves cursor along a humanized trajectory to `target`.
    pub fn execute_move(
        &self,
        start: Option<Point>,
        target: Point,
        profile: &Profile,
        dry_run: bool,
        cancel: Option<&AtomicBool>,
    ) -> Result<ActionResult> {
        self.execute_move_with(&self.planner, start, target, profile, dry_run, cancel)
    }

    /// Moves cursor using an explicit planner implementation.
    pub fn execute_move_with(
        &self,
        planner: &dyn TrajectoryPlanner,
        start: Option<Point>,
        target: Point,
        profile: &Profile,
        dry_run: bool,
        cancel: Option<&AtomicBool>,
    ) -> Result<ActionResult> {
        let start_pt = self.resolve_start(start);
        let trajectory = planner.generate(start_pt, target, profile)?;
        let duration_ms = trajectory.last().map(|p| p.elapsed_ms).unwrap_or(0);

        if !dry_run {
            self.injector.play_trajectory_cancellable(&trajectory, cancel)?;
        }

        Ok(Self::build_ok_result(trajectory, duration_ms, dry_run))
    }

    /// Moves cursor along a humanized trajectory to `target` and clicks `button`.
    pub fn execute_click(
        &self,
        start: Option<Point>,
        target: Point,
        button: MouseButton,
        profile: &Profile,
        dry_run: bool,
        cancel: Option<&AtomicBool>,
    ) -> Result<ActionResult> {
        self.execute_click_with(&self.planner, start, target, button, profile, dry_run, cancel)
    }

    /// Clicks using an explicit planner implementation.
    #[allow(clippy::too_many_arguments)]
    pub fn execute_click_with(
        &self,
        planner: &dyn TrajectoryPlanner,
        start: Option<Point>,
        target: Point,
        button: MouseButton,
        profile: &Profile,
        dry_run: bool,
        cancel: Option<&AtomicBool>,
    ) -> Result<ActionResult> {
        let start_pt = self.resolve_start(start);
        let trajectory = planner.generate(start_pt, target, profile)?;
        let move_ms = trajectory.last().map(|p| p.elapsed_ms).unwrap_or(0);

        let mut rng = rand::thread_rng();
        let hold_ms = rng.gen_range(profile.press_hold_min_ms..=profile.press_hold_max_ms);

        if !dry_run {
            self.injector.play_trajectory_cancellable(&trajectory, cancel)?;
            self.injector.click_button_cancellable(button, hold_ms, cancel)?;
        }

        Ok(Self::build_ok_result(trajectory, move_ms + hold_ms as u64, dry_run))
    }

    /// Approaches `from` smoothly, holds left button, drags to `to`, hovers, and releases.
    pub fn execute_drag(
        &self,
        from: Point,
        to: Point,
        profile: &Profile,
        dry_run: bool,
        cancel: Option<&AtomicBool>,
    ) -> Result<ActionResult> {
        self.execute_drag_path_with(&self.planner, &[from, to], profile, dry_run, cancel)
    }

    /// Executes a continuous multi-waypoint drag using an explicit planner.
    /// Guarantees left button release even if aborted mid-drag.
    pub fn execute_drag_path_with(
        &self,
        planner: &dyn TrajectoryPlanner,
        waypoints: &[Point],
        profile: &Profile,
        dry_run: bool,
        cancel: Option<&AtomicBool>,
    ) -> Result<ActionResult> {
        if waypoints.len() < 2 {
            return Err(WindActError::InvalidInput(
                "Drag path requires at least 2 waypoints".into(),
            ));
        }

        let from = waypoints[0];
        let mut approach_trajectory = Vec::new();
        if !dry_run {
            let current_pos = self.injector.current_cursor_pos().unwrap_or(from);
            if current_pos.distance_to(&from) > 3.0 {
                approach_trajectory = planner.generate(current_pos, from, &Profile::casual())?;
            }
        }

        let drag_trajectory = planner.generate_path(waypoints, profile)?;
        let mut rng = rand::thread_rng();
        let pre_drag_hold = rng.gen_range(20..=50);
        let hover_ms = rng.gen_range(profile.hover_min_ms..=profile.hover_max_ms);

        if !dry_run {
            if !approach_trajectory.is_empty() {
                self.injector
                    .play_trajectory_cancellable(&approach_trajectory, cancel)?;
            } else {
                let (fx, fy) = from.to_i32();
                self.injector.move_cursor(fx, fy)?;
            }

            if !hybrid_sleep_cancellable(Duration::from_millis(30), cancel) {
                return Err(WindActError::Aborted);
            }

            self.injector.press_button(MouseButton::Left)?;

            // Ensure left button is ALWAYS released on any early exit or abort
            let drag_res = (|| -> Result<()> {
                if !hybrid_sleep_cancellable(Duration::from_millis(pre_drag_hold), cancel) {
                    return Err(WindActError::Aborted);
                }
                self.injector
                    .play_trajectory_cancellable(&drag_trajectory, cancel)?;
                if !hybrid_sleep_cancellable(Duration::from_millis(hover_ms as u64), cancel) {
                    return Err(WindActError::Aborted);
                }
                Ok(())
            })();

            let release_res = self.injector.release_button(MouseButton::Left);
            drag_res?;
            release_res?;
        }

        let approach_ms = approach_trajectory.last().map(|p| p.elapsed_ms).unwrap_or(0);
        let drag_ms = drag_trajectory.last().map(|p| p.elapsed_ms).unwrap_or(0);
        let total_ms = approach_ms + 30 + pre_drag_hold + drag_ms + hover_ms as u64;

        Ok(Self::build_ok_result(drag_trajectory, total_ms, dry_run))
    }

    /// Executes humanized multi-step wheel scrolling with natural inter-step pauses.
    pub fn execute_scroll(
        &self,
        delta: i32,
        axis: ScrollAxis,
        steps: u32,
        dry_run: bool,
        cancel: Option<&AtomicBool>,
    ) -> Result<ActionResult> {
        if delta == 0 {
            return Err(WindActError::InvalidInput("scroll delta cannot be zero".into()));
        }
        let step_count = steps.max(1).min(delta.unsigned_abs().max(1));
        let base_step = delta / step_count as i32;
        let remainder = delta % step_count as i32;

        let mut rng = rand::thread_rng();
        let mut total_ms = 0u64;

        for i in 0..step_count {
            let notch = base_step + if (i as i32) < remainder.abs() { remainder.signum() } else { 0 };
            if notch != 0 && !dry_run {
                self.injector.scroll(notch, axis)?;
            }
            if i + 1 < step_count {
                let pause_ms = rng.gen_range(15..=45);
                total_ms += pause_ms;
                if !dry_run && !hybrid_sleep_cancellable(Duration::from_millis(pause_ms), cancel) {
                    return Err(WindActError::Aborted);
                }
            }
        }

        Ok(ActionResult {
            id: None,
            status: "ok".to_string(),
            total_points: Some(step_count as usize),
            duration_ms: total_ms,
            points: None,
            error: None,
        })
    }

    /// Executes a sequence of handwriting strokes with full physical lifecycle:
    /// - Airborne WindMouse transitions between strokes (pen lifted)
    /// - Landing dwell (15~35ms) after LeftDown before moving
    /// - Two-Thirds Power Law curvature-velocity coupling + terminal flick overshoot
    /// - Guaranteed LeftUp release even on mid-stroke abort
    pub fn execute_draw_strokes(
        &self,
        strokes: &[Vec<Point>],
        cfg: &StrokeConfig,
        dry_run: bool,
        cancel: Option<&AtomicBool>,
    ) -> Result<ActionResult> {
        if strokes.is_empty() {
            return Err(WindActError::InvalidInput("strokes cannot be empty".into()));
        }

        let stroke_planner = StrokePlanner::new();
        let mut rng = rand::thread_rng();
        let mut total_ms = 0u64;
        let mut all_points = Vec::new();

        let mut cursor_pos = if !dry_run {
            self.injector.current_cursor_pos().unwrap_or(strokes[0][0])
        } else {
            strokes[0][0]
        };

        for (idx, waypoints) in strokes.iter().enumerate() {
            if cancel.is_some_and(|f| f.load(std::sync::atomic::Ordering::Relaxed)) {
                return Err(WindActError::Aborted);
            }

            let stroke_start = waypoints
                .first()
                .copied()
                .ok_or_else(|| WindActError::InvalidInput("Stroke cannot be empty".into()))?;

            // 1. Airborne transition between strokes using WindMouse
            if !dry_run && (idx > 0 || cursor_pos.distance_to(&stroke_start) > 3.0) {
                let airborne = self.planner.generate(cursor_pos, stroke_start, &Profile::fast())?;
                total_ms += airborne.last().map(|p| p.elapsed_ms).unwrap_or(0);
                self.injector.play_trajectory_cancellable(&airborne, cancel)?;
            } else if !dry_run {
                let (sx, sy) = stroke_start.to_i32();
                self.injector.move_cursor(sx, sy)?;
            }

            // 2. Plan stroke geometry + kinematics
            let stroke_pts = stroke_planner.generate_stroke(waypoints, cfg)?;
            let stroke_ms = stroke_pts.last().map(|p| p.elapsed_ms).unwrap_or(0);
            let dwell_ms = rng.gen_range(cfg.landing_dwell_min_ms..=cfg.landing_dwell_max_ms);
            total_ms += dwell_ms as u64 + stroke_ms;

            if dry_run {
                all_points.extend(stroke_pts);
            } else {
                // 3. Pen down + landing dwell + stroke playback + guaranteed pen up
                self.injector.press_button(MouseButton::Left)?;
                let play_res = (|| -> Result<()> {
                    if !hybrid_sleep_cancellable(Duration::from_millis(dwell_ms as u64), cancel) {
                        return Err(WindActError::Aborted);
                    }
                    self.injector.play_trajectory_cancellable(&stroke_pts, cancel)?;
                    Ok(())
                })();
                let release_res = self.injector.release_button(MouseButton::Left);
                play_res?;
                release_res?;

                if let Some(last) = stroke_pts.last() {
                    cursor_pos = Point::new(last.x as f64, last.y as f64);
                }
            }
        }

        Ok(ActionResult {
            id: None,
            status: "ok".to_string(),
            total_points: Some(all_points.len()),
            duration_ms: total_ms,
            points: if dry_run { Some(all_points) } else { None },
            error: None,
        })
    }

    fn build_ok_result(
        trajectory: Vec<TrajectoryPoint>,
        duration_ms: u64,
        dry_run: bool,
    ) -> ActionResult {
        ActionResult {
            id: None,
            status: "ok".to_string(),
            total_points: Some(trajectory.len()),
            duration_ms,
            points: if dry_run { Some(trajectory) } else { None },
            error: None,
        }
    }
}
