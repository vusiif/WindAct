//! Integration tests for WindAct trajectory generation, engine, and stdio protocol.

use std::sync::atomic::{AtomicBool, AtomicI32, AtomicU32, Ordering};
use std::sync::Arc;
use std::thread;
use std::time::{Duration, Instant};

use windact::core::{BoundingBox, MouseButton, Point, Result, ScrollAxis, TrajectoryPoint, WindActError};
use windact::engine::ActionEngine;
use windact::injector::{hybrid_sleep_cancellable, InputInjector};
use windact::planner::{TrajectoryPlanner, WindMousePlanner};
use windact::profile::Profile;
use windact::stdio::{dispatch_command, StdioCommand};

/// In-memory test injector that records button state, moves, and scroll deltas without touching OS input.
#[derive(Default)]
struct TestInjector {
    press_count: AtomicU32,
    release_count: AtomicU32,
    scroll_sum: AtomicI32,
    last_x: AtomicI32,
    last_y: AtomicI32,
}

impl InputInjector for TestInjector {
    fn move_cursor(&self, x: i32, y: i32) -> Result<()> {
        self.last_x.store(x, Ordering::Relaxed);
        self.last_y.store(y, Ordering::Relaxed);
        Ok(())
    }

    fn press_button(&self, _button: MouseButton) -> Result<()> {
        self.press_count.fetch_add(1, Ordering::SeqCst);
        Ok(())
    }

    fn release_button(&self, _button: MouseButton) -> Result<()> {
        self.release_count.fetch_add(1, Ordering::SeqCst);
        Ok(())
    }

    fn click_button_cancellable(
        &self,
        button: MouseButton,
        hold_ms: u32,
        cancel: Option<&AtomicBool>,
    ) -> Result<()> {
        self.press_button(button)?;
        let ok = hybrid_sleep_cancellable(Duration::from_millis(hold_ms as u64), cancel);
        self.release_button(button)?;
        if ok { Ok(()) } else { Err(WindActError::Aborted) }
    }

    fn scroll(&self, delta: i32, _axis: ScrollAxis) -> Result<()> {
        self.scroll_sum.fetch_add(delta, Ordering::SeqCst);
        Ok(())
    }

    fn play_trajectory_cancellable(
        &self,
        points: &[TrajectoryPoint],
        cancel: Option<&AtomicBool>,
    ) -> Result<()> {
        for pt in points {
            if cancel.is_some_and(|f| f.load(Ordering::Relaxed)) {
                return Err(WindActError::Aborted);
            }
            if pt.delay_ms > 0
                && !hybrid_sleep_cancellable(Duration::from_millis(pt.delay_ms as u64), cancel)
            {
                return Err(WindActError::Aborted);
            }
            self.move_cursor(pt.x, pt.y)?;
        }
        Ok(())
    }

    fn current_cursor_pos(&self) -> Result<Point> {
        Ok(Point::new(
            self.last_x.load(Ordering::Relaxed) as f64,
            self.last_y.load(Ordering::Relaxed) as f64,
        ))
    }
}

#[test]
fn test_all_profiles_generate_valid_trajectories() {
    let planner = WindMousePlanner::new();
    let profiles = [
        Profile::casual(),
        Profile::precise(),
        Profile::slider(),
        Profile::fast(),
    ];

    let start = Point::new(150.0, 200.0);
    let target = Point::new(650.0, 480.0);

    for profile in profiles {
        let trajectory = planner
            .generate(start, target, &profile)
            .unwrap_or_else(|e| panic!("Profile {} failed: {e}", profile.name));

        assert!(
            trajectory.len() >= 10,
            "Trajectory for {} should have substantial steps",
            profile.name
        );

        let first = trajectory.first().unwrap();
        assert_eq!((first.x, first.y), (150, 200));

        let last = trajectory.last().unwrap();
        assert_eq!((last.x, last.y), (650, 480));

        for i in 1..trajectory.len() {
            assert!(
                trajectory[i].elapsed_ms >= trajectory[i - 1].elapsed_ms,
                "Timing must be monotonic"
            );
        }
    }
}

#[test]
fn test_slider_drag_smoothness() {
    let planner = WindMousePlanner::new();
    let profile = Profile::slider();
    let start = Point::new(200.0, 400.0);
    let target = Point::new(550.0, 402.0);

    let points = planner.generate(start, target, &profile).expect("Slider drag should plan");

    assert!(!points.is_empty());
    assert_eq!((points[0].x, points[0].y), (200, 400));
    assert_eq!((points.last().unwrap().x, points.last().unwrap().y), (550, 402));

    for i in 1..points.len() {
        let dx = (points[i].x - points[i - 1].x).abs();
        let dy = (points[i].y - points[i - 1].y).abs();
        let step_dist = ((dx * dx + dy * dy) as f64).sqrt();
        assert!(
            step_dist <= profile.max_step * 2.5,
            "Consecutive step jump {step_dist} too large at index {i}"
        );
    }
}

#[test]
fn test_gaussian_sampling_boundary_and_tiny_box() {
    let bbox = BoundingBox::new(500.0, 200.0, 80.0, 40.0);
    for _ in 0..200 {
        let pt = WindMousePlanner::sample_gaussian_point(&bbox).unwrap();
        assert!(pt.x >= bbox.x && pt.x <= bbox.x + bbox.width);
        assert!(pt.y >= bbox.y && pt.y <= bbox.y + bbox.height);
    }

    let tiny = BoundingBox::new(100.0, 100.0, 2.0, 1.5);
    let pt = WindMousePlanner::sample_gaussian_point(&tiny).unwrap();
    assert_eq!(pt, Point::new(101.0, 100.75));
}

#[test]
fn test_profile_validation_and_unknown_name() {
    assert!(Profile::from_name("unknown_profile").is_err());
    assert!(Profile::from_name("CASUAL").is_ok());

    let mut bad = Profile::casual();
    bad.min_delay_ms = 20;
    bad.max_delay_ms = 5;
    assert!(bad.validate().is_err());

    let planner = WindMousePlanner::new();
    assert!(planner.generate(Point::new(0.0, 0.0), Point::new(100.0, 100.0), &bad).is_err());
}

#[test]
fn test_extreme_wind_terminates_without_infinite_loop() {
    let planner = WindMousePlanner::new();
    let mut extreme = Profile::casual();
    extreme.gravity = 0.5;
    extreme.wind = 50.0;
    extreme.min_delay_ms = 5;
    extreme.max_delay_ms = 6;

    let points = planner
        .generate(Point::new(0.0, 0.0), Point::new(300.0, 300.0), &extreme)
        .expect("Extreme valid profile should terminate safely");
    assert_eq!(
        (points.last().unwrap().x, points.last().unwrap().y),
        (300, 300)
    );
}

#[test]
fn test_drag_abort_releases_mouse_button_safely() {
    let engine = Arc::new(ActionEngine::new(WindMousePlanner::new(), TestInjector::default()));
    let cancel = Arc::new(AtomicBool::new(false));

    let engine_worker = Arc::clone(&engine);
    let cancel_worker = Arc::clone(&cancel);

    let handle = thread::spawn(move || {
        // Long slow drag so abort definitely catches it mid-drag
        engine_worker.execute_drag(
            Point::new(0.0, 0.0),
            Point::new(1200.0, 800.0),
            &Profile::slider(),
            false,
            Some(&cancel_worker),
        )
    });

    // Wait briefly until drag presses button, then trigger abort
    thread::sleep(Duration::from_millis(60));
    let abort_start = Instant::now();
    cancel.store(true, Ordering::SeqCst);

    let res = handle.join().expect("Worker thread panicked");
    assert!(matches!(res, Err(WindActError::Aborted)), "Expected Aborted, got {res:?}");
    assert!(
        abort_start.elapsed() < Duration::from_millis(30),
        "Abort latency should be within a few milliseconds"
    );

    // Crucial invariant: every press must be balanced by a release
    let pressed = engine.injector().press_count.load(Ordering::SeqCst);
    let released = engine.injector().release_count.load(Ordering::SeqCst);
    assert_eq!(pressed, released, "Mouse button must never remain stuck pressed");
}

#[test]
fn test_stdio_dispatch_commands_and_scroll() {
    let engine = ActionEngine::new(WindMousePlanner::new(), TestInjector::default());

    let click_json = r#"{"id": 1, "op": "move_click", "x": 1464, "y": 326, "button": "left", "profile": "casual", "dry_run": true}"#;
    let cmd1: StdioCommand = serde_json::from_str(click_json).unwrap();
    let res1 = dispatch_command(&engine, cmd1, None);
    assert_eq!(res1.id, Some(1));
    assert_eq!(res1.status, "ok");
    assert!(res1.points.is_some());

    let drag_json = r#"{"id": 2, "op": "drag", "from": [500, 600], "to": [760, 602], "profile": "slider", "dry_run": true}"#;
    let cmd2: StdioCommand = serde_json::from_str(drag_json).unwrap();
    let res2 = dispatch_command(&engine, cmd2, None);
    assert_eq!(res2.id, Some(2));
    assert_eq!(res2.status, "ok");

    let scroll_json = r#"{"id": 3, "op": "scroll", "delta": -7, "axis": "vertical", "steps": 3}"#;
    let cmd3: StdioCommand = serde_json::from_str(scroll_json).unwrap();
    let res3 = dispatch_command(&engine, cmd3, None);
    assert_eq!(res3.id, Some(3));
    assert_eq!(res3.status, "ok");
    assert_eq!(engine.injector().scroll_sum.load(Ordering::SeqCst), -7);

    let path_drag_json = r#"{"id": 4, "op": "drag", "path": [[100,100],[250,105],[220,240]], "profile": "casual", "dry_run": true}"#;
    let cmd4: StdioCommand = serde_json::from_str(path_drag_json).unwrap();
    let res4 = dispatch_command(&engine, cmd4, None);
    assert_eq!(res4.id, Some(4));
    assert_eq!(res4.status, "ok");
    let pts = res4.points.expect("dry_run should include points");
    assert_eq!((pts.first().unwrap().x, pts.first().unwrap().y), (100, 100));
    assert_eq!((pts.last().unwrap().x, pts.last().unwrap().y), (220, 240));
}

#[test]
fn test_bezier_spline_planner_curves_and_waypoints() {
    use windact::planner::BezierSplinePlanner;

    let planner = BezierSplinePlanner::new();
    let profile = Profile::casual();
    let waypoints = [
        Point::new(100.0, 100.0),
        Point::new(220.0, 100.0),
        Point::new(190.0, 220.0),
    ];

    let points = planner.generate_path(&waypoints, &profile).unwrap();
    assert!(points.len() >= 20, "Spline path should have dense smooth points");
    assert_eq!((points[0].x, points[0].y), (100, 100));
    assert_eq!((points.last().unwrap().x, points.last().unwrap().y), (190, 220));

    for i in 1..points.len() {
        assert!(points[i].elapsed_ms >= points[i - 1].elapsed_ms);
    }
}
