//! Integration tests for WindAct trajectory generation and profiles.

use windact::core::{BoundingBox, Point};
use windact::planner::{TrajectoryPlanner, WindMousePlanner};
use windact::profile::Profile;

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

        // Verify timing monotonicity
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

    // Verify distance between consecutive points does not exceed physical reasonable step
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

    // Tiny box <= 4px should not panic on clamp
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
