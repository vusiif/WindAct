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
fn test_gaussian_sampling_boundary() {
    let bbox = BoundingBox::new(500.0, 200.0, 80.0, 40.0);
    for _ in 0..200 {
        let pt = WindMousePlanner::sample_gaussian_point(&bbox);
        assert!(pt.x >= bbox.x && pt.x <= bbox.x + bbox.width);
        assert!(pt.y >= bbox.y && pt.y <= bbox.y + bbox.height);
    }
}
