//! Built-in timing jitter and trajectory planner throughput benchmarks.

use std::time::{Duration, Instant};
use serde::Serialize;

use crate::core::Point;
use crate::injector::hybrid_sleep;
use crate::planner::{BezierSplinePlanner, StrokeConfig, StrokePlanner, TrajectoryPlanner, WindMousePlanner};
use crate::profile::Profile;

/// Statistics for a single sleep duration target (in microseconds).
#[derive(Debug, Clone, Serialize)]
pub struct JitterStats {
    pub target_ms: u64,
    pub samples: usize,
    pub timer_1ms_enabled: bool,
    pub mean_error_us: f64,
    pub p50_error_us: u64,
    pub p95_error_us: u64,
    pub p99_error_us: u64,
    pub max_error_us: u64,
}

/// Throughput statistics for a trajectory planner.
#[derive(Debug, Clone, Serialize)]
pub struct PlannerBenchStats {
    pub planner: String,
    pub iterations: usize,
    pub total_ms: f64,
    pub avg_us_per_trajectory: f64,
}

/// Full benchmark suite report.
#[derive(Debug, Clone, Serialize)]
pub struct BenchmarkReport {
    pub timer_jitter: Vec<JitterStats>,
    pub planner_throughput: Vec<PlannerBenchStats>,
}

fn measure_sleep_jitter(target_ms: u64, samples: usize, enable_1ms_timer: bool) -> JitterStats {
    #[cfg(windows)]
    let _guard = if enable_1ms_timer {
        Some(crate::injector::windows::PrecisionTimerGuard::new(1))
    } else {
        None
    };

    let target = Duration::from_millis(target_ms);
    let mut errors_us = Vec::with_capacity(samples);

    // Warm up once
    hybrid_sleep(target);

    for _ in 0..samples.max(1) {
        let start = Instant::now();
        hybrid_sleep(target);
        let elapsed = start.elapsed();
        let err_us = elapsed.saturating_sub(target).as_micros() as u64;
        errors_us.push(err_us);
    }

    errors_us.sort_unstable();
    let sum: u64 = errors_us.iter().sum();
    let n = errors_us.len();
    let idx = |pct: f64| -> usize { ((n as f64 * pct).ceil() as usize).saturating_sub(1).min(n - 1) };

    JitterStats {
        target_ms,
        samples: n,
        timer_1ms_enabled: enable_1ms_timer,
        mean_error_us: sum as f64 / n as f64,
        p50_error_us: errors_us[idx(0.50)],
        p95_error_us: errors_us[idx(0.95)],
        p99_error_us: errors_us[idx(0.99)],
        max_error_us: *errors_us.last().unwrap_or(&0),
    }
}

fn bench_planner<F>(name: &str, iterations: usize, mut f: F) -> PlannerBenchStats
where
    F: FnMut(),
{
    let start = Instant::now();
    for _ in 0..iterations.max(1) {
        f();
    }
    let elapsed = start.elapsed();
    let total_ms = elapsed.as_secs_f64() * 1000.0;
    PlannerBenchStats {
        planner: name.to_string(),
        iterations: iterations.max(1),
        total_ms: (total_ms * 100.0).round() / 100.0,
        avg_us_per_trajectory: (elapsed.as_micros() as f64 / iterations.max(1) as f64 * 100.0).round() / 100.0,
    }
}

/// Runs the timing jitter & planner throughput benchmark suite.
pub fn run_benchmarks(samples: usize, planner_iters: usize) -> BenchmarkReport {
    let mut timer_jitter = Vec::new();
    for &ms in &[1u64, 2, 5] {
        timer_jitter.push(measure_sleep_jitter(ms, samples, true));
    }

    let windmouse = WindMousePlanner::new();
    let bezier = BezierSplinePlanner::new();
    let stroke = StrokePlanner::new();
    let profile = Profile::casual();
    let stroke_cfg = StrokeConfig::default();

    let start = Point::new(100.0, 100.0);
    let end = Point::new(800.0, 500.0);
    let waypoints = [
        Point::new(100.0, 100.0),
        Point::new(400.0, 110.0),
        Point::new(380.0, 420.0),
    ];

    let planner_throughput = vec![
        bench_planner("WindMousePlanner (700px)", planner_iters, || {
            let _ = windmouse.generate(start, end, &profile);
        }),
        bench_planner("BezierSplinePlanner (3-pt)", planner_iters, || {
            let _ = bezier.generate_path(&waypoints, &profile);
        }),
        bench_planner("StrokePlanner (Two-Thirds)", planner_iters, || {
            let _ = stroke.generate_stroke(&waypoints, &stroke_cfg);
        }),
    ];

    BenchmarkReport {
        timer_jitter,
        planner_throughput,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_benchmark_suite_runs_cleanly() {
        let report = run_benchmarks(5, 20);
        assert_eq!(report.timer_jitter.len(), 3);
        assert_eq!(report.planner_throughput.len(), 3);
        for stat in &report.timer_jitter {
            assert_eq!(stat.samples, 5);
            assert!(stat.p95_error_us >= stat.p50_error_us);
        }
    }
}
