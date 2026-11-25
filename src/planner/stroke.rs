//! Kinematics-based handwriting stroke planner.
//!
//! Implements:
//! 1. Arc-length resampling of polyline/Catmull-Rom spline skeletons.
//! 2. Two-Thirds Power Law (curvature-speed coupling): straight segments move fast,
//!    sharp turns decelerate naturally.
//! 3. Low-frequency arc-length-driven Perlin noise drift (1-2 waves per stroke, zero jaggedness).
//! 4. Terminal tangent overshoot (stroke taper / flick at pen lift).

use rand::prelude::*;

use crate::core::{Point, Result, TrajectoryPoint, WindActError};
use crate::noise::PerlinNoise;

/// Configuration parameters for `StrokePlanner`.
#[derive(Debug, Clone, Copy)]
pub struct StrokeConfig {
    /// Base cruise speed in pixels per millisecond (e.g., 1.2 ~ 2.2 px/ms).
    pub base_speed_px_per_ms: f64,
    /// Curvature deceleration sensitivity (alpha in Two-Thirds Power Law).
    pub curvature_alpha: f64,
    /// Power law exponent beta (typically ~ 1/3).
    pub curvature_beta: f64,
    /// Low-frequency Perlin drift amplitude in pixels (e.g., 1.0 ~ 2.5 px).
    pub drift_amplitude_px: f64,
    /// Resampling spatial interval in pixels.
    pub resample_step_px: f64,
    /// Minimum landing dwell before stroke movement starts (ms).
    pub landing_dwell_min_ms: u32,
    /// Maximum landing dwell before stroke movement starts (ms).
    pub landing_dwell_max_ms: u32,
    /// Terminal tangent overshoot distance in pixels (stroke exit flick).
    pub overshoot_min_px: f64,
    pub overshoot_max_px: f64,
}

impl Default for StrokeConfig {
    fn default() -> Self {
        Self {
            base_speed_px_per_ms: 1.4,
            curvature_alpha: 45.0,
            curvature_beta: 1.0 / 3.0,
            drift_amplitude_px: 1.8,
            resample_step_px: 2.0,
            landing_dwell_min_ms: 15,
            landing_dwell_max_ms: 35,
            overshoot_min_px: 2.0,
            overshoot_max_px: 5.0,
        }
    }
}

/// Kinematics-driven stroke trajectory planner.
#[derive(Debug, Clone)]
pub struct StrokePlanner {
    perlin: PerlinNoise,
}

impl Default for StrokePlanner {
    fn default() -> Self {
        Self::new()
    }
}

impl StrokePlanner {
    pub fn new() -> Self {
        let mut rng = thread_rng();
        Self {
            perlin: PerlinNoise::with_seed(&mut rng),
        }
    }

    /// Evaluates Catmull-Rom spline at t in [0, 1].
    #[inline]
    fn catmull_rom(p0: Point, p1: Point, p2: Point, p3: Point, t: f64) -> Point {
        let t2 = t * t;
        let t3 = t2 * t;
        let x = 0.5
            * ((2.0 * p1.x)
                + (-p0.x + p2.x) * t
                + (2.0 * p0.x - 5.0 * p1.x + 4.0 * p2.x - p3.x) * t2
                + (-p0.x + 3.0 * p1.x - 3.0 * p2.x + p3.x) * t3);
        let y = 0.5
            * ((2.0 * p1.y)
                + (-p0.y + p2.y) * t
                + (2.0 * p0.y - 5.0 * p1.y + 4.0 * p2.y - p3.y) * t2
                + (-p0.y + 3.0 * p1.y - 3.0 * p2.y + p3.y) * t3);
        Point::new(x, y)
    }

    /// Densifies waypoints into a smooth spline skeleton, then resamples at uniform arc-length intervals.
    fn resample_by_arclength(waypoints: &[Point], step_px: f64) -> Vec<Point> {
        if waypoints.len() < 2 {
            return waypoints.to_vec();
        }

        // 1. Build dense curve samples from Catmull-Rom (or direct line for 2 points)
        let mut dense = Vec::new();
        if waypoints.len() == 2 {
            let dist = waypoints[0].distance_to(&waypoints[1]);
            let sub = ((dist / 1.0).ceil() as usize).max(2);
            for i in 0..=sub {
                let t = i as f64 / sub as f64;
                dense.push(Point::new(
                    waypoints[0].x + (waypoints[1].x - waypoints[0].x) * t,
                    waypoints[0].y + (waypoints[1].y - waypoints[0].y) * t,
                ));
            }
        } else {
            let n = waypoints.len();
            for i in 0..(n - 1) {
                let p0 = if i == 0 {
                    Point::new(
                        2.0 * waypoints[0].x - waypoints[1].x,
                        2.0 * waypoints[0].y - waypoints[1].y,
                    )
                } else {
                    waypoints[i - 1]
                };
                let p1 = waypoints[i];
                let p2 = waypoints[i + 1];
                let p3 = if i + 2 >= n {
                    Point::new(
                        2.0 * waypoints[n - 1].x - waypoints[n - 2].x,
                        2.0 * waypoints[n - 1].y - waypoints[n - 2].y,
                    )
                } else {
                    waypoints[i + 2]
                };

                let seg_len = p1.distance_to(&p2);
                let sub = ((seg_len / 1.0).ceil() as usize).max(4);
                let start_idx = if i == 0 { 0 } else { 1 };
                for j in start_idx..=sub {
                    let t = j as f64 / sub as f64;
                    dense.push(Self::catmull_rom(p0, p1, p2, p3, t));
                }
            }
        }

        // 2. Resample dense polyline by uniform arc length `step_px`
        let mut resampled = vec![dense[0]];
        let mut accum_dist = 0.0;
        let mut prev = dense[0];

        for &curr in dense.iter().skip(1) {
            let d = prev.distance_to(&curr);
            accum_dist += d;
            if accum_dist >= step_px {
                resampled.push(curr);
                accum_dist = 0.0;
            }
            prev = curr;
        }

        if let Some(&last_dense) = dense.last()
            && resampled.last().is_some_and(|p| p.distance_to(&last_dense) > 0.5)
        {
            resampled.push(last_dense);
        }

        resampled
    }

    /// Computes Menger curvature kappa_i from three consecutive points.
    #[inline]
    fn menger_curvature(p_prev: Point, p_curr: Point, p_next: Point) -> f64 {
        let a = p_prev.distance_to(&p_curr);
        let b = p_curr.distance_to(&p_next);
        let c = p_prev.distance_to(&p_next);
        let denom = a * b * c;
        if denom < 1e-6 {
            return 0.0;
        }
        // 2 * signed triangle area via cross product
        let cross = ((p_curr.x - p_prev.x) * (p_next.y - p_prev.y)
            - (p_curr.y - p_prev.y) * (p_next.x - p_prev.x))
            .abs();
        (2.0 * cross) / denom
    }

    /// Generates a timed stroke trajectory applying:
    /// - Uniform arc-length resampling
    /// - Low-frequency arc-length-driven Perlin drift (1~2 gentle waves per stroke)
    /// - Two-Thirds Power Law curvature-velocity coupling
    /// - Terminal tangent overshoot (stroke flick)
    pub fn generate_stroke(
        &self,
        waypoints: &[Point],
        cfg: &StrokeConfig,
    ) -> Result<Vec<TrajectoryPoint>> {
        if waypoints.len() < 2 {
            return Err(WindActError::InvalidInput(
                "Stroke requires at least 2 waypoints".into(),
            ));
        }

        let mut rng = thread_rng();
        let mut pts = Self::resample_by_arclength(waypoints, cfg.resample_step_px.max(0.5));
        if pts.len() < 2 {
            let p0 = waypoints[0].to_i32();
            return Ok(vec![TrajectoryPoint::new(p0.0, p0.1, 0, 0)]);
        }

        // Compute cumulative arc length along the stroke
        let mut arc_lengths = vec![0.0_f64; pts.len()];
        for i in 1..pts.len() {
            arc_lengths[i] = arc_lengths[i - 1] + pts[i - 1].distance_to(&pts[i]);
        }
        let total_len = (*arc_lengths.last().unwrap_or(&1.0)).max(1.0);

        // Apply low-frequency Perlin drift parameterized by arc length `s`:
        // freq chosen so entire stroke contains ~1.2-1.8 smooth half-waves (zero jaggedness)
        let freq = 1.5 / total_len;
        let phase_x = rng.gen_range(0.0..50.0);
        let phase_y = rng.gen_range(50.0..100.0);

        for i in 1..(pts.len() - 1) {
            let s = arc_lengths[i];
            let norm_s = s / total_len;
            // Taper drift gently at start/end
            let envelope = (norm_s * std::f64::consts::PI).sin();
            let dx = self.perlin.noise(s * freq + phase_x, 0.25, 0.5)
                * cfg.drift_amplitude_px
                * envelope;
            let dy = self.perlin.noise(s * freq + phase_y, 0.75, 0.5)
                * cfg.drift_amplitude_px
                * envelope;
            pts[i].x += dx;
            pts[i].y += dy;
        }

        // Terminal tangent overshoot (stroke exit flick)
        let n = pts.len();
        let prev_pt = pts[n - 2];
        let end_pt = pts[n - 1];
        let tx = end_pt.x - prev_pt.x;
        let ty = end_pt.y - prev_pt.y;
        let t_mag = (tx * tx + ty * ty).sqrt();
        if t_mag > 1e-3 && cfg.overshoot_max_px > 0.0 {
            let overshoot_dist = rng.gen_range(cfg.overshoot_min_px..=cfg.overshoot_max_px);
            let ux = tx / t_mag;
            let uy = ty / t_mag;
            // Add 2 decelerating exit points along tangent
            pts.push(Point::new(
                end_pt.x + ux * (overshoot_dist * 0.55),
                end_pt.y + uy * (overshoot_dist * 0.55),
            ));
            pts.push(Point::new(
                end_pt.x + ux * overshoot_dist,
                end_pt.y + uy * overshoot_dist,
            ));
        }

        // Compute Two-Thirds Power Law timing: v_i = v_base * (1 + alpha * kappa_i)^(-beta)
        let start_i32 = pts[0].to_i32();
        let mut trajectory = vec![TrajectoryPoint::new(start_i32.0, start_i32.1, 0, 0)];
        let mut last_i32 = start_i32;
        let mut accum_time_ms = 0.0_f64;
        let mut last_recorded_ms = 0_u64;

        let base_v = cfg.base_speed_px_per_ms.max(0.2);

        for i in 1..pts.len() {
            let seg_dist = pts[i - 1].distance_to(&pts[i]);
            let kappa = if i + 1 < pts.len() {
                Self::menger_curvature(pts[i - 1], pts[i], pts[i + 1])
            } else {
                // Terminal exit flick decelerates naturally
                0.15
            };

            let speed = base_v * (1.0 + cfg.curvature_alpha * kappa).powf(-cfg.curvature_beta);
            let dt = seg_dist / speed.max(0.08);
            accum_time_ms += dt;

            let curr_i32 = pts[i].to_i32();
            if curr_i32 != last_i32 {
                last_i32 = curr_i32;
                let target_elapsed = accum_time_ms.round() as u64;
                let delay = target_elapsed.saturating_sub(last_recorded_ms).max(1) as u32;
                last_recorded_ms += delay as u64;
                trajectory.push(TrajectoryPoint::new(
                    curr_i32.0,
                    curr_i32.1,
                    delay,
                    last_recorded_ms,
                ));
            }
        }

        Ok(trajectory)
    }
}
