//! Bezier and Catmull-Rom spline trajectory planner for smooth curves and multi-point paths.

use rand::prelude::*;

use crate::core::{Point, Result, TrajectoryPoint, WindActError};
use crate::noise::PerlinNoise;
use crate::planner::TrajectoryPlanner;
use crate::profile::Profile;

/// Trajectory planner using Cubic Bezier curves (for 2-point moves) and
/// Centripetal Catmull-Rom splines (for multi-waypoint continuous paths).
#[derive(Debug, Clone)]
pub struct BezierSplinePlanner {
    perlin: PerlinNoise,
}

impl Default for BezierSplinePlanner {
    fn default() -> Self {
        Self::new()
    }
}

impl BezierSplinePlanner {
    pub fn new() -> Self {
        let mut rng = thread_rng();
        Self {
            perlin: PerlinNoise::with_seed(&mut rng),
        }
    }

    /// Evaluates a Cubic Bezier curve at parameter t in [0, 1].
    #[inline]
    fn cubic_bezier(p0: Point, p1: Point, p2: Point, p3: Point, t: f64) -> Point {
        let u = 1.0 - t;
        let tt = t * t;
        let uu = u * u;
        let uuu = uu * u;
        let ttt = tt * t;

        let x = uuu * p0.x + 3.0 * uu * t * p1.x + 3.0 * u * tt * p2.x + ttt * p3.x;
        let y = uuu * p0.y + 3.0 * uu * t * p1.y + 3.0 * u * tt * p2.y + ttt * p3.y;
        Point::new(x, y)
    }

    /// Evaluates a Catmull-Rom spline segment between p1 and p2 at t in [0, 1].
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

    /// Smooth ease-in-out sine curve mapping t in [0, 1] -> [0, 1].
    #[inline]
    fn ease_in_out_sine(t: f64) -> f64 {
        -(std::f64::consts::PI * t).cos() * 0.5 + 0.5
    }

    /// Generates two natural wrist-arc control points for a 2-point Bezier curve.
    fn generate_bezier_controls(
        start: Point,
        target: Point,
        profile: &Profile,
        rng: &mut ThreadRng,
    ) -> (Point, Point) {
        let dx = target.x - start.x;
        let dy = target.y - start.y;
        let dist = (dx * dx + dy * dy).sqrt().max(1.0);

        // Unit perpendicular vector for wrist arc bowing
        let nx = -dy / dist;
        let ny = dx / dist;

        let side = if rng.gen_bool(0.5) { 1.0 } else { -1.0 };
        let wind_factor = (profile.wind / 3.0).clamp(0.3, 2.0);
        let spread = (dist * 0.14 * wind_factor).clamp(2.5, 55.0);

        let t1 = rng.gen_range(0.22..0.42);
        let t2 = rng.gen_range(0.58..0.82);
        let bow1 = side * rng.gen_range(0.4..1.0) * spread;
        let bow2 = side * rng.gen_range(0.2..0.9) * spread;

        let p1 = Point::new(start.x + dx * t1 + nx * bow1, start.y + dy * t1 + ny * bow1);
        let p2 = Point::new(start.x + dx * t2 + nx * bow2, start.y + dy * t2 + ny * bow2);
        (p1, p2)
    }

    /// Samples a continuous curve evaluation function into discrete timed TrajectoryPoints.
    #[allow(clippy::too_many_arguments)]
    fn sample_curve<F>(
        &self,
        start: Point,
        end: Point,
        estimated_len: f64,
        profile: &Profile,
        base_elapsed_ms: u64,
        rng: &mut ThreadRng,
        eval_fn: F,
    ) -> (Vec<TrajectoryPoint>, u64)
    where
        F: Fn(f64) -> Point,
    {
        let step_px = (profile.max_step * 0.35).clamp(2.0, 6.0);
        let steps = ((estimated_len / step_px).ceil() as usize).clamp(10, 400);

        let mut points = Vec::new();
        let mut last_i32 = start.to_i32();
        let mut elapsed_ms = base_elapsed_ms;
        let trem_amp = (profile.wind * profile.noise_weight * 1.4).clamp(0.5, 4.5);

        // Low-frequency arc-length-driven Perlin drift (1~2 smooth waves per curve, zero jaggedness)
        let total_len = estimated_len.max(1.0);
        let freq = 1.6 / total_len;
        let phase_x = rng.gen_range(0.0..50.0);
        let phase_y = rng.gen_range(50.0..100.0);

        for i in 1..=steps {
            let linear_t = i as f64 / steps as f64;
            let eased_t = Self::ease_in_out_sine(linear_t);
            let mut pt = eval_fn(eased_t);

            // Envelope tapers noise to 0 at start & end so endpoints remain exact
            let s = linear_t * total_len;
            let envelope = (linear_t * std::f64::consts::PI).sin();
            let nx = self
                .perlin
                .noise(s * freq + phase_x, 0.25, 0.5)
                * trem_amp
                * envelope;
            let ny = self
                .perlin
                .noise(s * freq + phase_y, 0.75, 0.5)
                * trem_amp
                * envelope;

            pt.x += nx;
            pt.y += ny;

            let next_i32 = pt.to_i32();
            if next_i32 != last_i32 {
                last_i32 = next_i32;

                // Slower at start/end (high curvature), faster in middle cruise
                let speed_factor = (linear_t * std::f64::consts::PI).sin();
                let min_d = profile.min_delay_ms;
                let max_d = profile.max_delay_ms.max(min_d);
                let delay = if speed_factor > 0.7 {
                    rng.gen_range(min_d..=(min_d + 2).min(max_d))
                } else if speed_factor > 0.35 {
                    rng.gen_range((min_d + 1).min(max_d)..=(max_d / 2 + 3).clamp(min_d, max_d))
                } else {
                    rng.gen_range((min_d + 3).min(max_d)..=max_d)
                };

                elapsed_ms += delay as u64;
                points.push(TrajectoryPoint::new(next_i32.0, next_i32.1, delay, elapsed_ms));
            }
        }

        // Snap final pixel to exact target
        let end_i32 = end.to_i32();
        if last_i32 != end_i32 {
            let final_delay = profile.min_delay_ms.max(2);
            elapsed_ms += final_delay as u64;
            points.push(TrajectoryPoint::new(end_i32.0, end_i32.1, final_delay, elapsed_ms));
        }

        (points, elapsed_ms)
    }

    /// Generates a smooth continuous trajectory passing through all given `waypoints`.
    /// Uses Catmull-Rom spline interpolation for >= 3 waypoints, producing natural rounded corners.
    pub fn generate_spline_path(
        &self,
        waypoints: &[Point],
        profile: &Profile,
    ) -> Result<Vec<TrajectoryPoint>> {
        profile.validate()?;
        if waypoints.len() < 2 {
            return Err(WindActError::InvalidInput(
                "Spline path requires at least 2 waypoints".into(),
            ));
        }

        if waypoints.len() == 2 {
            return self.generate(waypoints[0], waypoints[1], profile);
        }

        let mut rng = thread_rng();
        let start_i32 = waypoints[0].to_i32();
        let mut full_points = vec![TrajectoryPoint::new(start_i32.0, start_i32.1, 0, 0)];
        let mut elapsed_ms = 0u64;

        let n = waypoints.len();
        for i in 0..(n - 1) {
            let p0 = if i == 0 {
                // Virtual mirrored start control point for smooth tangent
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
            let (mut seg_pts, next_elapsed) =
                self.sample_curve(p1, p2, seg_len, profile, elapsed_ms, &mut rng, |t| {
                    Self::catmull_rom(p0, p1, p2, p3, t)
                });
            full_points.append(&mut seg_pts);
            elapsed_ms = next_elapsed;
        }

        Ok(full_points)
    }
}

impl TrajectoryPlanner for BezierSplinePlanner {
    fn generate(&self, start: Point, target: Point, profile: &Profile) -> Result<Vec<TrajectoryPoint>> {
        profile.validate()?;
        let mut rng = thread_rng();

        let start_i32 = start.to_i32();
        let mut full_points = vec![TrajectoryPoint::new(start_i32.0, start_i32.1, 0, 0)];

        let dist = start.distance_to(&target);
        if dist < 1.0 {
            return Ok(full_points);
        }

        let (c1, c2) = Self::generate_bezier_controls(start, target, profile, &mut rng);
        let (mut seg, _) =
            self.sample_curve(start, target, dist, profile, 0, &mut rng, |t| {
                Self::cubic_bezier(start, c1, c2, target, t)
            });

        full_points.append(&mut seg);
        Ok(full_points)
    }

    fn generate_path(&self, waypoints: &[Point], profile: &Profile) -> Result<Vec<TrajectoryPoint>> {
        self.generate_spline_path(waypoints, profile)
    }
}
