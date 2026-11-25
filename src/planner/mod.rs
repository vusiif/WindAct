//! Trajectory planning algorithms and implementations.

use std::f64::consts::PI;
use rand::prelude::*;

use crate::core::{BoundingBox, Point, Result, TrajectoryPoint, WindActError};
use crate::noise::PerlinNoise;
use crate::profile::Profile;

/// Hard upper bound on simulation iterations per segment to guarantee termination.
const MAX_SIMULATION_ITERATIONS: usize = 10_000;

/// Trait defining a trajectory planning algorithm.
pub trait TrajectoryPlanner: Send + Sync {
    /// Generates a human-like mouse movement path from `start` to `target`.
    fn generate(&self, start: Point, target: Point, profile: &Profile) -> Result<Vec<TrajectoryPoint>>;
}

/// Enhanced WindMouse trajectory planner with Perlin noise perturbation,
/// dynamic velocity-based delay distributions, and human overshoot callbacks.
#[derive(Debug, Clone)]
pub struct WindMousePlanner {
    perlin: PerlinNoise,
}

impl Default for WindMousePlanner {
    fn default() -> Self {
        Self::new()
    }
}

impl WindMousePlanner {
    /// Creates a new WindMousePlanner with a fresh Perlin noise generator.
    pub fn new() -> Self {
        let mut rng = thread_rng();
        Self {
            perlin: PerlinNoise::with_seed(&mut rng),
        }
    }

    /// Samples a target point inside a bounding box using a 2D Gaussian distribution.
    /// Points cluster near the center with natural spread, safely handling small boxes.
    pub fn sample_gaussian_point(bbox: &BoundingBox) -> Result<Point> {
        if bbox.width < 0.0 || bbox.height < 0.0 || !bbox.width.is_finite() || !bbox.height.is_finite() {
            return Err(WindActError::InvalidInput(
                "BoundingBox width and height must be non-negative and finite".into(),
            ));
        }

        let cx = bbox.x + bbox.width / 2.0;
        let cy = bbox.y + bbox.height / 2.0;

        let mut rng = thread_rng();
        // Box-Muller transform for standard normal distribution
        let u1: f64 = rng.gen_range(1e-7..1.0);
        let u2: f64 = rng.gen_range(0.0..1.0);
        let r = (-2.0 * u1.ln()).sqrt();
        let theta = 2.0 * PI * u2;
        let z0 = r * theta.cos();
        let z1 = r * theta.sin();

        let x = if bbox.width <= 2.0 {
            cx
        } else {
            let pad_x = (bbox.width * 0.1).min(2.0);
            let sigma_x = bbox.width * 0.15;
            (cx + z0 * sigma_x).clamp(bbox.x + pad_x, bbox.x + bbox.width - pad_x)
        };

        let y = if bbox.height <= 2.0 {
            cy
        } else {
            let pad_y = (bbox.height * 0.1).min(2.0);
            let sigma_y = bbox.height * 0.15;
            (cy + z1 * sigma_y).clamp(bbox.y + pad_y, bbox.y + bbox.height - pad_y)
        };

        Ok(Point::new(x, y))
    }

    /// Internal generator for a single segment of WindMouse trajectory.
    fn generate_segment(
        &self,
        start: Point,
        destination: Point,
        profile: &Profile,
        base_elapsed_ms: u64,
        rng: &mut ThreadRng,
    ) -> (Vec<TrajectoryPoint>, u64) {
        let sqrt3 = 3.0_f64.sqrt();
        let sqrt5 = 5.0_f64.sqrt();

        let mut current = start;
        let mut current_pos_i32 = (current.x.round() as i32, current.y.round() as i32);
        let mut velocity_x = 0.0;
        let mut velocity_y = 0.0;
        let mut wind_x = rng.gen_range(-1.0..1.0) * profile.wind;
        let mut wind_y = rng.gen_range(-1.0..1.0) * profile.wind;
        let mut current_max_step = profile.max_step;

        let mut points = Vec::new();
        let mut elapsed_ms = base_elapsed_ms;

        // Main physical simulation loop with hard iteration cap
        for _ in 0..MAX_SIMULATION_ITERATIONS {
            let dist = current.distance_to(&destination);
            if dist <= 1.2 {
                break;
            }

            let wind_current = profile.wind.min(dist);

            // Dampen Perlin noise proportionally when approaching target to prevent orbiting
            let noise_damp = if profile.target_distance > 0.0 && dist < profile.target_distance {
                (dist / profile.target_distance).clamp(0.0, 1.0)
            } else {
                1.0
            };

            if dist >= profile.target_distance {
                let wx_rand: f64 = rng.gen_range(-1.0..1.0);
                let wy_rand: f64 = rng.gen_range(-1.0..1.0);
                wind_x = wind_x / sqrt3 + wx_rand * wind_current / sqrt5;
                wind_y = wind_y / sqrt3 + wy_rand * wind_current / sqrt5;
            } else {
                wind_x /= sqrt3;
                wind_y /= sqrt3;
                if current_max_step < 3.0 {
                    current_max_step = rng.gen_range(2.0_f64..4.0_f64).min(dist.max(1.5));
                } else {
                    current_max_step /= sqrt5;
                }
            }

            // Perlin noise continuous perturbation
            let noise_x = self.perlin.noise(
                current.x * profile.noise_scale,
                current.y * profile.noise_scale,
                0.0,
            ) * profile.wind * profile.noise_weight * noise_damp;

            let noise_y = self.perlin.noise(
                current.x * profile.noise_scale,
                current.y * profile.noise_scale,
                100.0,
            ) * profile.wind * profile.noise_weight * noise_damp;

            // Apply gravity + wind + noise to instantaneous velocity
            velocity_x += wind_x + profile.gravity * (destination.x - current.x) / dist + noise_x;
            velocity_y += wind_y + profile.gravity * (destination.y - current.y) / dist + noise_y;

            // Clip velocity magnitude
            let vel_mag = (velocity_x * velocity_x + velocity_y * velocity_y).sqrt();
            if vel_mag > current_max_step && vel_mag > 1e-6 {
                let clip = current_max_step * 0.5 + rng.gen_range(0.0..1.0) * current_max_step * 0.5;
                velocity_x = (velocity_x / vel_mag) * clip;
                velocity_y = (velocity_y / vel_mag) * clip;
            }

            // Update floating-point position
            current.x += velocity_x;
            current.y += velocity_y;

            let next_pos_i32 = (current.x.round() as i32, current.y.round() as i32);

            // Record point when pixel position changes
            if next_pos_i32 != current_pos_i32 {
                current_pos_i32 = next_pos_i32;

                let step_velocity = vel_mag.min(profile.max_step);
                let delay = self.calculate_delay(step_velocity, dist, profile, rng);
                elapsed_ms += delay as u64;

                points.push(TrajectoryPoint::new(
                    current_pos_i32.0,
                    current_pos_i32.1,
                    delay,
                    elapsed_ms,
                ));
            }
        }

        (points, elapsed_ms)
    }

    /// Computes a realistic delay (in ms) according to physical velocity and distance.
    fn calculate_delay(
        &self,
        velocity: f64,
        distance_to_target: f64,
        profile: &Profile,
        rng: &mut ThreadRng,
    ) -> u32 {
        let normalized_vel = (velocity / profile.max_step).clamp(0.0, 1.0);
        let min_d = profile.min_delay_ms;
        let max_d = profile.max_delay_ms.max(min_d);

        // Near target: small precision delays
        if distance_to_target < 6.0 {
            let low = min_d.max(5).min(max_d.max(5));
            let high = max_d.max(12).max(low);
            return rng.gen_range(low..=high);
        }

        // Fast cruise speed: mostly min_delay_ms with occasional slight bump
        let base_delay = if normalized_vel > 0.8 {
            if rng.gen_bool(0.85) {
                min_d
            } else {
                (min_d + 2).min(max_d)
            }
        } else if normalized_vel > 0.4 {
            let low = (min_d + 1).min(max_d);
            let high = (max_d / 2 + 3).clamp(low, max_d);
            rng.gen_range(low..=high)
        } else {
            let low = (min_d + 3).min(max_d);
            rng.gen_range(low..=max_d)
        };

        // Rare micro-hesitation (1.2% probability, 25~60ms) simulating human cognition
        if rng.gen_bool(0.012) {
            base_delay + rng.gen_range(25..60)
        } else {
            base_delay
        }
    }
}

impl TrajectoryPlanner for WindMousePlanner {
    fn generate(&self, start: Point, target: Point, profile: &Profile) -> Result<Vec<TrajectoryPoint>> {
        profile.validate()?;

        let mut rng = thread_rng();
        let total_dist = start.distance_to(&target);

        // Start from initial point
        let start_i32 = start.to_i32();
        let mut full_points = vec![TrajectoryPoint::new(start_i32.0, start_i32.1, 0, 0)];
        let mut current_elapsed_ms = 0u64;

        // Check if movement is long enough to trigger human overshoot and correction
        let should_overshoot = total_dist > 75.0 && rng.gen_bool(profile.overshoot_chance);

        let intermediate_target = if should_overshoot {
            // Direction vector
            let dx = target.x - start.x;
            let dy = target.y - start.y;
            let mag = (dx * dx + dy * dy).sqrt();
            let ux = dx / mag;
            let uy = dy / mag;

            // Random overshoot distance and perpendicular jitter
            let overshoot_dist = if profile.overshoot_radius > 4.0 {
                rng.gen_range(4.0..=profile.overshoot_radius)
            } else {
                profile.overshoot_radius
            };
            let perp_dist = rng.gen_range(-4.0..=4.0);

            Point::new(
                target.x + ux * overshoot_dist - uy * perp_dist,
                target.y + uy * overshoot_dist + ux * perp_dist,
            )
        } else {
            target
        };

        // Phase 1: Move to intermediate target (or direct target)
        let (mut seg1, elapsed1) = self.generate_segment(start, intermediate_target, profile, current_elapsed_ms, &mut rng);
        full_points.append(&mut seg1);
        current_elapsed_ms = elapsed1;

        // Phase 2: If overshoot occurred, correct back to exact target
        if should_overshoot {
            let last_pos = full_points
                .last()
                .map(|p| Point::new(p.x as f64, p.y as f64))
                .unwrap_or(intermediate_target);

            let mut snapback_profile = profile.clone();
            snapback_profile.max_step = (profile.max_step * 0.6).max(4.0);
            snapback_profile.gravity = profile.gravity * 1.5;

            let (mut seg2, elapsed2) = self.generate_segment(last_pos, target, &snapback_profile, current_elapsed_ms, &mut rng);
            full_points.append(&mut seg2);
            current_elapsed_ms = elapsed2;
        }

        // Guarantee final point lands exactly at target coordinate
        let target_i32 = target.to_i32();
        let needs_final_snap = full_points
            .last()
            .map(|p| (p.x, p.y) != target_i32)
            .unwrap_or(true);

        if needs_final_snap {
            let high = profile.max_delay_ms.min(8).max(profile.min_delay_ms);
            let final_delay = rng.gen_range(profile.min_delay_ms..=high);
            current_elapsed_ms += final_delay as u64;
            full_points.push(TrajectoryPoint::new(
                target_i32.0,
                target_i32.1,
                final_delay,
                current_elapsed_ms,
            ));
        }

        Ok(full_points)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_windmouse_planner_end_to_end() {
        let planner = WindMousePlanner::new();
        let profile = Profile::casual();
        let start = Point::new(100.0, 100.0);
        let target = Point::new(800.0, 600.0);

        let trajectory = planner.generate(start, target, &profile).expect("Trajectory should generate successfully");

        assert!(trajectory.len() >= 5, "Trajectory should have multiple points");
        assert_eq!((trajectory[0].x, trajectory[0].y), (100, 100), "Start point must match");
        assert_eq!(
            (trajectory.last().unwrap().x, trajectory.last().unwrap().y),
            (800, 600),
            "Target point must match exactly"
        );

        // Verify timing monotonicity
        for i in 1..trajectory.len() {
            assert!(
                trajectory[i].elapsed_ms >= trajectory[i - 1].elapsed_ms,
                "Elapsed time must be monotonically non-decreasing"
            );
        }
    }

    #[test]
    fn test_gaussian_point_sampling() {
        let bbox = BoundingBox::new(200.0, 300.0, 100.0, 50.0);
        for _ in 0..100 {
            let pt = WindMousePlanner::sample_gaussian_point(&bbox).unwrap();
            assert!(pt.x >= bbox.x && pt.x <= bbox.x + bbox.width, "X must be within bbox");
            assert!(pt.y >= bbox.y && pt.y <= bbox.y + bbox.height, "Y must be within bbox");
        }
    }
}
