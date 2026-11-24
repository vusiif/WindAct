//! Movement profiles and configuration parameters.

use serde::{Deserialize, Serialize};

/// Physical and behavioral configuration parameters for mouse trajectory generation.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Profile {
    /// Name identifier of the profile.
    pub name: String,
    /// Gravitational pull force toward destination (G_0).
    pub gravity: f64,
    /// Wind force fluctuation magnitude (W_0).
    pub wind: f64,
    /// Maximum velocity clip threshold per step.
    pub max_step: f64,
    /// Distance threshold from destination where wind dampens.
    pub target_distance: f64,
    /// Minimum sleep delay per movement step in milliseconds.
    pub min_delay_ms: u32,
    /// Maximum sleep delay per movement step in milliseconds.
    pub max_delay_ms: u32,
    /// Frequency scale for Perlin noise perturbation.
    pub noise_scale: f64,
    /// Weight/amplitude of Perlin noise applied to velocity.
    pub noise_weight: f64,
    /// Probability (0.0 to 1.0) of overshooting destination before snapback.
    pub overshoot_chance: f64,
    /// Maximum distance in pixels of overshoot deviation.
    pub overshoot_radius: f64,
    /// Minimum mouse button press hold duration in milliseconds.
    pub press_hold_min_ms: u32,
    /// Maximum mouse button press hold duration in milliseconds.
    pub press_hold_max_ms: u32,
    /// Minimum hover/dwell time at destination before releasing (slider mode).
    pub hover_min_ms: u32,
    /// Maximum hover/dwell time at destination before releasing (slider mode).
    pub hover_max_ms: u32,
}

impl Profile {
    /// Casual profile: natural human everyday movement, generous arcs and mild overshoot.
    pub fn casual() -> Self {
        Self {
            name: "casual".to_string(),
            gravity: 8.5,
            wind: 3.5,
            max_step: 16.0,
            target_distance: 14.0,
            min_delay_ms: 2,
            max_delay_ms: 18,
            noise_scale: 0.08,
            noise_weight: 0.5,
            overshoot_chance: 0.28,
            overshoot_radius: 18.0,
            press_hold_min_ms: 45,
            press_hold_max_ms: 110,
            hover_min_ms: 30,
            hover_max_ms: 70,
        }
    }

    /// Precise profile: sharp focus, low wind, high gravity, minimal overshoot for small UI targets.
    pub fn precise() -> Self {
        Self {
            name: "precise".to_string(),
            gravity: 15.0,
            wind: 1.2,
            max_step: 9.0,
            target_distance: 20.0,
            min_delay_ms: 3,
            max_delay_ms: 12,
            noise_scale: 0.03,
            noise_weight: 0.2,
            overshoot_chance: 0.05,
            overshoot_radius: 6.0,
            press_hold_min_ms: 60,
            press_hold_max_ms: 120,
            hover_min_ms: 40,
            hover_max_ms: 90,
        }
    }

    /// Slider profile: dedicated for captcha sliders with anti-rebound dwell time.
    pub fn slider() -> Self {
        Self {
            name: "slider".to_string(),
            gravity: 11.0,
            wind: 2.2,
            max_step: 14.0,
            target_distance: 12.0,
            min_delay_ms: 2,
            max_delay_ms: 14,
            noise_scale: 0.12,
            noise_weight: 0.6,
            overshoot_chance: 0.15,
            overshoot_radius: 8.0,
            press_hold_min_ms: 80,
            press_hold_max_ms: 160,
            hover_min_ms: 80,
            hover_max_ms: 160,
        }
    }

    /// Fast profile: rapid movement with maintained physical continuity.
    pub fn fast() -> Self {
        Self {
            name: "fast".to_string(),
            gravity: 14.0,
            wind: 2.0,
            max_step: 24.0,
            target_distance: 10.0,
            min_delay_ms: 1,
            max_delay_ms: 8,
            noise_scale: 0.05,
            noise_weight: 0.3,
            overshoot_chance: 0.20,
            overshoot_radius: 12.0,
            press_hold_min_ms: 30,
            press_hold_max_ms: 70,
            hover_min_ms: 20,
            hover_max_ms: 50,
        }
    }

    /// Looks up a profile by name (case-insensitive). Defaults to `casual` if unknown.
    pub fn from_name(name: &str) -> Self {
        match name.to_lowercase().as_str() {
            "precise" => Self::precise(),
            "slider" => Self::slider(),
            "fast" => Self::fast(),
            _ => Self::casual(),
        }
    }
}

impl Default for Profile {
    fn default() -> Self {
        Self::casual()
    }
}
