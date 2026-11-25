//! Core data types and error definitions for WindAct.

use serde::{Deserialize, Serialize};
use thiserror::Error;

/// A 2D floating-point coordinate point.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Point {
    pub x: f64,
    pub y: f64,
}

impl Point {
    pub const fn new(x: f64, y: f64) -> Self {
        Self { x, y }
    }

    pub fn distance_to(&self, other: &Point) -> f64 {
        ((self.x - other.x).powi(2) + (self.y - other.y).powi(2)).sqrt()
    }

    pub fn to_i32(&self) -> (i32, i32) {
        (self.x.round() as i32, self.y.round() as i32)
    }
}

impl From<(f64, f64)> for Point {
    fn from(tuple: (f64, f64)) -> Self {
        Self::new(tuple.0, tuple.1)
    }
}

impl From<(i32, i32)> for Point {
    fn from(tuple: (i32, i32)) -> Self {
        Self::new(tuple.0 as f64, tuple.1 as f64)
    }
}

/// A discrete point on a trajectory containing coordinate and timing information.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct TrajectoryPoint {
    /// Screen X coordinate in physical pixels.
    pub x: i32,
    /// Screen Y coordinate in physical pixels.
    pub y: i32,
    /// Delay in milliseconds since the previous point.
    pub delay_ms: u32,
    /// Total elapsed time in milliseconds from trajectory start.
    pub elapsed_ms: u64,
}

impl TrajectoryPoint {
    pub const fn new(x: i32, y: i32, delay_ms: u32, elapsed_ms: u64) -> Self {
        Self {
            x,
            y,
            delay_ms,
            elapsed_ms,
        }
    }
}

/// Mouse buttons supported by WindAct.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize, clap::ValueEnum)]
#[serde(rename_all = "lowercase")]
pub enum MouseButton {
    #[default]
    Left,
    Right,
    Middle,
}

/// Mouse wheel scroll axis.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize, clap::ValueEnum)]
#[serde(rename_all = "lowercase")]
pub enum ScrollAxis {
    #[default]
    Vertical,
    Horizontal,
}

/// Keyboard keys supported by `key_press` and modifier combinations.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum KeyCode {
    Enter,
    Tab,
    Space,
    Backspace,
    Escape,
    Delete,
    Home,
    End,
    PageUp,
    PageDown,
    Left,
    Right,
    Up,
    Down,
    Ctrl,
    Alt,
    Shift,
    Win,
    F(u8),
    Char(char),
}

impl KeyCode {
    /// Parses a single key name (case-insensitive), e.g. "enter", "ctrl", "f5", "a".
    pub fn parse_single(name: &str) -> Result<Self> {
        let trimmed = name.trim();
        if trimmed.is_empty() {
            return Err(WindActError::InvalidInput("Key name cannot be empty".into()));
        }
        let lower = trimmed.to_lowercase();
        match lower.as_str() {
            "enter" | "return" => Ok(Self::Enter),
            "tab" => Ok(Self::Tab),
            "space" => Ok(Self::Space),
            "backspace" | "bs" => Ok(Self::Backspace),
            "esc" | "escape" => Ok(Self::Escape),
            "del" | "delete" => Ok(Self::Delete),
            "home" => Ok(Self::Home),
            "end" => Ok(Self::End),
            "pageup" | "pgup" => Ok(Self::PageUp),
            "pagedown" | "pgdn" => Ok(Self::PageDown),
            "left" => Ok(Self::Left),
            "right" => Ok(Self::Right),
            "up" => Ok(Self::Up),
            "down" => Ok(Self::Down),
            "ctrl" | "control" => Ok(Self::Ctrl),
            "alt" | "menu" => Ok(Self::Alt),
            "shift" => Ok(Self::Shift),
            "win" | "meta" | "super" => Ok(Self::Win),
            _ if lower.starts_with('f') && lower.len() <= 3 => {
                if let Ok(num) = lower[1..].parse::<u8>()
                    && (1..=12).contains(&num)
                {
                    return Ok(Self::F(num));
                }
                Err(WindActError::InvalidInput(format!("Unsupported function key '{trimmed}'")))
            }
            _ => {
                let mut chars = trimmed.chars();
                if let (Some(ch), None) = (chars.next(), chars.next()) {
                    Ok(Self::Char(ch))
                } else {
                    Err(WindActError::InvalidInput(format!("Unsupported key '{trimmed}'")))
                }
            }
        }
    }

    /// Parses a key combination like "ctrl+shift+s" or "enter" into an ordered slice of keys.
    pub fn parse_combo(combo: &str) -> Result<Vec<Self>> {
        let parts: Vec<&str> = combo.split('+').collect();
        if parts.is_empty() {
            return Err(WindActError::InvalidInput("Key combo cannot be empty".into()));
        }
        parts.into_iter().map(Self::parse_single).collect()
    }
}

/// Humanized typing rhythm configuration.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct TypingConfig {
    pub inter_key_min_ms: u32,
    pub inter_key_max_ms: u32,
    pub hold_min_ms: u32,
    pub hold_max_ms: u32,
    pub pause_chance: f64,
    pub pause_min_ms: u32,
    pub pause_max_ms: u32,
}

impl Default for TypingConfig {
    fn default() -> Self {
        Self::casual()
    }
}

impl TypingConfig {
    pub fn casual() -> Self {
        Self {
            inter_key_min_ms: 45,
            inter_key_max_ms: 135,
            hold_min_ms: 35,
            hold_max_ms: 95,
            pause_chance: 0.05,
            pause_min_ms: 120,
            pause_max_ms: 300,
        }
    }

    pub fn fast() -> Self {
        Self {
            inter_key_min_ms: 20,
            inter_key_max_ms: 65,
            hold_min_ms: 20,
            hold_max_ms: 55,
            pause_chance: 0.02,
            pause_min_ms: 80,
            pause_max_ms: 160,
        }
    }

    pub fn slow() -> Self {
        Self {
            inter_key_min_ms: 85,
            inter_key_max_ms: 200,
            hold_min_ms: 50,
            hold_max_ms: 120,
            pause_chance: 0.08,
            pause_min_ms: 160,
            pause_max_ms: 380,
        }
    }

    pub fn from_name(name: &str) -> Result<Self> {
        match name.to_lowercase().as_str() {
            "casual" => Ok(Self::casual()),
            "fast" => Ok(Self::fast()),
            "slow" => Ok(Self::slow()),
            _ => Err(WindActError::InvalidInput(format!(
                "Unknown typing profile '{name}'. Expected one of: casual, fast, slow"
            ))),
        }
    }
}

/// Execution status result of a trajectory or action.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ActionResult {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub id: Option<u64>,
    pub status: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub total_points: Option<usize>,
    pub duration_ms: u64,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub points: Option<Vec<TrajectoryPoint>>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub error: Option<String>,
}

/// Bounding box for Gaussian randomized destination sampling.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct BoundingBox {
    pub x: f64,
    pub y: f64,
    pub width: f64,
    pub height: f64,
}

impl BoundingBox {
    pub const fn new(x: f64, y: f64, width: f64, height: f64) -> Self {
        Self {
            x,
            y,
            width,
            height,
        }
    }
}

/// WindAct common error types.
#[derive(Debug, Error)]
pub enum WindActError {
    #[error("Invalid coordinate or boundary: {0}")]
    InvalidInput(String),

    #[error("Trajectory generation failed: {0}")]
    TrajectoryError(String),

    #[error("OS injection failed: {0}")]
    InjectionError(String),

    #[error("Action aborted")]
    Aborted,

    #[error("IO error: {0}")]
    Io(#[from] std::io::Error),

    #[error("JSON serialization error: {0}")]
    Json(#[from] serde_json::Error),
}

pub type Result<T> = std::result::Result<T, WindActError>;
