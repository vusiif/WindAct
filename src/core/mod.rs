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

/// Execution status result of a trajectory or action.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ActionResult {
    pub id: Option<u64>,
    pub status: String,
    pub total_points: usize,
    pub duration_ms: u64,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub points: Option<Vec<TrajectoryPoint>>,
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
