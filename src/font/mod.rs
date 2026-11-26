//! Single-line vector font layout engine for `draw_text`.
//!
//! - Built-in ASCII/digits/punctuation via Hershey single-stroke fonts (`Romans`, `Scripts`).
//! - Optional Chinese/CJK stroke centerline loading from `Make Me a Hanzi` (`graphics.txt`).
//! - Per-glyph affine jitter (scale, tilt, baseline drift, spacing variance).

use std::collections::{HashMap, HashSet};
use std::fs::File;
use std::io::{BufRead, BufReader};
use std::path::Path;

use rand::prelude::*;
use serde::Deserialize;
use vector_text_core::Renderer;
use vector_text_hershey::{HersheyFont, HersheyRenderer};

use crate::core::{Point, Result, WindActError};

/// Built-in single-line ASCII font style.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, serde::Deserialize, clap::ValueEnum)]
#[serde(rename_all = "lowercase")]
pub enum AsciiFontStyle {
    /// Clean single-line Roman Simplex font.
    #[default]
    Romans,
    /// Cursive single-line Script Simplex font.
    Scripts,
}

impl AsciiFontStyle {
    fn to_hershey(self) -> HersheyFont {
        match self {
            Self::Romans => HersheyFont::Romans,
            Self::Scripts => HersheyFont::Scripts,
        }
    }
}

#[derive(Debug, Deserialize)]
struct HanziGraphicRecord {
    character: String,
    medians: Vec<Vec<[f64; 2]>>,
}

/// Streams `Make Me a Hanzi` `graphics.txt` line-by-line and extracts only needed characters.
fn load_hanzi_glyphs(
    path: &Path,
    needed: &HashSet<char>,
) -> Result<HashMap<char, Vec<Vec<Point>>>> {
    let file = File::open(path).map_err(|e| {
        WindActError::InvalidInput(format!(
            "Failed to open hanzi font file '{}': {e}",
            path.display()
        ))
    })?;
    let reader = BufReader::new(file);
    let mut map = HashMap::with_capacity(needed.len());

    for line_res in reader.lines() {
        let line = line_res?;
        let trimmed = line.trim();
        if trimmed.is_empty() {
            continue;
        }
        // Quick byte pre-check before full JSON parse: look up `"character":"X"`
        if let Ok(rec) = serde_json::from_str::<HanziGraphicRecord>(trimmed)
            && let Some(ch) = rec.character.chars().next()
            && rec.character.chars().count() == 1
            && needed.contains(&ch)
        {
            // Make Me a Hanzi uses 1024x1024 grid with Y pointing up (-124..900)
            let strokes = rec
                .medians
                .into_iter()
                .filter(|s| s.len() >= 2)
                .map(|s| {
                    s.into_iter()
                        .map(|[x, y]| Point::new(x / 1024.0, (900.0 - y) / 1024.0))
                        .collect()
                })
                .collect();
            map.insert(ch, strokes);
            if map.len() == needed.len() {
                break;
            }
        }
    }

    Ok(map)
}

#[inline]
fn is_cjk(ch: char) -> bool {
    matches!(
        ch as u32,
        0x4E00..=0x9FFF | 0x3400..=0x4DBF | 0xF900..=0xFAFF
    )
}

/// Converts a single ASCII character to normalized `[0..width, 0..1]` strokes using Hershey fonts.
fn hershey_char_strokes(ch: char, style: AsciiFontStyle) -> (Vec<Vec<Point>>, f64) {
    let mut buf = [0u8; 4];
    let s = ch.encode_utf8(&mut buf);
    let pts = HersheyRenderer::render_text(s, style.to_hershey());

    if pts.is_empty() {
        return (Vec::new(), 0.45);
    }

    // Hershey Simplex vertical cap height is roughly ~21 units; y ranges ~ -12..+12
    const HERSHEY_HEIGHT: f64 = 21.0;

    let mut strokes: Vec<Vec<Point>> = Vec::new();
    let mut current_stroke: Vec<Point> = Vec::new();
    let mut max_x = 0.0_f64;

    for p in pts {
        let nx = (p.x as f64) / HERSHEY_HEIGHT;
        let ny = (p.y as f64 + 12.0) / HERSHEY_HEIGHT;
        max_x = max_x.max(nx);

        if !p.pen {
            if current_stroke.len() >= 2 {
                strokes.push(current_stroke);
            }
            current_stroke = vec![Point::new(nx, ny)];
        } else {
            current_stroke.push(Point::new(nx, ny));
        }
    }
    if current_stroke.len() >= 2 {
        strokes.push(current_stroke);
    }

    let advance = (max_x + 0.16).max(0.42);
    (strokes, advance)
}

/// Applies per-glyph natural affine jitter (scale +-4%, tilt -2.5~+4 deg, baseline drift).
fn apply_glyph_jitter<R: Rng + ?Sized>(
    strokes: Vec<Vec<Point>>,
    center_x: f64,
    center_y: f64,
    font_size_px: f64,
    rng: &mut R,
) -> Vec<Vec<Point>> {
    let scale = 1.0 + rng.gen_range(-0.04..=0.04);
    let angle_rad = rng.gen_range(-2.5_f64..=4.0_f64).to_radians();
    let cos_a = angle_rad.cos();
    let sin_a = angle_rad.sin();
    let max_drift = (font_size_px * 0.04).clamp(1.5, 4.0);
    let baseline_dy = rng.gen_range(-max_drift..=max_drift);

    strokes
        .into_iter()
        .map(|stroke| {
            stroke
                .into_iter()
                .map(|pt| {
                    let dx = (pt.x - center_x) * scale;
                    let dy = (pt.y - center_y) * scale;
                    Point::new(
                        center_x + dx * cos_a - dy * sin_a,
                        center_y + dx * sin_a + dy * cos_a + baseline_dy,
                    )
                })
                .collect()
        })
        .collect()
}

/// Converts text into a sequence of screen-space strokes ready for `execute_draw_strokes`.
pub fn layout_text_strokes(
    text: &str,
    origin: Point,
    font_size_px: f64,
    style: AsciiFontStyle,
    hanzi_font_path: Option<&Path>,
) -> Result<Vec<Vec<Point>>> {
    if text.trim().is_empty() {
        return Err(WindActError::InvalidInput("Text to draw cannot be empty".into()));
    }
    if font_size_px <= 4.0 || !font_size_px.is_finite() {
        return Err(WindActError::InvalidInput(
            "font_size_px must be > 4.0 and finite".into(),
        ));
    }

    // Collect any CJK characters that require `graphics.txt`
    let needed_cjk: HashSet<char> = text.chars().filter(|&c| is_cjk(c)).collect();
    let hanzi_map = if !needed_cjk.is_empty() {
        let path = hanzi_font_path.ok_or_else(|| {
            WindActError::InvalidInput(
                "Text contains CJK characters; please specify --hanzi-font path/to/graphics.txt (Make Me a Hanzi)"
                    .into(),
            )
        })?;
        load_hanzi_glyphs(path, &needed_cjk)?
    } else {
        HashMap::new()
    };

    let mut rng = thread_rng();
    let mut all_strokes = Vec::new();
    let mut cursor_x = origin.x;
    let mut cursor_y = origin.y;
    let line_height = font_size_px * 1.45;

    for ch in text.chars() {
        match ch {
            '\n' => {
                cursor_x = origin.x;
                cursor_y += line_height;
                continue;
            }
            '\r' => continue,
            ' ' | '\t' => {
                let space_w = font_size_px * if ch == '\t' { 1.4 } else { 0.45 };
                cursor_x += space_w + rng.gen_range(-2.0..=3.0);
                continue;
            }
            _ => {}
        }

        let (norm_strokes, advance_ratio) = if is_cjk(ch) {
            if let Some(strokes) = hanzi_map.get(&ch) {
                (strokes.clone(), 1.05)
            } else {
                // Unknown CJK fallback: advance space cleanly
                cursor_x += font_size_px * 1.05;
                continue;
            }
        } else {
            hershey_char_strokes(ch, style)
        };

        if !norm_strokes.is_empty() {
            let scaled: Vec<Vec<Point>> = norm_strokes
                .into_iter()
                .map(|s| {
                    s.into_iter()
                        .map(|p| Point::new(cursor_x + p.x * font_size_px, cursor_y + p.y * font_size_px))
                        .collect()
                })
                .collect();

            let cx = cursor_x + (advance_ratio * font_size_px) * 0.5;
            let cy = cursor_y + font_size_px * 0.5;
            let jittered = apply_glyph_jitter(scaled, cx, cy, font_size_px, &mut rng);
            all_strokes.extend(jittered);
        }

        let spacing_jitter = rng.gen_range(-font_size_px * 0.04..=font_size_px * 0.05);
        cursor_x += advance_ratio * font_size_px + spacing_jitter;
    }

    if all_strokes.is_empty() {
        return Err(WindActError::InvalidInput(
            "No drawable strokes generated from input text".into(),
        ));
    }

    Ok(all_strokes)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_hershey_ascii_layout() {
        let strokes = layout_text_strokes("Hello 123!", Point::new(200.0, 300.0), 60.0, AsciiFontStyle::Romans, None)
            .expect("ASCII layout should succeed");
        assert!(!strokes.is_empty());
        for s in &strokes {
            assert!(s.len() >= 2);
            for p in s {
                assert!(p.x >= 180.0 && p.y >= 260.0);
            }
        }
    }

    #[test]
    fn test_cjk_without_font_returns_clear_error() {
        let res = layout_text_strokes("你好", Point::new(200.0, 300.0), 60.0, AsciiFontStyle::Romans, None);
        assert!(res.is_err());
    }
}
