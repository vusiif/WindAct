//! Execution coordinator bridging trajectory planning and OS input injection.

use std::sync::atomic::AtomicBool;
use std::time::Duration;
use rand::Rng;

use crate::core::{
    ActionResult, KeyCode, MouseButton, Point, Result, ScrollAxis, TrajectoryPoint, TypingConfig,
    WindActError,
};
use crate::injector::{hybrid_sleep_cancellable, InputInjector};
use crate::planner::{StrokeConfig, StrokePlanner, TrajectoryPlanner};
use crate::profile::Profile;

/// Coordinates trajectory generation and cancellable OS input execution.
pub struct ActionEngine<P: TrajectoryPlanner, I: InputInjector> {
    planner: P,
    injector: I,
}

impl<P: TrajectoryPlanner, I: InputInjector> ActionEngine<P, I> {
    pub fn new(planner: P, injector: I) -> Self {
        Self { planner, injector }
    }

    pub fn injector(&self) -> &I {
        &self.injector
    }

    pub fn planner(&self) -> &P {
        &self.planner
    }

    /// Resolves start coordinate, falling back to current cursor position or (0, 0).
    pub fn resolve_start(&self, explicit_start: Option<Point>) -> Point {
        explicit_start.unwrap_or_else(|| {
            self.injector
                .current_cursor_pos()
                .unwrap_or(Point::new(0.0, 0.0))
        })
    }

    /// Moves cursor along a humanized trajectory to `target`.
    pub fn execute_move(
        &self,
        start: Option<Point>,
        target: Point,
        profile: &Profile,
        dry_run: bool,
        cancel: Option<&AtomicBool>,
    ) -> Result<ActionResult> {
        self.execute_move_with(&self.planner, start, target, profile, dry_run, cancel)
    }

    /// Moves cursor using an explicit planner implementation.
    pub fn execute_move_with(
        &self,
        planner: &dyn TrajectoryPlanner,
        start: Option<Point>,
        target: Point,
        profile: &Profile,
        dry_run: bool,
        cancel: Option<&AtomicBool>,
    ) -> Result<ActionResult> {
        let start_pt = self.resolve_start(start);
        let trajectory = planner.generate(start_pt, target, profile)?;
        let duration_ms = trajectory.last().map(|p| p.elapsed_ms).unwrap_or(0);

        if !dry_run {
            self.injector.play_trajectory_cancellable(&trajectory, cancel)?;
        }

        Ok(Self::build_ok_result(trajectory, duration_ms, dry_run))
    }

    /// Moves cursor along a humanized trajectory to `target` and clicks `button`.
    pub fn execute_click(
        &self,
        start: Option<Point>,
        target: Point,
        button: MouseButton,
        profile: &Profile,
        dry_run: bool,
        cancel: Option<&AtomicBool>,
    ) -> Result<ActionResult> {
        self.execute_click_with(&self.planner, start, target, button, profile, dry_run, cancel)
    }

    /// Clicks using an explicit planner implementation.
    #[allow(clippy::too_many_arguments)]
    pub fn execute_click_with(
        &self,
        planner: &dyn TrajectoryPlanner,
        start: Option<Point>,
        target: Point,
        button: MouseButton,
        profile: &Profile,
        dry_run: bool,
        cancel: Option<&AtomicBool>,
    ) -> Result<ActionResult> {
        let start_pt = self.resolve_start(start);
        let trajectory = planner.generate(start_pt, target, profile)?;
        let move_ms = trajectory.last().map(|p| p.elapsed_ms).unwrap_or(0);

        let mut rng = rand::thread_rng();
        let hold_ms = rng.gen_range(profile.press_hold_min_ms..=profile.press_hold_max_ms);

        if !dry_run {
            self.injector.play_trajectory_cancellable(&trajectory, cancel)?;
            self.injector.click_button_cancellable(button, hold_ms, cancel)?;
        }

        Ok(Self::build_ok_result(trajectory, move_ms + hold_ms as u64, dry_run))
    }

    /// Approaches `from` smoothly, holds left button, drags to `to`, hovers, and releases.
    pub fn execute_drag(
        &self,
        from: Point,
        to: Point,
        profile: &Profile,
        dry_run: bool,
        cancel: Option<&AtomicBool>,
    ) -> Result<ActionResult> {
        self.execute_drag_path_with(&self.planner, &[from, to], profile, dry_run, cancel)
    }

    /// Executes a continuous multi-waypoint drag using an explicit planner.
    /// Guarantees left button release even if aborted mid-drag.
    pub fn execute_drag_path_with(
        &self,
        planner: &dyn TrajectoryPlanner,
        waypoints: &[Point],
        profile: &Profile,
        dry_run: bool,
        cancel: Option<&AtomicBool>,
    ) -> Result<ActionResult> {
        if waypoints.len() < 2 {
            return Err(WindActError::InvalidInput(
                "Drag path requires at least 2 waypoints".into(),
            ));
        }

        let from = waypoints[0];
        let mut approach_trajectory = Vec::new();
        if !dry_run {
            let current_pos = self.injector.current_cursor_pos().unwrap_or(from);
            if current_pos.distance_to(&from) > 3.0 {
                approach_trajectory = planner.generate(current_pos, from, &Profile::casual())?;
            }
        }

        let drag_trajectory = planner.generate_path(waypoints, profile)?;
        let mut rng = rand::thread_rng();
        let pre_drag_hold = rng.gen_range(20..=50);
        let hover_ms = rng.gen_range(profile.hover_min_ms..=profile.hover_max_ms);

        if !dry_run {
            if !approach_trajectory.is_empty() {
                self.injector
                    .play_trajectory_cancellable(&approach_trajectory, cancel)?;
            } else {
                let (fx, fy) = from.to_i32();
                self.injector.move_cursor(fx, fy)?;
            }

            if !hybrid_sleep_cancellable(Duration::from_millis(30), cancel) {
                return Err(WindActError::Aborted);
            }

            self.injector.press_button(MouseButton::Left)?;

            // Ensure left button is ALWAYS released on any early exit or abort
            let drag_res = (|| -> Result<()> {
                if !hybrid_sleep_cancellable(Duration::from_millis(pre_drag_hold), cancel) {
                    return Err(WindActError::Aborted);
                }
                self.injector
                    .play_trajectory_cancellable(&drag_trajectory, cancel)?;
                if !hybrid_sleep_cancellable(Duration::from_millis(hover_ms as u64), cancel) {
                    return Err(WindActError::Aborted);
                }
                Ok(())
            })();

            let release_res = self.injector.release_button(MouseButton::Left);
            drag_res?;
            release_res?;
        }

        let approach_ms = approach_trajectory.last().map(|p| p.elapsed_ms).unwrap_or(0);
        let drag_ms = drag_trajectory.last().map(|p| p.elapsed_ms).unwrap_or(0);
        let total_ms = approach_ms + 30 + pre_drag_hold + drag_ms + hover_ms as u64;

        Ok(Self::build_ok_result(drag_trajectory, total_ms, dry_run))
    }

    /// Executes humanized multi-step wheel scrolling with natural inter-step pauses.
    pub fn execute_scroll(
        &self,
        delta: i32,
        axis: ScrollAxis,
        steps: u32,
        dry_run: bool,
        cancel: Option<&AtomicBool>,
    ) -> Result<ActionResult> {
        if delta == 0 {
            return Err(WindActError::InvalidInput("scroll delta cannot be zero".into()));
        }
        let step_count = steps.max(1).min(delta.unsigned_abs().max(1));
        let base_step = delta / step_count as i32;
        let remainder = delta % step_count as i32;

        let mut rng = rand::thread_rng();
        let mut total_ms = 0u64;

        for i in 0..step_count {
            let notch = base_step + if (i as i32) < remainder.abs() { remainder.signum() } else { 0 };
            if notch != 0 && !dry_run {
                self.injector.scroll(notch, axis)?;
            }
            if i + 1 < step_count {
                let pause_ms = rng.gen_range(15..=45);
                total_ms += pause_ms;
                if !dry_run && !hybrid_sleep_cancellable(Duration::from_millis(pause_ms), cancel) {
                    return Err(WindActError::Aborted);
                }
            }
        }

        Ok(ActionResult {
            id: None,
            status: "ok".to_string(),
            total_points: Some(step_count as usize),
            duration_ms: total_ms,
            points: None,
            error: None,
        })
    }

    /// Executes a sequence of handwriting strokes with full physical lifecycle:
    /// - Airborne WindMouse transitions between strokes (pen lifted)
    /// - Landing dwell (15~35ms) after LeftDown before moving
    /// - Two-Thirds Power Law curvature-velocity coupling + terminal flick overshoot
    /// - Guaranteed LeftUp release even on mid-stroke abort
    pub fn execute_draw_strokes(
        &self,
        strokes: &[Vec<Point>],
        cfg: &StrokeConfig,
        dry_run: bool,
        cancel: Option<&AtomicBool>,
    ) -> Result<ActionResult> {
        if strokes.is_empty() {
            return Err(WindActError::InvalidInput("strokes cannot be empty".into()));
        }

        let stroke_planner = StrokePlanner::new();
        let mut rng = rand::thread_rng();
        let mut total_ms = 0u64;
        let mut all_points = Vec::new();

        let mut cursor_pos = if !dry_run {
            self.injector.current_cursor_pos().unwrap_or(strokes[0][0])
        } else {
            strokes[0][0]
        };

        for (idx, waypoints) in strokes.iter().enumerate() {
            if cancel.is_some_and(|f| f.load(std::sync::atomic::Ordering::Relaxed)) {
                return Err(WindActError::Aborted);
            }

            let stroke_start = waypoints
                .first()
                .copied()
                .ok_or_else(|| WindActError::InvalidInput("Stroke cannot be empty".into()))?;

            // 1. Airborne transition between strokes using WindMouse
            if !dry_run && (idx > 0 || cursor_pos.distance_to(&stroke_start) > 3.0) {
                let airborne = self.planner.generate(cursor_pos, stroke_start, &Profile::fast())?;
                total_ms += airborne.last().map(|p| p.elapsed_ms).unwrap_or(0);
                self.injector.play_trajectory_cancellable(&airborne, cancel)?;
            } else if !dry_run {
                let (sx, sy) = stroke_start.to_i32();
                self.injector.move_cursor(sx, sy)?;
            }

            // 2. Plan stroke geometry + kinematics
            let stroke_pts = stroke_planner.generate_stroke(waypoints, cfg)?;
            let stroke_ms = stroke_pts.last().map(|p| p.elapsed_ms).unwrap_or(0);
            let dwell_ms = rng.gen_range(cfg.landing_dwell_min_ms..=cfg.landing_dwell_max_ms);
            total_ms += dwell_ms as u64 + stroke_ms;

            if dry_run {
                all_points.extend(stroke_pts);
            } else {
                // 3. Pen down + landing dwell + stroke playback + guaranteed pen up
                self.injector.press_button(MouseButton::Left)?;
                let play_res = (|| -> Result<()> {
                    if !hybrid_sleep_cancellable(Duration::from_millis(dwell_ms as u64), cancel) {
                        return Err(WindActError::Aborted);
                    }
                    self.injector.play_trajectory_cancellable(&stroke_pts, cancel)?;
                    Ok(())
                })();
                let release_res = self.injector.release_button(MouseButton::Left);
                play_res?;
                release_res?;

                if let Some(last) = stroke_pts.last() {
                    cursor_pos = Point::new(last.x as f64, last.y as f64);
                }
            }
        }

        Ok(ActionResult {
            id: None,
            status: "ok".to_string(),
            total_points: Some(all_points.len()),
            duration_ms: total_ms,
            points: if dry_run { Some(all_points) } else { None },
            error: None,
        })
    }

    /// Types text with realistic human rhythm:
    /// - English words: fast intra-word burst, then a natural cognitive pause after space/punctuation.
    /// - Chinese/CJK: grouped into 1~4 char IME phrases committed rapidly together, with Pinyin thinking pauses between phrases.
    pub fn execute_send_text(
        &self,
        text: &str,
        cfg: &TypingConfig,
        dry_run: bool,
        cancel: Option<&AtomicBool>,
    ) -> Result<ActionResult> {
        if text.contains('\0') {
            return Err(WindActError::InvalidInput("Text cannot contain null bytes".into()));
        }

        let mut rng = rand::thread_rng();
        let chunks = tokenize_typing_chunks(text, &mut rng);
        let char_count = text.chars().count();
        let mut total_ms = 0u64;

        let hold_low = cfg.hold_min_ms;
        let hold_high = cfg.hold_max_ms.max(hold_low);
        let inter_low = cfg.inter_key_min_ms;
        let inter_high = cfg.inter_key_max_ms.max(inter_low);
        let word_pause_low = cfg.word_pause_min_ms;
        let word_pause_high = cfg.word_pause_max_ms.max(word_pause_low);
        let cjk_pause_low = cfg.cjk_chunk_pause_min_ms;
        let cjk_pause_high = cfg.cjk_chunk_pause_max_ms.max(cjk_pause_low);

        for (chunk_idx, chunk) in chunks.iter().enumerate() {
            if cancel.is_some_and(|f| f.load(std::sync::atomic::Ordering::Relaxed)) {
                return Err(WindActError::Aborted);
            }

            let is_last_chunk = chunk_idx + 1 == chunks.len();

            match chunk {
                TypingChunk::Word(word_chars) => {
                    for (i, &ch) in word_chars.iter().enumerate() {
                        let hold_ms = rng.gen_range(hold_low..=hold_high);
                        total_ms += hold_ms as u64;
                        if !dry_run {
                            self.injector.type_char_cancellable(ch, hold_ms, cancel)?;
                        }
                        if i + 1 < word_chars.len() {
                            let burst_ms = rng.gen_range(inter_low..=inter_high);
                            total_ms += burst_ms as u64;
                            if !dry_run
                                && !hybrid_sleep_cancellable(Duration::from_millis(burst_ms as u64), cancel)
                            {
                                return Err(WindActError::Aborted);
                            }
                        }
                    }
                    // Tiny transition before following space/punctuation if not last
                    if !is_last_chunk {
                        let tail_ms = rng.gen_range(inter_low..=inter_high);
                        total_ms += tail_ms as u64;
                        if !dry_run
                            && !hybrid_sleep_cancellable(Duration::from_millis(tail_ms as u64), cancel)
                        {
                            return Err(WindActError::Aborted);
                        }
                    }
                }

                TypingChunk::Boundary(ch) => {
                    let hold_ms = rng.gen_range(hold_low..=hold_high);
                    total_ms += hold_ms as u64;
                    if !dry_run {
                        self.injector.type_char_cancellable(*ch, hold_ms, cancel)?;
                    }
                    // Cognitive thinking pause AFTER space or punctuation
                    if !is_last_chunk {
                        let pause_ms = rng.gen_range(word_pause_low..=word_pause_high);
                        total_ms += pause_ms as u64;
                        if !dry_run
                            && !hybrid_sleep_cancellable(Duration::from_millis(pause_ms as u64), cancel)
                        {
                            return Err(WindActError::Aborted);
                        }
                    }
                }

                TypingChunk::CjkPhrase(phrase_chars) => {
                    // IME commits an entire phrase almost instantaneously (0~3ms per char)
                    for (i, &ch) in phrase_chars.iter().enumerate() {
                        if !dry_run {
                            self.injector.type_char_cancellable(ch, 0, cancel)?;
                        }
                        if i + 1 < phrase_chars.len() {
                            let commit_gap = rng.gen_range(1..=4);
                            total_ms += commit_gap;
                            if !dry_run
                                && !hybrid_sleep_cancellable(Duration::from_millis(commit_gap), cancel)
                            {
                                return Err(WindActError::Aborted);
                            }
                        }
                    }
                    // Pause between Chinese phrases (simulating typing Pinyin + selecting candidate)
                    if !is_last_chunk {
                        let phrase_pause = rng.gen_range(cjk_pause_low..=cjk_pause_high);
                        total_ms += phrase_pause as u64;
                        if !dry_run
                            && !hybrid_sleep_cancellable(Duration::from_millis(phrase_pause as u64), cancel)
                        {
                            return Err(WindActError::Aborted);
                        }
                    }
                }
            }
        }

        Ok(ActionResult {
            id: None,
            status: "ok".to_string(),
            total_points: Some(char_count),
            duration_ms: total_ms,
            points: None,
            error: None,
        })
    }

    /// Presses a single key or modifier combination (e.g. "enter", "ctrl+a", "ctrl+shift+s").
    /// Uses staggered modifier press/release and guarantees all held keys are released on abort.
    pub fn execute_key_press(
        &self,
        combo: &str,
        hold_ms: Option<u32>,
        dry_run: bool,
        cancel: Option<&AtomicBool>,
    ) -> Result<ActionResult> {
        let keys = KeyCode::parse_combo(combo)?;
        let mut rng = rand::thread_rng();
        let hold = hold_ms.unwrap_or_else(|| rng.gen_range(45..=100));
        let mut total_ms = hold as u64;

        let mut stagger_delays = Vec::with_capacity(keys.len().saturating_sub(1));
        for _ in 0..keys.len().saturating_sub(1) {
            let d = rng.gen_range(15..=35);
            total_ms += d as u64 * 2;
            stagger_delays.push(d);
        }

        if !dry_run {
            let mut pressed = Vec::with_capacity(keys.len());
            let press_res = (|| -> Result<()> {
                for (i, &k) in keys.iter().enumerate() {
                    if cancel.is_some_and(|f| f.load(std::sync::atomic::Ordering::Relaxed)) {
                        return Err(WindActError::Aborted);
                    }
                    self.injector.key_down(k)?;
                    pressed.push(k);
                    if i < stagger_delays.len()
                        && !hybrid_sleep_cancellable(Duration::from_millis(stagger_delays[i] as u64), cancel)
                    {
                        return Err(WindActError::Aborted);
                    }
                }
                if hold > 0
                    && !hybrid_sleep_cancellable(Duration::from_millis(hold as u64), cancel)
                {
                    return Err(WindActError::Aborted);
                }
                Ok(())
            })();

            // Always release all pressed keys in reverse order (LIFO)
            let mut release_err = None;
            for (idx, &k) in pressed.iter().rev().enumerate() {
                if let Err(e) = self.injector.key_up(k)
                    && release_err.is_none()
                {
                    release_err = Some(e);
                }
                if idx + 1 < pressed.len() {
                    let _ = hybrid_sleep_cancellable(Duration::from_millis(stagger_delays[pressed.len() - 2 - idx] as u64), None);
                }
            }

            press_res?;
            if let Some(e) = release_err {
                return Err(e);
            }
        }

        Ok(ActionResult {
            id: None,
            status: "ok".to_string(),
            total_points: Some(keys.len()),
            duration_ms: total_ms,
            points: None,
            error: None,
        })
    }

    fn build_ok_result(
        trajectory: Vec<TrajectoryPoint>,
        duration_ms: u64,
        dry_run: bool,
    ) -> ActionResult {
        ActionResult {
            id: None,
            status: "ok".to_string(),
            total_points: Some(trajectory.len()),
            duration_ms,
            points: if dry_run { Some(trajectory) } else { None },
            error: None,
        }
    }
}

#[derive(Debug, PartialEq, Eq)]
enum TypingChunk {
    Word(Vec<char>),
    Boundary(char),
    CjkPhrase(Vec<char>),
}

#[inline]
fn is_cjk_ideograph(ch: char) -> bool {
    matches!(ch as u32,
        0x4E00..=0x9FFF | // CJK Unified Ideographs
        0x3400..=0x4DBF | // CJK Extension A
        0xF900..=0xFAFF   // CJK Compatibility Ideographs
    )
}

#[inline]
fn is_boundary_char(ch: char) -> bool {
    ch.is_whitespace() || matches!(ch, ',' | '.' | '!' | '?' | ';' | ':' | '，' | '。' | '！' | '？' | '；' | '：' | '、')
}

/// Splits text into fast-burst ASCII words, post-space/punctuation boundaries,
/// and 1~4 character CJK IME phrases.
fn tokenize_typing_chunks<R: Rng + ?Sized>(text: &str, rng: &mut R) -> Vec<TypingChunk> {
    let chars: Vec<char> = text.chars().collect();
    let mut chunks = Vec::new();
    let mut i = 0;

    while i < chars.len() {
        let ch = chars[i];
        if is_boundary_char(ch) {
            chunks.push(TypingChunk::Boundary(ch));
            i += 1;
        } else if is_cjk_ideograph(ch) {
            // Group 1~4 CJK characters (weighted toward 2~3 char words) into one IME commit chunk
            let max_phrase_len = match rng.gen_range(0..10) {
                0..=1 => 1,
                2..=6 => 2,
                7..=8 => 3,
                _ => 4,
            };
            let mut phrase = Vec::with_capacity(max_phrase_len);
            while i < chars.len() && is_cjk_ideograph(chars[i]) && phrase.len() < max_phrase_len {
                phrase.push(chars[i]);
                i += 1;
            }
            chunks.push(TypingChunk::CjkPhrase(phrase));
        } else {
            let mut word = Vec::new();
            while i < chars.len() && !is_boundary_char(chars[i]) && !is_cjk_ideograph(chars[i]) {
                word.push(chars[i]);
                i += 1;
            }
            chunks.push(TypingChunk::Word(word));
        }
    }

    chunks
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_tokenize_mixed_english_and_cjk() {
        let mut rng = rand::thread_rng();
        let text = "Hello WindAct! 中文输入测试：你好";
        let chunks = tokenize_typing_chunks(text, &mut rng);

        // Reconstruct text from chunks to guarantee zero character loss
        let mut reconstructed = String::new();
        let mut saw_hello = false;
        let mut saw_space = false;
        let mut saw_cjk_phrase = false;

        for c in &chunks {
            match c {
                TypingChunk::Word(w) => {
                    let s: String = w.iter().collect();
                    if s == "Hello" {
                        saw_hello = true;
                    }
                    reconstructed.push_str(&s);
                }
                TypingChunk::Boundary(ch) => {
                    if *ch == ' ' {
                        saw_space = true;
                    }
                    reconstructed.push(*ch);
                }
                TypingChunk::CjkPhrase(p) => {
                    assert!((1..=4).contains(&p.len()));
                    saw_cjk_phrase = true;
                    for ch in p {
                        reconstructed.push(*ch);
                    }
                }
            }
        }

        assert!(saw_hello && saw_space && saw_cjk_phrase);
        assert_eq!(reconstructed, text);
    }
}
