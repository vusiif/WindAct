//! Input injection layer for dispatching hardware/OS events.

use std::sync::atomic::{AtomicBool, Ordering};
use std::thread;
use std::time::{Duration, Instant};

use crate::core::{KeyCode, MouseButton, Point, Result, ScrollAxis, TrajectoryPoint, WindActError};

/// High-precision sleep combining thread sleep and CPU busy-spin loop.
/// Yields the thread for the bulk of the wait, then spins to achieve sub-millisecond accuracy.
pub fn hybrid_sleep(target_duration: Duration) {
    let _ = hybrid_sleep_cancellable(target_duration, None);
}

/// Cancellable high-precision sleep that checks an atomic abort flag every ~1ms.
/// Returns `true` if the full duration elapsed, or `false` if cancelled early.
pub fn hybrid_sleep_cancellable(target_duration: Duration, cancel: Option<&AtomicBool>) -> bool {
    let start = Instant::now();
    let check_cancelled = || cancel.is_some_and(|flag| flag.load(Ordering::Relaxed));

    if check_cancelled() {
        return false;
    }

    while let Some(remaining) = target_duration.checked_sub(start.elapsed()) {
        if check_cancelled() {
            return false;
        }
        if remaining > Duration::from_millis(2) {
            // Sleep in 1ms slices when cancellable so abort responds within ~1-2ms
            let slice = if cancel.is_some() {
                Duration::from_millis(1)
            } else {
                remaining - Duration::from_millis(1)
            };
            thread::sleep(slice);
        } else {
            while start.elapsed() < target_duration {
                if check_cancelled() {
                    return false;
                }
                std::hint::spin_loop();
            }
            break;
        }
    }

    !check_cancelled()
}

/// Common trait for injecting mouse movement, button, and scroll events into the OS.
pub trait InputInjector: Send + Sync {
    /// Moves the cursor to physical screen coordinates (x, y).
    fn move_cursor(&self, x: i32, y: i32) -> Result<()>;

    /// Presses down the specified mouse button.
    fn press_button(&self, button: MouseButton) -> Result<()>;

    /// Releases the specified mouse button.
    fn release_button(&self, button: MouseButton) -> Result<()>;

    /// Clicks the specified mouse button with a humanized hold duration in milliseconds.
    fn click_button(&self, button: MouseButton, hold_ms: u32) -> Result<()> {
        self.click_button_cancellable(button, hold_ms, None)
    }

    /// Clicks the specified mouse button with cancellation support during the hold period.
    fn click_button_cancellable(
        &self,
        button: MouseButton,
        hold_ms: u32,
        cancel: Option<&AtomicBool>,
    ) -> Result<()>;

    /// Scrolls the mouse wheel by `delta` notches (positive = up/right, negative = down/left).
    fn scroll(&self, delta: i32, axis: ScrollAxis) -> Result<()>;

    /// Plays a sequence of pre-computed trajectory points with precise inter-step timing.
    fn play_trajectory(&self, points: &[TrajectoryPoint]) -> Result<()> {
        self.play_trajectory_cancellable(points, None)
    }

    /// Plays a trajectory while polling an optional atomic cancellation flag.
    fn play_trajectory_cancellable(
        &self,
        points: &[TrajectoryPoint],
        cancel: Option<&AtomicBool>,
    ) -> Result<()>;

    /// Retrieves the current physical cursor location.
    fn current_cursor_pos(&self) -> Result<Point>;

    /// Presses down a keyboard key.
    fn key_down(&self, key: KeyCode) -> Result<()>;

    /// Releases a keyboard key.
    fn key_up(&self, key: KeyCode) -> Result<()>;

    /// Types a single character via OS Unicode or key mapping with a humanized hold time.
    fn type_char_cancellable(
        &self,
        ch: char,
        hold_ms: u32,
        cancel: Option<&AtomicBool>,
    ) -> Result<()>;
}

// ---------------------------------------------------------------------------
// Windows Implementation
// ---------------------------------------------------------------------------

#[cfg(windows)]
pub mod windows {
    use super::*;
    use std::mem::size_of;
    use windows_sys::Win32::Foundation::POINT;
    use windows_sys::Win32::UI::HiDpi::{
        DPI_AWARENESS_CONTEXT, DPI_AWARENESS_CONTEXT_PER_MONITOR_AWARE, SetThreadDpiAwarenessContext,
    };
    use windows_sys::Win32::UI::Input::KeyboardAndMouse::{
        GetKeyboardLayout, MapVirtualKeyExW, SendInput, INPUT, INPUT_0, INPUT_KEYBOARD,
        INPUT_MOUSE, KEYBDINPUT, KEYEVENTF_EXTENDEDKEY, KEYEVENTF_KEYUP, KEYEVENTF_UNICODE,
        MAPVK_VK_TO_VSC_EX, MOUSEEVENTF_ABSOLUTE, MOUSEEVENTF_HWHEEL, MOUSEEVENTF_LEFTDOWN,
        MOUSEEVENTF_LEFTUP, MOUSEEVENTF_MIDDLEDOWN, MOUSEEVENTF_MIDDLEUP, MOUSEEVENTF_MOVE,
        MOUSEEVENTF_RIGHTDOWN, MOUSEEVENTF_RIGHTUP, MOUSEEVENTF_VIRTUALDESK, MOUSEEVENTF_WHEEL,
        MOUSEINPUT, VK_BACK, VK_CONTROL, VK_DELETE, VK_DOWN, VK_END, VK_ESCAPE, VK_F1, VK_HOME,
        VK_LEFT, VK_LWIN, VK_MENU, VK_NEXT, VK_PRIOR, VK_RETURN, VK_RIGHT, VK_SHIFT, VK_SPACE,
        VK_TAB, VK_UP,
    };
    use windows_sys::Win32::UI::WindowsAndMessaging::{
        GetCursorPos, GetForegroundWindow, GetSystemMetrics, GetWindowThreadProcessId,
        SM_CXSCREEN, SM_CXVIRTUALSCREEN, SM_CYSCREEN, SM_CYVIRTUALSCREEN, SM_XVIRTUALSCREEN,
        SM_YVIRTUALSCREEN, WHEEL_DELTA,
    };

    #[link(name = "winmm")]
    unsafe extern "system" {
        fn timeBeginPeriod(uperiod: u32) -> u32;
        fn timeEndPeriod(uperiod: u32) -> u32;
    }

    /// RAII Guard that enforces 1ms timer interrupt resolution via timeBeginPeriod.
    pub struct PrecisionTimerGuard {
        period_ms: u32,
    }

    impl PrecisionTimerGuard {
        pub fn new(period_ms: u32) -> Self {
            unsafe {
                timeBeginPeriod(period_ms);
            }
            Self { period_ms }
        }
    }

    impl Drop for PrecisionTimerGuard {
        fn drop(&mut self) {
            unsafe {
                timeEndPeriod(self.period_ms);
            }
        }
    }

    /// RAII Guard that temporarily enforces Per-Monitor DPI awareness on the current thread.
    pub struct ThreadDpiGuard {
        previous: Option<DPI_AWARENESS_CONTEXT>,
    }

    impl ThreadDpiGuard {
        pub fn per_monitor() -> Self {
            let previous = unsafe { SetThreadDpiAwarenessContext(DPI_AWARENESS_CONTEXT_PER_MONITOR_AWARE) };
            Self {
                previous: if !previous.is_null() { Some(previous) } else { None },
            }
        }
    }

    impl Drop for ThreadDpiGuard {
        fn drop(&mut self) {
            if let Some(prev) = self.previous.take() {
                unsafe {
                    SetThreadDpiAwarenessContext(prev);
                }
            }
        }
    }

    /// Cached virtual desktop dimensions for zero-overhead coordinate normalization in hot loops.
    #[derive(Debug, Clone, Copy)]
    struct VirtualDesktop {
        x_virt: i32,
        y_virt: i32,
        width_minus_one: i64,
        height_minus_one: i64,
    }

    impl VirtualDesktop {
        fn capture() -> Self {
            let _dpi_guard = ThreadDpiGuard::per_monitor();
            let x_virt = unsafe { GetSystemMetrics(SM_XVIRTUALSCREEN) };
            let y_virt = unsafe { GetSystemMetrics(SM_YVIRTUALSCREEN) };
            let cx_virt = unsafe { GetSystemMetrics(SM_CXVIRTUALSCREEN) };
            let cy_virt = unsafe { GetSystemMetrics(SM_CYVIRTUALSCREEN) };

            let cx = if cx_virt > 0 { cx_virt } else { unsafe { GetSystemMetrics(SM_CXSCREEN) } };
            let cy = if cy_virt > 0 { cy_virt } else { unsafe { GetSystemMetrics(SM_CYSCREEN) } };

            Self {
                x_virt,
                y_virt,
                width_minus_one: (cx as i64 - 1).max(1),
                height_minus_one: (cy as i64 - 1).max(1),
            }
        }

        /// Maps physical screen coordinates [0, width-1] to Win32 normalized range [0, 65535].
        #[inline]
        fn normalize(&self, x: i32, y: i32) -> (i32, i32) {
            let dx = (x - self.x_virt) as i64;
            let dy = (y - self.y_virt) as i64;
            let norm_x = (dx * 65535 + (self.width_minus_one / 2) * dx.signum()) / self.width_minus_one;
            let norm_y = (dy * 65535 + (self.height_minus_one / 2) * dy.signum()) / self.height_minus_one;
            (norm_x as i32, norm_y as i32)
        }
    }

    /// Windows Win32 SendInput based mouse input injector.
    #[derive(Debug, Default)]
    pub struct WindowsInjector;

    impl WindowsInjector {
        pub fn new() -> Self {
            Self
        }

        fn send_mouse_input(flags: u32, mouse_data: u32, dx: i32, dy: i32) -> Result<()> {
            let input = INPUT {
                r#type: INPUT_MOUSE,
                Anonymous: INPUT_0 {
                    mi: MOUSEINPUT {
                        dx,
                        dy,
                        mouseData: mouse_data,
                        dwFlags: flags,
                        time: 0,
                        dwExtraInfo: 0,
                    },
                },
            };

            let sent = unsafe { SendInput(1, &input, size_of::<INPUT>() as i32) };
            if sent == 1 {
                Ok(())
            } else {
                let err = std::io::Error::last_os_error();
                Err(WindActError::InjectionError(format!(
                    "SendInput failed: {err}"
                )))
            }
        }

        fn send_key_input(flags: u32, vk: u16, scan: u16) -> Result<()> {
            let input = INPUT {
                r#type: INPUT_KEYBOARD,
                Anonymous: INPUT_0 {
                    ki: KEYBDINPUT {
                        wVk: vk,
                        wScan: scan,
                        dwFlags: flags,
                        time: 0,
                        dwExtraInfo: 0,
                    },
                },
            };

            let sent = unsafe { SendInput(1, &input, size_of::<INPUT>() as i32) };
            if sent == 1 {
                Ok(())
            } else {
                let err = std::io::Error::last_os_error();
                Err(WindActError::InjectionError(format!(
                    "SendInput keyboard failed: {err}"
                )))
            }
        }

        fn keycode_to_vk(key: KeyCode) -> Result<(u16, bool)> {
            let pair = match key {
                KeyCode::Enter => (VK_RETURN, false),
                KeyCode::Tab => (VK_TAB, false),
                KeyCode::Space => (VK_SPACE, false),
                KeyCode::Backspace => (VK_BACK, false),
                KeyCode::Escape => (VK_ESCAPE, false),
                KeyCode::Delete => (VK_DELETE, true),
                KeyCode::Home => (VK_HOME, true),
                KeyCode::End => (VK_END, true),
                KeyCode::PageUp => (VK_PRIOR, true),
                KeyCode::PageDown => (VK_NEXT, true),
                KeyCode::Left => (VK_LEFT, true),
                KeyCode::Right => (VK_RIGHT, true),
                KeyCode::Up => (VK_UP, true),
                KeyCode::Down => (VK_DOWN, true),
                KeyCode::Ctrl => (VK_CONTROL, false),
                KeyCode::Alt => (VK_MENU, false),
                KeyCode::Shift => (VK_SHIFT, false),
                KeyCode::Win => (VK_LWIN, true),
                KeyCode::F(n) if (1..=12).contains(&n) => (VK_F1 + (n as u16 - 1), false),
                KeyCode::Char(c) if c.is_ascii_alphanumeric() => {
                    (c.to_ascii_uppercase() as u16, false)
                }
                _ => {
                    return Err(WindActError::InvalidInput(format!(
                        "Key {key:?} cannot be mapped to a Win32 virtual key"
                    )));
                }
            };
            Ok(pair)
        }

        fn send_vk(&self, key: KeyCode, is_up: bool) -> Result<()> {
            let (vk, is_ext) = Self::keycode_to_vk(key)?;
            let layout = unsafe { GetKeyboardLayout(GetWindowThreadProcessId(GetForegroundWindow(), std::ptr::null_mut())) };
            let scan = unsafe { MapVirtualKeyExW(vk as u32, MAPVK_VK_TO_VSC_EX, layout) } as u16;
            let mut flags = if is_ext { KEYEVENTF_EXTENDEDKEY } else { 0 };
            if is_up {
                flags |= KEYEVENTF_KEYUP;
            }
            Self::send_key_input(flags, vk, scan)
        }
    }

    impl InputInjector for WindowsInjector {
        fn move_cursor(&self, x: i32, y: i32) -> Result<()> {
            let desktop = VirtualDesktop::capture();
            let (norm_x, norm_y) = desktop.normalize(x, y);
            let flags = MOUSEEVENTF_MOVE | MOUSEEVENTF_ABSOLUTE | MOUSEEVENTF_VIRTUALDESK;
            Self::send_mouse_input(flags, 0, norm_x, norm_y)
        }

        fn press_button(&self, button: MouseButton) -> Result<()> {
            let flag = match button {
                MouseButton::Left => MOUSEEVENTF_LEFTDOWN,
                MouseButton::Right => MOUSEEVENTF_RIGHTDOWN,
                MouseButton::Middle => MOUSEEVENTF_MIDDLEDOWN,
            };
            Self::send_mouse_input(flag, 0, 0, 0)
        }

        fn release_button(&self, button: MouseButton) -> Result<()> {
            let flag = match button {
                MouseButton::Left => MOUSEEVENTF_LEFTUP,
                MouseButton::Right => MOUSEEVENTF_RIGHTUP,
                MouseButton::Middle => MOUSEEVENTF_MIDDLEUP,
            };
            Self::send_mouse_input(flag, 0, 0, 0)
        }

        fn click_button_cancellable(
            &self,
            button: MouseButton,
            hold_ms: u32,
            cancel: Option<&AtomicBool>,
        ) -> Result<()> {
            self.press_button(button)?;
            let completed = if hold_ms > 0 {
                hybrid_sleep_cancellable(Duration::from_millis(hold_ms as u64), cancel)
            } else {
                true
            };
            // Always release button even if aborted mid-click
            self.release_button(button)?;
            if completed {
                Ok(())
            } else {
                Err(WindActError::Aborted)
            }
        }

        fn scroll(&self, delta: i32, axis: ScrollAxis) -> Result<()> {
            let wheel_delta = delta
                .checked_mul(WHEEL_DELTA as i32)
                .ok_or_else(|| WindActError::InvalidInput("Scroll delta overflow".into()))?;
            let flag = match axis {
                ScrollAxis::Vertical => MOUSEEVENTF_WHEEL,
                ScrollAxis::Horizontal => MOUSEEVENTF_HWHEEL,
            };
            Self::send_mouse_input(flag, wheel_delta as u32, 0, 0)
        }

        fn play_trajectory_cancellable(
            &self,
            points: &[TrajectoryPoint],
            cancel: Option<&AtomicBool>,
        ) -> Result<()> {
            if points.is_empty() {
                return Ok(());
            }

            // Capture virtual desktop bounds once and enforce 1ms multimedia timer period
            let desktop = VirtualDesktop::capture();
            let _timer_guard = PrecisionTimerGuard::new(1);
            let flags = MOUSEEVENTF_MOVE | MOUSEEVENTF_ABSOLUTE | MOUSEEVENTF_VIRTUALDESK;

            for point in points {
                if cancel.is_some_and(|flag| flag.load(Ordering::Relaxed)) {
                    return Err(WindActError::Aborted);
                }
                if point.delay_ms > 0
                    && !hybrid_sleep_cancellable(Duration::from_millis(point.delay_ms as u64), cancel)
                {
                    return Err(WindActError::Aborted);
                }
                let (norm_x, norm_y) = desktop.normalize(point.x, point.y);
                Self::send_mouse_input(flags, 0, norm_x, norm_y)?;
            }

            Ok(())
        }

        fn current_cursor_pos(&self) -> Result<Point> {
            let _dpi_guard = ThreadDpiGuard::per_monitor();
            let mut pt = POINT { x: 0, y: 0 };
            if unsafe { GetCursorPos(&mut pt) } != 0 {
                Ok(Point::new(pt.x as f64, pt.y as f64))
            } else {
                let err = std::io::Error::last_os_error();
                Err(WindActError::InjectionError(format!("GetCursorPos failed: {err}")))
            }
        }

        fn key_down(&self, key: KeyCode) -> Result<()> {
            self.send_vk(key, false)
        }

        fn key_up(&self, key: KeyCode) -> Result<()> {
            self.send_vk(key, true)
        }

        fn type_char_cancellable(
            &self,
            ch: char,
            hold_ms: u32,
            cancel: Option<&AtomicBool>,
        ) -> Result<()> {
            match ch {
                '\n' => {
                    self.key_down(KeyCode::Enter)?;
                    let ok = hybrid_sleep_cancellable(Duration::from_millis(hold_ms as u64), cancel);
                    self.key_up(KeyCode::Enter)?;
                    return if ok { Ok(()) } else { Err(WindActError::Aborted) };
                }
                '\r' => return Ok(()),
                '\t' => {
                    self.key_down(KeyCode::Tab)?;
                    let ok = hybrid_sleep_cancellable(Duration::from_millis(hold_ms as u64), cancel);
                    self.key_up(KeyCode::Tab)?;
                    return if ok { Ok(()) } else { Err(WindActError::Aborted) };
                }
                '\0' => {
                    return Err(WindActError::InvalidInput(
                        "Text cannot contain null bytes".into(),
                    ));
                }
                _ => {}
            }

            let mut buf = [0u16; 2];
            let units = ch.encode_utf16(&mut buf);
            for &code_unit in units.iter() {
                Self::send_key_input(KEYEVENTF_UNICODE, 0, code_unit)?;
            }
            let ok = if hold_ms > 0 {
                hybrid_sleep_cancellable(Duration::from_millis(hold_ms as u64), cancel)
            } else {
                true
            };
            for &code_unit in units.iter().rev() {
                Self::send_key_input(KEYEVENTF_UNICODE | KEYEVENTF_KEYUP, 0, code_unit)?;
            }
            if ok { Ok(()) } else { Err(WindActError::Aborted) }
        }
    }
}

// ---------------------------------------------------------------------------
// Non-Windows Mock Implementation (for testing/cross-compilation)
// ---------------------------------------------------------------------------

#[cfg(not(windows))]
pub mod mock {
    use super::*;
    use std::sync::atomic::AtomicI32;

    #[derive(Debug, Default)]
    pub struct MockInjector {
        last_x: AtomicI32,
        last_y: AtomicI32,
    }

    impl MockInjector {
        pub fn new() -> Self {
            Self::default()
        }
    }

    impl InputInjector for MockInjector {
        fn move_cursor(&self, x: i32, y: i32) -> Result<()> {
            self.last_x.store(x, Ordering::Relaxed);
            self.last_y.store(y, Ordering::Relaxed);
            Ok(())
        }

        fn press_button(&self, _button: MouseButton) -> Result<()> {
            Ok(())
        }

        fn release_button(&self, _button: MouseButton) -> Result<()> {
            Ok(())
        }

        fn click_button_cancellable(
            &self,
            _button: MouseButton,
            hold_ms: u32,
            cancel: Option<&AtomicBool>,
        ) -> Result<()> {
            let completed = if hold_ms > 0 {
                hybrid_sleep_cancellable(Duration::from_millis(hold_ms as u64), cancel)
            } else {
                true
            };
            if completed {
                Ok(())
            } else {
                Err(WindActError::Aborted)
            }
        }

        fn scroll(&self, _delta: i32, _axis: ScrollAxis) -> Result<()> {
            Ok(())
        }

        fn play_trajectory_cancellable(
            &self,
            points: &[TrajectoryPoint],
            cancel: Option<&AtomicBool>,
        ) -> Result<()> {
            for point in points {
                if cancel.is_some_and(|flag| flag.load(Ordering::Relaxed)) {
                    return Err(WindActError::Aborted);
                }
                if point.delay_ms > 0
                    && !hybrid_sleep_cancellable(Duration::from_millis(point.delay_ms as u64), cancel)
                {
                    return Err(WindActError::Aborted);
                }
                self.move_cursor(point.x, point.y)?;
            }
            Ok(())
        }

        fn current_cursor_pos(&self) -> Result<Point> {
            Ok(Point::new(
                self.last_x.load(Ordering::Relaxed) as f64,
                self.last_y.load(Ordering::Relaxed) as f64,
            ))
        }

        fn key_down(&self, _key: KeyCode) -> Result<()> {
            Ok(())
        }

        fn key_up(&self, _key: KeyCode) -> Result<()> {
            Ok(())
        }

        fn type_char_cancellable(
            &self,
            _ch: char,
            hold_ms: u32,
            cancel: Option<&AtomicBool>,
        ) -> Result<()> {
            let ok = if hold_ms > 0 {
                hybrid_sleep_cancellable(Duration::from_millis(hold_ms as u64), cancel)
            } else {
                true
            };
            if ok { Ok(()) } else { Err(WindActError::Aborted) }
        }
    }
}

/// Creates the default platform input injector.
#[cfg(windows)]
pub fn default_injector() -> windows::WindowsInjector {
    windows::WindowsInjector::new()
}

/// Creates the default platform input injector (mock for non-Windows).
#[cfg(not(windows))]
pub fn default_injector() -> mock::MockInjector {
    mock::MockInjector::new()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_hybrid_sleep_accuracy() {
        let target = Duration::from_millis(5);
        let start = Instant::now();
        hybrid_sleep(target);
        let elapsed = start.elapsed();
        assert!(elapsed >= target, "Elapsed {elapsed:?} should be at least target {target:?}");
        assert!(elapsed < target + Duration::from_millis(5), "Elapsed {elapsed:?} exceeded threshold");
    }

    #[test]
    fn test_cancellable_sleep_aborts_immediately() {
        let cancel = AtomicBool::new(true);
        let start = Instant::now();
        let completed = hybrid_sleep_cancellable(Duration::from_millis(200), Some(&cancel));
        assert!(!completed);
        assert!(start.elapsed() < Duration::from_millis(10));
    }

    #[test]
    fn test_injector_creation() {
        let injector = default_injector();
        let pos = injector.current_cursor_pos();
        match pos {
            Ok(pt) => println!("Cursor position: ({}, {})", pt.x, pt.y),
            Err(e) => println!("Cursor position not available in headless environment: {e}"),
        }
    }
}
