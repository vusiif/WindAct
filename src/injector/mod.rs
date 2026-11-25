//! Input injection layer for dispatching hardware/OS events.

use std::thread;
use std::time::{Duration, Instant};

use crate::core::{MouseButton, Point, Result, TrajectoryPoint, WindActError};

/// High-precision sleep combining thread sleep and CPU busy-spin loop.
/// Yields the thread for the bulk of the wait, then spins to achieve sub-millisecond accuracy.
pub fn hybrid_sleep(target_duration: Duration) {
    let start = Instant::now();
    if target_duration > Duration::from_millis(2) {
        thread::sleep(target_duration - Duration::from_millis(1));
    }
    while start.elapsed() < target_duration {
        std::hint::spin_loop();
    }
}

/// Common trait for injecting mouse movement and button events into the OS.
pub trait InputInjector: Send + Sync {
    /// Moves the cursor to physical screen coordinates (x, y).
    fn move_cursor(&self, x: i32, y: i32) -> Result<()>;

    /// Presses down the specified mouse button.
    fn press_button(&self, button: MouseButton) -> Result<()>;

    /// Releases the specified mouse button.
    fn release_button(&self, button: MouseButton) -> Result<()>;

    /// Clicks the specified mouse button with a humanized hold duration in milliseconds.
    fn click_button(&self, button: MouseButton, hold_ms: u32) -> Result<()>;

    /// Plays a sequence of pre-computed trajectory points with precise inter-step timing.
    fn play_trajectory(&self, points: &[TrajectoryPoint]) -> Result<()>;

    /// Retrieves the current physical cursor location.
    fn current_cursor_pos(&self) -> Result<Point>;
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
        SendInput, INPUT, INPUT_0, INPUT_MOUSE, MOUSEINPUT,
        MOUSEEVENTF_ABSOLUTE, MOUSEEVENTF_LEFTDOWN, MOUSEEVENTF_LEFTUP,
        MOUSEEVENTF_MIDDLEDOWN, MOUSEEVENTF_MIDDLEUP, MOUSEEVENTF_MOVE,
        MOUSEEVENTF_RIGHTDOWN, MOUSEEVENTF_RIGHTUP, MOUSEEVENTF_VIRTUALDESK,
    };
    use windows_sys::Win32::UI::WindowsAndMessaging::{
        GetCursorPos, GetSystemMetrics, SM_CXSCREEN, SM_CXVIRTUALSCREEN,
        SM_CYSCREEN, SM_CYVIRTUALSCREEN, SM_XVIRTUALSCREEN, SM_YVIRTUALSCREEN,
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

        fn send_mouse_input(flags: u32, dx: i32, dy: i32) -> Result<()> {
            let input = INPUT {
                r#type: INPUT_MOUSE,
                Anonymous: INPUT_0 {
                    mi: MOUSEINPUT {
                        dx,
                        dy,
                        mouseData: 0,
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
    }

    impl InputInjector for WindowsInjector {
        fn move_cursor(&self, x: i32, y: i32) -> Result<()> {
            let desktop = VirtualDesktop::capture();
            let (norm_x, norm_y) = desktop.normalize(x, y);
            let flags = MOUSEEVENTF_MOVE | MOUSEEVENTF_ABSOLUTE | MOUSEEVENTF_VIRTUALDESK;
            Self::send_mouse_input(flags, norm_x, norm_y)
        }

        fn press_button(&self, button: MouseButton) -> Result<()> {
            let flag = match button {
                MouseButton::Left => MOUSEEVENTF_LEFTDOWN,
                MouseButton::Right => MOUSEEVENTF_RIGHTDOWN,
                MouseButton::Middle => MOUSEEVENTF_MIDDLEDOWN,
            };
            Self::send_mouse_input(flag, 0, 0)
        }

        fn release_button(&self, button: MouseButton) -> Result<()> {
            let flag = match button {
                MouseButton::Left => MOUSEEVENTF_LEFTUP,
                MouseButton::Right => MOUSEEVENTF_RIGHTUP,
                MouseButton::Middle => MOUSEEVENTF_MIDDLEUP,
            };
            Self::send_mouse_input(flag, 0, 0)
        }

        fn click_button(&self, button: MouseButton, hold_ms: u32) -> Result<()> {
            self.press_button(button)?;
            if hold_ms > 0 {
                hybrid_sleep(Duration::from_millis(hold_ms as u64));
            }
            self.release_button(button)
        }

        fn play_trajectory(&self, points: &[TrajectoryPoint]) -> Result<()> {
            if points.is_empty() {
                return Ok(());
            }

            // Capture virtual desktop bounds once and enforce 1ms multimedia timer period
            let desktop = VirtualDesktop::capture();
            let _timer_guard = PrecisionTimerGuard::new(1);
            let flags = MOUSEEVENTF_MOVE | MOUSEEVENTF_ABSOLUTE | MOUSEEVENTF_VIRTUALDESK;

            for point in points {
                if point.delay_ms > 0 {
                    hybrid_sleep(Duration::from_millis(point.delay_ms as u64));
                }
                let (norm_x, norm_y) = desktop.normalize(point.x, point.y);
                Self::send_mouse_input(flags, norm_x, norm_y)?;
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
    }
}

// ---------------------------------------------------------------------------
// Non-Windows Mock Implementation (for testing/cross-compilation)
// ---------------------------------------------------------------------------

#[cfg(not(windows))]
pub mod mock {
    use super::*;
    use std::sync::atomic::{AtomicI32, Ordering};

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

        fn click_button(&self, _button: MouseButton, hold_ms: u32) -> Result<()> {
            if hold_ms > 0 {
                hybrid_sleep(Duration::from_millis(hold_ms as u64));
            }
            Ok(())
        }

        fn play_trajectory(&self, points: &[TrajectoryPoint]) -> Result<()> {
            for point in points {
                if point.delay_ms > 0 {
                    hybrid_sleep(Duration::from_millis(point.delay_ms as u64));
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
    fn test_injector_creation() {
        let injector = default_injector();
        let pos = injector.current_cursor_pos();
        // In interactive desktop sessions, pos is Ok(point). In headless/service runners, it may return AccessDenied.
        match pos {
            Ok(pt) => println!("Cursor position: ({}, {})", pt.x, pt.y),
            Err(e) => println!("Cursor position not available in headless environment: {e}"),
        }
    }
}
