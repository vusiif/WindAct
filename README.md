# WindAct

WindAct (Wind Physics Action Executor) is a high-performance, human-like mouse & keyboard physical action executor for Windows, written in Rust.

## Core Features

- **Physics & Kinematics Trajectory Planners**:
  - `WindMousePlanner`: Gravity, random wind, target damping, overshoot snapback, and low-frequency arc-length-driven Perlin drift.
  - `BezierSplinePlanner`: Cubic Bezier wrist-arc bowing (2-point) and Catmull-Rom splines (multi-waypoint continuous curves).
  - `StrokePlanner` (`draw_strokes`): Two-Thirds Power Law curvature-velocity coupling, low-frequency Perlin drift, landing dwell (`15~35ms`), tangent exit flick, and airborne WindMouse transitions.
- **High-Precision Win32 Input Injection**:
  - `SendInput` with multi-monitor virtual desktop normalization (`0 ~ 65535` via `SM_XVIRTUALSCREEN`) and temporary Per-Monitor DPI awareness.
  - Enforces `1ms` Windows multimedia timer resolution (`timeBeginPeriod(1)`) paired with sub-millisecond `hybrid_sleep` (`thread::sleep` + `spin_loop`).
- **Humanized Keyboard Simulation**:
  - `send-text` / `send_text`: English word-burst typing with post-space cognitive pauses, CJK IME phrase-level chunking (`1~4` chars), and full UTF-16 `KEYEVENTF_UNICODE` support.
  - `key-press` / `key_press`: Staggered modifier key presses (e.g., `ctrl+shift+s`) with guaranteed LIFO release even on `abort`.
- **Single-Line Vector Handwriting (`draw-text`)**:
  - Built-in Hershey ASCII single-line fonts (`romans`, `scripts`) with zero external dependencies.
  - Optional streaming loader for `Make Me a Hanzi` (`graphics.txt`) Chinese stroke centerlines, with per-glyph affine jitter.
- **Dual Operating Modes**:
  - **CLI Mode**: Single-shot commands with `--dry-run` deterministic JSON output.
  - **Stdio Mode (`--stdio`)**: Persistent JSONL stream with dual-thread reader/worker architecture and millisecond-level `abort` cancellation.

## Build & Test

```powershell
# Run unit and integration tests
cargo test

# Build optimized release binary
cargo build --release

# Run built-in 1ms timer jitter & planner throughput benchmarks
.\target\release\windact.exe bench --samples 100 --iterations 1000
```

## CLI Usage

### Mouse Actions

```powershell
# Humanized click (explicit coordinate or 2D Gaussian sampling inside --bbox)
windact click --x 1464 --y 326 --button left --profile casual
windact click --bbox 400,300,120,50 --profile precise --dry-run

# Humanized cursor move
windact move --x 800 --y 500 --profile casual --planner windmouse

# Slider / curved drag (supports 2-point or multi-waypoint --path)
windact drag --from 500,600 --to 760,602 --profile slider
windact drag --path "300,400;450,405;430,520" --profile casual --planner bezier

# Multi-step humanized wheel scroll
windact scroll --delta -5 --axis vertical --steps 3
```

### Handwriting & Keyboard Actions

```powershell
# Handwrite text using built-in Hershey single-line fonts (romans / scripts)
windact draw-text --text "WindAct 2026!" --x 320 --y 380 --size 70 --style scripts

# Handwrite mixed English + Chinese with Make Me a Hanzi graphics.txt
windact draw-text --text "Hello 你好" --x 320 --y 380 --size 70 --hanzi-font path\to\graphics.txt

# Type text with humanized word/IME cadence
windact send-text --text "Hello WindAct! 你好世界" --profile casual

# Press single key or modifier combo
windact key-press --key "ctrl+shift+s"
```

## `--stdio` JSONL Pipeline Protocol

Launch persistent worker process:

```powershell
windact --stdio
```

### Request Examples (1 JSON object per line)

```json
{"id": 1, "op": "move_click", "x": 1464, "y": 326, "button": "left", "profile": "casual"}
{"id": 2, "op": "move_click", "bbox": [400, 300, 80, 40], "profile": "precise"}
{"id": 3, "op": "move", "x": 800, "y": 500, "planner": "bezier"}
{"id": 4, "op": "drag", "from": [500, 600], "to": [760, 602], "profile": "slider"}
{"id": 5, "op": "drag", "path": [[300, 400], [450, 405], [430, 520]], "profile": "casual"}
{"id": 6, "op": "scroll", "delta": -5, "axis": "vertical", "steps": 3}
{"id": 7, "op": "draw_strokes", "strokes": [[[300, 300], [500, 300], [480, 420]]], "base_speed": 1.4}
{"id": 8, "op": "draw_text", "text": "WindAct", "x": 320, "y": 380, "size": 60, "style": "scripts"}
{"id": 9, "op": "send_text", "text": "Hello 你好", "profile": "casual"}
{"id": 10, "op": "key_press", "key": "ctrl+a"}
{"id": 11, "op": "abort"}
```

### Response Format

```json
{"id": 1, "status": "ok", "total_points": 46, "duration_ms": 240}
{"id": 4, "status": "aborted", "duration_ms": 58, "error": "Action aborted by user"}
```

## Architecture

```text
CLI / Stdio JSONL Reader (with instant Abort flag)
        │
        ▼
ActionEngine (Move / Click / Drag / Scroll / DrawStrokes / DrawText / SendText / KeyPress)
        │
        ├──► TrajectoryPlanner (WindMousePlanner / BezierSplinePlanner / StrokePlanner)
        ├──► Single-Line Font Layout (Hershey ASCII + Make Me a Hanzi)
        │
        ▼
InputInjector (Win32 SendInput + 1ms PrecisionTimerGuard + hybrid_sleep_cancellable)
```

## License

MIT OR Apache-2.0
