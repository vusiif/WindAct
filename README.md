# WindAct

WindAct (Wind Physics Action Executor) is a high-performance, human-like mouse trajectory generator and low-level input injector for Windows, written in Rust.

## Core Features

- **Physics-Based Trajectory Planning**: Integrates WindMouse mechanics (gravity, wind, damping, step clipping) with Perlin noise perturbation and velocity-dependent humanized delay distribution.
- **Precision Input Injection**: Leverages Win32 `SendInput` with multi-monitor virtual screen coordinate normalization, DPI awareness, and Windows `timeBeginPeriod(1)` 1ms high-precision timing control.
- **Dual Operating Modes**:
  - **CLI Mode**: Fast single-shot commands with `--dry-run` offline verification.
  - **Stdio Mode**: Persistent JSONL stream communication with atomic `abort` cancellation for orchestrator integration.
- **Customizable Profiles**: Pre-tuned movement profiles including `casual`, `precise`, and anti-rebound `slider` dragging.

## Architecture

```text
CLI / Stdio Interface
        │
        ▼
Profile Manager (casual / precise / slider)
        │
        ▼
TrajectoryPlanner (WindMouse + Perlin Noise)
        │
        ▼
InputInjector (Win32 SendInput + 1ms Precision Timer)
```

## License

MIT OR Apache-2.0
