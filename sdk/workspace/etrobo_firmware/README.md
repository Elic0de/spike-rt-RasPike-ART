# etrobo_firmware

This workspace is the firmware-side port target for the control-critical parts of
`etrobo2026/spike`.

## Responsibility Boundary

- SPIKE firmware owns deterministic decisions: line tracing, PID/PWM, mission
  progression, distance/yaw estimation, stop/brake, and fail-safe behavior.
- Raspberry Pi owns perception and operations: camera capture, QR/object/line
  recognition, logging, diagnostics, and tuning UI.
- Communication carries observations and configuration only. It must not be the
  control loop for motor PWM.

## Porting Order

1. Bring `lib/control` logic into firmware-side modules.
2. Bring `lib/estimation` and `lib/localization` logic after control timing is
   measured.
3. Add a firmware mission runner that mirrors `MissionRunner` and
   `mission/actions`.
4. Keep telemetry files, camera processing, and heavy diagnostics on Raspberry
   Pi.

## Directory Layout

The layout follows the direction of `etrobo2026/spike`.

- `app.cpp`: TOPPERS task entry points.
- `include/app`, `src/app`: firmware application orchestration.
- `include/comm`, `src/comm`: serial frame protocol.
- `include/mission`, `src/mission`: mission boundary and future mission runner.
- `include/lib/control`, `src/lib/control`: deterministic control logic.
- `protocol.h`: shared wire protocol with Raspberry Pi.

## Current State

- `line_controller` is the low-level line trace/PID/PWM module.
- `mission_manager` is the firmware mission boundary. It currently delegates to
  `line_controller`, and is the intended landing point for the future
  `etrobo2026/spike` mission/state port.
- `serial_protocol` accepts config/start/stop/QR/status frames from Raspberry
  Pi.
