# Robot Control Architecture

This document outlines the firmware architecture and control approach for the NewmaniaSampleBot. 

## 1. System Overview
The robot is controlled by an ESP32-S3 microcontroller using an asymmetric dual-core architecture:
- **Core 0 (Network & Parsing):** Handles Wi-Fi, asynchronous UDP receiving, and telemetry broadcasting. It runs asynchronously, typically processing network events and sending `STATUS` messages at 10 Hz (`100ms` intervals).
- **Core 1 (Motion & Kinematics):** Dedicated to the high-frequency motor control loop, running at a rigid target of ~333 Hz (`3ms` interval). It handles motion queue processing, trajectory profiling, and dead-reckoning integration.

State synchronization between cores is strictly managed via a FreeRTOS mutex (`stateMutex`).

## 2. Communication Protocol (UDP)
The robot is remote-controlled via a custom UDP protocol using short ASCII strings. This design minimizes parsing overhead while remaining human-readable.

**Key Commands:**
- `M` / `D`: Move / Move Duration in a specific direction (Supports optional `[:<action>]` servo parameter).
- `W`: Waypoint navigation to absolute coordinates `(x, y)` (Supports optional `[:<action>]` servo parameter. Repeated commands within a 15mm tolerance of the active target are deduplicated).
- `T` / `O`: Rotate to target angle / Rotate for duration.
- `V`: Direct velocity control (`vx`, `vy`, `omega`) with a timeout.
- `C`: Camera position updates from the phone.
- `P` / `Q`: Ping / Pong for measuring network RTT and clock synchronization.
- `E`: Execute standalone Servo Action (Supports complex sequencing like `LOWER_LEFT`, `LOWER_RIGHT`, `UPPER_LEFT`, `UPPER_RIGHT`, and discrete primitives like `GRABBER_LEFT`, `SLIDER_UP`, `ARM_DOWN`, etc.).
- `U`: Set Serial Monitor (`U:<0|1>`). Opt-in to stream debug logs back to the phone.
- `X`: Mission sequence DONE signal. Triggers visual 4-blink LED feedback on the robot.
- `A`: Immediate emergency stop (ABORT).

Telemetry is sent periodically back to the phone via `STATUS` messages, which includes the estimated `(x, y, angle)`, motion queue length, current velocity, and drift. Additionally, if the Serial Monitor is enabled via the `U` command, the robot streams asynchronous `L` (Log) messages across UDP containing live debugging strings and system state right to the phone interface.

## 3. Motion Queue and Trajectory Profiling
The `MotionQueue` subsystem receives high-level commands and sequences them into `MotionSegment` objects.
- **Continuous Execution:** Segments are queued (up to 16) and executed back-to-back. The robot does not stop between contiguous segments.
- **S-Curve Profiling:** For smooth acceleration and deceleration, the queue employs a jerk-limited S-Curve profiler. The linear parameters are rigidly tuned to `250 mm/s²` peak acceleration and `1200 mm/s³` peak jerk. This guarantees smooth trapezoidal acceleration that respects the robot's physical traction limits.
- **Predictive Steering & Braking:** The queue implements a Lookahead predictor (`0.15s` or adaptive based on speed) to anticipate drift/momentum. When arriving at a target, a predictive physics-based brake triggers and scales back kinetic energy (assuming a `200 mm/s²` decel capability) to prevent overshoot.

## 4. Motor Kinematics and Actuation
The robot employs a four-wheel Mecanum drive.
- **Dead-Zone Compensation:** The ESP32's PWM duties are scaled linearly against calibrated ranges (`duty = v * 43.017 + 57.165`) to bypass hardware dead-zones. Absolute theoretical speed limits trace up to `72.5 mm/s` (Fast), `43.5 mm/s` (Normal), and `29 mm/s` (Slow).
- **Kickstart:** To overcome static friction, an immediate power boost (duty `70`) is applied across `3` frames (~9ms) whenever a motor abruptly breaks static hold.
- **Precision Modes:** 
  - *Low-Speed Mode:* For speeds `< 10 mm/s`, kickstart is disabled and a lower absolute duty floor is utilized to prevent wiggling and allow smooth crawling.
  - *Precision Mode:* When `< 50mm` from a target coordinate, the robot transitions to a finer duty floor. Instead of completely disabling kickstart, it utilizes a dynamic "induction kickstart" which applies a brief power boost only if motor stall is detected (observed speed ≈ 0 while commanded speed > 0.05), ensuring reliable sub-millimeter positioning without unnecessary oscillation.
  - *Single-Motor Mode (Micro-steps):* When `< 15mm` from a target coordinate, only the single most optimally aligned wheel is fired at minimum duty to produce the smallest physical displacement. For ultra-short adjustments (`< 8mm`), the power floor is further restricted to the minimal reliable start threshold, enabling extreme micro-adjustments (~1-2mm) without overshooting.
- **Anti-Slip / Stall Detection:** If the observed velocity falls below `5 mm/s` while the commanded speed remains over `20 mm/s` for 25 control ticks (~75ms), the firmware detects a stall/slip. It applies an immediate `1.35x` multiplicative power boost (up to 40 ticks) to break free from the obstacle or friction trap.
- **Drift Trim:** The translation outputs are pre-rotated slightly depending on speed to compensate for systematic hardware skew.

## 5. Dead Reckoning and Latency-Compensated Vision
Rather than reacting abruptly to real-time observations, the system relies fundamentally on internal Dead-Reckoning, corrected smoothly by sporadic vision updates from an external tracker (an Android phone).

1. **Dead Reckoning (`DeadReckoning`)**: Integrates commanded and estimated velocities at each control loop interval (3ms) to confidently track position. A 1D Kalman Filter (per axis, Process Noise: `0.5`, Velocity: `50.0`) fuses raw motor velocity vectors to reject anomalies and provide smooth kinematic estimations.
2. **Clock Offset (`PING`/`PONG`)**: The robot evaluates precise ping RTTs every `500ms` against the phone to synchronize its event timeline down to the millisecond.
3. **Latency Compensation (`LatencyCompensator`)**: 
   - When a camera update (`CAM`) arrives, it is predictably assumed to bear an inherent ~`150ms` processing/network latency.
   - It cross-references its `clockOffset` against its historical buffer to find where it *thought* it was at the exact instance the timeframe was captured.
   - The difference between the visual observation and the historical estimate becomes the measured drift. If the drift is > `5.0mm`, an active correction triggers.
   - This drift error is then blended forward into the active integration over a `200ms` interpolation window using a smooth correction policy. This gracefully bends the robot's coordinate system toward absolute truth without triggering erratic motor jumps.

## 6. Physical Specifications & Kinematic Limits
Based on hardware measurements, the system accounts for two distinct physical profiles for each robot configuration:

### Robot A (Standard Configuration)
- **Mass / Weight:** ~460 grams (0.460 kg)
- **Actuation Force (Vertical Translation):** ~3.79 Newtons
- **Static Friction Coefficient:** ~0.847
- **Theoretical Max Acceleration (Motor Limit):** ~8.24 m/s² (8242 mm/s²) derived via `a = F/m`.
- **Traction Limit (Slip Boundary):** ~8.31 m/s² (8308 mm/s²) derived via `a_slip = µ * g`.

*Kinematic Implication:* The traction limit (8308 mm/s²) for Robot A is *higher* than its motor limit (8242 mm/s²). This means Robot A is remarkably traction-dominant; the motors are physically incapable of outputting enough torque to break traction via acceleration alone on its baseline surface.

### Robot B (Heavy Configuration)
- **Mass / Weight:** ~926.3 grams (0.926 kg)
- **Actuation Force:** Unmeasured / Variable
- **Static Friction Force:** ~3.38 Newtons
- **Static Friction Coefficient:** ~0.372
- **Traction Limit (Slip Boundary):** ~3.65 m/s² (3652 mm/s²) derived via `a_slip = µ * g`.

*Kinematic Implication:* Robot B has significantly less grip relative to its mass. Its traction limit is harshly capped at `~3652 mm/s²`, meaning applying too much acceleration will easily cause the wheels to stall and slip.

### S-Curve Tuning Strategy (Universal)
Consequently, to guarantee a unified and reliable dead-reckoning engine across both robots without risking acceleration-induced wheel slip, the operational bounds in the firmware's S-Curve profiler (such as `maxAccel` and `maxDecel`) must be rigidly capped. They must be tuned safely below the lowest common traction limit across the fleet (Robot B's ~3652 mm/s² threshold). Enforcing this universal kinematic constraint prevents motor slip and ensures precise dead-reckoning displacement mapping regardless of the active bot ID.
