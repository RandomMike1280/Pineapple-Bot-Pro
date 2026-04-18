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
- `HELLO`: Auto-discovery beacon (Format: `H:<robot_id>:<ip_address>`). Sent by the robot every 3 seconds.
- `REGISTER`: Sent by the robot to register its capabilities (Format: `R:<robot_id>:mecanum4wd:<ip_address>`).
- `M` / `D`: Move / Move Duration in a specific direction (Supports optional `[:<action>]` servo parameter).
- `W`: Waypoint navigation to absolute coordinates `(x, y)`. Supports multiple formats:
  - `W:x:y:angle:speed:policy[:action]` (Full form)
  - `W:x:y:speed:policy` (Short form: angle defaults to `0`).
  - Repeated commands within a 15mm tolerance of the active target are deduplicated).
- `T` / `O`: Rotate to target angle / Rotate for duration.
- `V`: Direct velocity control (`vx`, `vy`, `omega`) with a timeout.
- `C`: Camera position updates from the phone.
- `P` / `Q`: Ping / Pong for measuring network RTT and clock synchronization.
- `E`: Execute standalone Servo Action (Supports complex sequencing like `LOWER_LEFT`, `LOWER_RIGHT`, `UPPER_LEFT`, `UPPER_RIGHT`, and discrete primitives like `GRABBER_LEFT`, `GRABBER_RIGHT`, `GRABBER_CENTER`, `SLIDER_UP`, `ARM_DOWN`, etc.).
- `U`: Set Serial Monitor (`U:<0|1>`). Opt-in to stream asynchronous debug logs (`L` messages) back to the phone.
- `X`: Mission sequence DONE signal. Triggers a visual 4-blink LED feedback sequence on the robot.
- `A`: Immediate emergency stop (ABORT).

Telemetry is sent periodically back to the phone via `STATUS` messages (`S:y:x:angle:qLen:drift`), where `x` and `y` are swapped to match the phone's local coordinate system. Additionally, if the Serial Monitor is enabled via the `U` command, the robot streams asynchronous `L` (Log) messages containing live debugging strings and system state right to the phone interface. Log messages use the type prefix `L:<S|U|I>` where `S` is Static, `U` is Update-in-place, and `I` is Important (Red).

## 3. Motion Queue and Trajectory Profiling
The `MotionQueue` subsystem receives high-level commands and sequences them into `MotionSegment` objects.
- **Continuous Execution & Coasting Bridge:** Segments are queued (up to 16) and executed back-to-back. To prevent stuttering due to network jitter, a 40ms "Coasting Bridge" (`motionCommandGapHoldMs`) holds the previous velocity if the queue momentarily empties between contiguous commands.
- **S-Curve Profiling & Feedforward:** For smooth acceleration and deceleration, the queue employs a jerk-limited S-Curve profiler. It includes velocity and acceleration feedforward compensation (`FEEDFORWARD_KV`, `FEEDFORWARD_KA`) to reduce tracking lag. The linear parameters are tuned to `250 mm/s²` peak acceleration and `1200 mm/s³` peak jerk.
- **Adaptive Lookahead:** Instead of a fixed lookahead, the steering algorithm utilizes an adaptive lookahead ($0.04\text{s}$ to $0.18\text{s}$) that scales with current speed ($lookahead = BASE + speed \times GAIN$). This compensates for momentum more aggressively at high speeds while maintaining stability during slow maneuvers.
- **Predictive Braking:** The queue implements a physics-based overshoot prevention mechanism. It computes the maximum safe entry speed ($v_{safe} = \sqrt{2 \cdot a \cdot d}$) for the remaining distance $d$, assuming a $350\text{ mm/s}^2$ deceleration capability ($a$) and a $1.2\text{x}$ safety margin. If the closing speed exceeds $v_{safe}$, the velocity scale is proactively reduced.
- **Latency-Aware Speed Governor:** To counter network latency staleness (where a "stop" command from the phone may arrive ~150ms late), the robot enforces a hard upper bound on commanded speed. It ensures the robot can always stop within the remaining distance $d$ assuming a $300\text{ mm/s}^2$ latency-aware deceleration capability ($v_{cap} = \sqrt{2 \cdot 300 \cdot d}$).
- **Blended Angle Stabilization:** To prevent "strafe-jumps" or oscillations during heading correction, the motion controller uses a **Blended Angle**. This combines the high-frequency internal Dead-Reckoning angle with the low-frequency Camera Ground-Truth angle, providing a smooth but authoritative heading reference for the PID stabilization loop.

## 4. Motor Kinematics and Actuation
The robot employs a four-wheel Mecanum drive.
- **Torque Balancing & Calibration:** The ESP32's PWM duties are scaled linearly against calibrated ranges (`duty = v * 43.017 + 57.165`) to bypass hardware dead-zones. To compensate for physical torque imbalance, right-side motors (M1, M4) have their duty reduced by a fixed `-3.0` offset, ensuring straight-line travel.
- **Kickstart:** To overcome static friction, an immediate power boost (duty `70`) is applied across `3` frames (~9ms) whenever a motor abruptly breaks static hold.
- **Operating Modes:**
  - *Precision Mode:* When `< 50mm` from a target, the robot enters precision mode. It utilizes **Induction Kickstart**—a conditional boost applied only if the motor is stalled (commanded speed $> 0.05$ while observed speed $< 1.5\text{ mm/s}$), preventing unnecessary wiggle during fine adjustments.
  - *Settling Band:* For the final $20\text{mm}$ of an approach, the firmware switches from a fixed duty floor to a linear ramp-to-zero.
  - *Single-Motor Mode (Micro-steps):* When `< 15mm` from a target and moving slowly ($< 10\text{ mm/s}$), the robot activates **Single-Motor Mode**. It fires only the one wheel most optimally aligned with the desired velocity vector. This enables extreme micro-adjustments (~1-2mm) without overshooting or rotating.
- **Anti-Slip / Stall Detection:** If the observed velocity falls below `5 mm/s` while the commanded speed remains over `20 mm/s` for 25 control ticks (~75ms), the firmware detects a stall. It applies a `1.35x` multiplicative power boost to break free.
- **Active Braking (Reverse Thrust):** When a motion segment ends, the robot applies active reverse thrust (Reverse PWM) to kill momentum:
  - *Rotation:* Reverse duty `75` for `5` frames (~15ms).
  - *Translation:* Reverse duty `80` for `6` frames (~18ms), only if observed speed is $> 5\text{ mm/s}$.
- **Drift Trim:** The translation outputs are pre-rotated slightly to counteract hardware skew (e.g., $9.36^\circ$ for `SLOW` mode).

## 5. Dead Reckoning and Latency-Compensated Vision
Rather than reacting abruptly to real-time observations, the system relies fundamentally on internal Dead-Reckoning, corrected smoothly by sporadic vision updates from an external tracker (an Android phone).

1. **Dead Reckoning (`DeadReckoning`)**: Integrates commanded and estimated velocities at each control loop interval (3ms) to confidently track position. A 1D Kalman Filter (per axis, Process Noise: `0.5` Pos / `50.0` Vel) fuses raw motor velocity vectors to reject anomalies. Dead-reckoning for horizontal strafing is calibrated with a $1.25$ factor, while vertical motion uses $1.0$. A **Stationary Deadzone** of $5.0\text{ mm/s}$ is applied to the Kalman velocity estimate to ensure it zero-centers correctly near the target, preventing stale velocity signals from influencing the braking logic.
2. **Clock Offset (`PING`/`PONG`)**: The robot evaluates precise ping RTTs every `500ms` against the phone to synchronize its event timeline down to the millisecond.
3. **Latency Compensation (`LatencyCompensator`)**: 
   - When a camera update (`CAM`) arrives, it is predictably assumed to bear an inherent ~`150ms` processing/network latency.
   - It cross-references its `clockOffset` against its historical buffer to find where it *thought* it was at the exact instance the timeframe was captured.
   - The difference between the visual observation and the historical estimate becomes the measured drift. If the drift is > `5.0mm`, an active correction triggers.
   - This drift error is then blended forward into the active integration over a `200ms` interpolation window using a smooth correction policy.
   - **Heading Authority**: For motion stabilization, the controller utilizes the **Blended Angle** (Camera Ground-Truth fused with high-frequency DR) to prevent heading jumps and ensure stable straight-line tracking even during correction blends.

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
