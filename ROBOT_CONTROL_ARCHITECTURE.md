# Robot Control Architecture

This document outlines the firmware architecture and control approach for the NewmaniaSampleBot. 

## 1. System Overview
The robot is controlled by an ESP32-S3 microcontroller using a dual-core architecture:
- **Core 0**: Handles Wi-Fi, asynchronous UDP network communication (receiving commands and sending telemetry), and parsing.
- **Core 1**: Dedicated to the high-frequency motor control loop, motion queue processing, and dead-reckoning integration.

State synchronization between cores is managed via a FreeRTOS mutex (`stateMutex`).

## 2. Communication Protocol (UDP)
The robot is remote-controlled via a custom UDP protocol using short ASCII strings. This design minimizes parsing overhead while remaining human-readable.

**Key Commands:**
- `M` / `D`: Move / Move Duration in a specific direction (Supports optional `[:<action>]` servo parameter).
- `W`: Waypoint navigation to absolute coordinates `(x, y)` (Supports optional `[:<action>]` servo parameter).
- `T` / `O`: Rotate to target angle / Rotate for duration.
- `V`: Direct velocity control (`vx`, `vy`, `omega`) with a timeout.
- `C`: Camera position updates from the phone.
- `P` / `Q`: Ping / Pong for measuring network RTT and clock synchronization.
- `E`: Execute standalone Servo Action (`grabber`, `slider`, `arm` configurations).
- `A`: Immediate emergency stop (ABORT).

Telemetry is sent periodically back to the phone via `STATUS` messages, which includes the estimated `(x, y, angle)`, motion queue length, current velocity, and drift.

## 3. Motion Queue and Trajectory Profiling
The `MotionQueue` subsystem receives high-level commands and sequences them into `MotionSegment` objects.
- **Continuous Execution:** Segments are queued (up to 16) and executed back-to-back. The robot does not stop between contiguous segments.
- **S-Curve Profiling:** For smooth acceleration and deceleration, the queue employs an S-Curve profiler. This limits jerk (rate of change of acceleration) to avoid slipping and eliminate abrupt power spikes.
- **Predictive Braking:** When arriving at a target or stopping, an active brake counteracts coasting momentum using commanded reverse thrust.

## 4. Motor Kinematics and Actuation
The robot employs a four-wheel Mecanum drive.
- **Dead-Zone Compensation:** Requested speeds are run through a mapping function (`speed_to_motor_duty`) to bypass hardware dead-zones.
- **Kickstart:** To overcome static friction, a brief power boost is applied whenever a motor first begins moving.
- **Precision Modes:** 
  - *Low-Speed Mode:* For speeds < 10 mm/s, kickstart is disabled and absolute duty floor is utilized to prevent wiggling.
  - *Precision Mode:* When < 50mm from a target coordinate, the robot transitions to a finer duty floor and disables kickstart to prepare for sub-millimeter positioning.
  - *Single-Motor Mode (Micro-steps):* When < 15mm from a target coordinate, only the single most optimal wheel is fired at minimum duty. This allows extreme micro-adjustments (~1-2mm) without overshooting.
- **Anti-Slip / Stall Detection:** If the observed velocity falls below `5 mm/s` while the commanded speed remains over `20 mm/s` for 25 control ticks (~75ms), the firmware detects a stall/slip. It applies an immediate `1.35x` multiplicative power boost (up to 40 ticks) to break free from the obstacle or friction trap.
- **Drift Trim:** The translation outputs are pre-rotated slightly depending on speed to compensate for systematic hardware skew.

## 5. Dead Reckoning and Latency-Compensated Vision
Rather than reacting abruptly to real-time observations, the system relies fundamentally on internal Dead-Reckoning, corrected smoothly by sporadic vision updates from an external tracker (an Android phone).

1. **Dead Reckoning (`DeadReckoning`)**: Integrates commanded and estimated velocities at each control loop interval to confidently track position without external input.
2. **Clock Offset (`PING`/`PONG`)**: The robot periodically synchronizes its clock with the phone to measure Round-Trip Time (RTT) and maintain a time offset.
3. **Latency Compensation (`LatencyCompensator`)**: 
   - When a camera update (`CAM`) arrives, it includes the exact target capture time from the phone.
   - The robot relies on its exact `clockOffset` to check its dead-reckoning history buffer and find where it *thought* it was at that exact timestamp.
   - The difference between the camera's observation and the historical estimate becomes the measured drift.
   - This drift error is then blended forward to the present over ~200ms using a smooth correction policy, gently shifting the robot's coordinate system to ground truth without triggering erratic jumps in motor command.

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
