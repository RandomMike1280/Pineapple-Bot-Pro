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
- `M` / `D`: Move / Move Duration in a specific direction.
- `W`: Waypoint navigation to absolute coordinates `(x, y)`.
- `T` / `O`: Rotate to target angle / Rotate for duration.
- `V`: Direct velocity control (`vx`, `vy`, `omega`) with a timeout.
- `C`: Camera position updates from the phone.
- `P` / `Q`: Ping / Pong for measuring network RTT and clock synchronization.
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
  - *Single-Motor Mode (Micro-steps):* When < 15mm from a target coordinate, only the single most optimal wheel is fired at minimum duty. This allows extreme micro-adjustments (~1-2mm) without overshooting.
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
Based on hardware measurements, the robot possesses the following physical profile:
- **Mass / Weight:** ~460 grams (0.460 kg)
- **Actuation Force (Vertical Translation):** ~3.44 Newtons
- **Static Friction Coefficient:** ~0.353
- **Theoretical Max Acceleration:** ~7.48 m/s² (7479 mm/s²) derived via `a = F/m`.
- **Time to Max Speed:** ~10ms derived via `t = v/a`. Knowing max speed is 72.5 mm/s.

This establishes an absolute physical cap on the robot's acceleration capability. While the theoretical limit is extremely high (~7479 mm/s²), the operational acceleration bounds in the firmware's S-Curve profiler (such as `maxAccel` and `maxDecel`) must be deliberately tuned well below this threshold. This prevents motor stall, mitigates excessive wheel slip given the friction coefficient, and ensures reliable dead-reckoning.
