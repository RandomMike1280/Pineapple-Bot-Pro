# Robot Control Architecture Documentation

## 1. System Overview

The Pineapple-Bot-Pro is a dead-reckoning robot control system consisting of:
- **ESP32-S3 Firmware** (C++/PlatformIO): Real-time motor control and motion planning
- **Android Control App** (Kotlin): Vision processing, navigation planning, and high-level control
- **UDP Communication Protocol**: Low-latency robot↔phone communication over WiFi

### Key Design Principles
- **Phone-as-Brain**: The Android device handles vision (OpenCV ArUco marker tracking) and navigation decisions
- **Firmware-as-Muscles**: The ESP32 executes motion commands with real-time precision (~333Hz control loop)
- **Dead Reckoning + Visual Correction**: Time-based position integration with latency-compensated camera corrections
- **Back-to-Back Motion Queue**: Seamless segment chaining without stopping between commands

---

## 2. Communication Architecture

### 2.1 UDP Protocol
All messages are short ASCII strings on port 4210:

| Type | Format | Direction | Description |
|------|--------|-----------|-------------|
| **HELLO** | `H:<robot_id>:<ip>` | Robot → Phone | Auto-discovery beacon (every 3s) |
| **REGISTER** | `R:<id>:<caps>:<ip>` | Robot → Phone | Initial registration with capabilities |
| **STATUS** | `S:<y>:<x>:<angle>:<queue>:<drift>:<vy>:<vx>` | Robot → Phone | Telemetry (every 100ms) |
| **PING/PONG** | `P:<timestamp>` / `Q:<ts>:<phone_ts>` | Bidirectional | RTT measurement & clock sync |
| **LOG** | `L:<S\|U\|I>:[id]:<msg>` | Robot → Phone | Serial monitor forwarding |
| **MOVE** | `M:<dir>:<dist_mm>:<speed>:<policy>` | Phone → Robot | Directional move command |
| **MOVE_DUR** | `D:<dir>:<duration_ms>:<speed>:<policy>` | Phone → Robot | Time-based move burst |
| **WAYPOINT** | `W:<x>:<y>:[<angle>]:<speed>:<policy>:[servo]` | Phone → Robot | Absolute coordinate target |
| **ROTATE** | `T:<target_angle>:<speed>:<policy>` | Phone → Robot | Absolute heading command |
| **ROT_DUR** | `O:<cw\|ccw>:<duration_ms>:<speed>:<policy>` | Phone → Robot | Rotation burst (camera-guided) |
| **VELOCITY** | `V:<vx>:<vy>:<omega>:<timeout_ms>` | Phone → Robot | Direct velocity command (PID-driven) |
| **CAM** | `C:<timestamp>:<x>:<y>:<angle>` | Phone → Robot | Camera observation for correction |
| **ABORT** | `A` | Phone → Robot | Emergency stop & queue clear |
| **DONE** | `X` | Phone → Robot | Mission complete notification |

### 2.2 Clock Synchronization
The system uses a "Time-Ago" mechanism with absolute clock synchronization:
- Phone timestamps each camera frame with its local clock
- PING/PONG exchanges maintain an estimated RTT and clock offset
- The `LatencyCompensator` translates phone-time to robot-time for historical position lookup

---

## 3. ESP32 Firmware Architecture

### 3.1 Dual-Core Design
| Core | Task | Priority |
|------|------|----------|
| **Core 0** | UDP processing (commands, camera, broadcasts) | 2 (above idle) |
| **Core 1** | Motor control + dead reckoning (~333Hz) | Default |

A FreeRTOS mutex (`stateMutex`) protects shared state between cores.

### 3.2 Key Subsystems

#### DeadReckoning (`lib/DeadReckoning/`)
Maintains position estimate via time-based velocity integration:
- **Store 1**: Internal Odometer (`_ix`, `_iy`, `_ia`) — continuous integration
- **Store 2**: Global Anchor (`_gx`, `_gy`, `_ga`) — camera-corrected world position
- **Store 3**: Anchor Sync (`_ax`, `_ay`, `_aa`) — odometer at capture time
- **History Ring Buffer**: 256-entry circular buffer for latency-compensated lookups

```cpp
// Position = Anchor + (CurrentOdo - OdoAtCaptureTime)
void getCurrentPosition(float &x, float &y, float &angle) const {
    x = _gx + (_ix - _ax);
    y = _gy + (_iy - _ay);
    angle = _ga + (_ia - _aa);
}
```

#### MotionQueue (`lib/MotionQueue/`)
Converts high-level commands into timed motion segments:
- **S-Curve Profiler**: Jerk-limited velocity shaping (asymmetric accel/decel)
- **Kalman Filter**: 1D Kalman per axis for velocity estimation (replaces EMA)
- **Segment States**: `PENDING` → `ACTIVE` → `HOLDING` → `COMPLETED`
- **Queue Capacity**: 16 segments (MQ_MAX_SEGMENTS)

#### LatencyCompensator (`lib/LatencyCompensator/`)
Handles delayed camera observations:
1. Receives camera timestamp (phone time)
2. Translates to robot-time via clock offset
3. Looks up "where did we think we were at capture time?" from DR history
4. Computes drift: `observed_pos - historical_dr_pos`
5. Applies correction via `setAnchor()` with smooth blending

---

## 4. Motion Control System

### 4.1 Control Loop (~333Hz)
```cpp
void loop() {
    uint32_t now = millis();
    uint32_t dt = now - lastTick;
    if (dt >= CONTROL_LOOP_INTERVAL_MS) {
        lastTick = now;
        
        // 1. Update motion queue (compute target velocity)
        motionQueue.tick(dt, currentX, currentY, currentAngle);
        
        // 2. Apply motors from queue state
        applyMotors();
        
        // 3. Update dead reckoning with executed velocity
        deadReckoning.update(currentVx, currentVy, currentOmega, dt);
    }
}
```

### 4.2 Motor Control Pipeline

#### Step 1: Velocity Extraction
```cpp
float vx, vy, omega;
motionQueue.getCurrentVelocity(vx, vy, omega);
```

#### Step 2: Coordinate Transform (World → Robot)
```cpp
double headingRad = c_angle * (PI / 180.0f);
V = (vx * cosA + vy * sinA) / SPEED_FAST_MM_S;  // Forward component
H = (vx * sinA - vy * cosA) / SPEED_FAST_MM_S;  // Strafe component
```

#### Step 3: Mecanum Kinematics
```cpp
// Motor layout (top view):
//   M4(FL) ---- M1(FR)
//     |            |
//   M3(BL) ---- M2(BR)

// Mirrored formula: H=forward/back, V=strafe left/right
mspeedf[0] = -H - V - A;  // Front Right
mspeedf[1] = -H + V - A;  // Back Right
mspeedf[2] = -H + V + A;  // Front Left
mspeedf[3] = -H - V + A;  // Back Left
```

#### Step 4: Physics Compensation
- **speed_to_motor_duty()**: Compensates for motor dead zone
  ```
  duty = 43.0174 * speed + 57.1650 * sign(speed)
  ```
- **Kickstart**: Brief high-duty pulse (3 frames @ 70%) to overcome static friction
- **Normalization**: Clamp to [-100, 100], then apply DRIVE_CLAMP_LOW floor
- **Drift Trim**: Speed-dependent rotation correction (CCW trim for leftward bias)

### 4.3 Advanced Features

#### S-Curve Trajectory Profile
```cpp
struct SCurveProfile {
    float maxAccel;      // 250 mm/s² (ramp-up)
    float maxDecel;      // 500 mm/s² (ramp-down — 2x accel)
    float maxJerk;       // 1200 mm/s³
    float maxDecelJerk;  // 3000 mm/s³ (2.5x for aggressive braking)
};
```
- Jerk-limited acceleration smoothing
- Asymmetric limits: faster deceleration than acceleration
- Bypassed in settling band (within 4× tolerance) for instant speed changes

#### Predictive Steering
Uses Kalman-estimated velocity to anticipate drift:
```cpp
float lookahead = base + speed * gain;  // adaptive lookahead
predictedX = currentX + estVx * lookahead;
predictedY = currentY + estVy * lookahead;
// Steer toward target from predicted position
```

#### Predictive Braking
Physics-based overshoot prevention using `v² = 2*a*d`:
```cpp
float stoppingDistance = (obsSpeed * obsSpeed) / (2 * decel) * safetyFactor;
if (stoppingDistance > remainingDistance) {
    speedScale *= remainingDistance / stoppingDistance;  // slow down
}
```

#### Active Braking Systems
| Type | Trigger | Action |
|------|---------|--------|
| **Rotation Brake** | Rotation-only segment ends, \|ω\| > 5°/s | 5 frames reverse thrust @ 75% duty |
| **Translation Brake** | Zero velocity commanded, coasting > 5mm/s | 4 frames reverse thrust @ 55% duty |

#### Anti-Slip / Stall Detection
```cpp
if (commandedSpeed > 15mm/s && observedSpeed < 5mm/s for 25 ticks) {
    apply 1.35x speed boost for up to 40 ticks;
}
```

### 4.4 Settling Band (Precision Mode)
When within 4× tolerance of target:
- Aggressive linear ramp (0.15 factor) instead of S-curve
- Heading stabilization gain tapers DOWN to 40% (was UP to 150%)
- Max stabilization omega tapers in close approach
- Velocity gate on completion: must be within tolerance AND speed < precisionMin × 1.5
- Slip detection disabled
- Low-speed motor mode: no kickstart, no DRIVE_CLAMP_LOW re-inflation

---

## 5. Position Estimation & Correction

### 5.1 Two-Store Architecture
The system maintains two parallel position estimates:

| Store | Purpose | Updates |
|-------|---------|---------|
| **Internal Odometer** | Continuous integration of commanded velocity | Every control tick |
| **Global Anchor** | Camera-corrected world position | On camera update |

### 5.2 Camera Correction Flow
1. Phone detects ArUco marker → sends `C:<timestamp>:<x>:<y>:<angle>`
2. Robot receives message (with network latency)
3. `LatencyCompensator` translates timestamp via clock offset
4. `DeadReckoning` looks up historical odometer position at capture time
5. Computes drift error: `observed - historical_dr`
6. `setAnchor()` updates Global Anchor such that:
   - `NewWorldPos = ObservedWorldPos`
   - Future DR positions smoothly incorporate the correction

### 5.3 Thresholds
| Threshold | Value | Action |
|-----------|-------|--------|
| Drift | 5mm | Apply full correction |
| Emergency | 25mm | Decelerate + aggressive correction |
| Blend Time | 200ms | Smooth correction transition |

---

## 6. Android Control Interface

### 6.1 Control Modes

| Mode | Description | Command Strategy |
|------|-------------|------------------|
| **Tap Target** | User taps field → robot navigates there | `W` (waypoint) commands with live updates |
| **Heading Lock** | Maintain specific heading (e.g., 0°) | `O` (rotation bursts) until within tolerance, then `A` |
| **Course** | Execute multi-waypoint sequence | Continuous navigation with phase management |
| **Square Test** | Legacy 400mm square pattern | Duration-based bursts with camera re-evaluation |

### 6.2 Tap Target Controller
```kotlin
// Arming phase
startTapTarget(robotId, targetXmm, targetYmm, holdAngleDeg)

// Each camera frame (~30Hz)
updateTapTarget(robotId, curX, curY, curAngle) {
    // 1. Check arrival (within tolerance + stable frames)
    // 2. If arrived → send "A" (abort/stop)
    // 3. Else if heading error > 15° → rotation burst "O:cw/ccw:duration:speed:policy"
    // 4. Else → waypoint "W:x:y:slow:live" (preserves heading via firmware stabilization)
}
```

### 6.3 Heading Lock Controller
```kotlin
startHeadingLock(robotId, targetAngleDeg)

updateHeadingLock(robotId, curAngle) {
    // Speed buckets based on |error|:
    //   > 45° → fast (95ms burst / 150ms cooldown)
    //   > 20° → normal (85ms / 175ms)
    //   > 10° → slow (80ms / 185ms)
    //   > 4°  → fine (75ms / 195ms)
    //   > 3°  → finer (60ms / 225ms)
    //   else  → finest (50ms / 255ms)
    // Within 1.25° → send "A" (arrived)
}
```

### 6.4 Continuous Motion Controller
A 50Hz PID-based controller for complex navigation:
- **Phases**: IDLE → ALIGNING → APPROACHING → FINE_ADJUST → BRAKING → ARRIVED
- **Command Strategy**:
  - ALIGNING: `O` (rotation bursts)
  - APPROACHING: `W` (waypoint)
  - FINE_ADJUST: `V` (velocity commands)
  - BRAKING: `V:0:0:0` or `A`

---

## 7. Key Tuning Parameters

### 7.1 Speed Calibration (`main.h`)
| Parameter | Value | Description |
|-----------|-------|-------------|
| SPEED_SLOW_MM_S | 29.0 | Measured from PS2 physics |
| SPEED_NORMAL_MM_S | 43.5 | ~60% of max duty |
| SPEED_FAST_MM_S | 72.5 | Max linear speed |
| SPEED_SLOW_DEG_S | 15.0 | Slow rotation (reduced from 34.56) |
| SPEED_NORMAL_DEG_S | 51.84 | Normal rotation |
| SPEED_FAST_DEG_S | 86.4 | Max rotation speed |

### 7.2 Motion Control
| Parameter | Value | Description |
|-----------|-------|-------------|
| DRIVE_CLAMP_LOW | 65 | Minimum duty to overcome static friction |
| DRIVE_CLAMP_HIGH | 100 | Safety cap |
| KICKSTART_FRAMES | 3 | Duration of friction-break pulse |
| KICKSTART_SPEED | 70 | Kickstart duty level |
| RAMP_DURATION_MS | 75 | Acceleration ramp time |
| RAMP_START_FRACTION | 0.15 | Initial duty fraction |

### 7.3 S-Curve Profile
| Parameter | Value | Description |
|-----------|-------|-------------|
| SCURVE_MAX_ACCEL_MM_S2 | 250 | Max linear acceleration |
| SCURVE_MAX_JERK_MM_S3 | 1200 | Max jerk (accel rate) |
| SCURVE_MAX_ROT_ACCEL_DEG_S2 | 180 | Max rotational acceleration |
| SCURVE_MAX_ROT_JERK_DEG_S3 | 900 | Max rotational jerk |

### 7.4 Precision / Deceleration
| Parameter | Value | Description |
|-----------|-------|-------------|
| DECCEL_DISTANCE_MM | 500 | Start slowing at this distance |
| CLOSE_APPROACH_DISTANCE_MM | 120 | Precision zone entry |
| WAYPOINT_TOLERANCE_MM | 5 | Arrival tolerance |
| ROTATION_TOLERANCE_DEG | 1.5 | Heading tolerance |
| STABILIZATION_GAIN | 2.5 | Corrective ω per degree error |
| MAX_STABILIZATION_OMEGA | 35 | Max correction rotation speed |

### 7.5 Latency Compensation
| Parameter | Value | Description |
|-----------|-------|-------------|
| CAMERA_LATENCY_MS | 150 | Default OpenCV processing delay |
| DRIFT_THRESHOLD_MM | 5.0 | Apply correction above this |
| EMERGENCY_THRESHOLD_MM | 25.0 | Aggressive correction threshold |
| CORRECTION_BLEND_MS | 200 | Smooth transition time |

---

## 8. File Structure

```
NewmaniaSampleBot/
├── src/
│   ├── main.cpp              # Main firmware with dual-core setup
│   ├── main.h                # Tuning parameters and constants
│   └── auto_discovery.cpp/h  # WiFi/UDP setup
├── lib/
│   ├── MotionQueue/          # Motion planning & S-curve profiler
│   ├── DeadReckoning/        # Position estimation & history
│   ├── LatencyCompensator/   # Camera correction handling
│   ├── UdpProtocol/          # Message parsing & building
│   ├── ESP32_Motor_Controller/ # Motor driver interface
│   ├── ESP32_Servo_Controller/ # Servo control
│   ├── ServoControl/         # Servo action coordination
│   ├── Rotation/             # Angle normalization utilities
│   ├── UdpLogger/            # Debug logging over UDP
│   └── PS2 Controller/       # Legacy PS2 input (unused)
└── platformio.ini            # Build configuration

PineappleRobotPro/ (Android)
└── app/src/main/java/com/example/pineapplerobotpro/
    ├── MainActivity.kt       # UI and main app logic
    ├── RobotUdpBridge.kt     # UDP communication & controllers
    ├── PerspectiveWarp.kt    # Camera field calibration
    ├── ArucoDetector.kt      # Marker detection
    ├── ContinuousMotionController.kt # PID navigation
    └── NavigationHistoryManager.kt   # Undo/redo for paths
```

---

## 9. Common Issues & Solutions

### Oscillation Near Target (Wiggle)
**Root Cause**: Kickstart and DRIVE_CLAMP_LOW re-inflating tiny velocities

**Fix** (in `computeMecanumSpeeds`):
```cpp
bool lowSpeedMode = (cmdMag > 0.001f && cmdMag < 10.0f);
if (lowSpeedMode) {
    // Skip kickstart, use lower floor
    // Don't rearm kickstart
}
```

### Overshoot During Rotation
**Fix**: Active rotation brake when rotation-only segment ends:
```cpp
if (!isMoving && lastMotionWasRotationOnly && fabsf(omega) >= 5.0f) {
    applyReverseThrust(rotationBrakeDirection, 75, 5);
}
```

### Drift During Translation
**Fix**: Speed-dependent drift trim (CCW rotation of velocity vector):
```cpp
trim_deg = (speed == SLOW) ? 9.36f : (speed == NORMAL) ? 2.5f : 0.0f;
```

### Stale Command Interference
**Fix**: Same-target optimization in WAYPOINT handler:
```cpp
bool sameTarget = cur && (cur->state == ACTIVE || cur->state == HOLDING) &&
                  fabsf(cur->target_x - msg.target_x) < 15.0f;
if (!sameTarget) { motionQueue.abort(); enqueueNew(); }
```

---

## 10. Development Workflow

### Flashing Firmware
1. Set `BOT_ID` in `main.h` ("A" or "B")
2. Connect ESP32-S3 via USB
3. PlatformIO: Build and Upload
4. Robot auto-connects to phone hotspot

### Android Development
1. Open `PineappleRobotPro` in Android Studio
2. Deploy to phone
3. Grant camera permissions
4. Set field warp corners (perspective calibration)
5. Arm robot and execute commands

### Testing Modes
Uncomment `#define TEST_MODE` in `main.h` to:
- Disable actual motor output
- Print virtual motor commands to Serial
- Blink LED for command visualization

---

*Document Version: 1.0*
*Last Updated: April 2026*
