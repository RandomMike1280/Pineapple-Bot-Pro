// ============================================================================
// Dead-Reckoning Robot Control Firmware
// ============================================================================
// ESP32-S3 firmware for a mecanum-wheeled robot that:
//   1. Accepts move commands over UDP (direction, distance, speed, policy)
//   2. Executes them via a motion queue (back-to-back, no stops)
//   3. Maintains a dead-reckoning position estimate (time × speed)
//   4. Receives camera observations from the phone and applies
//      latency-compensated corrections (smooth, no jumps)
//
// The same firmware runs on both robots — only BOT_ID in main.h differs.
// ============================================================================

#include <WiFi.h>
#include <WiFiUdp.h>
#include "main.h"
#include <dead_reckoning.hpp>
#include <motion_queue.hpp>
#include <latency_compensator.hpp>
#include <udp_protocol.hpp>
#include <ServoControl/servo_control.hpp>

// ============================================================================
// WiFi Configuration
// ============================================================================
const char* ssid     = "IPhone 19 Professional";
const char* password = "sixseven";

// ============================================================================
// Subsystem instances
// ============================================================================
WiFiUDP          udp;
const int        udpPort = 4210;
IPAddress        phoneIP;

DeadReckoning    deadReckoning;
MotionQueue      motionQueue;
LatencyCompensator latencyComp;

// ============================================================================
// Timing state
// ============================================================================
uint32_t lastTick       = 0;
uint32_t lastHelloTime  = 0;
uint32_t lastStatusTime = 0;
uint32_t lastPingTime   = 0;

// ============================================================================
// Latency / Correction Tuning
// ============================================================================
// CAMERA_LATENCY_MS is now DEPRECATED. The robot now uses a "Time-Ago" 
// mechanism where the phone explicitly reports its OpenCV processing delay.
// #define DRIFT_THRESHOLD_MM       20.0f    // apply full correction above this

// ============================================================================
// Dual-core synchronization
// ============================================================================
SemaphoreHandle_t stateMutex;                // protects motionQueue, deadReckoning, latencyComp
volatile bool     abortFlag = false;         // set by Core 0, read by Core 1 for instant motor stop

// Forward declarations for dual-core
void udpTask(void* parameter);
void handleParsedMessage(const UdpMessage &msg);

// ============================================================================
// Current motor velocities (for dead-reckoning integration)
// ============================================================================
float currentVx = 0;  // mm/s
float currentVy = 0;  // mm/s
float currentOmega = 0; // deg/s

// ============================================================================
// PS2 physics-based motor control
// ============================================================================

// Kickstart state per motor
static int mkickstart[4] = {KICKSTART_FRAMES, KICKSTART_FRAMES, KICKSTART_FRAMES, KICKSTART_FRAMES};

static void rearmKickstart() {
    for (int i = 0; i < 4; i++) {
        mkickstart[i] = KICKSTART_FRAMES;
    }
}

static float motorRampFactor = 0;
static bool motorWasMoving = false;
static bool lastMotionWasRotationOnly = false;
static float lastRotationOmegaDegS = 0;
static int rotationBrakeFrames = 0;
static int rotationBrakeDirection = 0;
static bool suppressRotationBrake = false;
static int translationBrakeFrames = 0;
static float translationBrakeDirX = 0;  // normalized direction of observed velocity at brake start
static float translationBrakeDirY = 0;
static uint32_t motionIdleStartTime = 0;
static const uint32_t motionCommandGapHoldMs = 40;
static const float rampIncrement =
    (1.0f - RAMP_START_FRACTION) / (RAMP_DURATION_MS / (float)CONTROL_LOOP_INTERVAL_MS);

static void resetMotionControlState(bool suppressBrake) {
    motorRampFactor = 0;
    motorWasMoving = false;
    lastMotionWasRotationOnly = false;
    lastRotationOmegaDegS = 0;
    rotationBrakeFrames = 0;
    rotationBrakeDirection = 0;
    suppressRotationBrake = suppressBrake;
    translationBrakeFrames = 0;
    translationBrakeDirX = 0;
    translationBrakeDirY = 0;
    rearmKickstart();
    motionIdleStartTime = 0;
}

int sign(double x) {
    return (x > 0) - (x < 0);
}

int speed_to_motor_duty(double speed) {
    if (speed > 0) {
        return MOTOR_MAP_SLOPE * speed + MOTOR_MAP_OFFSET;
    } else if (speed < 0) {
        return MOTOR_MAP_SLOPE * speed - MOTOR_MAP_OFFSET;
    }
    return 0;
}

MecanumSpeeds computeMecanumSpeeds(double V, double H, double A, bool lowSpeedMode) {
    MecanumSpeeds s;
    double mspeedf[4];

    // Mecanum formula with mirrored front/back motors
    // Mirrored layout swaps V↔H: H=forward/back, V=strafe left/right
    // Diagonal pairing: FL+BR share +V, FR+BL share -V
    mspeedf[0] = -H - V - A;  // Front Right
    mspeedf[1] = -H + V - A;  // Back Right
    mspeedf[2] = -H + V + A;  // Front Left
    mspeedf[3] = -H - V + A;  // Back Left

    // Apply speed_to_motor_duty to compensate for motor dead zone
    for (int i = 0; i < 4; i++) {
        if (abs(mspeedf[i]) > 0) {
            mspeedf[i] = speed_to_motor_duty(mspeedf[i]);
        }
    }

    // Normalize duties to [-DRIVE_CLAMP_HIGH, DRIVE_CLAMP_HIGH]
    double maxDuty = 0;
    for (int i = 0; i < 4; i++) {
        maxDuty = max(maxDuty, abs(mspeedf[i]));
    }
    if (maxDuty > DRIVE_CLAMP_HIGH) {
        for (int i = 0; i < 4; i++) {
            mspeedf[i] = mspeedf[i] / maxDuty * DRIVE_CLAMP_HIGH;
        }
    }

    if (lowSpeedMode) {
        // Near the target: skip kickstart and use a much lower duty floor.
        // This allows the robot to actually crawl at tiny velocities without
        // the kickstart/clamp re-inflating them to full duty (which causes wiggle).
        int lowClamp = DRIVE_CLAMP_LOW;  // Same floor — motors won't turn below this. Anti-wiggle comes from no kickstart.
        for (int i = 0; i < 4; i++) {
            double absSpeed = abs(mspeedf[i]);
            if (absSpeed > 0 && absSpeed < lowClamp) {
                mspeedf[i] = sign(mspeedf[i]) * lowClamp;
            }
        }
        // Don't rearm kickstart in low-speed mode
    } else {
        // Kickstart: briefly boost duty to overcome static friction
        // Damping: scale the kickstart power proportional to requested speed
        for (int i = 0; i < 4; i++) {
            double absSpeed = abs(mspeedf[i]);
            if (absSpeed < DRIVE_CLAMP_LOW && absSpeed > 0) {
                mkickstart[i] = KICKSTART_FRAMES;
            }
            if (mkickstart[i] > 0) {
                mkickstart[i]--;
                // Scale kickstart power between [CLAMP_LOW, KICKSTART_SPEED] based on target speed
                double kickPower = DRIVE_CLAMP_LOW + (absSpeed / 100.0) * (KICKSTART_SPEED - DRIVE_CLAMP_LOW);
                mspeedf[i] = sign(mspeedf[i]) * max(kickPower, absSpeed);
            } else if (absSpeed > 0) {
                mspeedf[i] = sign(mspeedf[i]) * max((double)DRIVE_CLAMP_LOW, absSpeed);
            }
        }
    }

    s.m1 = (int)mspeedf[0];  // Front Right
    s.m2 = (int)mspeedf[1];  // Back Right
    s.m3 = (int)mspeedf[2];  // Back Left
    s.m4 = (int)mspeedf[3];  // Front Left
    return s;
}

// ============================================================================
// Apply motor speeds from current motion queue state
// ============================================================================
void applyMotors() {
    // Acceleration ramp state — persists across calls
    float prevCurrentVx = currentVx;
    float prevCurrentVy = currentVy;
    float prevCurrentOmega = currentOmega;
    float vx, vy, omega;
    motionQueue.getCurrentVelocity(vx, vy, omega);
    currentVx = vx;
    currentVy = vy;
    currentOmega = omega;
    const MotionSegment* seg = motionQueue.currentSegment();

    bool isMoving = (vx != 0 || vy != 0 || omega != 0);
    bool rotationOnly = (omega != 0 && vx == 0 && vy == 0);

    if (!isMoving && motorWasMoving && lastMotionWasRotationOnly &&
        !suppressRotationBrake && rotationBrakeFrames == 0 &&
        fabsf(lastRotationOmegaDegS) >= ROTATION_BRAKE_MIN_OMEGA_DEG_S) {
        rotationBrakeFrames = ROTATION_BRAKE_FRAMES;
        rotationBrakeDirection = (lastRotationOmegaDegS > 0) ? 1 : -1;
    }

    if (!isMoving && rotationBrakeFrames > 0 && rotationBrakeDirection != 0) {
        currentVx = 0;
        currentVy = 0;
        currentOmega = 0; // Decouple from dead-reckoning during the brake kick
#ifndef TEST_MODE
        Motor1.Run(-rotationBrakeDirection * ROTATION_BRAKE_DUTY);
        Motor2.Run(-rotationBrakeDirection * ROTATION_BRAKE_DUTY);
        Motor3.Run(rotationBrakeDirection * ROTATION_BRAKE_DUTY);
        Motor4.Run(rotationBrakeDirection * ROTATION_BRAKE_DUTY);
#endif
        rotationBrakeFrames--;
        if (rotationBrakeFrames == 0) {
            resetMotionControlState(false);
        }
        return;
    }

    // --- Active Translation Brake ---
    // When the queue commands zero velocity but the robot is still physically
    // moving (coasting), apply reverse motor thrust to kill all momentum.
    // Uses Kalman-estimated velocity to determine brake direction.
    if (!isMoving && motorWasMoving && !lastMotionWasRotationOnly &&
        translationBrakeFrames == 0 && !suppressRotationBrake) {
        float estVx, estVy;
        motionQueue.getEstimatedVelocity(estVx, estVy);
        float obsSpeed = sqrtf(estVx * estVx + estVy * estVy);
        if (obsSpeed >= TRANSLATION_BRAKE_MIN_SPEED_MM_S) {
            translationBrakeFrames = TRANSLATION_BRAKE_FRAMES;
            translationBrakeDirX = estVx / obsSpeed;  // unit vector of coasting direction
            translationBrakeDirY = estVy / obsSpeed;
        }
    }

    if (!isMoving && translationBrakeFrames > 0) {
        currentVx = 0;
        currentVy = 0;
        currentOmega = 0; // Decouple from dead-reckoning during brake
        // Compute reverse motor command from brake direction
        float cx, cy, c_angle;
        deadReckoning.getCurrentPosition(cx, cy, c_angle);
        double headingRad = c_angle * (PI / 180.0f);
        double cosA = cos(headingRad);
        double sinA = sin(headingRad);
        // Reverse of observed velocity direction → brake thrust
        double bV = -(translationBrakeDirX * cosA + translationBrakeDirY * sinA);
        double bH = -(translationBrakeDirX * sinA - translationBrakeDirY * cosA);
        // Scale to brake duty
        double scale = (double)TRANSLATION_BRAKE_DUTY / (double)DRIVE_CLAMP_HIGH;
        bV *= scale;
        bH *= scale;
        // Apply mecanum axis correction (same as normal path)
        bH /= sqrt(2.0);
        bV *= sqrt(2.0);
        MecanumSpeeds bs = computeMecanumSpeeds(bV, bH, 0, false);
#ifndef TEST_MODE
        Motor1.Run(bs.m1);
        Motor2.Run(bs.m2);
        Motor3.Run(bs.m3);
        Motor4.Run(bs.m4);
#endif
        translationBrakeFrames--;
        if (translationBrakeFrames == 0) {
            resetMotionControlState(false);
        }
        return;
    }

    if (!isMoving) {
        if (motionIdleStartTime == 0) motionIdleStartTime = millis();
        // Gap-hold: keep previous velocity briefly to bridge timing gaps between segments.
        // Skip this in HOLDING state — we've arrived and want instant stop, not coasting.
        const MotionSegment* holdSeg = motionQueue.currentSegment();
        bool isHolding = holdSeg && holdSeg->state == SegmentState::HOLDING;
        if (!isHolding && motorWasMoving && (millis() - motionIdleStartTime) <= motionCommandGapHoldMs) {
            currentVx = prevCurrentVx;
            currentVy = prevCurrentVy;
            currentOmega = prevCurrentOmega;
            return;
        }

        #ifndef TEST_MODE
        Motor1.Stop();
        Motor2.Stop();
        Motor3.Stop();
        Motor4.Stop();
        #endif

        if (millis() - motionIdleStartTime > 50) {
            resetMotionControlState(false);
        }
        return;
    }
    motionIdleStartTime = 0;

    suppressRotationBrake = false;
    // Auto-suppress brake for very short micro-refinement bursts
    if (seg && seg->duration_ms > 0 && seg->duration_ms < 150) {
        suppressRotationBrake = true;
    }
    rotationBrakeFrames = 0;
    rotationBrakeDirection = 0;
    translationBrakeFrames = 0;

    if (!motorWasMoving) {
        motorRampFactor = RAMP_START_FRACTION;
        motorWasMoving = true;
        rearmKickstart();
    } else if (motorRampFactor < 1.0f) {
        motorRampFactor += rampIncrement;
        if (motorRampFactor > 1.0f) motorRampFactor = 1.0f;
    }

    float cx, cy, c_angle;
    deadReckoning.getCurrentPosition(cx, cy, c_angle);

    double V = 0, H = 0;
    if (SPEED_FAST_MM_S > 0.1f) {
        double headingRad = c_angle * (PI / 180.0f);
        double cosA = cos(headingRad);
        double sinA = sin(headingRad);
        V = (vx * cosA + vy * sinA) / SPEED_FAST_MM_S;
        H = (vx * sinA - vy * cosA) / SPEED_FAST_MM_S;
    }

    V *= motorRampFactor;
    H *= motorRampFactor;

    // Mecanum wheels: H=forward(vertical) is √2× faster than V=strafe(horizontal).
    // Divide H by √2 to slow vertical, multiply V by √2 to boost strafe.
    // Both adjustments equalize physical speed between axes.
    H /= sqrt(2.0);
    V *= sqrt(2.0);

    // Drift trim: pre-rotate (V, H) CCW by a speed-dependent angle to counteract
    // the measured systematic leftward bias in translation.
    // With corrected mecanum formula, positive H = rightward physical strafe.
    // To compensate leftward drift, we need positive H → CW rotation of velocity.
    {
        float trim_deg = 0;
        if (seg) {
            switch (seg->speed) {
                case SpeedLevel::SLOW:   trim_deg = DRIFT_TRIM_SLOW_DEG;   break;
                case SpeedLevel::NORMAL: trim_deg = DRIFT_TRIM_NORMAL_DEG; break;
                case SpeedLevel::FAST:   trim_deg = DRIFT_TRIM_FAST_DEG;   break;
                default: break;
            }
        }
        if (trim_deg != 0) {
            float trim_rad = trim_deg * (PI / 180.0f);
            float cosT = cosf(trim_rad);
            float sinT = sinf(trim_rad);
            // CCW rotation in (H, V) plane:
            //   H' = H*cos - V*sin   (adds negative H for forward V → rightward physical)
            //   V' = H*sin + V*cos
            double H2 = H * cosT - V * sinT;
            double V2 = H * sinT + V * cosT;
            V = V2;
            H = H2;
        }
    }

    float target_angle = seg ? seg->target_angle : c_angle;

    float angle_err = target_angle - c_angle;
    while (angle_err > 180.0f) angle_err -= 360.0f;
    while (angle_err < -180.0f) angle_err += 360.0f;

    double A = 0;
    if (SPEED_FAST_DEG_S > 0.1f) {
        A = -omega / SPEED_FAST_DEG_S;  // invert: math CCW+ → mecanum CW+
        A *= rotationOnly ? 1.0 : motorRampFactor;
    }

    // --- Feedforward compensation from S-curve profiler ---
    // Add acceleration-proportional terms to overcome inertia/friction
    float ffVx, ffVy, ffOmega;
    motionQueue.getFeedforward(ffVx, ffVy, ffOmega);
    if (SPEED_FAST_MM_S > 0.1f) {
        double headingRad = c_angle * (PI / 180.0f);
        double cosA = cos(headingRad);
        double sinA = sin(headingRad);
        V += (ffVx * cosA + ffVy * sinA) / SPEED_FAST_MM_S;
        H += (ffVx * sinA - ffVy * cosA) / SPEED_FAST_MM_S;
    }
    if (SPEED_FAST_DEG_S > 0.1f) {
        A += -ffOmega / SPEED_FAST_DEG_S;
    }

    // Detect settling band: commanded speed is very low → use low-speed motor mode
    // to prevent kickstart and DRIVE_CLAMP_LOW from re-inflating tiny velocities
    float cmdMag = sqrtf(vx * vx + vy * vy);
    bool lowSpeedMode = (cmdMag > 0.001f && cmdMag < 10.0f);  // below ~10 mm/s
    MecanumSpeeds s = computeMecanumSpeeds(V, H, A, lowSpeedMode);

#if ENABLE_DEBUG_LOGGING
    static unsigned long lastDebugTime = 0;
    if (millis() - lastDebugTime >= 100) {
        Serial.printf("[Motor Debug] M1:%d M2:%d M3:%d M4:%d | Target: V:%.2f H:%.2f A:%.2f\n",
            s.m1, s.m2, s.m3, s.m4, V, H, A);
        lastDebugTime = millis();
    }
#endif

#ifdef TEST_MODE
    static unsigned long lastPrintTime = 0;
    if (millis() - lastPrintTime >= 500) {
        Serial.printf("[TEST MODE] Virtual Motors: M1=%d, M2=%d, M3=%d, M4=%d  V=%.2f H=%.2f A=%.2f\n",
            s.m1, s.m2, s.m3, s.m4, V, H, A);
        lastPrintTime = millis();
    }
#else
    Motor1.Run(s.m1);
    Motor2.Run(s.m2);
    Motor3.Run(s.m3);
    Motor4.Run(s.m4);
#endif

    lastMotionWasRotationOnly = rotationOnly;
    lastRotationOmegaDegS = omega;
}

// ============================================================================
// Process a pre-parsed UDP message (called from Core 0 under mutex)
// ============================================================================
void handleParsedMessage(const UdpMessage &msg) {
    switch (msg.type) {
        case MsgType::MOVE: {
            // Get current estimated position for segment target computation
            float cx, cy, c_angle;
            deadReckoning.getCurrentPosition(cx, cy, c_angle);

            bool ok = motionQueue.enqueue(
                msg.direction, msg.distance_mm,
                msg.speed, msg.correctionPolicy, msg.servoAction,
                cx, cy, c_angle
            );
            if (ok) {
                Serial.printf("[CMD] MOVE queued: dir=%d dist=%d spd=%d policy=%d (queue=%d)\n",
                    (int)msg.direction, msg.distance_mm, (int)msg.speed,
                    (int)msg.correctionPolicy, motionQueue.remaining());
            } else {
                Serial.println("[CMD] MOVE rejected — queue full!");
            }
            break;
        }

        case MsgType::MOVE_DURATION: {
            float cx, cy, c_angle;
            deadReckoning.getCurrentPosition(cx, cy, c_angle);

            bool ok = motionQueue.enqueueDuration(
                msg.direction, msg.duration_ms,
                msg.speed, msg.correctionPolicy,
                cx, cy, c_angle
            );
            if (ok) {
                Serial.printf("[CMD] MOVE_DUR queued: dir=%d dur=%lums spd=%d policy=%d (queue=%d)\n",
                    (int)msg.direction, (unsigned long)msg.duration_ms, (int)msg.speed,
                    (int)msg.correctionPolicy, motionQueue.remaining());
            } else {
                Serial.println("[CMD] MOVE_DUR rejected — queue full!");
            }
            break;
        }

        case MsgType::ROTATE_DURATION: {
            float cx, cy, c_angle;
            deadReckoning.getCurrentPosition(cx, cy, c_angle);

            bool ok = motionQueue.enqueueRotateDuration(
                msg.rotationDirection, msg.duration_ms,
                msg.speed, msg.correctionPolicy,
                cx, cy, c_angle
            );
            if (ok) {
                Serial.printf("[CMD] ROT_DUR queued: dir=%d dur=%lums spd=%d policy=%d (queue=%d)\n",
                    (int)msg.rotationDirection, (unsigned long)msg.duration_ms, (int)msg.speed,
                    (int)msg.correctionPolicy, motionQueue.remaining());
            } else {
                Serial.println("[CMD] ROT_DUR rejected — queue full!");
            }
            break;
        }

        case MsgType::WAYPOINT: {
            float cx, cy, c_angle;
            deadReckoning.getCurrentPosition(cx, cy, c_angle);

            // Same-target optimization: if already heading to this destination,
            // skip abort()+enqueue so the deceleration ramp runs uninterrupted.
            // This eliminates the kickstart surge caused by re-launching the segment
            // every time the phone resends the same W command at 500ms intervals.
            const MotionSegment* cur = motionQueue.currentSegment();
            bool sameTarget = (cur != nullptr &&
                               !cur->isDurationBased &&
                               (cur->state == SegmentState::ACTIVE ||
                                cur->state == SegmentState::HOLDING) &&
                               fabsf(cur->target_x - msg.target_x) < 15.0f &&
                               fabsf(cur->target_y - msg.target_y) < 15.0f);

            if (!sameTarget) {
                motionQueue.abort();
                bool ok = motionQueue.enqueueWaypoint(
                    msg.target_x, msg.target_y, msg.target_angle,
                    msg.speed, msg.correctionPolicy, msg.servoAction,
                    cx, cy, c_angle
                );
                if (ok) {
                    Serial.printf("[CMD] WAYPOINT: (%.1f, %.1f) @ %.1f°\n",
                        msg.target_x, msg.target_y, msg.target_angle);
                } else {
                    Serial.println("[CMD] WAYPOINT rejected — enqueue failed!");
                }
            } else {
                Serial.printf("[CMD] WAYPOINT same target — continuing decel\n");
            }
            break;
        }

        case MsgType::ROTATE: {
            float cx, cy, c_angle;
            deadReckoning.getCurrentPosition(cx, cy, c_angle);

            // Clear queue for immediate rotation override
            motionQueue.abort();

            bool ok = motionQueue.enqueueRotate(
                msg.target_angle, msg.speed, msg.correctionPolicy,
                cx, cy, c_angle
            );
            if (ok) {
                Serial.printf("[CMD] ROTATE queued: target=%.1f spd=%d policy=%d (queue=%d)\n",
                    msg.target_angle, (int)msg.speed,
                    (int)msg.correctionPolicy, motionQueue.remaining());
            } else {
                Serial.println("[CMD] ROTATE rejected — queue full!");
            }
            break;
        }

        case MsgType::CAM: {
            // We are using absolute clock synchronization. The phone sends the 
            // exact phone-clock timestamp of the frame capture. 
            // the LatencyCompensator translates this to robot-time via the offset.
            latencyComp.onCameraUpdate(
                msg.cam_timestamp, msg.cam_x, msg.cam_y, msg.cam_angle,
                CORRECTION_BLEND_MS
            );
            break;
        }

        case MsgType::PING: {
            // Respond immediately with PONG. 
            // Note: The phone will include its own timestamp in its PONG reply to us.
            char pongBuf[32];
            int pongLen = buildPongMessage(pongBuf, sizeof(pongBuf), msg.ping_timestamp);
            udp.beginPacket(udp.remoteIP(), udpPort);
            udp.write((uint8_t*)pongBuf, pongLen);
            udp.endPacket();
            break;
        }

        case MsgType::PONG: {
            // Include the remote (phone) timestamp to update our clock offset
            latencyComp.onPong(msg.ping_timestamp, msg.remote_timestamp);
            Serial.printf("[RTT] %lu ms\n", (unsigned long)latencyComp.getRttMs());
            break;
        }

        case MsgType::VELOCITY: {
            float cx, cy, c_angle;
            deadReckoning.getCurrentPosition(cx, cy, c_angle);

            // Abort existing queue (smooth transition — no abortFlag)
            motionQueue.abort();

            bool ok = motionQueue.enqueueVelocity(
                msg.vel_vx, msg.vel_vy, msg.vel_omega,
                msg.duration_ms,
                cx, cy, c_angle
            );
            if (ok) {
                Serial.printf("[CMD] VELOCITY: vx=%.1f vy=%.1f ω=%.1f timeout=%lums\n",
                    msg.vel_vx, msg.vel_vy, msg.vel_omega,
                    (unsigned long)msg.duration_ms);
            } else {
                Serial.println("[CMD] VELOCITY rejected — enqueue failed!");
            }
            break;
        }

        case MsgType::ABORT: {
            Serial.println("[CMD] ABORT — emergency stop!");
            motionQueue.abort();
            currentVx = currentVy = currentOmega = 0;
            abortFlag = true;   // Core 1 will stop motors immediately
            break;
        }

        default:
            Serial.printf("[UDP] Unhandled message type: %d\n", (int)msg.type);
            break;
    }
} // Added closing bracket here

// ============================================================================
// Send telemetry status to phone (every STATUS_INTERVAL_MS)
// ============================================================================
void sendStatus() {
    float x, y, angle;
    deadReckoning.getCurrentPosition(x, y, angle);

    // Get estimated velocity from MotionQueue for visualization
    float vx, vy;
    motionQueue.getEstimatedVelocity(vx, vy);

    char buf[80];
    int len = buildStatusMessage(buf, sizeof(buf),
        x, y, angle,
        motionQueue.remaining(),
        latencyComp.getLastDriftMagnitude(),
        vx, vy
    );

    udp.beginPacket(phoneIP, udpPort);
    udp.write((uint8_t*)buf, len);
    udp.endPacket();
}

// ============================================================================
// Send HELLO beacon for auto-discovery (every HELLO_INTERVAL_MS)
// ============================================================================
void sendHello() {
    char buf[64];
    int len = snprintf(buf, sizeof(buf), "H:%s:%s", BOT_ID, WiFi.localIP().toString().c_str());
    udp.beginPacket(phoneIP, udpPort);
    udp.write((uint8_t*)buf, len);
    udp.endPacket();
}

// ============================================================================
// Send PING for RTT measurement (every PING_INTERVAL_MS)
// ============================================================================
void sendPing() {
    char buf[32];
    int len = snprintf(buf, sizeof(buf), "P:%lu", (unsigned long)millis());
    udp.beginPacket(phoneIP, udpPort);
    udp.write((uint8_t*)buf, len);
    udp.endPacket();
}

// ============================================================================
// SETUP
// ============================================================================
void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println("\n========================================");
    Serial.printf("  Dead-Reckoning Robot [%s]\n", BOT_ID);
    Serial.println("========================================\n");

    // --- WiFi ---
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    Serial.print("Connecting to hotspot");
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nConnected!");
    Serial.printf("  Robot IP:  %s\n", WiFi.localIP().toString().c_str());
    phoneIP = WiFi.gatewayIP();
    Serial.printf("  Phone IP:  %s\n", phoneIP.toString().c_str());

    udp.begin(udpPort);

    // --- Initialize subsystems ---
    deadReckoning.reset(0, 0, 0);
    deadReckoning.setDistanceFactors(DISTANCE_FACTOR_H, DISTANCE_FACTOR_V);

    initServoControl();

    Motor1.Reverse();
    Motor2.Reverse();

    motionQueue.setSpeedCalibration(SPEED_SLOW_MM_S, SPEED_NORMAL_MM_S, SPEED_FAST_MM_S);
    motionQueue.setRotationCalibration(SPEED_SLOW_DEG_S, SPEED_NORMAL_DEG_S, SPEED_FAST_DEG_S);
    motionQueue.setDistanceFactors(DISTANCE_FACTOR_H, DISTANCE_FACTOR_V);
    motionQueue.setPrecisionParameters(DECCEL_DISTANCE_MM, ROT_DECCEL_DEG, 
                                       MIN_SPEED_LIMIT_MM_S, MIN_ROT_LIMIT_DEG_S,
                                       PRECISION_MIN_SPEED_LIMIT_MM_S, PRECISION_MIN_ROT_LIMIT_DEG_S,
                                       CLOSE_APPROACH_DISTANCE_MM, CLOSE_ROT_APPROACH_DEG,
                                       WAYPOINT_TOLERANCE_MM, ROTATION_TOLERANCE_DEG,
                                       STABILIZATION_GAIN, MAX_STABILIZATION_OMEGA);
    motionQueue.setPredictiveParameters(PREDICTIVE_LOOKAHEAD_S);
    motionQueue.setSCurveParameters(SCURVE_MAX_ACCEL_MM_S2, SCURVE_MAX_JERK_MM_S3,
                                     SCURVE_MAX_ROT_ACCEL_DEG_S2, SCURVE_MAX_ROT_JERK_DEG_S3);
    motionQueue.setFeedforwardGains(FEEDFORWARD_KV, FEEDFORWARD_KA,
                                     FEEDFORWARD_KV_ROT, FEEDFORWARD_KA_ROT);
    motionQueue.setAdaptiveLookahead(ADAPTIVE_LOOKAHEAD_BASE_S, ADAPTIVE_LOOKAHEAD_GAIN,
                                      ADAPTIVE_LOOKAHEAD_MIN_S, ADAPTIVE_LOOKAHEAD_MAX_S);
    motionQueue.setKalmanParameters(KALMAN_PROCESS_NOISE_POS, KALMAN_PROCESS_NOISE_VEL,
                                     KALMAN_MEASUREMENT_NOISE);
    motionQueue.setSlipDetection(SLIP_CMD_SPEED_THRESH_MM_S, SLIP_OBS_SPEED_THRESH_MM_S,
                                  SLIP_DETECT_TICKS, SLIP_BOOST_FACTOR, SLIP_BOOST_MAX_TICKS);
    motionQueue.setPredictiveBraking(PREDICTIVE_BRAKE_DECEL_MM_S2, PREDICTIVE_BRAKE_SAFETY);

    latencyComp.init(&deadReckoning, &motionQueue);
    latencyComp.setThresholds(DRIFT_THRESHOLD_MM, EMERGENCY_THRESHOLD_MM);
    latencyComp.setCameraLatency(CAMERA_LATENCY_MS);

    // --- Send registration to phone ---
    char regBuf[80];
    int regLen = snprintf(regBuf, sizeof(regBuf), "R:%s:mecanum4wd:%s", BOT_ID, WiFi.localIP().toString().c_str());
    udp.beginPacket(phoneIP, udpPort);
    udp.write((uint8_t*)regBuf, regLen);
    udp.endPacket();

#ifdef TEST_MODE
    Serial.println("\n*** TEST MODE ENABLED ***");
    Serial.println("Motors are disabled. Commands are printed to Serial.");
    ledcSetup(7, 10, 12); // Channel 7, 10 kHz, 12-bit resolution
    ledcAttachPin(LED, 7);
#endif

    // --- Dual-core: create mutex and start UDP task on Core 0 ---
    stateMutex = xSemaphoreCreateMutex();
    xTaskCreatePinnedToCore(
        udpTask,      // function
        "UDPTask",    // name
        8192,         // stack size (bytes)
        NULL,         // parameter
        2,            // priority (above idle, below WiFi events)
        NULL,         // task handle
        0             // Core 0
    );

    Serial.println("\nReady. Waiting for commands...\n");
    Serial.println("  Core 0: UDP processing");
    Serial.println("  Core 1: Motor control + dead reckoning");
    lastTick = millis();
}

// ============================================================================
// CORE 0 TASK — UDP processing (commands, camera, broadcasts)
// ============================================================================
void udpTask(void* parameter) {
    (void)parameter;
    Serial.println("[Core 0] UDP task started");

    while (true) {
        // --- Process all waiting UDP packets ---
        for (int i = 0; i < 10; i++) {
            int packetSize = udp.parsePacket();
            if (!packetSize) break;

            char buffer[256];
            int len = udp.read(buffer, sizeof(buffer) - 1);
            if (len <= 0) continue;
            buffer[len] = '\0';

#ifdef TEST_MODE
            Serial.printf("[TEST MODE] Received UDP Packet: %s\n", buffer);
#endif

            UdpMessage msg;
            if (!parseUdpMessage(buffer, len, msg)) {
                Serial.printf("[UDP] Failed to parse: %s\n", buffer);
                continue;
            }

            // ABORT is processed without waiting for mutex — instant response
            if (msg.type == MsgType::ABORT) {
                Serial.println("[CMD] ABORT — emergency stop!");
                xSemaphoreTake(stateMutex, portMAX_DELAY);
                motionQueue.abort();
                currentVx = currentVy = currentOmega = 0;
                xSemaphoreGive(stateMutex);
                abortFlag = true;  // Core 1 will stop motors immediately
                continue;
            }

            // All other messages need mutex for shared state
            xSemaphoreTake(stateMutex, portMAX_DELAY);
            handleParsedMessage(msg);
            xSemaphoreGive(stateMutex);
        }

        // --- Periodic broadcasts ---
        uint32_t now = millis();
        if (now - lastStatusTime >= STATUS_INTERVAL_MS) {
            xSemaphoreTake(stateMutex, portMAX_DELAY);
            sendStatus();
            xSemaphoreGive(stateMutex);
            lastStatusTime = now;
        }

        if (now - lastHelloTime >= HELLO_INTERVAL_MS) {
            sendHello();
            lastHelloTime = now;
        }

        if (now - lastPingTime >= PING_INTERVAL_MS) {
            sendPing();
            lastPingTime = now;
        }

        vTaskDelay(1);  // yield to WiFi stack (~1ms tick)
    }
}

// ============================================================================
// MAIN LOOP — Core 1: motor control + dead reckoning
// ============================================================================
void loop() {
    uint32_t now = millis();
    uint32_t dt  = now - lastTick;

    // Target ~10ms control loop period
    if (dt < CONTROL_LOOP_INTERVAL_MS) return;
    lastTick = now;

    // ---- Check abort flag (set by Core 0 for instant stop) ----
    if (abortFlag) {
        abortFlag = false;
#ifndef TEST_MODE
        Motor1.Stop();
        Motor2.Stop();
        Motor3.Stop();
        Motor4.Stop();
#else
        Serial.println("[TEST MODE] Motors stopped (ABORT)");
#endif
        resetMotionControlState(true);
        return;
    }

    xSemaphoreTake(stateMutex, portMAX_DELAY);

    // ---- Tick the motion queue ----
    float current_x, current_y, current_angle;
    deadReckoning.getCurrentPosition(current_x, current_y, current_angle);
    bool moving = motionQueue.tick(dt, current_x, current_y, current_angle);

    ServoAction actionToExecute = ServoAction::NONE;
    if (motionQueue.segmentJustCompleted) {
        actionToExecute = motionQueue.getLastCompletedServoAction();
        // Clear flag now that we've captured the action
        motionQueue.segmentJustCompleted = false;

        if (motionQueue.getActivePolicy() == CorrectionPolicy::DEFERRED) {
            float dcX, dcY, dcA;
            motionQueue.getDeferredCorrection(dcX, dcY, dcA);
            latencyComp.applyCorrection(dcX, dcY, dcA, CORRECTION_BLEND_MS);
        }
    }


    // ---- Handle emergency deceleration ----
    if (latencyComp.wasEmergencyTriggered() && moving) {
        Serial.println("[LC] Emergency correction triggered — continuing with corrected state");
    }

    // ---- Apply motor speeds ----
    applyMotors();

    // ---- Update dead-reckoning ----
    // Use the same dt calculated at the top of the loop for consistency
    deadReckoning.update(currentVx, currentVy, currentOmega, dt);

    xSemaphoreGive(stateMutex);

    // ---- Execution (OUTSIDE mutex) ----
    // This allows Core 0 to handle heartbeats and new waypoint commands 
    // while the robot is busy with its servo sequence.
    if (actionToExecute != ServoAction::NONE) {
        Serial.printf("[SERVO] Executing Action: %d\n", (int)actionToExecute);
        executeServoAction(actionToExecute);
        
        // Rearm kickstart because the motors have been stopped for a while
        rearmKickstart();
    }

#ifdef TEST_MODE
    static uint32_t lastLedBlinkTime = 0;
    static bool ledState = false;
    if (now - lastLedBlinkTime >= 500) {
        lastLedBlinkTime = now;
        ledState = !ledState;
        ledcWrite(7, ledState ? 2048 : 0);
    }
#endif
}