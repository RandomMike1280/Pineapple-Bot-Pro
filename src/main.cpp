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
// The same firmware runs on both robots — only the config file selected
// in main.h differs.  See main.h and the config_*.h files for details.
// ============================================================================

#include <WiFi.h>
#include <WiFiUdp.h>
#include <functional>
#include "main.h"
#include <dead_reckoning.hpp>
#include <motion_queue.hpp>
#include <latency_compensator.hpp>
#include <udp_protocol.hpp>
#include <servo_control.hpp>
#include <udp_logger.hpp>

// ============================================================================
// WiFi Configuration — pulled from the selected robot config (config_a.h / config_b.h)
// ============================================================================
#ifndef ROBOT_WIFI_SSID
#define ROBOT_WIFI_SSID     "UNKNOWN_SSID"
#endif
#ifndef ROBOT_WIFI_PASS
#define ROBOT_WIFI_PASS     "UNKNOWN_PASS"
#endif
const char* ssid     = ROBOT_WIFI_SSID;
const char* password = ROBOT_WIFI_PASS;

// ============================================================================
// Subsystem instances
// ============================================================================
WiFiUDP          udp;
const int        udpPort = 4210;
IPAddress        phoneIP;

DeadReckoning    deadReckoning;
MotionQueue      motionQueue;
LatencyCompensator latencyComp;
UdpLogger        logger(udp, phoneIP, udpPort);

// ============================================================================
// Timing state
// ============================================================================
uint32_t lastTick       = 0;
uint32_t lastHelloTime  = 0;
uint32_t lastStatusTime = 0;
uint32_t lastPingTime   = 0;
static int doneFeedbackBlinks = 0;
static uint32_t lastFeedbackToggleTime = 0;

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
// Shared helper — fetch current pose in one call
// ============================================================================
static inline void getCurrentPose(float& x, float& y, float& angle) {
    deadReckoning.getCurrentPosition(x, y, angle);
}

// ============================================================================
// PS2 physics-based motor control
// ============================================================================

// Kickstart state per motor
static int mkickstart[4] = {KICKSTART_FRAMES, KICKSTART_FRAMES, KICKSTART_FRAMES, KICKSTART_FRAMES};
static bool isMoving = false;

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

// ============================================================================
// Duty normalization — scale all motors proportionally to fit within DRIVE_CLAMP_HIGH
// ============================================================================
static void applyDutyNormalization(double mspeedf[4]) {
    double maxDuty = 0;
    for (int i = 0; i < 4; i++) {
        maxDuty = max(maxDuty, abs(mspeedf[i]));
    }
    if (maxDuty > DRIVE_CLAMP_HIGH) {
        for (int i = 0; i < 4; i++) {
            mspeedf[i] = mspeedf[i] / maxDuty * DRIVE_CLAMP_HIGH;
        }
    }
}

MecanumSpeeds computeMecanumSpeeds(double V, double H, double A, bool lowSpeedMode, bool precisionMode) {
    MecanumSpeeds s;
    double mspeedf[4];

    // Mecanum formula with mirrored front/back motors
    // Mirrored layout swaps V↔H: H=forward/back, V=strafe left/right
    // Diagonal pairing: FL+BR share +V, FR+BL share -V
    mspeedf[0] = -H - V - A;  // Front Right
    mspeedf[1] = -H + V - A;  // Back Right
    mspeedf[2] = -H + V + A;  // Front Left
    mspeedf[3] = -H - V + A;  // Back Left

    // Convert to motor duty and compensate for dead zone
    for (int i = 0; i < 4; i++) {
        if (abs(mspeedf[i]) > 0) {
            mspeedf[i] = speed_to_motor_duty(mspeedf[i]);
        }
    }

    // Normalize to keep within DRIVE_CLAMP_HIGH
    applyDutyNormalization(mspeedf);

    if (precisionMode) {
        // Precision mode: use lower duty floor for very fine movements.
        // Use induction kickstart: if commanded speed > 0 but observed speed ≈ 0,
        // the motors are stalled — give a brief boost to break static friction.
        // If observed speed > 0, motors are already moving and don't need a boost.
        float estVx, estVy;
        motionQueue.getEstimatedVelocity(estVx, estVy);
        float estSpeed = sqrtf(estVx * estVx + estVy * estVy);
        float cmdSpeed = sqrtf(V * V + H * H);
        bool motorsStalled = (cmdSpeed > 0.05f && estSpeed < 1.5f);

        for (int i = 0; i < 4; i++) {
            double absSpeed = abs(mspeedf[i]);
            if (absSpeed > 0 && absSpeed < DRIVE_CLAMP_FINE) {
                mspeedf[i] = sign(mspeedf[i]) * DRIVE_CLAMP_FINE;
            }
        }

        // Induction kickstart: boost only when motors are actually stalled
        if (motorsStalled) {
            for (int i = 0; i < 4; i++) {
                if (abs(mspeedf[i]) > 0) {
                    double absSpeed = abs(mspeedf[i]);
                    double kickPower = DRIVE_CLAMP_FINE + 15;
                    mspeedf[i] = sign(mspeedf[i]) * max(kickPower, absSpeed);
                }
            }
        }
    } else if (lowSpeedMode) {
        // Near the target: skip kickstart and use a much lower duty floor.
        // This allows the robot to actually crawl at tiny velocities without
        // the kickstart/clamp re-inflating them to full duty (which causes wiggle).
        for (int i = 0; i < 4; i++) {
            double absSpeed = abs(mspeedf[i]);
            if (absSpeed > 0 && absSpeed < DRIVE_CLAMP_LOW) {
                mspeedf[i] = sign(mspeedf[i]) * DRIVE_CLAMP_LOW;
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

    // Motor balance: right-side motors (M1=FR, M4=BR) produce more torque than
    // left-side motors (M2=FL, M3=BL). Equalize by reducing right-side PWM magnitude.
    //
    // MecanumSpeeds layout: M1=FR, M2=FL, M3=BL, M4=BR
    // Right side = M1, M4 (indices 0, 3)
    // Left side  = M2, M3 (indices 1, 2)
    //
    // The mecanum formula H-V-A / H+V-A / H+V+A / H-V+A means right side gets
    // the "+A" terms added and left side gets "-A" terms subtracted, creating
    // a base imbalance of 2*A in addition to the inherent torque difference.
    const float BALANCE_RIGHT_REDUCE = 3.0f;
    for (int i = 0; i < 4; i++) {
        if (i == 0 || i == 3) {  // right-side motors: M1 (FR), M4 (BR)
            mspeedf[i] -= BALANCE_RIGHT_REDUCE;
            float clamp_limit = DRIVE_CLAMP_LOW * 0.5f;
            if (mspeedf[i] > 0 && mspeedf[i] < clamp_limit) mspeedf[i] = clamp_limit;
            if (mspeedf[i] < 0 && mspeedf[i] > -clamp_limit) mspeedf[i] = -clamp_limit;
        }
    }

    s.m1 = (int)mspeedf[0];  // Front Right (right side)
    s.m2 = (int)mspeedf[1];  // Front Left  (left side)
    s.m3 = (int)mspeedf[2];  // Back Left   (left side)
    s.m4 = (int)mspeedf[3];  // Back Right  (right side)

    // DEBUG: Motor balance diagnostics — log balance adjustment near target
    {
        static unsigned long lastBalLogMs = 0;
        if (millis() - lastBalLogMs >= 100) {
            float cmdSpd = sqrtf(V * V + H * H);
            // Only log when V or H is small (near target / precision mode)
            if (cmdSpd < 0.3f && cmdSpd > 0.001f) {
                // Compute raw speeds before balance
                double rawM[4] = {mspeedf[0], mspeedf[1], mspeedf[2], mspeedf[3]};
                Serial.printf("[BAL] cmdSpd=%.3f M1=%+4d M2=%+4d M3=%+4d M4=%+4d | avg=%.1f | Ravg=%.1f Lavg=%.1f\n",
                    cmdSpd, s.m1, s.m2, s.m3, s.m4,
                    (s.m1 + s.m2 + s.m3 + s.m4) / 4.0f,
                    (s.m1 + s.m4) / 2.0f,  // right: M1 (FR), M4 (BR)
                    (s.m2 + s.m3) / 2.0f); // left:  M2 (FL), M3 (BL)
            }
            lastBalLogMs = millis();
        }
    }

    return s;
}

// ============================================================================
// Single-Motor Precision Mode — ultra-fine micro-adjustments
// ============================================================================
// For distances < ~15mm, activate only ONE wheel at a time.
// This produces the smallest possible robot displacement (~1-2mm per step).
MecanumSpeeds computeSingleMotorSpeeds(double V, double H) {
    MecanumSpeeds s = {0, 0, 0, 0};
    
    // Select the dominant wheel based on desired velocity direction
    // Mecanum wheel vectors (body frame):
    //   M1(FR):  vx=-1, vy=-1   (forward-right quadrant)
    //   M2(BR):  vx=-1, vy=+1   (backward-right quadrant)
    //   M3(BL):  vx=+1, vy=+1   (backward-left quadrant)
    //   M4(FL):  vx=+1, vy=-1   (forward-left quadrant)
    
    float desiredSpeed = sqrtf(V * V + H * H);
    if (desiredSpeed < 0.001f) return s;  // no movement requested
    
    // Normalize direction vector
    float dirX = V / desiredSpeed;
    float dirY = H / desiredSpeed;
    
    // Dot product with each wheel's velocity vector to find dominant wheel
    // Wheel vectors (normalized):
    float w1 = -dirX - dirY;  // M1
    float w2 = -dirX + dirY;  // M2
    float w3 =  dirX + dirY;  // M3
    float w4 =  dirX - dirY;  // M4
    
    // Select wheel with largest positive dot (most aligned with desired direction)
    float maxDot = w1;
    int selectedWheel = 0;
    if (w2 > maxDot) { maxDot = w2; selectedWheel = 1; }
    if (w3 > maxDot) { maxDot = w3; selectedWheel = 2; }
    if (w4 > maxDot) { maxDot = w4; selectedWheel = 3; }
    
    // If no wheel has positive alignment (unusual), select strongest wheel anyway
    if (maxDot <= 0) {
        float absVals[4] = {fabsf(w1), fabsf(w2), fabsf(w3), fabsf(w4)};
        selectedWheel = 0;
        for (int i = 1; i < 4; i++) {
            if (absVals[i] > absVals[selectedWheel]) selectedWheel = i;
        }
    }
    
// Single-motor mode: bypass speed mapping, use fixed duty cycle.
// Since speed_to_motor_duty() has a dead zone (~65), use duty >= DRIVE_CLAMP_LOW
// to guarantee motor starts, while still keeping speed low.
int duty = DRIVE_CLAMP_LOW + 5;  // 70 — conservative start value (~5-8mm/s)

// For very short distances (<8mm), use the minimum duty that still starts
if (desiredSpeed < 8.0f) {
    duty = DRIVE_CLAMP_LOW;  // 65 — minimal reliable start
}
    
    // Assign to selected wheel only
    // Use H >= 0 (not V >= 0) to determine forward/back direction:
    // M1/M2 spin on the V axis (-vx-vy, -vx+vy), so they naturally flip
    // when V reverses. M3/M4 spin on the H axis (+vx+vy, +vx-vy), so they
    // flip when H reverses. The wheel selector uses H as the dominant axis
    // for M3/M4, so the polarity test must match.
    switch (selectedWheel) {
        case 0: s.m1 = (V >= 0) ? duty : -duty; break;  // FR — responds to V
        case 1: s.m2 = (V >= 0) ? duty : -duty; break;  // BR — responds to V
        case 2: s.m3 = (H >= 0) ? duty : -duty; break;  // BL — responds to H
        case 3: s.m4 = (H >= 0) ? duty : -duty; break;  // FL — responds to H
    }
    
    return s;
}

// ============================================================================
// Motor state helpers
// ============================================================================

static void rearmKickstart() {
    for (int i = 0; i < 4; i++) {
        mkickstart[i] = KICKSTART_FRAMES;
    }
}

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

// ============================================================================
// Brake duty computation — shared by translation and rotation brakes
// ============================================================================

static MecanumSpeeds computeBrakeDuty(float brakeDirX, float brakeDirY,
                                      float headingAngle, int brakeDuty) {
    double cosA = cos(headingAngle);
    double sinA = sin(headingAngle);
    double bV = -(brakeDirX * cosA + brakeDirY * sinA);
    double bH = -(brakeDirX * sinA - brakeDirY * cosA);
    double scale = (double)brakeDuty / (double)DRIVE_CLAMP_HIGH;
    return computeMecanumSpeeds(bV * scale, bH * scale, 0, false, false);
}

// ============================================================================
// Rotation brake — kick brief reverse thrust to kill angular momentum
// ============================================================================

static bool applyRotationBrake() {
    if (!isMoving && rotationBrakeFrames > 0 && rotationBrakeDirection != 0) {
        currentVx = 0;
        currentVy = 0;
        currentOmega = 0;
#ifndef TEST_MODE
        Motor1.Run(-rotationBrakeDirection * ROTATION_BRAKE_DUTY);
        Motor2.Run(-rotationBrakeDirection * ROTATION_BRAKE_DUTY);
        Motor3.Run( rotationBrakeDirection * ROTATION_BRAKE_DUTY);
        Motor4.Run( rotationBrakeDirection * ROTATION_BRAKE_DUTY);
#endif
        rotationBrakeFrames--;
        if (rotationBrakeFrames == 0) {
            resetMotionControlState(false);
        }
        return true;
    }
    return false;
}

// ============================================================================
// Translation brake — reverse thrust to kill physical coasting momentum
// ============================================================================

static bool detectAndStartTranslationBrake() {
    if (!isMoving && motorWasMoving && !lastMotionWasRotationOnly &&
        translationBrakeFrames == 0 && !suppressRotationBrake) {
        float estVx, estVy;
        motionQueue.getEstimatedVelocity(estVx, estVy);
        float obsSpeed = sqrtf(estVx * estVx + estVy * estVy);
        if (obsSpeed >= TRANSLATION_BRAKE_MIN_SPEED_MM_S) {
            translationBrakeFrames = TRANSLATION_BRAKE_FRAMES;
            translationBrakeDirX = estVx / obsSpeed;
            translationBrakeDirY = estVy / obsSpeed;
        }
    }
    if (!isMoving && translationBrakeFrames > 0) {
        currentVx = 0;
        currentVy = 0;
        currentOmega = 0;
        float cx, cy, c_angle;
        getCurrentPose(cx, cy, c_angle);
        MecanumSpeeds bs = computeBrakeDuty(
            translationBrakeDirX, translationBrakeDirY,
            c_angle * (PI / 180.0f), TRANSLATION_BRAKE_DUTY);
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
        return true;
    }
    return false;
}

// ============================================================================
// Detect precision / single-motor operating modes based on distance and speed
// ============================================================================

static void detectOperatingMode(const MotionSegment* seg, float cmdMag, float cx, float cy,
                                bool& precisionMode, bool& singleMotorMode) {
    precisionMode = false;
    singleMotorMode = false;
    if (seg && seg->state == SegmentState::ACTIVE) {
        float dx = seg->target_x - cx;
        float dy = seg->target_y - cy;
        float distRemaining = sqrtf(dx * dx + dy * dy);

        logger.update("DEBUG", "DIST_CHECK: dist=%.2f cmdMag=%.3f", distRemaining, cmdMag);

        if (distRemaining < 15.0f && cmdMag > 0.001f && cmdMag < 10.0f) {
            singleMotorMode = true;
        } else if (distRemaining < PRECISION_MODE_THRESH_MM &&
                   cmdMag > 0.001f && cmdMag < 15.0f) {
            precisionMode = true;
        }
    }
}

// ============================================================================
// Compute heading-frame (V, H) velocities from world-frame (vx, vy, omega)
// Includes: heading transform, drift trim, S-curve feedforward, omega
// ============================================================================

static void computeHeadingVelocities(float vx, float vy, float omega,
                                    const MotionSegment* seg,
                                    double& V, double& H, double& A) {
    // Use blended camera-DR angle for heading transform when available.
    // The blended angle converges to ground-truth and avoids the spurious strafe
    // that occurs when the transform uses a drifting DR angle.
    float c_angle;
    if (!motionQueue.getBlendedAngle(c_angle)) {
        float dummy_x, dummy_y;
        deadReckoning.getCurrentPosition(dummy_x, dummy_y, c_angle);
    }
    double V_out = 0, H_out = 0;

    if (SPEED_FAST_MM_S > 0.1f) {
        double headingRad = c_angle * (PI / 180.0f);
        double cosA = cos(headingRad);
        double sinA = sin(headingRad);
        V_out = (vx * cosA + vy * sinA) / SPEED_FAST_MM_S;
        H_out = (vx * sinA - vy * cosA) / SPEED_FAST_MM_S;
    }

    V_out *= motorRampFactor;
    H_out *= motorRampFactor;

    // Mecanum H axis is naturally faster by √2 — normalize to match V
    H_out /= sqrt(2.0);

    // Drift trim: pre-rotate (V, H) to counteract systematic lateral bias
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
            double H2 = H_out * cosT - V_out * sinT;
            double V2 = H_out * sinT + V_out * cosT;
            V_out = V2;
            H_out = H2;
        }
    }

    // S-curve feedforward: compensate inertia and friction
    // Feedforward is computed in world frame by the motion queue, but we need
    // to project it into the robot's heading frame (V, H).  The correct
    // projection uses the INVERSE rotation (transpose), not the forward matrix:
    //   V += ( ffVx * cosA - ffVy * sinA) / SPEED_FAST_MM_S
    //   H += ( ffVx * sinA + ffVy * cosA) / SPEED_FAST_MM_S
    float ffVx, ffVy, ffOmega;
    motionQueue.getFeedforward(ffVx, ffVy, ffOmega);
    if (SPEED_FAST_MM_S > 0.1f) {
        double headingRad = c_angle * (PI / 180.0f);
        double cosA = cos(headingRad);
        double sinA = sin(headingRad);
        V_out += (ffVx * cosA - ffVy * sinA) / SPEED_FAST_MM_S;
        H_out += (ffVx * sinA + ffVy * cosA) / SPEED_FAST_MM_S;
    }

    V = V_out;
    H = H_out;

    // Angular velocity
    bool rotationOnly = (omega != 0 && vx == 0 && vy == 0);
    A = 0;
    if (SPEED_FAST_DEG_S > 0.1f) {
        A = -omega / SPEED_FAST_DEG_S;
        A *= rotationOnly ? 1.0 : motorRampFactor;
    }
}

// ============================================================================
// Apply motor speeds from current motion queue state
// ============================================================================
void applyMotors() {
    // Capture previous velocity for gap-hold bridging
    float prevCurrentVx = currentVx;
    float prevCurrentVy = currentVy;
    float prevCurrentOmega = currentOmega;

    float vx, vy, omega;
    motionQueue.getCurrentVelocity(vx, vy, omega);
    currentVx = vx;
    currentVy = vy;
    currentOmega = omega;
    const MotionSegment* seg = motionQueue.currentSegment();

    isMoving = (vx != 0 || vy != 0 || omega != 0);
    bool rotationOnly = (omega != 0 && vx == 0 && vy == 0);

    // --- Trigger rotation brake when transitioning from rotation to stop ---
    if (!isMoving && motorWasMoving && lastMotionWasRotationOnly &&
        !suppressRotationBrake && rotationBrakeFrames == 0 &&
        fabsf(lastRotationOmegaDegS) >= ROTATION_BRAKE_MIN_OMEGA_DEG_S) {
        rotationBrakeFrames = ROTATION_BRAKE_FRAMES;
        rotationBrakeDirection = (lastRotationOmegaDegS > 0) ? 1 : -1;
    }

    // --- Rotation brake execution ---
    if (applyRotationBrake()) return;

    // --- Translation brake detection and execution ---
    if (detectAndStartTranslationBrake()) return;

    // --- Idle: coasting bridge, then stop ---
    if (!isMoving) {
        if (motorWasMoving) {
            logger.update("DEBUG", "!isMoving: motorWasMoving=%d rotOnly=%d rotBrake=%d transBrake=%d",
                motorWasMoving, lastMotionWasRotationOnly,
                rotationBrakeFrames > 0, translationBrakeFrames > 0);
        }

        if (motionIdleStartTime == 0) motionIdleStartTime = millis();
        const MotionSegment* holdSeg = motionQueue.currentSegment();
        bool isHolding = holdSeg && holdSeg->state == SegmentState::HOLDING;
        if (!isHolding && motorWasMoving && (millis() - motionIdleStartTime) <= motionCommandGapHoldMs) {
            currentVx = prevCurrentVx;
            currentVy = prevCurrentVy;
            currentOmega = prevCurrentOmega;
            return;
        }

        // If we have a segment that's still active and ramp isn't finished, keep ramping.
        // This handles the case where _currentVx == 0 due to Kalman deadzone but the
        // motion queue still intends to move (e.g. short distance moves near target).
        if (seg && seg->state == SegmentState::ACTIVE && motorRampFactor < 1.0f) {
            if (!motorWasMoving) {
                motorRampFactor = RAMP_START_FRACTION;
                motorWasMoving = true;
                rearmKickstart();
            } else {
                motorRampFactor += rampIncrement;
                if (motorRampFactor > 1.0f) motorRampFactor = 1.0f;
            }
            // Suppress brakes during ongoing segment ramp-up
            rotationBrakeFrames = 0;
            rotationBrakeDirection = 0;
            translationBrakeFrames = 0;
            suppressRotationBrake = true;
            motionIdleStartTime = 0;
            return;
        }

#ifndef TEST_MODE
        // Only stop motors when ramp is fully complete (not during startup acceleration).
        // When ramping up, _currentVx may be 0 momentarily while the motion queue
        // is still commanding movement — stopping motors here would kill the startup.
        if (motorRampFactor >= 1.0f) {
            Motor1.Stop();
            Motor2.Stop();
            Motor3.Stop();
            Motor4.Stop();
        }
#endif

        if (millis() - motionIdleStartTime > 50) {
            resetMotionControlState(false);
        }
        return;
    }
    motionIdleStartTime = 0;

    // --- Startup: reset brake state, start acceleration ramp ---
    suppressRotationBrake = false;
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

    // --- Compute heading-frame velocities ---
    float cx, cy, c_angle;
    getCurrentPose(cx, cy, c_angle);
    double V, H, A;
    computeHeadingVelocities(vx, vy, omega, seg, V, H, A);

    // #region agent_debug_log
    // Trace omega from motionQueue through to motor duty — verify stabilization is working
    {
        static unsigned long lastDebugTime = 0;
        if (millis() - lastDebugTime >= 100) {
            float heading_err = 0;
            float dist_rem = 0;
            if (seg && seg->state == SegmentState::ACTIVE) {
                // Use blended angle for heading error to match what stabilization uses
                float stable_angle = c_angle;
                if (!motionQueue.getBlendedAngle(stable_angle)) {
                    stable_angle = c_angle;  // no camera yet — fall back to DR
                }
                heading_err = (Rotation(seg->target_angle) - Rotation(stable_angle));
                float dx = seg->target_x - cx;
                float dy = seg->target_y - cy;
                dist_rem = sqrtf(dx*dx + dy*dy);
            }
            float gtAngle = latencyComp.getLastObservedAngle();
            logger.update("STAB", "err=%.1f omega=%.1f A=%.3f dist=%.1f V=%.2f H:%.2f gt=%.1f",
                heading_err, omega, A, dist_rem, V, H, isnan(gtAngle) ? 0.0f : gtAngle);
            lastDebugTime = millis();
        }
    }
    // #endregion

    // --- Detect operating mode ---
    float cmdMag = sqrtf(vx * vx + vy * vy);
    bool lowSpeedMode = (cmdMag > 0.001f && cmdMag < 10.0f);
    bool precisionMode = false;
    bool singleMotorMode = false;
    detectOperatingMode(seg, cmdMag, cx, cy, precisionMode, singleMotorMode);

    // --- Compute motor duties ---
    MecanumSpeeds s;
    if (singleMotorMode) {
        s = computeSingleMotorSpeeds(V, H);
    } else {
        s = computeMecanumSpeeds(V, H, A, lowSpeedMode, precisionMode);
    }

    logger.update("MOTOR_CMD", "singleMotor=%d M1=%d M2=%d M3=%d M4=%d",
        singleMotorMode, s.m1, s.m2, s.m3, s.m4);

#if ENABLE_DEBUG_LOGGING
    static unsigned long lastDebugTime = 0;
    if (millis() - lastDebugTime >= 100) {
        logger.update("MOTORS", "M1:%d M2:%d M3:%d M4:%d | T: V:%.2f H:%.2f A:%.2f",
            s.m1, s.m2, s.m3, s.m4, V, H, A);
        lastDebugTime = millis();
    }
#endif

#ifdef TEST_MODE
    static unsigned long lastPrintTime = 0;
    if (millis() - lastPrintTime >= 500) {
        logger.update("TEST", "Virtual Motors: M1=%d, M2=%d, M3=%d, M4=%d  V=%.2f H=%.2f A=%.2f",
            s.m1, s.m2, s.m3, s.m4, V, H, A);
        lastPrintTime = millis();
    }
#else
    logger.update("DEBUG", "RUNNING: M1=%d M2=%d M3=%d M4=%d", s.m1, s.m2, s.m3, s.m4);
    Motor1.Run(s.m1);
    Motor2.Run(s.m2);
    Motor3.Run(s.m3);
    Motor4.Run(s.m4);
#endif

    lastMotionWasRotationOnly = rotationOnly;
    lastRotationOmegaDegS = omega;
}

// ============================================================================
// Enqueue helper with automatic pose fetch and success/error logging
// ============================================================================
static bool enqueueAndLog(const char* cmdName,
                           std::function<bool(float cx, float cy, float c_angle)> enqueueFn) {
    float cx, cy, c_angle;
    getCurrentPose(cx, cy, c_angle);
    bool ok = enqueueFn(cx, cy, c_angle);
    if (!ok) {
        logger.important("[ERR] %s rejected — queue full!", cmdName);
    }
    return ok;
}

// ============================================================================
// Process a pre-parsed UDP message (called from Core 0 under mutex)
// ============================================================================
void handleParsedMessage(const UdpMessage &msg) {
    switch (msg.type) {
        case MsgType::MOVE: {
            if (enqueueAndLog("MOVE", [&](float cx, float cy, float c_angle) {
                return motionQueue.enqueue(
                    msg.direction, msg.distance_mm,
                    msg.speed, msg.correctionPolicy, msg.servoAction,
                    cx, cy, c_angle);
            })) {
                logger.log("[CMD] MOVE: dir=%d dist=%d spd=%d policy=%d (q=%d)",
                    (int)msg.direction, msg.distance_mm, (int)msg.speed,
                    (int)msg.correctionPolicy, motionQueue.remaining());
            }
            break;
        }

        case MsgType::MOVE_DURATION: {
            if (enqueueAndLog("MOVE_DUR", [&](float cx, float cy, float c_angle) {
                return motionQueue.enqueueDuration(
                    msg.direction, msg.duration_ms,
                    msg.speed, msg.correctionPolicy,
                    cx, cy, c_angle);
            })) {
                logger.log("[CMD] MOVE_DUR: dir=%d dur=%lums spd=%d policy=%d (q=%d)",
                    (int)msg.direction, (unsigned long)msg.duration_ms, (int)msg.speed,
                    (int)msg.correctionPolicy, motionQueue.remaining());
            }
            break;
        }

        case MsgType::ROTATE_DURATION: {
            if (enqueueAndLog("ROT_DUR", [&](float cx, float cy, float c_angle) {
                return motionQueue.enqueueRotateDuration(
                    msg.rotationDirection, msg.duration_ms,
                    msg.speed, msg.correctionPolicy,
                    cx, cy, c_angle);
            })) {
                logger.log("[CMD] ROT_DUR: dir=%d dur=%lums spd=%d policy=%d (q=%d)",
                    (int)msg.rotationDirection, (unsigned long)msg.duration_ms, (int)msg.speed,
                    (int)msg.correctionPolicy, motionQueue.remaining());
            }
            break;
        }

        case MsgType::WAYPOINT: {
            float cx, cy, c_angle;
            getCurrentPose(cx, cy, c_angle);
            const MotionSegment* cur = motionQueue.currentSegment();
            bool sameTarget = (cur != nullptr &&
                              !cur->isDurationBased &&
                              (cur->state == SegmentState::ACTIVE ||
                               cur->state == SegmentState::HOLDING) &&
                              fabsf(cur->target_x - msg.target_x) < WAYPOINT_SAME_TARGET_TOL_MM &&
                              fabsf(cur->target_y - msg.target_y) < WAYPOINT_SAME_TARGET_TOL_MM &&
                              msg.servoAction == ServoAction::NONE);

            if (!sameTarget) {
                motionQueue.abort();
                bool ok = motionQueue.enqueueWaypoint(
                    msg.target_x, msg.target_y, msg.target_angle,
                    msg.speed, msg.correctionPolicy, msg.servoAction,
                    cx, cy, c_angle
                );
                if (ok) {
                    logger.log("[CMD] WAYPOINT: (%.1f, %.1f) @ %.1f°",
                        msg.target_x, msg.target_y, msg.target_angle);
                } else {
                    logger.important("[ERR] WAYPOINT rejected!");
                }
            } else {
                logger.log("[CMD] WAYPOINT (same target) - continuing");
            }
            break;
        }

        case MsgType::DONE: {
            logger.important("[MISSION] DONE!");
            doneFeedbackBlinks = DONE_FEEDBACK_BLINKS;
            break;
        }

        case MsgType::ROTATE: {
            float cx, cy, c_angle;
            getCurrentPose(cx, cy, c_angle);
            motionQueue.abort();
            bool ok = motionQueue.enqueueRotate(
                msg.target_angle, msg.speed, msg.correctionPolicy,
                cx, cy, c_angle
            );
            if (ok) {
                logger.log("[CMD] ROTATE: target=%.1f spd=%d policy=%d",
                    msg.target_angle, (int)msg.speed,
                    (int)msg.correctionPolicy);
            } else {
                logger.important("[ERR] ROTATE rejected!");
            }
            break;
        }

        case MsgType::CAM: {
            latencyComp.onCameraUpdate(
                msg.cam_timestamp, msg.cam_x, msg.cam_y, msg.cam_angle,
                CORRECTION_BLEND_MS
            );
            break;
        }

        case MsgType::PING: {
            char pongBuf[32];
            int pongLen = buildPongMessage(pongBuf, sizeof(pongBuf), msg.ping_timestamp);
            udp.beginPacket(udp.remoteIP(), udpPort);
            udp.write((uint8_t*)pongBuf, pongLen);
            udp.endPacket();
            break;
        }

        case MsgType::PONG: {
            latencyComp.onPong(msg.ping_timestamp, msg.remote_timestamp);
            logger.log("[RTT] %lu ms", (unsigned long)latencyComp.getRttMs());
            break;
        }

        case MsgType::VELOCITY: {
            float cx, cy, c_angle;
            getCurrentPose(cx, cy, c_angle);
            motionQueue.abort();
            bool ok = motionQueue.enqueueVelocity(
                msg.vel_vx, msg.vel_vy, msg.vel_omega,
                msg.duration_ms,
                cx, cy, c_angle
            );
            if (ok) {
                logger.log("[CMD] VELOCITY: vx=%.1f vy=%.1f ω=%.1f timeout=%lums",
                    msg.vel_vx, msg.vel_vy, msg.vel_omega,
                    (unsigned long)msg.duration_ms);
            } else {
                logger.important("[ERR] VELOCITY rejected!");
            }
            break;
        }

        case MsgType::ABORT: {
            logger.important("[CMD] ABORT — emergency stop!");
            motionQueue.abort();
            currentVx = currentVy = currentOmega = 0;
            abortFlag = true;
            break;
        }

        case MsgType::SET_SERIAL_MONITOR: {
            bool enabled = msg.duration_ms != 0;
            logger.setEnabled(enabled);
            logger.log("[SYS] Serial Monitor %s", enabled ? "ENABLED" : "DISABLED");
            break;
        }

        case MsgType::SERVO_EXEC: {
            logger.log("[CMD] SERVO_EXEC: %d", (int)msg.servoAction);
            executeServoAction(msg.servoAction);
            rearmKickstart();
            break;
        }

        default:
            logger.log("[UDP] Unhandled message type: %d", (int)msg.type);
            break;
    }
}

// ============================================================================
// Send telemetry status to phone (every STATUS_INTERVAL_MS)
// ============================================================================
void sendStatus() {
    float x, y, angle;
    deadReckoning.getCurrentPosition(x, y, angle);
    // Prefer camera ground-truth angle over dead-reckoning for App display.
    // Dead-reckoning drifts over time; camera angle is the authoritative truth.
    float gtAngle = latencyComp.getLastObservedAngle();
    if (!isnan(gtAngle)) angle = gtAngle;

    char buf[80];
    int len = buildStatusMessage(buf, sizeof(buf),
        x, y, angle, motionQueue.remaining(), latencyComp.getLastDriftMagnitude());

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
    
    logger.log("[SYS] Robot Online: %s", BOT_ID);
    logger.log("[SYS] IP: %s", WiFi.localIP().toString().c_str());

    udp.begin(udpPort);

    // --- Initialize subsystems ---
    deadReckoning.reset(0, 0, 0);
    deadReckoning.setDistanceFactors(DISTANCE_FACTOR_H, DISTANCE_FACTOR_V);

    initServoControl();
    
    pinMode(LED, OUTPUT);
    digitalWrite(LED, LOW);

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
            // Capture state under mutex, then send outside
            float x, y, angle;
            deadReckoning.getCurrentPosition(x, y, angle);
            lastStatusTime = now;
            xSemaphoreTake(stateMutex, portMAX_DELAY);
            int qRemaining = motionQueue.remaining();
            float driftMag = latencyComp.getLastDriftMagnitude();
            // Prefer camera ground-truth angle for App display
            float gtAngle = latencyComp.getLastObservedAngle();
            if (!isnan(gtAngle)) angle = gtAngle;
            xSemaphoreGive(stateMutex);

            char buf[80];
            int len = buildStatusMessage(buf, sizeof(buf),
                x, y, angle, qRemaining, driftMag);
            udp.beginPacket(phoneIP, udpPort);
            udp.write((uint8_t*)buf, len);
            udp.endPacket();
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
    // Use camera ground-truth angle for stabilization when available.
    // The dead-reckoning angle drifts; stabilization must use the authoritative
    // camera angle to correctly compute heading error during translation moves.
    float current_x, current_y, current_angle;
    deadReckoning.getCurrentPosition(current_x, current_y, current_angle);
    float gtAngle = latencyComp.getLastObservedAngle();
    bool hasGt = !isnan(gtAngle);
    bool moving = motionQueue.tick(dt, current_x, current_y, current_angle, gtAngle, hasGt);

    ServoAction actionToExecute = ServoAction::NONE;
    if (motionQueue.segmentJustCompleted) {
        actionToExecute = motionQueue.getLastCompletedServoAction();
        // Clear flag now that we've captured the action
        motionQueue.segmentJustCompleted = false;

        if (motionQueue.getActivePolicy() == CorrectionPolicy::DEFERRED) {
            float dcX, dcY, dcA;
            motionQueue.getDeferredCorrection(dcX, dcY, dcA);
            // Apply deferred correction as a new anchor on dead reckoning
            float gx, gy, ga;
            deadReckoning.getCurrentPosition(gx, gy, ga);
            deadReckoning.setAnchor(gx + dcX, gy + dcY, ga + dcA, millis());
        }
    }

    // ---- Handle emergency deceleration ----
    if (latencyComp.wasEmergencyTriggered() && moving) {
        Serial.println("[LC] Emergency correction triggered — continuing with corrected state");
    }

    // ---- Apply motor speeds ----
    applyMotors();

    // DEBUG: Wiggle diagnostics — only logs when near target
    motionQueue.debugLogWiggle(current_x, current_y, current_angle);

    // ---- Update dead-reckoning ----
    // Use the same dt calculated at the top of the loop for consistency
    deadReckoning.update(currentVx, currentVy, currentOmega, dt);

    xSemaphoreGive(stateMutex);

    // ---- Execution (OUTSIDE mutex) ----
    // This allows Core 0 to handle heartbeats and new waypoint commands 
    // while the robot is busy with its servo sequence.
    if (actionToExecute != ServoAction::NONE) {
        Serial.printf("[SERVO] Executing Action: %d\n", (int)actionToExecute);
        logger.log("[SERVO] Executing Action: %d", (int)actionToExecute);
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
#else
    // Creative feedback blinking for DONE command
    if (doneFeedbackBlinks > 0) {
        if (now - lastFeedbackToggleTime >= 100) {
            lastFeedbackToggleTime = now;
            bool state = (doneFeedbackBlinks % 2 != 0);
            digitalWrite(LED, state ? HIGH : LOW);
            doneFeedbackBlinks--;
        }
    }
#endif

    // #region agent_debug_log H3: motor control state diagnostics (every 5s)
    {
        static unsigned long lastDiagMs = 0;
        if (now - lastDiagMs >= 5000) {
            lastDiagMs = now;
            float ax, ay;
            deadReckoning.getAnchor(ax, ay);
            float odoX, odoY, odoAngle;
            deadReckoning.getOdoPosition(odoX, odoY, odoAngle);
            float odoMag = sqrtf(odoX*odoX + odoY*odoY);
            float anchorMag = sqrtf(ax*ax + ay*ay);
            Serial.printf(
                "{\"sessionId\":\"eb5734\",\"id\":\"main_%lu\",\"timestamp\":%lu,"
                "\"location\":\"main.cpp:loop\",\"message\":\"H3_motor_ramp_state\",\"hypothesisId\":\"H3\","
                "\"data\":{\"motorRampFactor\":%.4f,\"isMoving\":%d,"
                "\"motorWasMoving\":%d,\"lastMotionWasRotationOnly\":%d,"
                "\"mk0\":%d,\"mk1\":%d,\"mk2\":%d,\"mk3\":%d,"
                "\"rotBrakeFrames\":%d,\"transBrakeFrames\":%d,"
                "\"odoMag\":%.1f,\"anchorMag\":%.1f,\"odoAngle\":%.1f}}\n",
                now, now,
                motorRampFactor, isMoving,
                motorWasMoving, lastMotionWasRotationOnly,
                mkickstart[0], mkickstart[1], mkickstart[2], mkickstart[3],
                rotationBrakeFrames, translationBrakeFrames,
                (double)odoMag, (double)anchorMag, (double)odoAngle);
        }
    }
    // #endregion
}