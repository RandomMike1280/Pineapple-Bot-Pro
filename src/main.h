#ifndef MAIN_H
#define MAIN_H

// ============================================================================
// SWITCH ROBOT HERE — pick one, comment out the other
// ============================================================================
// Before uploading, ensure the correct config is selected below.
// Each config file contains its own WiFi credentials and robot-specific
// calibration values; all other constants are shared defaults.
#include "config_a.h"   // <-- Robot A
// #include "config_b.h" // <-- Robot B (uncomment this, comment out config_a.h)

// If you see this error, no robot config is selected. Uncomment one above.
#ifdef CONFIG_DEFAULTS_H
// good
#else
#error "No robot config selected — include config_a.h or config_b.h above."
#endif

// ============================================================================
// External Library Headers
// ============================================================================

#include <Arduino.h>
#include <esp32_motor.hpp>
#include <esp32_servo.hpp>
#include <udp_protocol.hpp>

// ============================================================================
// === Calibration: Speed & Distance Scaling ===
// ============================================================================
// ============================================================================
// === Robot Identity ===
// ============================================================================
// Comment out IsRobotB → Robot A, ArUco marker ID 0
// Uncomment IsRobotB   → Robot B, ArUco marker ID 1
//#define IsRobotB

#ifndef IsRobotB
  #define BOT_ID                   "A"
  #define ARUCO_ID                 0
#else
  #define BOT_ID                   "B"
  #define ARUCO_ID                 1
#endif

#ifndef LED
#define LED 2                        // Default LED pin if not defined elsewhere
#endif

#define SPEED_SCALING_FACTOR     1.0f      // Global multiplier for all movement speeds
#define ROTATION_SCALING_FACTOR  1.0f      // Specific multiplier for rotational speed
#define DISTANCE_FACTOR_V        1.0f      // Calibrates dead-reckoning for Vertical (Forward/Backward)
#define DISTANCE_FACTOR_H        1.25f     // Calibrates dead-reckoning for Horizontal (Strafe)

// Measured from PS2 controller physics:
//   Linear speed at max duty (100): ~72.5 mm/s (7.25 cm/s)
//   Rotation at max duty (100): 86.4 deg/s (0.24 interval/sec)
#define SPEED_SLOW_MM_S    (29.0f * SPEED_SCALING_FACTOR)
#define SPEED_NORMAL_MM_S  (43.5f * SPEED_SCALING_FACTOR)
#define SPEED_FAST_MM_S    (72.5f * SPEED_SCALING_FACTOR)

// Rotation speed calibration (deg/s at each speed level)
#define SPEED_SLOW_DEG_S    (15.0f * SPEED_SCALING_FACTOR * ROTATION_SCALING_FACTOR)
#define SPEED_NORMAL_DEG_S  (51.84f * SPEED_SCALING_FACTOR * ROTATION_SCALING_FACTOR)
#define SPEED_FAST_DEG_S    (86.4f * SPEED_SCALING_FACTOR * ROTATION_SCALING_FACTOR)

// ============================================================================
// === Motor Physics ===
// ============================================================================
#define ENABLE_DEBUG_LOGGING     1        // Set to 1 to enable detailed serial logs
#define DRIVE_CLAMP_LOW          65       // Minimum duty to overcome static friction
#define DRIVE_CLAMP_HIGH         100      // Maximum allowable duty (safety cap)
#define DRIVE_CLAMP_FINE         65       // Lower duty floor for precision mode (raised from 28 to overcome dead zone)
#define DRIVE_CLAMP_MINIMAL      25       // Ultra-low floor for single-motor micro-adjustments
#define MOTOR_MAP_SLOPE          43.0174f
#define MOTOR_MAP_OFFSET         57.1650f
#define MOTOR_STOP_THRESHOLD     35       // Below this duty, motors stall

// Kickstart: briefly boost duty to overcome static friction
#define KICKSTART_FRAMES        9         // 3x at 1kHz (was 3 at 333Hz, ~9ms kickstart)
#define KICKSTART_SPEED         70.0

// Acceleration ramp — RAMP_DURATION_MS stays at 75ms physical time
#define RAMP_DURATION_MS        75         // ms to reach full speed from start
#define RAMP_START_FRACTION     0.15f   // initial duty fraction (just enough to overcome friction)

// ============================================================================
// === Precision Control ===
// ============================================================================
#define PRECISION_MODE_THRESH_MM 50       // Enter precision mode when remaining distance < this
#define WAYPOINT_SAME_TARGET_TOL_MM 15.0f  // Threshold for "same target" deduplication
#define PRECISION_KICKSTART_SUPPRESS true // Disable kickstart in precision mode to prevent wiggle
#define PRECISION_MIN_SPEED_LIMIT_MM_S 18.0f
#define PRECISION_MIN_ROT_LIMIT_DEG_S  5.0f
#define CLOSE_APPROACH_DISTANCE_MM     120.0f
#define CLOSE_ROT_APPROACH_DEG         8.0f
#define WAYPOINT_TOLERANCE_MM    5.0f     // Tighten tolerance for arrival
#define ROTATION_TOLERANCE_DEG   1.5f

// ============================================================================
// === Drift Correction ===
// ============================================================================
// Counteract the systematic leftward bias of the mecanum physics.
#define DRIFT_TRIM_SLOW_DEG    9.36f
#define DRIFT_TRIM_NORMAL_DEG  0.00f   // disabled — was rotating aim vector and hurting straight-line tracking
#define DRIFT_TRIM_FAST_DEG    0.00f

#define CAMERA_LATENCY_MS        150      // Default delay compensation for OpenCV/Network
#define DRIFT_THRESHOLD_MM       5.0f     // apply full correction above this
#define EMERGENCY_THRESHOLD_MM   25.0f    // decelerate + correct above this
#define CORRECTION_BLEND_MS      200      // ms to blend corrections smoothly

// ============================================================================
// === Deceleration & Stabilization ===
// ============================================================================
#define DECCEL_DISTANCE_MM       500.0f   // Start slowing down at this distance (CNC-style long decel)
#define ROT_DECCEL_DEG           15.0f    // Start slowing down rotation at this angle
#define MIN_SPEED_LIMIT_MM_S     10.0f    // Floor speed during deccel (prevent stall)
#define MIN_ROT_LIMIT_DEG_S      5.0f     // Floor rotation during deccel
#define STABILIZATION_GAIN       2.5f     // Corrective OMEGA per degree of error
#define MAX_STABILIZATION_OMEGA  35.0f    // Max deg/s allowed for in-move correction

// ============================================================================
// === Trajectory Planning ===
// ============================================================================

// Predictive Steering — anticipate drift/momentum using observed velocity
// The robot tracks its own velocity from position deltas, predicts where it
// will be in LOOKAHEAD seconds, and steers toward the target from there.
// This compensates for momentum/drift before it becomes a large error.
#define PREDICTIVE_LOOKAHEAD_S   0.15f    // seconds to predict ahead (0.15s ≈ 11mm at fast speed)

// Predictive Braking — physics-based overshoot prevention
// Uses observed closing speed and kinematic equation v² = 2*a*d to decide
// if the robot can stop before the target.  If not, speed_scale is reduced.
#define PREDICTIVE_BRAKE_DECEL_MM_S2   350.0f   // assumed deceleration capability (mm/s²)
#define PREDICTIVE_BRAKE_SAFETY        1.2f     // safety margin (>1 = start braking earlier)

// Latency-Aware Speed Governor — cap speed so robot can always stop within
// the distance it would travel during one network round-trip.
// Uses kinematic equation: max_speed = sqrt(2 * decel * dist_remaining)
#define LATENCY_AWARE_DECEL_MM_S2  300.0f  // conservative deceleration for latency cap

// S-Curve Trajectory Profile (Jerk-Limited Motion)
// Instead of instantly commanding full speed, the trajectory generator
// ramps acceleration smoothly using bounded jerk.  This produces an
// S-shaped velocity curve that reduces vibration and overshoot.
#define SCURVE_MAX_ACCEL_MM_S2   250.0f   // max linear acceleration (mm/s²)
#define SCURVE_MAX_JERK_MM_S3    1200.0f  // max rate-of-change of acceleration (mm/s³)
#define SCURVE_MAX_ROT_ACCEL_DEG_S2  180.0f  // max rotational acceleration (deg/s²)
#define SCURVE_MAX_ROT_JERK_DEG_S3   900.0f  // max rotational jerk (deg/s³)

// Feedforward Compensation
// Pre-compensate for inertia and friction by injecting extra motor command
// proportional to desired acceleration and velocity.  Reduces tracking lag.
#define FEEDFORWARD_KV           0.02f    // velocity feedforward gain (duty per mm/s)
#define FEEDFORWARD_KA           0.04f    // acceleration feedforward gain (duty per mm/s²)
#define FEEDFORWARD_KV_ROT       0.01f    // rotational velocity feedforward
#define FEEDFORWARD_KA_ROT       0.02f    // rotational acceleration feedforward

// Adaptive Lookahead — speed-proportional predictive steering
// Instead of a fixed lookahead time, scale with current speed.
// lookahead = BASE + speed * GAIN, clamped to [MIN, MAX]
#define ADAPTIVE_LOOKAHEAD_BASE_S  0.06f   // base lookahead even at zero speed
#define ADAPTIVE_LOOKAHEAD_GAIN    0.0022f // seconds of lookahead per mm/s of speed
#define ADAPTIVE_LOOKAHEAD_MIN_S   0.04f   // minimum lookahead (seconds)
#define ADAPTIVE_LOOKAHEAD_MAX_S   0.18f   // maximum lookahead (seconds)

// Kalman Filter — velocity estimation (replaces EMA)
// 1D Kalman per axis: state = [position, velocity]
// Process noise Q and measurement noise R tune responsiveness vs smoothness.
#define KALMAN_PROCESS_NOISE_POS   0.5f    // position process noise variance
#define KALMAN_PROCESS_NOISE_VEL   50.0f   // velocity process noise variance
#define KALMAN_MEASUREMENT_NOISE   2.0f    // position measurement noise variance

// Anti-Slip / Stall Detection
// If commanded speed > threshold but observed speed < threshold for N ticks,
// the robot is likely stalled or slipping.  Boost command to break free.
#define SLIP_CMD_SPEED_THRESH_MM_S  20.0f  // min commanded speed to monitor slip (raised from 15)
#define SLIP_OBS_SPEED_THRESH_MM_S   5.0f  // observed speed below this = stalled
#define SLIP_DETECT_TICKS           75      // 75ms physical time equivalent (was 25 at 333Hz)
#define SLIP_BOOST_FACTOR            1.35f // multiplicative boost when stall detected
#define SLIP_BOOST_MAX_TICKS        100     // 100ms physical time equivalent (was 40 at 333Hz)

// ============================================================================
// === Timing ===
// ============================================================================
#define STATUS_INTERVAL_MS       100      // telemetry broadcast interval
#define HELLO_INTERVAL_MS        3000    // auto-discovery interval
#define PING_INTERVAL_MS         500      // RTT measurement interval
#define CONTROL_LOOP_INTERVAL_MS 1        // main loop target period (1kHz — 3× faster than 333Hz)
#define DONE_FEEDBACK_BLINKS     4        // number of LED blinks on DONE command

// ============================================================================
// === Mecanum Kinematics ===
// ============================================================================
// Motor layout (top view):
//   M4(FL) ---- M1(FR)
//     |            |
//   M3(BL) ---- M2(BR)
//
// PS2 mecanum formula (V, H, A in normalized [-1, 1] range):
//   FR = -V + H + A
//   BR = -V - H + A
//   BL =  V - H + A
//   FL =  V + H + A
//
// Convention: V > 0 = forward, H > 0 = right, A > 0 = CW rotation

// Brake parameters — frames scale with control rate (1kHz = 3x original 333Hz)
#define ROTATION_BRAKE_FRAMES       15        // was 5 at 333Hz → 15ms at 1kHz
#define ROTATION_BRAKE_DUTY         75
#define ROTATION_BRAKE_MIN_OMEGA_DEG_S 5.0f
#define TRANSLATION_BRAKE_FRAMES     18        // was 6 at 333Hz → 18ms at 1kHz
#define TRANSLATION_BRAKE_DUTY      80
#define TRANSLATION_BRAKE_MIN_SPEED_MM_S 5.0f

// ============================================================================
// === Motor Physics API ===
// ============================================================================

// speed_to_motor_duty: converts normalized speed [-1,1] to motor duty [-100,100]
// From PS2 calibration: duty = 43.0174134490 * speed + 57.16498038405947 * sign(speed)
// This compensates for the motor dead zone so that the robot moves linearly
// with the input speed value.
int speed_to_motor_duty(double speed);

struct MecanumSpeeds {
    int m1;  // Front Right
    int m2;  // Back Right
    int m3;  // Back Left
    int m4;  // Front Left
};

/// Compute individual motor duties from normalized velocity components.
/// V = forward/backward (-1..1), H = strafe (-1..1), A = rotation (-1..1)
/// Applies speed_to_motor_duty, normalization, and kickstart.
/// @param precisionMode when true, uses lower duty floor for fine positioning
MecanumSpeeds computeMecanumSpeeds(double V, double H, double A,
                                  bool lowSpeedMode = false,
                                  bool precisionMode = false);

/// Compute single-motor duty for ultra-fine micro-adjustments.
/// Only one motor is active at a time, producing ~1-2mm steps.
/// Uses DRIVE_CLAMP_MINIMAL duty floor (18).
MecanumSpeeds computeSingleMotorSpeeds(double V, double H);

// ============================================================================
// Robot Control API
// ============================================================================

/// Apply motor speeds from current motion queue state (~333 Hz loop on Core 1)
void applyMotors();

/// Process a pre-parsed UDP message (called from Core 0 under mutex)
void handleParsedMessage(const UdpMessage &msg);

// ============================================================================
// Communication API
// ============================================================================

/// Send telemetry status to phone (every STATUS_INTERVAL_MS)
void sendStatus();

/// Send HELLO beacon for auto-discovery (every HELLO_INTERVAL_MS)
void sendHello();

/// Send PING for RTT measurement (every PING_INTERVAL_MS)
void sendPing();

// ============================================================================
// Dual-Core API
// ============================================================================

/// Core 0 task: UDP packet processing, command parsing, periodic broadcasts
void udpTask(void* parameter);

#endif // MAIN_H
