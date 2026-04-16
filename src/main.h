#ifndef MAIN_H
#define MAIN_H

#include <Arduino.h>
#include <esp32_motor.hpp>
#include <esp32_servo.hpp>

// ============================================================================
// Robot Identity — change to "B" when flashing the second robot
// ============================================================================
#define BOT_ID "A"

// ============================================================================
// Operating Mode
// ============================================================================
// #define TEST_MODE // Uncomment to enable test mode (no motors, print commands, blinking LED)

#ifndef LED
#define LED 2 // Default LED pin if not defined elsewhere
#endif

// ============================================================================
// TUNING PARAMETERS
// ============================================================================
// Global scaling factors for physical distances and speeds
#define SPEED_SCALING_FACTOR     1.0f     // Global multiplier for all movement speeds
#define ROTATION_SCALING_FACTOR  1.0f     // Specific multiplier for rotational speed
#define DISTANCE_FACTOR_V        1.0f     // Calibrates dead-reckoning for Vertical (Forward/Backward)
#define DISTANCE_FACTOR_H        1.25f    // Calibrates dead-reckoning for Horizontal (Strafe)

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
// Diagnostics & Safety
// ============================================================================
#define ENABLE_DEBUG_LOGGING     0        // Set to 1 to enable detailed serial logs
#define DRIVE_CLAMP_LOW          80       // Minimum duty to overcome static friction
#define DRIVE_CLAMP_HIGH         100      // Maximum allowable duty (safety cap)
#define NEAR_TARGET_MOTOR_MIN    78       // Higher floor for motor duty when close to target

// Motor mapping constants
#define MOTOR_MAP_SLOPE          43.0174f
#define MOTOR_MAP_OFFSET         57.1650f

// ============================================================================
// Translation Drift Trim (speed-dependent)
// ============================================================================
// Counteract the systematic leftward bias of the mecanum physics.
#define DRIFT_TRIM_SLOW_DEG    9.36f
#define DRIFT_TRIM_NORMAL_DEG  2.50f
#define DRIFT_TRIM_FAST_DEG    0.00f

// ============================================================================
// Latency / Correction Tuning
// ============================================================================
#define CAMERA_LATENCY_MS        150      // Default delay compensation for OpenCV/Network
#define DRIFT_THRESHOLD_MM       5.0f     // apply full correction above this
#define EMERGENCY_THRESHOLD_MM   25.0f    // decelerate + correct above this
#define CORRECTION_BLEND_MS      200      // ms to blend corrections smoothly

// ============================================================================
// Precision & Deceleration
// ============================================================================
#define DECCEL_DISTANCE_MM       400.0f   // Start slowing down earlier for more controlled stops
#define ROT_DECCEL_DEG           15.0f    // Start slowing down rotation at this angle
#define MIN_SPEED_LIMIT_MM_S     15.0f    // Floor speed during deccel (prevent stall)
#define MIN_ROT_LIMIT_DEG_S      8.0f     // Floor rotation during deccel
#define PRECISION_MIN_SPEED_LIMIT_MM_S 15.0f
#define PRECISION_MIN_ROT_LIMIT_DEG_S  8.0f
#define CLOSE_APPROACH_DISTANCE_MM     100.0f  // Narrow precision zone
#define CLOSE_ROT_APPROACH_DEG         8.0f
#define WAYPOINT_TOLERANCE_MM    5.0f     // Tighten tolerance for arrival
#define ROTATION_TOLERANCE_DEG   1.5f

// ============================================================================
// Active Heading Stabilization (Holonomic Control)
// ============================================================================
// These constants define how the robot corrects its heading while moving.
// Increase the gain if the robot is "lazy" about correcting drift.
// Decrease the gain if the robot "oscillates" or shakes while strafing.
#define STABILIZATION_GAIN       2.5f     // Corrective OMEGA per degree of error
#define MAX_STABILIZATION_OMEGA  35.0f    // Max deg/s allowed for in-move correction

// ============================================================================
// Predictive Steering — anticipate drift/momentum using observed velocity
// ============================================================================
// The robot tracks its own velocity from position deltas, predicts where it
// will be in LOOKAHEAD seconds, and steers toward the target from there.
// This compensates for momentum/drift before it becomes a large error.
#define PREDICTIVE_LOOKAHEAD_S   0.15f    // seconds to predict ahead (0.15s ≈ 11mm at fast speed)

// Predictive Braking — physics-based overshoot prevention
// Uses observed closing speed and kinematic equation v² = 2*a*d to decide
// if the robot can stop before the target.  If not, speed_scale is reduced.
#define PREDICTIVE_BRAKE_DECEL_MM_S2   450.0f   // Higher decel = brake sooner/more aggressively
#define PREDICTIVE_BRAKE_SAFETY        1.2f     // Lower safety margin = more aggressive

// ============================================================================
// S-Curve Trajectory Profile (Jerk-Limited Motion)
// ============================================================================
// Instead of instantly commanding full speed, the trajectory generator
// ramps acceleration smoothly using bounded jerk.  This produces an
// S-shaped velocity curve that reduces vibration and overshoot.
#define SCURVE_MAX_ACCEL_MM_S2   250.0f   // max linear acceleration (mm/s²)
#define SCURVE_MAX_JERK_MM_S3    1200.0f  // max rate-of-change of acceleration (mm/s³)
#define SCURVE_MAX_ROT_ACCEL_DEG_S2  180.0f  // max rotational acceleration (deg/s²)
#define SCURVE_MAX_ROT_JERK_DEG_S3   900.0f  // max rotational jerk (deg/s³)
#define SCURVE_MAX_DECEL_MM_S2   600.0f   // Higher deceleration for faster braking
#define SCURVE_MAX_DECEL_JERK_MM_S3 2400.0f // Faster decel ramp

// ============================================================================
// Feedforward Compensation
// ============================================================================
// Pre-compensate for inertia and friction by injecting extra motor command
// proportional to desired acceleration and velocity.  Reduces tracking lag.
#define FEEDFORWARD_KV           0.02f    // velocity feedforward gain (duty per mm/s)
#define FEEDFORWARD_KA           0.04f    // acceleration feedforward gain (duty per mm/s²)
#define FEEDFORWARD_KV_ROT       0.01f    // rotational velocity feedforward
#define FEEDFORWARD_KA_ROT       0.02f    // rotational acceleration feedforward

// ============================================================================
// Adaptive Lookahead — speed-proportional predictive steering
// ============================================================================
// Instead of a fixed lookahead time, scale with current speed.
// lookahead = BASE + speed * GAIN, clamped to [MIN, MAX]
#define ADAPTIVE_LOOKAHEAD_BASE_S  0.06f   // base lookahead even at zero speed
#define ADAPTIVE_LOOKAHEAD_GAIN    0.0022f // seconds of lookahead per mm/s of speed
#define ADAPTIVE_LOOKAHEAD_MIN_S   0.04f   // minimum lookahead (seconds)
#define ADAPTIVE_LOOKAHEAD_MAX_S   0.18f   // maximum lookahead (seconds)

// ============================================================================
// Kalman Filter — velocity estimation (replaces EMA)
// ============================================================================
// 1D Kalman per axis: state = [position, velocity]
// Process noise Q and measurement noise R tune responsiveness vs smoothness.
#define KALMAN_PROCESS_NOISE_POS   0.5f    // position process noise variance
#define KALMAN_PROCESS_NOISE_VEL   50.0f   // velocity process noise variance
#define KALMAN_MEASUREMENT_NOISE   2.0f    // position measurement noise variance

// ============================================================================
// Anti-Slip / Stall Detection
// ============================================================================
// If commanded speed > threshold but observed speed < threshold for N ticks,
// the robot is likely stalled or slipping.  Boost command to break free.
#define SLIP_CMD_SPEED_THRESH_MM_S  15.0f  // min commanded speed to monitor slip
#define SLIP_OBS_SPEED_THRESH_MM_S   5.0f  // observed speed below this = stalled
#define SLIP_DETECT_TICKS           25     // consecutive ticks to confirm stall (~75ms at 333Hz)
#define SLIP_BOOST_FACTOR            1.35f // multiplicative boost when stall detected
#define SLIP_BOOST_MAX_TICKS        40     // max ticks to apply boost before giving up

// ============================================================================
// Timing Constants
// ============================================================================
#define STATUS_INTERVAL_MS       100      // telemetry broadcast interval
#define HELLO_INTERVAL_MS        3000     // auto-discovery interval
#define PING_INTERVAL_MS         500      // RTT measurement interval
#define CONTROL_LOOP_INTERVAL_MS 3        // main loop target period (~333 Hz)

// ============================================================================
// Mecanum Kinematics — from PS2 controller physics
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

// Motor stop threshold — below this duty, motors stall
#define MOTOR_STOP_THRESHOLD    35

// Kickstart: briefly boost duty to overcome static friction
#define KICKSTART_FRAMES        3
#define KICKSTART_SPEED         70.0
#define ROTATION_BRAKE_FRAMES   5
#define ROTATION_BRAKE_DUTY     75
#define ROTATION_BRAKE_MIN_OMEGA_DEG_S 5.0f

// Translation active brake: reverse motors briefly to kill physical momentum
// when the queue commands stop but the robot is still coasting.
#define TRANSLATION_BRAKE_FRAMES      4       // frames of reverse thrust (~12ms at 333Hz)
#define TRANSLATION_BRAKE_DUTY        55      // reverse duty (less than rotation — wheels grip better)
#define TRANSLATION_BRAKE_MIN_SPEED_MM_S 5.0f // only brake if observed speed above this

// Acceleration ramp: gradually increase motor duty at segment start
// to synchronize wheel engagement and reduce drift from motor timing mismatch.
// Ramp goes from RAMP_START_FRACTION to 1.0 over RAMP_DURATION_MS.
#define RAMP_DURATION_MS        75      // ms to reach full speed from start
#define RAMP_START_FRACTION     0.15f   // initial duty fraction (just enough to overcome friction)

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
MecanumSpeeds computeMecanumSpeeds(double V, double H, double A, bool lowSpeedMode = false);

#endif // MAIN_H