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
#define DISTANCE_FACTOR_H        1.0f     // Calibrates dead-reckoning for Horizontal (Strafe)

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
#define DRIVE_CLAMP_LOW          20       // Minimum duty to consider a motor "moving"
#define DRIVE_CLAMP_HIGH         100      // Maximum allowable duty (safety cap)

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
#define DRIFT_THRESHOLD_MM       20.0f    // apply full correction above this
#define EMERGENCY_THRESHOLD_MM   50.0f    // decelerate + correct above this
#define CORRECTION_BLEND_MS      200      // ms to blend corrections smoothly

// ============================================================================
// Precision & Deceleration
// ============================================================================
#define DECCEL_DISTANCE_MM       150.0f   // Start slowing down at this distance
#define ROT_DECCEL_DEG           25.0f    // Start slowing down rotation at this angle
#define MIN_SPEED_LIMIT_MM_S     25.0f    // Floor speed during deccel (prevent stall)
#define MIN_ROT_LIMIT_DEG_S      25.0f    // Floor rotation during deccel
#define WAYPOINT_TOLERANCE_MM    12.0f    // Tighten tolerance for arrival
#define ROTATION_TOLERANCE_DEG   2.0f

// ============================================================================
// Active Heading Stabilization (Holonomic Control)
// ============================================================================
// These constants define how the robot corrects its heading while moving.
// Increase the gain if the robot is "lazy" about correcting drift.
// Decrease the gain if the robot "oscillates" or shakes while strafing.
#define STABILIZATION_GAIN       1.5f     // Corrective OMEGA per degree of error
#define MAX_STABILIZATION_OMEGA  35.0f    // Max deg/s allowed for in-move correction

// ============================================================================
// Timing Constants
// ============================================================================
#define STATUS_INTERVAL_MS       100      // telemetry broadcast interval
#define HELLO_INTERVAL_MS        3000     // auto-discovery interval
#define PING_INTERVAL_MS         500      // RTT measurement interval
#define CONTROL_LOOP_INTERVAL_MS 10       // main loop target period

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
#define MOTOR_STOP_THRESHOLD    20

// Kickstart: briefly boost duty to overcome static friction
#define KICKSTART_FRAMES        3
#define KICKSTART_SPEED         70.0
#define ROTATION_BRAKE_FRAMES   2
#define ROTATION_BRAKE_DUTY     35
#define ROTATION_BRAKE_MIN_OMEGA_DEG_S 70.0f

// Acceleration ramp: gradually increase motor duty at segment start
// to synchronize wheel engagement and reduce drift from motor timing mismatch.
// Ramp goes from RAMP_START_FRACTION to 1.0 over RAMP_DURATION_MS.
#define RAMP_DURATION_MS        150     // ms to reach full speed from start
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
MecanumSpeeds computeMecanumSpeeds(double V, double H, double A);

// ============================================================================
// Per-Bot Servo Configuration (from legacy main_bk.cpp)
// ============================================================================
// Uncomment the appropriate block for your bot's gripper/arm calibration.

#ifdef BOT_A_SERVOS
    #define HOLD_THRESHOLD    130
    #define RELEASE_THRESHOLD  55
    #define PICKUP_THRESHOLD   55
    #define DROP_THRESHOLD     30
    #define SERVO3_ANG0        10
    #define SERVO3_ANG1       153
    #define SERVO3_ANG2       162
#endif

#ifdef BOT_B_SERVOS
    #define HOLD_THRESHOLD    147
    #define RELEASE_THRESHOLD  55
    #define PICKUP_THRESHOLD   55
    #define DROP_THRESHOLD     30
    #define SERVO3_ANG0        10
    #define SERVO3_ANG1       145
    #define SERVO3_ANG2       156
#endif

#endif // MAIN_H