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
// Speed Calibration (mm/s at each speed level)
// ============================================================================
// IMPORTANT: Measure these on your actual surface!
// Procedure:  send a "fast" command for 2 seconds, measure how far the robot
// traveled (in mm), divide by 2.  That's your SPEED_FAST_MM_S.
// Repeat for slow and normal duty cycles.
//
// These are conservative starting defaults for a mecanum bot.
#define SPEED_SLOW_MM_S    100.0f
#define SPEED_NORMAL_MM_S  250.0f
#define SPEED_FAST_MM_S    450.0f

// ============================================================================
// Motor Duty Cycle Mapping (speed level → motor duty %)
// ============================================================================
// The mecanum inverse kinematics expects a "base duty" for each speed level.
// This is the duty cycle (0-100) sent to motors for straight-line motion.
#define DUTY_SLOW     35
#define DUTY_NORMAL   60
#define DUTY_FAST     85

// ============================================================================
// Correction Thresholds
// ============================================================================
#define DRIFT_THRESHOLD_MM       20.0f    // apply full correction above this
#define EMERGENCY_THRESHOLD_MM   50.0f    // decelerate + correct above this
#define CORRECTION_BLEND_MS      200      // ms to blend corrections smoothly

// ============================================================================
// Timing Constants
// ============================================================================
#define STATUS_INTERVAL_MS       100      // telemetry broadcast interval
#define HELLO_INTERVAL_MS        3000     // auto-discovery interval
#define PING_INTERVAL_MS         500      // RTT measurement interval
#define CONTROL_LOOP_INTERVAL_MS 10       // main loop target period

// ============================================================================
// Mecanum Kinematics — same math from the original main.h
// ============================================================================
// Motor layout (top view):
//   M4(FL) ---- M1(FR)
//     |            |
//   M3(BL) ---- M2(BR)
//
// For pure translations:
//   Forward:  all motors forward
//   Right:    FL+BR forward, FR+BL backward (strafe)
//   Rotation: left side forward, right side backward

struct MecanumSpeeds {
    int m1;  // Front Right
    int m2;  // Back Right
    int m3;  // Back Left
    int m4;  // Front Left
};

/// Compute individual motor speeds from velocity components.
/// V = forward/backward component (-100..100)
/// H = left/right strafe component (-100..100)
/// A = angular/rotation component (-100..100, optional, default 0)
MecanumSpeeds computeMecanumSpeeds(int V, int H, int A = 0);

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