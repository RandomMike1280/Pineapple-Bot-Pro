#ifndef MAIN_H
#define MAIN_H

#include <Arduino.h>
#include <esp32_motor.hpp>
#include <esp32_servo.hpp>
#include <ps2x.hpp>

// ============================================================================
// PS2 Controller Setup
// ============================================================================
#define PS2_DAT  1
#define PS2_CMD  2
#define PS2_CS   4
#define PS2_CLK  5

typedef struct {
    uint8_t pressed;
    uint8_t lastPressed;
    int8_t X;
    int8_t Y;
} JoyStickButton;
extern JoyStickButton leftJoy;
extern JoyStickButton rightJoy;

typedef struct {
    uint8_t up;
    uint8_t down;
    uint8_t left;
    uint8_t right;
} DPadButton;
extern DPadButton DPad;
extern DPadButton lastDPad;

typedef struct {
    uint8_t triangle;
    uint8_t circle;
    uint8_t cross;
    uint8_t square;
} geoPadButton;
extern geoPadButton GeoPad;
extern geoPadButton lastGeoPad;

typedef struct {
    uint8_t L1;
    uint8_t L2;
    uint8_t R1;
    uint8_t R2;
} shoulderButton;
extern shoulderButton Shoulder;
extern shoulderButton lastShoulder;

#define joyThres 15
void getRemoteState(PS2X &remote);

#define pressedButton(current, last) ((current == 1 && last == 0) ? 1 : 0)
#define releasedButton(current, last) ((current == 0 && last == 1) ? 1 : 0)

#define verticalVelocity leftJoy.Y
#define horizontalVelocity leftJoy.X
#define angularVelocity rightJoy.X

// ============================================================================
// Physics & Motor Modeling
// ============================================================================
#define SLOW_SPEED

#ifdef FAST_SPEED
#define MAX_SPEED 100
#define vRate 0.01
#define hRate 0.01
#define angularRate 0.01
#endif

#ifdef SLOW_SPEED
#define MAX_SPEED 100
#define vRate (0.005 * 0.8888888889)
#define hRate (0.00625 * 0.8888888889)
#define angularRate 0.005
#endif

#define MOTOR_STOP_THRESHOLD    20
#define KICKSTART_FRAMES        3
#define KICKSTART_SPEED         70.0

extern int mspeed[4];
extern double mspeedf[4];
extern int mkickstart[4];

int sign(double x);
int speed_to_motor_duty(double speed);
void calculateMotorSpeeds();

// ============================================================================
// Robot Identity — change to "B" when flashing the second robot
// ============================================================================
#define BOT_ID "0"

// ============================================================================
// Operating Mode
// ============================================================================
// #define TEST_MODE // Uncomment to enable test mode (no motors, print commands, blinking LED)

#ifndef LED
#define LED 47 // Default LED pin updated per manual controls
#endif

// ============================================================================
// Speed Calibration (mm/s at each speed level)
// ============================================================================
// Based on 5cm wheels, max speed is ~72.5 mm/s.
#define SPEED_SLOW_MM_S    36.0f
#define SPEED_NORMAL_MM_S  55.0f
#define SPEED_FAST_MM_S    72.5f

// ============================================================================
// Motor Duty Cycle Mapping (speed level → motor duty %)
// ============================================================================
// The mecanum inverse kinematics expects a "base duty" for each speed level.
// (This is transitioning to the new physics model, but retained for legacy bridging)
#define DUTY_SLOW     40
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