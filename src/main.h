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
