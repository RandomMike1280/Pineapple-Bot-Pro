#ifndef CONFIG_DEFAULTS_H
#define CONFIG_DEFAULTS_H

// ============================================================================
// Default Configuration — safe baseline for any robot
// Copy this file as config_a.h or config_b.h and tweak values as needed.
// ============================================================================
//
// How the config layering works:
//
//   config_defaults.h   — safe baseline values for every constant
//   config_a.h         — #includes config_defaults.h, then overrides Robot A values
//   config_b.h         — #includes config_defaults.h, then overrides Robot B values
//   main.h             — #includes the selected robot config (e.g. config_a.h)
//                        then declares all the shared API (structs, function
//                        prototypes, etc.)
//   main.cpp           — #includes main.h, sets WiFi via ROBOT_WIFI_* macros
//
// To switch between robots, change which config file main.h includes
// (see the #error directive in main.h).
// WiFi credentials are set in main.cpp via ROBOT_WIFI_SSID / ROBOT_WIFI_PASS.
// ============================================================================

// ---------------------------------------------------------------------------
// Robot Identity
// ---------------------------------------------------------------------------
#ifndef BOT_ID
#define BOT_ID               "UNKNOWN"
#endif

#ifndef ROBOT_NAME
#define ROBOT_NAME            "Pineapple-Bot"
#endif

#ifndef ARUCO_ID
#define ARUCO_ID             0
#endif

#ifndef LED
#define LED                   2
#endif

// ---------------------------------------------------------------------------
// Speed & Distance Scaling  (calibrate per robot)
// ---------------------------------------------------------------------------
// SPEED_SCALING_FACTOR: global multiplier for all movement speeds.
//   < 1.0 makes the robot slower and more controllable.
// ROTATION_SCALING_FACTOR: multiplier for rotational speed independently.
// DISTANCE_FACTOR_*: dead-reckoning calibration — measure actual vs commanded
//   distance and adjust until the robot stops where it should.
//
// Calibration guide:
//   1. Command a 1000 mm forward move.
//   2. Measure actual distance traveled with a ruler.
//   3. V factor = commanded / actual  →  e.g. 1000/920 ≈ 1.09
//   4. Repeat for strafe to calibrate H factor.
// ---------------------------------------------------------------------------
#define DEFAULT_SPEED_SCALING_FACTOR      1.0f
#define DEFAULT_ROTATION_SCALING_FACTOR   1.0f
#define DEFAULT_DISTANCE_FACTOR_V         1.0f
#define DEFAULT_DISTANCE_FACTOR_H         1.0f

#ifndef SPEED_SCALING_FACTOR
#define SPEED_SCALING_FACTOR    DEFAULT_SPEED_SCALING_FACTOR
#endif

#ifndef ROTATION_SCALING_FACTOR
#define ROTATION_SCALING_FACTOR DEFAULT_ROTATION_SCALING_FACTOR
#endif

#ifndef DISTANCE_FACTOR_V
#define DISTANCE_FACTOR_V       DEFAULT_DISTANCE_FACTOR_V
#endif

#ifndef DISTANCE_FACTOR_H
#define DISTANCE_FACTOR_H       DEFAULT_DISTANCE_FACTOR_H
#endif

// ---------------------------------------------------------------------------
// Speed Profiles  (mm/s for translation, deg/s for rotation)
// ---------------------------------------------------------------------------
// These are measured from PS2 controller physics at 100% duty.
// Override SPEED_SCALING_FACTOR / ROTATION_SCALING_FACTOR instead of changing
// these directly, unless the motor characteristics differ significantly.
// ---------------------------------------------------------------------------
#define DEFAULT_SPEED_SLOW_MM_S      (29.0f  * SPEED_SCALING_FACTOR)
#define DEFAULT_SPEED_NORMAL_MM_S    (43.5f  * SPEED_SCALING_FACTOR)
#define DEFAULT_SPEED_FAST_MM_S      (72.5f  * SPEED_SCALING_FACTOR)

#define DEFAULT_SPEED_SLOW_DEG_S    (15.0f  * SPEED_SCALING_FACTOR * ROTATION_SCALING_FACTOR)
#define DEFAULT_SPEED_NORMAL_DEG_S  (51.84f * SPEED_SCALING_FACTOR * ROTATION_SCALING_FACTOR)
#define DEFAULT_SPEED_FAST_DEG_S    (86.4f  * SPEED_SCALING_FACTOR * ROTATION_SCALING_FACTOR)

#ifndef SPEED_SLOW_MM_S
#define SPEED_SLOW_MM_S      DEFAULT_SPEED_SLOW_MM_S
#endif

#ifndef SPEED_NORMAL_MM_S
#define SPEED_NORMAL_MM_S    DEFAULT_SPEED_NORMAL_MM_S
#endif

#ifndef SPEED_FAST_MM_S
#define SPEED_FAST_MM_S      DEFAULT_SPEED_FAST_MM_S
#endif

#ifndef SPEED_SLOW_DEG_S
#define SPEED_SLOW_DEG_S     DEFAULT_SPEED_SLOW_DEG_S
#endif

#ifndef SPEED_NORMAL_DEG_S
#define SPEED_NORMAL_DEG_S   DEFAULT_SPEED_NORMAL_DEG_S
#endif

#ifndef SPEED_FAST_DEG_S
#define SPEED_FAST_DEG_S     DEFAULT_SPEED_FAST_DEG_S
#endif

// ---------------------------------------------------------------------------
// Motor Physics
// ---------------------------------------------------------------------------
// DRIVE_CLAMP_*: safety bounds on motor duty cycle (0-100).
//   LOW    = minimum to overcome static friction (get wheels moving).
//   HIGH   = absolute maximum allowed duty (safety cap).
//   FINE   = minimum for precision mode (lower than LOW for fine movements).
//   MINIMAL= floor for single-motor micro-adjustments.
//
// MOTOR_MAP_*: linear calibration of duty vs normalized speed.
//   duty = MOTOR_MAP_SLOPE * speed + MOTOR_MAP_OFFSET * sign(speed)
//   Calibrate by plotting commanded duty vs measured wheel RPM.
//
// MOTOR_STOP_THRESHOLD: below this duty, motors stall and don't move.
// KICKSTART_*: brief extra power to overcome static friction at startup.
// ---------------------------------------------------------------------------
#define DEFAULT_ENABLE_DEBUG_LOGGING     1
#define DEFAULT_DRIVE_CLAMP_LOW         65
#define DEFAULT_DRIVE_CLAMP_HIGH        100
#define DEFAULT_DRIVE_CLAMP_FINE        65
#define DEFAULT_DRIVE_CLAMP_MINIMAL     25
#define DEFAULT_MOTOR_MAP_SLOPE         43.0174f
#define DEFAULT_MOTOR_MAP_OFFSET        57.1650f
#define DEFAULT_MOTOR_STOP_THRESHOLD    35
#define DEFAULT_KICKSTART_FRAMES         3
#define DEFAULT_KICKSTART_SPEED         70.0f
#define DEFAULT_RAMP_DURATION_MS        75
#define DEFAULT_RAMP_START_FRACTION     0.15f

#ifndef ENABLE_DEBUG_LOGGING
#define ENABLE_DEBUG_LOGGING    DEFAULT_ENABLE_DEBUG_LOGGING
#endif

#ifndef DRIVE_CLAMP_LOW
#define DRIVE_CLAMP_LOW        DEFAULT_DRIVE_CLAMP_LOW
#endif

#ifndef DRIVE_CLAMP_HIGH
#define DRIVE_CLAMP_HIGH       DEFAULT_DRIVE_CLAMP_HIGH
#endif

#ifndef DRIVE_CLAMP_FINE
#define DRIVE_CLAMP_FINE       DEFAULT_DRIVE_CLAMP_FINE
#endif

#ifndef DRIVE_CLAMP_MINIMAL
#define DRIVE_CLAMP_MINIMAL    DEFAULT_DRIVE_CLAMP_MINIMAL
#endif

#ifndef MOTOR_MAP_SLOPE
#define MOTOR_MAP_SLOPE        DEFAULT_MOTOR_MAP_SLOPE
#endif

#ifndef MOTOR_MAP_OFFSET
#define MOTOR_MAP_OFFSET       DEFAULT_MOTOR_MAP_OFFSET
#endif

#ifndef MOTOR_STOP_THRESHOLD
#define MOTOR_STOP_THRESHOLD   DEFAULT_MOTOR_STOP_THRESHOLD
#endif

#ifndef KICKSTART_FRAMES
#define KICKSTART_FRAMES       DEFAULT_KICKSTART_FRAMES
#endif

#ifndef KICKSTART_SPEED
#define KICKSTART_SPEED        DEFAULT_KICKSTART_SPEED
#endif

#ifndef RAMP_DURATION_MS
#define RAMP_DURATION_MS       DEFAULT_RAMP_DURATION_MS
#endif

#ifndef RAMP_START_FRACTION
#define RAMP_START_FRACTION    DEFAULT_RAMP_START_FRACTION
#endif

// ---------------------------------------------------------------------------
// Precision Control
// ---------------------------------------------------------------------------
// PRECISION_MODE_THRESH_MM: enter precision mode when within this distance.
// WAYPOINT_TOLERANCE_MM / ROTATION_TOLERANCE_DEG: consider arrival reached.
// CLOSE_APPROACH_*: trigger final creep speed before full stop.
// ---------------------------------------------------------------------------
#define DEFAULT_PRECISION_MODE_THRESH_MM     50
#define DEFAULT_WAYPOINT_SAME_TARGET_TOL_MM  15.0f
#define DEFAULT_PRECISION_KICKSTART_SUPPRESS true
#define DEFAULT_PRECISION_MIN_SPEED_LIMIT_MM_S 18.0f
#define DEFAULT_PRECISION_MIN_ROT_LIMIT_DEG_S  5.0f
#define DEFAULT_CLOSE_APPROACH_DISTANCE_MM    120.0f
#define DEFAULT_CLOSE_ROT_APPROACH_DEG         8.0f
#define DEFAULT_WAYPOINT_TOLERANCE_MM          5.0f
#define DEFAULT_ROTATION_TOLERANCE_DEG          1.5f

#ifndef PRECISION_MODE_THRESH_MM
#define PRECISION_MODE_THRESH_MM     DEFAULT_PRECISION_MODE_THRESH_MM
#endif

#ifndef WAYPOINT_SAME_TARGET_TOL_MM
#define WAYPOINT_SAME_TARGET_TOL_MM  DEFAULT_WAYPOINT_SAME_TARGET_TOL_MM
#endif

#ifndef PRECISION_KICKSTART_SUPPRESS
#define PRECISION_KICKSTART_SUPPRESS DEFAULT_PRECISION_KICKSTART_SUPPRESS
#endif

#ifndef PRECISION_MIN_SPEED_LIMIT_MM_S
#define PRECISION_MIN_SPEED_LIMIT_MM_S DEFAULT_PRECISION_MIN_SPEED_LIMIT_MM_S
#endif

#ifndef PRECISION_MIN_ROT_LIMIT_DEG_S
#define PRECISION_MIN_ROT_LIMIT_DEG_S  DEFAULT_PRECISION_MIN_ROT_LIMIT_DEG_S
#endif

#ifndef CLOSE_APPROACH_DISTANCE_MM
#define CLOSE_APPROACH_DISTANCE_MM    DEFAULT_CLOSE_APPROACH_DISTANCE_MM
#endif

#ifndef CLOSE_ROT_APPROACH_DEG
#define CLOSE_ROT_APPROACH_DEG         DEFAULT_CLOSE_ROT_APPROACH_DEG
#endif

#ifndef WAYPOINT_TOLERANCE_MM
#define WAYPOINT_TOLERANCE_MM          DEFAULT_WAYPOINT_TOLERANCE_MM
#endif

#ifndef ROTATION_TOLERANCE_DEG
#define ROTATION_TOLERANCE_DEG         DEFAULT_ROTATION_TOLERANCE_DEG
#endif

// ---------------------------------------------------------------------------
// Drift Correction  (counteract systematic mecanum physics bias)
// ---------------------------------------------------------------------------
// Mecanum wheels on flat surfaces tend to pull to one side (often left).
// Calibrate by commanding pure forward motion and observing how far the robot
// drifts laterally. Increase DRIFT_TRIM_* until the robot goes straight.
// ---------------------------------------------------------------------------
#define DEFAULT_DRIFT_TRIM_SLOW_DEG    9.36f
#define DEFAULT_DRIFT_TRIM_NORMAL_DEG  0.00f
#define DEFAULT_DRIFT_TRIM_FAST_DEG    0.00f
#define DEFAULT_CAMERA_LATENCY_MS      150
#define DEFAULT_DRIFT_THRESHOLD_MM     5.0f
#define DEFAULT_EMERGENCY_THRESHOLD_MM 25.0f
#define DEFAULT_CORRECTION_BLEND_MS   200

#ifndef DRIFT_TRIM_SLOW_DEG
#define DRIFT_TRIM_SLOW_DEG    DEFAULT_DRIFT_TRIM_SLOW_DEG
#endif

#ifndef DRIFT_TRIM_NORMAL_DEG
#define DRIFT_TRIM_NORMAL_DEG  DEFAULT_DRIFT_TRIM_NORMAL_DEG
#endif

#ifndef DRIFT_TRIM_FAST_DEG
#define DRIFT_TRIM_FAST_DEG    DEFAULT_DRIFT_TRIM_FAST_DEG
#endif

#ifndef CAMERA_LATENCY_MS
#define CAMERA_LATENCY_MS      DEFAULT_CAMERA_LATENCY_MS
#endif

#ifndef DRIFT_THRESHOLD_MM
#define DRIFT_THRESHOLD_MM     DEFAULT_DRIFT_THRESHOLD_MM
#endif

#ifndef EMERGENCY_THRESHOLD_MM
#define EMERGENCY_THRESHOLD_MM DEFAULT_EMERGENCY_THRESHOLD_MM
#endif

#ifndef CORRECTION_BLEND_MS
#define CORRECTION_BLEND_MS    DEFAULT_CORRECTION_BLEND_MS
#endif

// ---------------------------------------------------------------------------
// Deceleration & Stabilization
// ---------------------------------------------------------------------------
#define DEFAULT_DECCEL_DISTANCE_MM    500.0f
#define DEFAULT_ROT_DECCEL_DEG         15.0f
#define DEFAULT_MIN_SPEED_LIMIT_MM_S   10.0f
#define DEFAULT_MIN_ROT_LIMIT_DEG_S     5.0f
#define DEFAULT_STABILIZATION_GAIN      2.5f
#define DEFAULT_MAX_STABILIZATION_OMEGA 35.0f

#ifndef DECCEL_DISTANCE_MM
#define DECCEL_DISTANCE_MM    DEFAULT_DECCEL_DISTANCE_MM
#endif

#ifndef ROT_DECCEL_DEG
#define ROT_DECCEL_DEG         DEFAULT_ROT_DECCEL_DEG
#endif

#ifndef MIN_SPEED_LIMIT_MM_S
#define MIN_SPEED_LIMIT_MM_S   DEFAULT_MIN_SPEED_LIMIT_MM_S
#endif

#ifndef MIN_ROT_LIMIT_DEG_S
#define MIN_ROT_LIMIT_DEG_S    DEFAULT_MIN_ROT_LIMIT_DEG_S
#endif

#ifndef STABILIZATION_GAIN
#define STABILIZATION_GAIN     DEFAULT_STABILIZATION_GAIN
#endif

#ifndef MAX_STABILIZATION_OMEGA
#define MAX_STABILIZATION_OMEGA DEFAULT_MAX_STABILIZATION_OMEGA
#endif

// ---------------------------------------------------------------------------
// Trajectory Planning
// ---------------------------------------------------------------------------
// Predictive Steering — look ahead in time to compensate for momentum/drift.
// Predictive Braking — use kinematics to prevent overshoot.
// S-Curve Profile — jerk-limited acceleration for smooth starts/stops.
// Feedforward — pre-compensate inertia and friction.
// ---------------------------------------------------------------------------
#define DEFAULT_PREDICTIVE_LOOKAHEAD_S      0.15f
#define DEFAULT_PREDICTIVE_BRAKE_DECEL_MM_S2  350.0f
#define DEFAULT_PREDICTIVE_BRAKE_SAFETY        1.2f
#define DEFAULT_LATENCY_AWARE_DECEL_MM_S2     300.0f
#define DEFAULT_SCURVE_MAX_ACCEL_MM_S2        250.0f
#define DEFAULT_SCURVE_MAX_JERK_MM_S3        1200.0f
#define DEFAULT_SCURVE_MAX_ROT_ACCEL_DEG_S2   180.0f
#define DEFAULT_SCURVE_MAX_ROT_JERK_DEG_S3    900.0f
#define DEFAULT_FEEDFORWARD_KV               0.02f
#define DEFAULT_FEEDFORWARD_KA               0.04f
#define DEFAULT_FEEDFORWARD_KV_ROT           0.01f
#define DEFAULT_FEEDFORWARD_KA_ROT           0.02f
#define DEFAULT_ADAPTIVE_LOOKAHEAD_BASE_S     0.06f
#define DEFAULT_ADAPTIVE_LOOKAHEAD_GAIN       0.0022f
#define DEFAULT_ADAPTIVE_LOOKAHEAD_MIN_S       0.04f
#define DEFAULT_ADAPTIVE_LOOKAHEAD_MAX_S       0.18f

#ifndef PREDICTIVE_LOOKAHEAD_S
#define PREDICTIVE_LOOKAHEAD_S    DEFAULT_PREDICTIVE_LOOKAHEAD_S
#endif

#ifndef PREDICTIVE_BRAKE_DECEL_MM_S2
#define PREDICTIVE_BRAKE_DECEL_MM_S2 DEFAULT_PREDICTIVE_BRAKE_DECEL_MM_S2
#endif

#ifndef PREDICTIVE_BRAKE_SAFETY
#define PREDICTIVE_BRAKE_SAFETY   DEFAULT_PREDICTIVE_BRAKE_SAFETY
#endif

#ifndef LATENCY_AWARE_DECEL_MM_S2
#define LATENCY_AWARE_DECEL_MM_S2 DEFAULT_LATENCY_AWARE_DECEL_MM_S2
#endif

#ifndef SCURVE_MAX_ACCEL_MM_S2
#define SCURVE_MAX_ACCEL_MM_S2    DEFAULT_SCURVE_MAX_ACCEL_MM_S2
#endif

#ifndef SCURVE_MAX_JERK_MM_S3
#define SCURVE_MAX_JERK_MM_S3    DEFAULT_SCURVE_MAX_JERK_MM_S3
#endif

#ifndef SCURVE_MAX_ROT_ACCEL_DEG_S2
#define SCURVE_MAX_ROT_ACCEL_DEG_S2 DEFAULT_SCURVE_MAX_ROT_ACCEL_DEG_S2
#endif

#ifndef SCURVE_MAX_ROT_JERK_DEG_S3
#define SCURVE_MAX_ROT_JERK_DEG_S3 DEFAULT_SCURVE_MAX_ROT_JERK_DEG_S3
#endif

#ifndef FEEDFORWARD_KV
#define FEEDFORWARD_KV           DEFAULT_FEEDFORWARD_KV
#endif

#ifndef FEEDFORWARD_KA
#define FEEDFORWARD_KA           DEFAULT_FEEDFORWARD_KA
#endif

#ifndef FEEDFORWARD_KV_ROT
#define FEEDFORWARD_KV_ROT       DEFAULT_FEEDFORWARD_KV_ROT
#endif

#ifndef FEEDFORWARD_KA_ROT
#define FEEDFORWARD_KA_ROT       DEFAULT_FEEDFORWARD_KA_ROT
#endif

#ifndef ADAPTIVE_LOOKAHEAD_BASE_S
#define ADAPTIVE_LOOKAHEAD_BASE_S  DEFAULT_ADAPTIVE_LOOKAHEAD_BASE_S
#endif

#ifndef ADAPTIVE_LOOKAHEAD_GAIN
#define ADAPTIVE_LOOKAHEAD_GAIN    DEFAULT_ADAPTIVE_LOOKAHEAD_GAIN
#endif

#ifndef ADAPTIVE_LOOKAHEAD_MIN_S
#define ADAPTIVE_LOOKAHEAD_MIN_S   DEFAULT_ADAPTIVE_LOOKAHEAD_MIN_S
#endif

#ifndef ADAPTIVE_LOOKAHEAD_MAX_S
#define ADAPTIVE_LOOKAHEAD_MAX_S   DEFAULT_ADAPTIVE_LOOKAHEAD_MAX_S
#endif

// ---------------------------------------------------------------------------
// Kalman Filter — velocity estimation
// ---------------------------------------------------------------------------
#define DEFAULT_KALMAN_PROCESS_NOISE_POS     0.5f
#define DEFAULT_KALMAN_PROCESS_NOISE_VEL     50.0f
#define DEFAULT_KALMAN_MEASUREMENT_NOISE      2.0f

#ifndef KALMAN_PROCESS_NOISE_POS
#define KALMAN_PROCESS_NOISE_POS   DEFAULT_KALMAN_PROCESS_NOISE_POS
#endif

#ifndef KALMAN_PROCESS_NOISE_VEL
#define KALMAN_PROCESS_NOISE_VEL   DEFAULT_KALMAN_PROCESS_NOISE_VEL
#endif

#ifndef KALMAN_MEASUREMENT_NOISE
#define KALMAN_MEASUREMENT_NOISE  DEFAULT_KALMAN_MEASUREMENT_NOISE
#endif

// ---------------------------------------------------------------------------
// Anti-Slip / Stall Detection
// ---------------------------------------------------------------------------
#define DEFAULT_SLIP_CMD_SPEED_THRESH_MM_S  20.0f
#define DEFAULT_SLIP_OBS_SPEED_THRESH_MM_S  5.0f
#define DEFAULT_SLIP_DETECT_TICKS            75
#define DEFAULT_SLIP_BOOST_FACTOR            1.35f
#define DEFAULT_SLIP_BOOST_MAX_TICKS         100

#ifndef SLIP_CMD_SPEED_THRESH_MM_S
#define SLIP_CMD_SPEED_THRESH_MM_S  DEFAULT_SLIP_CMD_SPEED_THRESH_MM_S
#endif

#ifndef SLIP_OBS_SPEED_THRESH_MM_S
#define SLIP_OBS_SPEED_THRESH_MM_S  DEFAULT_SLIP_OBS_SPEED_THRESH_MM_S
#endif

#ifndef SLIP_DETECT_TICKS
#define SLIP_DETECT_TICKS           DEFAULT_SLIP_DETECT_TICKS
#endif

#ifndef SLIP_BOOST_FACTOR
#define SLIP_BOOST_FACTOR           DEFAULT_SLIP_BOOST_FACTOR
#endif

#ifndef SLIP_BOOST_MAX_TICKS
#define SLIP_BOOST_MAX_TICKS        DEFAULT_SLIP_BOOST_MAX_TICKS
#endif

// ---------------------------------------------------------------------------
// Timing
// ---------------------------------------------------------------------------
#define DEFAULT_STATUS_INTERVAL_MS    100
#define DEFAULT_HELLO_INTERVAL_MS   3000
#define DEFAULT_PING_INTERVAL_MS      500
#define DEFAULT_CONTROL_LOOP_INTERVAL_MS 1
#define DEFAULT_DONE_FEEDBACK_BLINKS   4

#ifndef STATUS_INTERVAL_MS
#define STATUS_INTERVAL_MS    DEFAULT_STATUS_INTERVAL_MS
#endif

#ifndef HELLO_INTERVAL_MS
#define HELLO_INTERVAL_MS     DEFAULT_HELLO_INTERVAL_MS
#endif

#ifndef PING_INTERVAL_MS
#define PING_INTERVAL_MS      DEFAULT_PING_INTERVAL_MS
#endif

#ifndef CONTROL_LOOP_INTERVAL_MS
#define CONTROL_LOOP_INTERVAL_MS DEFAULT_CONTROL_LOOP_INTERVAL_MS
#endif

#ifndef DONE_FEEDBACK_BLINKS
#define DONE_FEEDBACK_BLINKS  DEFAULT_DONE_FEEDBACK_BLINKS
#endif

// ---------------------------------------------------------------------------
// Mecanum Kinematics — Brake Parameters
// ---------------------------------------------------------------------------
// ROTATION_BRAKE_*: kick brief reverse thrust to kill angular momentum.
// TRANSLATION_BRAKE_*: reverse thrust to kill physical coasting momentum.
// ---------------------------------------------------------------------------
#define DEFAULT_ROTATION_BRAKE_FRAMES            15
#define DEFAULT_ROTATION_BRAKE_DUTY              75
#define DEFAULT_ROTATION_BRAKE_MIN_OMEGA_DEG_S    5.0f
#define DEFAULT_TRANSLATION_BRAKE_FRAMES           18
#define DEFAULT_TRANSLATION_BRAKE_DUTY            80
#define DEFAULT_TRANSLATION_BRAKE_MIN_SPEED_MM_S   5.0f

#ifndef ROTATION_BRAKE_FRAMES
#define ROTATION_BRAKE_FRAMES            DEFAULT_ROTATION_BRAKE_FRAMES
#endif

#ifndef ROTATION_BRAKE_DUTY
#define ROTATION_BRAKE_DUTY              DEFAULT_ROTATION_BRAKE_DUTY
#endif

#ifndef ROTATION_BRAKE_MIN_OMEGA_DEG_S
#define ROTATION_BRAKE_MIN_OMEGA_DEG_S   DEFAULT_ROTATION_BRAKE_MIN_OMEGA_DEG_S
#endif

#ifndef TRANSLATION_BRAKE_FRAMES
#define TRANSLATION_BRAKE_FRAMES          DEFAULT_TRANSLATION_BRAKE_FRAMES
#endif

#ifndef TRANSLATION_BRAKE_DUTY
#define TRANSLATION_BRAKE_DUTY           DEFAULT_TRANSLATION_BRAKE_DUTY
#endif

#ifndef TRANSLATION_BRAKE_MIN_SPEED_MM_S
#define TRANSLATION_BRAKE_MIN_SPEED_MM_S  DEFAULT_TRANSLATION_BRAKE_MIN_SPEED_MM_S
#endif

#endif // CONFIG_DEFAULTS_H
