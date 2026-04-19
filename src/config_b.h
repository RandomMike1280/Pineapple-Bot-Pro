#ifndef CONFIG_B_H
#define CONFIG_B_H

// ============================================================================
// Robot B Configuration
// ============================================================================
// Override only the values that differ from the defaults.
// All other constants come from config_defaults.h automatically.
//
// Values that are NOT overridden here will use the defaults — this means you
// only ever need to tweak the ones specific to this robot.

#define BOT_ID           "B"
#define IS_BOT_B
#define ROBOT_NAME       "Pineapple-Bot-B"

// Speed & distance calibration for Robot B
// TODO: Calibrate these values for Robot B by running the distance test:
//       command a 1000 mm forward move, measure actual distance, then
//       V factor = 1000 / measured_distance
//       Repeat for strafe to calibrate H factor.
#define DISTANCE_FACTOR_V         1.0f
#define DISTANCE_FACTOR_H         1.0f

// Drift correction for Robot B
// TODO: Calibrate by commanding pure forward motion and observing lateral drift.
//       Increase until the robot goes straight.
#define DRIFT_TRIM_SLOW_DEG       0.0f

// WiFi credentials for Robot B
// TODO: Set these to Robot B's hotspot SSID and password.
#ifndef ROBOT_WIFI_SSID
#define ROBOT_WIFI_SSID     "YOUR_HOTSPOT_SSID"
#endif
#ifndef ROBOT_WIFI_PASS
#define ROBOT_WIFI_PASS     "YOUR_HOTSPOT_PASSWORD"
#endif

#endif // CONFIG_B_H
