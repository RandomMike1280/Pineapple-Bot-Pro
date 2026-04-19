#ifndef CONFIG_A_H
#define CONFIG_A_H

// ============================================================================
// Robot A Configuration
// ============================================================================
// Override only the values that differ from the defaults.
// All other constants come from config_defaults.h automatically.
//
// Values that are NOT overridden here will use the defaults — this means you
// only ever need to tweak the ones specific to this robot.

#define BOT_ID           "A"
#define IS_BOT_A
#define ROBOT_NAME       "Pineapple-Bot-A"

// Speed & distance calibration for Robot A
#define DISTANCE_FACTOR_V         1.0f
#define DISTANCE_FACTOR_H         1.25f

// Drift correction for Robot A
#define DRIFT_TRIM_SLOW_DEG       9.36f

// WiFi credentials for Robot A
// Set these to your hotspot's SSID and password for Robot A.
#ifndef ROBOT_WIFI_SSID
#define ROBOT_WIFI_SSID     "IPhone 19 Professional"
#endif
#ifndef ROBOT_WIFI_PASS
#define ROBOT_WIFI_PASS     "sixseven"
#endif

#endif // CONFIG_A_H
