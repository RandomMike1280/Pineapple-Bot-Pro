#ifndef UDP_PROTOCOL_HPP
#define UDP_PROTOCOL_HPP

#include <Arduino.h>

// ============================================================================
// UDP Protocol — Structured message format for Robot Phone communication
// ============================================================================
// All messages are short ASCII strings for easy debugging on serial monitor.
// Format:  TYPE_CHAR:field1:field2:...
//
//   HELLO     H:<robot_id>
//   MOVE      M:<dir>:<dist_mm>:<speed>:<policy>
//   MOVE_DUR  D:<dir>:<duration_ms>:<speed>:<policy>
//   WAYPOINT  W:<target_x>:<target_y>[:<angle>]:<speed>:<policy>[:<servo_action>]
//   ROTATE    T:<target_angle_deg>:<speed>:<policy>
//   ROT_DUR   O:<cw|ccw>:<duration_ms>:<speed>:<policy>
//   CAM       C:<timestamp_ms>:<x_mm>:<y_mm>[:<angle_deg>]
//   PING      P:<timestamp_ms>
//   PONG      Q:<orig_timestamp_ms>
//   STATUS    S:<x>:<y>:<queue_len>:<drift_mm>
//   VELOCITY  V:<vx_mm_s>:<vy_mm_s>:<omega_deg_s>:<timeout_ms>
//   ABORT     A
//   REGISTER  R:<robot_id>:<caps>
// ============================================================================

enum class MsgType : uint8_t {
    HELLO,
    MOVE,
    CAM,
    PING,
    PONG,
    STATUS,
    ABORT,
    REGISTER,
    WAYPOINT,
    ROTATE,
    MOVE_DURATION,
    ROTATE_DURATION,
    VELOCITY,
    SET_SERIAL_MONITOR,
    SERIAL_LOG,
    DONE,
    SERVO_EXEC,
    UNKNOWN
};

enum class MoveDirection : uint8_t {
    UP = 0,
    DOWN,
    LEFT,
    RIGHT,
    INVALID
};

enum class RotationDirection : uint8_t {
    CW = 0,
    CCW,
    INVALID
};

enum class SpeedLevel : uint8_t {
    SLOW = 0,
    NORMAL,
    FAST,
    INVALID
};

enum class CorrectionPolicy : uint8_t {
    LIVE = 0,
    DEFERRED,
    NONE,
    INVALID
};

enum class ServoAction : uint8_t {
    NONE = 0,
    LOWER_LEFT,
    LOWER_RIGHT,
    UPPER_LEFT,
    UPPER_RIGHT,
    GRABBER_LEFT,
    GRABBER_RIGHT,
    GRABBER_CENTER,
    SLIDER_UP,
    SLIDER_DOWN,
    ARM_DOWN,
    ARM_UP,
    INVALID
};

// --- Parsed message container ---
struct UdpMessage {
    MsgType type;

    // MOVE fields
    MoveDirection direction;
    RotationDirection rotationDirection;
    uint16_t      distance_mm;
    SpeedLevel    speed;
    CorrectionPolicy correctionPolicy;

    // MOVE_DURATION fields
    uint32_t      duration_ms;

    // WAYPOINT fields
    float         target_x;
    float         target_y;

    // ROTATE fields
    float         target_angle;

    // CAM fields
    uint32_t cam_timestamp;
    float    cam_x;
    float    cam_y;
    float    cam_angle;

    // PING/PONG fields
    uint32_t ping_timestamp;
    uint32_t remote_timestamp;  // used for clock sync during pong

    // VELOCITY fields
    float vel_vx;          // mm/s world-frame
    float vel_vy;          // mm/s world-frame
    float vel_omega;       // deg/s
    // duration_ms is reused for velocity timeout

    // REGISTER fields
    char robot_id[8];

    // SERVO fields
    ServoAction   servoAction;
};

// --- Parsing ---

/// Parse a raw UDP buffer into a structured UdpMessage.
/// Returns true if parsing succeeded.
bool parseUdpMessage(const char* buffer, int len, UdpMessage &out);

// --- Building outgoing messages ---

/// Build a HELLO message: "H:<robot_id>"
int buildHelloMessage(char* buf, int maxLen, const char* robotId);

/// Build a PONG message: "Q:<orig_timestamp>"
int buildPongMessage(char* buf, int maxLen, uint32_t origTimestamp);

/// Build a STATUS message: "S:<y>:<x>:<angle>:<queue_len>:<drift>:<vy>:<vx>"
/// Note: x/y and vx/vy are swapped to match phone's rotated coordinate system
int buildStatusMessage(char* buf, int maxLen,
                       float x, float y, float angle, int queueLen, float driftMm,
                       float vx, float vy);

/// Build a REGISTER message: "R:<robot_id>:<caps>"
int buildRegisterMessage(char* buf, int maxLen,
                         const char* robotId, const char* capabilities);

/// Send a log message: "L:<S|U|I>[:<id>]:<msg>"
/// Type: S=Static, U=Update-in-place, I=Important (Red)
int buildLogMessage(char* buf, int maxLen,
                    char type, const char* logId, const char* msg);

// --- Helpers ---
MoveDirection   parseDirection(const char* str);
RotationDirection parseRotationDirection(const char* str);
SpeedLevel      parseSpeed(const char* str);
CorrectionPolicy parsePolicy(const char* str);
ServoAction      parseServoAction(const char* str);

/// Convert direction enum to velocity unit vector (vx, vy)
void directionToVector(MoveDirection dir, float &vx, float &vy);

#endif // UDP_PROTOCOL_HPP
